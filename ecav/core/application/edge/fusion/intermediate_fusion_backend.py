"""Intermediate fusion backend: WorldFusion (Where2comm) feature fusion.

Vehicles/RSUs extract spatial features locally and transmit to edge.
Edge runs backbone + Where2comm attention fusion + detection heads.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import os
from typing import Any, Dict, List

import numpy as np
import torch

from ecav.core.application.edge.fusion.base_fusion import BaseFusionBackend

logger = logging.getLogger(__name__)


class IntermediateFusionBackend(BaseFusionBackend):
    """Intermediate fusion using PointPillarWorldFusion (Where2comm).

    Loads the WorldFusion model at init, collects spatial features
    from vehicle/RSU perception managers, runs fusion and detection
    heads, and outputs AB3DMOT-format detections.

    Config keys:
        worldfusion_model:
            hypes_yaml: path to model config YAML
            checkpoint: path to model weights
        world_anchor: [x, y, z, roll, yaw, pitch]
        anchoring: bool (default True)
    """

    def __init__(self, cfg: dict):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.data_utils.post_processor.world_voxel_postprocessor import (
            WorldVoxelPostprocessor)
        from opencood.utils import box_utils, transformation_utils
        from opencood.tools import train_utils

        wf_cfg = cfg['worldfusion_model']
        self.hypes = load_yaml(wf_cfg['hypes_yaml'])
        self.world_anchor = cfg.get(
            'world_anchor', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.anchoring = cfg.get('anchoring', True)

        # Load model
        from opencood.models.point_pillar_worldfusion import (
            PointPillarWorldFusion)
        self.model = PointPillarWorldFusion(
            self.hypes['model']['args']).cuda().eval()
        ckpt_dir = os.path.dirname(wf_cfg['checkpoint'])
        epoch_id = int(
            wf_cfg['checkpoint'].split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(
            ckpt_dir, self.model, epoch_id)
        logger.info("WorldFusion model loaded from epoch %d", epoch_id)

        # Post-processor
        self.post_processor = WorldVoxelPostprocessor(
            self.hypes['postprocess'], dataset=None, train=False)

        # Keep references for transform computation
        self._box_utils = box_utils
        self._transformation_utils = transformation_utils

    def collect_and_push(self, frame_idx, vehicle_managers, rsu_managers,
                         jitter_buffer, latency_model, mac_model=None,
                         beacon_id_mgr=None, world=None, **kwargs):
        """Collect spatial features and poses from all agents."""
        feature_dicts = []
        poses = []

        vehicle_ids = [vm.vehicle.id for vm in vehicle_managers]
        mac_delivery = (mac_model.attempt_tick(frame_idx, vehicle_ids)
                        if mac_model else {v: True for v in vehicle_ids})

        for vm in vehicle_managers:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue
            pm = vm.perception_manager
            if hasattr(pm, 'feature_dict') and pm.feature_dict:
                feature_dicts.append(pm.feature_dict)
                localizer = vm.localizer
                poses.append(localizer.get_ego_pos())

        for rsu in rsu_managers:
            if hasattr(rsu, 'perception_manager'):
                pm = rsu.perception_manager
                if hasattr(pm, 'feature_dict') and pm.feature_dict:
                    feature_dicts.append(pm.feature_dict)
                    if hasattr(rsu, 'localizer'):
                        poses.append(rsu.localizer.get_ego_pos())

        # Tick beacon IDs for managed vehicles
        if beacon_id_mgr is not None:
            for vm in vehicle_managers:
                beacon_id_mgr.get_temp_id(
                    vm.vehicle.id,
                    vm.vehicle.get_location(),
                    frame_idx)

        arrival = latency_model.stamp(frame_idx)
        jitter_buffer.push(frame_idx, arrival,
                           (feature_dicts, poses))

    def detect(self, payload, frame_idx, beacon_id_mgr=None,
               vehicle_managers=None):
        """Run WorldFusion fusion + detection on collected features."""
        feature_dicts, poses = payload

        if not feature_dicts:
            return {'dets': np.empty((0, 8), np.float32),
                    'info': np.empty((0, 3), np.int64)}

        # Stack spatial features
        spatial_features = torch.stack(
            [fd['spatial_features'] for fd in feature_dicts])

        # Compute pairwise transforms
        pairwise_t = self._compute_pairwise_transforms(poses)
        record_len = torch.tensor([len(feature_dicts)])

        # Run fusion + detection
        with torch.no_grad():
            fused, pred_dict = self._run_fusion(
                spatial_features, pairwise_t, record_len)

        # Post-process to AB3DMOT format
        det_results = self._to_ab3dmot_format(pred_dict, frame_idx)

        # Filter self-detections near managed vehicle positions
        if vehicle_managers:
            det_results = self._filter_self_detections(
                det_results, poses[:len(vehicle_managers)])

        return det_results

    def _run_fusion(self, spatial_features, pairwise_t, record_len):
        """Run backbone + Where2comm fusion + detection heads."""
        batch = {
            'spatial_features': spatial_features.cuda(),
            'record_len': record_len.cuda(),
            'pairwise_t_matrix': pairwise_t.cuda(),
        }
        output = self.model(batch)
        return output.get('fused_feature'), output

    def _compute_pairwise_transforms(self, poses):
        """Compute pairwise transform matrices to world anchor."""
        import carla
        max_cav = max(len(poses), 2)
        pairwise = torch.zeros(1, max_cav, max_cav, 4, 4)

        anchor_tf = carla.Transform(
            carla.Location(
                x=self.world_anchor[0],
                y=self.world_anchor[1],
                z=self.world_anchor[2]),
            carla.Rotation(
                roll=self.world_anchor[3],
                yaw=self.world_anchor[4],
                pitch=self.world_anchor[5]))

        for i, pose in enumerate(poses):
            t = self._transformation_utils.x1_to_x2(pose, anchor_tf)
            t_inv = np.linalg.inv(t)
            pairwise[0, 0, i] = torch.from_numpy(t_inv).float()
            pairwise[0, i, 0] = torch.from_numpy(t).float()

        return pairwise

    def _to_ab3dmot_format(self, pred_dict, frame_idx):
        """Convert detection head output to AB3DMOT dets dict."""
        anchor_box = self.post_processor.generate_anchor_box()

        data_dict = {
            'ego': {
                'anchor_box': anchor_box,
                'lidar_pose': np.array([self.world_anchor]),
            }
        }
        output_dict = {'ego': pred_dict}

        pred_boxes, scores = self.post_processor.post_process(
            data_dict, output_dict)

        if pred_boxes is None or len(pred_boxes) == 0:
            return {'dets': np.empty((0, 8), np.float32),
                    'info': np.empty((0, 3), np.int64)}

        # Convert corners to center format
        if pred_boxes.ndim == 3:  # (N, 8, 3) corners
            boxes_center = self._box_utils.corner_to_center(
                pred_boxes)  # (N, 7)
        else:
            boxes_center = pred_boxes

        # Add world anchor offset
        boxes_center[:, 0] += self.world_anchor[0]
        boxes_center[:, 1] += self.world_anchor[1]

        # Reorder to AB3DMOT: [h,w,l,x,y,z,yaw]
        # boxes_center is [x,y,z,l,w,h,yaw] -> [h,w,l,x,y,z,yaw]
        n = len(boxes_center)
        dets = np.zeros((n, 8), dtype=np.float32)
        dets[:, 0] = boxes_center[:, 5]  # h
        dets[:, 1] = boxes_center[:, 4]  # w
        dets[:, 2] = boxes_center[:, 3]  # l
        dets[:, 3] = boxes_center[:, 0]  # x
        dets[:, 4] = boxes_center[:, 2]  # z (KITTI y)
        dets[:, 5] = boxes_center[:, 1]  # y (KITTI z)
        dets[:, 6] = boxes_center[:, 6]  # yaw
        dets[:, 7] = scores if scores is not None else np.ones(n)

        info = np.zeros((n, 3), dtype=np.int64)
        info[:, 0] = frame_idx

        return {'dets': dets, 'info': info}

    def _filter_self_detections(self, det_results, poses, thresh=2.0):
        """Remove detections matching managed vehicle positions."""
        dets = det_results['dets']
        info = det_results['info']
        if len(dets) == 0:
            return det_results

        keep = np.ones(len(dets), dtype=bool)
        for pose in poses:
            loc = pose.location
            # CARLA x = dets col 3, CARLA y = dets col 5
            dx = dets[:, 3] - loc.x
            dy = dets[:, 5] - loc.y
            dists = np.sqrt(dx**2 + dy**2)
            keep &= dists > thresh

        return {'dets': dets[keep], 'info': info[keep]}
