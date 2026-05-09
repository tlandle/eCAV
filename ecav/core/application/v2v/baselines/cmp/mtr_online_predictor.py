# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
MTR (Motion Transformer) online prediction for CMP baseline.

Builds the MTR batch_dict from live CARLA tracked object state and
CoBEVT fused BEV features. Produces multi-modal trajectory predictions
matching CMP's offline evaluation pipeline.

The offline CMP eval pipeline:
  1. OPV2V dataset provides GT/pred trajectory history + BEV lane images
  2. CoBEVT provides fused BEV features
  3. MTR takes all of the above and predicts future trajectories

For online inference:
  1. AB3DMOT tracker provides trajectory history (past 10 frames)
  2. CARLA HD map provides BEV lane image (rendered via OpenCDA map_manager)
  3. CoBEVT provides fused BEV features (from perception manager)
  4. MTR predicts future trajectories

Key difference from offline: no GT future trajectories (not needed for inference).
"""
from __future__ import annotations

import logging
import os
import sys
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch

logger = logging.getLogger("MTROnlinePredictor")

# CMP dataset config defaults matching OPV2V
_PAST_FRAMES = 10
_FUTURE_FRAMES = 50
_TIME_INTERVAL = 0.1  # seconds between frames


class TrajectoryHistoryBuffer:
    """
    Maintains per-object trajectory history for MTR input.

    Each tick, call update() with the current tracked objects.
    Call get_past_trajectories() to get the 10-frame history
    in the format MTR expects.

    Per-object state: [x, y, z, l, w, h, heading, valid]
    """

    def __init__(self, max_history: int = _PAST_FRAMES + 1):
        self.max_history = max_history
        # {track_id: deque of [x, y, z, l, w, h, heading, valid]}
        self._tracks: Dict[int, deque] = {}
        self._tick = 0

    def update(self, track_states: List[Dict]):
        """
        Update with current frame's tracked objects.

        Parameters
        ----------
        track_states : list of dict
            Each dict has: track_id, x, y, z, l, w, h, heading
        """
        active_ids = set()
        for ts in track_states:
            tid = ts['track_id']
            active_ids.add(tid)
            state = np.array([
                ts['x'], ts['y'], ts['z'],
                ts['l'], ts['w'], ts['h'],
                ts['heading'], 1.0  # valid=1
            ], dtype=np.float32)

            if tid not in self._tracks:
                self._tracks[tid] = deque(maxlen=self.max_history)
                # Pad with invalid states for missing history
                for _ in range(self.max_history - 1):
                    self._tracks[tid].append(np.zeros(8, dtype=np.float32))

            self._tracks[tid].append(state)

        # Mark inactive tracks with invalid state
        for tid in list(self._tracks.keys()):
            if tid not in active_ids:
                # Keep the track but mark current frame as invalid
                invalid = np.zeros(8, dtype=np.float32)
                self._tracks[tid].append(invalid)
                # Remove tracks with no valid states in recent history
                recent = list(self._tracks[tid])[-5:]
                if all(s[7] == 0 for s in recent):
                    del self._tracks[tid]

        self._tick += 1

    def get_past_trajectories(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get trajectory history for all active tracks.

        Returns
        -------
        obj_trajs_past : np.ndarray, shape (N, past_frames+1, 8)
            [x, y, z, l, w, h, heading, valid] for each object
        obj_ids : np.ndarray, shape (N,)
            Track IDs
        """
        if not self._tracks:
            return np.empty((0, _PAST_FRAMES + 1, 8), dtype=np.float32), np.array([], dtype=np.int64)

        trajs = []
        ids = []
        for tid, history in self._tracks.items():
            # Take the last (PAST_FRAMES+1) states
            states = list(history)
            n = len(states)
            if n < _PAST_FRAMES + 1:
                # Pad front with invalid
                pad = [np.zeros(8, dtype=np.float32)] * (_PAST_FRAMES + 1 - n)
                states = pad + states
            else:
                states = states[-(  _PAST_FRAMES + 1):]

            trajs.append(np.array(states, dtype=np.float32))
            ids.append(tid)

        return np.array(trajs), np.array(ids, dtype=np.int64)

    @property
    def num_tracks(self) -> int:
        return len(self._tracks)


class BEVLaneRenderer:
    """
    Renders BEV lane images from CARLA's HD map.

    Produces 256x256x3 top-down lane images matching OPV2V's
    additional/bev_lane.png format.
    """

    def __init__(self, carla_world, pixels_per_meter: float = 2.0,
                 image_size: int = 256):
        self.world = carla_world
        self.ppm = pixels_per_meter
        self.size = image_size
        self._map = carla_world.get_map()
        self._waypoint_cache = {}
        logger.info("BEV lane renderer initialized (size=%dx%d, ppm=%.1f)",
                     image_size, image_size, pixels_per_meter)

    def render(self, ego_location, ego_heading_rad: float) -> np.ndarray:
        """
        Render BEV lane image centered at ego position.

        Parameters
        ----------
        ego_location : carla.Location or [x, y, z]
        ego_heading_rad : float, ego heading in radians

        Returns
        -------
        lane_image : np.ndarray, shape (256, 256, 3), uint8
        """
        import carla
        import cv2

        if hasattr(ego_location, 'x'):
            cx, cy = ego_location.x, ego_location.y
        else:
            cx, cy = ego_location[0], ego_location[1]

        # Create blank image
        img = np.zeros((self.size, self.size, 3), dtype=np.uint8)

        # Query waypoints in the area
        radius = self.size / (2 * self.ppm)  # meters
        center_wp = self._map.get_waypoint(
            carla.Location(x=cx, y=cy, z=0),
            project_to_road=True,
            lane_type=carla.LaneType.Driving)

        if center_wp is None:
            return img

        # Generate waypoints along nearby lanes
        waypoints = self._map.generate_waypoints(2.0)

        # Draw lanes
        cos_h = np.cos(-ego_heading_rad)
        sin_h = np.sin(-ego_heading_rad)
        half = self.size // 2

        for wp in waypoints:
            # Transform to ego-centric coordinates
            dx = wp.transform.location.x - cx
            dy = wp.transform.location.y - cy

            # Rotate by ego heading
            rx = dx * cos_h - dy * sin_h
            ry = dx * sin_h + dy * cos_h

            # Check if within image bounds
            if abs(rx) > radius or abs(ry) > radius:
                continue

            # Convert to pixel coordinates
            px = int(half + rx * self.ppm)
            py = int(half - ry * self.ppm)  # flip y for image coords

            if 0 <= px < self.size and 0 <= py < self.size:
                # Draw lane marking
                lane_color = (255, 217, 82)  # yellow for normal lanes
                cv2.circle(img, (px, py), 2, lane_color, -1)

                # Draw lane width
                lane_width = wp.lane_width * self.ppm
                road_color = (17, 17, 31)
                # Draw a small rectangle for road surface
                hw = int(lane_width / 2)
                x1 = max(0, px - hw)
                x2 = min(self.size, px + hw)
                y1 = max(0, py - 1)
                y2 = min(self.size, py + 1)
                img[y1:y2, x1:x2] = road_color

        return img


class MTROnlinePredictor:
    """
    Runs MTR prediction online from tracked object state.

    Maintains trajectory history, renders BEV lanes, builds MTR
    batch_dict, and runs the model for multi-modal prediction.
    """

    def __init__(self, cmp_root: str, carla_world, cfg: Dict = None,
                 device: str = 'cuda:0'):
        self.device = device
        self._cmp_root = cmp_root
        self.world = carla_world

        # Trajectory history per vehicle
        self._history_buffers: Dict[int, TrajectoryHistoryBuffer] = {}

        # BEV lane renderer
        self._lane_renderer = BEVLaneRenderer(carla_world)

        # Load MTR model
        self._load_mtr_model(cmp_root, cfg or {})

        logger.info("MTR online predictor initialized")

    def _load_mtr_model(self, cmp_root: str, cfg: Dict):
        """Load the MTR prediction model."""
        mtr_root = os.path.join(cmp_root, 'MTR')
        if mtr_root not in sys.path:
            sys.path.insert(0, mtr_root)

        mtr_cfg_path = cfg.get('mtr_cfg',
            os.path.join(mtr_root, 'tools', 'cfgs', 'opv2v',
                         'opv2v_multiego_cobevt_c256.yaml'))
        mtr_ckpt = cfg.get('mtr_checkpoint',
            os.path.join(mtr_root, 'output', 'opv2v_multiego_cobevt_c256',
                         'ckpt', 'best_model.pth'))

        if not os.path.exists(mtr_ckpt):
            logger.warning("MTR checkpoint not found: %s", mtr_ckpt)
            self.model = None
            return

        from mtr.config import cfg as mtr_cfg, cfg_from_yaml_file
        from mtr.models_opv2v.multi_ego_mtr_model import (
            MotionTransformerWithMultiEgoAggregation)

        cfg_from_yaml_file(mtr_cfg_path, mtr_cfg)

        # Fix relative LANE_ENCODER path to absolute
        if not os.path.isabs(mtr_cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
            mtr_cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
                cmp_root, mtr_cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)

        self.mtr_cfg = mtr_cfg
        self.model = MotionTransformerWithMultiEgoAggregation(mtr_cfg.MODEL)
        self.model.to(self.device).eval()

        state_dict = torch.load(mtr_ckpt, map_location=self.device)
        if 'model_state' in state_dict:
            state_dict = state_dict['model_state']
        self.model.load_state_dict(state_dict, strict=False)
        logger.info("MTR model loaded from %s", mtr_ckpt)

    def get_or_create_buffer(self, veh_idx: int) -> TrajectoryHistoryBuffer:
        if veh_idx not in self._history_buffers:
            self._history_buffers[veh_idx] = TrajectoryHistoryBuffer()
        return self._history_buffers[veh_idx]

    def update_tracks(self, veh_idx: int, track_results: np.ndarray):
        """
        Update trajectory history with current frame's tracker output.

        Parameters
        ----------
        veh_idx : int
        track_results : np.ndarray, shape (N, 12)
            CMP's AB3DMOT output: [h,w,l,x,y,z,theta,ID,...]
        """
        buf = self.get_or_create_buffer(veh_idx)
        states = []
        if track_results is not None and len(track_results) > 0:
            for trk in track_results:
                states.append({
                    'track_id': int(trk[7]),
                    'x': float(trk[3]),
                    'y': float(trk[4]),
                    'z': float(trk[5]),
                    'l': float(trk[2]),
                    'w': float(trk[1]),
                    'h': float(trk[0]),
                    'heading': float(trk[6]),
                })
        buf.update(states)
        logger.info("MTR update_tracks veh=%d: %d tracks input, "
                     "buffer now has %d active tracks, tick=%d, "
                     "track_ids=%s",
                     veh_idx, len(states), buf.num_tracks, buf._tick,
                     [s['track_id'] for s in states])

    def predict(self, veh_idx: int, ego_pose, fused_feature: torch.Tensor
                ) -> Optional[Dict]:
        """
        Run MTR prediction for a vehicle.

        Parameters
        ----------
        veh_idx : int
        ego_pose : Transform with .location and .rotation
        fused_feature : torch.Tensor, shape [1, 256, 48, 176]

        Returns
        -------
        pred_dict : dict with 'pred_trajs' and 'pred_scores', or None
        """
        if self.model is None:
            logger.debug("MTR model not loaded")
            return None

        buf = self.get_or_create_buffer(veh_idx)
        if buf.num_tracks == 0:
            logger.info("MTR veh=%d: no tracks in buffer (num_tracks=%d)", veh_idx, buf.num_tracks)
            return None

        obj_trajs_past, obj_ids = buf.get_past_trajectories()
        if len(obj_trajs_past) == 0:
            logger.info("MTR veh=%d: empty trajectory history", veh_idx)
            return None

        logger.info("MTR veh=%d: %d tracks, history=%s, ids=%s",
                     veh_idx, len(obj_trajs_past),
                     obj_trajs_past.shape, obj_ids)

        # Render BEV lane image
        ego_heading = np.radians(ego_pose.rotation.yaw)
        lane_image = self._lane_renderer.render(ego_pose.location, ego_heading)

        # Build MTR batch_dict
        batch_dict = self._build_batch_dict(
            obj_trajs_past, obj_ids, lane_image, fused_feature, ego_pose)

        if batch_dict is None:
            logger.info("MTR veh=%d: batch_dict build failed", veh_idx)
            return None

        logger.info("MTR veh=%d: batch_dict built, running model", veh_idx)

        # Run MTR
        try:
            with torch.no_grad():
                output = self.model(batch_dict)
            logger.info("MTR veh=%d: output keys=%s", veh_idx, list(output.keys()))
            return output
        except Exception as e:
            logger.warning("MTR veh=%d prediction failed: %s", veh_idx, e)
            import traceback
            logger.warning(traceback.format_exc())
            return None

    def _build_batch_dict(self, obj_trajs_past, obj_ids, lane_image,
                           fused_feature, ego_pose):
        """
        Build MTR's batch_dict from online data.

        Matches the format produced by OPV2VMultiEgoDataset.organize_trajectory_timestamp_into_dict
        """
        from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

        num_objects = len(obj_trajs_past)
        if num_objects == 0:
            return None

        # obj_trajs_past: (N, past_frames+1, 8) = [x,y,z,l,w,h,heading,valid]
        # Convert headings to radians (already in radians from tracker)

        # All objects are vehicles
        obj_types = np.array(['TYPE_VEHICLE'] * num_objects)
        track_index_to_predict = np.arange(num_objects)
        sdc_track_index = 0

        # Timestamps for past frames
        timestamps = np.array([
            _TIME_INTERVAL * i for i in range(_PAST_FRAMES + 1)
        ], dtype=np.float32)

        # Get center objects (current state of objects to predict)
        center_objects = obj_trajs_past[:, -1, :].copy()  # last frame = current

        # Create dummy future trajectories (not used in inference)
        obj_trajs_future = np.zeros(
            (num_objects, _FUTURE_FRAMES, 8), dtype=np.float32)

        # Use CMP's own trajectory processing
        try:
            (obj_trajs_data, obj_trajs_mask, obj_trajs_pos, obj_trajs_last_pos,
             obj_trajs_future_state, obj_trajs_future_mask,
             center_gt_trajs, center_gt_trajs_mask,
             center_gt_final_valid_idx,
             track_index_to_predict_new, sdc_track_index_new,
             obj_types_out, obj_ids_out
             ) = OPV2VMultiEgoDataset.create_agent_data_for_center_objects(
                center_objects=center_objects,
                obj_trajs_past=obj_trajs_past,
                obj_trajs_future=obj_trajs_future,
                track_index_to_predict=track_index_to_predict,
                sdc_track_index=sdc_track_index,
                timestamps=timestamps,
                obj_types=obj_types,
                obj_ids=obj_ids,
            )
        except Exception as e:
            import traceback
            logger.info("create_agent_data failed: %s\n%s", e, traceback.format_exc())
            return None

        # Build lane image tensor
        lane_tensor = np.expand_dims(lane_image, axis=0)
        lane_tensor = np.repeat(lane_tensor, center_objects.shape[0], axis=0)

        # Build the ret_dict matching OPV2V dataset format
        ret_dict = {
            'scenario_id': np.array([0] * len(track_index_to_predict)),
            'scenario_name': np.array(['carla_online'] * len(track_index_to_predict)),
            'ego_cav_id': np.array(['ego'] * len(track_index_to_predict)),
            'timestamp_idx': np.array([self._history_buffers.get(0, TrajectoryHistoryBuffer())._tick] * len(track_index_to_predict)),
            'obj_trajs': obj_trajs_data,
            'obj_trajs_mask': obj_trajs_mask,
            'track_index_to_predict': track_index_to_predict_new,
            'obj_trajs_pos': obj_trajs_pos,
            'obj_trajs_last_pos': obj_trajs_last_pos,
            'obj_types': obj_types_out,
            'obj_ids': obj_ids_out,
            'center_objects_world': center_objects,
            'center_objects_id': obj_ids_out[track_index_to_predict_new] if len(track_index_to_predict_new) > 0 else np.array([]),
            'center_objects_type': obj_types_out[track_index_to_predict_new] if len(track_index_to_predict_new) > 0 else np.array([]),
            'obj_trajs_future_state': obj_trajs_future_state,
            'obj_trajs_future_mask': obj_trajs_future_mask,
            'center_gt_trajs': center_gt_trajs,
            'center_gt_trajs_mask': center_gt_trajs_mask,
            'center_gt_final_valid_idx': center_gt_final_valid_idx,
            'center_gt_trajs_src': np.zeros((len(track_index_to_predict), _PAST_FRAMES + 1 + _FUTURE_FRAMES, 8), dtype=np.float32),
            'map_polylines': lane_tensor,
            'fused_feature': fused_feature.cpu().numpy() if isinstance(fused_feature, torch.Tensor) else fused_feature,
        }

        # Collate manually (the dataset's collate_batch is an instance method)
        # For single-CAV online inference, wrap tensors with batch dim
        from mtr.utils import common_utils

        input_dict = {}
        for key, val in ret_dict.items():
            if key in ['obj_trajs', 'map_polylines',
                       'obj_trajs_pos', 'obj_trajs_last_pos',
                       'obj_trajs_future_state']:
                input_dict[key] = torch.from_numpy(val).float() if isinstance(val, np.ndarray) else val
            elif key in ['obj_trajs_mask', 'obj_trajs_future_mask']:
                input_dict[key] = torch.from_numpy(val).bool() if isinstance(val, np.ndarray) else val.bool()
            elif key in ['scenario_id', 'scenario_name', 'timestamp_idx',
                         'timestamp_key', 'ego_cav_id', 'obj_types', 'obj_ids',
                         'center_objects_type', 'center_objects_id']:
                input_dict[key] = val
            elif key in ['fused_feature']:
                input_dict[key] = [torch.from_numpy(val).float()] if isinstance(val, np.ndarray) else [val]
            elif key in ['track_index_to_predict']:
                input_dict[key] = torch.from_numpy(val).long() if isinstance(val, np.ndarray) else val
            elif isinstance(val, np.ndarray):
                input_dict[key] = torch.from_numpy(val).float()
            else:
                input_dict[key] = val

        batch = {
            'input_dict': input_dict,
            'batch_sample_count': [len(track_index_to_predict)],
            'num_cavs': 1,
        }

        # Move tensors to device
        for key, val in batch['input_dict'].items():
            if isinstance(val, torch.Tensor):
                batch['input_dict'][key] = val.to(self.device)
            elif isinstance(val, list) and len(val) > 0 and isinstance(val[0], torch.Tensor):
                batch['input_dict'][key] = [v.to(self.device) for v in val]

        return batch
