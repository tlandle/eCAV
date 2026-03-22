# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
BM2CP edge manager with AB3DMOT tracking and linear prediction.

Subclasses WorldFusionEdge, replacing the WorldFusion cooperative perception
model with PointPillarBM2CP (Bi-directional Multi-scale Camera + PointCloud).

Pipeline (identical to WorldFusion except steps 1-2):
    Vehicles/RSUs run BM2CPPerceptionManager.get_feature(), which extracts
    spatial_features + thres_map via multimodal fusion (camera + lidar) and
    transmits them to the edge.

    Edge:
        1. Collect spatial_features + thres_map from all agents
        2. Run backbone on each agent's features
        3. Run BM2CP collaborative fusion (AttenComm) with pairwise transforms
        4. Run detection heads on fused features
        5. Post-process to world-frame detections
        6. AB3DMOT tracking
        7. Linear prediction (25 future steps)
        8. Distribute predictions to vehicles
"""
from __future__ import annotations
import os
from typing import Dict, List, Tuple, Any

import torch

from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.tools import train_utils

from .edge_manager_worldfusion_ab3dmot_linear_predictor import WorldFusionEdge


class BM2CPEdge(WorldFusionEdge):
    """
    Edge manager using PointPillarBM2CP for cooperative perception.

    Inherits the full tracking, prediction, evaluation, and distribution
    pipeline from WorldFusionEdge. Overrides model loading and the fusion
    step to use BM2CP's multimodal (camera + lidar) feature extractor and
    AttenComm collaborative fusion.
    """

    def __init__(self, world, cfg, cav_world, carla_client, *, world_dt=0.05, **kwargs):
        # WorldFusionEdge.__init__ will call _load_model, which we override below.
        # We must set a sentinel before super().__init__ so _load_model knows
        # it is being called from BM2CPEdge context.
        self._is_bm2cp = True
        super().__init__(world, cfg, cav_world, carla_client, world_dt=world_dt, **kwargs)

    # ------------------------------------------------------------------
    # Override: model loading
    # ------------------------------------------------------------------
    def _load_model(self, cfg):
        """Load PointPillarBM2CP from the bm2cp_model config block."""
        from opencood.models.point_pillar_bm2cp import PointPillarBM2CP

        bm_cfg = cfg['bm2cp_model']
        hypes = load_yaml(bm_cfg['hypes_yaml'])
        self.hypes = hypes

        self.world_anchor = cfg.get('world_anchor', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print(f"[BM2CP Edge] World anchor: {self.world_anchor}")

        self.model = PointPillarBM2CP(hypes['model']['args']).cuda().eval()
        ckpt_dir = os.path.dirname(bm_cfg['checkpoint'])
        epoch_id = int(bm_cfg['checkpoint'].split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(ckpt_dir, self.model, epoch_id)
        print(f"[BM2CP Edge] Model loaded from epoch {epoch_id}.")

        from opencood.data_utils.post_processor.world_voxel_postprocessor import WorldVoxelPostprocessor
        self.post_processor = WorldVoxelPostprocessor(
            self.hypes['postprocess'],
            dataset=None,
            train=False
        )

    # ------------------------------------------------------------------
    # Override: cooperative fusion step
    # ------------------------------------------------------------------
    def _run_fusion(self,
                    spatial_features: torch.Tensor,
                    pairwise_t_matrix: torch.Tensor,
                    record_len: torch.Tensor) -> Tuple[torch.Tensor, Dict]:
        """
        Run BM2CP backbone + AttenComm collaborative fusion.

        Agents have already run the camera+lidar multimodal fusion locally
        (inside BM2CPPerceptionManager.get_feature()) and transmitted
        spatial_features + thres_map to the edge.  Here we:
          1. Run the BEV backbone on each agent's raw spatial_features.
          2. Optionally shrink the 2-D feature maps.
          3. Call fusion_net (AttenComm) for collaborative spatial fusion.
          4. Apply detection heads to get psm / rm.

        Returns
        -------
        fused_feature : torch.Tensor   [1, C, H, W]
        pred_dict     : dict           {'psm': ..., 'rm': ...}
        """
        N = spatial_features.shape[0]

        # Stack thres_maps if available from feature_dicts collected in update_information.
        # Fall back to zeros if any agent did not transmit a thres_map.
        thres_maps = []
        for fd in self._current_feature_dicts:
            tm = fd.get('thres_map')
            if tm is not None:
                thres_maps.append(tm.float().cuda())
            else:
                # shape placeholder: will be ignored by AttenComm if None
                thres_maps.append(None)

        # Run backbone on each agent's raw spatial_features
        bev_features = []
        psm_single_list = []
        for i in range(N):
            bd = {'spatial_features': spatial_features[i:i+1]}
            bd = self.model.backbone(bd)
            bev2d = bd['spatial_features_2d']
            if self.model.shrink_flag:
                bev2d = self.model.shrink_conv(bev2d)
            if self.model.compression:
                bev2d = self.model.naive_compressor(bev2d)
            bev_features.append(bev2d)
            psm_single_list.append(self.model.cls_head(bev2d))

        spatial_2d = torch.cat(bev_features, dim=0)   # [N, C, H, W]
        psm_single = torch.cat(psm_single_list, dim=0)  # [N, anchors, H, W]

        # Concatenate thres_maps if all are present; otherwise pass None
        all_have_thres = all(tm is not None for tm in thres_maps)
        thres_map_batch = torch.cat(thres_maps, dim=0) if all_have_thres else None

        # Collaborative fusion via AttenComm
        if self.model.multi_scale:
            fused_feature, _, _ = self.model.fusion_net(
                spatial_features,      # raw pre-backbone features for multi-scale warping
                psm_single,
                thres_map_batch,
                record_len,
                pairwise_t_matrix,
                self.model.backbone,
                [self.model.shrink_conv if self.model.shrink_flag else None,
                 self.model.cls_head, self.model.reg_head]
            )
            if self.model.shrink_flag:
                fused_feature = self.model.shrink_conv(fused_feature)
        else:
            fused_feature, _, _ = self.model.fusion_net(
                spatial_2d,
                psm_single,
                thres_map_batch,
                record_len,
                pairwise_t_matrix
            )

        pred_dict = {
            'psm': self.model.cls_head(fused_feature),
            'rm':  self.model.reg_head(fused_feature),
        }
        return fused_feature, pred_dict

    # ------------------------------------------------------------------
    # Override: collect feature_dicts so _run_fusion can access thres_map
    # ------------------------------------------------------------------
    def update_information(self, frame_idx: int):
        """
        Extend WorldFusionEdge.update_information to cache feature_dicts
        for thres_map access inside _run_fusion.
        """
        # We hook into the parent's logic by capturing feature_dicts before
        # they are pushed to the jitter buffer.  The parent already stores
        # them there, but _run_fusion fires after the buffer drain so the
        # original list reference would be stale.  We keep our own copy.
        self._current_feature_dicts = []
        for vm in self.vehicle_manager_list:
            pm = vm.perception_manager
            if hasattr(pm, 'feature_dict') and pm.feature_dict is not None:
                self._current_feature_dicts.append(pm.feature_dict)
        for rsu in self.rsu_manager_list:
            pm = rsu.perception_manager
            if hasattr(pm, 'feature_dict') and pm.feature_dict is not None:
                self._current_feature_dicts.append(pm.feature_dict)

        super().update_information(frame_idx)
