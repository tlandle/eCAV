# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
CoBEVT perception manager for CMP baseline.

Runs CMP's CoBEVT (PointPillar + SwapFusionEncoder) feature extraction
from raw LiDAR data. Produces compressed BEV features (256x) for V2V
intermediate fusion.

This is the perception backend for the CMP baseline (Wang et al., RA-L 2025).
It uses the actual CMP/opencood code for feature extraction to ensure
the features are compatible with CoBEVT's fusion module.

Usage in YAML:
    perception:
      backend: cobevt
      cobevt_model:
        hypes_yaml: path/to/config.yaml
        checkpoint: path/to/net_epoch25.pth
"""
from __future__ import annotations

import os
import sys
import pathlib
import logging
from typing import Any, Dict, Optional

import numpy as np
import torch

from ecav.core.sensing.perception.perception_manager import PerceptionManager
import ecav.core.sensing.perception.sensor_transformation as st

logger = logging.getLogger(__name__)

# CMP's opencood is separate from ecav's worldfusion/opencood.
# We import CMP's opencood from the baselines/cmp/CMP directory.
_CMP_OPENCOOD_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'application', 'v2v', 'baselines', 'cmp', 'CMP'))


def _ensure_cmp_path():
    """Add CMP's opencood to sys.path (before ecav's opencood)."""
    if _CMP_OPENCOOD_ROOT not in sys.path:
        # Insert at position 0 so CMP's opencood takes priority
        # over ecav's worldfusion/opencood when this manager is active
        sys.path.insert(0, _CMP_OPENCOOD_ROOT)


class CoBEVTPerceptionManager(PerceptionManager):
    """
    CoBEVT feature extraction for the CMP V2V baseline.

    Runs CMP's PointPillar VFE + BaseBEVBackbone + NaiveCompressor (256x)
    on raw LiDAR point clouds. Produces spatial_features in feature_dict
    for the V2V manager to broadcast over sidelink.

    LiDAR-only (no cameras needed, unlike BM2CP).
    """

    def __init__(
        self,
        vehicle,
        config_yaml: Dict[str, Any],
        cav_world=None,
        data_dump: bool = False,
        carla_world=None,
        infra_id: Optional[int] = None,
        tracking_manager=None,
        debug_helper=None,
        device: str = "cuda:0",
        **kw,
    ):
        super().__init__(
            vehicle=vehicle,
            config_yaml=config_yaml,
            cav_world=cav_world,
            data_dump=data_dump,
            carla_world=carla_world,
            infra_id=infra_id,
            tracking_manager=tracking_manager,
            debug_helper=debug_helper)

        self.device = device

        # Temporarily modify sys.path to import CMP's opencood
        _ensure_cmp_path()

        model_config = config_yaml.get('cobevt_model', config_yaml.get('bm2cp_model', {}))
        hypes_path = pathlib.Path(model_config['hypes_yaml'])
        ckpt_path = pathlib.Path(model_config['checkpoint'])

        # Import CMP's opencood modules
        from opencood.hypes_yaml.yaml_utils import load_yaml
        import opencood.tools.train_utils as train_utils
        from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor

        self.hypes = load_yaml(str(hypes_path))
        logger.info("CoBEVT hypes loaded from %s", hypes_path)

        # Voxel preprocessor (same as CMP uses)
        pre_config = self.hypes["preprocess"]
        self._vp = SpVoxelPreprocessor(pre_config, train=False)

        # Load CoBEVT model
        from opencood.models.point_pillar_cobevt import PointPillarCoBEVT
        self.model = PointPillarCoBEVT(self.hypes['model']['args']).to(self.device).eval()

        epoch_id = int(str(ckpt_path.name).split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(
            str(ckpt_path.parent), self.model, epoch=epoch_id)
        logger.info("CoBEVT model loaded (epoch %d)", epoch_id)

        self.feature_dict = None
        self.feature_map = None
        self._first_run = True

    def detect(self, ego_pos, **kw):
        """Run CoBEVT feature extraction if LiDAR data is ready."""
        if self.lidar and self.lidar.data is not None:
            self.run_step()
        return {"vehicles": [], "traffic_lights": [], "static": []}

    @torch.inference_mode()
    def run_step(self):
        """Extract CoBEVT BEV features from raw LiDAR."""
        batch = self._build_batch()

        _ensure_cmp_path()
        import opencood.tools.train_utils as train_utils
        batch = train_utils.to_device(batch, self.device)

        # Run through PointPillar VFE + scatter + backbone + shrink + compress
        # This matches the first half of CoBEVT's forward() method,
        # before the fusion step (which happens in the V2V manager)
        batch_dict = {
            'voxel_features': batch['processed_lidar']['voxel_features'],
            'voxel_coords': batch['processed_lidar']['voxel_coords'],
            'voxel_num_points': batch['processed_lidar']['voxel_num_points'],
            'record_len': batch['record_len'],
        }

        batch_dict = self.model.pillar_vfe(batch_dict)
        batch_dict = self.model.scatter(batch_dict)
        batch_dict = self.model.backbone(batch_dict)

        spatial_features_2d = batch_dict['spatial_features_2d']

        if self.model.shrink_flag:
            spatial_features_2d = self.model.shrink_conv(spatial_features_2d)

        if self.model.compression:
            spatial_features_2d = self.model.naive_compressor(spatial_features_2d)

        # Store features for the V2V manager to collect and broadcast
        self.feature_dict = {
            'spatial_features': spatial_features_2d.cpu(),
        }
        self.feature_map = self.feature_dict['spatial_features'].half()

        if self._first_run:
            feat_bytes = spatial_features_2d.nelement() * spatial_features_2d.element_size()
            logger.info("CoBEVT feature extraction OK. "
                        "Shape=%s, size=%dB (%.1fKB)",
                        list(spatial_features_2d.shape),
                        feat_bytes, feat_bytes / 1024)
            self._first_run = False

    def _build_batch(self) -> Dict[str, Any]:
        """Build input batch from raw LiDAR (same voxelization as CMP)."""
        lidar_np = np.ascontiguousarray(self.lidar.data, dtype=np.float32)
        proc_lidar_np = self._vp.preprocess(lidar_np)
        proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}

        # Add batch index column to voxel coords
        coords = proc_lidar['voxel_coords']
        batch_col = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
        proc_lidar['voxel_coords'] = torch.cat((batch_col, coords), dim=1).int()

        return {
            "processed_lidar": proc_lidar,
            "record_len": torch.tensor([1], dtype=torch.int64),
        }
