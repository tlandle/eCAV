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
_CMP_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', 'application', 'v2v', 'baselines', 'cmp', 'CMP'))


def _ensure_cmp_path():
    """Add CMP root to sys.path so cmp_opencood is importable."""
    if _CMP_ROOT not in sys.path:
        sys.path.insert(0, _CMP_ROOT)


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
        from cmp_opencood.hypes_yaml.yaml_utils import load_yaml
        import cmp_opencood.tools.train_utils as train_utils
        from cmp_opencood.data_utils.opv2v.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor

        self.hypes = load_yaml(str(hypes_path))
        logger.info("CoBEVT hypes loaded from %s", hypes_path)

        # Voxel preprocessor (same as CMP uses)
        pre_config = self.hypes["preprocess"]
        self._vp = SpVoxelPreprocessor(pre_config, train=False)

        # Load CoBEVT model
        import torch
        from cmp_opencood.models.point_pillar_cobevt import PointPillarCoBEVT
        self.model = PointPillarCoBEVT(self.hypes['model']['args']).to(self.device).eval()

        state_dict = torch.load(str(ckpt_path), map_location=self.device)
        self.model.load_state_dict(state_dict, strict=False)
        logger.info("CoBEVT model loaded from %s", ckpt_path)

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
        import cmp_opencood.tools.train_utils as train_utils
        batch = train_utils.to_device(batch, self.device)

        # Extract features only: VFE -> scatter -> backbone -> shrink -> encode.
        # Detection heads require CoBEVT fusion which runs in the V2V manager
        # with all received features (self + peers).
        import time as _time

        batch_dict = {
            'voxel_features': batch['processed_lidar']['voxel_features'],
            'voxel_coords': batch['processed_lidar']['voxel_coords'],
            'voxel_num_points': batch['processed_lidar']['voxel_num_points'],
            'record_len': batch['record_len'],
        }

        t_fwd = _time.time()
        batch_dict = self.model.pillar_vfe(batch_dict)
        batch_dict = self.model.scatter(batch_dict)
        batch_dict = self.model.backbone(batch_dict)
        spatial_features_2d = batch_dict['spatial_features_2d']
        if self.model.shrink_flag:
            spatial_features_2d = self.model.shrink_conv(spatial_features_2d)
        fwd_ms = (_time.time() - t_fwd) * 1000

        # Encode for V2V transmission
        t_enc = _time.time()
        if self.model.compression:
            compressed = self.model.naive_compressor.encoder(spatial_features_2d)
        else:
            compressed = spatial_features_2d
        encode_ms = (_time.time() - t_enc) * 1000

        # Store compressed features for V2V broadcast.
        # No psm/rm here. Detection requires fusion which runs in the
        # CMP manager with all received features.
        self.feature_dict = {
            'spatial_features': compressed.cpu(),
            'compressed': self.model.compression,
        }
        self.feature_map = compressed.cpu().half()

        if self._first_run:
            raw_bytes = spatial_features_2d.nelement() * spatial_features_2d.element_size()
            tx_bytes = compressed.nelement() * compressed.element_size()
            ratio = raw_bytes / max(tx_bytes, 1)
            logger.info("CoBEVT extract OK. "
                        "Pre-compress=%s (%dKB), "
                        "Post-compress=%s (%dKB, %.0fx), "
                        "backbone=%.1fms encode=%.1fms",
                        list(spatial_features_2d.shape), raw_bytes / 1024,
                        list(compressed.shape), tx_bytes / 1024, ratio,
                        fwd_ms, encode_ms)
            self._first_run = False

    def _build_batch(self) -> Dict[str, Any]:
        """Build input batch from raw LiDAR (same preprocessing as OPV2V)."""
        lidar_np = np.ascontiguousarray(self.lidar.data, dtype=np.float32)

        # Note: OPV2V ground plane is at z~-1.96, ours at z~-2.62 in sensor
        # frame. PointPillar's 4m z-voxel [-3,1] accommodates both.

        # Save every tick's point cloud for offline analysis
        _veh_id = self.vehicle.id if hasattr(self, 'vehicle') and self.vehicle else 'unknown'
        if not hasattr(self, '_dump_tick'):
            self._dump_tick = 0
            os.makedirs('/tmp/carla_scene_dump', exist_ok=True)
        np.save(f'/tmp/carla_scene_dump/veh{_veh_id}_tick{self._dump_tick:04d}.npy', lidar_np)
        self._dump_tick += 1

        logger.info("RAW LIDAR veh=%s tick=%d: shape=%s "
                     "x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f] "
                     "intensity=[%.3f,%.3f,mean=%.3f] "
                     "n_points=%d",
                     _veh_id, self._dump_tick - 1,
                     lidar_np.shape,
                     lidar_np[:, 0].min(), lidar_np[:, 0].max(),
                     lidar_np[:, 1].min(), lidar_np[:, 1].max(),
                     lidar_np[:, 2].min(), lidar_np[:, 2].max(),
                     lidar_np[:, 3].min(), lidar_np[:, 3].max(),
                     lidar_np[:, 3].mean(),
                     len(lidar_np))

        # Match OPV2V preprocessing pipeline:
        # 1. Shuffle points
        np.random.shuffle(lidar_np)

        # 2. Remove ego vehicle points (same mask as OPV2V)
        ego_mask = ((lidar_np[:, 0] >= -1.95) & (lidar_np[:, 0] <= 2.95) &
                    (lidar_np[:, 1] >= -1.1) & (lidar_np[:, 1] <= 1.1))
        lidar_np = lidar_np[~ego_mask]

        # 3. Clip to BEV range [-140.8, -38.4, -3, 140.8, 38.4, 1]
        lidar_range = self.hypes.get('preprocess', {}).get(
            'cav_lidar_range', [-140.8, -38.4, -3, 140.8, 38.4, 1])
        range_mask = ((lidar_np[:, 0] >= lidar_range[0]) &
                      (lidar_np[:, 0] <= lidar_range[3]) &
                      (lidar_np[:, 1] >= lidar_range[1]) &
                      (lidar_np[:, 1] <= lidar_range[4]) &
                      (lidar_np[:, 2] >= lidar_range[2]) &
                      (lidar_np[:, 2] <= lidar_range[5]))
        lidar_np = lidar_np[range_mask]

        if self._first_run:
            logger.info("AFTER PREPROC: %d pts (ego_removed=%d, range_clipped), "
                        "x=[%.1f,%.1f] y=[%.1f,%.1f] z=[%.1f,%.1f]",
                        len(lidar_np), ego_mask.sum(),
                        lidar_np[:, 0].min() if len(lidar_np) > 0 else 0,
                        lidar_np[:, 0].max() if len(lidar_np) > 0 else 0,
                        lidar_np[:, 1].min() if len(lidar_np) > 0 else 0,
                        lidar_np[:, 1].max() if len(lidar_np) > 0 else 0,
                        lidar_np[:, 2].min() if len(lidar_np) > 0 else 0,
                        lidar_np[:, 2].max() if len(lidar_np) > 0 else 0)

        proc_lidar_np = self._vp.preprocess(lidar_np)

        if self._first_run:
            logger.info("VOXELIZED: n_voxels=%d voxel_features=%s "
                        "voxel_coords=%s voxel_num_points=%s",
                        len(proc_lidar_np.get('voxel_features', [])),
                        proc_lidar_np['voxel_features'].shape if 'voxel_features' in proc_lidar_np else '?',
                        proc_lidar_np['voxel_coords'].shape if 'voxel_coords' in proc_lidar_np else '?',
                        proc_lidar_np['voxel_num_points'].shape if 'voxel_num_points' in proc_lidar_np else '?')
        proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}

        # Add batch index column to voxel coords
        coords = proc_lidar['voxel_coords']
        batch_col = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
        proc_lidar['voxel_coords'] = torch.cat((batch_col, coords), dim=1).int()

        return {
            "processed_lidar": proc_lidar,
            "record_len": torch.tensor([1], dtype=torch.int64),
        }
