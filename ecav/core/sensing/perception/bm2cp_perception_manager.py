# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
BM2CP perception manager.

Handles BM2CP cooperative perception model integration with feature
extraction for intermediate fusion.
"""
from __future__ import annotations
import pathlib
import yaml
import torch
import numpy as np
from typing import Dict, Any, Optional

from opencood.hypes_yaml.yaml_utils import load_yaml
import opencood.tools.train_utils as train_utils
from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor
from ecav.core.sensing.perception.perception_manager import PerceptionManager
import ecav.core.sensing.perception.sensor_transformation as st

import logging
logger = logging.getLogger(__name__)

def print_tensor_dict(d, indent=0):
    for key, value in d.items():
        if isinstance(value, torch.Tensor):
            logger.debug(' ' * indent + f"'{key}': Tensor(shape={value.shape}, dtype={value.dtype}, device={value.device})")
        elif isinstance(value, dict):
            logger.debug(' ' * indent + f"'{key}': {{")
            print_tensor_dict(value, indent + 2)
            logger.debug(' ' * indent + "}")
        else:
            logger.debug(' ' * indent + f"'{key}': {type(value)}")

class BM2CPPerceptionManager(PerceptionManager):
    def __init__(self,
                 vehicle,
                 config_yaml: Dict[str, Any],
                 cav_world=None,
                 data_dump: bool = False,
                 carla_world=None,
                 infra_id: Optional[int] = None,
                 tracking_manager=None,
                 debug_helper=None,
                 device: str = "cuda:0",
                 **kw):
        
        super().__init__(vehicle=vehicle,
                         config_yaml=config_yaml,
                         cav_world=cav_world,
                         data_dump=data_dump,
                         carla_world=carla_world,
                         infra_id=infra_id,
                         tracking_manager=tracking_manager,
                         debug_helper=debug_helper)
        self.device = device

        logger.debug("\nInitialising Perception Manager...")
        model_config = config_yaml['bm2cp_model']
        hypes_path = pathlib.Path(model_config['hypes_yaml'])
        ckpt_path = pathlib.Path(model_config['checkpoint'])

        self.hypes = load_yaml(str(hypes_path))
        logger.debug(f"ybound = {self.hypes['fusion']['args']['grid_conf']['ybound']}")
        logger.debug("Hypes loaded and parsed successfully.")
        
        pre_config = self.hypes["preprocess"]
        self._vp = SpVoxelPreprocessor(pre_config, train=False)
        logger.debug("SpVoxelPreprocessor initialized.")
        
        from opencood.models.point_pillar_bm2cp import PointPillarBM2CP
        self.model = PointPillarBM2CP(self.hypes['model']['args']).to(self.device).eval()
        logger.debug("Model created successfully.")

        epoch, self.model = train_utils.load_model(str(ckpt_path.parent), self.model, epoch=int(str(ckpt_path.name).split('epoch')[-1].split('.')[0]))
        logger.debug(f"Loaded model weights from epoch {epoch}.")

        self.feature_dict = None
        self.feature_map = None
        self._first_run = True
        logger.debug("Init complete.\n")

    def detect(self, ego_pos, **kw):
        lidar_ready = self.lidar and self.lidar.data is not None
        cams_ready = self.rgb_camera and all(c.image is not None for c in self.rgb_camera)
        
        if lidar_ready and cams_ready:
            self.run_step()
        return {"vehicles": [], "traffic_lights": [], "static": []}

    @torch.inference_mode()
    def run_step(self):
        logger.debug("\n>>> run_step() called.")
        batch = self._build_batch()
        logger.debug("Batch construction complete.")

        logger.debug("\nBATCH MANIFEST (before moving to device):")
        print_tensor_dict(batch)
        
        batch = train_utils.to_device(batch, self.device)
        logger.debug(f"\nBatch moved to {self.device}.")

        logger.debug("CALLING MODEL.GET_FEATURE() ====================")
        feature_dict_gpu = self.model.get_feature(batch)
        logger.debug("MODEL.GET_FEATURE() CALL COMPLETE ====================\n")
        
        # Store the dictionary of features
        self.feature_dict = {k: v.cpu() for k, v in feature_dict_gpu.items()}
        self.feature_map = self.feature_dict['spatial_features'].half()
        logger.debug("\nFeature dictionary generated successfully.")
        logger.debug("\nFEATURE DICTIONARY:")
        print_tensor_dict(self.feature_dict)
        logger.debug(f"\nFEATURE MAP SHAPE: {self.feature_map.shape}")
        logger.debug(f"\nFEATURE MAP DATA TYPE: {self.feature_map.dtype}")
        logger.debug(f"\nFEATURE MAP DEVICE: {self.feature_map.device}")
        
        if self._first_run:
            logger.debug(f"\n>>> SUCCESS: ON-VEHICLE FUSION COMPLETE. FEATURE DICTIONARY GENERATED. <<<\n")
            self._first_run = False

    def _build_batch(self) -> Dict[str, Any]:
        logger.debug("[_build_batch] Preparing LiDAR data...")
        proc_lidar_np = self._vp.preprocess(np.ascontiguousarray(self.lidar.data, dtype=np.float32))
        proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}
        coords = proc_lidar['voxel_coords']
        batch_index_column = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
        proc_lidar['voxel_coords'] = torch.cat((batch_index_column, coords), dim=1).int()
        logger.debug("[_build_batch] LiDAR processing complete.")

        logger.debug("[_build_batch] Preparing Camera data...")

        # Get target dimensions from model config (data_aug_conf.final_dim)
        data_aug_conf = self.hypes.get('fusion', {}).get('args', {}).get('data_aug_conf', {})
        final_dim = data_aug_conf.get('final_dim', [360, 480])  # [H, W]
        target_h, target_w = final_dim[0], final_dim[1]

        imgs, rots, trans, intrins = [], [], [], []
        for cam in self.rgb_camera:
            img_bgr_to_rgb = cam.image[..., ::-1].copy()
            orig_h, orig_w = img_bgr_to_rgb.shape[:2]

            # Resize image to match model's expected dimensions
            img_tensor = torch.from_numpy(img_bgr_to_rgb).permute(2, 0, 1).float()  # [C, H, W]
            img_resized = torch.nn.functional.interpolate(
                img_tensor.unsqueeze(0),  # [1, C, H, W]
                size=(target_h, target_w),
                mode='bilinear',
                align_corners=False
            ).squeeze(0)  # [C, H, W]
            imgs.append(img_resized)

            cam_to_lidar = np.array(self.lidar.sensor.get_transform().get_inverse_matrix()) @ np.array(cam.sensor.get_transform().get_matrix())
            rots.append(torch.from_numpy(cam_to_lidar[:3, :3].copy()).float())
            trans.append(torch.from_numpy(cam_to_lidar[:3, 3].copy()).float())

            # Scale intrinsics to match resized image
            K = st.get_camera_intrinsic(cam.sensor).copy()
            K[0, :] *= target_w / orig_w  # Scale fx, cx
            K[1, :] *= target_h / orig_h  # Scale fy, cy
            intrins.append(torch.from_numpy(K).float())

        B, N_cams = 1, len(self.rgb_camera)
        placeholder_depth = torch.zeros(B, N_cams, target_h, target_w)

        image_inputs = {
            "imgs": torch.stack(imgs).view(B, N_cams, *imgs[0].shape),
            "rots": torch.stack(rots).view(B, N_cams, 3, 3),
            "trans": torch.stack(trans).view(B, N_cams, 3),
            "intrins": torch.stack(intrins).view(B, N_cams, 3, 3),
            "post_rots": torch.eye(3, dtype=torch.float32).view(1, 1, 3, 3).expand(B, N_cams, 3, 3),
            "post_trans": torch.zeros(3, dtype=torch.float32).view(1, 1, 3).expand(B, N_cams, 3),
            "depth_map": placeholder_depth
        }
        logger.debug(f"[_build_batch] Camera processing complete. Resized to {target_h}x{target_w}")
        
        return {"processed_lidar": proc_lidar, "image_inputs": image_inputs, "record_len": torch.tensor([N_cams], dtype=torch.int64)}

