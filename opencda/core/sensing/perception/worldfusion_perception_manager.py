# -*- coding: utf-8 -*-
"""
worldfusion_perception_manager.py
Author: Tyler Landle <tlandle3@gatech.edu>
Description: Perception manager for WorldFusion model integration in eCAV.
             Extracts intermediate spatial_features (before backbone) for
             transmission to edge where Where2comm fusion is performed.
TDG non-commercial use license.
"""
from __future__ import annotations
import pathlib
import torch
import numpy as np
from typing import Dict, Any, Optional

from opencood.hypes_yaml.yaml_utils import load_yaml
import opencood.tools.train_utils as train_utils
from opencood.data_utils.pre_processor.sp_voxel_preprocessor import SpVoxelPreprocessor
from opencda.core.sensing.perception.perception_manager import PerceptionManager
import opencda.core.sensing.perception.sensor_transformation as st


class WorldFusionPerceptionManager(PerceptionManager):
    """
    Perception manager for WorldFusion edge pipeline.

    This manager extracts intermediate BEV features (spatial_features) from the
    WorldFusion sensor encoder BEFORE the backbone. These features are transmitted
    to the edge where Where2comm fusion and detection heads are run.

    Key differences from BM2CPPerceptionManager:
    - Uses PointPillarWorldFusion model (specifically the _SensorEncoder)
    - Only extracts and stores spatial_features (not psm/rm/thres_map)
    - Features are extracted before backbone (true intermediate fusion)
    """

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

        print("\n[WorldFusion] Initialising Perception Manager...")
        model_config = config_yaml['worldfusion_model']
        hypes_path = pathlib.Path(model_config['hypes_yaml'])
        ckpt_path = pathlib.Path(model_config['checkpoint'])

        self.hypes = load_yaml(str(hypes_path))
        print("[WorldFusion] Hypes loaded and parsed successfully.")

        pre_config = self.hypes["preprocess"]
        self._vp = SpVoxelPreprocessor(pre_config, train=False)
        print("[WorldFusion] SpVoxelPreprocessor initialized.")

        # Load the WorldFusion model (we'll use just the sensor encoder for feature extraction)
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        self.model = PointPillarWorldFusion(self.hypes['model']['args']).to(self.device).eval()
        print("[WorldFusion] Model created successfully.")

        epoch, self.model = train_utils.load_model(
            str(ckpt_path.parent),
            self.model,
            epoch=int(str(ckpt_path.name).split('epoch')[-1].split('.')[0])
        )
        print(f"[WorldFusion] Loaded model weights from epoch {epoch}.")

        # Feature dictionary - only stores spatial_features for edge transmission
        self.feature_dict = None
        self.feature_map = None
        self._first_run = True
        print("[WorldFusion] Init complete.\n")

    def detect(self, ego_pos, **kw):
        """
        Run detection step - extracts features for edge transmission.

        Note: Actual detection is performed on the edge, not here.
        This method just ensures features are extracted when sensors are ready.
        """
        lidar_ready = self.lidar and self.lidar.data is not None
        cams_ready = self.rgb_camera and all(c.image is not None for c in self.rgb_camera)

        if lidar_ready and cams_ready:
            self.run_step()
        return {"vehicles": [], "traffic_lights": [], "static": []}

    @torch.inference_mode()
    def run_step(self):
        """
        Extract intermediate spatial_features for edge transmission.

        This runs the sensor encoder up to (but not including) the backbone,
        extracting the fused multimodal BEV features.
        """
        # DEBUG: Print raw LiDAR statistics to check coordinate system
        lidar_data = self.lidar.data
        agent_type = "RSU" if self.vehicle is None else "Vehicle"

        # Quick periodic check for Lincoln's position (every 20 frames for RSU only)
        if agent_type == "RSU" and lidar_data is not None:
            frame_num = getattr(self, '_debug_frame', 0)
            self._debug_frame = frame_num + 1
            if frame_num % 20 == 0:
                try:
                    sensor_tf = self.lidar.sensor.get_transform()
                    for actor in self.carla_world.get_actors():
                        if 'lincoln' in actor.type_id.lower():
                            loc = actor.get_location()
                            # Compute Lincoln's LOCAL position
                            dx = loc.x - sensor_tf.location.x
                            dy = loc.y - sensor_tf.location.y
                            yaw_rad = np.radians(sensor_tf.rotation.yaw)
                            local_x = dx * np.cos(yaw_rad) + dy * np.sin(yaw_rad)
                            local_y = -dx * np.sin(yaw_rad) + dy * np.cos(yaw_rad)
                            # Count LiDAR points near Lincoln
                            dist = np.sqrt((lidar_data[:, 0] - local_x)**2 + (lidar_data[:, 1] - local_y)**2)
                            pts_near = (dist < 5.0).sum()
                            print(f"[LINCOLN CHECK] Frame {frame_num}: world=({loc.x:.1f},{loc.y:.1f},z={loc.z:.1f}), local=({local_x:.1f},{local_y:.1f}), LiDAR pts within 5m: {pts_near}")
                            break
                except Exception as e:
                    print(f"[LINCOLN CHECK] Frame {frame_num}: Error - {e}")

        if lidar_data is not None and self._first_run:
            # Print actual sensor world position
            sensor_tf = self.lidar.sensor.get_transform()
            print(f"\n[WorldFusion LIDAR DEBUG] {agent_type} id={self.id}")
            print(f"[WorldFusion LIDAR DEBUG] LiDAR SENSOR world pos: x={sensor_tf.location.x:.2f}, y={sensor_tf.location.y:.2f}, z={sensor_tf.location.z:.2f}")
            print(f"[WorldFusion LIDAR DEBUG] LiDAR SENSOR world rot: yaw={sensor_tf.rotation.yaw:.2f}")
            print(f"[WorldFusion LIDAR DEBUG] global_position from config: {self.global_position}")
            print(f"[WorldFusion LIDAR DEBUG] Raw LiDAR shape: {lidar_data.shape}")
            print(f"[WorldFusion LIDAR DEBUG] X range: [{lidar_data[:, 0].min():.1f}, {lidar_data[:, 0].max():.1f}]")
            print(f"[WorldFusion LIDAR DEBUG] Y range: [{lidar_data[:, 1].min():.1f}, {lidar_data[:, 1].max():.1f}]")
            print(f"[WorldFusion LIDAR DEBUG] Z range: [{lidar_data[:, 2].min():.1f}, {lidar_data[:, 2].max():.1f}]")

            # For RSU: compute Lincoln's expected local position dynamically
            # First, find Lincoln's ACTUAL position from CARLA actors
            lincoln_world_x, lincoln_world_y = -42.4, 127.7  # default from scenario
            try:
                actors = self.carla_world.get_actors()
                for actor in actors:
                    if 'lincoln' in actor.type_id.lower() or 'mkz' in actor.type_id.lower():
                        loc = actor.get_location()
                        lincoln_world_x, lincoln_world_y = loc.x, loc.y
                        print(f"[WorldFusion LIDAR DEBUG] Found Lincoln actor at ACTUAL world pos: ({loc.x:.2f}, {loc.y:.2f}, {loc.z:.2f})")
                        break
            except Exception as e:
                print(f"[WorldFusion LIDAR DEBUG] Could not get Lincoln actor: {e}")
            rsu_x, rsu_y = sensor_tf.location.x, sensor_tf.location.y
            rsu_yaw_rad = np.radians(sensor_tf.rotation.yaw)

            # Transform Lincoln's world position to sensor-local frame
            dx = lincoln_world_x - rsu_x
            dy = lincoln_world_y - rsu_y
            # Rotation: local = R^T @ world_delta (inverse rotation)
            cos_yaw, sin_yaw = np.cos(rsu_yaw_rad), np.sin(rsu_yaw_rad)
            lincoln_local_x = dx * cos_yaw + dy * sin_yaw
            lincoln_local_y = -dx * sin_yaw + dy * cos_yaw

            print(f"[WorldFusion LIDAR DEBUG] Lincoln world: ({lincoln_world_x}, {lincoln_world_y})")
            print(f"[WorldFusion LIDAR DEBUG] Sensor world: ({rsu_x:.2f}, {rsu_y:.2f}), yaw={sensor_tf.rotation.yaw:.1f}°")
            print(f"[WorldFusion LIDAR DEBUG] Expected Lincoln LOCAL: ({lincoln_local_x:.2f}, {lincoln_local_y:.2f})")

            # Check if there are points near Lincoln's expected position (within 5m)
            dist_to_lincoln = np.sqrt((lidar_data[:, 0] - lincoln_local_x)**2 + (lidar_data[:, 1] - lincoln_local_y)**2)
            nearby_points = lidar_data[dist_to_lincoln < 5.0]
            print(f"[WorldFusion LIDAR DEBUG] Points within 5m of expected Lincoln LOCAL ({lincoln_local_x:.1f}, {lincoln_local_y:.1f}): {len(nearby_points)}")
            if len(nearby_points) > 0:
                print(f"[WorldFusion LIDAR DEBUG]   Nearby points X: [{nearby_points[:, 0].min():.1f}, {nearby_points[:, 0].max():.1f}]")
                print(f"[WorldFusion LIDAR DEBUG]   Nearby points Y: [{nearby_points[:, 1].min():.1f}, {nearby_points[:, 1].max():.1f}]")

            # Print the cluster of points that are likely Lincoln (looking for vehicle-sized cluster around X=20-25)
            vehicle_region = lidar_data[(lidar_data[:, 0] > 15) & (lidar_data[:, 0] < 30) &
                                        (lidar_data[:, 1] > -5) & (lidar_data[:, 1] < 35)]
            if len(vehicle_region) > 0:
                print(f"[WorldFusion LIDAR DEBUG] Points in X=[15,30], Y=[-5,35]: {len(vehicle_region)}")
                print(f"[WorldFusion LIDAR DEBUG]   These points X: [{vehicle_region[:, 0].min():.1f}, {vehicle_region[:, 0].max():.1f}]")
                print(f"[WorldFusion LIDAR DEBUG]   These points Y: [{vehicle_region[:, 1].min():.1f}, {vehicle_region[:, 1].max():.1f}]")

        batch = self._build_batch()
        batch = train_utils.to_device(batch, self.device)

        # Run the sensor encoder to get intermediate features
        # We need to call the internal _SensorEncoder up to the point before backbone
        feature_dict_gpu = self._extract_intermediate_features(batch)

        # Store only spatial_features for transmission (keep on CPU for transfer)
        self.feature_dict = {
            'spatial_features': feature_dict_gpu['spatial_features'].cpu()
        }
        self.feature_map = self.feature_dict['spatial_features'].half()

        if self._first_run:
            print(f"\n[WorldFusion] >>> SUCCESS: FEATURE EXTRACTION COMPLETE. <<<")
            print(f"[WorldFusion] spatial_features shape: {self.feature_map.shape}")
            self._first_run = False

    def _extract_intermediate_features(self, batch: Dict) -> Dict:
        """
        Extract intermediate features from sensor encoder (before backbone).

        This replicates the first part of _SensorEncoder.forward() up to the
        multimodal fusion, but stops before running the backbone.
        """
        sensor = self.model.sensor

        # LiDAR branch
        pc = batch['processed_lidar']
        rec_len = batch['record_len']
        bd = {
            'voxel_features': pc['voxel_features'],
            'voxel_coords': pc['voxel_coords'],
            'voxel_num_points': pc['voxel_num_points'],
            'record_len': rec_len,
        }
        bd = sensor.scatter(sensor.pillar_vfe(bd))

        # Camera branch (if image_inputs available)
        if 'image_inputs' in batch and batch['image_inputs'] is not None:
            from einops import rearrange
            imgs = batch['image_inputs']['imgs']
            B, N, C, imH, imW = imgs.shape
            x = imgs.view(B * N, C, imH, imW)
            _, x = sensor.camenc(x, batch['image_inputs']['depth_map'], batch['record_len'])
            x = rearrange(x, '(b n) c d h w -> b n c d h w', b=B, n=N)
            x = x.permute(0, 1, 3, 4, 5, 2)  # [B,N,D,fH,fW,C]
            geom = sensor._get_geometry(batch['image_inputs'])  # [B,N,D,fH,fW,3]
            img_voxel = sensor.voxel_pooling(geom, x)

            # Fuse with LiDAR voxels
            bd, thres_map, mask, each_mask = sensor.vox_fuse(img_voxel, bd)
        else:
            # LiDAR-only mode: collapse Z dimension manually
            spatial_3d = bd['spatial_features_3d']
            b, c, z, y, x = spatial_3d.shape
            bd['spatial_features'] = spatial_3d.view(b, c * z, y, x)

        # Return intermediate features (before backbone)
        # spatial_features shape: [B, C*Z, Y, X] where typically [1, 64*4, 192, 704]
        return {
            'spatial_features': bd['spatial_features']
        }

    def _build_batch(self) -> Dict[str, Any]:
        """
        Build input batch from sensor data.
        Same as BM2CPPerceptionManager._build_batch().
        """
        # LiDAR processing
        proc_lidar_np = self._vp.preprocess(
            np.ascontiguousarray(self.lidar.data, dtype=np.float32)
        )
        proc_lidar = {k: torch.from_numpy(v) for k, v in proc_lidar_np.items()}
        coords = proc_lidar['voxel_coords']

        # DEBUG: Print voxel coordinate statistics
        if self._first_run:
            # Voxel coords are in [z, y, x] format for spconv
            voxel_x = coords[:, 2].numpy()
            voxel_y = coords[:, 1].numpy()
            voxel_z = coords[:, 0].numpy()
            print(f"[WorldFusion VOXEL DEBUG] Voxel coords shape: {coords.shape}")
            print(f"[WorldFusion VOXEL DEBUG] Voxel X (grid): [{voxel_x.min()}, {voxel_x.max()}]")
            print(f"[WorldFusion VOXEL DEBUG] Voxel Y (grid): [{voxel_y.min()}, {voxel_y.max()}]")
            print(f"[WorldFusion VOXEL DEBUG] Voxel Z (grid): [{voxel_z.min()}, {voxel_z.max()}]")

            # Expected Lincoln position in voxel space (voxel_size=0.4, range=[-40,40])
            # local (21.2, 13.7) -> voxel = (coord + 40) / 0.4
            lincoln_vx = int((21.2 + 40) / 0.4)  # = 153
            lincoln_vy = int((13.7 + 40) / 0.4)  # = 134
            print(f"[WorldFusion VOXEL DEBUG] Expected Lincoln voxel: ({lincoln_vx}, {lincoln_vy})")

            # Check if voxels near Lincoln's expected position exist
            nearby_mask = (np.abs(voxel_x - lincoln_vx) < 10) & (np.abs(voxel_y - lincoln_vy) < 10)
            nearby_count = nearby_mask.sum()
            print(f"[WorldFusion VOXEL DEBUG] Voxels near expected Lincoln: {nearby_count}")

        batch_index_column = torch.zeros(coords.shape[0], 1, dtype=torch.int32)
        proc_lidar['voxel_coords'] = torch.cat((batch_index_column, coords), dim=1).int()

        # Get target dimensions from model config (data_aug_conf.final_dim)
        data_aug_conf = self.hypes.get('fusion', {}).get('args', {}).get('data_aug_conf', {})
        final_dim = data_aug_conf.get('final_dim', [360, 480])  # [H, W]
        target_h, target_w = final_dim[0], final_dim[1]

        # Camera processing
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

            cam_to_lidar = (
                np.array(self.lidar.sensor.get_transform().get_inverse_matrix()) @
                np.array(cam.sensor.get_transform().get_matrix())
            )
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

        # record_len represents number of agents in this batch, not number of cameras
        # For a single vehicle/RSU extracting features, this is always 1
        return {
            "processed_lidar": proc_lidar,
            "image_inputs": image_inputs,
            "record_len": torch.tensor([1], dtype=torch.int64)
        }
