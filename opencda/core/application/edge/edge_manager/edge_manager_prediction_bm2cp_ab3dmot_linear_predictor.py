# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
BM2CP edge manager with AB3DMOT tracking and linear prediction.

Runs the BM2CP cooperative perception backbone on fused vehicle features,
followed by AB3DMOT 3D multi-object tracking and linear constant-velocity
prediction.
"""
from __future__ import annotations
import os, time, random
from collections import deque
from typing import Dict, List, Deque, Tuple, Any

import numpy as np
import carla
import torch
from easydict import EasyDict as edict
import matplotlib.pyplot as plt
import open3d as o3d

from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.data_utils.post_processor import VoxelPostprocessor
from opencood.utils import box_utils, transformation_utils
from opencood.tools import train_utils
from opencood.models.common_modules.torch_transformation_utils import warp_affine_simple
import torch.nn.functional as F
from AB3DMOT_libs.model import AB3DMOT

from opencda.core.prediction.linear_predictor_manager import LinearPredictorManager
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.application.edge.edge_profiler import EdgeProfiler
from .edge_manager_base import _BaseEdgeManager, logger


def _track_to_corners_and_bbx(track: np.ndarray):
    """
    Convert AB3DMOT track [h, w, l, x, y, z, theta, ...] to 8 corners and o3d bounding box.

    Returns:
        corners: np.ndarray of shape (8, 3) - 8 corners of the 3D bounding box
        o3d_bbx: open3d.geometry.OrientedBoundingBox
    """
    h, w, l, x, y, z, theta = track[:7]

    # Create 8 corners of the bounding box (before rotation)
    # Order: front-left-bottom, front-right-bottom, back-right-bottom, back-left-bottom,
    #        front-left-top, front-right-top, back-right-top, back-left-top
    dx, dy, dz = l / 2, w / 2, h / 2
    corners_local = np.array([
        [ dx,  dy, -dz],  # front-left-bottom
        [ dx, -dy, -dz],  # front-right-bottom
        [-dx, -dy, -dz],  # back-right-bottom
        [-dx,  dy, -dz],  # back-left-bottom
        [ dx,  dy,  dz],  # front-left-top
        [ dx, -dy,  dz],  # front-right-top
        [-dx, -dy,  dz],  # back-right-top
        [-dx,  dy,  dz],  # back-left-top
    ])

    # Rotation matrix around z-axis
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    R = np.array([
        [cos_t, -sin_t, 0],
        [sin_t,  cos_t, 0],
        [0,      0,     1]
    ])

    # Rotate and translate corners
    corners = (R @ corners_local.T).T + np.array([x, y, z])

    # Create Open3D oriented bounding box
    center = np.array([x, y, z])
    extent = np.array([l, w, h])  # full dimensions
    R_matrix = R
    o3d_bbx = o3d.geometry.OrientedBoundingBox(center, R_matrix, extent)

    return corners, o3d_bbx


def _box_to_transform(box: np.ndarray):
    """Helper to convert a single detection box to a picklable Transform."""
    from opencda.opencda_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    loc = _Loc(x=float(box[3]), y=float(box[4]), z=float(box[5]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)

class BM2CPEdge(_BaseEdgeManager):
    def __init__(self,
                 world: carla.World,
                 cfg: Dict[str, Any],
                 cav_world,
                 carla_client: carla.Client,
                 *,
                 world_dt: float = 0.05,
                 **kwargs):
        super().__init__(world, cfg, cav_world, carla_client, world_dt=world_dt, **kwargs)
        logger.debug(f"[__init__] START")

        bm_cfg = cfg['bm2cp_model']
        hypes = load_yaml(bm_cfg['hypes_yaml'])
        self.hypes = hypes
        logger.debug(f"ybound = {hypes['fusion']['args']['grid_conf']['ybound']}")
        logger.debug(f"[__init__] Hypes loaded.")

        # Get ML manager from cav_world
        self.ml_manager = cav_world.ml_manager if cav_world else None

        if not self.run_distributed:
            # Local mode - model already loaded in ml_manager
            logger.debug(f"Using local BM2CP model")
            bm_cfg = cfg['bm2cp_model']
            hypes = load_yaml(bm_cfg['hypes_yaml'])
            self.hypes = hypes
            logger.debug(f"ybound = {hypes['fusion']['args']['grid_conf']['ybound']}")
            
            # Create the post-processor instance using the loaded config
            self.post_processor = VoxelPostprocessor(self.hypes['postprocess'],
                                                     dataset=None,
                                                     train=False)
        else:
            logger.debug(f"Using distributed BM2CP services")
            # In distributed mode, post-processing happens on the edge service
            self.post_processor = None

        self.model = train_utils.create_model(hypes).cuda().eval()
        ckpt_dir = os.path.dirname(bm_cfg['checkpoint'])
        epoch_id = int(bm_cfg['checkpoint'].split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(ckpt_dir, self.model, epoch_id)
        logger.debug(f"[__init__] Model loaded from epoch {epoch_id}.")

        # Create the post-processor instance using the loaded config
        self.post_processor = VoxelPostprocessor(self.hypes['postprocess'],
                                                 dataset=None, # Not needed for inference
                                                 train=False)

        self.ab3dmot_config = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': False,
            'thres': 2.0, 'output_dir': None, 'min_hits': 1,  # Reduced from 3 to 1 for faster track confirmation
            'max_age': 10, 'ego_com': None, 'affi_pro': False,  # Increased from 2 to 10 for track persistence
            'dataset': "KITTI", 'det_name': "deprecated"
        })
        self.ab3dmot_category = 'Car'

        # Create persistent tracker instance (reused across frames)
        self.tracker = AB3DMOT(self.ab3dmot_config, self.ab3dmot_category)

        self.lin_pred = LinearPredictorManager(num_future_steps=25)
        self.tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self.feat_history: Deque[Tuple[int, List[Dict], List[carla.Transform], Dict]] = deque(maxlen=200)

        # GT snapshot history for metrics evaluation
        self.carla_snapshot_history: Deque[Dict] = deque(maxlen=100)

        # Resource profiler for capacity planning
        intersection_id = cfg.get('intersection_id', f"bm2cp_edge_{id(self)}")
        self.profiler = EdgeProfiler(
            intersection_id=intersection_id,
            history_size=2000,
            sample_gpu_utilization=True
        )

        logger.debug(f"[__init__] COMPLETE")

    def start_edge(self):
        logger.debug(f"[start_edge] Called.")
        pass

    def update_information(self, frame_idx: int):
        logger.debug(f"[update_information] Called for frame_idx: {frame_idx}")
        feature_dicts, poses = [], []

        # Capture CARLA ground truth snapshot for evaluation
        # Filter to vehicles within detection range of ego vehicle
        # BM2CP uses ego-centric detection with cav_lidar_range from hypes
        DETECTION_RANGE = 70.0  # meters - based on typical cav_lidar_range
        carla_snapshot = {}

        # Get managed vehicle positions for range filtering
        managed_locs = [vm.vehicle.get_location() for vm in self.vehicle_manager_list]

        try:
            actors = self.world.get_actors()
            for actor in actors:
                if 'vehicle' in actor.type_id.lower():
                    loc = actor.get_location()

                    # Filter out underground/invalid vehicles
                    if loc.z < -10.0:
                        continue

                    # Filter by detection range from any managed vehicle
                    in_range = False
                    for mloc in managed_locs:
                        dist = np.sqrt((loc.x - mloc.x)**2 + (loc.y - mloc.y)**2)
                        if dist <= DETECTION_RANGE:
                            in_range = True
                            break

                    if not in_range:
                        continue

                    rot = actor.get_transform().rotation
                    vel = actor.get_velocity()
                    carla_snapshot[actor.id] = {
                        'type': actor.type_id.split('.')[-1],
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'yaw': rot.yaw,
                        'vx': vel.x, 'vy': vel.y,
                        'speed': np.sqrt(vel.x**2 + vel.y**2)
                    }
        except Exception as e:
            logger.debug(f"Could not capture CARLA snapshot: {e}")
        self.carla_snapshot_history.appendleft(carla_snapshot)

        for i, vm in enumerate(self.vehicle_manager_list):
            pm = vm.perception_manager
            logger.debug(f"[update_information] Processing vehicle {i}...")
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                logger.debug(f"[update_information] Got feature dictionary from vehicle {i}")
                feature_dicts.append(pm.feature_dict)
                poses.append(vm.localizer.get_ego_pos())
            else:
                logger.debug(f"[update_information] No feature_dict found for vehicle {i}")

        for i, rsu in enumerate(self.rsu_manager_list):
            pm = rsu.perception_manager
            logger.debug(f"[update_information] Processing RSU {i}...")
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                logger.debug(f"[update_information] Got feature dictionary from RSU {i}")
                feature_dicts.append(pm.feature_dict)
                poses.append(rsu.localizer.get_ego_pos())
            else:
                logger.debug(f"[update_information] No feature_dict found for RSU {i}")

        if feature_dicts:
            self.feat_history.appendleft((frame_idx, feature_dicts, poses, carla_snapshot))
            logger.debug(f"[update_information] Stored {len(feature_dicts)} feature dictionaries for frame {frame_idx}.")
        else:
            logger.debug(f"[update_information] No features available for frame {frame_idx}.")

    def _world_to_ego(self, world_xyz1: np.ndarray, ego_tf: carla.Transform) -> np.ndarray:
        """world_xyz1: [x,y,z,1] homogeneous. Return xyz in ego frame."""
        T_ego_inv = np.linalg.inv(np.array(ego_tf.get_matrix()))
        return (T_ego_inv @ world_xyz1)[:3]

    def _ego_to_voxel(self, p_ego: np.ndarray) -> Tuple[float, float]:
        """Return (ix, iy) fractional voxel indices for BEV tensors."""
        # grab the SAME ranges actually used by runtime model
        vx = self.hypes['model']['args']['pc_params']['voxel_size'][0]  # 0.4
        vy = self.hypes['model']['args']['pc_params']['voxel_size'][1]  # 0.4
        x_min, y_min, _, x_max, y_max, _ = self.hypes['preprocess']['cav_lidar_range']
        W = self.hypes['postprocess']['anchor_args']['W']               # 704
        H = self.hypes['postprocess']['anchor_args']['H']               # 192

        # OpenCOOD convention: +Y left. CARLA: +Y right. Flip Y here:
        y_ego_lidar = -p_ego[1]
        ix = (p_ego[0] - x_min) / vx
        iy = (y_ego_lidar - y_min) / vy
        return ix, iy

    def print_other_car(self, ego_actor: carla.Actor, ego_tf: carla.Transform, target_id=None):
        """
        Print GT vehicle pose in world, ego and voxel coordinates.
        ego_actor: the ego actor to exclude.
        ego_tf: ego Transform (same as ego_actor.get_transform()).
        """
        world = ego_actor.get_world()
        all_vehicles = world.get_actors().filter('vehicle.*')

        if target_id is not None:
            target = world.get_actor(target_id)
        else:
            # choose closest non-ego by 2D distance
            non_ego = [a for a in all_vehicles if a.id != ego_actor.id]
            if not non_ego:
                return None
            ego_xy = np.array([ego_tf.location.x, ego_tf.location.y])
            target = min(non_ego, key=lambda a: np.linalg.norm(
                np.array([a.get_transform().location.x, a.get_transform().location.y]) - ego_xy))

        tf = target.get_transform()
        p_world = np.array([tf.location.x, tf.location.y, tf.location.z, 1.0])
        p_ego   = self._world_to_ego(p_world, ego_tf)
        ix, iy  = self._ego_to_voxel(p_ego)

        logger.debug(f"id={target.id} world=({tf.location.x:.2f},{tf.location.y:.2f}) "
              f"ego=({p_ego[0]:.2f},{p_ego[1]:.2f}) yaw={tf.rotation.yaw:.1f}")
        logger.debug(f"voxel ix={ix:.1f}/{self.hypes['postprocess']['anchor_args']['W']}, "
              f"iy={iy:.1f}/{self.hypes['postprocess']['anchor_args']['H']}")

        # Optional: warn if out-of-crop
        if not (0 <= ix < self.hypes['postprocess']['anchor_args']['W'] and 
                0 <= iy < self.hypes['postprocess']['anchor_args']['H']):
            logger.warning(f"GT target outside current BEV crop.")

        return {'id': target.id, 'world': tf, 'ego_xyz': p_ego, 'ixiy': (ix, iy)}

    def run_step(self, tick: int):
        logger.debug("\n>>> run_step() called.")
        
        lat_ms = self._sample_latency_ms()
        lag_steps = int(round(lat_ms / (self.dt * 1000)))
        target_id = tick - lag_steps
        logger.debug(f"Current Tick: {tick}, Latency: {lat_ms:.2f}ms, Target Frame ID: {target_id}")

        snapshot = next((item for item in self.feat_history if item[0] == target_id), None)
        
        if not snapshot or not snapshot[0]:
            logger.debug(f"No snapshot found for target_id {target_id}. Skipping.")
            self._update_agents_and_advance_sim(tick, None)
            return

        frame_id, feature_dicts, poses = snapshot
        ego_pose_for_fusion = poses[0] 
        #logger.debug(f"Found snapshot for frame {frame_id} with {len(feature_dicts)} feature dictionaries.")
        
        start_time = time.time()
        #logger.debug(f"Preparing for fusion...")
        features_tensor = torch.cat([d['spatial_features'] for d in feature_dicts], dim=0).cuda()
        psm_tensor = torch.cat([d['psm'] for d in feature_dicts], dim=0).cuda()
        rm_tensor = torch.cat([d['rm'] for d in feature_dicts], dim=0).cuda()
        thres_map_tensor = torch.cat([d['thres_map'] for d in feature_dicts], dim=0).cuda()
        record_len = torch.tensor([len(feature_dicts)], dtype=torch.int64).cuda()
        pairwise_t_matrix = self._get_pairwise_transform(poses)

        if thres_map_tensor.size(0) > 1:          # make sure we really have an RSU slicve
            thres_map_tensor[1] *= 0.03           # 0.3 ≈ drop the threshold by 70 %



        # inside run_step, after you compute pairwise_t_matrix
        obj_world     = np.array([-42.4, 127.7, 0 , 1])          # from XML
        P_rsu2ego     = pairwise_t_matrix[0, 0, 1].cpu().numpy() # RSU→ego 4×4
        obj_ego       = P_rsu2ego @ obj_world
        #logger.debug(f"Object in RSU frame: {obj_world[:3]}, Object in Ego frame: {obj_ego[:3]}")  # expected y ≈ −117  (> 40 so out of range)
        #logger.debug(f"Features tensor shape: {features_tensor.shape}, RM tensor shape: {rm_tensor.shape}, Threshold map tensor shape: {thres_map_tensor.shape}")
        #logger.debug(f"Record length: {record_len.item()}, Pairwise transformation matrix shape: {pairwise_t_matrix.shape}")
        #logger.debug(f"Ego pose for fusion: {ego_pose_for_fusion.location.x}, {ego_pose_for_fusion.location.y}, {ego_pose_for_fusion.rotation.yaw}")
        #logger.debug(f"Distance to objct in Ego frame: {np.linalg.norm(obj_ego[:3])}")
        #other_car = self.print_other_car(self.world, ego_pose_for_fusion)
        #vx = self.hypes['model']['args']['pc_params']['voxel_size'][0]  # 0.4
        #vy = self.hypes['model']['args']['pc_params']['voxel_size'][1]
        #x_min, y_min, _, x_max, y_max, _ = self.hypes['preprocess']['cav_lidar_range']
        #W = self.hypes['postprocess']['anchor_args']['W']
        #H = self.hypes['postprocess']['anchor_args']['H']
        gt = self.print_other_car(self.vehicle_manager_list[0].vehicle, ego_pose_for_fusion)


        # Example: want the RSU car to be inside [y_min, y_max]
        #obj_ego = obj_ego[:3]
        #iy = (obj_ego[1] - y_min) / vy   # voxel index in ego BEV Y axis
        #ix = (obj_ego[0] - x_min) / vx

        #print(f"[DEBUG] ix={ix:.1f}/{W}, iy={iy:.1f}/{H}")
        #logger.debug(f"Pairwise transformation matrix:")
        #for i in range(pairwise_t_matrix.shape[1]):
        #    for j in range(pairwise_t_matrix.shape[2]):
        #        logger.debug(f"Transformation from agent {i} to agent {j}:")
        #        print(pairwise_t_matrix[0, i, j].cpu().numpy())

        logger.debug(f"Feature maps shape: {features_tensor.shape}")
        for i in range(features_tensor.shape[0]):
            logger.debug(f"Feature map {i} shape: {features_tensor[i].shape}")

        logger.debug(f"RM tensor shape: {rm_tensor.shape}")
        for i in range(rm_tensor.shape[0]):
            logger.debug(f"RM tensor {i} shape: {rm_tensor[i].shape}")

        logger.debug(f"Threshold map tensor shape: {thres_map_tensor.shape}")
        for i in range(thres_map_tensor.shape[0]):
            logger.debug(f"Threshold map tensor {i} shape: {thres_map_tensor[i].shape}")

        logger.debug(f"Record length tensor: {record_len}")

        logger.debug(f"Poses:")
        for i, pose in enumerate(poses):
            logger.debug(f"Pose {i}: Location ({pose.location.x}, {pose.location.y}, {pose.location.z}), "
                  f"Rotation ({pose.rotation.roll}, {pose.rotation.yaw}, {pose.rotation.pitch})")
        for rsu in self.rsu_manager_list:
            logger.debug(f"RSU pose: Location ({rsu.localizer.get_ego_pos().location.x}, "
                  f"{rsu.localizer.get_ego_pos().location.y}, {rsu.localizer.get_ego_pos().location.z}), "
                  f"Rotation ({rsu.localizer.get_ego_pos().rotation.roll}, "
                  f"{rsu.localizer.get_ego_pos().rotation.yaw}, {rsu.localizer.get_ego_pos().rotation.pitch})")

        for vehicle_manager in self.vehicle_manager_list:
            logger.debug(f"Vehicle pose: Location "
                    f"({vehicle_manager.localizer.get_ego_pos().location.x}, "
                    f"{vehicle_manager.localizer.get_ego_pos().location.y}, "
                    f"{vehicle_manager.localizer.get_ego_pos().location.z}), "
                    f"Rotation ({vehicle_manager.localizer.get_ego_pos().rotation.roll}, "
                    f"{vehicle_manager.localizer.get_ego_pos().rotation.yaw}, "
                    f"{vehicle_manager.localizer.get_ego_pos().rotation.pitch})")


        # 2. Warp all feature maps to the anchor vehicle's frame BEFORE fusion
        #aligned_features_list = [features_tensor[0]] # The anchor is already aligned
        #H, W = features_tensor.shape[2], features_tensor.shape[3]
        
        #for i in range(1, features_tensor.shape[0]):
            
        #    transformation_matrix_4x4 = pairwise_t_matrix[0, 0, i]

            # === THE FINAL, DEFINITIVE FIX: INVERT THE MATRIX FOR PYTORCH ===
            # F.affine_grid requires the INVERSE transformation, which maps the
            # output coordinates back to the source for sampling.
            
            # Also, convert from the model's Right-Handed system back to CARLA's Left-Handed
            # system just for this operation, as the raw feature maps are in that frame.
        #    conversion_matrix = torch.tensor([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], 
         #                                    device=transformation_matrix_4x4.device, dtype=torch.float32)
            
            # Apply the inverse and the coordinate conversion
        #    inverse_transformation_matrix = torch.inverse(transformation_matrix_4x4)
        #    corrected_inverse_matrix = conversion_matrix @ inverse_transformation_matrix @ conversion_matrix

        #    affine_matrix = corrected_inverse_matrix[:2, [0, 1, 3]].unsqueeze(0)
            # === END FIX ===
            
            # Warp the feature map using the correctly normalized matrix
        #    feature_map_i = features_tensor[i].unsqueeze(0)
        #    grid = F.affine_grid(affine_matrix, feature_map_i.size(), align_corners=False)
        #    aligned_feature = F.grid_sample(feature_map_i, grid, align_corners=False)

            # === VISUALIZATION CODE ===
            # Convert the first channel of the tensors to NumPy for plotting
        #    original_bev_image = feature_map_i.squeeze(0)[0].cpu().numpy()
        #    warped_bev_image = aligned_feature.squeeze(0)[0].cpu().numpy()
            
            # Plot the original BEV from the RSU/other vehicle
        #    plt.imshow(original_bev_image)
        #    plt.title(f"Original BEV from Agent {i} (Frame {frame_id})")
        #    plt.savefig(f"debug_original_agent_{i}_frame_{frame_id}.png")
        #    plt.close()

            # Plot the warped BEV (how the anchor vehicle sees it)
        #    plt.imshow(warped_bev_image)
        #    plt.title(f"Warped BEV from Agent {i} (Frame {frame_id})")
        #    plt.savefig(f"debug_warped_agent_{i}_frame_{frame_id}.png")
        #    plt.close()
            # === END VISUALIZATION ===

            
            #aligned_features_list.append(aligned_feature.squeeze(0))

        #aligned_features_tensor = torch.stack(aligned_features_list)

        #if len(poses) > 1:
            #print("\n--- TRANSFORM DEBUG ---")
            #ego_pose = poses[0]
            #rsu_pose = poses[1]
            #print(f"Ego Pose (CARLA World): {ego_pose.location}")
            #print(f"RSU Pose (CARLA World): {rsu_pose.location}")
            
            # This is the matrix that transforms from RSU frame (agent 1) to Ego frame (agent 0)
            #rsu_to_ego_matrix = pairwise_t_matrix[0, 0, 1].cpu().numpy()
            #print("RSU-to-Ego Matrix:\n", np.round(rsu_to_ego_matrix, 2))
            
            # Test the transformation on the RSU's origin point (0,0,0)
            #rsu_origin_local = np.array([0, 0, 0, 1])
            #rsu_origin_in_ego_frame = rsu_to_ego_matrix @ rsu_origin_local
            #print("RSU origin (0,0,0) transformed into Ego's frame:", np.round(rsu_origin_in_ego_frame[:3], 2))
            #print("--- END TRANSFORM DEBUG ---\n")

            #logger.debug(f"Running detection heads...")

        # RSU-only detection (only if we have features from both vehicle and RSU)
        RSU_IDX = 1               # adapt if your ordering differs
        if features_tensor.shape[0] > RSU_IDX:
            rsu_raw   = features_tensor[RSU_IDX:RSU_IDX+1]          # [1,C,H,W]
            rsu_rm     = rm_tensor[RSU_IDX:RSU_IDX+1]
            rsu_thres  = thres_map_tensor[RSU_IDX:RSU_IDX+1]


            logger.debug(f"RSU raw : {rsu_raw.shape}")

            # --- call backbone the way it expects ---
            data_dict = {'spatial_features': rsu_raw}
            data_dict = self.model.backbone(data_dict)          # <- dict in, dict out
            rsu_feat  = data_dict['spatial_features_2d']        # [1,384,96,352]
            logger.debug(f"RSU feat after backbone: {rsu_feat.shape}")

            rsu_feat = self.model.shrink_conv(rsu_feat) if self.model.shrink_flag else rsu_feat

            logger.debug(f"RSU feature shape after backbone and shrink: {rsu_feat.shape}, RM shape: {rsu_rm.shape}, Threshold map shape: {rsu_thres.shape}")


            # ---- forward heads (no shrink‑conv) ----
            psm_rsu = self.model.cls_head(rsu_feat)                  # [1,2,H,W]
            rm_rsu  = self.model.reg_head(rsu_feat)                  # [1,14,H/2,W/2]

            # ---- build a faux‑batch dict for post_processor ----
            anchor_box_rsu = self.post_processor.generate_anchor_box()
            data_dict_rsu  = {
                'ego': {
                    'anchor_box'          : torch.from_numpy(anchor_box_rsu).cuda(),
                    'transformation_matrix': torch.eye(4).cuda()     # RSU frame ↔ RSU frame
                }
            }
            out_dict_rsu = {'ego': {'psm': psm_rsu, 'rm': rm_rsu}}

            boxes_rsu, scores_rsu = self.post_processor.post_process(
                    data_dict_rsu, out_dict_rsu)
        else:
            logger.debug(f"Not enough features for RSU-only detection (got {features_tensor.shape[0]}, need > {RSU_IDX})")
            boxes_rsu = None

        if boxes_rsu is None:
            logger.debug("RSU‑only heads produced **no** boxes")
        else:
            # convert 8×3 corners → centre lwh yaw for nicer print
            ctr = box_utils.corner_to_center(boxes_rsu.detach().cpu().numpy(), order='hwl')
            logger.debug("RSU‑ONLY DETECTIONS ===")
            for i,b in enumerate(ctr):
                logger.debug(f"[{i:02}] RSU‑frame xyz=({b[0]:6.2f},{b[1]:6.2f},{b[2]:4.1f}) "
                      f"lwh=({b[5]:.2f},{b[4]:.2f},{b[3]:.2f}) "
                      f"yaw={np.degrees(b[6]):6.1f}°  score={scores_rsu[i]:.3f}")
            # --- 1.  Ground‑truth vehicle in RSU frame ---------------------------
            rsu_tf   = poses[1]                 # carla.Transform of the RSU
            gt_world = np.array([gt['world'].location.x,
                                 gt['world'].location.y,
                                 gt['world'].location.z, 1.0])
            T_w2rsu  = np.linalg.inv(np.array(rsu_tf.get_matrix()))
            gt_rsu   = T_w2rsu @ gt_world
            logger.debug(f"GT in RSU frame xyz = {gt_rsu[:3]}")

            # --- 2.  Compare with the RSU box you printed ------------------------
            for i,b in enumerate(ctr):
                logger.debug(f"box[{i}] delta‑RSU‑frame (dx,dy) = {b[0]-gt_rsu[0]:.2f} {b[1]-gt_rsu[1]:.2f}")


            # …optionally move them to **world** to compare with GT
            rsu_world_T = np.array(poses[RSU_IDX].get_matrix())      # 4×4
            b_corners_h = np.hstack((ctr[:,:3], np.ones((len(ctr),1))))
            b_world     = (rsu_world_T @ b_corners_h.T).T
            logger.debug("--- same boxes in WORLD coords (x,y) ---")
            for i,p in enumerate(b_world):
                logger.debug(f"[{i:02}] (x={p[0]:7.2f}, y={p[1]:7.2f})")

        logger.debug(f"Calling fusion_net...")
        logger.debug(features_tensor.shape)  # expect (..., 1000) as the last dim
        fused_feature, communication_rates, result_dict = self.model.fusion_net(
            features_tensor,
            psm_tensor,
            thres_map_tensor,
            record_len,
            pairwise_t_matrix,
            backbone=self.model.backbone,
            heads=[self.model.shrink_conv, self.model.cls_head, self.model.reg_head]
        )
        logger.debug(f"Fusion complete. Fused feature shape: {fused_feature.shape}")

        if self.model.shrink_flag:
            fused_feature = self.model.shrink_conv(fused_feature)
        
        pred_dict = {'psm': self.model.cls_head(fused_feature), 'rm': self.model.reg_head(fused_feature)}


        
        
        logger.debug(f"Time taken for fusion and detection in ms: {(time.time() - start_time) * 1000} ms")
        det_results_ab3d = self._to_ab3dmot_format(pred_dict, frame_id, ego_pose_for_fusion)
        logger.debug(f"Detection complete. Found {len(det_results_ab3d.get('dets', []))} objects.")
        logger.debug(f"Detection results: {det_results_ab3d.get('dets', [])[:5]}... (showing first 5 detections)")
        
        logger.debug(f"Calling tracker...")
        # Use persistent tracker instance (created in __init__)
        tracked_result = self.tracker.track(det_results_ab3d, tick)

        # AB3DMOT returns (results, affi) tuple where results is a list containing one array
        # Extract the actual tracks array from the result
        if tracked_result is not None and len(tracked_result) > 0:
            results_list, affi = tracked_result
            tracked_dets = results_list[0] if len(results_list) > 0 else np.empty((0, 15))
        else:
            tracked_dets = np.empty((0, 15))

        logger.debug(f"Tracks result: {tracked_dets[:5]}... (showing first 5 tracks)" if len(tracked_dets) > 0 else "No tracks found.")
        logger.debug(f"Tracking complete. Found {len(tracked_dets)} tracks.")

        # Update trajectories from tracked detections
        self._update_trajectories(tracked_dets)

        # Generate predictions using linear predictor
        edge_predictions = self.lin_pred.generate_predicted_trajectories(self.tracked_trajectories)
        logger.debug(f"Prediction complete. Generated {len(edge_predictions)} predicted trajectories.")

        self._update_agents_and_advance_sim(tick, predictions=edge_predictions)


    def _get_pairwise_transform(self, poses: List[carla.Transform]) -> torch.Tensor:
        """
        Calculates the full pairwise transformation matrix by leveraging the
        repository's trusted utility functions.
        """
        L = len(poses)
        max_cav = self.hypes['train_params']['max_cav']

        # Convert carla.Transform objects to the list format [x, y, z, roll, yaw, pitch]
        # that the x1_to_x2 utility function expects.
        pose_list = []
        for pose in poses:
            pose_list.append([pose.location.x, pose.location.y, pose.location.z,
                              pose.rotation.roll, pose.rotation.yaw, pose.rotation.pitch])

        # Create the 5D tensor expected by the fusion network
        pairwise_t_matrix = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))


        

        for i in range(L):
            for j in range(L):
                # correct direction:  i -> j  (anchor -> neighbour)
                pairwise_t_matrix[0, i, j] = transformation_utils.x1_to_x2(
                    pose_list[i],   # FROM  (source / anchor)
                    pose_list[j])   # TO    (target / neighbour)

        return torch.from_numpy(pairwise_t_matrix).float().cuda()


    def _get_pairwise_transform_old(self, poses: List[carla.Transform]) -> torch.Tensor:
        """
        Calculates the full pairwise transformation matrix by first converting
        CARLA poses to the standard LiDAR coordinate system.
        """
        L = len(poses)
        max_cav = self.hypes['train_params']['max_cav']
        #logger.debug(f"[_get_pairwise_transform] Number of poses: {L}, Max CAV: {max_cav}")
        
        # This matrix converts from CARLA's Unreal Engine coordinates (X-fwd, Y-right, Z-up)
        # to the standard LiDAR coordinate system (X-fwd, Y-left, Z-up).
        carla_to_lidar_matrix = np.array([[0, -1, 0, 0],
                                          [1, 0, 0, 0],
                                          [0, 0, 1, 0],
                                          [0, 0, 0, 1]])
        
        world_mats_lidar_frame = []
        for pose in poses:
            # Convert each agent's pose matrix into the LiDAR coordinate system
            world_mat_carla_frame = np.array(pose.get_matrix())
            world_mats_lidar_frame.append(world_mat_carla_frame @ carla_to_lidar_matrix)

        inv_world_mats = [np.linalg.inv(m) for m in world_mats_lidar_frame]
        
        pairwise_t_matrix = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))

        for i in range(L):
            for j in range(L):
                # Calculate the transform from agent j to agent i (now in the correct coordinate system)
                pairwise_t_matrix[0, i, j] = inv_world_mats[i] @ world_mats_lidar_frame[j]

        #logger.debug(f"[_get_pairwise_transform] Pairwise transformation matrix shape: {pairwise_t_matrix.shape}")
                
        return torch.from_numpy(pairwise_t_matrix).float().cuda()

    def _to_ab3dmot_format(self, pred_dict: Dict, frame_id: int, anchor_pose: carla.Transform) -> Dict:
        #logger.debug(f"[_to_ab3dmot_format] START")

        # 1. Prepare inputs for the post-processor
        anchor_box_np = self.post_processor.generate_anchor_box()
        
        # Create the data_dict with ALL required keys
        data_dict_for_post = {
            'ego': {
                'anchor_box': torch.from_numpy(anchor_box_np).cuda(),
                'transformation_matrix': torch.eye(4).cuda()
            }
        }
        # Wrap the model's prediction output to match the expected structure
        output_dict_for_post = {'ego': pred_dict}
        #logger.debug(f"[_to_ab3dmot_format] Calling post_process with anchor box shape: {data_dict_for_post['ego']['anchor_box'].shape}")

        # 2. Call post-processing to get detections (as corners) in the ANCHOR VEHICLE's local frame
        pred_box_corners_tensor, pred_score_tensor = self.post_processor.post_process(data_dict_for_post, output_dict_for_post)
        
        if pred_box_corners_tensor is None or len(pred_box_corners_tensor) == 0:
            #logger.debug(f"[_to_ab3dmot_format] No detections returned from post_processor.")
            return {'dets': np.empty((0, 7)), 'info': np.empty((0, 3))}
        #logger.debug(f"[_to_ab3dmot_format] Post-processor returned {len(pred_box_corners_tensor)} boxes.")

        # 3. Convert corners to 7-DoF boxes [x,y,z,h,w,l,yaw] in the LOCAL frame
        box_corners_local_np = pred_box_corners_tensor.cpu().detach().numpy()
        scores_np = pred_score_tensor.cpu().detach().numpy()
        
        boxes_local_7dof = box_utils.corner_to_center(box_corners_local_np, order='hwl')
        boxes_local_7dof[:, 1] *= -1
        #logger.debug(f"[_to_ab3dmot_format] Converted corners to 7dof boxes, shape: {boxes_local_7dof.shape}")

        # 4. Transform the 7-DoF boxes into the GLOBAL WORLD frame
        anchor_world_matrix = np.array(anchor_pose.get_matrix())
        
        # Add a column of 1s for homogeneous coordinates
        box_centers_local_homo = np.hstack((boxes_local_7dof[:, :3], np.ones((len(boxes_local_7dof), 1))))
        
        # Transform centers using the anchor's world pose
        box_centers_world = (anchor_world_matrix @ box_centers_local_homo.T).T
        
        # Create the final boxes in world coordinates
        world_frame_boxes = boxes_local_7dof.copy()
        world_frame_boxes[:, :3] = box_centers_world[:, :3]
        
        # Add the anchor's yaw
        anchor_yaw_rad = np.radians(anchor_pose.rotation.yaw)
        world_frame_boxes[:, 6] += anchor_yaw_rad
        #logger.debug(f"[_to_ab3dmot_format] Transformed {len(world_frame_boxes)} boxes to world frame.")

        # 5. Reorder columns for AB3DMOT's required format (h,w,l,x,y,z,ry)
        ab3d_boxes = world_frame_boxes[:, [3, 4, 5, 0, 1, 2, 6]]
        info = np.array([[frame_id, i, -1] for i in range(len(scores_np))])
        logger.debug(f"DEBUG‑BOX  world_frame_boxes[:5] =\n {world_frame_boxes[:5]}")
        logger.debug(f"DEBUG‑INFO anchor world pose (x,y,yaw) = {anchor_pose.location.x, anchor_pose.location.y, anchor_pose.rotation.yaw}")
        logger.debug(f"DEBUG‑INFO first lidar corner (local) = {box_corners_local_np[0,0]}")
        logger.debug(f"DEBUG‑INFO anchor_world_matrix =\n {anchor_world_matrix}")
        
        return {'dets': ab3d_boxes, 'info': info, 'scores': scores_np}

    def _update_trajectories(self, tracked_dets: np.ndarray | None):
        logger.debug(f"[_update_trajectories] Updating trajectories.")
        updated_ids = set()

        # Don't clear trajectories when no detections - let AB3DMOT's max_age handle track persistence
        if tracked_dets is None or len(tracked_dets) == 0:
            logger.debug(f"[_update_trajectories] No tracks this frame, keeping {len(self.tracked_trajectories)} existing trajectories.")
            # Don't prune here - trajectories will be naturally pruned when tracks stop appearing
            return

        for trk in tracked_dets:
            tid = int(trk[7])
            tf = _box_to_transform(trk[:7])
            updated_ids.add(tid)

            if tid not in self.tracked_trajectories:
                # Create corners and o3d bounding box from track data
                corners, o3d_bbx = _track_to_corners_and_bbx(trk)
                obstacle = ObstacleVehicle(corners, o3d_bbx, track_id=tid)
                self.tracked_trajectories[tid] = ObstacleTrajectory(obstacle, deque(maxlen=10))

            self.tracked_trajectories[tid].trajectory.appendleft(tf)
            # Update the obstacle's transform and location
            self.tracked_trajectories[tid].obstacle.transform = tf
            self.tracked_trajectories[tid].obstacle.location = tf.location
            # KF velocity (m/tick) for prediction
            # KITTI dx(10)=CARLA vx, KITTI dz(12)=CARLA vy
            if len(trk) > 12:
                kf_vx, kf_vy = float(trk[10]), float(trk[12])
                self.tracked_trajectories[tid].obstacle.kf_speed_mps = (
                    (kf_vx**2 + kf_vy**2)**0.5) / 0.05
                self.tracked_trajectories[tid].obstacle.kf_vx = kf_vx
                self.tracked_trajectories[tid].obstacle.kf_vy = kf_vy

        # Only prune trajectories for tracks that AB3DMOT has stopped outputting
        # (AB3DMOT handles track aging internally with max_age parameter)
        pruned_ids = list(self.tracked_trajectories.keys() - updated_ids)
        for tid in pruned_ids:
            del self.tracked_trajectories[tid]
        logger.debug(f"[_update_trajectories] {len(updated_ids)} tracks updated, {len(pruned_ids)} tracks pruned.")
        
    def _update_agents_and_advance_sim(self, tick, predictions):
        logger.debug(f"[_update_agents_and_advance_sim] Broadcasting results and advancing simulation for tick {tick}.")
        for vm in self.vehicle_manager_list:
            if predictions is not None and random.random() * 100 > self.downlink_pl:
                 vm.agent.edge_predictions = predictions.copy()
            else:
                 vm.agent.edge_predictions.clear()
            if not self.run_distributed:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())
        if not self.run_distributed:
            for rsu in self.rsu_manager_list:
                rsu.update_info()
                rsu.run_step()

    def evaluate(self):
        """
        Return evaluation results for EvaluationManager integration.

        Returns:
            Tuple[matplotlib.figure.Figure, str, Dict]:
                - figure: Matplotlib figure with profiling visualization
                - perform_txt: Text summary for log file
                - metrics: Dict of metrics for global_metrics
        """
        return self.profiler.get_evaluation_result()

