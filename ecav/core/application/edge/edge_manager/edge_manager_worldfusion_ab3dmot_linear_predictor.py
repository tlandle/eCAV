# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
WorldFusion edge manager with AB3DMOT tracking and linear prediction.

Combines intermediate fusion from PointPillarWorldFusion (Where2comm
attention-based fusion in world coordinates) with AB3DMOT 3D multi-object
tracking and linear constant-velocity prediction.

Pipeline:
    Vehicles/RSUs extract spatial_features (before backbone)
    and transmit features + pose to edge.

    Edge:
        1. Collect spatial_features from all agents
        2. Compute pairwise transforms to world anchor
        3. Run backbone + Where2comm fusion
        4. Run detection heads on fused features
        5. Post-process to world-frame detections
        6. AB3DMOT tracking
        7. Linear prediction (25 future steps)
        8. Distribute predictions to vehicles
"""
from __future__ import annotations
import os
import math
import random
import time
import zlib
from collections import deque
from typing import Dict, List, Deque, Tuple, Any, Optional

import numpy as np
import carla
import torch
from easydict import EasyDict as edict

from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.data_utils.post_processor.world_voxel_postprocessor import WorldVoxelPostprocessor
from opencood.utils import box_utils, transformation_utils
from opencood.tools import train_utils
from AB3DMOT_libs.model import AB3DMOT

from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.application.edge.edge_profiler import EdgeProfiler
from ecav.core.application.edge.ego_uniqueness_monitor import EgoUniquenessMonitor
from ecav.core.application.edge.beacon_id_manager import BeaconIdManager
from ecav.core.application.edge.latency import JitterBuffer
from .edge_manager_base import _BaseEdgeManager


def _box_to_transform(box: np.ndarray):
    """Convert a detection box [x,y,z,h,w,l,yaw] to picklable Transform."""
    from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    loc = _Loc(x=float(box[0]), y=float(box[1]), z=float(box[2]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)


class WorldFusionEdge(_BaseEdgeManager):
    """
    Edge manager implementing WorldFusion with AB3DMOT tracking and linear prediction.

    This edge manager receives intermediate features (spatial_features) from vehicles
    and RSUs, runs Where2comm fusion in world coordinates, performs detection,
    tracking, and prediction, then distributes results back to vehicles.
    """

    def __init__(self,
                 world: carla.World,
                 cfg: Dict[str, Any],
                 cav_world,
                 carla_client: carla.Client,
                 *,
                 world_dt: float = 0.05,
                 **kwargs):
        super().__init__(world, cfg, cav_world, carla_client, world_dt=world_dt, **kwargs)

        print("[WorldFusion Edge] Initializing...")

        # Load model configuration
        wf_cfg = cfg['worldfusion_model']
        hypes = load_yaml(wf_cfg['hypes_yaml'])
        self.hypes = hypes

        # World anchor pose from config [x, y, z, roll, yaw, pitch]
        # Default to origin with world-aligned axes
        self.world_anchor = cfg.get('world_anchor', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        print(f"[WorldFusion Edge] World anchor: {self.world_anchor}")

        # Load WorldFusion model
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        self.model = PointPillarWorldFusion(hypes['model']['args']).cuda().eval()
        ckpt_dir = os.path.dirname(wf_cfg['checkpoint'])
        epoch_id = int(wf_cfg['checkpoint'].split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(ckpt_dir, self.model, epoch_id)
        print(f"[WorldFusion Edge] Model loaded from epoch {epoch_id}.")

        # Post-processor for detection output (must match training config)
        self.post_processor = WorldVoxelPostprocessor(
            self.hypes['postprocess'],
            dataset=None,
            train=False
        )

        # AB3DMOT tracker configuration
        # Tuned to reduce ghost tracks:
        # - min_hits=3: require 3 consecutive detections before confirming track
        # - max_age=3: kill tracks after 3 frames without detection
        self.ab3dmot_config = edict({
            'vis': False,
            'save_path': None,
            'use_3d_iou': False,
            'thres': 2.0,
            'output_dir': None,
            'min_hits': 3,  # Require 3 detections to confirm track (filters sporadic false positives)
            'max_age': 3,   # Kill tracks after 3 frames without detection (reduces ghost persistence)
            'ego_com': None,
            'affi_pro': False,
            'dataset': "KITTI",
            'det_name': "deprecated",
            'anchoring': cfg.get("anchoring", True),
            'dup_x_max': cfg.get("dup_x_max", 8.0),
            'dup_y_max': cfg.get("dup_y_max", 2.0),
            'dup_size_ratio': cfg.get("dup_size_ratio", 2.5),
            'cull_consec_ticks': cfg.get("cull_consec_ticks", 3)
        })
        self.anchoring = cfg.get("anchoring", True)
        self.ab3dmot_category = 'Car'

        # Create persistent tracker instance (reused across frames)
        self.tracker = AB3DMOT(self.ab3dmot_config, self.ab3dmot_category)
        self.mot_tracker = self.tracker  # alias for evaluate()

        # Trajectory predictor: SMART (joint multi-agent) or linear (KF velocity)
        pred_type = cfg.get('predictor_type', 'linear').lower()
        if pred_type == 'smart':
            from ecav.core.prediction.smart_predictor_manager import (
                SMARTPredictorManager3D)
            smart_cfg = cfg['smart_predictor']
            self.predictor = SMARTPredictorManager3D(
                checkpoint_path=smart_cfg['checkpoint'],
                map_cache_path=smart_cfg.get('map_cache'),
                device=smart_cfg.get('device', 'cuda'),
                num_output_steps=25)
            print(f"[WorldFusion Edge] Using SMART predictor")
        else:
            self.predictor = LinearPredictorManager(num_future_steps=25)
            print(f"[WorldFusion Edge] Using linear predictor")
        self.lin_pred = self.predictor  # alias for existing call sites

        # Tracked trajectories and history
        self.tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self.track_to_carla: Dict[int, int] = {}

        # Jitter buffer: features are stamped with per-packet arrival times
        # and drained in source-tick order (Apollo/Autoware pattern).
        # Payload: (feature_dicts, poses, carla_snapshot, excluded_vehicles)
        self._jitter_buffer: JitterBuffer = JitterBuffer(capacity=200)

        # GT snapshots indexed by source tick (for metrics evaluation)
        self._gt_snapshots: Dict[int, Dict] = {}
        self._excluded_snapshots: Dict[int, Dict] = {}
        self._latest_source_tick = None

        # Track history for AB3DMOT replay (following late fusion pattern)
        self.track_history: Deque[Dict] = deque(maxlen=30)

        # Persistent tracker output history for trajectory accumulation.
        # SMART needs 22 frames; 30 gives headroom.
        self._tracker_output_history: Deque[np.ndarray] = deque(maxlen=30)

        # Track velocity history for ghost track filtering
        self.track_velocities: Dict[int, Deque[float]] = {}

        # Resource profiler for capacity planning
        intersection_id = cfg.get('intersection_id', f"edge_{id(self)}")
        self.profiler = EdgeProfiler(
            intersection_id=intersection_id,
            history_size=2000,
            sample_gpu_utilization=True
        )

        # Tracking metrics accumulators
        self._prev_track_ids: set = set()  # Track IDs from previous frame
        self._track_to_gt_mapping: Dict[int, int] = {}  # track_id -> carla_vehicle_id
        self._cumulative_id_switches: int = 0
        self._cumulative_fragmentations: int = 0

        # Prediction metrics history (for computing ADE/FDE from past predictions)
        self._prediction_history: Deque[Dict] = deque(maxlen=100)
        self._last_ade_fde: Dict[str, float] = {
            'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0
        }

        self._last_update_tick = -1  # Guard against double-update

        # Ego-Uniqueness monitor
        self.ego_monitor = EgoUniquenessMonitor()

        # BSM J2945-inspired temporary ID rotation manager
        self.beacon_id_mgr = BeaconIdManager(
            rotation_interval_ticks=cfg.get(
                "beacon_id_rotation_interval", 200),
            rotation_distance_m=cfg.get(
                "beacon_id_rotation_distance", 100.0),
            world_dt=world_dt,
        )

        print("[WorldFusion Edge] Initialization complete.")

    def start_edge(self):
        for vm in self.vehicle_manager_list:
            vm.agent._anchoring = self.anchoring

    def update_information(self, frame_idx: int):
        """
        Collect spatial_features from all managed vehicles and RSUs.

        Stores features with their poses in the history buffer for latency-aware
        processing. Also captures a snapshot of all CARLA vehicle positions at
        this moment for accurate latency-aware evaluation.
        """
        # Guard against double-update in same tick
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx

        feature_dicts, poses = [], []

        # Capture snapshot of CARLA vehicles within the model's world_range box
        # Model uses world_range: [-40, -40, -3, 40, 40, 1] which is ±40m from anchor
        # GT filtering must match this box, not use a circular radius
        DETECTION_HALF_RANGE = 40.0  # meters - matches world_range in hypes yaml
        anchor_x, anchor_y = self.world_anchor[0], self.world_anchor[1]

        # Vehicle types the model was trained to detect (V2XSim "Car" class)
        # Exclude firetrucks, ambulances, police cars, etc. that aren't in training data
        VALID_VEHICLE_TYPES = {'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
                               'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
                               'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
                               'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
                               'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
                               'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
                               'patrol', 'wrangler', 'rubicon',  # Added: common CARLA vehicles
                               'patrol', 'mkz_2017', 'model3', 'wrangler', 'carlacola'}

        carla_vehicles_snapshot = {}
        excluded_vehicles_snapshot = {}  # Track vehicles we exclude from GT (firetrucks, etc.)
        try:
            actors = self.world.get_actors()
            for actor in actors:
                if 'vehicle' in actor.type_id.lower():
                    loc = actor.get_location()

                    # Filter out invalid/underground vehicles (z < -10m means not in scene)
                    if loc.z < -10.0:
                        continue

                    # Check vehicle type first to determine if valid or excluded
                    vehicle_type = actor.type_id.split('.')[-1].lower()
                    is_valid_type = any(vt in vehicle_type for vt in VALID_VEHICLE_TYPES)

                    rot = actor.get_transform().rotation
                    vel = actor.get_velocity()
                    vehicle_data = {
                        'type': actor.type_id.split('.')[-1],
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'yaw': rot.yaw,
                        'vx': vel.x, 'vy': vel.y,
                        'speed': np.sqrt(vel.x**2 + vel.y**2)
                    }

                    if not is_valid_type:
                        # Track ALL excluded vehicles (firetrucks, etc.) regardless of range
                        # We need their positions to filter detections near them
                        excluded_vehicles_snapshot[actor.id] = vehicle_data
                        continue

                    # Only include VALID vehicles within the model's world_range box (±40m from anchor)
                    dx = abs(loc.x - anchor_x)
                    dy = abs(loc.y - anchor_y)
                    if dx > DETECTION_HALF_RANGE or dy > DETECTION_HALF_RANGE:
                        continue

                    carla_vehicles_snapshot[actor.id] = vehicle_data
        except Exception as e:
            print(f"[WorldFusion Edge] Warning: Could not capture CARLA snapshot: {e}")

        # Log excluded vehicles (emergency vehicles not in training data)
        if excluded_vehicles_snapshot:
            print(f"[WorldFusion Edge] {len(excluded_vehicles_snapshot)} excluded vehicles (not in training data)")

        # Collect from vehicles
        for vm in self.vehicle_manager_list:
            pm = vm.perception_manager
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                feature_dicts.append(pm.feature_dict)
                pos = vm.localizer.get_ego_pos()
                poses.append(pos)
                # Debug: Print CARLA position AND offset from world anchor
                dx = pos.location.x - self.world_anchor[0]
                dy = pos.location.y - self.world_anchor[1]
                # Model outputs in CARLA convention - detection coords should match (dx, dy)
                print(f"[WorldFusion Edge] Vehicle CARLA pos: x={pos.location.x:.2f}, y={pos.location.y:.2f}, yaw={pos.rotation.yaw:.2f}deg")
                print(f"[WorldFusion Edge]   Offset from anchor: dx={dx:.2f}, dy={dy:.2f} (detection LOCAL should match this)")

        # Collect from RSUs
        for rsu in self.rsu_manager_list:
            pm = rsu.perception_manager
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                feature_dicts.append(pm.feature_dict)
                pos = rsu.localizer.get_ego_pos()
                if pos is None and hasattr(rsu, 'spawn_position'):
                    sp = rsu.spawn_position
                    pos = carla.Transform(
                        carla.Location(x=sp[0], y=sp[1], z=sp[2] if len(sp) > 2 else 0.0),
                        carla.Rotation()
                    )
                poses.append(pos)
                dx = pos.location.x - self.world_anchor[0]
                dy = pos.location.y - self.world_anchor[1]
                print(f"[WorldFusion Edge] RSU CARLA pos: x={pos.location.x:.2f}, y={pos.location.y:.2f}, yaw={pos.rotation.yaw:.2f}deg")
                print(f"[WorldFusion Edge]   CARLA offset from anchor: dx={dx:.2f}, dy={dy:.2f}")

        # Tick the BSM temp-ID rotation for every managed vehicle so that
        # time/distance thresholds are evaluated each frame.
        for vm in self.vehicle_manager_list:
            loc = vm.vehicle.get_location()
            self.beacon_id_mgr.get_temp_id(vm.vehicle.id, loc, frame_idx)

        if feature_dicts:
            print(f"[WorldFusion Edge] Collected {len(feature_dicts)} feature_dicts from {len(self.vehicle_manager_list)} vehicles + {len(self.rsu_manager_list)} RSUs")
            # Stamp with per-packet arrival time and push to jitter buffer
            arrival = self.latency_model.stamp(frame_idx)
            payload = (feature_dicts, poses, carla_vehicles_snapshot, excluded_vehicles_snapshot)
            self._jitter_buffer.push(frame_idx, arrival, payload)
            # Store GT snapshots by source tick for metrics evaluation
            self._gt_snapshots[frame_idx] = carla_vehicles_snapshot
            self._excluded_snapshots[frame_idx] = excluded_vehicles_snapshot
            # Prune old GT entries (keep last 200 ticks)
            for old in [k for k in self._gt_snapshots if frame_idx - k > 200]:
                del self._gt_snapshots[old]
                self._excluded_snapshots.pop(old, None)
        else:
            print(f"[WorldFusion Edge] WARNING: No feature_dicts collected!")

    def run_step(self, tick: int):
        """
        Main edge processing step with profiling.

        1. Handle latency to get delayed snapshot
        2. Stack features and compute pairwise transforms
        3. Run backbone + Where2comm fusion
        4. Run detection heads and post-process
        5. Track with AB3DMOT
        6. Generate predictions
        7. Distribute to vehicles
        """
        with self.profiler.profile_frame(tick) as frame:
            # Feature collection (includes update_information)
            with frame.time("feature_collection"):
                self.update_information(tick)

                # Drain jitter buffer for all arrived frames
                new_frames = self._jitter_buffer.drain(tick)

            if not new_frames:
                serialized_preds = self._update_agents(tick, None)
                frame.set_counts(num_agents=0, num_detections=0, num_tracks=0, num_predictions=0)
                return serialized_preds

            # Process the latest arrived frame (NN fusion is expensive)
            latest_source_tick, (feature_dicts, poses, carla_snapshot_at_capture, excluded_vehicles) = new_frames[-1]
            frame_id = latest_source_tick
            lag_steps = tick - latest_source_tick
            frame.set_aoi_ticks(lag_steps)
            num_agents = len(feature_dicts)

            # 3. Stack features and compute pairwise transforms
            with frame.time("fusion"):
                torch.cuda.empty_cache()
                spatial_features = torch.cat(
                    [d['spatial_features'] for d in feature_dicts], dim=0
                ).float().cuda()

                pairwise_t_matrix = self._compute_world_pairwise_transforms(poses)
                record_len = torch.tensor([len(feature_dicts)], dtype=torch.int64).cuda()

                # 4. Run backbone + Where2comm fusion
                with torch.no_grad():
                    fused_feature, pred_dict = self._run_fusion(
                        spatial_features, pairwise_t_matrix, record_len
                    )

            # 5. Post-process to get detections in world frame
            with frame.time("detection"):
                det_results = self._to_ab3dmot_format(pred_dict, frame_id)
                num_dets = len(det_results.get('dets', []))
                print(f"[WorldFusion Edge] Detection: {num_dets} objects found (before self-filter)")

                # 5.5 Filter out self-detections using beacon positions of managed VEHICLES only
                num_vehicles = len(self.vehicle_manager_list)
                vehicle_poses = poses[:num_vehicles]
                det_results = self._filter_self_detections(det_results, vehicle_poses)
                num_dets_after = len(det_results.get('dets', []))
                print(f"[WorldFusion Edge] Detection: {num_dets_after} objects after self-beacon filter")
                if num_dets_after > 0:
                    print(f"[WorldFusion Edge] First det: {det_results['dets'][0]}")

                # 5.6 Compute detection metrics (TP/FP/FN) against ground truth
                det_metrics = self._compute_detection_metrics(
                    det_results, carla_snapshot_at_capture, vehicle_poses,
                    excluded_vehicles=excluded_vehicles
                )

            # Set detection metrics on profiler
            frame.set_detection_metrics(
                true_positives=det_metrics['tp'],
                false_positives=det_metrics['fp'],
                false_negatives=det_metrics['fn']
            )

            # 6. Track with AB3DMOT using persistent tracker
            with frame.time("tracking"):
                self.track_history.appendleft(det_results)
                tracks, _ = self.tracker.track(det_results, frame_id)

                # Reconcile any pending BSM temp-ID rotations
                for evt in self.beacon_id_mgr.pop_pending_rotations():
                    rec = self.beacon_id_mgr.get_record(evt.carla_id)
                    old_pos = evt.position
                    new_pos = rec.last_position if rec is not None else evt.position
                    old_vel = evt.velocity if evt.velocity is not None \
                        else np.zeros(3, dtype=np.float32)
                    elapsed = max(tick - evt.tick, 1)
                    if self.beacon_id_mgr.reconcile_id_change(
                            evt.old_temp_id, evt.new_temp_id,
                            old_pos, new_pos, old_vel, elapsed):
                        BeaconIdManager.remap_tracker_identity(
                            self.tracker, evt.old_temp_id, evt.new_temp_id)
                    else:
                        print(
                            f"[WorldFusion Edge] BSM rotation reconcile FAILED "
                            f"carla={evt.carla_id} {evt.old_temp_id}->{evt.new_temp_id}")

                if tracks and len(tracks[0]) > 0:
                    self._tracker_output_history.append(tracks[0])

                # 7. Convert tracks to trajectories
                self._latest_source_tick = latest_source_tick
                self._ab3d_history_to_trajs(self._tracker_output_history, horizon=30)

                # 7.5 Filter out ghost/static tracks
                self._filter_ghost_tracks()

                num_tracks = len(self.tracked_trajectories)

                # 7.6 Compute tracking metrics (ID switches, MOTA, MOTP)
                track_metrics = self._compute_tracking_metrics(
                    tracks, carla_snapshot_at_capture, vehicle_poses
                )

            # Set tracking metrics on profiler
            frame.set_tracking_metrics(
                id_switches=track_metrics['id_switches'],
                fragmentations=track_metrics['fragmentations'],
                mota=track_metrics['mota']
            )

            # Ego-Uniqueness analysis
            latest_tracks = tracks[0] if tracks and len(tracks[0]) > 0 else None
            managed_ids = {vm.vehicle.id for vm in self.vehicle_manager_list}
            ego_poses = {}
            for vm in self.vehicle_manager_list:
                tf = vm.vehicle.get_transform()
                ego_poses[vm.vehicle.id] = (
                    tf.location.x, tf.location.y,
                    math.radians(tf.rotation.yaw))
            self.ego_monitor.update(tick, latest_tracks,
                                    carla_snapshot_at_capture, managed_ids,
                                    ego_poses=ego_poses)
            tick_record = self.ego_monitor.per_tick_records[-1]
            ego_ghost_count = sum(
                1 for cid in tick_record.duplicate_identities
                if cid in managed_ids
            )
            frame.set_ego_uniqueness_metrics(
                violations=tick_record.num_duplicates,
                duplicate_tracks=tick_record.num_duplicates,
                ego_ghosts=ego_ghost_count,
                geom_dup_clusters=tick_record.geom_dup_clusters,
                geom_dup_tracks=tick_record.geom_dup_tracks,
            )

            # 8. Linear prediction
            with frame.time("prediction"):
                predictions = self.lin_pred.generate_predicted_trajectories(
                    self.tracked_trajectories,
                    source_tick=latest_source_tick,
                    publish_tick=tick
                )
                num_predictions = len(predictions) if predictions else 0

                # 8.5 Evaluate predictions vs actual trajectories and get metrics
                pred_metrics = self._evaluate_predictions(
                    tick, predictions, carla_snapshot_at_capture, lag_steps
                )

            # Set prediction metrics on profiler
            frame.set_prediction_metrics(
                error_1s_m=pred_metrics.get('ade_1s', 0.0),
                error_2s_m=pred_metrics.get('ade_2s', 0.0),
                error_3s_m=pred_metrics.get('ade_3s', 0.0),
                fde_m=pred_metrics.get('fde', 0.0),
                miss_rate=pred_metrics.get('miss_rate', 0.0)
            )

            # 9. Distribute predictions to vehicles
            with frame.time("distribution"):
                serialized_preds = self._update_agents(tick, predictions)

            # Record counts for this frame
            frame.set_counts(
                num_agents=num_agents,
                num_detections=num_dets_after,
                num_tracks=num_tracks,
                num_predictions=num_predictions
            )

        return serialized_preds

    def _run_fusion(self, spatial_features: torch.Tensor,
                    pairwise_t_matrix: torch.Tensor,
                    record_len: torch.Tensor) -> Tuple[torch.Tensor, Dict]:
        """
        Run backbone and Where2comm fusion on intermediate features.

        Args:
            spatial_features: Stacked features from all agents [N, C*Z, H, W]
            pairwise_t_matrix: Transform matrices [1, max_cav, max_cav, 4, 4]
            record_len: Number of agents [1]

        Returns:
            fused_feature: Fused BEV feature [1, C, H, W]
            pred_dict: Dictionary with 'psm' and 'rm' detection outputs
        """
        sensor = self.model.sensor
        N = spatial_features.shape[0]

        # Run backbone on each agent's features to get spatial_features_2d
        bev_features = []
        psm_single_list = []

        for i in range(N):
            bd = {'spatial_features': spatial_features[i:i+1]}
            bd = sensor.backbone(bd)
            bev2d = bd['spatial_features_2d']
            if sensor.shrink_flag:
                bev2d = sensor.shrink_conv(bev2d)
            bev_features.append(bev2d)
            psm_single_list.append(self.model.cls_head(bev2d))

        spatial_2d = torch.cat(bev_features, dim=0)  # [N, C, H, W]
        psm_single = torch.cat(psm_single_list, dim=0)  # [N, anchor_num, H, W]

        # DEBUG: Check per-agent PSM at Lincoln's expected position
        # Lincoln at LOCAL (21.2, 13.7) from world anchor
        # But RSU is AT the anchor, so Lincoln is at (21.2, 13.7) in RSU's frame too
        h, w = psm_single.shape[2], psm_single.shape[3]
        lincoln_grid_x = int((21.2 + 40) / (80.0 / w))
        lincoln_grid_y = int((13.7 + 40) / (80.0 / h))
        for agent_i in range(N):
            agent_psm = psm_single[agent_i].sigmoid()  # [anchors, H, W]
            region = agent_psm[:, max(0,lincoln_grid_y-3):lincoln_grid_y+4,
                                 max(0,lincoln_grid_x-3):lincoln_grid_x+4]
            max_val = region.max().item() if region.numel() > 0 else 0

            # Find top 3 detection locations for this agent
            psm_flat = agent_psm.max(dim=0)[0]  # Max over anchors -> [H, W]
            top_vals, top_idxs = psm_flat.flatten().topk(3)
            top_locs = []
            for val, idx in zip(top_vals, top_idxs):
                gy, gx = idx // w, idx % w
                local_x = gx * (80.0 / w) - 40
                local_y = gy * (80.0 / h) - 40
                top_locs.append(f"({local_x:.1f},{local_y:.1f})={val:.3f}")
            print(f"[AGENT PSM] Agent {agent_i} Lincoln@({lincoln_grid_x},{lincoln_grid_y})={max_val:.4f}, Top3: {', '.join(top_locs)}")

        # Run Where2comm fusion
        # Note: Where2comm expects spatial_features (before backbone) for multi-scale
        # but we've already run the backbone, so we pass spatial_2d directly
        fused_feature, comm_rate, _ = self.model.fusion(
            spatial_features,  # Original pre-backbone features for warping
            psm_single,
            record_len,
            pairwise_t_matrix,
            backbone=sensor.backbone,
            heads=[sensor.shrink_conv if sensor.shrink_flag else None,
                   self.model.cls_head, self.model.reg_head]
        )

        # Apply shrink conv if needed
        if sensor.shrink_flag:
            fused_feature = sensor.shrink_conv(fused_feature)

        # Run detection heads
        pred_dict = {
            'psm': self.model.cls_head(fused_feature),
            'rm': self.model.reg_head(fused_feature)
        }

        # DEBUG: Check PSM values at Lincoln's expected position
        # Lincoln LOCAL: (21.2, 13.7), Canvas: ±40m with 0.4m voxels, stride 2
        # Grid position: ((21.2 + 40) / 0.8, (13.7 + 40) / 0.8) = (76.5, 67.1)
        psm = pred_dict['psm'].sigmoid()  # Convert to probabilities
        h, w = psm.shape[2], psm.shape[3]  # Typically 100x100
        lincoln_grid_x = int((21.2 + 40) / (80.0 / w))  # ~76
        lincoln_grid_y = int((13.7 + 40) / (80.0 / h))  # ~67

        # Also check ego position: LOCAL (-21.2, -14.0)
        ego_grid_x = int((-21.2 + 40) / (80.0 / w))  # ~23
        ego_grid_y = int((-14.0 + 40) / (80.0 / h))  # ~32

        # Get max PSM values in a 5x5 region around each expected position
        def get_local_max(psm, cx, cy, radius=3):
            x1, x2 = max(0, cx-radius), min(w, cx+radius+1)
            y1, y2 = max(0, cy-radius), min(h, cy+radius+1)
            region = psm[0, :, y1:y2, x1:x2]  # [anchors, dy, dx]
            return region.max().item(), region.argmax().item()

        lincoln_max, lincoln_argmax = get_local_max(psm, lincoln_grid_x, lincoln_grid_y)
        ego_max, ego_argmax = get_local_max(psm, ego_grid_x, ego_grid_y)

        print(f"\n[PSM DEBUG] Ego expected at grid ({ego_grid_x}, {ego_grid_y}): max_psm={ego_max:.4f}")
        print(f"[PSM DEBUG] Lincoln expected at grid ({lincoln_grid_x}, {lincoln_grid_y}): max_psm={lincoln_max:.4f}")
        print(f"[PSM DEBUG] PSM shape: {psm.shape}, global max: {psm.max().item():.4f}")

        # Find top 5 detection locations
        psm_flat = psm[0].max(dim=0)[0]  # Max over anchors -> [H, W]
        top_vals, top_idxs = psm_flat.flatten().topk(5)
        for i, (val, idx) in enumerate(zip(top_vals, top_idxs)):
            gy, gx = idx // w, idx % w
            local_x = gx * (80.0 / w) - 40  # Convert grid to local coords
            local_y = gy * (80.0 / h) - 40
            print(f"[PSM DEBUG] Top {i+1}: grid=({gx}, {gy}), local=({local_x:.1f}, {local_y:.1f}), psm={val:.4f}")

        return fused_feature, pred_dict

    def _compute_world_pairwise_transforms(self, poses: List[carla.Transform]) -> torch.Tensor:
        """
        Compute transforms from world anchor to each agent.

        The world anchor is a fixed reference frame defined in config.
        Row 0 of the pairwise matrix contains transforms from world anchor to each agent.

        Args:
            poses: List of carla.Transform for each agent

        Returns:
            pairwise_t_matrix: [1, max_cav, max_cav, 4, 4] transform matrices
        """
        L = len(poses)
        max_cav = self.hypes['train_params']['max_cav']

        # Convert poses to [x, y, z, roll, yaw, pitch] format
        pose_list = [
            [p.location.x, p.location.y, p.location.z,
             p.rotation.roll, p.rotation.yaw, p.rotation.pitch]
            for p in poses
        ]

        # Initialize with identity matrices
        pairwise = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))

        # Compute transforms from world anchor to each agent
        # T_anchor_to_j = x1_to_x2(world_anchor, pose_j)
        T_to_anchor = []
        for j in range(L):
            T_j_to_anchor = transformation_utils.x1_to_x2(
                pose_list[j],       # FROM agent j
                self.world_anchor   # TO world anchor
            )
            T_to_anchor.append(T_j_to_anchor)

            # DEBUG: Print transform details
            rot_deg = np.degrees(np.arctan2(T_j_to_anchor[1, 0], T_j_to_anchor[0, 0]))
            tx, ty = T_j_to_anchor[0, 3], T_j_to_anchor[1, 3]
            print(f"[XFORM DEBUG] T_agent{j}_to_anchor: trans=({tx:.2f}, {ty:.2f}), rot={rot_deg:.1f}°")
            print(f"[XFORM DEBUG]   agent{j} pose: x={pose_list[j][0]:.2f}, y={pose_list[j][1]:.2f}, yaw={pose_list[j][4]:.2f}°")
            print(f"[XFORM DEBUG]   anchor pose: x={self.world_anchor[0]:.2f}, y={self.world_anchor[1]:.2f}, yaw={self.world_anchor[4]:.2f}°")

        # Row 0 contains transforms from world anchor to each agent (for warping to world frame)
        for j in range(L):
            T_anchor_to_j = np.linalg.inv(T_to_anchor[j])
            pairwise[0, 0, j] = T_anchor_to_j

            # DEBUG: Print inverted transform
            rot_deg = np.degrees(np.arctan2(T_anchor_to_j[1, 0], T_anchor_to_j[0, 0]))
            tx, ty = T_anchor_to_j[0, 3], T_anchor_to_j[1, 3]
            print(f"[XFORM DEBUG] T_anchor_to_agent{j}: trans=({tx:.2f}, {ty:.2f}), rot={rot_deg:.1f}°")

        # Fill rest of matrix with standard pairwise transforms
        for i in range(1, L):
            for j in range(L):
                if i == j:
                    pairwise[0, i, j] = np.eye(4)
                else:
                    T_i_to_j = np.linalg.inv(T_to_anchor[j]) @ T_to_anchor[i]
                    pairwise[0, i, j] = T_i_to_j

        return torch.from_numpy(pairwise).float().cuda()

    def _to_ab3dmot_format(self, pred_dict: Dict, frame_id: int) -> Dict:
        """
        Convert model predictions to AB3DMOT format.

        Post-processes detection outputs and transforms to world coordinates.

        Args:
            pred_dict: Dictionary with 'psm' and 'rm' tensors
            frame_id: Current frame ID

        Returns:
            Dictionary with 'dets', 'info', 'scores', 'frame' for AB3DMOT
        """
        # Prepare inputs for post-processor
        # WorldVoxelPostprocessor.post_process expects world_anchor and lidar_pose
        # to avoid falling back to VoxelPostprocessor (see lines 191-206)
        anchor_box_np = self.post_processor.generate_anchor_box()

        # lidar_pose: array of agent poses, shape (N, 6) where 6 = [x,y,z,roll,yaw,pitch]
        lidar_pose_np = np.array([self.world_anchor])  # Use world anchor as reference pose

        data_dict_for_post = {
            'ego': {
                'anchor_box': torch.from_numpy(anchor_box_np).cuda(),
                'transformation_matrix': torch.eye(4).cuda(),
                'world_anchor': [self.world_anchor],  # List containing the anchor pose
                'lidar_pose': lidar_pose_np,  # Agent poses array
            }
        }
        output_dict_for_post = {'ego': pred_dict}

        # Post-process to get detections
        pred_box_corners_tensor, pred_score_tensor = self.post_processor.post_process(
            data_dict_for_post, output_dict_for_post
        )

        if pred_box_corners_tensor is None or len(pred_box_corners_tensor) == 0:
            return {
                'dets': np.empty((0, 7)),
                'info': np.empty((0, 3)),
                'scores': np.empty(0),
                'frame': frame_id
            }

        # Convert corners to 7-DoF boxes [x,y,z,h,w,l,yaw]
        box_corners_np = pred_box_corners_tensor.cpu().detach().numpy()
        scores_np = pred_score_tensor.cpu().detach().numpy()

        boxes_7dof = box_utils.corner_to_center(box_corners_np, order='hwl')

        # boxes_7dof format: [x, y, z, h, w, l, yaw] in local BEV grid frame
        # Debug: print local coordinates BEFORE transform
        print(f"\n[COORD DEBUG] World anchor: x={self.world_anchor[0]:.2f}, y={self.world_anchor[1]:.2f}")
        for i in range(min(len(boxes_7dof), 3)):
            local_x, local_y, local_yaw = boxes_7dof[i, 0], boxes_7dof[i, 1], boxes_7dof[i, 6]
            print(f"[COORD DEBUG] Det {i} LOCAL: x={local_x:.2f}, y={local_y:.2f}, yaw={np.degrees(local_yaw):.1f}deg (should match vehicle offset from anchor)")

        # DEBUG: Check what CARLA actors exist at detected positions vs actual vehicle positions
        print(f"\n[CARLA ACTORS] Checking what's actually in the scene:")
        try:
            # Find all vehicles in CARLA
            actors = self.world.get_actors()
            vehicles_in_scene = []
            for actor in actors:
                if 'vehicle' in actor.type_id.lower():
                    loc = actor.get_location()
                    # Compute LOCAL position (relative to world anchor)
                    local_x = loc.x - self.world_anchor[0]
                    local_y = loc.y - self.world_anchor[1]
                    vehicles_in_scene.append({
                        'type': actor.type_id,
                        'id': actor.id,
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'local_x': local_x, 'local_y': local_y
                    })
                    vname = actor.type_id.split('.')[-1]
                    print(f"[CARLA ACTORS] {vname}: world=({loc.x:.1f}, {loc.y:.1f}), LOCAL=({local_x:.1f}, {local_y:.1f}), z={loc.z:.1f}")

            # Compare detections with actual vehicles (using LOCAL coords)
            print(f"\n[DETECTION VS REALITY] Comparing {len(boxes_7dof)} detections with {len(vehicles_in_scene)} vehicles:")
            for i in range(len(boxes_7dof)):
                det_local_x = boxes_7dof[i, 0]  # Already in LOCAL coords (before anchor offset)
                det_local_y = boxes_7dof[i, 1]

                # Find closest actual vehicle using LOCAL coordinates
                min_dist = float('inf')
                closest_vehicle = None
                offset_x, offset_y = 0, 0
                for v in vehicles_in_scene:
                    dist = np.sqrt((v['local_x'] - det_local_x)**2 + (v['local_y'] - det_local_y)**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_vehicle = v
                        offset_x = det_local_x - v['local_x']
                        offset_y = det_local_y - v['local_y']

                if closest_vehicle:
                    vname = closest_vehicle['type'].split('.')[-1]
                    status = "MATCH" if min_dist < 5.0 else f"OFFSET {min_dist:.1f}m"
                    print(f"[DETECTION VS REALITY] Det {i}: LOCAL=({det_local_x:.1f}, {det_local_y:.1f}), score={scores_np[i]:.3f}")
                    print(f"[DETECTION VS REALITY]   -> Closest: {vname} at LOCAL=({closest_vehicle['local_x']:.1f}, {closest_vehicle['local_y']:.1f})")
                    print(f"[DETECTION VS REALITY]   -> ERROR: dx={offset_x:+.1f}m, dy={offset_y:+.1f}m, dist={min_dist:.1f}m - {status}")

        except Exception as e:
            import traceback
            print(f"[CARLA ACTORS] Error: {e}")
            traceback.print_exc()

        # Model output is ALREADY in CARLA's coordinate convention (left-handed, Y right)
        # No coordinate flip needed - just add anchor offset
        # (Previous Y/yaw negation was incorrect)

        # Now add world anchor offset to get CARLA world coordinates
        boxes_7dof[:, 0] += self.world_anchor[0]  # x
        boxes_7dof[:, 1] += self.world_anchor[1]  # y
        boxes_7dof[:, 2] += self.world_anchor[2]  # z

        # Debug: print world coordinates AFTER adding anchor offset
        for i in range(min(len(boxes_7dof), 3)):
            world_x, world_y, world_yaw = boxes_7dof[i, 0], boxes_7dof[i, 1], boxes_7dof[i, 6]
            print(f"[COORD DEBUG] Det {i} CARLA WORLD: x={world_x:.2f}, y={world_y:.2f}, yaw={np.degrees(world_yaw):.1f}deg (should match vehicle CARLA pos)")

        # Reorder columns for AB3DMOT's required format (h,w,l,x,y,z,ry,score)
        # Score is appended as column 8 so Box3D.array2bbox_raw sets bbox.s correctly;
        # without it, bbox.s stays None and np.argsort in nms() raises TypeError.
        ab3d_boxes = np.column_stack([boxes_7dof[:, [3, 4, 5, 0, 1, 2, 6]], scores_np])
        info = np.array([[frame_id, i, -1] for i in range(len(scores_np))])

        return {
            'dets': ab3d_boxes,
            'info': info,
            'scores': scores_np,
            'frame': frame_id
        }

    def _filter_self_detections(self, det_results: Dict, poses: List[carla.Transform],
                                 threshold_m: float = 2.0) -> Dict:
        """
        Filter out detections that match managed vehicles' beacon positions (self-detections).

        The edge knows the positions of all managed vehicles from their beacons/localization.
        Any detection that falls within threshold distance of a managed vehicle is considered
        a self-detection and should be removed.

        Args:
            det_results: Detection results dict with 'dets', 'info', 'scores', 'frame'
            poses: List of carla.Transform for each managed vehicle/RSU
            threshold_m: Distance threshold in meters to consider a detection as self

        Returns:
            Filtered detection results dict
        """
        if len(det_results.get('dets', [])) == 0:
            return det_results

        dets = det_results['dets']  # (N, 7) in format [h,w,l,x,y,z,ry]
        info = det_results['info']
        scores = det_results['scores']
        frame_id = det_results['frame']

        # Get beacon positions of all managed vehicles
        beacon_positions = []
        for pose in poses:
            beacon_positions.append((pose.location.x, pose.location.y))

        # Also add RSU positions (they shouldn't detect themselves either)
        # RSUs are already in poses list from update_information

        # Filter detections
        keep_mask = []
        for i in range(len(dets)):
            det_x, det_y = dets[i, 3], dets[i, 4]  # x, y from [h,w,l,x,y,z,ry]

            is_self = False
            for bx, by in beacon_positions:
                dist = np.sqrt((det_x - bx)**2 + (det_y - by)**2)
                if dist < threshold_m:
                    print(f"[SELF-BEACON FILTER] Removing det at ({det_x:.1f}, {det_y:.1f}) - "
                          f"matches beacon at ({bx:.1f}, {by:.1f}), dist={dist:.2f}m")
                    is_self = True
                    break

            keep_mask.append(not is_self)

        # Apply mask
        keep_mask = np.array(keep_mask)
        if keep_mask.sum() == 0:
            return {
                'dets': np.empty((0, 7)),
                'info': np.empty((0, 3)),
                'scores': np.empty(0),
                'frame': frame_id
            }

        return {
            'dets': dets[keep_mask],
            'info': info[keep_mask],
            'scores': scores[keep_mask],
            'frame': frame_id
        }

    def _compute_detection_metrics(self, det_results: Dict,
                                    gt_vehicles: Optional[Dict],
                                    managed_poses: List[carla.Transform],
                                    excluded_vehicles: Optional[Dict] = None,
                                    distance_threshold: float = 10.0) -> Dict[str, int]:
        """
        Compute detection metrics (TP, FP, FN) by comparing detections to ground truth.

        A detection is a True Positive if it matches a ground truth vehicle (not managed)
        within distance_threshold meters.

        Args:
            det_results: Detection results with 'dets' array
            gt_vehicles: Dict of CARLA vehicle states (from snapshot) - filtered to valid types
            managed_poses: Poses of managed vehicles (to exclude from GT)
            excluded_vehicles: Dict of vehicles excluded from GT (e.g., firetrucks) -
                               detections near these are ignored, not counted as FP
            distance_threshold: Max distance for a detection to match GT

        Returns:
            Dict with 'tp', 'fp', 'fn' counts
        """
        if gt_vehicles is None:
            return {'tp': 0, 'fp': 0, 'fn': 0, 'motp_sum': 0.0}

        # Build list of excluded vehicle positions (detections near these are ignored)
        excluded_positions = []
        if excluded_vehicles:
            for v_id, v_data in excluded_vehicles.items():
                excluded_positions.append((v_data['x'], v_data['y']))

        dets = det_results.get('dets', np.empty((0, 7)))

        # Filter detections that are near excluded vehicles (e.g., emergency vehicles not in training data)
        # These are valid detections of things we chose not to evaluate, not FPs
        EXCLUSION_RADIUS = 10.0  # meters - for excluded vehicle types
        if len(dets) > 0 and excluded_positions:
            keep_mask = []
            for det_idx in range(len(dets)):
                det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]
                near_excluded = False
                for ex, ey in excluded_positions:
                    if np.sqrt((det_x - ex)**2 + (det_y - ey)**2) < EXCLUSION_RADIUS:
                        near_excluded = True
                        break
                keep_mask.append(not near_excluded)
            keep_mask = np.array(keep_mask)
            num_excluded_dets = (~keep_mask).sum()
            if num_excluded_dets > 0:
                print(f"[DET METRICS] Filtered {num_excluded_dets} detections near excluded vehicles")
            dets = dets[keep_mask] if keep_mask.any() else np.empty((0, 7))

        if len(dets) == 0:
            # All GT vehicles are false negatives
            # But exclude managed vehicles from GT count
            managed_positions = [(p.location.x, p.location.y) for p in managed_poses]
            fn_count = 0
            for v_id, v_data in gt_vehicles.items():
                # Check if this GT vehicle is managed (ego vehicle)
                is_managed = False
                for mx, my in managed_positions:
                    if np.sqrt((v_data['x'] - mx)**2 + (v_data['y'] - my)**2) < 3.0:
                        is_managed = True
                        break
                if not is_managed:
                    fn_count += 1
            return {'tp': 0, 'fp': 0, 'fn': fn_count, 'motp_sum': 0.0}

        # Get managed vehicle positions to exclude from GT matching
        managed_positions = [(p.location.x, p.location.y) for p in managed_poses]

        # Build list of GT positions (excluding managed vehicles)
        gt_positions = []
        for v_id, v_data in gt_vehicles.items():
            gx, gy = v_data['x'], v_data['y']
            # Check if this is a managed vehicle
            is_managed = False
            for mx, my in managed_positions:
                if np.sqrt((gx - mx)**2 + (gy - my)**2) < 3.0:
                    is_managed = True
                    break
            if not is_managed:
                gt_positions.append((v_id, gx, gy))

        # Greedy matching: match each detection to closest GT
        gt_matched = set()
        det_matched = set()
        motp_errors = []  # For MOTP calculation

        for det_idx in range(len(dets)):
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]  # x, y from [h,w,l,x,y,z,ry]

            best_dist = float('inf')
            best_gt_idx = -1

            for gt_idx, (v_id, gx, gy) in enumerate(gt_positions):
                if gt_idx in gt_matched:
                    continue
                dist = np.sqrt((det_x - gx)**2 + (det_y - gy)**2)
                if dist < best_dist and dist < distance_threshold:
                    best_dist = dist
                    best_gt_idx = gt_idx

            if best_gt_idx >= 0:
                gt_matched.add(best_gt_idx)
                det_matched.add(det_idx)
                motp_errors.append(best_dist)

        tp = len(det_matched)
        fp = len(dets) - tp
        fn = len(gt_positions) - len(gt_matched)
        motp_sum = sum(motp_errors)

        # Debug output for EVERY frame to track GT and detection counts over time
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        print(f"\n[DET DEBUG] GT={len(gt_positions)}, Dets={len(dets)}, TP={tp}, FP={fp}, FN={fn}, P={precision:.2f}, R={recall:.2f}")

        # Print each GT position (non-managed vehicles)
        for i, (v_id, gx, gy) in enumerate(gt_positions):
            v_data = gt_vehicles.get(v_id, {})
            v_type = v_data.get('type', 'unknown')
            matched = i in gt_matched
            status = "MATCHED" if matched else "MISSED"
            print(f"[DET DEBUG]   GT[{i}]: {v_type} id={v_id} at ({gx:.1f}, {gy:.1f}) - {status}")

        # Print each detection and what it matched
        for det_idx in range(len(dets)):
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]
            matched = det_idx in det_matched
            status = "TP" if matched else "FP"
            print(f"[DET DEBUG]   DET[{det_idx}]: ({det_x:.1f}, {det_y:.1f}) - {status}")

        return {'tp': tp, 'fp': fp, 'fn': fn, 'motp_sum': motp_sum}

    def _compute_tracking_metrics(self, tracks: Tuple,
                                   gt_vehicles: Optional[Dict],
                                   managed_poses: List[carla.Transform]) -> Dict[str, float]:
        """
        Compute tracking metrics: ID switches, fragmentations, MOTA.

        MOTA = 1 - (FN + FP + ID_switches) / num_GT
        ID Switch: when a track changes which GT vehicle it matches

        Args:
            tracks: Output from AB3DMOT tracker
            gt_vehicles: Dict of CARLA vehicle states
            managed_poses: Poses of managed vehicles (to exclude)

        Returns:
            Dict with 'id_switches', 'fragmentations', 'mota', 'motp'
        """
        if gt_vehicles is None or not tracks or len(tracks[0]) == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        track_array = tracks[0]  # Shape: (N, 8) - [h,w,l,x,y,z,ry,track_id]

        # Get managed positions
        managed_positions = [(p.location.x, p.location.y) for p in managed_poses]

        # Build GT list (excluding managed)
        gt_list = []
        for v_id, v_data in gt_vehicles.items():
            gx, gy = v_data['x'], v_data['y']
            is_managed = any(
                np.sqrt((gx - mx)**2 + (gy - my)**2) < 3.0
                for mx, my in managed_positions
            )
            if not is_managed:
                gt_list.append((v_id, gx, gy))

        num_gt = len(gt_list)
        if num_gt == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 1.0, 'motp': 0.0}

        # Match tracks to GT and detect ID switches
        current_track_ids = set()
        new_mapping = {}
        id_switches = 0
        motp_errors = []

        for i in range(track_array.shape[0]):
            track_id = int(track_array[i, 7])
            tx, ty = track_array[i, 3], track_array[i, 4]
            current_track_ids.add(track_id)

            # Find closest GT
            best_dist = float('inf')
            best_gt_id = None
            for v_id, gx, gy in gt_list:
                dist = np.sqrt((tx - gx)**2 + (ty - gy)**2)
                if dist < best_dist and dist < 10.0:
                    best_dist = dist
                    best_gt_id = v_id

            if best_gt_id is not None:
                motp_errors.append(best_dist)
                new_mapping[track_id] = best_gt_id

                # Check for ID switch
                if track_id in self._track_to_gt_mapping:
                    if self._track_to_gt_mapping[track_id] != best_gt_id:
                        id_switches += 1
                        print(f"[TRACK METRICS] ID Switch: track {track_id} "
                              f"changed from GT {self._track_to_gt_mapping[track_id]} to {best_gt_id}")

        # Detect fragmentations (tracks that disappeared and reappeared)
        fragmentations = 0
        lost_tracks = self._prev_track_ids - current_track_ids
        # If any lost track reappears later, that's a fragmentation
        # For now, just count lost tracks as potential fragmentations
        for lost_id in lost_tracks:
            if lost_id in self._track_to_gt_mapping:
                fragmentations += 1

        # Update state for next frame
        self._prev_track_ids = current_track_ids
        self._track_to_gt_mapping = new_mapping
        self._cumulative_id_switches += id_switches
        self._cumulative_fragmentations += fragmentations

        # Compute MOTA: 1 - (FN + FP + IDS) / GT
        # FN = GT objects not matched, FP = tracks not matched to GT
        matched_gt = set(new_mapping.values())
        fn = num_gt - len(matched_gt)
        fp = len(current_track_ids) - len(new_mapping)

        if num_gt > 0:
            mota = 1.0 - (fn + fp + id_switches) / num_gt
            mota = max(-1.0, min(1.0, mota))  # Clamp to [-1, 1]
        else:
            mota = 1.0

        # MOTP: mean localization error for matched tracks
        motp = np.mean(motp_errors) if motp_errors else 0.0

        return {
            'id_switches': id_switches,
            'fragmentations': fragmentations,
            'mota': mota,
            'motp': motp
        }

    def _correct_yaw_from_velocity(self, traj, current_yaw_rad: float) -> float:
        """
        Correct 180° yaw ambiguity using velocity direction.

        The model doesn't have a direction classification head, so vehicles
        at 180° get predicted as 0°. We fix this by checking if the velocity
        direction matches the predicted yaw.

        Args:
            traj: ObstacleTrajectory with history
            current_yaw_rad: Current predicted yaw in radians

        Returns:
            Corrected yaw in radians
        """
        if len(traj.trajectory) < 3:
            # Need at least 3 points to compute reliable velocity
            return current_yaw_rad

        # Get recent positions (trajectory is appendleft, so [0] is newest)
        try:
            pos_new = traj.trajectory[0].location
            pos_old = traj.trajectory[2].location  # 2 frames back for smoothing

            dx = pos_new.x - pos_old.x
            dy = pos_new.y - pos_old.y

            # Check if vehicle is moving (at least 0.5m over 2 frames)
            speed = np.sqrt(dx*dx + dy*dy)
            if speed < 0.5:
                return current_yaw_rad

            # Compute velocity direction
            vel_yaw_rad = np.arctan2(dy, dx)

            # Compute angular difference
            diff = vel_yaw_rad - current_yaw_rad
            # Normalize to [-pi, pi]
            diff = np.arctan2(np.sin(diff), np.cos(diff))

            # If difference is more than 90°, flip by 180°
            if abs(diff) > np.pi / 2:
                corrected_yaw = current_yaw_rad + np.pi
                # Normalize to [-pi, pi]
                corrected_yaw = np.arctan2(np.sin(corrected_yaw), np.cos(corrected_yaw))
                print(f"[YAW CORRECTION] Track: vel_yaw={np.degrees(vel_yaw_rad):.1f}°, "
                      f"pred_yaw={np.degrees(current_yaw_rad):.1f}°, "
                      f"corrected={np.degrees(corrected_yaw):.1f}° (flipped 180°)")
                return corrected_yaw

            return current_yaw_rad

        except Exception as e:
            return current_yaw_rad

    def _ab3d_history_to_trajs(self, hist: Deque[np.ndarray], horizon: int = 10):
        """
        Convert AB3DMOT tracking history to trajectory objects.

        Following the same pattern as late fusion edge manager.

        Args:
            hist: Deque of tracking results from AB3DMOT
            horizon: Maximum trajectory length
        """
        updated: set = set()
        # Clear trajectory deques — each tick replays AB3DMOT from scratch,
        # so stale positions from the previous tick's replay must not remain.
        for traj in self.tracked_trajectories.values():
            traj.trajectory.clear()

        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                # Track format: [h,w,l,x,y,z,ry,track_id,carla_id,...]
                tid = int(trk[7])
                cid_raw = int(trk[8]) if len(trk) > 8 else -1
                if self.anchoring:
                    real_cid = self.beacon_id_mgr.get_carla_id_for_temp(cid_raw)
                    cid = real_cid if real_cid is not None else cid_raw
                else:
                    cid = cid_raw

                # Convert to [x,y,z,h,w,l,yaw] for transform
                box_7dof = np.array([trk[3], trk[4], trk[5], trk[0], trk[1], trk[2], trk[6]])
                tf = _box_to_transform(box_7dof)

                updated.add(tid)

                if tid not in self.tracked_trajectories:
                    dummy = ObstacleVehicle(
                        corners=np.zeros((8, 3)),
                        o3d_bbx=None,
                        track_id=tid,
                        tick_id=0
                    )
                    self.tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon)
                    )

                traj = self.tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)

                # Correct 180° yaw ambiguity using velocity direction
                corrected_yaw = self._correct_yaw_from_velocity(traj, box_7dof[6])
                if corrected_yaw != box_7dof[6]:
                    # Update transform with corrected yaw (ecav_carla types
                    # to stay consistent with _box_to_transform output)
                    tf = _box_to_transform(np.array([
                        tf.location.x, tf.location.y, tf.location.z,
                        box_7dof[3], box_7dof[4], box_7dof[5],
                        corrected_yaw
                    ]))
                    # Also update the trajectory entry
                    traj.trajectory[0] = tf

                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid
                self.track_to_carla[tid] = cid
                # KF velocity (m/tick) for prediction
                # KITTI dx(10)=CARLA vx, KITTI dz(12)=CARLA vy
                if len(trk) > 12:
                    kf_vx, kf_vy = float(trk[10]), float(trk[12])
                    traj.obstacle.kf_speed_mps = (
                        (kf_vx**2 + kf_vy**2)**0.5) / 0.05
                    traj.obstacle.kf_vx = kf_vx
                    traj.obstacle.kf_vy = kf_vy

        # Only prune trajectories for tracks that AB3DMOT has stopped outputting
        # Don't prune if no updates this frame (let trajectories persist)
        if updated:
            for tid in list(self.tracked_trajectories):
                if tid not in updated:
                    del self.tracked_trajectories[tid]
        print(f"[WorldFusion Edge] {len(updated)} tracks updated, {len(self.tracked_trajectories)} total trajectories")

        # Spatial self-identification (anchoring OFF only)
        # Time-aligned (used) + naive (logged). See oracle for docstring.
        if not self.anchoring:
            src_tick = getattr(self, '_latest_source_tick', None)

            for vm in self.vehicle_manager_list:
                ego_loc_now = vm.vehicle.get_location()

                ego_x_aligned = ego_loc_now.x
                ego_y_aligned = ego_loc_now.y

                best_tid, best_dist = None, float('inf')
                naive_best_dist = float('inf')
                for tid, traj in self.tracked_trajectories.items():
                    loc = traj.obstacle.location
                    d_aligned = np.sqrt((loc.x - ego_x_aligned)**2 +
                                        (loc.y - ego_y_aligned)**2)
                    d_naive = np.sqrt((loc.x - ego_loc_now.x)**2 +
                                      (loc.y - ego_loc_now.y)**2)
                    if d_naive < naive_best_dist:
                        naive_best_dist = d_naive
                    if d_aligned < best_dist and d_aligned < self._self_id_radius:
                        best_dist = d_aligned
                        best_tid = tid
                if best_tid is not None:
                    self.tracked_trajectories[best_tid].obstacle.carla_id = \
                        vm.vehicle.id
                    self.track_to_carla[best_tid] = vm.vehicle.id

    def _filter_ghost_tracks(self, min_speed_threshold: float = 0.3, static_frames_to_remove: int = 5):
        """
        Filter out ghost/static tracks that haven't moved for several frames.

        Ghost tracks typically occur from:
        - False positive detections at fixed locations
        - Detections on static objects (parked cars, buildings)
        - Stale tracks that lost their association

        Args:
            min_speed_threshold: Minimum speed (m/frame) to consider a track moving
            static_frames_to_remove: Number of consecutive static frames before removal
        """
        tracks_to_remove = []

        for tid, traj in self.tracked_trajectories.items():
            # Initialize velocity history for this track if needed
            if tid not in self.track_velocities:
                self.track_velocities[tid] = deque(maxlen=static_frames_to_remove + 2)

            # Compute velocity from recent trajectory
            if len(traj.trajectory) >= 2:
                pos_new = traj.trajectory[0].location
                pos_old = traj.trajectory[1].location
                dx = pos_new.x - pos_old.x
                dy = pos_new.y - pos_old.y
                speed = np.sqrt(dx*dx + dy*dy)
            else:
                speed = 0.0

            self.track_velocities[tid].append(speed)

            # Check if track has been static for too long
            if len(self.track_velocities[tid]) >= static_frames_to_remove:
                recent_speeds = list(self.track_velocities[tid])[-static_frames_to_remove:]
                if all(s < min_speed_threshold for s in recent_speeds):
                    tracks_to_remove.append(tid)
                    pos = traj.trajectory[0].location if traj.trajectory else None
                    pos_str = f"({pos.x:.1f}, {pos.y:.1f})" if pos else "unknown"
                    print(f"[GHOST FILTER] Removing static track {tid} at {pos_str} "
                          f"(speeds: {[f'{s:.2f}' for s in recent_speeds]})")

        # Remove ghost tracks
        for tid in tracks_to_remove:
            del self.tracked_trajectories[tid]
            if tid in self.track_velocities:
                del self.track_velocities[tid]
            if tid in self.track_to_carla:
                del self.track_to_carla[tid]

        if tracks_to_remove:
            print(f"[GHOST FILTER] Removed {len(tracks_to_remove)} ghost tracks, "
                  f"{len(self.tracked_trajectories)} remaining")

    @staticmethod
    def _merge_wf_batches(batch_list):
        """
        Merge N per-agent WorldFusion Tensor batches into one for batch inference.

        voxel_coords column 0 is the batch index; must be reindexed per agent.
        All other fields are concatenated on dim 0 (the batch dimension).
        record_len = [1] * N tells the model how many agents are in the batch.
        """
        all_vf, all_vc, all_vnp = [], [], []
        all_imgs, all_rots, all_trans, all_intrins = [], [], [], []
        all_post_rots, all_post_trans, all_depth = [], [], []

        for agent_idx, batch in enumerate(batch_list):
            pc = batch['processed_lidar']
            vc = pc['voxel_coords'].clone()
            vc[:, 0] = agent_idx   # reindex batch_idx column
            all_vf.append(pc['voxel_features'])
            all_vc.append(vc)
            all_vnp.append(pc['voxel_num_points'])

            ii = batch['image_inputs']
            all_imgs.append(ii['imgs'])
            all_rots.append(ii['rots'])
            all_trans.append(ii['trans'])
            all_intrins.append(ii['intrins'])
            all_post_rots.append(ii['post_rots'])
            all_post_trans.append(ii['post_trans'])
            all_depth.append(ii['depth_map'])

        N = len(batch_list)
        return {
            'processed_lidar': {
                'voxel_features':   torch.cat(all_vf,  dim=0),
                'voxel_coords':     torch.cat(all_vc,  dim=0),
                'voxel_num_points': torch.cat(all_vnp, dim=0),
            },
            'image_inputs': {
                'imgs':       torch.cat(all_imgs,       dim=0),
                'rots':       torch.cat(all_rots,       dim=0),
                'trans':      torch.cat(all_trans,      dim=0),
                'intrins':    torch.cat(all_intrins,    dim=0),
                'post_rots':  torch.cat(all_post_rots,  dim=0),
                'post_trans': torch.cat(all_post_trans, dim=0),
                'depth_map':  torch.cat(all_depth,      dim=0),
            },
            'record_len': torch.tensor([1] * N, dtype=torch.int64),
        }

    def _run_o5_batch_encoder(self):
        """
        O5: gather pending batches from all WorldFusion-gRPC PMs, send one
        batched gRPC call to the WorldFusion server, and distribute
        spatial_features back.

        Returns True if the batch was sent, False if skipped (no eligible PMs or
        no sensors ready yet).  When True, each PM's _features_extracted_this_tick
        is set so that the subsequent detect() calls skip per-agent extraction.

        Borrows _wf_stub from ready[0] — all PMs connect to the same endpoint;
        gRPC channels are thread-safe.
        """
        import msgpack
        import msgpack_numpy as m_np
        m_np.patch()
        import numpy as np
        import perception_pb2
        from ecav.core.sensing.perception.worldfusion_perception_manager import (
            WorldFusionPerceptionManager)

        # Collect all WF-gRPC PMs (vehicles first, then RSUs — order must be
        # consistent between build_batch() and apply_features() splitting)
        wf_pms = []
        for vm in self.vehicle_manager_list:
            pm = vm.perception_manager
            if isinstance(pm, WorldFusionPerceptionManager) and pm.use_grpc:
                wf_pms.append(pm)
        for rsu in self.rsu_manager_list:
            pm = rsu.perception_manager
            if isinstance(pm, WorldFusionPerceptionManager) and pm.use_grpc:
                wf_pms.append(pm)

        if not wf_pms:
            return False

        # Phase 1: build all batches (CPU only, no RPC)
        for pm in wf_pms:
            pm.build_batch()

        ready = [pm for pm in wf_pms if pm._pending_batch is not None]
        if not ready:
            return False

        # Phase 2: merge + numpy conversion + gRPC call
        t_merge = time.time()
        merged = self._merge_wf_batches([pm._pending_batch for pm in ready])

        def _to_numpy(obj):
            if isinstance(obj, torch.Tensor):
                return obj.contiguous().numpy()
            if isinstance(obj, dict):
                return {k: _to_numpy(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [_to_numpy(v) for v in obj]
            return obj

        batch_np = _to_numpy(merged)
        # Cast imgs uint8 to reduce payload (~4× smaller), same as single-agent path
        if 'image_inputs' in batch_np and 'imgs' in batch_np['image_inputs']:
            batch_np['image_inputs']['imgs'] = batch_np['image_inputs']['imgs'].astype(np.uint8)

        payload = zlib.compress(msgpack.packb(batch_np, use_bin_type=True))
        t_pack = time.time()

        stub = ready[0]._wf_stub
        response = stub.ExtractWfFeatures(
            perception_pb2.WfRequest(payload=payload, actor_id=0),
            timeout=30,
        )
        t_grpc = time.time()

        result = msgpack.unpackb(response.payload, raw=False)
        t_unpack = time.time()

        # spatial_features shape: (N, C, H, W) — split on dim 0
        spatial_features = torch.from_numpy(result['spatial_features'])

        timing = {
            'to_numpy_ms':         0,
            'pack_ms':             round((t_pack  - t_merge) * 1000.0, 2),
            'payload_bytes':       len(payload),
            'grpc_ms':             round((t_grpc  - t_pack)  * 1000.0, 2),
            'server_read_ms':      0,
            'server_decode_ms':    round(response.unpack_ms, 2),
            'server_inference_ms': round(response.inference_ms, 2),
            'server_encode_ms':    round(response.pack_ms, 2),
            'server_total_ms':     round(response.total_ms, 2),
            'response_bytes':      len(response.payload),
            'unpack_ms':           round((t_unpack - t_grpc) * 1000.0, 2),
            'to_tensor_ms':        0,
            'total_e2e_ms':        round((t_unpack - t_merge) * 1000.0, 2),
        }

        print(f"[WorldFusion O5] batch={len(ready)} | "
              f"merge+pack={((t_pack-t_merge)*1000):.0f}ms ({len(payload)/1024:.0f}KB) | "
              f"grpc={((t_grpc-t_pack)*1000):.0f}ms "
              f"(srv_total={response.total_ms:.0f}ms) | "
              f"unpack={((t_unpack-t_grpc)*1000):.0f}ms | "
              f"total={((t_unpack-t_merge)*1000):.0f}ms", flush=True)

        # Phase 3: distribute features back to each PM
        for i, pm in enumerate(ready):
            pm.apply_features({'spatial_features': spatial_features[i:i+1].cpu()}, timing)

        return True

    def _update_agents(self, tick: int, predictions: Optional[List]):
        """
        Update all managed vehicles and RSUs with predictions.

        Returns ecloud.EdgeObjects for distributed prediction push.
        In sequential mode the return value is ignored by the scenario loop.
        """
        import pickle
        import ecloud_pb2 as ecloud

        print(f"[WorldFusion Edge] Broadcasting {len(predictions) if predictions else 0} predictions for tick {tick}")

        serialized_preds = ecloud.EdgeObjects()
        pickled_edge_predictions = None

        if predictions is not None and len(predictions) > 0:
            try:
                pickled_edge_predictions = pickle.dumps(predictions)
            except Exception as e:
                print(f"[WorldFusion Edge] Error serializing predictions: {e}")

        # Ego-consistency gate check at publish boundary
        if predictions:
            self._check_ego_gate_violations(tick, predictions)

        # O5: build+merge+POST all WF-litserve agents in one batch before the
        # per-agent update_info() loop.  Each PM's _features_extracted_this_tick
        # is set so detect() skips individual extraction on this tick.
        if not self.run_distributed:
            self._run_o5_batch_encoder()

        # Distribute predictions to vehicles
        for index, vm in enumerate(self.vehicle_manager_list):
            if predictions is not None and len(predictions) > 0 and random.random() * 100 > self.downlink_pl:
                vm.agent.edge_predictions = list(predictions)
                print(f"[WorldFusion Edge] Set {len(vm.agent.edge_predictions)} edge_predictions on vehicle")
            else:
                vm.agent.edge_predictions = []

            if pickled_edge_predictions is not None:
                object_buffer = ecloud.ObjectBuffer(
                    vehicle_id=index,
                    pickled_edge_predictions=pickled_edge_predictions
                )
                serialized_preds.all_object_buffers.append(object_buffer)

            if not self.run_distributed:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())
                self._label_brake_attributions_gt(vm)
                self._record_time_to_events(tick, vm)

        # Update RSUs
        if not self.run_distributed:
            for rsu in self.rsu_manager_list:
                rsu.update_info()
                rsu.run_step()

        return serialized_preds

    def _evaluate_predictions(self, tick: int, predictions: Optional[List],
                               carla_snapshot_at_capture: Optional[Dict] = None,
                               lag_steps: int = 0) -> Dict[str, float]:
        """
        Evaluate predicted trajectories against actual CARLA vehicle positions.

        Computes:
        - Current position error (distance from predicted to actual position)
        - Heading error (difference between predicted and actual yaw)
        - Velocity alignment (predicted direction vs actual movement)

        Also stores prediction snapshots to later evaluate ADE/FDE once the
        future becomes the present.

        Args:
            tick: Current simulation tick
            predictions: List of ObstaclePrediction objects from linear predictor
            carla_snapshot_at_capture: Vehicle positions at feature capture time (for latency-aware eval)
            lag_steps: Number of steps of latency

        Returns:
            Dict with prediction metrics: ade_1s, ade_2s, ade_3s, fde, miss_rate
        """
        default_metrics = {'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0}

        if not predictions:
            return default_metrics

        # Get current vehicles from CARLA
        try:
            actors = self.world.get_actors()
            carla_vehicles_now = {}
            for actor in actors:
                if 'vehicle' in actor.type_id.lower():
                    loc = actor.get_location()
                    rot = actor.get_transform().rotation
                    vel = actor.get_velocity()
                    carla_vehicles_now[actor.id] = {
                        'type': actor.type_id.split('.')[-1],
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'yaw': rot.yaw,
                        'vx': vel.x, 'vy': vel.y,
                        'speed': np.sqrt(vel.x**2 + vel.y**2)
                    }
        except Exception as e:
            print(f"[EVAL] Error getting CARLA actors: {e}")
            return

        # Use capture-time snapshot if available, otherwise use current positions
        carla_vehicles = carla_snapshot_at_capture if carla_snapshot_at_capture else carla_vehicles_now
        using_capture_time = carla_snapshot_at_capture is not None

        print(f"\n{'='*60}")
        print(f"[TRAJECTORY EVALUATION] Tick {tick}, lag={lag_steps} steps")
        print(f"{'='*60}")
        print(f"Predictions: {len(predictions)}, CARLA vehicles: {len(carla_vehicles)}")
        print(f"Comparing to: {'CAPTURE-TIME positions' if using_capture_time else 'CURRENT positions'}")

        # Match each prediction to closest CARLA vehicle
        for pred in predictions:
            # ObstaclePrediction has: obstacle_trajectory, transform, probability, predicted_trajectory
            # Get track_id from the obstacle
            try:
                track_id = pred.obstacle_trajectory.obstacle.track_id
            except AttributeError:
                track_id = -1

            # Get current position from the prediction's transform (in world coordinates)
            current_tf = pred.transform
            pred_x = current_tf.location.x
            pred_y = current_tf.location.y
            pred_yaw = current_tf.rotation.yaw

            # Find closest actual vehicle
            min_dist = float('inf')
            matched_vehicle = None
            matched_id = None

            for v_id, v_data in carla_vehicles.items():
                dist = np.sqrt((v_data['x'] - pred_x)**2 + (v_data['y'] - pred_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    matched_vehicle = v_data
                    matched_id = v_id

            if matched_vehicle is None:
                print(f"[EVAL] Track {track_id}: No match found")
                continue

            # Compute errors
            pos_error = min_dist
            heading_error = matched_vehicle['yaw'] - pred_yaw
            # Normalize heading error to [-180, 180]
            while heading_error > 180:
                heading_error -= 360
            while heading_error < -180:
                heading_error += 360

            # Print evaluation
            status = "GOOD" if pos_error < 2.0 else ("OK" if pos_error < 5.0 else "POOR")
            print(f"\n[EVAL] Track {track_id} -> {matched_vehicle['type']} (CARLA ID {matched_id})")
            print(f"  Predicted:  x={pred_x:7.2f}, y={pred_y:7.2f}, yaw={pred_yaw:6.1f}°")
            print(f"  Actual:     x={matched_vehicle['x']:7.2f}, y={matched_vehicle['y']:7.2f}, yaw={matched_vehicle['yaw']:6.1f}°")
            print(f"  Position Error: {pos_error:.2f}m [{status}]")
            print(f"  Heading Error:  {heading_error:+.1f}°")
            print(f"  Actual Speed:   {matched_vehicle['speed']:.2f} m/s")

            # Evaluate future predictions if available
            future_traj = pred.predicted_trajectory
            if future_traj and len(future_traj) > 0:
                # Get predicted positions at different horizons
                horizons = [5, 10, 25]  # steps into future (at 0.05s/step = 0.25s, 0.5s, 1.25s)
                pred_positions = []
                for h in horizons:
                    if h < len(future_traj):
                        fp = future_traj[h]
                        if hasattr(fp, 'location'):
                            pred_positions.append((h, fp.location.x, fp.location.y))
                        elif isinstance(fp, (list, tuple, np.ndarray)) and len(fp) >= 2:
                            pred_positions.append((h, fp[0], fp[1]))

                if pred_positions:
                    print(f"  Future predictions (from current pos):")
                    for h, fx, fy in pred_positions:
                        dx = fx - pred_x
                        dy = fy - pred_y
                        dist_from_current = np.sqrt(dx*dx + dy*dy)
                        pred_dir = np.degrees(np.arctan2(dy, dx))
                        print(f"    +{h*0.05:.2f}s: ({fx:.1f}, {fy:.1f}), "
                              f"dist={dist_from_current:.1f}m, dir={pred_dir:.0f}°")

        # Store current predictions for later ADE/FDE evaluation
        # (when future becomes present, we can compare)
        if not hasattr(self, '_prediction_history'):
            self._prediction_history = deque(maxlen=50)

        snapshot = {
            'tick': tick,
            'predictions': {},
            'actuals': dict(carla_vehicles)
        }
        for pred in predictions:
            try:
                track_id = pred.obstacle_trajectory.obstacle.track_id
            except AttributeError:
                continue

            future_traj = pred.predicted_trajectory
            if future_traj and len(future_traj) > 0:
                # Store current position + future predicted positions
                future_points = [(pred.transform.location.x, pred.transform.location.y)]
                for fp in future_traj:
                    if hasattr(fp, 'location'):
                        future_points.append((fp.location.x, fp.location.y))
                    elif isinstance(fp, (list, tuple, np.ndarray)) and len(fp) >= 2:
                        future_points.append((fp[0], fp[1]))
                snapshot['predictions'][track_id] = future_points

        self._prediction_history.append(snapshot)

        # Compute ADE/FDE from past predictions and store in self._last_ade_fde
        self._compute_historical_ade_fde(tick, carla_vehicles)

        print(f"{'='*60}\n")

        # Return the latest computed metrics
        return self._last_ade_fde.copy()

    def _compute_historical_ade_fde(self, current_tick: int, current_vehicles: Dict):
        """
        Compute ADE/FDE by comparing past predictions with current actual positions.

        ADE (Average Displacement Error): Mean position error over all predicted steps
        FDE (Final Displacement Error): Position error at the final predicted step

        Updates self._last_ade_fde with computed metrics.

        Args:
            current_tick: Current simulation tick
            current_vehicles: Dict of current CARLA vehicle states
        """
        if not hasattr(self, '_prediction_history') or len(self._prediction_history) < 2:
            return

        # Horizons: 20 steps = 1s, 40 steps = 2s, 60 steps = 3s (at 0.05s/step)
        # Linear predictor now generates 60 steps for full 3s prediction
        horizons_to_check = {
            'ade_1s': 20,   # 1 second = 20 steps
            'ade_2s': 40,   # 2 seconds = 40 steps
            'ade_3s': 60,   # 3 seconds = 60 steps
        }

        all_errors = []
        miss_count = 0
        total_predictions = 0
        miss_threshold = 4.0  # Miss threshold in meters (relaxed for urban scenarios)

        for metric_name, horizon in horizons_to_check.items():
            past_tick = current_tick - horizon

            # Find the prediction snapshot from that tick
            past_snapshot = None
            for snap in self._prediction_history:
                if snap['tick'] == past_tick:
                    past_snapshot = snap
                    break

            if past_snapshot is None:
                continue

            print(f"[ADE/FDE] Evaluating predictions from tick {past_tick} (horizon={horizon} steps = {horizon*0.05:.2f}s ago)")

            errors = []
            for track_id, pred_traj in past_snapshot['predictions'].items():
                # pred_traj[0] is current position at past_tick
                # pred_traj[1:] are future predictions
                # So pred_traj[horizon] is the prediction for 'horizon' steps into future
                if len(pred_traj) <= horizon:
                    continue

                # Get predicted position at this horizon
                # Index 0 = current pos at past_tick, index 1 = +1 step, ..., index horizon = +horizon steps
                pred_x, pred_y = pred_traj[horizon]

                # Find the actual vehicle that this prediction should match
                # Use past actuals to find the match, then look up current position
                past_actuals = past_snapshot['actuals']

                # Find which CARLA vehicle was closest to the track at prediction time
                min_dist = float('inf')
                matched_carla_id = None

                # We need the track's position at past_tick, which is pred_traj[0]
                if len(pred_traj) > 0:
                    track_x, track_y = pred_traj[0]
                    for v_id, v_data in past_actuals.items():
                        dist = np.sqrt((v_data['x'] - track_x)**2 + (v_data['y'] - track_y)**2)
                        if dist < min_dist and dist < 10.0:  # Must be within 10m to match
                            min_dist = dist
                            matched_carla_id = v_id

                if matched_carla_id is None:
                    continue

                # Now check if that vehicle still exists in current state
                if matched_carla_id not in current_vehicles:
                    continue

                actual = current_vehicles[matched_carla_id]
                actual_x, actual_y = actual['x'], actual['y']

                # Compute displacement error
                displacement_error = np.sqrt((pred_x - actual_x)**2 + (pred_y - actual_y)**2)
                errors.append(displacement_error)
                all_errors.append(displacement_error)

                # Track miss rate
                total_predictions += 1
                if displacement_error > miss_threshold:
                    miss_count += 1

                print(f"  Track {track_id}: pred=({pred_x:.1f}, {pred_y:.1f}), "
                      f"actual=({actual_x:.1f}, {actual_y:.1f}), "
                      f"error={displacement_error:.2f}m")

            if errors:
                ade = np.mean(errors)
                print(f"  Horizon {horizon}: ADE={ade:.2f}m (n={len(errors)})")
                # Store in metrics
                self._last_ade_fde[metric_name] = ade

        # Compute FDE (error at final step) and miss rate
        if all_errors:
            self._last_ade_fde['fde'] = all_errors[-1] if all_errors else 0.0
            self._last_ade_fde['miss_rate'] = miss_count / total_predictions if total_predictions > 0 else 0.0
            print(f"[ADE/FDE] Overall: FDE={self._last_ade_fde['fde']:.2f}m, "
                  f"Miss Rate={self._last_ade_fde['miss_rate']:.1%}")

    def evaluate(self):
        """
        Return evaluation results for EvaluationManager integration.

        Returns:
            Tuple[matplotlib.figure.Figure, str, Dict]:
                - figure: Matplotlib figure with profiling visualization
                - perform_txt: Text summary for log file
                - metrics: Dict of metrics for global_metrics
        """
        fig, txt, metrics = self.profiler.get_evaluation_result()
        metrics['ego_uniqueness'] = self.ego_monitor.get_metrics()
        metrics.update(self._get_latency_component_stats())
        metrics.update(self._get_contract_metrics())
        if hasattr(self, 'mot_tracker') and self.mot_tracker is not None:
            metrics['birth_gate'] = {
                'birth_attempts_anon': self.mot_tracker.birth_attempts_anon,
                'birth_suppressed_by_gate': self.mot_tracker.birth_suppressed_by_gate,
                'births_anon_after_gate': self.mot_tracker.births_anon_after_gate,
                'anon_cull_count': self.mot_tracker.anon_cull_count,
            }
        return fig, txt, metrics
