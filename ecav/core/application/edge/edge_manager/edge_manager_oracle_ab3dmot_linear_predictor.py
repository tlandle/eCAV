# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Oracle edge manager with AB3DMOT tracking and linear prediction.

Uses CARLA ground-truth transforms and bounding boxes as the detection
front-end instead of camera/LiDAR perception.  Everything downstream
(jitter buffer, AB3DMOT tracker, linear predictor, metrics, ego monitor)
is identical to the late-fusion manager so that the *only* variable
under test is detection quality.

Pipeline:
    CARLA GT actors  -->  latency buffer  -->  AB3DMOT (3-D MOT)
                               |
                               +--> track history (10 frames) --> linear
                                    constant-velocity predictor
                                    --> 25 future steps
"""
from __future__ import annotations

import math, random, time, logging, pickle
from collections import deque
from typing import Dict, List, Deque

import numpy as np
import carla

from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT

import ecloud_pb2 as ecloud

from ecav.core.prediction.linear_predictor_manager import \
    LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import \
    ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from ecav.core.application.edge.edge_metrics import EdgeMetrics
from ecav.core.application.edge.edge_profiler import EdgeProfiler
from ecav.core.application.edge.ego_uniqueness_monitor import EgoUniquenessMonitor
from ecav.core.application.edge.beacon_id_manager import BeaconIdManager

from .edge_manager_base import _BaseEdgeManager, logger


# ──────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────
_GUID = 0


def _box_to_transform(box):
    """Convert AB3DMOT box [h,w,l,x,y,z,yaw] to picklable Transform.

    Coordinates are in KITTI camera convention inside the tracker:
        KITTI x = CARLA x,  KITTI y = CARLA z (height),  KITTI z = CARLA y
    Swap y<->z back to CARLA world coordinates on output.
    """
    from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    h, w, l, x, ky, kz, yaw = box  # ky=CARLA_z, kz=CARLA_y
    loc = _Loc(x=float(x), y=float(kz), z=float(ky))
    rot = _Rot(yaw=np.degrees(float(yaw)))
    return _Tf(location=loc, rotation=rot)


def _collect_oracle_detections(edge,
                               gt_actors: List[dict],
                               beacons: Dict[int, tuple],
                               frame_idx: int,
                               beacon_id_mgr: BeaconIdManager = None):
    """
    Build the dict that AB3DMOT.track() expects from ground-truth actors.

    *gt_actors* is a list of dicts with keys:
        carla_id, x, y, z, yaw, hx, hy, hz  (extent half-sizes)

    Beacons (managed vehicles) are injected exactly as in late fusion so
    that the anchoring protocol can still operate.
    """
    global _GUID
    det_rows, info_rows = [], []

    # a) beacons (one per managed vehicle) --------------------------------
    #    KITTI camera convention: KITTI_x=CARLA_x, KITTI_y=CARLA_z, KITTI_z=CARLA_y
    managed_ids = set()
    for vm in edge.vehicle_manager_list:
        loc, ext = beacons[vm.vehicle.id]
        managed_ids.add(vm.vehicle.id)
        h, w, l = ext.z * 2, ext.y * 2, ext.x * 2
        det_rows.append([h, w, l, loc.x, loc.z, loc.y, 0.0, 1.0])
        _GUID += 1
        if beacon_id_mgr is not None:
            identity = beacon_id_mgr.get_temp_id(
                vm.vehicle.id, loc, frame_idx)
        else:
            identity = vm.vehicle.id
        info_rows.append([frame_idx, _GUID, identity])

    # b) ground-truth detections (non-managed actors) ----------------------
    for actor_info in gt_actors:
        if actor_info['carla_id'] in managed_ids:
            continue  # skip managed vehicles — they appear as beacons
        h = actor_info['hz'] * 2
        w = actor_info['hy'] * 2
        l = actor_info['hx'] * 2
        x, y, z = actor_info['x'], actor_info['y'], actor_info['z']
        yaw = np.radians(actor_info['yaw'])
        # KITTI: x=CARLA_x, y=CARLA_z, z=CARLA_y
        det_rows.append([h, w, l, x, z, y, yaw, 1.0])
        _GUID += 1
        # Oracle carries true CARLA actor IDs for all detections, so
        # oracle failures are purely physics-limited (latency), not
        # identity-pipeline artifacts.
        info_rows.append([frame_idx, _GUID, actor_info['carla_id']])

    return {'dets': np.asarray(det_rows, np.float32) if det_rows else np.empty((0, 8), np.float32),
            'info': np.asarray(info_rows, np.int64) if info_rows else np.empty((0, 3), np.int64)}


# ──────────────────────────────────────────────────────────────────────
#  Main edge-manager subclass
# ──────────────────────────────────────────────────────────────────────
class OracleEdge(_BaseEdgeManager):
    """
    Oracle front-end: CARLA ground-truth detections fed through the
    standard AB3DMOT + linear-predictor pipeline.
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        self.dt = world_dt

        # managers
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # AB3DMOT tracker — persistent across ticks (jitter-buffer arch)
        self.anchoring = cfg.get("anchoring", True)
        self.mot_cfg = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': True, 'thres': 2.0,
            'output_dir': None, 'min_hits': 3, 'max_age': 6, 'ego_com': None,
            'affi_pro': False, 'dataset': 'KITTI', 'det_name': 'pvrcnn',
            'anchoring': self.anchoring,
            'dup_x_max': cfg.get("dup_x_max", 8.0),
            'dup_y_max': cfg.get("dup_y_max", 2.0),
            'dup_size_ratio': cfg.get("dup_size_ratio", 2.5),
            'cull_consec_ticks': cfg.get("cull_consec_ticks", 3)})
        self.mot_category = 'Car'
        self.tracker = AB3DMOT(self.mot_cfg, self.mot_category)
        self.mot_tracker = self.tracker  # alias for evaluate()

        # Jitter buffer
        from ecav.core.application.edge.latency import JitterBuffer
        self._jitter_buffer: JitterBuffer = JitterBuffer(capacity=100)
        self._track_history: Deque[np.ndarray] = deque(maxlen=10)

        # GT snapshots indexed by source tick (for metrics evaluation)
        self._gt_snapshots: Dict[int, Dict] = {}
        self._excluded_snapshots: Dict[int, Dict] = {}

        self.tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self.track_to_carla: Dict[int, int] = {}

        # Tracking metrics accumulators
        self._prev_track_ids: set = set()
        self._track_to_gt_mapping: Dict[int, int] = {}
        self._prediction_history: Deque[Dict] = deque(maxlen=50)
        self._last_ade_fde: Dict[str, float] = {
            'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0
        }

        self.debug = EdgeMetrics(0)
        self._last_update_tick = -1
        self._latest_source_tick = None

        # Compute-contention cache
        self._prev_per_vehicle_preds: Dict[int, list] = {}
        self._prev_pickled_preds: bytes | None = None

        # Edge profiler
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get('intersection_id', f"oracle_{id(self)}"),
            history_size=2000,
            sample_gpu_utilization=True
        )

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

        # Vehicle types we consider valid detections (for GT filtering)
        self._valid_vehicle_types = {
            'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
            'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
            'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
            'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
            'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
            'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
            'patrol', 'mkz_2017', 'model3', 'wrangler', 'carlacola'}

        logger.info("OracleEdge initialized — GT front-end, anchoring=%s",
                     self.anchoring)

    # ------------------------------------------------------------------
    def start_edge(self):
        for vm in self.vehicle_manager_list:
            vm.agent._anchoring = self.anchoring

    # ------------------------------------------------------------------
    def _query_gt_actors(self, frame_idx: int):
        """Query CARLA world for all vehicles in detection range.

        Returns:
            gt_actors: list of dicts for the oracle detection collector
            carla_snapshot: dict keyed by actor.id for metrics
            excluded_snapshot: dict of non-detectable vehicle types
        """
        DETECTION_RANGE = 50.0
        managed_locs = [vm.vehicle.get_location()
                        for vm in self.vehicle_manager_list]

        gt_actors = []
        carla_snapshot = {}
        excluded_snapshot = {}

        try:
            actors = self.world.get_actors()
            for actor in actors:
                if 'vehicle' not in actor.type_id.lower():
                    continue
                loc = actor.get_location()
                if loc.z < -10.0:
                    continue

                # Range filter
                in_range = False
                for mloc in managed_locs:
                    if np.sqrt((loc.x - mloc.x)**2 + (loc.y - mloc.y)**2) <= DETECTION_RANGE:
                        in_range = True
                        break
                if not in_range:
                    continue

                vehicle_type = actor.type_id.split('.')[-1].lower()
                is_valid = any(vt in vehicle_type for vt in self._valid_vehicle_types)

                vel = actor.get_velocity()
                tf = actor.get_transform()
                bb = actor.bounding_box

                vehicle_data = {
                    'type': actor.type_id.split('.')[-1],
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'yaw': tf.rotation.yaw,
                    'vx': vel.x, 'vy': vel.y,
                    'speed': np.sqrt(vel.x**2 + vel.y**2),
                }

                if is_valid:
                    carla_snapshot[actor.id] = vehicle_data
                    gt_actors.append({
                        'carla_id': actor.id,
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'yaw': tf.rotation.yaw,
                        'hx': bb.extent.x, 'hy': bb.extent.y, 'hz': bb.extent.z,
                    })
                else:
                    excluded_snapshot[actor.id] = vehicle_data

        except Exception as e:
            logger.warning(f"Could not query CARLA GT actors: {e}")

        return gt_actors, carla_snapshot, excluded_snapshot

    # ------------------------------------------------------------------
    def update_information(self, frame_idx: int = 0):
        """Collect GT detections and beacons, push through jitter buffer."""
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx

        gt_actors, carla_snapshot, excluded_snapshot = \
            self._query_gt_actors(frame_idx)

        # Store GT snapshots for metrics
        self._gt_snapshots[frame_idx] = carla_snapshot
        self._excluded_snapshots[frame_idx] = excluded_snapshot
        for old in [k for k in self._gt_snapshots if frame_idx - k > 100]:
            del self._gt_snapshots[old]
            self._excluded_snapshots.pop(old, None)

        # Collect beacons (per-vehicle uplink loss check)
        beacons = {}
        vehicle_ids = [vm.vehicle.id for vm in self.vehicle_manager_list]
        mac_delivery = self.mac_model.attempt_tick(frame_idx, vehicle_ids)

        for vm in self.vehicle_manager_list:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue
            beacons[vid] = (vm.vehicle.get_location(),
                            vm.vehicle.bounding_box.extent)

        # Stamp with latency and push to jitter buffer
        arrival = self.latency_model.stamp(frame_idx)
        self._jitter_buffer.push(frame_idx, arrival,
                                 (gt_actors, beacons))

    # ------------------------------------------------------------------
    def run_step(self, tick: int):
        with self.profiler.profile_frame(tick) as frame:
            # ===== Feature collection =====================================
            with frame.time("feature_collection"):
                self.update_information(tick)
                num_agents = len(self.vehicle_manager_list) + len(self.rsu_manager_list)

            # ===== 1. Drain jitter buffer -> feed persistent tracker ======
            with frame.time("tracking"):
                new_frames = self._jitter_buffer.drain(tick)
                latest_dets = None
                latest_source_tick = None
                num_dets = 0

                for source_tick, (gt_actors, beacons) in new_frames:
                    if not beacons:
                        dets_all = {'dets': np.empty((0, 7)),
                                    'info': np.empty((0, 3))}
                    else:
                        dets_all = _collect_oracle_detections(
                            self, gt_actors, beacons,
                            frame_idx=source_tick,
                            beacon_id_mgr=self.beacon_id_mgr)
                        num_dets = max(num_dets, len(dets_all['dets']))

                    _t0 = time.perf_counter()
                    tracks, _ = self.tracker.track(dets_all, source_tick)
                    logger.debug("tracker.track() tick=%d src=%d dets=%d "
                                 "took %.1fms", tick, source_tick,
                                 len(dets_all['dets']),
                                 (time.perf_counter() - _t0) * 1000)
                    if tracks and len(tracks[0]) > 0:
                        self._track_history.append(tracks[0])
                    latest_dets = dets_all
                    latest_source_tick = source_tick

                # BSM rotation reconciliation
                for evt in self.beacon_id_mgr.pop_pending_rotations():
                    rec = self.beacon_id_mgr.get_record(evt.carla_id)
                    old_pos = evt.position
                    new_pos = (rec.last_position if rec is not None
                               else evt.position)
                    old_vel = (evt.velocity if evt.velocity is not None
                               else np.zeros(3, dtype=np.float32))
                    elapsed = max(tick - evt.tick, 1)
                    if self.beacon_id_mgr.reconcile_id_change(
                            evt.old_temp_id, evt.new_temp_id,
                            old_pos, new_pos, old_vel, elapsed):
                        BeaconIdManager.remap_tracker_identity(
                            self.tracker, evt.old_temp_id, evt.new_temp_id)
                    else:
                        logger.warning(
                            "BSM rotation reconcile FAILED carla=%d "
                            "%d->%d", evt.carla_id,
                            evt.old_temp_id, evt.new_temp_id)

                tracker_ms = (self.tracker.total_time
                              if hasattr(self.tracker, 'total_time') else 0.0)

            # No new data arrived this tick — skip prediction but still
            # advance vehicles so _step_count stays in sync with edge tick
            # and vehicle control is applied every tick.
            if not new_frames:
                frame.set_counts(num_agents=num_agents, num_detections=0,
                                 num_tracks=0, num_predictions=0)
                if not self.run_distributed:
                    for vm in self.vehicle_manager_list:
                        vm.update_info(tick)
                        vm.vehicle.apply_control(vm.run_step())
                        self._label_brake_attributions_gt(vm)
                        self._record_time_to_events(tick, vm)
                    for rsu in self.rsu_manager_list:
                        rsu.update_info(); rsu.run_step()
                return ecloud.EdgeObjects()

            # ===== 2. convert to trajectories & predict ===================
            with frame.time("detection"):
                self._latest_source_tick = latest_source_tick
                self._ab3d_history_to_trajs(self._track_history, horizon=10)
                num_tracks = len(self.tracked_trajectories)

                gt_snapshot = self._gt_snapshots.get(latest_source_tick)
                excluded_vehicles = self._excluded_snapshots.get(
                    latest_source_tick)

                # Compute detection metrics (oracle should be near-perfect)
                managed_positions = [vm.vehicle.get_location()
                                     for vm in self.vehicle_manager_list]
                det_metrics = self._compute_detection_metrics(
                    latest_dets, gt_snapshot, managed_positions,
                    excluded_vehicles=excluded_vehicles)

            frame.set_detection_metrics(
                true_positives=det_metrics['tp'],
                false_positives=det_metrics['fp'],
                false_negatives=det_metrics['fn']
            )

            # Ego-Uniqueness analysis
            latest_tracks = (self._track_history[-1]
                             if self._track_history else None)
            # Oracle: always translate temp_ids for uniqueness monitor
            # (single-source — ego identity is never ambiguous)
            if latest_tracks is not None and len(latest_tracks) > 0:
                latest_tracks = latest_tracks.copy()
                for i_trk in range(len(latest_tracks)):
                    if len(latest_tracks[i_trk]) > 8:
                        raw = int(latest_tracks[i_trk][8])
                        real = self.beacon_id_mgr.get_carla_id_for_temp(raw)
                        if real is not None:
                            latest_tracks[i_trk] = latest_tracks[i_trk].copy()
                            latest_tracks[i_trk][8] = real
            managed_ids = {vm.vehicle.id for vm in self.vehicle_manager_list}
            ego_poses = {}
            for vm in self.vehicle_manager_list:
                tf = vm.vehicle.get_transform()
                ego_poses[vm.vehicle.id] = (
                    tf.location.x, tf.location.y,
                    math.radians(tf.rotation.yaw))
            self.ego_monitor.update(tick, latest_tracks, gt_snapshot,
                                    managed_ids, ego_poses=ego_poses)
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

            with frame.time("prediction"):
                t0 = time.perf_counter()
                preds = self.lin_pred.generate_predicted_trajectories(
                    self.tracked_trajectories,
                    source_tick=latest_source_tick,
                    publish_tick=tick)
                predict_ms = (time.perf_counter() - t0) * 1e3
                num_predictions = len(preds)

                track_metrics = self._compute_tracking_metrics(
                    self._track_history, gt_snapshot, managed_positions)

            frame.set_tracking_metrics(
                id_switches=track_metrics['id_switches'],
                fragmentations=track_metrics['fragmentations'],
                mota=track_metrics['mota'],
                motp=track_metrics.get('motp', 0.0)
            )

            lag_steps = tick - latest_source_tick if latest_source_tick else 0
            frame.set_aoi_ticks(lag_steps)
            pred_metrics = self._evaluate_predictions(
                tick, preds, gt_snapshot, lag_steps)

            frame.set_prediction_metrics(
                error_1s_m=pred_metrics.get('ade_1s', 0.0),
                error_2s_m=pred_metrics.get('ade_2s', 0.0),
                error_3s_m=pred_metrics.get('ade_3s', 0.0),
                fde_m=pred_metrics.get('fde', 0.0),
                miss_rate=pred_metrics.get('miss_rate', 0.0)
            )

            total_ms = lag_steps * self.dt * 1000
            self.debug.update_edge(0, tracking_time=tracker_ms,
                                   prediction_time=predict_ms,
                                   latency=total_ms)

            # ===== 3. distribute predictions ===============================
            with frame.time("distribution"):
                serialized_preds = ecloud.EdgeObjects()
                pickled_fresh = None
                try:
                    pickled_fresh = pickle.dumps(preds)
                except Exception as e:
                    logging.warning("Error serializing predictions: %s", e)

                budget = self.compute_budget_ms
                per_veh = self.per_vehicle_compute_ms
                cumulative_ms = 0.0
                contended_ids: list[int] = []

                for index, vm in enumerate(self.vehicle_manager_list):
                    cumulative_ms += per_veh
                    is_contended = (budget is not None
                                    and cumulative_ms > budget)
                    if is_contended:
                        contended_ids.append(vm.vehicle.id)

                    # Oracle knows every track's identity: always
                    # suppress the ego's own track before publishing.
                    ego_preds = [
                        p for p in preds
                        if getattr(p.obstacle_trajectory.obstacle,
                                   'carla_id', -1) != vm.vehicle.id
                    ]
                    try:
                        ego_pickled = pickle.dumps(ego_preds)
                    except Exception:
                        ego_pickled = pickled_fresh

                    if random.random() * 100 < self.downlink_pl:
                        vm.agent.edge_predictions.clear()
                    elif is_contended:
                        stale = self._prev_per_vehicle_preds.get(index, [])
                        vm.agent.edge_predictions = list(stale)
                        stale_pickled = self._prev_pickled_preds
                        if stale_pickled is not None:
                            object_buffer = ecloud.ObjectBuffer(
                                vehicle_id=index,
                                pickled_edge_predictions=stale_pickled)
                        else:
                            object_buffer = ecloud.ObjectBuffer(
                                vehicle_id=index,
                                pickled_edge_predictions=ego_pickled)
                        serialized_preds.all_object_buffers.append(
                            object_buffer)
                    else:
                        object_buffer = ecloud.ObjectBuffer(
                            vehicle_id=index,
                            pickled_edge_predictions=ego_pickled)
                        serialized_preds.all_object_buffers.append(
                            object_buffer)
                        vm.agent.edge_predictions = ego_preds.copy()

                    self._prev_per_vehicle_preds[index] = ego_preds.copy()

                self._prev_pickled_preds = pickled_fresh

                if contended_ids:
                    overshoot = cumulative_ms - (budget if budget else 0.0)
                    logger.info(
                        "[CONTENTION] tick=%d budget=%.1fms used=%.1fms "
                        "contended %d vehicles: %s",
                        tick, budget, cumulative_ms,
                        len(contended_ids), contended_ids)
                    frame.set_contention_metrics(
                        vehicles_affected=len(contended_ids),
                        budget_exceeded_ms=max(overshoot, 0.0))
                else:
                    frame.set_contention_metrics(
                        vehicles_affected=0,
                        budget_exceeded_ms=0.0)

            # ===== 4. ego-consistency gate check at publish boundary ========
            self._check_ego_gate_violations(tick, preds)

            # ===== 5. advance vehicles ===================================
            if not self.run_distributed:
                for vm in self.vehicle_manager_list:
                    vm.update_info(tick)
                    vm.vehicle.apply_control(vm.run_step())
                    self._label_brake_attributions_gt(vm)
                    self._record_time_to_events(tick, vm)
                for rsu in self.rsu_manager_list:
                    rsu.update_info(); rsu.run_step()

            frame.set_counts(
                num_agents=num_agents,
                num_detections=num_dets,
                num_tracks=num_tracks,
                num_predictions=num_predictions
            )

            return serialized_preds

    # ------------------------------------------------------------------
    #  Trajectory conversion  (identical to late fusion)
    # ------------------------------------------------------------------
    def _ab3d_history_to_trajs(self, hist: Deque[np.ndarray], horizon: int = 10):
        updated: set[int] = set()
        for traj in self.tracked_trajectories.values():
            traj.trajectory.clear()
        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                tid = int(trk[7])
                cid_raw = int(trk[8])

                # Oracle is single-source: ego identity is unambiguous.
                # Always resolve beacon temp_ids regardless of anchoring
                # flag so the ego track is never mistaken for an obstacle.
                real_cid = self.beacon_id_mgr.get_carla_id_for_temp(cid_raw)
                cid = real_cid if real_cid is not None else cid_raw

                tf = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self.tracked_trajectories:
                    dummy = ObstacleVehicle(corners=np.zeros((8, 3)),
                                            o3d_bbx=None,
                                            track_id=tid,
                                            tick_id=0)
                    self.tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))
                traj = self.tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid
                self.track_to_carla[tid] = cid
                if len(trk) > 12:
                    kf_vx, kf_vy = float(trk[10]), float(trk[12])
                    traj.obstacle.kf_speed_mps = ((kf_vx**2 + kf_vy**2)**0.5) / self.dt
                    traj.obstacle.kf_vx = kf_vx
                    traj.obstacle.kf_vy = kf_vy

        for tid in list(self.tracked_trajectories):
            if tid not in updated:
                del self.tracked_trajectories[tid]

        # NOTE: Spatial self-identification removed from no-anchoring
        # baseline.  Without SBA the vehicle has no identity protocol
        # and cannot distinguish its own track from others.

    # ------------------------------------------------------------------
    #  Metrics  (identical to late fusion)
    # ------------------------------------------------------------------
    def _compute_detection_metrics(self, det_results: Dict, gt_vehicles: Dict,
                                   managed_positions: List,
                                   distance_threshold: float = 5.0,
                                   excluded_vehicles: Dict = None) -> Dict:
        if gt_vehicles is None:
            return {'tp': 0, 'fp': 0, 'fn': 0}

        dets = det_results.get('dets', np.empty((0, 7)))

        excluded_positions = []
        if excluded_vehicles:
            excluded_positions = [(v['x'], v['y']) for v in excluded_vehicles.values()]
        EXCLUSION_RADIUS = 15.0

        if len(dets) == 0:
            managed_xy = [(p.x, p.y) for p in managed_positions]
            fn_count = sum(1 for v in gt_vehicles.values()
                          if not any(np.sqrt((v['x'] - mx)**2 + (v['y'] - my)**2) < 3.0
                                     for mx, my in managed_xy))
            return {'tp': 0, 'fp': 0, 'fn': fn_count}

        managed_xy = [(p.x, p.y) for p in managed_positions]

        gt_list = [(v_id, v['x'], v['y']) for v_id, v in gt_vehicles.items()
                   if not any(np.sqrt((v['x'] - mx)**2 + (v['y'] - my)**2) < 3.0
                              for mx, my in managed_xy)]

        valid_det_indices = []
        for det_idx in range(len(dets)):
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 5]

            if any(np.sqrt((det_x - mx)**2 + (det_y - my)**2) < 3.0
                   for mx, my in managed_xy):
                continue

            near_excluded = False
            for ex, ey in excluded_positions:
                if np.sqrt((det_x - ex)**2 + (det_y - ey)**2) < EXCLUSION_RADIUS:
                    near_excluded = True
                    break
            if not near_excluded:
                valid_det_indices.append(det_idx)

        gt_matched, det_matched = set(), set()
        for det_idx in valid_det_indices:
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 5]
            best_dist, best_gt_idx = float('inf'), -1

            for gt_idx, (v_id, gx, gy) in enumerate(gt_list):
                if gt_idx in gt_matched:
                    continue
                dist = np.sqrt((det_x - gx)**2 + (det_y - gy)**2)
                if dist < best_dist and dist < distance_threshold:
                    best_dist, best_gt_idx = dist, gt_idx

            if best_gt_idx >= 0:
                gt_matched.add(best_gt_idx)
                det_matched.add(det_idx)

        tp = len(det_matched)
        fp = len(valid_det_indices) - tp
        fn = len(gt_list) - len(gt_matched)
        return {'tp': tp, 'fp': fp, 'fn': fn}

    def _compute_tracking_metrics(self, history: Deque, gt_vehicles: Dict,
                                  managed_positions: List) -> Dict:
        if gt_vehicles is None or not history:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        latest_tracks = history[-1] if history and len(history[-1]) > 0 else np.empty((0, 9))
        if len(latest_tracks) == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        managed_xy = [(p.x, p.y) for p in managed_positions]
        gt_list = [(v_id, v['x'], v['y']) for v_id, v in gt_vehicles.items()
                   if not any(np.sqrt((v['x'] - mx)**2 + (v['y'] - my)**2) < 3.0
                              for mx, my in managed_xy)]

        num_gt = len(gt_list)
        if num_gt == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 1.0, 'motp': 0.0}

        current_track_ids = set()
        new_mapping = {}
        id_switches = 0
        motp_errors = []

        for trk in latest_tracks:
            track_id = int(trk[7])
            tx, ty = trk[3], trk[5]
            current_track_ids.add(track_id)

            best_dist, best_gt_id = float('inf'), None
            for v_id, gx, gy in gt_list:
                dist = np.sqrt((tx - gx)**2 + (ty - gy)**2)
                if dist < best_dist and dist < 10.0:
                    best_dist, best_gt_id = dist, v_id

            if best_gt_id is not None:
                motp_errors.append(best_dist)
                new_mapping[track_id] = best_gt_id
                if track_id in self._track_to_gt_mapping:
                    if self._track_to_gt_mapping[track_id] != best_gt_id:
                        id_switches += 1

        fragmentations = len(self._prev_track_ids - current_track_ids)
        self._prev_track_ids = current_track_ids
        self._track_to_gt_mapping = new_mapping

        matched_gt = set(new_mapping.values())
        fn = num_gt - len(matched_gt)
        fp = len(current_track_ids) - len(new_mapping)
        mota = max(-1.0, min(1.0, 1.0 - (fn + fp + id_switches) / num_gt)) if num_gt > 0 else 1.0
        motp = np.mean(motp_errors) if motp_errors else 0.0

        return {
            'id_switches': id_switches,
            'fragmentations': fragmentations,
            'mota': mota,
            'motp': motp
        }

    def _evaluate_predictions(self, tick: int, predictions: List,
                              gt_snapshot: Dict, lag_steps: int) -> Dict:
        default = {'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0}
        if not predictions or gt_snapshot is None:
            return default

        snapshot = {'tick': tick, 'predictions': {}, 'actuals': dict(gt_snapshot)}
        for pred in predictions:
            try:
                track_id = pred.obstacle_trajectory.obstacle.track_id
                future_traj = pred.predicted_trajectory
                if future_traj and len(future_traj) > 0:
                    future_points = [(pred.transform.location.x, pred.transform.location.y)]
                    for fp in future_traj:
                        if hasattr(fp, 'location'):
                            future_points.append((fp.location.x, fp.location.y))
                        elif isinstance(fp, (list, tuple, np.ndarray)) and len(fp) >= 2:
                            future_points.append((fp[0], fp[1]))
                    snapshot['predictions'][track_id] = future_points
            except AttributeError:
                continue

        self._prediction_history.append(snapshot)
        self._compute_historical_ade_fde(tick, gt_snapshot)
        return self._last_ade_fde.copy()

    def _compute_historical_ade_fde(self, current_tick: int, current_vehicles: Dict):
        if len(self._prediction_history) < 2:
            return

        horizons = {'ade_1s': 20, 'ade_2s': 25, 'ade_3s': 25}
        all_errors, miss_count, total_preds = [], 0, 0
        miss_threshold = 2.0

        for metric_name, horizon in horizons.items():
            past_tick = current_tick - horizon
            past_snapshot = next(
                (s for s in self._prediction_history if s['tick'] == past_tick), None)
            if past_snapshot is None:
                continue

            errors = []
            for track_id, pred_traj in past_snapshot['predictions'].items():
                if len(pred_traj) <= horizon:
                    continue
                pred_x, pred_y = pred_traj[horizon]
                past_actuals = past_snapshot['actuals']

                if len(pred_traj) > 0:
                    track_x, track_y = pred_traj[0]
                    matched_id = min(
                        ((v_id, np.sqrt((v['x'] - track_x)**2 + (v['y'] - track_y)**2))
                         for v_id, v in past_actuals.items()),
                        key=lambda x: x[1], default=(None, float('inf'))
                    )
                    if matched_id[0] is None or matched_id[1] > 10.0:
                        continue
                    if matched_id[0] not in current_vehicles:
                        continue

                    actual = current_vehicles[matched_id[0]]
                    error = np.sqrt((pred_x - actual['x'])**2 + (pred_y - actual['y'])**2)
                    errors.append(error)
                    all_errors.append(error)
                    total_preds += 1
                    if error > miss_threshold:
                        miss_count += 1

            if errors:
                self._last_ade_fde[metric_name] = np.mean(errors)

        if all_errors:
            self._last_ade_fde['fde'] = all_errors[-1]
            self._last_ade_fde['miss_rate'] = miss_count / total_preds if total_preds > 0 else 0.0

    # ------------------------------------------------------------------
    #  Evaluation
    # ------------------------------------------------------------------
    def evaluate(self):
        fig, txt, metrics = self.profiler.get_evaluation_result()
        metrics['ego_uniqueness'] = self.ego_monitor.get_metrics()
        metrics.update(self._get_latency_component_stats())
        metrics.update(self._get_contract_metrics())
        self.mac_model.metrics.finalize()
        metrics['mac'] = self.mac_model.metrics.get_summary()
        if hasattr(self, 'mot_tracker') and self.mot_tracker is not None:
            metrics['birth_gate'] = {
                'birth_attempts_anon': self.mot_tracker.birth_attempts_anon,
                'birth_suppressed_by_gate': self.mot_tracker.birth_suppressed_by_gate,
                'births_anon_after_gate': self.mot_tracker.births_anon_after_gate,
                'anon_cull_count': self.mot_tracker.anon_cull_count,
            }
        return fig, txt, metrics
