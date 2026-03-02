# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
VIPS-style temporal alignment edge manager.

Implements the core idea from VIPS (MobiCom'22): compensate for
communication latency by extrapolating stale detections forward in time
using their last-known velocity.  Key differences from late fusion:

1. **No anchoring** — ``anchoring: false`` in AB3DMOT config.
   No ``BeaconIdManager``.
2. **Beacons are anonymous** — vehicle beacons carry ``carla_id = -1``
   (same as sensor detections).  The tracker must associate by spatial
   proximity / IoU only.
3. **Time-rectification** — when draining the jitter buffer, stale
   detections are extrapolated forward:
       corrected_pos = pos + kf_velocity * (current_tick - source_tick) * dt
   This is the core VIPS compensation mechanism.

The test question: does spatiotemporal alignment alone prevent
self-ghosting?  If ego's old beacon arrives late and velocity-
extrapolated position doesn't match the current beacon, AB3DMOT may
birth a duplicate track → self-ghosting.

Pipeline:
    ego/RSU detections  -->  latency buffer  -->  time-rectification
                                                    |
                              AB3DMOT (3-D MOT) <---+
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
from scipy.optimize import linear_sum_assignment
import carla

from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT

import ecloud_pb2 as ecloud

from opencda.core.prediction.linear_predictor_manager import \
    LinearPredictorManager
from opencda.core.sensing.tracking.obstacle_trajectory import \
    ObstacleTrajectory
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from opencda.core.application.edge.edge_metrics import EdgeMetrics
from opencda.core.application.edge.edge_profiler import EdgeProfiler
from opencda.core.application.edge.ego_uniqueness_monitor import EgoUniquenessMonitor

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
    from opencda.opencda_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    h, w, l, x, ky, kz, yaw = box  # ky=CARLA_z, kz=CARLA_y
    loc = _Loc(x=float(x), y=float(kz), z=float(ky))
    rot = _Rot(yaw=np.degrees(float(yaw)))
    return _Tf(location=loc, rotation=rot)


def _collect_anonymous_detections(edge,
                                  objects: Dict[str, List],
                                  beacons: Dict[int, tuple],
                                  frame_idx: int):
    """
    Build the dict that AB3DMOT.track() expects.

    VIPS-temporal difference from late fusion:
    - Beacons carry ``carla_id = -1`` (anonymous) instead of the real
      vehicle ID.  The tracker associates purely by spatial proximity.
    - No BeaconIdManager — no anchoring protocol at all.
    """
    global _GUID
    det_rows, info_rows = [], []

    # a) beacons — treated as anonymous sensor detections ----------------
    #    KITTI camera convention: KITTI_x=CARLA_x, KITTI_y=CARLA_z, KITTI_z=CARLA_y
    for vm in edge.vehicle_manager_list:
        if vm.vehicle.id not in beacons:
            continue
        loc, ext = beacons[vm.vehicle.id]
        h, w, l = ext.z * 2, ext.y * 2, ext.x * 2
        det_rows.append([h, w, l, loc.x, loc.z, loc.y, 0.0, 1.0])
        _GUID += 1
        info_rows.append([frame_idx, _GUID, -1])  # anonymous

    # b) sensor detections -----------------------------------------------
    for obj in objects.get("vehicles", []):
        bbx = obj.bounding_box.extent
        h, w, l = bbx.z * 2, bbx.y * 2, bbx.x * 2
        loc = obj.location
        det_rows.append([h, w, l, loc.x, loc.z, loc.y, 0.0, 0.5])
        _GUID += 1
        info_rows.append([frame_idx, _GUID, -1])  # anonymous

    return {'dets': np.asarray(det_rows, np.float32) if det_rows else np.empty((0, 8), np.float32),
            'info': np.asarray(info_rows, np.int64) if info_rows else np.empty((0, 3), np.int64)}


def _time_rectify_detections(dets_dict: dict,
                             source_tick: int,
                             current_tick: int,
                             dt: float,
                             tracker: AB3DMOT):
    """
    Extrapolate stale detections forward using the tracker's last-known
    KF velocity for each matched track.

    This is the core VIPS idea: compensate for communication delay by
    predicting where objects *are now* given where they *were* and their
    velocity.

    For detections that don't yet have a matching track (new objects),
    no correction is applied — there is no velocity estimate yet.

    Parameters
    ----------
    dets_dict : dict  {'dets': np.ndarray (N,8), 'info': np.ndarray (N,3)}
    source_tick : int  tick when the detection was captured
    current_tick : int  current simulation tick
    dt : float  seconds per tick
    tracker : AB3DMOT  the persistent tracker instance

    Returns
    -------
    dict : corrected copy of dets_dict
    """
    staleness = current_tick - source_tick
    if staleness <= 0 or len(dets_dict['dets']) == 0:
        return dets_dict

    dets = dets_dict['dets'].copy()

    # Get existing tracks from tracker for velocity lookup
    # Track state: [h,w,l, x(KITTI), y(KITTI), z(KITTI), yaw, tid, cid, ?, dx, dy, dz]
    track_states = {}
    if hasattr(tracker, 'trackers'):
        for t in tracker.trackers:
            if not hasattr(t, 'kf'):
                continue
            # KF state: 10x1 column vector [x, y, z, theta, l, w, h, dx, dy, dz]
            x_state = t.kf.x
            # In KITTI coords: x[0]=CARLA_x, x[2]=CARLA_y (ground plane)
            # Velocity: x[7]=dx(CARLA vx), x[9]=dz(CARLA vy)
            kitti_x = float(x_state[0, 0])
            kitti_z = float(x_state[2, 0])
            kf_dx = float(x_state[7, 0])
            kf_dz = float(x_state[9, 0])
            track_states[t.id] = {
                'kitti_x': kitti_x, 'kitti_z': kitti_z,
                'kf_dx': kf_dx, 'kf_dz': kf_dz,
            }

    if not track_states:
        return dets_dict

    # For each detection, find the nearest existing track and extrapolate
    # using its KF velocity
    for i in range(len(dets)):
        det_x = dets[i, 3]   # KITTI x = CARLA x
        det_z = dets[i, 5]   # KITTI z = CARLA y

        best_tid, best_dist = None, float('inf')
        for tid, ts in track_states.items():
            d = np.sqrt((det_x - ts['kitti_x'])**2 +
                        (det_z - ts['kitti_z'])**2)
            if d < best_dist and d < 10.0:  # association radius
                best_dist = d
                best_tid = tid

        if best_tid is not None:
            ts = track_states[best_tid]
            # Extrapolate: pos += velocity * staleness_ticks
            # kf_dx, kf_dz are in m/tick (KITTI x, z = CARLA x, y)
            dets[i, 3] += ts['kf_dx'] * staleness  # KITTI x
            dets[i, 5] += ts['kf_dz'] * staleness  # KITTI z

    return {'dets': dets, 'info': dets_dict['info']}


# ──────────────────────────────────────────────────────────────────────
#  Main edge-manager subclass
# ──────────────────────────────────────────────────────────────────────
class VIPSTemporalEdge(_BaseEdgeManager):
    """
    VIPS-style temporal alignment baseline.

    Late-fusion detections + anonymous beacons + time-rectification.
    Two modes controlled by ``anchoring`` config:

    * ``anchoring: false`` — faithful MobiCom'22 VIPS baseline.
      No identity protocol, no ego-consistency suppression.
    * ``anchoring: true`` — VIPS + our ego-uniqueness invariant.
      Per-ego ego-consistency suppression at publish boundary using
      gate G(e): distance + speed similarity.  Shows added value of
      the identity invariant on top of temporal alignment.

    In both modes the AB3DMOT tracker operates without beacon
    anchoring (all beacons are anonymous).
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        self.dt = world_dt

        # managers
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # AB3DMOT tracker — NO beacon anchoring in the tracker itself.
        # When anchoring=True, we add per-ego ego-consistency suppression
        # at publish boundary (VIPS + our invariant).  When False, this is
        # the faithful MobiCom'22 VIPS baseline with no identity protocol.
        self.anchoring = cfg.get("anchoring", False)
        self.mot_cfg = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': True, 'thres': 2.0,
            'output_dir': None, 'min_hits': 3, 'max_age': 6, 'ego_com': None,
            'affi_pro': False, 'dataset': 'KITTI', 'det_name': 'pvrcnn',
            'anchoring': False})
        self.mot_category = 'Car'
        self.tracker = AB3DMOT(self.mot_cfg, self.mot_category)

        # Jitter buffer
        from opencda.core.application.edge.latency import JitterBuffer
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

        # Compute-contention cache
        self._prev_per_vehicle_preds: Dict[int, list] = {}
        self._prev_pickled_preds: bytes | None = None

        # Edge profiler
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get('intersection_id', f"vips_temporal_{id(self)}"),
            history_size=2000,
            sample_gpu_utilization=True
        )

        # Ego-Uniqueness monitor
        self.ego_monitor = EgoUniquenessMonitor()

        # Spatial self-ID failure tracking
        self._self_id_failures = 0
        self._tick_count = 0
        self._latest_source_tick = None

        # Valid vehicle types (for GT snapshot filtering)
        self._valid_vehicle_types = {
            'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
            'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
            'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
            'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
            'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
            'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
            'patrol', 'mkz_2017', 'model3', 'wrangler', 'carlacola'}

        logger.info("VIPSTemporalEdge initialized — anonymous beacons + "
                     "time rectification, anchoring=%s", self.anchoring)

    # ------------------------------------------------------------------
    def start_edge(self):
        for vm in self.vehicle_manager_list:
            vm.agent._anchoring = self.anchoring

    # ------------------------------------------------------------------
    def update_information(self, frame_idx: int = 0):
        """Collect detections from VMs and RSUs, push through jitter buffer."""
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx

        objects: Dict[str, List] = {}

        # Capture CARLA GT snapshot for metrics
        DETECTION_RANGE = 50.0
        carla_snapshot = {}
        excluded_vehicles_snapshot = {}
        managed_locs = [vm.vehicle.get_location()
                        for vm in self.vehicle_manager_list]

        try:
            actors = self.world.get_actors()
            for actor in actors:
                if 'vehicle' not in actor.type_id.lower():
                    continue
                loc = actor.get_location()
                if loc.z < -10.0:
                    continue

                vehicle_type = actor.type_id.split('.')[-1].lower()
                is_valid = any(vt in vehicle_type
                               for vt in self._valid_vehicle_types)

                in_range = False
                for mloc in managed_locs:
                    if np.sqrt((loc.x - mloc.x)**2 + (loc.y - mloc.y)**2) <= DETECTION_RANGE:
                        in_range = True
                        break
                if not in_range:
                    continue

                vel = actor.get_velocity()
                vehicle_data = {
                    'type': actor.type_id.split('.')[-1],
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'yaw': actor.get_transform().rotation.yaw,
                    'vx': vel.x, 'vy': vel.y,
                    'speed': np.sqrt(vel.x**2 + vel.y**2),
                }
                if is_valid:
                    carla_snapshot[actor.id] = vehicle_data
                else:
                    excluded_vehicles_snapshot[actor.id] = vehicle_data
        except Exception as e:
            logger.warning(f"Could not capture CARLA snapshot: {e}")

        self._gt_snapshots[frame_idx] = carla_snapshot
        self._excluded_snapshots[frame_idx] = excluded_vehicles_snapshot
        for old in [k for k in self._gt_snapshots if frame_idx - k > 100]:
            del self._gt_snapshots[old]
            self._excluded_snapshots.pop(old, None)

        # Collect detections + anonymous beacons
        beacons = {}
        vehicle_ids = [vm.vehicle.id for vm in self.vehicle_manager_list]
        mac_delivery = self.mac_model.attempt_tick(frame_idx, vehicle_ids)

        for vm in self.vehicle_manager_list:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue
            self._dict_extend(objects, vm.agent.objects)
            beacons[vid] = (vm.vehicle.get_location(),
                            vm.vehicle.bounding_box.extent)

        for rsu in self.rsu_manager_list:
            self._dict_extend(objects, rsu.objects)

        # Stamp with per-packet arrival time and push to jitter buffer
        arrival = self.latency_model.stamp(frame_idx)
        self._jitter_buffer.push(frame_idx, arrival, (objects, beacons))

    # ------------------------------------------------------------------
    def run_step(self, tick: int):
        with self.profiler.profile_frame(tick) as frame:
            # ===== Feature collection =====================================
            with frame.time("feature_collection"):
                self.update_information(tick)
                num_agents = len(self.vehicle_manager_list) + len(self.rsu_manager_list)

            # ===== 1. Drain jitter buffer -> track (no pre-rectification) ==
            # The real VIPS algorithm tracks first, then extrapolates
            # tracked outputs for cross-source graph matching.  Pre-
            # rectifying detections before the tracker corrupts the KF's
            # velocity estimate (detection positions are at current_tick
            # but the tracker predicts only to source_tick → systematic
            # velocity inflation).  We let the KF handle staleness
            # natively via its temporal model, like late fusion.
            with frame.time("tracking"):
                new_frames = self._jitter_buffer.drain(tick)
                latest_dets = None
                latest_source_tick = None
                num_dets = 0

                for source_tick, (objects, beacons) in new_frames:
                    if not beacons:
                        dets_all = {'dets': np.empty((0, 7)),
                                    'info': np.empty((0, 3))}
                    else:
                        # Collect as anonymous (no anchoring IDs)
                        dets_all = _collect_anonymous_detections(
                            self, objects, beacons,
                            frame_idx=source_tick)

                        num_dets = max(num_dets, len(dets_all['dets']))

                    _t0 = time.perf_counter()
                    tracks, _ = self.tracker.track(dets_all, source_tick)
                    logger.debug("tracker.track() tick=%d src=%d dets=%d "
                                 "took %.1fms (VIPS temporal)",
                                 tick, source_tick,
                                 len(dets_all['dets']),
                                 (time.perf_counter() - _t0) * 1000)
                    if tracks and len(tracks[0]) > 0:
                        self._track_history.append(tracks[0])
                    latest_dets = dets_all
                    latest_source_tick = source_tick

                tracker_ms = (self.tracker.total_time
                              if hasattr(self.tracker, 'total_time') else 0.0)

            # No new data arrived this tick — return empty
            if not new_frames:
                frame.set_counts(num_agents=num_agents, num_detections=0,
                                 num_tracks=0, num_predictions=0)
                return ecloud.EdgeObjects()

            # ===== 2. convert to trajectories & predict ===================
            with frame.time("detection"):
                self._latest_source_tick = latest_source_tick
                self._ab3d_history_to_trajs(self._track_history, horizon=10)
                num_tracks = len(self.tracked_trajectories)

                gt_snapshot = self._gt_snapshots.get(latest_source_tick)
                excluded_vehicles = self._excluded_snapshots.get(
                    latest_source_tick)

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

            # Ego-Uniqueness analysis (no temp_id translation needed —
            # VIPS temporal uses carla_id=-1 for all, tracker assigns its own)
            latest_tracks = (self._track_history[-1]
                             if self._track_history else None)
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

            # ===== Per-ego ego-consistency suppression (publish boundary) ==
            # When anchoring=True (VIPS + our invariant), suppress anonymous
            # tracks satisfying gate G(e): dist + speed similarity per ego.
            # When anchoring=False (pure VIPS baseline), no suppression.
            #
            # IMPORTANT: Never mutate obs.carla_id — preds is shared across
            # all per-ego iterations.  Use ego-local `cid` override only.
            ego_suppress_sets = {}  # {vehicle_id: set of pred indices}
            _SWAP_MARGIN = 0.5   # metres — avoid borderline flips
            _FOOTPRINT_L = 1.2   # longitudinal margin beyond half-length
            _FOOTPRINT_W = 1.0   # lateral margin beyond half-width
            if self.anchoring:
                managed_ids_supp = {vm.vehicle.id
                                    for vm in self.vehicle_manager_list}
                # Time-consistent snapshot: use get_transform() once per
                # ego so position and yaw come from the same pose.
                managed_tfs = {vm.vehicle.id: vm.vehicle.get_transform()
                               for vm in self.vehicle_manager_list}
                managed_locs = {vid: tf.location
                                for vid, tf in managed_tfs.items()}
                for vm in self.vehicle_manager_list:
                    ego_tf = managed_tfs[vm.vehicle.id]
                    ego_loc = ego_tf.location
                    ego_vel = vm.vehicle.get_velocity()
                    ego_speed = (ego_vel.x**2 + ego_vel.y**2)**0.5

                    # Ego footprint for stationary-track suppression
                    yaw_rad = math.radians(ego_tf.rotation.yaw)
                    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
                    ext = vm.vehicle.bounding_box.extent
                    half_L = ext.x + _FOOTPRINT_L
                    half_W = ext.y + _FOOTPRINT_W

                    suppress_idx = set()
                    # Instrumentation counters
                    n_footprint = 0
                    n_speed_gate = 0
                    n_swap_detect = 0
                    n_stat_cand = 0

                    for i, pred in enumerate(preds):
                        obs = pred.obstacle_trajectory.obstacle
                        cid = getattr(obs, 'carla_id', -1)
                        if cid in managed_ids_supp:
                            # Swap detection: track carries another ego's
                            # ID but is closer to *us* than the claimed ego.
                            ploc = obs.location
                            d_to_us = ((ploc.x - ego_loc.x)**2 +
                                       (ploc.y - ego_loc.y)**2)**0.5
                            if d_to_us >= self._self_id_radius:
                                continue
                            claimed_loc = managed_locs.get(cid)
                            if claimed_loc is not None:
                                d_to_claimed = (
                                    (ploc.x - claimed_loc.x)**2 +
                                    (ploc.y - claimed_loc.y)**2)**0.5
                                if d_to_us + _SWAP_MARGIN < d_to_claimed:
                                    cid = -1  # ego-local only
                                    n_swap_detect += 1
                                    logger.debug(
                                        "[SWAP-DETECT] tick=%d ego=%d "
                                        "pred_idx=%d: d_to_us=%.2f "
                                        "d_to_claimed=%.2f margin=%.1f",
                                        tick, vm.vehicle.id, i,
                                        d_to_us, d_to_claimed,
                                        _SWAP_MARGIN)
                                else:
                                    continue  # legit other-ego track
                            else:
                                continue  # can't verify — pass through
                        else:
                            ploc = obs.location
                            d_to_us = ((ploc.x - ego_loc.x)**2 +
                                       (ploc.y - ego_loc.y)**2)**0.5
                        if d_to_us >= self._self_id_radius:
                            continue
                        obs_speed = getattr(obs, 'kf_speed_mps', 0.0)

                        # (a) Ego-footprint overlap for stationary tracks
                        if obs_speed < 1.0:
                            n_stat_cand += 1
                            wx = ploc.x - ego_loc.x
                            wy = ploc.y - ego_loc.y
                            dx_ego = wx * cos_y + wy * sin_y
                            dy_ego = -wx * sin_y + wy * cos_y
                            if abs(dx_ego) <= half_L and \
                                    abs(dy_ego) <= half_W:
                                n_footprint += 1
                                suppress_idx.add(i)
                                logger.debug(
                                    "[FOOTPRINT] tick=%d ego=%d "
                                    "pred_idx=%d dx=%.2f dy=%.2f "
                                    "half_L=%.2f half_W=%.2f "
                                    "obs_spd=%.1f d=%.2f",
                                    tick, vm.vehicle.id, i,
                                    dx_ego, dy_ego, half_L, half_W,
                                    obs_speed, d_to_us)
                                continue
                        # (b) Speed-gate for moving tracks
                        if abs(obs_speed - ego_speed) >= \
                                self._self_id_speed_gate:
                            continue
                        n_speed_gate += 1
                        suppress_idx.add(i)
                        logger.debug(
                            "[SPEED-GATE] tick=%d ego=%d "
                            "pred_idx=%d obs_spd=%.2f ego_spd=%.2f "
                            "gate=%.1f d=%.2f",
                            tick, vm.vehicle.id, i,
                            obs_speed, ego_speed,
                            self._self_id_speed_gate, d_to_us)
                    ego_suppress_sets[vm.vehicle.id] = suppress_idx
                    if suppress_idx or n_swap_detect:
                        logger.debug(
                            "[EGO-SUPPRESS] tick=%d ego=%d "
                            "suppressed=%d (footprint=%d speed_gate=%d) "
                            "swaps_detected=%d stat_candidates=%d "
                            "r_pos=%.1f r_v=%.1f",
                            tick, vm.vehicle.id, len(suppress_idx),
                            n_footprint, n_speed_gate, n_swap_detect,
                            n_stat_cand, self._self_id_radius,
                            self._self_id_speed_gate)

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

                    # Per-ego filtered predictions (publish boundary)
                    suppress_set = ego_suppress_sets.get(
                        vm.vehicle.id, set())
                    if suppress_set:
                        ego_preds = [p for i, p in enumerate(preds)
                                     if i not in suppress_set]
                        try:
                            ego_pickled = pickle.dumps(ego_preds)
                        except Exception:
                            ego_pickled = pickled_fresh
                    else:
                        ego_preds = preds
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
    #  Trajectory conversion (VIPS spatial self-identification)
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
                cid = int(trk[8])  # will be -1 (anonymous) for all

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

        # VIPS spatial self-identification — Hungarian assignment.
        # Uses min-cost matching across all egos and tracks to prevent
        # one ego from stealing another's track in multi-ego scenarios.
        self._tick_count += 1
        src_tick = getattr(self, '_latest_source_tick', None)
        gt_snap = self._gt_snapshots.get(src_tick) if src_tick else None

        egos = []  # [(vm, ego_x, ego_y)]
        for vm in self.vehicle_manager_list:
            ego_loc_now = vm.vehicle.get_location()
            if gt_snap and vm.vehicle.id in gt_snap:
                ex = gt_snap[vm.vehicle.id]['x']
                ey = gt_snap[vm.vehicle.id]['y']
            else:
                ex = ego_loc_now.x
                ey = ego_loc_now.y
            egos.append((vm, ex, ey))

        tids = list(self.tracked_trajectories.keys())
        n_egos = len(egos)
        n_tracks = len(tids)

        if n_egos > 0 and n_tracks > 0:
            INF = 1e9
            cost = np.full((n_egos, n_tracks), INF, dtype=np.float64)
            for i, (vm, ex, ey) in enumerate(egos):
                for j, tid in enumerate(tids):
                    loc = self.tracked_trajectories[tid].obstacle.location
                    d = np.sqrt((loc.x - ex)**2 + (loc.y - ey)**2)
                    if d < self._self_id_radius:
                        cost[i, j] = d

            row_ind, col_ind = linear_sum_assignment(cost)

            assigned_egos = set()
            for ri, ci in zip(row_ind, col_ind):
                if cost[ri, ci] >= INF:
                    continue
                vm, ex, ey = egos[ri]
                tid = tids[ci]
                self.tracked_trajectories[tid].obstacle.carla_id = \
                    vm.vehicle.id
                self.track_to_carla[tid] = vm.vehicle.id
                assigned_egos.add(ri)
                logger.debug(
                    "VIPS self-id: vm %d -> track %d "
                    "(dist=%.2fm, src_tick=%s)",
                    vm.vehicle.id, tid, cost[ri, ci], src_tick)

            for i, (vm, ex, ey) in enumerate(egos):
                if i not in assigned_egos:
                    self._self_id_failures += 1
                    logger.warning(
                        "[SELF-ID FAIL] vm %d: no track within "
                        "radius=%.1fm (src_tick=%s, n_tracks=%d)",
                        vm.vehicle.id, self._self_id_radius,
                        src_tick, n_tracks)

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
        metrics['spatial_self_id_failures'] = self._self_id_failures
        metrics['spatial_self_id_failure_rate'] = (
            self._self_id_failures / max(1, self._tick_count))
        metrics.update(self._get_latency_component_stats())
        metrics.update(self._get_contract_metrics())
        self.mac_model.metrics.finalize()
        metrics['mac'] = self.mac_model.metrics.get_summary()
        return fig, txt, metrics
