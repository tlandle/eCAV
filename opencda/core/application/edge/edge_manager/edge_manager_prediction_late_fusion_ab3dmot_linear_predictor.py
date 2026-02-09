# -*- coding: utf-8 -*-
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
edge_manager.prediction
=======================

“Late-fusion” pipeline:

    ego/R SU detections  ─►  latency buffer  ─►  AB3DMOT (3-D MOT)
                               │
                               └►  track history (10 frames) ─►  linear
                                   constant-velocity predictor
                                   → 25 future steps

All helper code (IoU gates, distance checks, trajectory conversion, etc.)
is directly ported from the old monolithic *EdgeManager* so behaviour
stays unchanged.

Author : Tyler Landle <tlandle3@gatech.edu>
License: TDG-Attribution-NonCommercial-NoDistrib
"""
from __future__ import annotations

import math, random, time, logging, pickle
from collections import deque, defaultdict
from typing import Dict, List, Deque

import numpy as np
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

from .edge_manager_base import _BaseEdgeManager, logger


# ──────────────────────────────────────────────────────────────────────
#  Helpers (unchanged logic)
# ──────────────────────────────────────────────────────────────────────
_GUID = 0
_MIN_EDGE, _MIN_VOLUME = 0.60, 1.0      # sliver reject

def _xyz(loc: carla.Location) -> np.ndarray:
    return np.asarray([loc.x, loc.y, loc.z], np.float32)

def _is_sliver(h,w,l):
    return (min(h,w,l) < _MIN_EDGE) or (h*w*l < _MIN_VOLUME)

def _aabb_iou_2d(box_xy, box_wh, ego_xy, ego_wh):
    dx, dy = np.abs(box_xy - ego_xy)
    ix = max(0., .5*(box_wh[0]+ego_wh[0]) - dx)
    iy = max(0., .5*(box_wh[1]+ego_wh[1]) - dy)
    inter = ix*iy
    if inter==0: return 0.
    union = box_wh[0]*box_wh[1] + ego_wh[0]*ego_wh[1] - inter
    return inter/union

def _box_to_transform(box):
    """Convert AB3DMOT box format [h, w, l, x, y, z, yaw] to picklable Transform."""
    from opencda.opencda_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    h, w, l, x, y, z, yaw = box  # AB3DMOT format: [h, w, l, x, y, z, yaw]
    loc = _Loc(x=float(x), y=float(y), z=float(z))
    rot = _Rot(yaw=np.degrees(float(yaw)))
    return _Tf(location=loc, rotation=rot)

def _collect_ab3d_detections(edge,
                             objects: Dict[str,List],
                             beacons: Dict[int,tuple],
                             frame_idx: int):
    """
    Build the dict that AB3DMOT.track() expects.
    *beacons* = {carla_id : (loc, extent)}
    """
    global _GUID
    det_rows, info_rows, beacons_xyz = [], [], []

    # a) beacons (one per managed vehicle) ----------------------------
    for vm in edge.vehicle_manager_list:
        loc, ext = beacons[vm.vehicle.id]
        h,w,l = ext.z*2, ext.y*2, ext.x*2
        det_rows.append([h,w,l, loc.x,loc.y,loc.z, 0.0])
        _GUID += 1
        info_rows.append([frame_idx, _GUID, vm.vehicle.id])
        beacons_xyz.append(_xyz(loc))
        ego_h,ego_w,ego_l = h,w,l
    ego_xy = beacons_xyz[0][:2];  ego_wh = np.array([ego_w, ego_l],np.float32)

    # b) sensor detections -------------------------------------------
    for obj in objects.get("vehicles", []):
        bbx = obj.bounding_box.extent
        h,w,l = bbx.z*2, bbx.y*2, bbx.x*2
        if _is_sliver(h,w,l): continue
        # Use obj.location (world coords), NOT obj.bounding_box.location
        # which is a local offset after set_vehicle() in ground-truth mode
        loc = obj.location
        if np.linalg.norm(_xyz(loc) - beacons_xyz[0]) < 0.7*max(ego_wh): continue
        if _aabb_iou_2d(_xyz(loc)[:2],[w,l], ego_xy, ego_wh) > .25:      continue
        det_rows.append([h,w,l, loc.x,loc.y,loc.z, 0.0])
        _GUID += 1
        info_rows.append([frame_idx, _GUID, -1])

    return {'dets': np.asarray(det_rows,np.float32),
            'info': np.asarray(info_rows,np.int64)}


# ──────────────────────────────────────────────────────────────────────
#  Main edge-manager subclass
# ──────────────────────────────────────────────────────────────────────
class PredictionLateFusionEdge(_BaseEdgeManager):
    """
    Back-end for *mode: PREDICTION* (late-fusion with AB3DMOT).
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        # latency / loss ------------------------------------------------
        self.lat_ms   = cfg.get("latency", 0)*1000.0
        self.jit_ms   = cfg.get("jitter_std", 0)*1000.0
        self.lat_dist = cfg.get("latency_distribution", "normal")
        self.uplink_loss   = cfg.get("uplink_packet_loss_pct", 0)
        self.downlink_loss = cfg.get("downlink_packet_loss_pct", 0)
        self.dt = world_dt

        # managers ------------------------------------------------------
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # AB3DMOT tracker template (fresh instance for every replay)
        self.mot_cfg = edict({
            'vis':False,'save_path':None,'use_3d_iou':False,'thres':2.0,
            'output_dir':None,'min_hits':3,'max_age':2,'ego_com':None,
            'affi_pro':False,'dataset':'KITTI','det_name':'deprecated'})
        self.mot_category = 'Car'

        # history buffers ----------------------------------------------
        self.objects_deque : Deque[Dict] = deque(maxlen=100)
        self.beacon_history= defaultdict(lambda: deque(maxlen=100))
        self.tracked_trajectories : Dict[int,ObstacleTrajectory] = {}
        self.track_to_carla : Dict[int,int] = {}
        self.carla_snapshot_history: Deque[Dict] = deque(maxlen=100)  # GT snapshots

        # Tracking metrics accumulators
        self._prev_track_ids: set = set()
        self._track_to_gt_mapping: Dict[int, int] = {}
        self._prediction_history: Deque[Dict] = deque(maxlen=50)
        self._last_ade_fde: Dict[str, float] = {
            'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0
        }

        self.debug = EdgeMetrics(0)
        self._last_update_tick = -1  # Guard against double-update

        # Edge profiler for capacity planning
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get('intersection_id', f"late_fusion_{id(self)}"),
            history_size=2000,
            sample_gpu_utilization=True
        )

    # ------------------------------------------------------------------
    def start_edge(self):   # nothing special to pre-compute
        pass

    # ------------------------------------------------------------------
    def update_information(self, frame_idx:int=0):
        """Collect latest detections from every VM and RSU into history."""
        # Guard against double-update in same tick
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx

        objects: Dict[str,List] = {}

        # Capture CARLA ground truth snapshot for evaluation
        # Filter to only include vehicles within detection range of managed vehicles
        DETECTION_RANGE = 50.0  # meters - typical LiDAR range for late fusion
        carla_snapshot = {}
        excluded_vehicles_snapshot = {}  # Track firetrucks, ambulances, etc.

        # Vehicle types the model was trained to detect (standard passenger vehicles)
        # Exclude firetrucks, ambulances, police cars, etc. that aren't in training data
        VALID_VEHICLE_TYPES = {'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
                               'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
                               'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
                               'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
                               'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
                               'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
                               'patrol', 'mkz_2017', 'model3', 'wrangler', 'carlacola'}

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

                    # Filter out vehicle types the model wasn't trained on
                    vehicle_type = actor.type_id.split('.')[-1].lower()
                    is_valid_type = any(vt in vehicle_type for vt in VALID_VEHICLE_TYPES)

                    # Filter by detection range from any managed vehicle
                    in_range = False
                    for mloc in managed_locs:
                        dist = np.sqrt((loc.x - mloc.x)**2 + (loc.y - mloc.y)**2)
                        if dist <= DETECTION_RANGE:
                            in_range = True
                            break

                    if not in_range:
                        continue

                    vehicle_data = {
                        'type': actor.type_id.split('.')[-1],
                        'x': loc.x, 'y': loc.y, 'z': loc.z,
                        'yaw': actor.get_transform().rotation.yaw,
                        'vx': actor.get_velocity().x, 'vy': actor.get_velocity().y,
                        'speed': np.sqrt(actor.get_velocity().x**2 + actor.get_velocity().y**2)
                    }

                    if is_valid_type:
                        carla_snapshot[actor.id] = vehicle_data
                    else:
                        # Track excluded vehicles (firetrucks, etc.) so we can filter detections near them
                        excluded_vehicles_snapshot[actor.id] = vehicle_data

        except Exception as e:
            logger.warning(f"Could not capture CARLA snapshot: {e}")
        self.carla_snapshot_history.appendleft(carla_snapshot)

        # Store excluded vehicles for detection filtering
        if not hasattr(self, 'excluded_vehicles_history'):
            self.excluded_vehicles_history = deque(maxlen=200)
        self.excluded_vehicles_history.appendleft(excluded_vehicles_snapshot)

        for vm in self.vehicle_manager_list:
            # uplink loss simulation
            if random.random()*100 < self.uplink_loss: continue
            self._dict_extend(objects, vm.agent.objects)
            # beacon history (for delayed pose)
            self.beacon_history[vm.vehicle.id].appendleft((
                frame_idx,
                vm.vehicle.get_location(),
                vm.vehicle.bounding_box.extent))

        for rsu in self.rsu_manager_list:
            self._dict_extend(objects, rsu.objects)

        self.objects_deque.appendleft(objects)

    # ------------------------------------------------------------------
    def run_step(self, tick:int):
        with self.profiler.profile_frame(tick) as frame:
            # ===== Feature collection =====================================
            with frame.time("feature_collection"):
                self.update_information(tick)
                num_agents = len(self.vehicle_manager_list) + len(self.rsu_manager_list)

            # ===== latency handling =======================================
            total_ms  = self._sample_latency_ms()
            lag_steps = int(round(total_ms / (self.dt*1000)))
            if lag_steps >= len(self.objects_deque):
                frame.set_counts(num_agents=num_agents, num_detections=0,
                                 num_tracks=0, num_predictions=0)
                return ecloud.EdgeObjects()
            as_of = tick - lag_steps

            # ===== 1. replay detection history into fresh AB3DMOT =========
            with frame.time("tracking"):
                tracker   = AB3DMOT(self.mot_cfg, self.mot_category)
                history   : Deque[np.ndarray] = deque(maxlen=10)
                oldest    = max(0, tick-(len(self.objects_deque)-1))
                num_dets = 0
                for step in range(oldest, as_of+1):
                    idx = tick - step
                    if idx >= len(self.objects_deque):
                        dets_all = {'dets':np.empty((0,7)), 'info':np.empty((0,3))}
                    else:
                        snapshot = self.objects_deque[idx]
                        # build beacons dict for this historical frame
                        beacons = {}
                        for vm in self.vehicle_manager_list:
                            hist = self.beacon_history[vm.vehicle.id]
                            if idx < len(hist):
                                _,loc,ext = hist[idx];  beacons[vm.vehicle.id]=(loc,ext)
                        if not beacons:
                            dets_all = {'dets':np.empty((0,7)), 'info':np.empty((0,3))}
                        else:
                            dets_all = _collect_ab3d_detections(
                                self, snapshot, beacons, frame_idx=step)
                            num_dets = max(num_dets, len(dets_all['dets']))
                    tracks, _ = tracker.track(dets_all, step)
                    if tracks and len(tracks[0])>0: history.append(tracks[0])

                tracker_ms = tracker.total_time if hasattr(tracker,'total_time') \
                             else 0.0

            # ===== 2. convert to trajectories & predict ===================
            with frame.time("detection"):
                self._ab3d_history_to_trajs(history, horizon=10)
                num_tracks = len(self.tracked_trajectories)

                # Get delayed GT snapshot for evaluation
                gt_snapshot = None
                if lag_steps < len(self.carla_snapshot_history):
                    gt_snapshot = self.carla_snapshot_history[lag_steps]

                # Get excluded vehicles snapshot for filtering detections near firetrucks
                excluded_vehicles = None
                if hasattr(self, 'excluded_vehicles_history') and lag_steps < len(self.excluded_vehicles_history):
                    excluded_vehicles = self.excluded_vehicles_history[lag_steps]

                # Compute detection metrics
                managed_positions = [vm.vehicle.get_location() for vm in self.vehicle_manager_list]
                det_metrics = self._compute_detection_metrics(dets_all, gt_snapshot, managed_positions, excluded_vehicles=excluded_vehicles)

            # Set detection metrics
            frame.set_detection_metrics(
                true_positives=det_metrics['tp'],
                false_positives=det_metrics['fp'],
                false_negatives=det_metrics['fn']
            )

            with frame.time("prediction"):
                t0 = time.perf_counter()
                preds = self.lin_pred.generate_predicted_trajectories(
                            self.tracked_trajectories)
                predict_ms = (time.perf_counter()-t0)*1e3
                num_predictions = len(preds)

                # Compute tracking metrics
                track_metrics = self._compute_tracking_metrics(history, gt_snapshot, managed_positions)

            # Set tracking metrics
            frame.set_tracking_metrics(
                id_switches=track_metrics['id_switches'],
                fragmentations=track_metrics['fragmentations'],
                mota=track_metrics['mota'],
                motp=track_metrics.get('motp', 0.0)
            )

            # Compute prediction metrics
            pred_metrics = self._evaluate_predictions(tick, preds, gt_snapshot, lag_steps)

            # Set prediction metrics
            frame.set_prediction_metrics(
                error_1s_m=pred_metrics.get('ade_1s', 0.0),
                error_2s_m=pred_metrics.get('ade_2s', 0.0),
                error_3s_m=pred_metrics.get('ade_3s', 0.0),
                fde_m=pred_metrics.get('fde', 0.0),
                miss_rate=pred_metrics.get('miss_rate', 0.0)
            )

            self.debug.update_edge(0, tracking_time=tracker_ms,
                                      prediction_time=predict_ms,
                                      latency=total_ms)

            # ===== 3. distribute predictions =============================
            with frame.time("distribution"):
                serialized_preds = ecloud.EdgeObjects()
                pickled_edge_predictions = None
                try:
                    pickled_edge_predictions = pickle.dumps(preds)
                except Exception as e:
                    logging.warning("Error serializing predictions: %s", e)

                for index, vm in enumerate(self.vehicle_manager_list):
                    if random.random()*100 < self.downlink_loss:
                        vm.agent.edge_predictions.clear()
                    else:
                        object_buffer = ecloud.ObjectBuffer(
                            vehicle_id=index,
                            pickled_edge_predictions=pickled_edge_predictions)
                        serialized_preds.all_object_buffers.append(object_buffer)
                        vm.agent.edge_predictions = preds.copy()

            # ===== 4. advance vehicles ===================================
            if not self.run_distributed:
                for vm in self.vehicle_manager_list:
                    vm.update_info(tick)
                    vm.vehicle.apply_control(vm.run_step())
                for rsu in self.rsu_manager_list:
                    rsu.update_info();  rsu.run_step()

            # Set profiler counts
            frame.set_counts(
                num_agents=num_agents,
                num_detections=num_dets,
                num_tracks=num_tracks,
                num_predictions=num_predictions
            )

            return serialized_preds

    # ------------------------------------------------------------------
    #  Trajectory conversion (unchanged from legacy code)
    # ------------------------------------------------------------------
    def _ab3d_history_to_trajs(self, hist:Deque[np.ndarray], horizon:int=10):
        updated: set[int] = set()
        for frame in hist:
            if frame is None or len(frame)==0: continue
            for trk in frame:
                tid = int(trk[7]);  cid = int(trk[8])
                tf  = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self.tracked_trajectories:
                    dummy = ObstacleVehicle(corners=np.zeros((8,3)),
                                            o3d_bbx=None,
                                            track_id=tid,
                                            tick_id=0)
                    self.tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))
                traj = self.tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location  = tf.location
                traj.obstacle.carla_id  = cid
                self.track_to_carla[tid]= cid

        # prune stale
        for tid in list(self.tracked_trajectories):
            if tid not in updated:
                del self.tracked_trajectories[tid]

    # ------------------------------------------------------------------
    #  Metrics computation
    # ------------------------------------------------------------------
    def _compute_detection_metrics(self, det_results: Dict, gt_vehicles: Dict,
                                    managed_positions: List, distance_threshold: float = 5.0,
                                    excluded_vehicles: Dict = None) -> Dict:
        """Compute detection metrics (TP, FP, FN) against ground truth.

        Args:
            excluded_vehicles: Dict of vehicles excluded from GT (firetrucks, etc.)
                               Detections near these are ignored (not counted as FP).
        """
        if gt_vehicles is None:
            return {'tp': 0, 'fp': 0, 'fn': 0}

        dets = det_results.get('dets', np.empty((0, 7)))

        # Build excluded vehicle positions for filtering
        excluded_positions = []
        if excluded_vehicles:
            excluded_positions = [(v['x'], v['y']) for v in excluded_vehicles.values()]
        EXCLUSION_RADIUS = 15.0  # meters - filter detections near excluded vehicles

        if len(dets) == 0:
            # Count non-managed GT vehicles as FN
            managed_xy = [(p.x, p.y) for p in managed_positions]
            fn_count = sum(1 for v in gt_vehicles.values()
                          if not any(np.sqrt((v['x']-mx)**2 + (v['y']-my)**2) < 3.0
                                    for mx, my in managed_xy))
            return {'tp': 0, 'fp': 0, 'fn': fn_count}

        managed_xy = [(p.x, p.y) for p in managed_positions]

        # Build GT list (excluding managed vehicles)
        gt_list = [(v_id, v['x'], v['y']) for v_id, v in gt_vehicles.items()
                   if not any(np.sqrt((v['x']-mx)**2 + (v['y']-my)**2) < 3.0
                             for mx, my in managed_xy)]

        # Filter detections near excluded vehicles (firetrucks, etc.)
        # These are ignored - not counted as TP or FP
        valid_det_indices = []
        excluded_det_count = 0
        for det_idx in range(len(dets)):
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]
            near_excluded = False
            for ex, ey in excluded_positions:
                if np.sqrt((det_x - ex)**2 + (det_y - ey)**2) < EXCLUSION_RADIUS:
                    near_excluded = True
                    excluded_det_count += 1
                    break
            if not near_excluded:
                valid_det_indices.append(det_idx)

        if excluded_det_count > 0:
            print(f"[LateFusion] Filtered {excluded_det_count} detections near excluded vehicles (firetrucks, etc.)")

        # Greedy matching with filtered detections
        # AB3DMOT detection format: [h, w, l, x, y, z, yaw]
        gt_matched, det_matched = set(), set()
        for det_idx in valid_det_indices:
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]  # x, y from detection (indices 3, 4)
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
        fp = len(valid_det_indices) - tp  # Only count valid (non-excluded) detections
        fn = len(gt_list) - len(gt_matched)

        # Debug output for every frame
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        print(f"\n[DET DEBUG] GT={len(gt_list)}, Dets={len(valid_det_indices)} (filtered {excluded_det_count}), TP={tp}, FP={fp}, FN={fn}, P={precision:.2f}, R={recall:.2f}")

        for i, (v_id, gx, gy) in enumerate(gt_list):
            v_data = gt_vehicles.get(v_id, {})
            v_type = v_data.get('type', 'unknown')
            matched = i in gt_matched
            status = "MATCHED" if matched else "MISSED"
            print(f"[DET DEBUG]   GT[{i}]: {v_type} id={v_id} at ({gx:.1f}, {gy:.1f}) - {status}")

        for det_idx in valid_det_indices:
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 4]
            matched = det_idx in det_matched
            status = "TP" if matched else "FP"
            print(f"[DET DEBUG]   DET[{det_idx}]: ({det_x:.1f}, {det_y:.1f}) - {status}")

        return {'tp': tp, 'fp': fp, 'fn': fn}

    def _compute_tracking_metrics(self, history: Deque, gt_vehicles: Dict,
                                   managed_positions: List) -> Dict:
        """Compute tracking metrics: ID switches, MOTA, MOTP."""
        if gt_vehicles is None or not history:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        # Get latest tracks
        latest_tracks = history[0] if history and len(history[0]) > 0 else np.empty((0, 9))
        if len(latest_tracks) == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        managed_xy = [(p.x, p.y) for p in managed_positions]
        gt_list = [(v_id, v['x'], v['y']) for v_id, v in gt_vehicles.items()
                   if not any(np.sqrt((v['x']-mx)**2 + (v['y']-my)**2) < 3.0
                             for mx, my in managed_xy)]

        num_gt = len(gt_list)
        if num_gt == 0:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 1.0, 'motp': 0.0}

        # Match tracks to GT
        current_track_ids = set()
        new_mapping = {}
        id_switches = 0
        motp_errors = []

        # AB3DMOT track format: [h, w, l, x, y, z, yaw, track_id, carla_id, guid, ...]
        for trk in latest_tracks:
            track_id = int(trk[7])
            tx, ty = trk[3], trk[4]  # x, y are at indices 3, 4
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

        # Fragmentations
        fragmentations = len(self._prev_track_ids - current_track_ids)

        # Update state
        self._prev_track_ids = current_track_ids
        self._track_to_gt_mapping = new_mapping

        # MOTA
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
        """Evaluate predictions and return metrics."""
        default = {'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0}
        if not predictions or gt_snapshot is None:
            return default

        # Store predictions for later ADE/FDE evaluation
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

        # Compute historical ADE/FDE
        self._compute_historical_ade_fde(tick, gt_snapshot)
        return self._last_ade_fde.copy()

    def _compute_historical_ade_fde(self, current_tick: int, current_vehicles: Dict):
        """Compute ADE/FDE from past predictions."""
        if len(self._prediction_history) < 2:
            return

        horizons = {'ade_1s': 20, 'ade_2s': 25, 'ade_3s': 25}
        all_errors, miss_count, total_preds = [], 0, 0
        miss_threshold = 2.0

        for metric_name, horizon in horizons.items():
            past_tick = current_tick - horizon
            past_snapshot = next((s for s in self._prediction_history if s['tick'] == past_tick), None)
            if past_snapshot is None:
                continue

            errors = []
            for track_id, pred_traj in past_snapshot['predictions'].items():
                if len(pred_traj) <= horizon:
                    continue
                pred_x, pred_y = pred_traj[horizon]
                past_actuals = past_snapshot['actuals']

                # Match track to GT
                if len(pred_traj) > 0:
                    track_x, track_y = pred_traj[0]
                    matched_id = min(
                        ((v_id, np.sqrt((v['x']-track_x)**2 + (v['y']-track_y)**2))
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
    #  Utilities
    # ------------------------------------------------------------------
    def _dict_extend(self, dest:dict, src:dict):
        for k,v in src.items(): dest.setdefault(k, []).extend(v)

    def _sample_latency_ms(self)->float:
        if self.lat_dist=="normal":
            return max(0., random.gauss(self.lat_ms, self.jit_ms))
        elif self.lat_dist=="lognormal":
            mean = math.log(self.lat_ms) if self.lat_ms>0 else 0
            return np.random.lognormal(mean, 0.5)
        else:
            return self.lat_ms

    # ------------------------------------------------------------------
    #  Evaluation
    # ------------------------------------------------------------------
    def evaluate(self):
        """
        Return evaluation results for EvaluationManager.

        Returns:
            Tuple[figure, perform_txt, metrics]
        """
        return self.profiler.get_evaluation_result()

# ---------------------------------------------------------------------
# alias expected by edge_manager/__init__.py
LateFusionEdge = PredictionLateFusionEdge      # exported name
