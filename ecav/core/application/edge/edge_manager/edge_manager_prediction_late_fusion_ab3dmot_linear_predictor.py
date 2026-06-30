# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Late-fusion edge manager with AB3DMOT tracking and linear prediction.

Pipeline:
    ego/RSU detections  ->  latency buffer  ->  AB3DMOT (3-D MOT)
                              |
                              +->  track history (10 frames)  ->  linear
                                   constant-velocity predictor
                                   -> 25 future steps
"""
from __future__ import annotations

import math, random, time, logging, pickle, os
from collections import deque
from typing import Any, Dict, List, Deque

import numpy as np
from scipy.optimize import linear_sum_assignment
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
from ecav.core.tracking.ab3dmot_format import (
    stack_rows,
    vehicles_to_ab3dmot_rows,
)
from ecav.core.application.edge.latency.ns3_lut_sampler import (
    get_default as _get_lut_sampler,
)

from .edge_manager_base import _BaseEdgeManager, logger

# Late-fusion uplink payload per source: 96 B self-beacon + object list.
# Each 3D box ~48 B serialized; the paper reports mean ~7 boxes, <10 worst case.
# This is ~430 B, far below WorldFusion's ~16.9 KB feature tensors, so the ns-3
# Uu radio latency lands at the light-payload end of the LUT.
_LF_BEACON_BYTES = 96
_LF_PER_BOX_BYTES = 48


# ──────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────
_GUID = 0


def _next_edge_guid() -> int:
    global _GUID
    _GUID += 1
    return _GUID

def _xyz(loc: carla.Location) -> np.ndarray:
    return np.asarray([loc.x, loc.y, loc.z], np.float32)

def _box_to_transform(box):
    """Convert AB3DMOT box format [h, w, l, x, y, z, yaw] to picklable Transform.

    Coordinates are in KITTI camera convention inside the tracker:
        KITTI x = CARLA x,  KITTI y = CARLA z (height),  KITTI z = CARLA y
    Swap y↔z back to CARLA world coordinates on output.
    """
    from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
    h, w, l, x, ky, kz, yaw = box  # ky=CARLA_z, kz=CARLA_y
    loc = _Loc(x=float(x), y=float(kz), z=float(ky))
    rot = _Rot(yaw=np.degrees(float(yaw)))
    return _Tf(location=loc, rotation=rot)

def _collect_ab3d_detections(edge,
                             objects: Dict[str,List],
                             beacons: Dict[int,tuple],
                             frame_idx: int,
                             beacon_id_mgr: BeaconIdManager = None):
    """
    Build the dict that AB3DMOT.track() expects.
    *beacons* = {carla_id : (loc, extent)}

    When *beacon_id_mgr* is provided the CID column in the info array
    carries a **temporary ID** (rotated per J2945 policy) instead of
    the raw ``carla_id``.  This lets the anchoring protocol inside
    ``matching.py`` work with pseudonymous identities.
    """
    det_rows, info_rows = [], []

    # a) beacons (one per managed vehicle) ----------------------------
    #    KITTI camera convention: x=right, y=down(height), z=front
    #    Map: KITTI_x=CARLA_x, KITTI_y=CARLA_z, KITTI_z=CARLA_y
    for vm in edge.vehicle_manager_list:
        # Infra-only architectures have no vehicle uplink, so a managed
        # vehicle may have no beacon this frame. Skip the beacon row for it;
        # the RSU sees it (or not) as an anonymous sensor detection below.
        if vm.vehicle.id not in beacons:
            continue
        loc, ext = beacons[vm.vehicle.id]
        h,w,l = ext.z*2, ext.y*2, ext.x*2
        # Heading-align the exclusion zone: yaw=0 hardcoded puts the 2m
        # lateral gate on the longitudinal axis, letting depth-error ghosts
        # through.  CARLA yaw (°, CW from +x) maps directly to KITTI theta.
        beacon_theta = math.radians(vm.vehicle.get_transform().rotation.yaw)
        det_rows.append([h,w,l, loc.x,loc.z,loc.y, beacon_theta, 1.0])
        # Use temp_id instead of raw carla_id when manager is present
        if beacon_id_mgr is not None:
            identity = beacon_id_mgr.get_temp_id(
                vm.vehicle.id, loc, frame_idx)
        else:
            identity = vm.vehicle.id
        info_rows.append([frame_idx, _next_edge_guid(), identity])

    # b) sensor detections -- shared library, see ab3dmot_format.py
    sensor_dets, sensor_info = vehicles_to_ab3dmot_rows(
        objects.get("vehicles", []),
        frame_idx=frame_idx,
        guid_provider=_next_edge_guid,
    )
    det_rows.extend(sensor_dets)
    info_rows.extend(sensor_info)

    bundle = stack_rows(det_rows, info_rows)
    dets_arr = bundle['dets']
    info_arr = bundle['info']
    # Box-quality normalization (class-conditioned size prior).
    # Roadside camera-lidar fusion fits an axis-aligned box to the visible
    # (near-face) LiDAR points, so a fast crossing vehicle yields degenerate
    # extents (sliver widths, or giant road/wall artifacts) even when the
    # CENTER is accurate. Clamp anonymous vehicle-detection extents
    # [h, w, l] = cols 0,1,2 to plausible car bounds so an unstable extent is
    # not treated as association/identity evidence and the downstream collision
    # footprint is a real vehicle, not a 0.4 m sliver. Beacon rows (cid>0,
    # known size) are left untouched. Centers/yaw are not modified.
    if len(dets_arr) > 0:
        _CAR_H, _CAR_W, _CAR_L = (1.2, 2.0), (1.6, 2.4), (3.5, 5.5)
        for _i in range(len(dets_arr)):
            if int(info_arr[_i, 2]) <= 0:  # anonymous sensor detection
                dets_arr[_i, 0] = min(max(float(dets_arr[_i, 0]), _CAR_H[0]), _CAR_H[1])
                dets_arr[_i, 1] = min(max(float(dets_arr[_i, 1]), _CAR_W[0]), _CAR_W[1])
                dets_arr[_i, 2] = min(max(float(dets_arr[_i, 2]), _CAR_L[0]), _CAR_L[1])
    # Log raw detections with source tag
    # Beacon detections (from vehicle_manager_list) have cid > 0
    # Sensor detections come from objects["vehicles"] with _det_source tag
    if len(dets_arr) > 0:
        obj_list = objects.get("vehicles", [])
        n_beacons = len(beacons)
        for idx, (d, inf) in enumerate(zip(dets_arr, info_arr)):
            if inf[2] > 0:
                src = "beacon"
            else:
                obj_idx = idx - n_beacons
                if 0 <= obj_idx < len(obj_list):
                    src = getattr(obj_list[obj_idx], '_det_source', '?')
                else:
                    src = '?'
            logger.info("[DET] tick=%d src=%s x=%.1f y=%.1f conf=%.2f cid=%d",
                        frame_idx, src, d[3], d[5], d[7], inf[2])
    return {'dets': dets_arr, 'info': info_arr}


def _cross_source_nms(dets_dict, cdist_thresh=3.0):
    """Deduplicate anonymous sensor detections from multiple sources.

    Beacons (info CID > 0) are kept unconditionally.
    Among anonymous detections (CID == -1), if two detections are within
    *cdist_thresh* meters (CARLA x,y center distance), keep the one with
    higher confidence (det column 7).

    This prevents the tracker from receiving N copies of the same
    real-world object when N egos all observe it, keeping association
    cost O(objects²) instead of O((N*objects)²).
    """
    dets = dets_dict['dets']
    info = dets_dict['info']
    if len(dets) == 0:
        return dets_dict

    # Split into beacons (identified) and anonymous sensor detections
    cids = info[:, 2] if info.ndim == 2 and info.shape[1] > 2 else np.full(len(info), -1)
    beacon_mask = cids > 0
    anon_mask = ~beacon_mask

    if anon_mask.sum() <= 1:
        return dets_dict

    anon_dets = dets[anon_mask]
    anon_info = info[anon_mask]

    # CARLA x = det col 3, CARLA y = det col 5 (KITTI z = CARLA y)
    centers = anon_dets[:, [3, 5]]

    # Greedy NMS by center distance: keep highest-confidence first
    scores = anon_dets[:, 7] if anon_dets.shape[1] > 7 else np.ones(len(anon_dets))
    order = np.argsort(-scores)
    keep = []
    suppressed = np.zeros(len(anon_dets), dtype=bool)

    for i in order:
        if suppressed[i]:
            continue
        keep.append(i)
        dists = np.linalg.norm(centers[i] - centers, axis=1)
        suppressed |= dists < cdist_thresh

    anon_keep = np.array(keep)
    kept_dets = np.concatenate([dets[beacon_mask], anon_dets[anon_keep]], axis=0)
    kept_info = np.concatenate([info[beacon_mask], anon_info[anon_keep]], axis=0)

    return {'dets': kept_dets, 'info': kept_info}


# ──────────────────────────────────────────────────────────────────────
#  Main edge-manager subclass
# ──────────────────────────────────────────────────────────────────────
class PredictionLateFusionEdge(_BaseEdgeManager):
    """
    Back-end for *mode: PREDICTION* (late-fusion with AB3DMOT).
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, is_proxy=False, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, is_proxy=is_proxy, **kw)

        self.dt = world_dt

        # ns-3 Uu radio LUT for the uplink (sources -> edge) delay. When on,
        # per-source UL latency is sampled from the ns-3 5G-NR LUT (N- and
        # payload-aware) instead of the SEE-V2X HybridModel, so radio delay
        # grows with sender count and message size. Backhaul (RSU->edge wired)
        # stays in the HybridModel/base. Gated by cfg use_ns3_lut (default on).
        self._use_ns3_lut = bool(cfg.get("use_ns3_lut", False)) if not is_proxy else False
        self._lut_sampler = _get_lut_sampler() if self._use_ns3_lut else None
        self._sim_dt_ms = world_dt * 1000.0

        if is_proxy:
            self.profiler = None
            self.ego_monitor = None
            self.tracker = None
            self._jitter_buffer = None
            self._track_history = None
            self._last_update_tick = -1
            return

        # managers ------------------------------------------------------
        pred_type = cfg.get('predictor_type', 'linear').lower()
        if pred_type == 'smart':
            from ecav.core.prediction.smart_predictor_manager import (
                SMARTPredictorManager3D)
            smart_cfg = cfg['smart_predictor']
            try:
                self.predictor = SMARTPredictorManager3D(
                    checkpoint_path=smart_cfg['checkpoint'],
                    map_cache_path=smart_cfg.get('map_cache'),
                    device=smart_cfg.get('device', 'cuda'),
                    num_output_steps=25)
                print(f"[LateFusion Edge] Using SMART predictor")
            except FileNotFoundError:
                print(f"[LateFusion Edge] SMART checkpoint not found "
                      f"({smart_cfg['checkpoint']}); falling back to linear predictor")
                self.predictor = LinearPredictorManager(num_future_steps=25)
        else:
            self.predictor = LinearPredictorManager(num_future_steps=25)
            print(f"[LateFusion Edge] Using linear predictor")
        self.lin_pred = self.predictor  # alias for existing call sites

        # AB3DMOT tracker — persistent across ticks (jitter-buffer arch)
        self.anchoring = cfg.get("anchoring", True)
        # LF-guarded: withhold coasting (stale) tracks from published collision
        # predictions. Default off (LF-basic); on => LF-guarded. Orthogonal to SBA.
        self.stale_track_suppression = bool(cfg.get('stale_track_suppression', False))
        self.stale_track_n = int(cfg.get('stale_track_n', 2))
        # Env override (for sweeps that cannot edit the YAML per arm):
        # STALE_TRACK_SUPPRESSION=1/0 forces on/off; STALE_TRACK_N sets the gate.
        _env_sts = os.environ.get('STALE_TRACK_SUPPRESSION')
        if _env_sts is not None:
            self.stale_track_suppression = _env_sts.strip() not in ('', '0', 'false', 'False')
        self.stale_track_n = int(os.environ.get('STALE_TRACK_N', self.stale_track_n))
        self.mot_cfg = edict({
            'vis':False,'save_path':None,'use_3d_iou':True,'thres':2.0,
            'output_dir':None,'min_hits':3,'max_age':6,'ego_com':None,
            'affi_pro':False,'dataset':'KITTI','det_name':'pvrcnn',
            'anchoring': self.anchoring})
        self.mot_category = 'Car'
        self.tracker = AB3DMOT(self.mot_cfg, self.mot_category)

        # Jitter buffer: detections are stamped with per-packet arrival
        # times and drained in source-tick order (Apollo/Autoware pattern).
        from ecav.core.application.edge.latency import JitterBuffer
        self._jitter_buffer: JitterBuffer = JitterBuffer(capacity=100)
        # Track history stores the last N AB3DMOT output frames.
        # _ab3d_history_to_trajs rebuilds trajectories from this history.
        # SMART requires 22 frames (11 at 10Hz subsampled from 20Hz).
        # 30 gives headroom for track maturity convergence.
        self._track_history: Deque[np.ndarray] = deque(maxlen=30)

        # GT snapshots indexed by source tick (for metrics evaluation)
        self._gt_snapshots: Dict[int, Dict] = {}
        self._excluded_snapshots: Dict[int, Dict] = {}

        self.tracked_trajectories : Dict[int,ObstacleTrajectory] = {}
        self.track_to_carla : Dict[int,int] = {}

        # Tracking metrics accumulators
        self._prev_track_ids: set = set()
        self._track_to_gt_mapping: Dict[int, int] = {}
        # Per-track provenance history: track_id -> deque of nearest-GT actor
        # ids over recent ticks. Used to classify a brake-triggering track as
        # self_ghost (consistently ego) vs track_merge (ego AND non-ego appear)
        # vs external_stale (consistently a non-ego actor), instead of the
        # nearest-ego-NOW heuristic which mislabels stale/merged tracks.
        self._track_provenance: Dict[int, Deque] = {}
        self._prediction_history: Deque[Dict] = deque(maxlen=50)
        self._last_ade_fde: Dict[str, float] = {
            'ade_1s': 0.0, 'ade_2s': 0.0, 'ade_3s': 0.0, 'fde': 0.0, 'miss_rate': 0.0
        }

        self.debug = EdgeMetrics(0)
        self._last_update_tick = -1  # Guard against double-update
        self._latest_source_tick = None
        # Previous beacon locations for position-based ego_speed (Fix 2)
        self._prev_beacon_locs: Dict[int, Any] = {}
        # Short rolling history of ego (x,y) per vehicle, for swept-path
        # self-echo suppression. ~1s at edge_dt cadence. A stale ego-echo sits
        # on this path; a VRU / occluder / stopped vehicle does not, so this
        # discriminates self-echoes from real obstacles without spatial-only
        # proximity suppression (which would blind the ego to nearby VRUs).
        self._ego_path_hist: Dict[int, Deque] = {}

        # Compute-contention: cache of previous tick's per-vehicle predictions
        self._prev_per_vehicle_preds: Dict[int, list] = {}  # vehicle index -> preds
        self._prev_pickled_preds: bytes | None = None

        # Edge profiler for capacity planning
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get('intersection_id', f"late_fusion_{id(self)}"),
            history_size=2000,
            sample_gpu_utilization=True
        )

        # Ego-Uniqueness monitor
        self.ego_monitor = EgoUniquenessMonitor()

        # Spatial self-ID failure tracking
        self._self_id_failures = 0
        self._tick_count = 0

        # BSM J2945-inspired temporary ID rotation manager ----------------
        self.beacon_id_mgr = BeaconIdManager(
            rotation_interval_ticks=cfg.get(
                "beacon_id_rotation_interval", 200),
            rotation_distance_m=cfg.get(
                "beacon_id_rotation_distance", 100.0),
            world_dt=world_dt,
        )

    # ------------------------------------------------------------------
    def start_edge(self):
        if self.is_proxy:
            return
        for vm in self.vehicle_manager_list:
            vm.agent._anchoring = self.anchoring

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

        # GT snapshots stored by source tick (always immediately available)
        self._gt_snapshots[frame_idx] = carla_snapshot
        self._excluded_snapshots[frame_idx] = excluded_vehicles_snapshot
        # Prune old entries (keep last 100 ticks)
        for old in [k for k in self._gt_snapshots if frame_idx - k > 100]:
            del self._gt_snapshots[old]
            self._excluded_snapshots.pop(old, None)

        # Collect detections + beacons together as one packet
        beacons = {}
        vehicle_ids = [vm.vehicle.id for vm in self.vehicle_manager_list]
        mac_delivery = self.mac_model.attempt_tick(frame_idx, vehicle_ids)

        for vm in self.vehicle_manager_list:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue  # uplink loss — per-vehicle
            # Tag each detection with its source for diagnostics
            for obj in vm.agent.objects.get('vehicles', []):
                obj._det_source = f"ego_{vid}"
            self._dict_extend(objects, vm.agent.objects)
            beacons[vid] = (vm.vehicle.get_location(),
                            vm.vehicle.bounding_box.extent)

        for rsu in self.rsu_manager_list:
            rsu_id = getattr(rsu, 'id', 'rsu')
            for obj in rsu.objects.get('vehicles', []):
                obj._det_source = f"rsu_{rsu_id}"
            self._dict_extend(objects, rsu.objects)

        # Stamp with per-packet arrival time and push to jitter buffer
        if self._lut_sampler is not None:
            # ns-3 Uu uplink: one packet per vehicle source (RSU is wired
            # backhaul, not over Uu, so excluded from radio contention).
            # "wait for slowest source" = max over per-source samples. Payload
            # per source = 96 B beacon + object boxes it contributed.
            n_cav = max(1, len(self.vehicle_manager_list))
            n_boxes = len(objects.get('vehicles', []))
            ul_bytes = _LF_BEACON_BYTES + n_boxes * _LF_PER_BOX_BYTES
            ul_samples = [
                self._lut_sampler.sample_ms(n_cav, ul_bytes, 'ul')
                for _ in range(n_cav)
            ]
            ul_ms = max(ul_samples) if ul_samples else 0.0
            ul_ticks = int(math.ceil(ul_ms / self._sim_dt_ms))
            arrival = frame_idx + ul_ticks
        else:
            arrival = self.latency_model.stamp(frame_idx)
        self._jitter_buffer.push(frame_idx, arrival, (objects, beacons))

    # ------------------------------------------------------------------
    def _collect_detections_for_frame(self, objects, beacons, source_tick):
        """Build the AB3DMOT detection dict for one drained jitter-buffer frame.

        Default = object-level perception (RSU/vehicle detections) with
        cross-source NMS. A detector-mode subclass overrides this to source
        detections from a ground-truth front-end (DETECTOR=oracle), which lets
        the architecture comparison run without the perception-noise confound.
        """
        has_sensor_dets = bool(objects.get("vehicles")) if hasattr(objects, "get") else False
        if not beacons and not has_sensor_dets:
            return {'dets': np.empty((0, 7)), 'info': np.empty((0, 3))}
        dets_all = _collect_ab3d_detections(
            self, objects, beacons, frame_idx=source_tick,
            beacon_id_mgr=self.beacon_id_mgr)
        dets_all = _cross_source_nms(dets_all, cdist_thresh=3.0)
        return dets_all

    # ------------------------------------------------------------------
    def run_step(self, tick:int):
        with self.profiler.profile_frame(tick) as frame:
            # ===== Feature collection =====================================
            with frame.time("feature_collection"):
                self.update_information(tick)
                num_agents = len(self.vehicle_manager_list) + len(self.rsu_manager_list)

            # ===== 1. Drain jitter buffer → feed persistent tracker ========
            with frame.time("tracking"):
                new_frames = self._jitter_buffer.drain(tick)
                latest_dets = None
                latest_source_tick = None
                num_dets = 0

                for source_tick, (objects, beacons) in new_frames:
                    # Detection-source dispatch. Default = object-level
                    # perception (RSU/vehicle detections) with cross-source NMS.
                    # A detector-mode subclass (DETECTOR=oracle) overrides
                    # _collect_detections_for_frame to source GT detections, so
                    # the architecture comparison can be run without the
                    # perception-noise confound.
                    dets_all = self._collect_detections_for_frame(
                        objects, beacons, source_tick)
                    num_dets = max(num_dets, len(dets_all['dets']))

                    # DET_TRACE: log fused detection world (x,y) per source tick
                    # to separate detection dropout from tracker association
                    # failure for the moving cross-traffic (CARLA x=det[3],
                    # CARLA y=det[5]).
                    if os.environ.get('DET_TRACE'):
                        _dxy = [(round(float(d[3]), 1), round(float(d[5]), 1))
                                for d in dets_all['dets']]
                        _tesla = [p for p in _dxy if 125.0 <= p[1] <= 131.0]
                        # box dims (l,w,yaw) for tesla-path dets to see if the
                        # box is consistent enough for the GIoU gate
                        _tbox = [(round(float(d[3]),1), round(float(d[5]),1),
                                  round(float(d[2]),1), round(float(d[1]),1),
                                  round(float(d[6]),2))
                                 for d in dets_all['dets']
                                 if 125.0 <= float(d[5]) <= 131.0]
                        logger.warning("[DETS] src=%d n=%d tesla(x,y,l,w,yaw)=%s",
                                       source_tick, len(_dxy), _tbox)

                    _t0 = time.perf_counter()
                    tracks, _ = self.tracker.track(dets_all, source_tick)
                    if os.environ.get('DET_TRACE'):
                        logger.warning(
                            "[ASSOC] src=%d ntracks=%d birth_attempts=%s "
                            "suppressed=%s births_after_gate=%s cull=%s",
                            source_tick,
                            len(getattr(self.tracker, 'trackers', [])),
                            getattr(self.tracker, 'birth_attempts_anon', None),
                            getattr(self.tracker, 'birth_suppressed_by_gate', None),
                            getattr(self.tracker, 'births_anon_after_gate', None),
                            getattr(self.tracker, 'anon_cull_count', None))
                    logger.debug("tracker.track() tick=%d src=%d dets=%d "
                                 "took %.1fms", tick, source_tick,
                                 len(dets_all['dets']),
                                 (time.perf_counter() - _t0) * 1000)
                    if tracks and len(tracks[0]) > 0:
                        self._track_history.append(tracks[0])
                    latest_dets = dets_all
                    latest_source_tick = source_tick

                # BSM rotation reconciliation (persistent tracker)
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

            # No new data arrived this tick — return empty
            if not new_frames:
                frame.set_counts(num_agents=num_agents, num_detections=0,
                                 num_tracks=0, num_predictions=0)
                return ecloud.EdgeObjects()

            # ===== 2. convert to trajectories & predict ===================
            with frame.time("detection"):
                self._latest_source_tick = latest_source_tick
                self._ab3d_history_to_trajs(self._track_history, horizon=30)
                num_tracks = len(self.tracked_trajectories)

                # GT snapshot at the source tick of the latest drained frame
                gt_snapshot = self._gt_snapshots.get(latest_source_tick)
                excluded_vehicles = self._excluded_snapshots.get(
                    latest_source_tick)

                # Compute detection metrics
                managed_positions = [vm.vehicle.get_location()
                                     for vm in self.vehicle_manager_list]
                det_metrics = self._compute_detection_metrics(
                    latest_dets, gt_snapshot, managed_positions,
                    excluded_vehicles=excluded_vehicles)

            # Set detection metrics
            frame.set_detection_metrics(
                true_positives=det_metrics['tp'],
                false_positives=det_metrics['fp'],
                false_negatives=det_metrics['fn']
            )

            # Ego-Uniqueness analysis
            # With anchoring, tracks carry temp_ids from BeaconIdManager;
            # translate back so the monitor can match managed vehicles.
            # Without anchoring, tracks have raw temp_ids/cid=-1 which
            # won't match managed vehicle IDs — the monitor will see
            # these as potential duplicates/ghosts (correct behaviour).
            latest_tracks = (self._track_history[-1]
                             if self._track_history else None)
            if self.anchoring and latest_tracks is not None and len(latest_tracks) > 0:
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

                # Compute tracking metrics
                track_metrics = self._compute_tracking_metrics(
                    self._track_history, gt_snapshot, managed_positions)

            # Set tracking metrics
            frame.set_tracking_metrics(
                id_switches=track_metrics['id_switches'],
                fragmentations=track_metrics['fragmentations'],
                mota=track_metrics['mota'],
                motp=track_metrics.get('motp', 0.0)
            )

            # Compute prediction metrics
            lag_steps = tick - latest_source_tick if latest_source_tick else 0
            frame.set_aoi_ticks(lag_steps)
            pred_metrics = self._evaluate_predictions(
                tick, preds, gt_snapshot, lag_steps)

            # Set prediction metrics
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

            # Temporary: log all prediction tracks for Lincoln detection debugging
            if preds and 90 <= tick <= 140:
                logger.warning(
                    "[PREDS] tick=%d n=%d tracks=%s",
                    tick, len(preds),
                    [(getattr(p.obstacle_trajectory.obstacle, 'carla_id', -1),
                      f"{getattr(p.obstacle_trajectory.obstacle, 'kf_speed_mps', 0.0):.1f}",
                      f"({p.obstacle_trajectory.obstacle.location.x:.1f},"
                      f"{p.obstacle_trajectory.obstacle.location.y:.1f})")
                     for p in preds])
            # ===== Per-ego ego-consistency suppression (publish boundary) ==
            # Part of the anchoring contract: anchor + enforce ego-uniqueness.
            # For each consumer ego, suppress anonymous tracks satisfying
            # gate G(e): distance < r_pos AND |v_track - v_ego| < r_v.
            # Per-ego lists prevent cross-consumer coupling (suppressing a
            # legitimate obstacle near ego2 from ego1's view) and the speed
            # gate avoids false-suppression of adjacent-lane vehicles.
            #
            # IMPORTANT: Never mutate obs.carla_id — preds is shared across
            # all per-ego iterations.  Use ego-local `cid` override only.
            ego_suppress_sets = {}  # {vehicle_id: set of pred indices}
            # LF-guarded: indices of coasting (stale) tracks to withhold from
            # every ego, independent of anchoring. A track matched this tick has
            # time_since_update==0, so a real detected obstacle is never withheld.
            stale_pred_idx = set()
            if getattr(self, 'stale_track_suppression', False):
                _stale_tids = {trk.id + 1 for trk in self.tracker.trackers
                               if trk.time_since_update >= self.stale_track_n}
                stale_pred_idx = {
                    i for i, p in enumerate(preds)
                    if getattr(p.obstacle_trajectory.obstacle, 'track_id', None)
                    in _stale_tids}
            _SWAP_MARGIN = 0.5   # metres — avoid borderline flips
            _FOOTPRINT_L = 1.2   # longitudinal margin beyond half-length
            _FOOTPRINT_W = 1.0   # lateral margin beyond half-width
            # Swept-path tolerance: a stationary track within this distance of
            # the ego's OWN recent (x,y) trace is a self-echo. Kept tight
            # (~half a lane) so a vehicle one lane over is never matched.
            _SWEPT_PATH_TOL = 1.5  # metres
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
                    _prev_loc = self._prev_beacon_locs.get(vm.vehicle.id)
                    if _prev_loc is not None:
                        ego_speed = (
                            (ego_loc.x - _prev_loc.x)**2 +
                            (ego_loc.y - _prev_loc.y)**2
                        )**0.5 / self.dt
                    else:
                        ego_vel = vm.vehicle.get_velocity()
                        ego_speed = (ego_vel.x**2 + ego_vel.y**2)**0.5

                    # Ego footprint for stationary-track suppression:
                    # suppress obs within inflated bounding box in ego
                    # frame.  Catches "sensor echo on own body" without
                    # nuking a real stopped lead vehicle whose centroid
                    # is ahead of the footprint.
                    yaw_rad = math.radians(ego_tf.rotation.yaw)
                    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
                    ext = vm.vehicle.bounding_box.extent
                    half_L = ext.x + _FOOTPRINT_L
                    half_W = ext.y + _FOOTPRINT_W

                    suppress_idx = set()
                    # Instrumentation counters
                    n_footprint = 0    # suppressed by footprint overlap
                    n_speed_gate = 0   # suppressed by speed gate match
                    n_swap_detect = 0  # swaps detected (ID stripped)
                    n_stat_cand = 0    # stationary candidates examined
                    n_swept = 0        # suppressed by swept-path self-echo

                    # SAFETY CHECK: a suppressed track whose nearest GT actor is
                    # NOT the ego is a real-obstacle suppression (a VRU / occluder
                    # / stopped vehicle erased). This must never happen. We log
                    # any such event loudly so the property is measurable.
                    _gt_snap = (self._gt_snapshots.get(tick)
                                or self._gt_snapshots.get(self._latest_source_tick)
                                or {})
                    def _nearest_gt_is_ego(px, py, ego_id=vm.vehicle.id):
                        # Returns (is_ego, nearest_id, dist). If GT unavailable
                        # this tick, returns (True, ...) so we do NOT raise a
                        # spurious real-obstacle warning on missing data.
                        if not _gt_snap:
                            return True, None, float('inf')
                        best_id, best_d = None, float('inf')
                        for aid, vd in _gt_snap.items():
                            d = ((vd['x'] - px) ** 2 + (vd['y'] - py) ** 2) ** 0.5
                            if d < best_d:
                                best_d, best_id = d, aid
                        return best_id == ego_id, best_id, best_d

                    for i, pred in enumerate(preds):
                        obs = pred.obstacle_trajectory.obstacle
                        cid = getattr(obs, 'carla_id', -1)
                        if cid in managed_ids_supp:
                            if cid == vm.vehicle.id:
                                # Track carries THIS ego's own beacon
                                # id: suppress unconditionally.
                                suppress_idx.add(i)
                                logger.debug(
                                    "[OWN-BEACON] tick=%d ego=%d "
                                    "pred_idx=%d cid=%d suppressed",
                                    tick, vm.vehicle.id, i, cid)
                                continue
                            # Track carries ANOTHER ego's ID — check
                            # for beacon identity swap: track is
                            # physically closer to us than to the
                            # claimed ego.
                            ploc = obs.location
                            d_to_us = ((ploc.x - ego_loc.x)**2 +
                                       (ploc.y - ego_loc.y)**2)**0.5
                            # Guard 1: only suspect swap if near us
                            if d_to_us >= self._self_id_radius:
                                continue
                            # Guard 2: compare to claimed ego's location
                            claimed_loc = managed_locs.get(cid)
                            if claimed_loc is not None:
                                d_to_claimed = (
                                    (ploc.x - claimed_loc.x)**2 +
                                    (ploc.y - claimed_loc.y)**2)**0.5
                                if d_to_us + _SWAP_MARGIN < d_to_claimed:
                                    # Ego-local override only — do NOT
                                    # mutate obs.carla_id (shared object).
                                    cid = -1
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
                        # Anonymous / stripped-ID suppression gate
                        if d_to_us >= self._self_id_radius:
                            continue
                        obs_speed = getattr(obs, 'kf_speed_mps', 0.0)
                        logger.warning(
                            "[SUPP-TRACE] tick=%d ego=%d pred_idx=%d "
                            "cid=%d d=%.2fm obs_spd=%.2f ego_spd=%.2f "
                            "gate=%.1f",
                            tick, vm.vehicle.id, i, cid, d_to_us,
                            obs_speed, ego_speed,
                            self._self_id_speed_gate)

                        # Two suppression paths:
                        # (a) Ego-footprint overlap: stationary track
                        #     whose center lies within the inflated ego
                        #     bounding box.  Catches camera self-echo
                        #     without suppressing a stopped lead vehicle
                        #     whose centroid is ahead of the footprint.
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
                                _is_ego, _gid, _gd = _nearest_gt_is_ego(ploc.x, ploc.y)
                                if not _is_ego:
                                    logger.warning(
                                        "[SUPP-REAL-OBSTACLE] FOOTPRINT tick=%d "
                                        "ego=%d suppressed track near GT actor %s "
                                        "(d=%.2fm) at (%.1f,%.1f) — NOT ego!",
                                        tick, vm.vehicle.id, _gid, _gd,
                                        ploc.x, ploc.y)
                                logger.debug(
                                    "[FOOTPRINT] tick=%d ego=%d "
                                    "pred_idx=%d dx=%.2f dy=%.2f "
                                    "half_L=%.2f half_W=%.2f "
                                    "obs_spd=%.1f d=%.2f",
                                    tick, vm.vehicle.id, i,
                                    dx_ego, dy_ego, half_L, half_W,
                                    obs_speed, d_to_us)
                                continue
                            # (a2) Swept-path self-echo: a stationary track
                            #      outside the tight footprint but sitting on
                            #      the ego's OWN recent trajectory is a stale
                            #      self-echo (the ego physically occupied that
                            #      point a moment ago). A VRU / occluder /
                            #      stopped vehicle is never on the ego's just-
                            #      traversed path, so this does not blind the
                            #      ego to real obstacles near it.
                            _hist = self._ego_path_hist.get(vm.vehicle.id)
                            if _hist:
                                _min_d = min(
                                    ((ploc.x - hx) ** 2 + (ploc.y - hy) ** 2) ** 0.5
                                    for (hx, hy) in _hist)
                                if _min_d <= _SWEPT_PATH_TOL:
                                    suppress_idx.add(i)
                                    n_swept += 1
                                    _is_ego, _gid, _gd = _nearest_gt_is_ego(ploc.x, ploc.y)
                                    if not _is_ego:
                                        logger.warning(
                                            "[SUPP-REAL-OBSTACLE] SWEPT-PATH tick=%d "
                                            "ego=%d suppressed track near GT actor %s "
                                            "(d=%.2fm) at (%.1f,%.1f) path_d=%.2f — NOT ego!",
                                            tick, vm.vehicle.id, _gid, _gd,
                                            ploc.x, ploc.y, _min_d)
                                    logger.debug(
                                        "[SWEPT-PATH] tick=%d ego=%d pred_idx=%d "
                                        "path_d=%.2f d_to_us=%.2f obs_spd=%.2f",
                                        tick, vm.vehicle.id, i, _min_d,
                                        d_to_us, obs_speed)
                                    continue
                        # (b) Speed-gate: moving track whose speed
                        #     matches the ego (within tolerance).
                        if abs(obs_speed - ego_speed) >= \
                                self._self_id_speed_gate:
                            continue
                        n_speed_gate += 1
                        suppress_idx.add(i)
                        _is_ego, _gid, _gd = _nearest_gt_is_ego(ploc.x, ploc.y)
                        if not _is_ego:
                            logger.warning(
                                "[SUPP-REAL-OBSTACLE] SPEED-GATE tick=%d "
                                "ego=%d suppressed track near GT actor %s "
                                "(d=%.2fm) at (%.1f,%.1f) — NOT ego!",
                                tick, vm.vehicle.id, _gid, _gd,
                                ploc.x, ploc.y)
                        logger.debug(
                            "[SPEED-GATE] tick=%d ego=%d "
                            "pred_idx=%d obs_spd=%.2f ego_spd=%.2f "
                            "gate=%.1f d=%.2f",
                            tick, vm.vehicle.id, i,
                            obs_speed, ego_speed,
                            self._self_id_speed_gate, d_to_us)
                    ego_suppress_sets[vm.vehicle.id] = suppress_idx
                    if suppress_idx or n_swap_detect:
                        logger.warning(
                            "[EGO-SUPPRESS] tick=%d ego=%d "
                            "suppressed=%d (footprint=%d swept=%d speed_gate=%d) "
                            "swaps_detected=%d stat_candidates=%d "
                            "r_pos=%.1f r_v=%.1f",
                            tick, vm.vehicle.id, len(suppress_idx),
                            n_footprint, n_swept, n_speed_gate, n_swap_detect,
                            n_stat_cand, self._self_id_radius,
                            self._self_id_speed_gate)

            # Advance beacon position history for next tick's ego_speed estimate
            for vid, (loc, _ext) in beacons.items():
                self._prev_beacon_locs[vid] = loc
                # Append to the rolling swept-path history (~1s window).
                hist = self._ego_path_hist.get(vid)
                if hist is None:
                    hist = deque(maxlen=20)
                    self._ego_path_hist[vid] = hist
                hist.append((loc.x, loc.y))

            # ===== 3. distribute predictions (with compute contention) =====
            with frame.time("distribution"):
                serialized_preds = ecloud.EdgeObjects()
                # Pickle full set as fallback (egos without suppression)
                pickled_fresh = None
                try:
                    pickled_fresh = pickle.dumps(preds)
                except Exception as e:
                    logging.warning("Error serializing predictions: %s", e)

                # --- compute contention model ---
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
                        vm.vehicle.id, set()) | stale_pred_idx
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

                    if random.random()*100 < self.downlink_pl:
                        vm.agent.edge_predictions.clear()
                    elif is_contended:
                        # stale: use previous tick's predictions
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

                    # cache per-ego predictions for contention fallback
                    self._prev_per_vehicle_preds[index] = ego_preds.copy()

                self._prev_pickled_preds = pickled_fresh

                # log and record contention
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
                self._advance_actors(tick)

            # Set profiler counts
            frame.set_counts(
                num_agents=num_agents,
                num_detections=num_dets,
                num_tracks=num_tracks,
                num_predictions=num_predictions
            )

            return serialized_preds

    # ------------------------------------------------------------------
    #  Per-tick vehicle/RSU advance (override hook for CIP and friends)
    # ------------------------------------------------------------------
    def _advance_actors(self, tick: int) -> None:
        """Default actor advance: each VM perceives, plans, and actuates
        locally; each RSU updates and runs its step.

        Subclasses (e.g., CIPEdge) override this when planning moves to
        the edge or when the consumer boundary changes.
        """
        for vm in self.vehicle_manager_list:
            vm.update_info(tick)
            vm.vehicle.apply_control(vm.run_step())
            self._label_brake_attributions_gt(vm)
            self._record_time_to_events(tick, vm)
        # Use the unfiltered live snapshot, not _gt_snapshots: the latter is
        # range-limited to 50m of a managed vehicle and tick-keyed by source
        # frame, so far/approaching cross-traffic is missing exactly when the
        # closing geometry matters.
        self._log_conflict_kinematics(tick, self._live_gt_snapshot())

    # ------------------------------------------------------------------
    #  Edge-only distributed mode: collect / apply split
    # ------------------------------------------------------------------
    def collect_features(self, step: int):
        """
        Edge-only distributed mode: drive perception, then serialize each actor's
        YOLO detections and pose into an IntermediateFeaturesBatch for the RPC.

        RSUs are packed first (agent-0 convention, mirroring WorldFusion).
        Callers pass the returned batch to EdgeFusionClient.fuse().
        """
        import pickle as _pickle
        import ecloud_pb2 as ecloud

        self.update_information(step)

        batch = ecloud.IntermediateFeaturesBatch(tick_id=step)

        for rsu in self.rsu_manager_list:
            pos = rsu.localizer.get_ego_pos()
            pose_list = [
                pos.location.x, pos.location.y, pos.location.z,
                pos.rotation.roll, pos.rotation.yaw, pos.rotation.pitch
            ] if pos is not None else [0.0] * 6
            actor_id = rsu.vehicle.id if hasattr(rsu, 'vehicle') else 0
            objects = {k: v for k, v in getattr(rsu, 'objects', {}).items()
                       if k != 'traffic_lights'}
            batch.features.append(ecloud.IntermediateFeatures(
                agent_id=actor_id,
                agent_type=ecloud.RSU,
                pose=ecloud.AgentPose(pose=pose_list),
                pickled_objects=_pickle.dumps(objects),
            ))

        for vm in self.vehicle_manager_list:
            pos = vm.localizer.get_ego_pos()
            pose_list = [
                pos.location.x, pos.location.y, pos.location.z,
                pos.rotation.roll, pos.rotation.yaw, pos.rotation.pitch
            ] if pos is not None else [0.0] * 6
            objects = {k: v for k, v in getattr(vm.agent, 'objects', {}).items()
                       if k != 'traffic_lights'}
            batch.features.append(ecloud.IntermediateFeatures(
                agent_id=vm.vehicle.id,
                agent_type=ecloud.VEHICLE,
                pose=ecloud.AgentPose(pose=pose_list),
                pickled_objects=_pickle.dumps(objects),
            ))

        return batch

    def apply_predictions(self, step: int, fusion_result):
        """
        Edge-only distributed mode: unpack per-vehicle FusionResult from edge
        container, inject predictions, then run planning and control.

        Late fusion uses per-ego filtered predictions (ego-suppression may differ
        per vehicle), so each vehicle gets its own entry from all_preds.
        """
        import pickle as _pickle

        all_preds = {}
        if fusion_result is not None and fusion_result.pickled_predictions:
            try:
                all_preds = _pickle.loads(fusion_result.pickled_predictions)
            except Exception as exc:
                logger.warning("apply_predictions: failed to unpack FusionResult: %s", exc)

        logger.debug("apply_predictions: tick=%d  vehicles=%d  pred_keys=%s",
                     step, len(self.vehicle_manager_list), list(all_preds.keys()))

        for idx, vm in enumerate(self.vehicle_manager_list):
            preds_bytes = all_preds.get(idx)
            if preds_bytes:
                try:
                    predictions = _pickle.loads(preds_bytes)
                except Exception:
                    predictions = []
            else:
                predictions = []
            vm.agent.edge_predictions = list(predictions)
            vm.update_info(step)
            vm.vehicle.apply_control(vm.run_step())
            self._label_brake_attributions_gt(vm)
            self._record_time_to_events(step, vm)
        for rsu in self.rsu_manager_list:
            rsu.update_info()
            rsu.run_step()

    # ------------------------------------------------------------------
    #  Trajectory conversion
    # ------------------------------------------------------------------
    def _ab3d_history_to_trajs(self, hist:Deque[np.ndarray], horizon:int=10):
        updated: set[int] = set()
        # Only rebuild/publish tracks the tracker still considers ALIVE. The
        # 30-frame history buffer outlives AB3DMOT's max_age pruning, so a track
        # aged out of self.tracker.trackers can otherwise linger in `hist` and be
        # republished as a frozen "zombie" at its last position, which the
        # consumer planner then brakes for. Gate on the live tracker id set
        # (published tid = trk.id + 1) so a pruned track stops being published.
        live_tids = None
        try:
            live_tids = {trk.id + 1 for trk in self.tracker.trackers}
        except Exception:
            live_tids = None  # fail open if tracker state is unavailable
        # Drop trajectories whose track is no longer alive.
        if live_tids is not None:
            for _dead in [t for t in self.tracked_trajectories if t not in live_tids]:
                del self.tracked_trajectories[_dead]
        # Clear and rebuild from the last N track outputs in hist.
        for traj in self.tracked_trajectories.values():
            traj.trajectory.clear()
        for frame in hist:
            if frame is None or len(frame)==0: continue
            for trk in frame:
                tid = int(trk[7]);  cid_raw = int(trk[8])
                if live_tids is not None and tid not in live_tids:
                    continue  # zombie: aged out of tracker, do not republish

                if self.anchoring:
                    # With beacon anchoring: the vehicle knows its own
                    # temp_ids (it generated them), so it can identify
                    # beacon-associated tracks by matching temp_id.
                    # This is realistic — the reverse-mapping represents
                    # the vehicle's own knowledge of its pseudonyms.
                    real_cid = self.beacon_id_mgr.get_carla_id_for_temp(cid_raw)
                    cid = real_cid if real_cid is not None else cid_raw
                else:
                    # Without beacon anchoring: naive edge baseline.
                    # All tracks are anonymous — the vehicle has no
                    # identity protocol and must rely on spatial self-
                    # identification (below) to find itself among tracks.
                    cid = cid_raw

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
                # KF velocity (m/tick) → m/s for downstream prediction gating
                # KITTI dx(10)=CARLA vx, KITTI dz(12)=CARLA vy (ground plane)
                if len(trk) > 12:
                    kf_vx, kf_vy = float(trk[10]), float(trk[12])
                    traj.obstacle.kf_speed_mps = ((kf_vx**2 + kf_vy**2)**0.5) / self.dt
                    # Store per-axis velocity (m/tick) for KF-based prediction
                    traj.obstacle.kf_vx = kf_vx
                    traj.obstacle.kf_vy = kf_vy

        # prune stale
        for tid in list(self.tracked_trajectories):
            if tid not in updated:
                del self.tracked_trajectories[tid]

        # Per-track provenance: GT-match each live track's current position to
        # the nearest GT actor this tick and record (ego/nonego). The history
        # lets the brake classifier tell self_ghost (consistently ego) from
        # track_merge (ego AND non-ego) and external_stale (consistently a
        # non-ego actor), instead of nearest-ego-now which mislabels stale/
        # merged tracks.
        _gt_snap = self._gt_snapshots.get(self._latest_source_tick)
        if os.environ.get('GUARD_TRACE'):
            logger.warning(
                "[PROV-POP] latest_source_tick=%s snap_keys=%s snap_hit=%s n_tracks=%d",
                self._latest_source_tick,
                sorted(self._gt_snapshots.keys())[-5:],
                bool(_gt_snap), len(self.tracked_trajectories))
        if _gt_snap:
            _managed = {m.vehicle.id for m in self.vehicle_manager_list}
            _gids = list(_gt_snap.keys())
            if _gids:
                _gx = np.array([_gt_snap[g]['x'] for g in _gids])
                _gy = np.array([_gt_snap[g]['y'] for g in _gids])
                for _tid, _traj in self.tracked_trajectories.items():
                    if not _traj.trajectory:
                        continue
                    _loc = _traj.trajectory[0].location
                    _d = np.hypot(_gx - _loc.x, _gy - _loc.y)
                    _j = int(np.argmin(_d))
                    _nid = _gids[_j]
                    _h = self._track_provenance.get(_tid)
                    if _h is None:
                        _h = deque(maxlen=15)
                        self._track_provenance[_tid] = _h
                    _h.append(('ego' if _nid in _managed else 'nonego',
                               _nid, float(_d[_j])))
            # Retain provenance for recently-dead tracks: brakes can fire on a
            # stale prediction whose edge track was already pruned, and the
            # classifier needs that track's provenance to label it
            # track_merge / external_stale instead of falling back to
            # nearest-now (which mislabels it self_ghost). Cap to bound memory.
            if len(self._track_provenance) > 256:
                _live = set(self.tracked_trajectories)
                _dead = [t for t in self._track_provenance if t not in _live]
                for _tid in _dead[:len(self._track_provenance) - 256]:
                    del self._track_provenance[_tid]

        # Temporary: log tracked trajectory summary for debugging
        if self.tracked_trajectories:
            summary = [(tid,
                        getattr(t.obstacle, 'carla_id', -1),
                        f"{getattr(t.obstacle, 'kf_speed_mps', 0.0):.1f}",
                        f"({t.obstacle.location.x:.1f},{t.obstacle.location.y:.1f})",
                        len(t.trajectory))
                       for tid, t in self.tracked_trajectories.items()]
            logger.warning("[TRACKS] n=%d %s", len(summary), summary)

        # ZOMBIE TRACE: for each PUBLISHED track, is its tid still alive in the
        # AB3DMOT tracker (tracker.trackers), and what is its time_since_update?
        # Published tid = trk.id + 1. A published track absent from tracker.trackers
        # is a zombie (aged out of AB3DMOT but still rebuilt from the 30-frame
        # _track_history). This is what the ego can brake for.
        if os.environ.get('ZOMBIE_TRACE') and self.tracked_trajectories:
            live = {}
            try:
                for trk in self.tracker.trackers:
                    live[trk.id + 1] = getattr(trk, 'time_since_update', -1)
            except Exception as _e:
                live = {'ERR': str(_e)}
            pub = sorted(self.tracked_trajectories.keys())
            rows = [(tid, ('LIVE' if tid in live else 'ZOMBIE'),
                     live.get(tid, None)) for tid in pub]
            logger.warning("[ZOMBIE] hist_len=%d tracker_live=%s published=%s",
                           len(self._track_history), sorted(live.items()), rows)

        # Anchoring OFF: no edge-side self-identification.
        # The edge sends all predictions unsuppressed.  The vehicle
        # does its own proximity-based self-suppression on receive.

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
        # AND beacon detections near managed vehicles (these are GPS
        # anchoring injections, not real sensor detections).
        # After KITTI coord swap: index 3=CARLA_x, index 5=CARLA_y (ground plane)
        valid_det_indices = []
        excluded_det_count = 0
        beacon_det_count = 0
        for det_idx in range(len(dets)):
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 5]

            # Skip beacon detections (near managed ego vehicles)
            if any(np.sqrt((det_x - mx)**2 + (det_y - my)**2) < 3.0
                   for mx, my in managed_xy):
                beacon_det_count += 1
                continue

            near_excluded = False
            for ex, ey in excluded_positions:
                if np.sqrt((det_x - ex)**2 + (det_y - ey)**2) < EXCLUSION_RADIUS:
                    near_excluded = True
                    excluded_det_count += 1
                    break
            if not near_excluded:
                valid_det_indices.append(det_idx)

        if excluded_det_count > 0 or beacon_det_count > 0:
            logger.debug("Filtered %d beacon dets, %d near excluded vehicles",
                         beacon_det_count, excluded_det_count)

        # Greedy matching with filtered detections
        # AB3DMOT detection format: [h, w, l, x(KITTI), y(KITTI=CARLA_z), z(KITTI=CARLA_y), yaw]
        gt_matched, det_matched = set(), set()
        for det_idx in valid_det_indices:
            det_x, det_y = dets[det_idx, 3], dets[det_idx, 5]  # KITTI x, z = CARLA x, y
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

        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        logger.debug("[DET] GT=%d Dets=%d TP=%d FP=%d FN=%d P=%.2f R=%.2f",
                     len(gt_list), len(valid_det_indices), tp, fp, fn, precision, recall)

        return {'tp': tp, 'fp': fp, 'fn': fn}

    def _compute_tracking_metrics(self, history: Deque, gt_vehicles: Dict,
                                   managed_positions: List) -> Dict:
        """Compute tracking metrics: ID switches, MOTA, MOTP."""
        if gt_vehicles is None or not history:
            return {'id_switches': 0, 'fragmentations': 0, 'mota': 0.0, 'motp': 0.0}

        # Get latest tracks (history[-1] is the most recent replayed step)
        latest_tracks = history[-1] if history and len(history[-1]) > 0 else np.empty((0, 9))
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

        # AB3DMOT track format: [h,w,l, KITTI_x, KITTI_y, KITTI_z, yaw, track_id, ...]
        # KITTI x=CARLA_x (idx 3), KITTI z=CARLA_y (idx 5)
        for trk in latest_tracks:
            track_id = int(trk[7])
            tx, ty = trk[3], trk[5]  # KITTI x, z = CARLA x, y
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
    #  Evaluation
    # ------------------------------------------------------------------
    def evaluate(self):
        """
        Return evaluation results for EvaluationManager.

        Returns:
            Tuple[figure, perform_txt, metrics]
        """
        if self.is_proxy:
            return None, "", self._proxy_metrics
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

# ---------------------------------------------------------------------
# alias expected by edge_manager/__init__.py
LateFusionEdge = PredictionLateFusionEdge      # exported name
