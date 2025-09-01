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

import math, random, time, logging
from collections import deque, defaultdict
from typing import Dict, List, Deque

import numpy as np
import carla

from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT

from opencda.core.prediction.linear_predictor_manager import \
    LinearPredictorManager
from opencda.core.sensing.tracking.obstacle_trajectory import \
    ObstacleTrajectory
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from opencda.core.application.edge.edge_debug_helper import EdgeDebugHelper

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

def _box_to_transform(box) -> carla.Transform:
    x,y,z,h,w,l,yaw = box
    loc = carla.Location(x=float(x), y=float(y), z=float(z))
    rot = carla.Rotation(yaw=np.degrees(float(yaw)))
    return carla.Transform(loc, rot)

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
        loc = obj.bounding_box.location
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


        self.debug = EdgeDebugHelper(0)

    # ------------------------------------------------------------------
    def start_edge(self):   # nothing special to pre-compute
        pass

    # ------------------------------------------------------------------
    def update_information(self, frame_idx:int=0):
        """Collect latest detections from every VM and RSU into history."""
        objects: Dict[str,List] = {}

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
        self.update_information(tick)

        # ===== latency handling =======================================
        total_ms  = self._sample_latency_ms()
        lag_steps = int(round(total_ms / (self.dt*1000)))
        if lag_steps >= len(self.objects_deque): return
        as_of = tick - lag_steps

        # ===== 1. replay detection history into fresh AB3DMOT =========
        tracker   = AB3DMOT(self.mot_cfg, self.mot_category)
        history   : Deque[np.ndarray] = deque(maxlen=10)
        oldest    = max(0, tick-(len(self.objects_deque)-1))
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
            tracks, _ = tracker.track(dets_all, step)
            if tracks and len(tracks[0])>0: history.append(tracks[0])

        tracker_ms = tracker.total_time if hasattr(tracker,'total_time') \
                     else 0.0

        # ===== 2. convert to trajectories & predict ===================
        self._ab3d_history_to_trajs(history, horizon=10)
        t0 = time.perf_counter()
        preds = self.lin_pred.generate_predicted_trajectories(
                    self.tracked_trajectories)
        predict_ms = (time.perf_counter()-t0)*1e3

        self.debug.update_edge(0, tracking_time=tracker_ms,
                                  prediction_time=predict_ms,
                                  latency=total_ms)

        # ===== 3. distribute predictions =============================
        for vm in self.vehicle_manager_list:
            if random.random()*100 < self.downlink_loss:
                vm.agent.edge_predictions.clear()
            else:
                vm.agent.edge_predictions = preds.copy()

        # ===== 4. advance vehicles ===================================
        for vm in self.vehicle_manager_list:
            vm.update_info(tick)
            vm.vehicle.apply_control(vm.run_step())
        for rsu in self.rsu_manager_list:
            rsu.update_info();  rsu.run_step()

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

# ---------------------------------------------------------------------
# alias expected by edge_manager/__init__.py
LateFusionEdge = PredictionLateFusionEdge      # exported name
