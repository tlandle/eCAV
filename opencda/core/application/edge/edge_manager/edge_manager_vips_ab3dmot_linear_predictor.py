# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
edge_manager_vips.py
====================

VIPS (Vehicular Intersection Perception System) baseline implementation.

Infrastructure-only perception pipeline for benchmarking against cooperative
perception approaches (late fusion with self-beacon anchoring, BM2CP, WorldFusion).

Key characteristics:
- Uses ONLY RSU/infrastructure sensors (no vehicle sensor contributions)
- NO self-beacon anchoring (key difference from late fusion)
- AB3DMOT for 3D multi-object tracking
- Linear constant-velocity predictor for trajectory prediction

Pipeline:
    RSU detections  -->  latency buffer  -->  AB3DMOT (3-D MOT)
                              |
                              +--> track history (10 frames)  -->  linear
                                   constant-velocity predictor
                                   --> 25 future steps

This provides a baseline to answer the reviewer question:
"Can VIPS (infrastructure-only) achieve what self-beacon anchoring provides?"

Author : Tyler Landle <tlandle3@gatech.edu>
License: TDG-Attribution-NonCommercial-NoDistrib
"""
from __future__ import annotations

import math
import random
import time
import logging
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
from opencda.core.application.edge.edge_metrics import EdgeMetrics
from opencda.core.application.edge.edge_profiler import EdgeProfiler

from .edge_manager_base import _BaseEdgeManager, logger


# ──────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────
_GUID = 0
_MIN_EDGE, _MIN_VOLUME = 0.60, 1.0  # sliver reject thresholds


def _xyz(loc: carla.Location) -> np.ndarray:
    return np.asarray([loc.x, loc.y, loc.z], np.float32)


def _is_sliver(h, w, l):
    """Reject detection if too small (noise/artifact)."""
    return (min(h, w, l) < _MIN_EDGE) or (h * w * l < _MIN_VOLUME)


def _box_to_transform(box) -> carla.Transform:
    """Convert AB3DMOT box [x,y,z,h,w,l,yaw] to CARLA Transform."""
    x, y, z, h, w, l, yaw = box
    loc = carla.Location(x=float(x), y=float(y), z=float(z))
    rot = carla.Rotation(yaw=np.degrees(float(yaw)))
    return carla.Transform(loc, rot)


def _collect_rsu_only_detections(rsu_objects: Dict[str, List],
                                  frame_idx: int):
    """
    Build the dict that AB3DMOT.track() expects.

    VIPS difference: Only uses RSU detections, NO vehicle beacons or sensor data.
    """
    global _GUID
    det_rows, info_rows = [], []

    # Only RSU detections - no vehicle contributions
    for obj in rsu_objects.get("vehicles", []):
        bbx = obj.bounding_box.extent
        h, w, l = bbx.z * 2, bbx.y * 2, bbx.x * 2
        if _is_sliver(h, w, l):
            continue
        loc = obj.bounding_box.location
        det_rows.append([h, w, l, loc.x, loc.y, loc.z, 0.0])
        _GUID += 1
        info_rows.append([frame_idx, _GUID, -1])  # -1 = not a beacon

    return {
        'dets': np.asarray(det_rows, np.float32) if det_rows else np.empty((0, 7), np.float32),
        'info': np.asarray(info_rows, np.int64) if info_rows else np.empty((0, 3), np.int64)
    }


# ──────────────────────────────────────────────────────────────────────
#  Main edge-manager subclass
# ──────────────────────────────────────────────────────────────────────
class VIPSEdge(_BaseEdgeManager):
    """
    VIPS: Infrastructure-only perception baseline.

    Uses only RSU sensors for detection and tracking. No vehicle sensor
    contributions and no self-beacon anchoring. This provides a baseline
    for benchmarking cooperative perception approaches.
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        # latency / loss from base class
        self.lat_ms = cfg.get("latency", 0) * 1000.0
        self.jit_ms = cfg.get("jitter_std", 0) * 1000.0
        self.lat_dist = cfg.get("latency_distribution", "normal")
        self.uplink_loss = cfg.get("uplink_packet_loss_pct", 0)
        self.downlink_loss = cfg.get("downlink_packet_loss_pct", 0)
        self.dt = world_dt

        # managers
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # AB3DMOT tracker configuration
        self.mot_cfg = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': False, 'thres': 2.0,
            'output_dir': None, 'min_hits': 3, 'max_age': 2, 'ego_com': None,
            'affi_pro': False, 'dataset': 'KITTI', 'det_name': 'deprecated'
        })
        self.mot_category = 'Car'

        # history buffers
        self.rsu_objects_deque: Deque[Dict] = deque(maxlen=100)
        self.tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self.track_to_carla: Dict[int, int] = {}

        self.debug = EdgeMetrics(0)

        # Edge profiler for capacity planning
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get('intersection_id', f"vips_{id(self)}"),
            history_size=2000,
            sample_gpu_utilization=True
        )

        logger.info("VIPSEdge initialized - infrastructure-only baseline")

    # ------------------------------------------------------------------
    def start_edge(self):
        """Initialize edge processing."""
        pass

    # ------------------------------------------------------------------
    def update_information(self, frame_idx: int = 0):
        """
        Collect latest detections from RSUs ONLY.

        VIPS: No vehicle sensor data collected - infrastructure only.
        """
        rsu_objects: Dict[str, List] = {}

        # Only collect from RSUs - no vehicle sensor data
        for rsu in self.rsu_manager_list:
            self._dict_extend(rsu_objects, rsu.objects)

        self.rsu_objects_deque.appendleft(rsu_objects)

    # ------------------------------------------------------------------
    def run_step(self, tick: int):
        """
        Main processing step.

        VIPS pipeline:
        1. Collect RSU detections (update_information)
        2. Apply latency model
        3. Replay detection history through AB3DMOT tracker
        4. Convert tracks to trajectories
        5. Generate predictions with linear predictor
        6. Distribute predictions to vehicles
        """
        with self.profiler.profile_frame(tick) as frame:

            # ===== Feature collection (RSU only) =========================
            with frame.time("feature_collection"):
                self.update_information(tick)
                num_agents = len(self.rsu_manager_list)  # Only RSUs

            # ===== Latency handling ======================================
            total_ms = self._sample_latency_ms()
            lag_steps = int(round(total_ms / (self.dt * 1000)))
            if lag_steps >= len(self.rsu_objects_deque):
                frame.set_counts(num_agents=num_agents, num_detections=0,
                                 num_tracks=0, num_predictions=0)
                return
            as_of = tick - lag_steps

            # ===== Tracking: replay detection history into fresh AB3DMOT ==
            with frame.time("tracking"):
                tracker = AB3DMOT(self.mot_cfg, self.mot_category)
                history: Deque[np.ndarray] = deque(maxlen=10)
                oldest = max(0, tick - (len(self.rsu_objects_deque) - 1))

                num_dets = 0
                for step in range(oldest, as_of + 1):
                    idx = tick - step
                    if idx >= len(self.rsu_objects_deque):
                        dets_all = {'dets': np.empty((0, 7)), 'info': np.empty((0, 3))}
                    else:
                        snapshot = self.rsu_objects_deque[idx]
                        dets_all = _collect_rsu_only_detections(snapshot, frame_idx=step)
                        num_dets = max(num_dets, len(dets_all['dets']))

                    tracks, _ = tracker.track(dets_all, step)
                    if tracks and len(tracks[0]) > 0:
                        history.append(tracks[0])

                tracker_ms = tracker.total_time if hasattr(tracker, 'total_time') else 0.0

            # ===== Convert to trajectories ================================
            with frame.time("detection"):
                self._ab3d_history_to_trajs(history, horizon=10)
                num_tracks = len(self.tracked_trajectories)

            # ===== Prediction =============================================
            with frame.time("prediction"):
                t0 = time.perf_counter()
                preds = self.lin_pred.generate_predicted_trajectories(
                    self.tracked_trajectories)
                predict_ms = (time.perf_counter() - t0) * 1e3
                num_predictions = len(preds)

            self.debug.update_edge(0, tracking_time=tracker_ms,
                                   prediction_time=predict_ms,
                                   latency=total_ms)

            # ===== Distribute predictions =================================
            with frame.time("distribution"):
                for vm in self.vehicle_manager_list:
                    if random.random() * 100 < self.downlink_loss:
                        vm.agent.edge_predictions.clear()
                    else:
                        vm.agent.edge_predictions = preds.copy()

            # ===== Advance vehicles (they still need control) =============
            for vm in self.vehicle_manager_list:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())
            for rsu in self.rsu_manager_list:
                rsu.update_info()
                rsu.run_step()

            # Set profiler counts
            frame.set_counts(
                num_agents=num_agents,
                num_detections=num_dets,
                num_tracks=num_tracks,
                num_predictions=num_predictions
            )

    # ------------------------------------------------------------------
    #  Trajectory conversion
    # ------------------------------------------------------------------
    def _ab3d_history_to_trajs(self, hist: Deque[np.ndarray], horizon: int = 10):
        """Convert AB3DMOT track history to ObstacleTrajectory objects."""
        updated: set[int] = set()
        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                tid = int(trk[7])
                cid = int(trk[8])
                tf = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self.tracked_trajectories:
                    dummy = ObstacleVehicle(
                        corners=np.zeros((8, 3)),
                        o3d_bbx=None,
                        track_id=tid,
                        tick_id=0
                    )
                    self.tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))

                traj = self.tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid
                self.track_to_carla[tid] = cid

        # prune stale tracks
        for tid in list(self.tracked_trajectories):
            if tid not in updated:
                del self.tracked_trajectories[tid]

    # ------------------------------------------------------------------
    #  Utilities
    # ------------------------------------------------------------------
    def _dict_extend(self, dest: dict, src: dict):
        for k, v in src.items():
            dest.setdefault(k, []).extend(v)

    def _sample_latency_ms(self) -> float:
        if self.lat_dist == "normal":
            return max(0., random.gauss(self.lat_ms, self.jit_ms))
        elif self.lat_dist == "lognormal":
            mean = math.log(self.lat_ms) if self.lat_ms > 0 else 0
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
VIPSLateFusionEdge = VIPSEdge  # exported name
