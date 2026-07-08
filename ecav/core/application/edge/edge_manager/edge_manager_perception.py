# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Implements PERCEPTION mode for the EdgeManager.

Fuses objects from ego-vehicle perception stacks and RSUs, buffers
configurable history for latency-delayed snapshots, and forwards
results to every VehicleManager.
"""
from __future__ import annotations

import time, logging, random
from collections import deque, defaultdict
from typing import Dict, List

import numpy as np
import carla

from .edge_manager_base import _BaseEdgeManager, logger
from ecav.core.application.edge.edge_metrics import EdgeMetrics


# ──────────────────────────────────────────────────────────────────────
#  Perception-edge manager
# ──────────────────────────────────────────────────────────────────────
class PerceptionEdge(_BaseEdgeManager):
    """
    Edge back-end for *PERCEPTION* mode (no AB3DMOT / no BM2CP).

    All constructor arguments are forwarded to `_BaseEdgeManager`.
    """
    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.05, **kw):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        # history buffers – identical semantics to the previous monolith
        self.objects_deque : deque[Dict[str,list]] = deque(maxlen=100)
        self.traj_dict     : Dict[int,deque]       = {}
        self.vehicle_speeds: Dict[int,carla.Vector3D] = {}
        self.other_vehicles: list = kw.get("other_vehicles", [])

        # NOTE: latency_model (uplink latency + loss) and downlink_pl
        # are created by _BaseEdgeManager.__init__ from the same cfg.
        # dt and debug are also set by the base class.

    # ------------------------------------------------------------------
    #  Life-cycle hooks required by _BaseEdgeManager
    # ------------------------------------------------------------------
    def start_edge(self):
        # nothing to pre-compute for perception mode
        for vm in self.vehicle_manager_list:
            self.traj_dict[vm.vehicle.id] = deque(maxlen=50)
        for seq in self.other_vehicles:
            self.traj_dict[seq._actor.id] = deque(maxlen=50)

    # ------------------------------------------------------------------
    def update_information(self, frame_idx: int = 0):
        """
        Pull fresh object lists from every VehicleManager & RSU and copy
        them into self.objects + self.objects_deque (no deduplication –
        that is up to the downstream fusion in each VM).
        """
        # fresh global dict each tick
        objects: Dict[str, List] = {}

        # ── a)  merge every VehicleManager’s perception ----------------
        for vm in self.vehicle_manager_list:

            # keep each ego’s own objects *except* those belonging to itself
            vm_objects = {}
            for otype, olist in vm.agent.objects.items():
                vm_objects[otype] = [
                    o for o in olist
                    if o.get_location().distance(vm.vehicle.get_location()) > 3
                ]
            # uplink packet loss simulation (delegated to latency model)
            if not self.latency_model.should_drop():
                self._dict_extend(objects, vm_objects)

            # record trajectory / speed helpers for co-operative planning
            self.traj_dict[vm.vehicle.id] = vm.agent.get_local_planner()\
                                              .get_waypoint_buffer().copy()
            self.vehicle_speeds[vm.vehicle.id] = vm.vehicle.get_velocity()

        # ── b)  non-managed vehicles under TrafficManager control -------
        for seq in self.other_vehicles:
            lp = seq._local_planner_dict.get(seq._actor)
            if lp is None: continue
            self.traj_dict[seq._actor.id] = lp._waypoints_queue.copy()
            self.vehicle_speeds[seq._actor.id] = seq._actor.get_velocity()

        # ── c)  RSUs ----------------------------------------------------
        for rsu in self.rsu_manager_list:
            self._dict_extend(objects, rsu.objects)

        # push into history
        self.objects_deque.appendleft(objects)

    # ------------------------------------------------------------------
    def evaluate(self):
        """Minimal edge-eval hook.

        PerceptionEdge ships only detections; tracking, prediction, and
        planning happen on the vehicle, so the self-ghost / brake metrics
        are produced by the per-vehicle brake-attribution path, not by an
        edge-side profiler. Return an empty figure/text and a small metrics
        dict so EvaluationManager can finish and write simulation_metrics.json.
        """
        if self.is_proxy:
            return None, "", self._proxy_metrics
        metrics = {
            'mode': 'PERCEPTION',
            'edge_publishes': 'detections_only',
        }
        return None, "", metrics

    # ------------------------------------------------------------------
    def run_step(self, tick: int):
        """
        Called from the simulation supervisor once every world tick.
        Implements latency sampling, snapshot replay and distribution to
        VehicleManagers; then lets the local controllers step.
        """
        self.update_information(tick)

        # ===== latency sampling (via pluggable latency model) =========
        arrival = self.latency_model.stamp(tick)
        lag_steps = arrival - tick
        if lag_steps >= len(self.objects_deque):
            # not enough history yet – skip this edge step
            return
        snapshot = self.objects_deque[lag_steps].copy()

        # ===== forward to every VehicleManager ========================
        for vm in self.vehicle_manager_list:

            # simulate down-link loss
            if random.random() * 100 < self.downlink_pl:
                vm.edge_objects.clear()
            else:
                # make a per-car copy so downstream modifications don’t clash
                objs = {k:list(v) for k,v in snapshot.items()}
                # remove self duplicates just in case
                for olist in objs.values():
                    olist[:] = [o for o in olist
                                if o.get_location().distance(
                                    vm.vehicle.get_location()) > 3.0]
                vm.edge_objects = objs

            # helpers for driving decisions
            vm.agent.other_car_trajectories = {
                k: deque(v, maxlen=50) for k,v in self.traj_dict.items()}
            vm.agent.other_car_speeds = self.vehicle_speeds.copy()

        # ===== advance each VehicleManager ============================
        if not self.run_distributed:
            for vm in self.vehicle_manager_list:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())
                # GT-label brake events so self-ghost / other-FP / TP are
                # measured. The edge ships only detections here; the vehicle
                # tracks and predicts locally, so any self-ghost that appears
                # is produced by multi-source object disagreement reaching the
                # on-board tracker, not by edge-side prediction.
                self._label_brake_attributions_gt(vm)

            self._log_conflict_kinematics(tick, self._live_gt_snapshot())

            for rsu in self.rsu_manager_list:
                rsu.update_info()
                rsu.run_step()

