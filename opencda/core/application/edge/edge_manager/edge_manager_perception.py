#-*- coding: utf-8 -*-
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
edge_manager.perception
=======================

Implements *PERCEPTION* mode:  
• fuses objects from the ego-vehicle perception stacks + RSUs  
• buffers a configurable amount of history so we can replay a
  latency-delayed snapshot  
• forwards that snapshot (and other helpers such as predicted waypoint
  buffers / speeds) to every VehicleManager.

Author : Tyler Landle <tlandle3@gatech.edu>
License: TDG-Attribution-NonCommercial-NoDistrib
"""
from __future__ import annotations

import time, math, logging, random
from collections import deque, defaultdict
from typing import Dict, List

import numpy as np
import carla

from .edge_manager_base import _BaseEdgeManager, logger
from opencda.core.application.edge.edge_debug_helper import EdgeDebugHelper


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

        # packet-loss parameters
        self.uplink_loss_pct   = cfg.get("uplink_packet_loss_pct", 0)
        self.downlink_loss_pct = cfg.get("downlink_packet_loss_pct", 0)

        # latency modelling
        self.latency_ms        = cfg.get("latency", 0) * 1000.0   # ms
        self.latency_jitter_ms = cfg.get("jitter_std", 0) * 1000.0
        self.latency_dist      = cfg.get("latency_distribution", "normal")

        # step-time
        self.dt   : float = world_dt          # seconds
        self.debug: EdgeDebugHelper = EdgeDebugHelper(0)

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
            # uplink packet loss simulation
            if random.random()*100 >= self.uplink_loss_pct:
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
    def run_step(self, tick: int):
        """
        Called from the simulation supervisor once every world tick.
        Implements latency sampling, snapshot replay and distribution to
        VehicleManagers; then lets the local controllers step.
        """
        self.update_information(tick)

        # ===== latency sampling =======================================
        total_latency_ms = self._sample_latency_ms()
        lag_steps = int(round(total_latency_ms / (self.dt * 1000.0)))
        if lag_steps >= len(self.objects_deque):
            # not enough history yet – skip this edge step
            return
        snapshot = self.objects_deque[lag_steps].copy()

        # ===== forward to every VehicleManager ========================
        for vm in self.vehicle_manager_list:

            # simulate down-link loss
            if random.random()*100 < self.downlink_loss_pct:
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
        for vm in self.vehicle_manager_list:
            vm.update_info(tick)
            vm.vehicle.apply_control(vm.run_step())

        for rsu in self.rsu_manager_list:
            rsu.update_info()
            rsu.run_step()

    # ------------------------------------------------------------------
    #  Utility helpers
    # ------------------------------------------------------------------
    def _dict_extend(self, dest:dict, src:dict):
        """dest[key] += src[key]  (creating lists if necessary)."""
        for k, v in src.items():
            dest.setdefault(k, []).extend(v)

    # ------------------------------------------------------------------
    def _sample_latency_ms(self) -> float:
        """Return a (jittered) latency sample in *milliseconds*."""
        if self.latency_dist == "normal":
            return max(0.0, random.gauss(self.latency_ms,
                                         self.latency_jitter_ms))
        elif self.latency_dist == "lognormal":
            mean = math.log(self.latency_ms) if self.latency_ms > 0 else 0
            return np.random.lognormal(mean, 0.5)
        else:      # fixed
            return self.latency_ms
