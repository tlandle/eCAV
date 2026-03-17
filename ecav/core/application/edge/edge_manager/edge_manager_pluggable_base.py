"""Shared base for pluggable edge managers (SOTAEdge, AdaptiveEdge).

Composes fusion + tracker from YAML config using component registries.
Provides the common collect/track/advance loop. Subclasses implement
run_step() with their own prediction strategy.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import random
from collections import deque
from typing import Any, Dict

from ecav.core.application.edge.edge_manager.edge_manager_base import (
    _BaseEdgeManager)
from ecav.core.application.edge.fusion import get_fusion
from ecav.core.tracking import get_tracker
from ecav.core.application.edge.track_utils import ab3d_tracks_to_trajectories
from ecav.core.application.edge.latency import JitterBuffer
from ecav.core.application.edge.beacon_id_manager import BeaconIdManager
from ecav.core.application.edge.edge_profiler import EdgeProfiler
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory

logger = logging.getLogger(__name__)


class _PluggableEdgeBase(_BaseEdgeManager):
    """Common init and pipeline plumbing for SOTAEdge and AdaptiveEdge."""

    def __init__(self, world, cfg, cav_world, carla_client, *,
                 world_dt=0.05, **kwargs):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        # Fusion backend
        fusion_name = cfg.get('fusion_backend', 'late_fusion')
        fusion_kwargs = {}
        if fusion_name.upper() == 'ORACLE':
            fusion_kwargs['world'] = world
        self.fusion = get_fusion(fusion_name, cfg, **fusion_kwargs)
        logger.info("[%s] Fusion: %s", self._label, fusion_name)

        # Tracker
        tracker_name = cfg.get('tracker', 'ab3dmot')
        tracker_cfg = cfg.get('tracker_cfg', {})
        tracker_cfg.setdefault('anchoring', cfg.get('anchoring', True))
        self.tracker = get_tracker(tracker_name, tracker_cfg)
        self.anchoring = tracker_cfg.get('anchoring', True)
        logger.info("[%s] Tracker: %s", self._label, tracker_name)

        # Beacon ID manager
        self.beacon_id_mgr = BeaconIdManager(
            rotation_interval_ticks=cfg.get(
                'beacon_id_rotation_interval', 200),
            rotation_distance_m=cfg.get(
                'beacon_id_rotation_distance', 100.0),
            world_dt=world_dt)

        # Jitter buffer
        self._jitter_buffer = JitterBuffer(capacity=200)

        # Track state
        self.tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self.track_to_carla: Dict[int, int] = {}
        self._tracker_output_history = deque(maxlen=30)
        self._last_update_tick = -1
        self._latest_source_tick = None

        # Profiler
        self.profiler = EdgeProfiler(
            intersection_id=cfg.get(
                'intersection_id', f"{self._label}_{self.edgeid}"))

        # GT snapshots for metrics
        self._gt_snapshots: Dict[int, Dict] = {}

    @property
    def _label(self) -> str:
        return self.__class__.__name__

    def start_edge(self):
        for vm in self.vehicle_manager_list:
            vm.agent._anchoring = self.anchoring

    def update_information(self, frame_idx):
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx
        self.fusion.collect_and_push(
            frame_idx,
            self.vehicle_manager_list,
            self.rsu_manager_list,
            self._jitter_buffer,
            self.latency_model,
            mac_model=self.mac_model,
            beacon_id_mgr=self.beacon_id_mgr,
            world=self.world)

    def _drain_and_track(self, tick, frame):
        """Drain jitter buffer, run detection + tracking, build trajectories."""
        new_frames = self._jitter_buffer.drain(tick)
        for source_tick, payload in new_frames:
            dets = self.fusion.detect(
                payload, source_tick,
                beacon_id_mgr=self.beacon_id_mgr,
                vehicle_managers=self.vehicle_manager_list)
            tracks, _ = self.tracker.track(dets, source_tick)
            if tracks and len(tracks[0]) > 0:
                self._tracker_output_history.append(tracks[0])
            self._latest_source_tick = source_tick

        ab3d_tracks_to_trajectories(
            self._tracker_output_history,
            self.tracked_trajectories,
            self.track_to_carla,
            horizon=30,
            dt=self.dt,
            beacon_id_mgr=self.beacon_id_mgr,
            anchoring=self.anchoring)

        return len(self.tracked_trajectories)

    def _advance_vehicles(self, tick, predictions):
        """Push predictions to vehicles and advance simulation."""
        for vm in self.vehicle_manager_list:
            if predictions and random.random() * 100 > self.downlink_pl:
                vm.agent.edge_predictions = list(predictions)
            else:
                vm.agent.edge_predictions = []
            if not self.run_distributed:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())
                self._label_brake_attributions_gt(vm)

        for rsu in self.rsu_manager_list:
            if not self.run_distributed:
                rsu.update_info()
                rsu.run_step()
