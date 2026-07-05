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
from typing import Dict, Optional

import numpy as np

from ecav.core.application.edge.edge_manager.edge_manager_base import (
    _BaseEdgeManager)
from ecav.core.application.edge.fusion import get_fusion
from ecav.core.tracking import get_tracker
from ecav.core.application.edge.track_utils import ab3d_tracks_to_trajectories
from ecav.core.application.edge.latency import JitterBuffer
from ecav.core.application.edge.beacon_id_manager import BeaconIdManager
from ecav.core.application.edge.edge_profiler import EdgeProfiler
from ecav.core.application.edge.migration.payload import KFState, MigrationPayload, TrackLatent
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from AB3DMOT_libs.kalman_filter import KF as _AB3DMOT_KF

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

    # ─── State-transfer overrides (backend-dispatched) ────────────────
    # self.tracker is a BaseTracker wrapper; the raw backend underneath is
    # either AB3DMOT (KFState snapshot path) or Mamba3DTracker (full-latent
    # path via migration.factories). Dispatch by duck-typing the raw tracker.
    def _raw_tracker(self):
        return getattr(self.tracker, 'tracker', self.tracker)

    @staticmethod
    def _is_mamba(raw) -> bool:
        return hasattr(raw, 'tracked_tracklets')

    def _export_track_latent(self, carla_id: int, tid: int) -> TrackLatent:
        """Build a TrackLatent for carla_id from whichever backend runs."""
        raw = self._raw_tracker()
        if self._is_mamba(raw):
            tracklet = next(
                (t for t in raw.tracked_tracklets
                 if getattr(t, 'carla_id', None) == carla_id),
                None,
            )
            if tracklet is not None:
                from ecav.core.application.edge.migration.factories import (
                    latent_from_tracklet)
                return latent_from_tracklet(
                    tracklet, persistent_vehicle_id=carla_id)
            logger.warning(
                "_export_track_latent: no Mamba tracklet for carla_id %d",
                carla_id)
            return TrackLatent(track_id=tid, persistent_vehicle_id=carla_id)

        kf_obj = next(
            (t for t in raw.trackers
             if getattr(t, 'carla_id', None) == carla_id),
            None,
        )
        kf_state = None
        if kf_obj is not None:
            kf_state = KFState(
                state_vector=kf_obj.kf.x.flatten().copy(),
                covariance=kf_obj.kf.P.copy(),
                hits=kf_obj.hits,
                anchoring_age=getattr(kf_obj, 'anchoring_age', 0),
            )
        return TrackLatent(
            track_id=tid,
            persistent_vehicle_id=carla_id,
            hidden_state=np.zeros(1, dtype=np.float16),
            kf_state=kf_state,
        )

    def _import_track_latent(self, carla_id: int, track: TrackLatent) -> None:
        """Inject a migrated TrackLatent into whichever backend runs.

        A record whose state does not match this backend (Mamba latent into
        an AB3DMOT edge, or KF-only into a Mamba edge) is logged and skipped;
        the destination cold-starts that track, per the fallback contract.
        """
        raw = self._raw_tracker()
        if self._is_mamba(raw):
            if track.memo_bank is None:
                logger.warning(
                    "_import_track_latent: no Mamba latent for carla_id %d "
                    "(schema mismatch) — cold start", carla_id)
                return
            from ecav.core.application.edge.migration.factories import (
                inject_latent_into_tracker)
            injected = inject_latent_into_tracker(raw, track)
            injected.carla_id = carla_id
            self.track_to_carla[int(injected.track_id)] = carla_id
            logger.info(
                "_import_track_latent: carla_id=%d -> mamba tid=%d "
                "(memo=%d frames)",
                carla_id, injected.track_id, len(injected.memo_bank))
            return

        if track.kf_state is None:
            logger.warning(
                "_import_track_latent: no KF state for carla_id %d "
                "(schema mismatch) — cold start", carla_id)
            return
        ks = track.kf_state
        new_tid = raw.ID_count[0]
        raw.ID_count[0] += 1
        info = np.array([0, -1, carla_id])
        new_kf = _AB3DMOT_KF(ks.state_vector[:7], info, new_tid)
        new_kf.kf.x = ks.state_vector.reshape(10, 1).copy()
        new_kf.kf.P = ks.covariance.copy()
        new_kf.carla_id = carla_id
        new_kf.hits = max(ks.hits, raw.min_hits)
        new_kf.time_since_update = 0
        new_kf.anchoring_age = ks.anchoring_age
        raw.trackers.append(new_kf)
        self.track_to_carla[new_tid] = carla_id
        logger.info(
            "_import_track_latent: carla_id=%d -> kf tid=%d (hits=%d, x=%.2f,%.2f)",
            carla_id, new_tid, new_kf.hits,
            float(new_kf.kf.x[0, 0]), float(new_kf.kf.x[1, 0]),
        )

    def export_vehicle_state(self, vehicle_id: int) -> Optional[MigrationPayload]:
        """Export tracker state for vehicle_id (Mamba latent or KF snapshot)."""
        if self._vm_by_carla_id(vehicle_id) is None:
            return None
        tid = next(
            (t for t, c in self.track_to_carla.items() if c == vehicle_id),
            -1,
        )
        track = self._export_track_latent(vehicle_id, tid)
        return MigrationPayload(
            source_locale_id="",
            destination_locale_id="",
            trigger_time_s=0.0,
            tracks=[track],
        )

    def import_vehicle_state(self, vehicle_id: int, payload: MigrationPayload) -> None:
        """Inject a warm track for vehicle_id into this edge's tracker."""
        track = next(
            (t for t in payload.tracks if t.persistent_vehicle_id == vehicle_id),
            None,
        )
        if track is None:
            logger.warning("import_vehicle_state: no record for vehicle %d", vehicle_id)
            return
        self._import_track_latent(vehicle_id, track)

    def accept(self, vm) -> None:
        """Add a VehicleManager to this edge and wire up anchoring."""
        super().accept(vm)
        vm.agent._anchoring = self.anchoring

    def export_tracked_obstacle_state(self, carla_id: int) -> Optional[MigrationPayload]:
        """Export tracker state for any tracked obstacle (no VehicleManager required)."""
        raw = self._raw_tracker()
        pool = raw.tracked_tracklets if self._is_mamba(raw) else raw.trackers
        if not any(getattr(t, 'carla_id', None) == carla_id for t in pool):
            return None
        tid = next((t for t, c in self.track_to_carla.items() if c == carla_id), -1)
        track = self._export_track_latent(carla_id, tid)
        return MigrationPayload(
            source_locale_id="",
            destination_locale_id="",
            trigger_time_s=0.0,
            tracks=[track],
        )

    def import_tracked_obstacle_state(self, carla_id: int, payload: MigrationPayload) -> None:
        """Inject a warm track for a tracked obstacle into this edge's tracker.

        No relinquish/accept — the source locale keeps tracking the obstacle
        concurrently. The destination gets a warm start without a
        confirmation dwell.
        """
        track = next(
            (t for t in payload.tracks if t.persistent_vehicle_id == carla_id), None
        )
        if track is None:
            logger.warning(
                "import_tracked_obstacle_state: no record for carla_id %d", carla_id
            )
            return
        self._import_track_latent(carla_id, track)
