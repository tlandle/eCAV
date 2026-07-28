# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""AB3DMOT-aware state-transfer mixin for edge managers.

Implements the migration snapshot contract (export/import of per-vehicle and
per-obstacle Kalman-filter state) for any edge manager whose tracker is
AB3DMOT. Mixed into ``PredictionLateFusionEdge``, ``WorldFusionEdge``,
``OracleEdge``, and ``_PluggableEdgeBase`` — previously these methods lived
only on ``_PluggableEdgeBase``, so the late-fusion manager silently fell back
to the tracker-agnostic base stubs (minimal payload, no-op import) and no
live handoff ever moved real KF state.

Host-class requirements (all satisfied by the AB3DMOT-based managers):
    self.tracker            AB3DMOT instance (``trackers``, ``ID_count``, ``min_hits``)
    self.track_to_carla     Dict[int, int] tid -> carla_id
    self.anchoring          bool, wired into accepted vehicles' agents
    self._vm_by_carla_id    from _BaseEdgeManager
    self.edgeid             for logging
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from AB3DMOT_libs.kalman_filter import KF as _AB3DMOT_KF
from ecav.core.application.edge.migration.payload import (
    KFState,
    MigrationPayload,
    TrackLatent,
)

logger = logging.getLogger(__name__)


class AB3DMOTStateTransferMixin:
    """Snapshot export/import + ownership hooks for AB3DMOT-backed edges."""

    # ─── KF lookup ───────────────────────────────────────────────
    def _has_tracker(self) -> bool:
        """Proxy-mode edges construct without a tracker; transfers are no-ops."""
        return getattr(self, 'tracker', None) is not None

    def _kf_for_carla_id(self, carla_id: int):
        """Resolve a carla_id to its live tracker KF.

        Camera-detection KFs are constructed with ``carla_id = -1``; the
        beacon-anchored identity lives in ``track_to_carla`` (tid -> cid), not
        on the KF. Try the direct field first (beacon-constructed / imported
        tracks), then resolve through the anchoring map via ``KF.id``.
        """
        kf_obj = next(
            (t for t in self.tracker.trackers if t.carla_id == carla_id), None
        )
        if kf_obj is not None:
            return kf_obj
        tids = {t for t, c in self.track_to_carla.items() if c == carla_id}
        if not tids:
            return None
        return next((t for t in self.tracker.trackers if t.id in tids), None)

    def _find_obstacle_kf(self, carla_id: int, position, max_dist_m: float):
        """Locate the tracker KF for an obstacle.

        Identity resolution first (:meth:`_kf_for_carla_id`). Unmanaged
        obstacles (Scenario B's NPC) never beacon, so when a true ``position``
        is supplied we fall back to the nearest KF within ``max_dist_m`` — the
        scenario knows where the obstacle is even though the edge's tracker
        does not label it.
        """
        kf_obj = self._kf_for_carla_id(carla_id)
        if kf_obj is not None or position is None:
            return kf_obj
        px, py = float(position[0]), float(position[1])
        best, best_d = None, max_dist_m
        for t in self.tracker.trackers:
            # kf.x layout: [x, y, z, theta, l, w, h, dx, dy, dz]
            # x[0]=CARLA_x, x[1]=CARLA_z(height), x[2]=CARLA_y(lateral)
            d = ((float(t.kf.x[0]) - px) ** 2 + (float(t.kf.x[2]) - py) ** 2) ** 0.5
            if d <= best_d:
                best, best_d = t, d
        return best

    def _kf_to_payload(self, kf_obj, carla_id: int) -> MigrationPayload:
        """Wrap one KF (or None) into a single-track MigrationPayload."""
        kf_state = None
        if kf_obj is not None:
            kf_state = KFState(
                state_vector=kf_obj.kf.x.flatten().copy(),
                covariance=kf_obj.kf.P.copy(),
                hits=kf_obj.hits,
                anchoring_age=kf_obj.anchoring_age,
            )
        tid = next(
            (t for t, c in self.track_to_carla.items() if c == carla_id), -1
        )
        track = TrackLatent(
            track_id=tid,
            persistent_vehicle_id=carla_id,
            hidden_state=np.zeros(1, dtype=np.float16),
            kf_state=kf_state,
        )
        return MigrationPayload(
            source_locale_id="",
            destination_locale_id="",
            trigger_time_s=0.0,
            tracks=[track],
        )

    def _inject_warm_kf(self, carla_id: int, ks: KFState) -> int:
        """Create a warm KF from a KFState snapshot; returns the new tid.

        Sets hits >= min_hits so the track appears in output immediately,
        without a confirmation-dwell wait. Assigns a fresh tid from this
        edge's own ID_count counter; carla_id is the stable cross-edge key.
        """
        new_tid = self.tracker.ID_count[0]
        self.tracker.ID_count[0] += 1
        info = np.array([0, -1, carla_id])
        new_kf = _AB3DMOT_KF(ks.state_vector[:7], info, new_tid)
        new_kf.kf.x = ks.state_vector.reshape(10, 1).copy()
        new_kf.kf.P = ks.covariance.copy()
        new_kf.carla_id = carla_id
        new_kf.hits = max(ks.hits, self.tracker.min_hits)
        new_kf.time_since_update = 0
        new_kf.anchoring_age = ks.anchoring_age
        self.tracker.trackers.append(new_kf)
        self.track_to_carla[new_tid] = carla_id
        return new_tid

    # ─── Managed-vehicle snapshot contract ───────────────────────
    def export_vehicle_state(self, vehicle_id: int) -> Optional[MigrationPayload]:
        """Export KF state for a managed vehicle from the AB3DMOT tracker."""
        if not self._has_tracker() or self._vm_by_carla_id(vehicle_id) is None:
            return None
        return self._kf_to_payload(self._kf_for_carla_id(vehicle_id), vehicle_id)

    def _warm_import_enabled(self) -> bool:
        """Destination-side KF injection is a Phase 1.5 concern.

        Injecting a snapshot track into a tracker that also sees the same
        object through its own sensors risks stale-duplicate ghosts until
        import-side reconciliation exists. Default OFF: transfers still
        export real payloads (mechanism + cost measurement intact); the
        destination tracker is left untouched. Enable per-edge by setting
        ``handoff_warm_import = True`` (e.g. from YAML) once Phase 1.5
        lands reconciliation.
        """
        return getattr(self, 'handoff_warm_import', False)

    def import_vehicle_state(self, vehicle_id: int, payload: MigrationPayload) -> None:
        """Inject a warm KF for a managed vehicle into this edge's tracker."""
        track = next(
            (t for t in payload.tracks if t.persistent_vehicle_id == vehicle_id),
            None,
        )
        if not self._has_tracker() or track is None or track.kf_state is None:
            logger.warning("import_vehicle_state: no KF state for vehicle %d", vehicle_id)
            return
        if not self._warm_import_enabled():
            logger.info(
                "import_vehicle_state: warm import disabled (Phase 1.5) — "
                "vehicle=%d payload received, tracker untouched", vehicle_id)
            return
        new_tid = self._inject_warm_kf(vehicle_id, track.kf_state)
        logger.info(
            "import_vehicle_state: vehicle=%d -> tid=%d (hits=%d)",
            vehicle_id, new_tid, self.tracker.trackers[-1].hits,
        )

    # ─── Obstacle snapshot contract (no ownership move) ──────────
    def export_tracked_obstacle_state(
        self, carla_id: int, position=None, max_dist_m: float = 15.0
    ) -> Optional[MigrationPayload]:
        """Export KF state for any AB3DMOT-tracked obstacle (no VehicleManager).

        ``carla_id`` is the logical id the transfer is keyed on (and asserted
        onto the destination track). ``position`` is the obstacle's true (x, y);
        it locates the source KF when the tracker has not labelled it.
        """
        if not self._has_tracker():
            return None
        kf_obj = self._find_obstacle_kf(carla_id, position, max_dist_m)
        if kf_obj is None:
            return None
        return self._kf_to_payload(kf_obj, carla_id)

    def import_tracked_obstacle_state(self, carla_id: int, payload: MigrationPayload) -> None:
        """Inject a warm KF for a tracked obstacle into this edge's tracker.

        No relinquish/accept — the source edge keeps tracking the obstacle
        concurrently; this edge gets a warm start.
        """
        track = next(
            (t for t in payload.tracks if t.persistent_vehicle_id == carla_id), None
        )
        if not self._has_tracker() or track is None or track.kf_state is None:
            logger.warning(
                "import_tracked_obstacle_state: no KF for carla_id %d", carla_id
            )
            return
        if not self._warm_import_enabled():
            logger.info(
                "import_tracked_obstacle_state: warm import disabled (Phase 1.5) — "
                "carla_id=%d payload received, tracker untouched", carla_id)
            return
        new_tid = self._inject_warm_kf(carla_id, track.kf_state)
        logger.info(
            "import_tracked_obstacle_state: carla_id=%d -> tid=%d (hits=%d)",
            carla_id, new_tid, self.tracker.trackers[-1].hits,
        )

    # ─── Ownership hook ──────────────────────────────────────────
    def accept(self, vm) -> None:
        """Add a VehicleManager to this edge and wire up anchoring."""
        super().accept(vm)
        vm.agent._anchoring = self.anchoring
