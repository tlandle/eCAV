"""Smoke test: SequentialMigrationDaemon — no CARLA required.

Verifies the full mechanism + measurement chain introduced in Phase 1:

  1. Store round-trip  — store_vehicle_state / retrieve_vehicle_state.
  2. Export→import    — AB3DMOT KF state (x, P, hits, anchoring_age) preserved.
  3. Ownership moved  — VM leaves src.vehicle_manager_list, arrives at dst.
  4. TransferCost     — one record emitted, fields match payload + latency.
  5. Cost sensitivity — higher serialize_rate → higher sim_serialize_ms.
  6. Fallback export  — daemon handles empty store via direct src export.

Usage:
    conda run -n ecav310 python tests/test_edge_state_handoff.py
    conda run -n ecav310 pytest tests/test_edge_state_handoff.py -v
"""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import numpy as np
from AB3DMOT_libs.kalman_filter import KF

from ecav.core.application.edge.migration.daemon import SequentialMigrationDaemon
from ecav.core.application.edge.migration.link import InterLocaleLink
from ecav.core.application.edge.migration.payload import (
    KFState,
    MigrationPayload,
    TrackLatent,
)
from ecav.core.application.edge.latency.latency_model import FixedLatencyModel


# ──────────────────────────────────────────────────────────────
#  Stubs — minimal edge and store, no CARLA
# ──────────────────────────────────────────────────────────────

class _StubVehicle:
    def __init__(self, vehicle_id: int) -> None:
        self.id = vehicle_id


class _StubVM:
    def __init__(self, vehicle_id: int) -> None:
        self.vehicle = _StubVehicle(vehicle_id)


class _StubTrackerState:
    def __init__(self, id_start: int = 0, min_hits: int = 3) -> None:
        self.trackers = []
        self.ID_count = [id_start]
        self.min_hits = min_hits


class _StubEdge:
    """Minimal edge implementing the snapshot + ownership interface (no CARLA)."""

    def __init__(self, edge_id: str, tracker_id_start: int = 0) -> None:
        self.edgeid = edge_id
        self.vehicle_manager_list = []
        self._tracker = _StubTrackerState(id_start=tracker_id_start)
        self._track_to_carla: dict = {}

    def seed_vehicle(self, vehicle_id: int, *, state: np.ndarray,
                     hits: int = 5, anchoring_age: int = 2) -> None:
        """Insert a synthetic tracked vehicle into this stub edge."""
        info = np.array([0, -1, vehicle_id])
        kf_obj = KF(state[:7], info, self._tracker.ID_count[0])
        kf_obj.kf.x = state.reshape(10, 1).copy()
        kf_obj.kf.P = np.eye(10) * 2.0
        kf_obj.carla_id = vehicle_id
        kf_obj.hits = hits
        kf_obj.anchoring_age = anchoring_age
        tid = self._tracker.ID_count[0]
        self._tracker.trackers.append(kf_obj)
        self._tracker.ID_count[0] += 1
        self._track_to_carla[tid] = vehicle_id
        self.vehicle_manager_list.append(_StubVM(vehicle_id))

    def export_vehicle_state(self, vehicle_id: int) -> MigrationPayload:
        kf_obj = next(
            (t for t in self._tracker.trackers if t.carla_id == vehicle_id), None
        )
        kf_state = None
        if kf_obj is not None:
            kf_state = KFState(
                state_vector=kf_obj.kf.x.flatten().copy(),
                covariance=kf_obj.kf.P.copy(),
                hits=kf_obj.hits,
                anchoring_age=kf_obj.anchoring_age,
            )
        tid = next(
            (t for t, c in self._track_to_carla.items() if c == vehicle_id), -1
        )
        track = TrackLatent(
            track_id=tid,
            persistent_vehicle_id=vehicle_id,
            hidden_state=np.zeros(1, dtype=np.float16),
            kf_state=kf_state,
        )
        return MigrationPayload(
            source_locale_id=self.edgeid,
            destination_locale_id="",
            trigger_time_s=0.0,
            tracks=[track],
        )

    def import_vehicle_state(self, vehicle_id: int, payload: MigrationPayload) -> None:
        track = next(
            (t for t in payload.tracks if t.persistent_vehicle_id == vehicle_id), None
        )
        if track is None or track.kf_state is None:
            return
        ks = track.kf_state
        new_tid = self._tracker.ID_count[0]
        self._tracker.ID_count[0] += 1
        info = np.array([0, -1, vehicle_id])
        new_kf = KF(ks.state_vector[:7], info, new_tid)
        new_kf.kf.x = ks.state_vector.reshape(10, 1).copy()
        new_kf.kf.P = ks.covariance.copy()
        new_kf.carla_id = vehicle_id
        new_kf.hits = max(ks.hits, self._tracker.min_hits)
        new_kf.time_since_update = 0
        new_kf.anchoring_age = ks.anchoring_age
        self._tracker.trackers.append(new_kf)
        self._track_to_carla[new_tid] = vehicle_id

    def relinquish(self, vehicle_id: int) -> _StubVM:
        for i, vm in enumerate(self.vehicle_manager_list):
            if vm.vehicle.id == vehicle_id:
                return self.vehicle_manager_list.pop(i)
        raise KeyError(f"vehicle {vehicle_id} not owned by {self.edgeid}")

    def accept(self, vm: _StubVM) -> None:
        self.vehicle_manager_list.append(vm)


class _StubStore:
    def __init__(self) -> None:
        self._store: dict = {}

    def store_vehicle_state(self, vehicle_id: int, payload: MigrationPayload) -> None:
        self._store[vehicle_id] = payload

    def retrieve_vehicle_state(self, vehicle_id: int):
        return self._store.get(vehicle_id)


def _make_link(latency_ms: float = 5.0, rate: float = 1e-5) -> InterLocaleLink:
    return InterLocaleLink(FixedLatencyModel(mean_ms=latency_ms, dt=0.05),
                           serialize_rate_ms_per_byte=rate)


# ──────────────────────────────────────────────────────────────
#  Test cases
# ──────────────────────────────────────────────────────────────

VID = 99
STATE = np.array([10.0, 5.0, 0.0, 0.1, 4.5, 2.0, 1.8, 1.0, 0.5, 0.0])


def test_store_round_trip():
    src = _StubEdge("e0")
    src.seed_vehicle(VID, state=STATE)
    store = _StubStore()

    payload = src.export_vehicle_state(VID)
    store.store_vehicle_state(VID, payload)

    retrieved = store.retrieve_vehicle_state(VID)
    assert retrieved is payload, "Phase 1: store must return the same object (no real pickle)"
    assert retrieved.tracks[0].persistent_vehicle_id == VID


def test_export_import_state_preserved():
    src = _StubEdge("e0", tracker_id_start=0)
    src.seed_vehicle(VID, state=STATE, hits=5, anchoring_age=3)

    dst = _StubEdge("e1", tracker_id_start=10)
    payload = src.export_vehicle_state(VID)
    dst.import_vehicle_state(VID, payload)

    kf = next(t for t in dst._tracker.trackers if t.carla_id == VID)
    np.testing.assert_allclose(kf.kf.x.flatten(), STATE)
    np.testing.assert_allclose(kf.kf.P, np.eye(10) * 2.0)
    assert kf.hits >= dst._tracker.min_hits
    assert kf.anchoring_age == 3
    assert dst._track_to_carla[10] == VID


def test_ownership_moved():
    src = _StubEdge("e0")
    src.seed_vehicle(VID, state=STATE)
    dst = _StubEdge("e1", tracker_id_start=10)
    store = _StubStore()
    store.store_vehicle_state(VID, src.export_vehicle_state(VID))
    daemon = SequentialMigrationDaemon()

    daemon.request_handoff(VID, src, dst, store, _make_link(), tick=5)

    assert all(vm.vehicle.id != VID for vm in src.vehicle_manager_list), \
        "src should no longer own the vehicle"
    assert any(vm.vehicle.id == VID for vm in dst.vehicle_manager_list), \
        "dst should now own the vehicle"


def test_transfer_cost_recorded():
    src = _StubEdge("e0")
    src.seed_vehicle(VID, state=STATE)
    dst = _StubEdge("e1", tracker_id_start=10)
    store = _StubStore()
    store.store_vehicle_state(VID, src.export_vehicle_state(VID))
    daemon = SequentialMigrationDaemon()

    cost = daemon.request_handoff(VID, src, dst, store, _make_link(latency_ms=5.0), tick=7)

    assert len(daemon.costs) == 1
    assert cost.vehicle_id == VID
    assert cost.tick == 7
    assert cost.payload_bytes > 0
    assert cost.sim_network_ms == 5.0
    assert cost.total_ms > cost.sim_network_ms, "serialize overhead must add to total"


def test_cost_sensitive_to_serialize_rate():
    def run(rate: float) -> float:
        src = _StubEdge("e0")
        src.seed_vehicle(VID, state=STATE)
        dst = _StubEdge("e1", tracker_id_start=10)
        store = _StubStore()
        store.store_vehicle_state(VID, src.export_vehicle_state(VID))
        cost = daemon_single().request_handoff(VID, src, dst, store, _make_link(rate=rate), tick=0)
        return cost.sim_serialize_ms

    def daemon_single():
        return SequentialMigrationDaemon()

    slow = run(rate=1e-4)
    fast = run(rate=1e-5)
    assert slow > fast, f"higher rate must yield higher serialize_ms: slow={slow} fast={fast}"


def test_daemon_fallback_to_direct_export():
    """Daemon handles empty store by exporting directly from src."""
    src = _StubEdge("e0")
    src.seed_vehicle(VID, state=STATE)
    dst = _StubEdge("e1", tracker_id_start=10)
    store = _StubStore()  # intentionally empty
    daemon = SequentialMigrationDaemon()

    cost = daemon.request_handoff(VID, src, dst, store, _make_link(), tick=1)

    assert any(vm.vehicle.id == VID for vm in dst.vehicle_manager_list)
    assert cost.payload_bytes > 0


# ──────────────────────────────────────────────────────────────
#  Standalone entry point
# ──────────────────────────────────────────────────────────────

def main() -> int:
    tests = [
        test_store_round_trip,
        test_export_import_state_preserved,
        test_ownership_moved,
        test_transfer_cost_recorded,
        test_cost_sensitive_to_serialize_rate,
        test_daemon_fallback_to_direct_export,
    ]
    passed = 0
    for fn in tests:
        try:
            fn()
            print(f"  PASS  {fn.__name__}")
            passed += 1
        except Exception as exc:
            print(f"  FAIL  {fn.__name__}: {exc}")
    print(f"\n{passed}/{len(tests)} passed.")
    return 0 if passed == len(tests) else 1


if __name__ == "__main__":
    sys.exit(main())
