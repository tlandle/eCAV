# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""SequentialMigrationDaemon: single-process vehicle hand-off coordinator.

Wires the mechanism path (instant store-pull + ownership move) with the
measurement path (InterLocaleLink.model_transfer records a TransferCost)
without conflating the two.

Phase 1 (this module) — sequential single-process:
  The state store is a dict in sim_api.ScenarioManager.
  The ownership move is a pair of in-process member-function calls.

Phase 2 (-eo) upgrade path:
  Swap store  → registration-server do-tick state array.
  Swap ownership ping → peer RPC.

Phase 3 (C++) upgrade path:
  Swap store → C++ shared buffer.
  TransferCost + InterLocaleLink are unchanged across phases.
"""
from __future__ import annotations

import logging
from typing import TYPE_CHECKING, List, Optional, Protocol, runtime_checkable

from ecav.core.application.edge.migration.link import InterLocaleLink, TransferCost
from ecav.core.application.edge.migration.payload import MigrationPayload

if TYPE_CHECKING:
    from ecav.core.application.edge.edge_manager.edge_manager_base import _BaseEdgeManager

logger = logging.getLogger(__name__)


@runtime_checkable
class _StateStore(Protocol):
    """Duck-typed interface for the state store (sim_api.ScenarioManager in Phase 1)."""

    def retrieve_vehicle_state(self, vehicle_id: int) -> Optional[MigrationPayload]:
        ...


class SequentialMigrationDaemon:
    """Executes sequential vehicle hand-offs and records their simulated cost.

    Typical call site::

        daemon = SequentialMigrationDaemon()
        event = handoff_manager.evaluate(vid, tick, sim_time_s, ...)
        if event:
            cost = daemon.request_handoff(vid, src_edge, dst_edge, store, link, tick)
    """

    def __init__(self) -> None:
        self._costs: List[TransferCost] = []

    def request_handoff(
        self,
        vehicle_id: int,
        src_edge: "_BaseEdgeManager",
        dst_edge: "_BaseEdgeManager",
        store: _StateStore,
        link: InterLocaleLink,
        tick: int,
        import_state: bool = True,
    ) -> TransferCost:
        """Execute one vehicle hand-off and return a TransferCost record.

        Mechanism (instant — no real I/O):
          1. Pull snapshot from store (populated by per-tick snapshot writes).
             Falls back to a direct export from src if the store is empty.
          2. Relinquish vehicle from src (removes VM from vehicle_manager_list).
          3. Import tracker state into dst.
          4. Accept VM at dst (adds VM to vehicle_manager_list).

        Measurement (decoupled from mechanism — never gates it):
          5. Model transfer cost from payload bytes + latency sample.
        """
        # 1. Pull state — must happen before relinquish so export fallback
        #    can still find the VM in src's vehicle_manager_list.
        payload: Optional[MigrationPayload] = store.retrieve_vehicle_state(vehicle_id)
        if payload is None:
            logger.warning(
                "request_handoff: store has no snapshot for vehicle %d — exporting directly",
                vehicle_id,
            )
            payload = src_edge.export_vehicle_state(vehicle_id)

        # 2. Remove vehicle from src ownership.
        vm = src_edge.relinquish(vehicle_id)
        logger.info(
            "request_handoff: vehicle=%d tick=%d %s -> %s",
            vehicle_id, tick, src_edge.edgeid, dst_edge.edgeid,
        )

        # 3. Inject tracker state into dst. ``import_state=False`` is the
        #    cold-start experimental arm: ownership moves, state does not.
        if payload is not None and import_state:
            dst_edge.import_vehicle_state(vehicle_id, payload)
        elif not import_state:
            logger.info(
                "request_handoff: COLD arm — state import skipped for vehicle %d",
                vehicle_id,
            )
        else:
            logger.warning(
                "request_handoff: no payload for vehicle %d — ownership moved, tracker not warm",
                vehicle_id,
            )

        # 4. Transfer VM ownership to dst.
        dst_edge.accept(vm)

        # 5. Model cost (measurement — pure model, decoupled from mechanism).
        effective_payload = payload or MigrationPayload(
            source_locale_id="", destination_locale_id="",
            trigger_time_s=0.0, tracks=[],
        )
        cost = link.model_transfer(effective_payload, src_edge, dst_edge, tick)
        self._costs.append(cost)
        return cost

    def transfer_obstacle_state(
        self,
        carla_id: int,
        src_edge: "_BaseEdgeManager",
        dst_edge: "_BaseEdgeManager",
        link: InterLocaleLink,
        tick: int,
        position=None,
    ) -> Optional[TransferCost]:
        """Share a tracked obstacle's KF state from src_edge to dst_edge.

        Unlike request_handoff, there is no ownership transfer — no VehicleManager
        is moved. Both edges continue tracking the obstacle independently after the
        call. Dst_edge gets a warm KF (hits >= min_hits) so it can immediately
        include the obstacle in predictions without a confirmation dwell.

        ``position`` is the obstacle's true (x, y); it lets the source edge locate
        the KF by position when the tracker has not labelled it with carla_id
        (unmanaged obstacles never beacon). Returns None (with a warning log) if
        src_edge has no KF for the obstacle.
        """
        payload = src_edge.export_tracked_obstacle_state(carla_id, position=position)
        if payload is None:
            logger.warning(
                "transfer_obstacle_state: no KF for carla_id=%d on edge %s",
                carla_id, src_edge.edgeid,
            )
            return None

        # ── Ownership epoch (T7): source PREPARE increments the per-track
        # epoch; the payload carries it; the destination installs it as a
        # shadow and commits (this call is prepare+commit for obstacles —
        # the two-step split with a separate commit lives in the vehicle
        # handoff; obstacle transfers are one-shot by design).
        import os as _os
        own_src = getattr(src_edge, 'ownership', None)
        own_dst = getattr(dst_edge, 'ownership', None)
        if own_src is None:
            from ecav.core.application.edge.migration.ownership import                 OwnershipManager
            own_src = src_edge.ownership = OwnershipManager(
                node_id=str(src_edge.edgeid))
        if own_dst is None:
            from ecav.core.application.edge.migration.ownership import                 OwnershipManager
            own_dst = dst_edge.ownership = OwnershipManager(
                node_id=str(dst_edge.edgeid))
        own_src.source_owns(carla_id)
        pe, _cur = own_src.source_prepare(carla_id)
        payload.epoch = pe

        # ── FAULT_MODE (T7): inject exactly one fault at this transfer.
        fault = _os.environ.get('FAULT_MODE', '')
        fired = getattr(self, '_fault_fired', False)
        if fault and not fired:
            self._fault_fired = True
            logger.warning("[FAULT] injecting %s at tid=%d epoch=%d",
                           fault, carla_id, pe)
            if fault == 'lost_prepare':
                # payload never arrives; source keeps ownership
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost
            if fault == 'dst_crash':
                # destination dies after prepare: shadow install then no commit
                own_dst.dest_prepare(carla_id, pe)
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost
            if fault == 'dup_commit':
                own_dst.dest_prepare(carla_id, pe)
                dst_edge.import_tracked_obstacle_state(carla_id, payload)
                own_dst.dest_commit(carla_id, pe)
                own_dst.dest_commit(carla_id, pe)  # duplicate
                own_src.source_commit(carla_id)
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost
            if fault == 'reorder':
                # commit observed before prepare: must degrade, not dual-publish
                ok = own_dst.dest_commit(carla_id, pe)
                assert not ok
                own_dst.dest_prepare(carla_id, pe)
                dst_edge.import_tracked_obstacle_state(carla_id, payload)
                own_dst.dest_commit(carla_id, pe)
                own_src.source_commit(carla_id)
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost
            if fault == 'lost_ack':
                # destination committed; source retries the whole transfer
                own_dst.dest_prepare(carla_id, pe)
                dst_edge.import_tracked_obstacle_state(carla_id, payload)
                own_dst.dest_commit(carla_id, pe)
                own_dst.dest_prepare(carla_id, pe)   # retry: stale, no-op
                own_dst.dest_commit(carla_id, pe)    # retry: idempotent
                own_src.source_commit(carla_id)
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost
            if fault == 'lost_commit':
                # prepare lands, commit message lost: source retains
                # authority (never released before ack) — degraded warm
                own_dst.dest_prepare(carla_id, pe)
                dst_edge.import_tracked_obstacle_state(carla_id, payload)
                cost = link.model_transfer(payload, src_edge, dst_edge, tick)
                self._costs.append(cost)
                return cost

        # TRANSFER_MODE=grpc (T8): send the real serialized bytes to the
        # relay process over loopback (netem-impaired when configured) and
        # measure per-phase wall time; the in-process import then proceeds.
        # parametric (default) keeps the modeled link only.
        if _os.environ.get('TRANSFER_MODE', 'parametric') == 'grpc':
            try:
                import time as _time
                import grpc as _grpc
                import sys as _sys
                if 'migration_pb2' not in _sys.modules:
                    _sys.path.insert(0, 'ecav/protos')
                import migration_pb2 as _mpb
                import migration_pb2_grpc as _mgrpc
                _ch = getattr(self, '_relay_ch', None)
                if _ch is None:
                    _ch = self._relay_ch = _grpc.insecure_channel(
                        _os.environ.get('RELAY_ADDR', '127.0.0.1:50771'))
                    self._relay_stub = _mgrpc.MigrationRelayStub(_ch)
                _blob = payload.serialize()
                _t0 = _time.monotonic()
                self._relay_stub.Prepare(_mpb.TransferChunk(
                    payload=_blob, actor_id=carla_id, epoch=pe,
                    schema_version=payload.schema_version), timeout=5.0)
                _t1 = _time.monotonic()
                self._relay_stub.Commit(_mpb.TransferChunk(
                    payload=b'', actor_id=carla_id, epoch=pe,
                    schema_version=payload.schema_version), timeout=5.0)
                _t2 = _time.monotonic()
                logger.info(
                    "[XFERROW] actor=%d epoch=%d bytes=%d prepare_ms=%.2f "
                    "commit_ms=%.2f", carla_id, pe, len(_blob),
                    (_t1 - _t0) * 1e3, (_t2 - _t1) * 1e3)
            except Exception:  # noqa: BLE001
                logger.exception("grpc transfer failed; parametric fallback")

        own_dst.dest_prepare(carla_id, pe)
        dst_edge.import_tracked_obstacle_state(carla_id, payload)
        own_dst.dest_commit(carla_id, pe)
        # FENCING=off baseline (T7): the source does not fence itself at
        # commit, so both sides stay publishable until the source would
        # naturally drop the track — the double-publish window the paper's
        # failure model admits. Default (fencing on) closes it atomically.
        if _os.environ.get('FENCING', 'on').lower() != 'off':
            own_src.source_commit(carla_id)
        cost = link.model_transfer(payload, src_edge, dst_edge, tick)
        self._costs.append(cost)
        logger.info(
            "[OBSTACLE_HANDOFF] carla_id=%d %s->%s tick=%d bytes=%d total_ms=%.3f",
            carla_id, src_edge.edgeid, dst_edge.edgeid,
            tick, cost.payload_bytes, cost.total_ms,
        )
        return cost

    @property
    def costs(self) -> List[TransferCost]:
        """All TransferCost records emitted by this daemon (newest last)."""
        return list(self._costs)
