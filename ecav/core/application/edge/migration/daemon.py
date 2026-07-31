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
        dst_edge.import_tracked_obstacle_state(carla_id, payload)
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
