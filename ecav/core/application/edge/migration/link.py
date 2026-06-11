# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""InterLocaleLink: simulated cost model for edge-to-edge state transfer.

Returns a TransferCost record that models the time to serialize, transmit,
and deserialize a MigrationPayload between two edges.

Decoupling invariant: this module is pure measurement. It never delays,
blocks, or gates the instant mechanism (store-pull + import). The daemon
records the TransferCost independently of the actual handoff.

Serialization cost is modeled from payload byte size and a configured
throughput rate — never from perf_counter around the real pickle call,
which would conflate simulator-host performance with the simulated
measurement.

Network cost is sampled from a LatencyModel. For Phase 1 the caller
typically passes the source edge's existing latency_model. Phase 2 will
substitute a per-link model derived from simulated edge geometry (e.g.
ns3_lut_sampler).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING

from ecav.core.application.edge.latency.latency_model import LatencyModel, create_latency_model
from ecav.core.application.edge.migration.payload import MigrationPayload

if TYPE_CHECKING:
    from ecav.core.application.edge.edge_manager.edge_manager_base import _BaseEdgeManager

logger = logging.getLogger(__name__)

# Default: ~100 MB/s effective throughput (serialize overhead per byte)
_DEFAULT_SERIALIZE_RATE_MS_PER_BYTE: float = 1e-5


@dataclass
class TransferCost:
    """Simulated cost record for one edge-to-edge vehicle-state transfer.

    Recorded by the daemon after each handoff; never fed back into the
    mechanism path.
    """
    vehicle_id: int
    src_edge_id: str
    dst_edge_id: str
    tick: int
    payload_bytes: int
    sim_serialize_ms: float   # cost to serialize at source (and symmetric deserialize at dst)
    sim_network_ms: float     # one-way simulated network latency
    total_ms: float           # sim_serialize_ms + sim_network_ms + sim_serialize_ms


class InterLocaleLink:
    """Models the simulated transfer cost between two locales.

    Parameters
    ----------
    latency_model:
        Drives the simulated network hop. For Phase 1 pass the source
        edge's ``latency_model`` directly. Phase 2 will substitute a
        geometry-parameterized per-link model.
    serialize_rate_ms_per_byte:
        Simulated serialization throughput in ms/byte. Default 1e-5
        (≈100 MB/s). Tune via scenario config to explore sensitivity.
    """

    def __init__(
        self,
        latency_model: LatencyModel,
        serialize_rate_ms_per_byte: float = _DEFAULT_SERIALIZE_RATE_MS_PER_BYTE,
    ):
        self._latency_model = latency_model
        self._rate = serialize_rate_ms_per_byte

    @classmethod
    def from_cfg(
        cls,
        cfg: dict,
        dt: float,
        serialize_rate_ms_per_byte: float = _DEFAULT_SERIALIZE_RATE_MS_PER_BYTE,
    ) -> "InterLocaleLink":
        """Convenience factory: build from an edge YAML config block."""
        return cls(create_latency_model(cfg, dt), serialize_rate_ms_per_byte)

    def model_transfer(
        self,
        payload: MigrationPayload,
        src_edge: "_BaseEdgeManager",
        dst_edge: "_BaseEdgeManager",
        tick: int,
    ) -> TransferCost:
        """Return a TransferCost for moving payload from src_edge to dst_edge.

        Pure model — no I/O, no state mutation, does not gate the handoff.
        """
        n_bytes = payload.payload_bytes()
        sim_serialize_ms = n_bytes * self._rate
        sim_network_ms = self._latency_model.sample_ms()
        total_ms = sim_serialize_ms + sim_network_ms + sim_serialize_ms

        vid = payload.tracks[0].persistent_vehicle_id if payload.tracks else -1

        cost = TransferCost(
            vehicle_id=vid,
            src_edge_id=src_edge.edgeid,
            dst_edge_id=dst_edge.edgeid,
            tick=tick,
            payload_bytes=n_bytes,
            sim_serialize_ms=sim_serialize_ms,
            sim_network_ms=sim_network_ms,
            total_ms=total_ms,
        )
        logger.debug(
            "model_transfer: vid=%d %s->%s tick=%d bytes=%d "
            "serialize=%.4fms network=%.4fms total=%.4fms",
            vid, src_edge.edgeid, dst_edge.edgeid, tick, n_bytes,
            sim_serialize_ms, sim_network_ms, total_ms,
        )
        return cost
