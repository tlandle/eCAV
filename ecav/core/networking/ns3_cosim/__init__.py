# -*- coding: utf-8 -*-
"""
ns-3 co-simulation bridge for NR V2X network modeling.

Two operating modes:
    "sidelink" (V2V direct): NR V2X Mode 2 sidelink. Vehicles communicate
        directly with autonomous resource selection. PRR degrades at high N
        due to MAC collisions. Used by V2V managers.

    "uu_uplink" (vehicle-to-edge): NR Uu uplink to gNB. Centralized
        scheduling by gNB. Used by edge managers for Uu link modeling.

Provides shared-memory IPC between the CARLA simulation (Python) and
a persistent ns-3 NR V2X process (C++). Each simulation tick, vehicle
positions are written to shared memory, ns-3 runs the NR MAC, and
per-link delivery results are read back.

Falls back to the analytical channel engine if ns-3 is not available.
"""

from .ns3_bridge import NS3Bridge, LinkResult
from .shm_layout import compute_shm_size, MAX_VEHICLES_DEFAULT

__all__ = [
    "NS3Bridge", "LinkResult",
    "get_network_engine", "get_v2v_engine", "get_edge_engine",
]


def get_network_engine(cfg: dict = None, **kwargs):
    """
    Factory: returns NS3Bridge if the binary exists, else analytical fallback.

    Config keys:
        network_engine: "ns3" | "analytical" | "auto" (default "auto")
        ns3_binary: path to nr_v2x_cosim_server binary
        max_vehicles: max vehicle count (default 128)
        mode: "sidelink" | "uu_uplink" (default "sidelink")
    """
    cfg = cfg or {}
    engine_type = cfg.get("network_engine", kwargs.get("network_engine", "auto"))

    if engine_type == "analytical":
        from ecav.core.networking.channel_engine_py import get_channel_engine
        return get_channel_engine(**kwargs)

    if engine_type in ("ns3", "auto"):
        ns3_binary = cfg.get("ns3_binary", kwargs.get("ns3_binary"))
        mode = cfg.get("mode", kwargs.get("mode", "sidelink"))
        try:
            bridge = NS3Bridge(
                ns3_binary=ns3_binary,
                max_vehicles=cfg.get("max_vehicles", MAX_VEHICLES_DEFAULT),
                mode=mode,
                **{k: v for k, v in kwargs.items()
                   if k in ("carrier_ghz", "bandwidth_mhz", "tx_power_dbm",
                            "numerology", "tick_duration_s")},
            )
            return bridge
        except Exception as e:
            if engine_type == "ns3":
                raise
            import logging
            logging.getLogger("NS3Bridge").warning(
                "ns-3 bridge not available (%s), using analytical fallback", e)
            from ecav.core.networking.channel_engine_py import get_channel_engine
            return get_channel_engine(**kwargs)

    from ecav.core.networking.channel_engine_py import get_channel_engine
    return get_channel_engine(**kwargs)


def get_v2v_engine(cfg: dict = None, **kwargs):
    """Factory specifically for V2V direct sidelink mode."""
    cfg = cfg or {}
    cfg.setdefault("mode", "sidelink")
    return get_network_engine(cfg, **kwargs)


def get_edge_engine(cfg: dict = None, **kwargs):
    """Factory specifically for Uu uplink (vehicle-to-edge) mode."""
    cfg = cfg or {}
    cfg.setdefault("mode", "uu_uplink")
    return get_network_engine(cfg, **kwargs)
