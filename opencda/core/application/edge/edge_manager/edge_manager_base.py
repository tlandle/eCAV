# -*- coding: utf-8 -*-
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
edge_manager_base.py
====================

Common utilities used by *all* Edge-manager back-ends
(BM2CP, late-fusion, manoeuvre, …):

* global latency / packet-loss model (pandas-only CSV loader)
* shared debug-helper instances
* lightweight `_BaseEdgeManager` superclass that holds the lists of
  VehicleManager / RSUManager, plus:
    - add_member
    - add_rsu
    - set_destination
"""

from __future__ import annotations

import logging
import uuid
import weakref
import random
import math
import pathlib
import time
from collections import deque, defaultdict
from typing import Dict, List, Deque, Any

import numpy as np
import pandas as pd
import carla
from easydict import EasyDict as edict

from opencda.core.application.edge.edge_metrics import EdgeMetrics
from opencda.core.prediction.linear_predictor_manager   import LinearPredictorManager
from opencda.core.sensing.tracking.obstacle_trajectory  import ObstacleTrajectory
from opencda.core.sensing.perception.obstacle_vehicle   import ObstacleVehicle


logger = logging.getLogger("EdgeManager")

# ──────────────────────────────────────────────────────────────
#  Latency-trace loader (pandas-only version)
# ──────────────────────────────────────────────────────────────
_TRACE_FILE = pathlib.Path("merged_latency.csv")
if _TRACE_FILE.is_file():
    try:
        _C_V2X_TRACE_MS: np.ndarray = (
            pd.read_csv(_TRACE_FILE, usecols=["latency_ms"])
              .latency_ms
              .dropna()
              .to_numpy(dtype=np.float32)
        )
        logger.info("loaded %d C-V2X RTT samples", _C_V2X_TRACE_MS.size)
    except Exception as exc:
        logger.error("failed to parse merged_latency.csv: %s", exc)
        _C_V2X_TRACE_MS = None
else:
    logger.warning(
        "'merged_latency.csv' not found – radio RTT will be U(30–80 ms) fallback"
    )
    _C_V2X_TRACE_MS = None

# back-haul log-normal fit (from your paper / old code)
_BACKHAUL_MU, _BACKHAUL_SIGMA = 2.9957, 0.6556   # ms in log-space


def _draw_latency_ms(base_ms: float,
                     jitter_ms: float,
                     dist: str) -> float:
    """
    One random latency sample according to *latency_distribution*.

    Parameters
    ----------
    base_ms   : YAML 'latency' (already in ms)
    jitter_ms : YAML 'jitter_std' (ms, only used for 'normal')
    dist      : one of {'fixed','normal','lognormal','hybrid'}
    """
    dist = (dist or "fixed").lower()

    if dist in ("fixed", "none"):
        return base_ms

    if dist == "normal":
        return max(0.0, random.gauss(base_ms, jitter_ms))

    if dist == "lognormal":
        mu = math.log(max(base_ms, 1e-3))
        return float(np.random.lognormal(mu, 0.5))

    if dist == "hybrid":
        # — radio link RTT (one-way)
        if _C_V2X_TRACE_MS is not None and _C_V2X_TRACE_MS.size > 0:
            rtt_radio = float(np.random.choice(_C_V2X_TRACE_MS))
        else:
            rtt_radio = random.uniform(30.0, 80.0)
        # — back-haul RTT
        rtt_back = float(np.random.lognormal(_BACKHAUL_MU, _BACKHAUL_SIGMA))
        return rtt_radio + rtt_back + base_ms

    raise ValueError(f"unknown latency_distribution '{dist}'")


# ──────────────────────────────────────────────────────────────
#  Minimal shared base-class
# ──────────────────────────────────────────────────────────────
class _BaseEdgeManager:
    """
    Functionality identical for every Edge-manager variant:

    * bookkeeping (vehicle / RSU lists, CavWorld registration)
    * latency / loss parameters parsed from YAML
    * shared debug helper
    * default hooks: add_member, add_rsu, set_destination
    """

    def __init__(
        self,
        world: carla.World,
        cfg:   Dict[str, Any],
        cav_world,
        carla_client: carla.Client,
        *,
        world_dt: float = 0.05,
        **kwargs,
    ):
        self.edgeid = str(uuid.uuid4())[:8]
        self.world  = world
        self.carla  = carla_client
        self.dt     = world_dt

        # lists of VehicleManagers / RSUManagers
        self.vehicle_manager_list: List[Any] = []
        self.rsu_manager_list:     List[Any] = []

        # latency / packet-loss (ms or %)
        self.latency_ms  = float(cfg.get("latency",        0) * 1000)
        self.jitter_ms   = float(cfg.get("jitter_std",     0) * 1000)
        self.lat_dist    = cfg.get("latency_distribution", "fixed").lower()
        self.uplink_pl   = float(cfg.get("uplink_packet_loss_pct",   0))
        self.downlink_pl = float(cfg.get("downlink_packet_loss_pct", 0))

        # shared predictor helper (used by some back-ends)
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # global debug helper for perf / viz
        self.debug = EdgeMetrics(0)

        # distributed mode flag (from cav_world)
        self.run_distributed = getattr(cav_world, 'run_distributed', False) if cav_world else False

        # register with CavWorld
        weakref.ref(cav_world)().update_edge(self)

    # ─── abstract hooks every backend must override ────────────
    def start_edge(self):                     raise NotImplementedError
    def update_information(self, frame_idx):  raise NotImplementedError
    def run_step(self, tick):                 raise NotImplementedError

    # ─── convenience for latency sampling ─────────────────────
    def _sample_latency_ms(self) -> float:
        return _draw_latency_ms(self.latency_ms,
                                self.jitter_ms,
                                self.lat_dist)

    # ─── helpers common to all concrete back-ends ────────────
    def add_member(self, vm: Any) -> None:
        """Register a new VehicleManager under this edge."""
        self.vehicle_manager_list.append(vm)

    def add_rsu(self, rsu: Any) -> None:
        """Register a new RSUManager under this edge."""
        self.rsu_manager_list.append(rsu)

    def set_destination(self, destination: carla.Location) -> None:
        """
        Called by sim_api to tell the edge where its goal is.
        Store it for use in your edge logic.
        """
        self.destination = destination

    @staticmethod
    def _dict_extend(dest: Dict[str, list], src: Dict[str, list]) -> None:
        """`dest[k] += src[k]` for every key, creating lists if needed."""
        for k, v in src.items():
            dest.setdefault(k, []).extend(v)


# ──────────────────────────────────────────────────────────────
#  Compatibility aliases (import convenience)
# ──────────────────────────────────────────────────────────────
BaseEdgeManager = _BaseEdgeManager
EdgeManagerBase = _BaseEdgeManager

__all__ = ["_BaseEdgeManager", "BaseEdgeManager", "EdgeManagerBase"]
