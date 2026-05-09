# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Unified C-V2X network model for cooperative driving architectures.

Models the complete communication substrate available to C-V2X equipped
vehicles, including both radio interfaces and the network infrastructure.

A C-V2X OBU has two radio interfaces:
  - PC5 sidelink: direct V2V broadcast, contention-based (SB-SPS)
  - Uu interface: scheduled cellular uplink/downlink to gNB

Three cooperative architectures use these interfaces differently:

  Scenario A (V2V PC5):
    Vehicles broadcast features on PC5 sidelink.
    Contention-based. PRR degrades with N.
    No network infrastructure involved.

  Scenario B (V2N2V relay):
    Vehicles upload features via Uu to gNB.
    gNB relays features to other vehicles via Uu downlink.
    Scheduled access, no contention.
    BUT: N*(N-1) relay flows through backhaul.

  Scenario C (Edge compute):
    Vehicles upload features via Uu to edge server.
    Edge fuses once, tracks once, predicts once.
    Returns compact results via Uu downlink.
    N uplinks + N small downlinks.

This module models all three paths with realistic parameters.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("V2XNetworkModel")


# ──────────────────────────────────────────────────────────────────────
#  Physical layer / system parameters
# ──────────────────────────────────────────────────────────────────────

@dataclass
class V2XNetworkConfig:
    """Configuration for the unified V2X network model."""

    # --- PC5 sidelink (V2V direct) ---
    pc5_bandwidth_mhz: float = 10.0         # ITS 5.9 GHz band allocation
    pc5_num_subchannels: int = 20            # SB-SPS resource pool
    pc5_tx_power_dbm: float = 23.0
    pc5_effective_rate_mbps: float = 6.0     # after MAC overhead
    pc5_slot_duration_ms: float = 1.0        # one TTI

    # --- Uu interface (cellular) ---
    uu_uplink_bandwidth_mhz: float = 10.0    # scheduled uplink allocation
    uu_downlink_bandwidth_mhz: float = 20.0  # typically 2x uplink
    uu_uplink_rate_mbps: float = 50.0        # peak rate per cell
    uu_downlink_rate_mbps: float = 100.0     # peak rate per cell
    uu_air_interface_ms: float = 4.0         # one-way Uu latency (HARQ)

    # --- Backhaul (gNB to edge/core) ---
    backhaul_bandwidth_mbps: float = 1000.0  # 1 Gbps fiber typical
    backhaul_latency_ms: float = 2.0         # ~2ms wired backhaul

    # --- Edge server ---
    edge_compute_ms: float = 50.0            # fusion + tracking + prediction

    # --- Payload sizes ---
    feature_size_bytes: int = 200_000        # intermediate fusion BEV features
    bsm_size_bytes: int = 300                # basic safety message
    result_size_bytes: int = 5_000           # edge result (objects + trajectories)
    object_share_size_bytes: int = 50_000    # AutoCast-style object point cloud

    # --- Update rates ---
    update_rate_hz: float = 10.0             # cooperative perception update rate


@dataclass
class LinkBudget:
    """Per-tick network budget calculation for a given architecture and N."""
    n_vehicles: int = 0
    architecture: str = ""

    # Offered load
    uplink_offered_mbps: float = 0.0
    downlink_offered_mbps: float = 0.0
    backhaul_offered_mbps: float = 0.0

    # Capacity
    uplink_capacity_mbps: float = 0.0
    downlink_capacity_mbps: float = 0.0
    backhaul_capacity_mbps: float = 0.0

    # Utilization (0-1, >1 means overloaded)
    uplink_utilization: float = 0.0
    downlink_utilization: float = 0.0
    backhaul_utilization: float = 0.0

    # Per-vehicle effective throughput
    per_vehicle_uplink_mbps: float = 0.0
    per_vehicle_downlink_mbps: float = 0.0

    # Latency components (ms)
    upload_latency_ms: float = 0.0
    backhaul_latency_ms: float = 0.0
    compute_latency_ms: float = 0.0
    download_latency_ms: float = 0.0
    total_e2e_latency_ms: float = 0.0

    # Feasibility
    feasible: bool = True
    bottleneck: str = ""
    prr: float = 1.0  # for PC5 path


class V2XNetworkModel:
    """
    Computes link budgets and feasibility for cooperative driving architectures.

    Given N vehicles and a payload configuration, determines whether each
    architecture can sustain the required update rate and what the E2E
    latency looks like.
    """

    def __init__(self, cfg: V2XNetworkConfig = None):
        self.cfg = cfg or V2XNetworkConfig()

    def compute_v2v_pc5_budget(self, N: int) -> LinkBudget:
        """
        Scenario A: V2V broadcast on PC5 sidelink.

        Each vehicle broadcasts features. All others receive (if no collision).
        Channel capacity is shared. SB-SPS contention degrades PRR.
        """
        cfg = self.cfg
        budget = LinkBudget(n_vehicles=N, architecture="V2V_PC5")

        # Offered load: N vehicles each broadcasting
        per_vehicle_mbps = (cfg.feature_size_bytes * 8 / 1e6) * cfg.update_rate_hz
        budget.uplink_offered_mbps = N * per_vehicle_mbps
        budget.uplink_capacity_mbps = cfg.pc5_effective_rate_mbps

        budget.uplink_utilization = budget.uplink_offered_mbps / max(
            budget.uplink_capacity_mbps, 0.001)

        # SB-SPS collision probability
        # P(at least one collision on a given slot) = 1 - ((M-1)/M)^(N-1)
        M = cfg.pc5_num_subchannels
        if N > 1:
            p_collision = 1.0 - ((M - 1) / M) ** (N - 1)
        else:
            p_collision = 0.0
        budget.prr = max(0.0, 1.0 - p_collision)

        # Per-vehicle effective throughput
        budget.per_vehicle_uplink_mbps = per_vehicle_mbps
        budget.per_vehicle_downlink_mbps = 0  # no downlink in pure V2V

        # Latency: just the air interface (1 TTI for broadcast)
        tx_time_ms = (cfg.feature_size_bytes * 8) / (
            cfg.pc5_effective_rate_mbps * 1e6) * 1000
        budget.upload_latency_ms = tx_time_ms
        budget.total_e2e_latency_ms = tx_time_ms

        # No backhaul, no compute, no downlink
        budget.backhaul_capacity_mbps = 0
        budget.downlink_capacity_mbps = 0

        # Feasibility
        if budget.prr < 0.5:
            budget.feasible = False
            budget.bottleneck = f"PC5 PRR={budget.prr:.2f} (SB-SPS collision)"
        elif budget.uplink_utilization > 1.0:
            budget.feasible = False
            budget.bottleneck = f"PC5 overloaded ({budget.uplink_utilization:.1f}x)"

        return budget

    def compute_v2n2v_budget(self, N: int) -> LinkBudget:
        """
        Scenario B: V2N2V relay via Uu.

        Each vehicle uploads features via Uu. gNB relays to N-1 others.
        Scheduled access (no contention). Backhaul carries relay traffic.
        """
        cfg = self.cfg
        budget = LinkBudget(n_vehicles=N, architecture="V2N2V_Relay")

        per_vehicle_mbps = (cfg.feature_size_bytes * 8 / 1e6) * cfg.update_rate_hz

        # Uplink: N vehicles each upload once
        budget.uplink_offered_mbps = N * per_vehicle_mbps
        budget.uplink_capacity_mbps = cfg.uu_uplink_rate_mbps
        budget.uplink_utilization = budget.uplink_offered_mbps / max(
            budget.uplink_capacity_mbps, 0.001)

        # Backhaul: gNB must forward each vehicle's features to N-1 others
        # Total relay: N * (N-1) * feature_size * rate
        relay_flows = N * (N - 1)
        budget.backhaul_offered_mbps = relay_flows * per_vehicle_mbps
        budget.backhaul_capacity_mbps = cfg.backhaul_bandwidth_mbps
        budget.backhaul_utilization = budget.backhaul_offered_mbps / max(
            budget.backhaul_capacity_mbps, 0.001)

        # Downlink: each vehicle receives N-1 feature sets
        budget.downlink_offered_mbps = N * (N - 1) * per_vehicle_mbps
        budget.downlink_capacity_mbps = cfg.uu_downlink_rate_mbps
        budget.downlink_utilization = budget.downlink_offered_mbps / max(
            budget.downlink_capacity_mbps, 0.001)

        # Per-vehicle throughput
        budget.per_vehicle_uplink_mbps = min(
            per_vehicle_mbps,
            cfg.uu_uplink_rate_mbps / max(N, 1))
        budget.per_vehicle_downlink_mbps = min(
            (N - 1) * per_vehicle_mbps,
            cfg.uu_downlink_rate_mbps / max(N, 1))

        # Latency: Uu uplink + backhaul relay + Uu downlink
        # Upload time depends on per-vehicle allocated bandwidth
        allocated_up = cfg.uu_uplink_rate_mbps / max(N, 1)
        upload_ms = (cfg.feature_size_bytes * 8) / (allocated_up * 1e6) * 1000
        budget.upload_latency_ms = cfg.uu_air_interface_ms + upload_ms
        budget.backhaul_latency_ms = cfg.backhaul_latency_ms
        budget.download_latency_ms = cfg.uu_air_interface_ms + upload_ms  # same size
        budget.total_e2e_latency_ms = (budget.upload_latency_ms
                                       + budget.backhaul_latency_ms
                                       + budget.download_latency_ms)

        budget.prr = 1.0  # scheduled, no contention

        # Feasibility
        bottlenecks = []
        if budget.uplink_utilization > 1.0:
            bottlenecks.append(f"Uu uplink ({budget.uplink_utilization:.1f}x)")
        if budget.backhaul_utilization > 1.0:
            bottlenecks.append(f"backhaul relay ({budget.backhaul_utilization:.1f}x)")
        if budget.downlink_utilization > 1.0:
            bottlenecks.append(f"Uu downlink ({budget.downlink_utilization:.1f}x)")
        if bottlenecks:
            budget.feasible = False
            budget.bottleneck = " + ".join(bottlenecks)

        return budget

    def compute_edge_budget(self, N: int,
                            edge_compute_ms: float = None) -> LinkBudget:
        """
        Scenario C: Centralized edge compute via Uu.

        Vehicles upload features via Uu. Edge fuses, tracks, predicts.
        Returns compact results via Uu downlink.
        """
        cfg = self.cfg
        compute_ms = edge_compute_ms or cfg.edge_compute_ms
        budget = LinkBudget(n_vehicles=N, architecture="Edge_Compute")

        per_vehicle_up_mbps = (cfg.feature_size_bytes * 8 / 1e6) * cfg.update_rate_hz
        per_vehicle_down_mbps = (cfg.result_size_bytes * 8 / 1e6) * cfg.update_rate_hz

        # Uplink: N vehicles upload features
        budget.uplink_offered_mbps = N * per_vehicle_up_mbps
        budget.uplink_capacity_mbps = cfg.uu_uplink_rate_mbps
        budget.uplink_utilization = budget.uplink_offered_mbps / max(
            budget.uplink_capacity_mbps, 0.001)

        # Backhaul: N uploads to edge + N downloads from edge
        budget.backhaul_offered_mbps = (N * per_vehicle_up_mbps
                                        + N * per_vehicle_down_mbps)
        budget.backhaul_capacity_mbps = cfg.backhaul_bandwidth_mbps
        budget.backhaul_utilization = budget.backhaul_offered_mbps / max(
            budget.backhaul_capacity_mbps, 0.001)

        # Downlink: N vehicles receive compact results
        budget.downlink_offered_mbps = N * per_vehicle_down_mbps
        budget.downlink_capacity_mbps = cfg.uu_downlink_rate_mbps
        budget.downlink_utilization = budget.downlink_offered_mbps / max(
            budget.downlink_capacity_mbps, 0.001)

        # Per-vehicle throughput
        budget.per_vehicle_uplink_mbps = min(
            per_vehicle_up_mbps,
            cfg.uu_uplink_rate_mbps / max(N, 1))
        budget.per_vehicle_downlink_mbps = min(
            per_vehicle_down_mbps,
            cfg.uu_downlink_rate_mbps / max(N, 1))

        # Latency
        allocated_up = cfg.uu_uplink_rate_mbps / max(N, 1)
        upload_ms = (cfg.feature_size_bytes * 8) / (allocated_up * 1e6) * 1000
        budget.upload_latency_ms = cfg.uu_air_interface_ms + upload_ms
        budget.backhaul_latency_ms = cfg.backhaul_latency_ms * 2  # round trip
        budget.compute_latency_ms = compute_ms

        allocated_down = cfg.uu_downlink_rate_mbps / max(N, 1)
        download_ms = (cfg.result_size_bytes * 8) / (allocated_down * 1e6) * 1000
        budget.download_latency_ms = cfg.uu_air_interface_ms + download_ms

        budget.total_e2e_latency_ms = (budget.upload_latency_ms
                                       + budget.backhaul_latency_ms
                                       + budget.compute_latency_ms
                                       + budget.download_latency_ms)

        budget.prr = 1.0  # scheduled

        # Feasibility
        bottlenecks = []
        if budget.uplink_utilization > 1.0:
            bottlenecks.append(f"Uu uplink ({budget.uplink_utilization:.1f}x)")
        if budget.backhaul_utilization > 1.0:
            bottlenecks.append(f"backhaul ({budget.backhaul_utilization:.1f}x)")
        if budget.total_e2e_latency_ms > 100.0:
            bottlenecks.append(
                f"E2E latency ({budget.total_e2e_latency_ms:.0f}ms > 100ms)")
        if bottlenecks:
            budget.feasible = False
            budget.bottleneck = " + ".join(bottlenecks)

        return budget

    def sweep(self, n_values: List[int],
              edge_compute_fn=None) -> Dict[str, List[LinkBudget]]:
        """
        Compute link budgets for all three architectures across N values.

        Args:
            n_values: list of vehicle counts to evaluate
            edge_compute_fn: optional function N -> compute_ms for
                             modeling compute scaling with N

        Returns:
            Dict mapping architecture name to list of LinkBudget.
        """
        results = {"V2V_PC5": [], "V2N2V_Relay": [], "Edge_Compute": []}

        for N in n_values:
            results["V2V_PC5"].append(self.compute_v2v_pc5_budget(N))
            results["V2N2V_Relay"].append(self.compute_v2n2v_budget(N))

            compute_ms = edge_compute_fn(N) if edge_compute_fn else None
            results["Edge_Compute"].append(
                self.compute_edge_budget(N, compute_ms))

        return results

    @staticmethod
    def print_sweep(results: Dict[str, List[LinkBudget]]):
        """Pretty-print sweep results."""
        archs = list(results.keys())
        n_values = [b.n_vehicles for b in results[archs[0]]]

        header = f"{'N':>4}"
        for arch in archs:
            header += f" | {arch:>20} E2E(ms) PRR   Feas Bottleneck"
        print(header)
        print("-" * len(header))

        for i, N in enumerate(n_values):
            row = f"{N:>4}"
            for arch in archs:
                b = results[arch][i]
                feas = "OK" if b.feasible else "FAIL"
                bn = b.bottleneck[:25] if b.bottleneck else ""
                row += (f" | {arch:>20} {b.total_e2e_latency_ms:6.1f} "
                        f"{b.prr:.3f} {feas:>4} {bn}")
            print(row)
