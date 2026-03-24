# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Pure-Python fallback for the C++ channel engine.
Used when the pybind11 module has not been compiled.
Functionally identical but ~50-100x slower for large N.

Provides the same API: ChannelEngine.compute_tick(positions, occlusion, tick)
"""
from __future__ import annotations

import logging
import math
import random
from dataclasses import dataclass
from typing import List

import numpy as np

logger = logging.getLogger("ChannelEnginePy")


@dataclass
class OcclusionInfo:
    building_blocked: bool = False
    num_vehicles_blocking: int = 0
    extra_loss_db: float = 0.0


@dataclass
class LinkResult:
    tx_id: int = 0
    rx_id: int = 0
    delivered: bool = False
    sinr_db: float = -999.0
    delay_ms: float = 1.0


class ChannelEngine:
    """Pure-Python C-V2X channel engine (fallback implementation)."""

    def __init__(
        self,
        carrier_ghz: float = 5.9,
        bw_mhz: float = 10.0,
        tx_power_dbm: float = 23.0,
        num_subchannels: int = 20,
        rc_min: int = 5,
        rc_max: int = 15,
        p_keep: float = 0.0,
        sinr_thresh_db: float = 0.0,
        antenna_h: float = 1.5,
        noise_floor_dbm: float = -95.0,
        shadow_std_los: float = 3.0,
        shadow_std_nlos: float = 4.0,
        rician_k_db: float = 3.0,
    ):
        self.carrier_ghz = carrier_ghz
        self.tx_power_dbm = tx_power_dbm
        self.antenna_h = antenna_h
        self.noise_floor_dbm = noise_floor_dbm
        self.sinr_thresh_db = sinr_thresh_db
        self.shadow_std_los = shadow_std_los
        self.shadow_std_nlos = shadow_std_nlos
        self.rician_k_linear = 10.0 ** (rician_k_db / 10.0)
        self.num_subchannels = num_subchannels
        self.rc_min = rc_min
        self.rc_max = rc_max
        self.p_keep = p_keep

        self.subchannel: List[int] = []
        self.reselect_counter: List[int] = []
        self._rng = np.random.default_rng()

    def reset(self):
        self.subchannel.clear()
        self.reselect_counter.clear()

    # ----- WINNER+ B1 path loss (3GPP TR 36.885 / ns-3 CNI model) -----

    def _path_loss_freespace(self, dist_m: float) -> float:
        d = max(dist_m, 1.0)
        return (20.0 * math.log10(d)
                + 46.4
                + 20.0 * math.log10(self.carrier_ghz / 5.0))

    def _path_loss_los(self, dist_m: float) -> float:
        d = max(dist_m, 3.0)
        fc = self.carrier_ghz
        hbs1 = self.antenna_h - 1.0
        hms1 = self.antenna_h - 1.0
        freq_hz = self.carrier_ghz * 1e9
        d_bp = 4.0 * hbs1 * hms1 * freq_hz / 3e8

        if d <= d_bp:
            loss = 22.7 * math.log10(d) + 27.0 + 20.0 * math.log10(fc)
        else:
            loss = (40.0 * math.log10(d)
                    + 7.56
                    - 17.3 * math.log10(hbs1)
                    - 17.3 * math.log10(hms1)
                    + 2.7 * math.log10(fc))
        return max(self._path_loss_freespace(d), loss)

    def _path_loss_nlos(self, dist_m: float) -> float:
        d = max(dist_m, 3.0)
        fc = self.carrier_ghz
        hbs = self.antenna_h
        nlos_offset = -5.0
        loss = ((44.9 - 6.55 * math.log10(hbs)) * math.log10(d)
                + 5.83 * math.log10(hbs)
                + 18.38
                + 23.0 * math.log10(fc)
                + nlos_offset)
        return max(self._path_loss_freespace(d), loss)

    def _compute_path_loss(self, dist_m: float, occ: OcclusionInfo) -> float:
        if occ.building_blocked:
            return self._path_loss_nlos(dist_m)
        return self._path_loss_los(dist_m) + occ.extra_loss_db

    # ----- Fading -----

    def _sample_fading(self, los: bool) -> float:
        if los:
            K = self.rician_k_linear
            mu = math.sqrt(K / (K + 1.0))
            sigma = math.sqrt(0.5 / (K + 1.0))
            re = mu + sigma * self._rng.standard_normal()
            im = sigma * self._rng.standard_normal()
            power = re * re + im * im
        else:
            power = self._rng.exponential(1.0)
        return 10.0 * math.log10(max(power, 1e-10))

    def _sample_shadow(self, los: bool) -> float:
        sigma = self.shadow_std_los if los else self.shadow_std_nlos
        return float(sigma * self._rng.standard_normal())

    # ----- SB-SPS MAC -----

    def _update_sps(self, n: int):
        while len(self.subchannel) < n:
            self.subchannel.append(random.randint(0, self.num_subchannels - 1))
            self.reselect_counter.append(
                random.randint(self.rc_min, self.rc_max))

        self.subchannel = self.subchannel[:n]
        self.reselect_counter = self.reselect_counter[:n]

        for i in range(n):
            self.reselect_counter[i] -= 1
            if self.reselect_counter[i] <= 0:
                if random.random() >= self.p_keep:
                    self.subchannel[i] = random.randint(
                        0, self.num_subchannels - 1)
                self.reselect_counter[i] = random.randint(
                    self.rc_min, self.rc_max)

    # ----- Main entry point -----

    def compute_tick(
        self,
        positions: list,
        occlusion: list,
        tick: int,
    ) -> List[LinkResult]:
        N = len(positions)
        self._update_sps(N)

        # Compute received power for all pairs
        rx_power = np.full((N, N), -999.0, dtype=np.float64)
        for i in range(N):
            for j in range(i + 1, N):
                dx = positions[i][0] - positions[j][0]
                dy = positions[i][1] - positions[j][1]
                dz = positions[i][2] - positions[j][2]
                dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)

                occ = occlusion[i][j]
                los = (not occ.building_blocked
                       and occ.num_vehicles_blocking == 0)

                pl = self._compute_path_loss(dist_m, occ)
                shadow = self._sample_shadow(los)
                fading = self._sample_fading(los)

                rx = self.tx_power_dbm - pl - shadow + fading
                rx_power[i, j] = rx
                rx_power[j, i] = rx

        # Compute SINR and delivery for each directed pair
        noise_lin = 10.0 ** (self.noise_floor_dbm / 10.0)
        results: List[LinkResult] = []

        for rx_id in range(N):
            for tx_id in range(N):
                if tx_id == rx_id:
                    continue

                tx_ch = self.subchannel[tx_id]
                sig_lin = 10.0 ** (rx_power[tx_id, rx_id] / 10.0)

                interf_lin = 0.0
                for k in range(N):
                    if k == tx_id or k == rx_id:
                        continue
                    if self.subchannel[k] == tx_ch:
                        interf_lin += 10.0 ** (rx_power[k, rx_id] / 10.0)

                sinr_lin = sig_lin / (interf_lin + noise_lin)
                sinr_db = 10.0 * math.log10(max(sinr_lin, 1e-10))
                delivered = sinr_db >= self.sinr_thresh_db

                results.append(LinkResult(
                    tx_id=tx_id, rx_id=rx_id,
                    delivered=delivered, sinr_db=sinr_db,
                    delay_ms=1.0))

        return results

    def get_subchannel_assignments(self) -> List[int]:
        return list(self.subchannel)

    @staticmethod
    def backhaul_queue_delay_ms(
        n_vehicles: int,
        feature_bytes: int = 175000,
        backhaul_bw_mbps: float = 100.0,
    ) -> float:
        total_bits = n_vehicles * feature_bytes * 8.0
        bw_bps = backhaul_bw_mbps * 1e6
        service_ms = (total_bits / bw_bps) * 1000.0
        return service_ms * 0.5


def get_channel_engine(**kwargs) -> ChannelEngine:
    """Factory: try C++ module first, fall back to Python."""
    try:
        import importlib
        _cpp = importlib.import_module("channel_engine")
        if hasattr(_cpp, "ChannelEngine"):
            logger.info("Using C++ channel engine")
            return _cpp.ChannelEngine(**kwargs)
    except (ImportError, ModuleNotFoundError):
        pass
    logger.info("Using Python channel engine fallback")
    return ChannelEngine(**kwargs)
