# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
MAC-layer models for C-V2X PC5 sidelink packet delivery.

SB-SPS (Sensing-Based Semi-Persistent Scheduling) governs L_access
(vehicle -> roadside access node).  Existing LatencyModel controls
delay shape for delivered packets; MAC controls delivery/drop.

Models:
    LegacyMacAdapter  - wraps LatencyModel.should_drop(), exact old behavior
    NullMac           - explicit no-loss (all delivered)
    SbSpsMac          - C-V2X PC5 Mode 4 resource scheduling with burst losses

Shared registry:
    All edge managers in a run share the same SbSpsMac instance via
    module-level _GLOBAL_MACS registry keyed by (channel_id, seed, config).
    Each test_runner sweep run is a separate subprocess so the registry
    starts empty, but reset_global_macs() is called in edge __init__
    for robustness against in-process test runs.
"""

from __future__ import annotations

import json
import logging
import random
from abc import ABC, abstractmethod
from collections import defaultdict
from typing import Any, Dict, List, Optional

logger = logging.getLogger("MACModel")


# ──────────────────────────────────────────────────────────────────────
#  Metrics
# ──────────────────────────────────────────────────────────────────────
class MACMetrics:
    """PRR + burst tracking with histogram storage (bounded)."""

    def __init__(self):
        self.attempted = 0
        self.delivered = 0
        self.collision_drops = 0
        self.fading_drops = 0
        self._burst_histogram: Dict[int, int] = {}   # {length: count}
        self._current_burst: Dict[int, int] = {}     # {sender_id: run_length}
        self._finalized = False

    def record(self, sender_id: int, delivered: bool,
               drop_cause: str = None):
        self.attempted += 1
        if delivered:
            self.delivered += 1
            # close any in-progress burst for this sender
            if sender_id in self._current_burst:
                bl = self._current_burst.pop(sender_id)
                capped = min(bl, 200)
                self._burst_histogram[capped] = (
                    self._burst_histogram.get(capped, 0) + 1)
        else:
            if drop_cause == 'collision':
                self.collision_drops += 1
            elif drop_cause == 'fading':
                self.fading_drops += 1
            self._current_burst[sender_id] = (
                self._current_burst.get(sender_id, 0) + 1)

    def finalize(self):
        """Flush in-progress bursts at end-of-run. Idempotent."""
        if self._finalized:
            return
        for sid, bl in self._current_burst.items():
            capped = min(bl, 200)
            self._burst_histogram[capped] = (
                self._burst_histogram.get(capped, 0) + 1)
        self._current_burst.clear()
        self._finalized = True

    def prr(self) -> float:
        return self.delivered / max(self.attempted, 1)

    def _weighted_stats(self):
        """Compute mean/p95/p99 from histogram without materializing list."""
        if not self._burst_histogram:
            return 0.0, 0.0, 0.0
        total = sum(self._burst_histogram.values())
        mean = sum(l * c for l, c in self._burst_histogram.items()) / max(total, 1)
        sorted_lens = sorted(self._burst_histogram.keys())
        cumul = 0
        p95 = p99 = sorted_lens[-1]
        for l in sorted_lens:
            cumul += self._burst_histogram[l]
            if cumul >= total * 0.95 and p95 == sorted_lens[-1]:
                p95 = l
            if cumul >= total * 0.99:
                p99 = l
                break
        return float(mean), float(p95), float(p99)

    def get_summary(self) -> Dict[str, Any]:
        self.finalize()
        mean, p95, p99 = self._weighted_stats()
        return {
            'prr': self.prr(),
            'total_attempted': self.attempted,
            'total_delivered': self.delivered,
            'collision_drops': self.collision_drops,
            'fading_drops': self.fading_drops,
            'burst_histogram': self._burst_histogram,
            'burst_length_mean': mean,
            'burst_length_p95': p95,
            'burst_length_p99': p99,
            'mac_effective_hz_under_20hz_offered': self.prr() * 20,
            'mac_burstiness_factor': p95 / max(mean, 0.01),
        }


# ──────────────────────────────────────────────────────────────────────
#  Abstract base
# ──────────────────────────────────────────────────────────────────────
class MACModel(ABC):
    """Base class for MAC-layer delivery models."""

    @abstractmethod
    def attempt_tick(self, tick: int,
                     sender_ids: List[int]) -> Dict[int, bool]:
        """Determine delivery for each sender this tick.

        Returns {sender_id: delivered_bool}.
        """

    @property
    @abstractmethod
    def metrics(self) -> MACMetrics:
        """Access accumulated metrics."""


# ──────────────────────────────────────────────────────────────────────
#  Legacy adapter (wraps should_drop)
# ──────────────────────────────────────────────────────────────────────
class LegacyMacAdapter(MACModel):
    """Wraps latency_model.should_drop() -- exact old behavior.

    Used when no ``mac:`` YAML block is present.  NOT shared (each
    manager wraps its own latency_model).  Fairness invariant: one
    manager per run.
    """

    def __init__(self, latency_model):
        self._lm = latency_model
        self._metrics = MACMetrics()

    def attempt_tick(self, tick: int,
                     sender_ids: List[int]) -> Dict[int, bool]:
        result = {}
        for sid in sender_ids:
            delivered = not self._lm.should_drop()
            result[sid] = delivered
            self._metrics.record(sid, delivered)
        return result

    @property
    def metrics(self):
        return self._metrics


# ──────────────────────────────────────────────────────────────────────
#  Null MAC (explicit no-loss)
# ──────────────────────────────────────────────────────────────────────
class NullMac(MACModel):
    """Explicit no-loss: all packets delivered. For ``mac.type: null``."""

    def __init__(self):
        self._metrics = MACMetrics()

    def attempt_tick(self, tick: int,
                     sender_ids: List[int]) -> Dict[int, bool]:
        for sid in sender_ids:
            self._metrics.record(sid, True)
        return {sid: True for sid in sender_ids}

    @property
    def metrics(self):
        return self._metrics


# ──────────────────────────────────────────────────────────────────────
#  SB-SPS MAC (C-V2X PC5 Mode 4)
# ──────────────────────────────────────────────────────────────────────
class SbSpsMac(MACModel):
    """C-V2X PC5 Mode 4 SB-SPS resource scheduling.

    Tick-idempotent: caches result per tick.  Subset queries of a
    previously-computed tick return slices without recomputation.
    New senders in same tick raise RuntimeError.

    Parameters (3GPP TS 36.321):
        M          : number of orthogonal sidelink resources per update
        RC_min/max : reselection counter bounds
        p_keep     : probability of keeping current resource at reselection
        p_loss_base: baseline fading/propagation loss probability
        seed       : mandatory RNG seed for reproducibility
    """

    def __init__(self, M: int = 20, RC_min: int = 5, RC_max: int = 15,
                 p_keep: float = 0.4, p_loss_base: float = 0.02,
                 seed: int = 0):
        self._M = M
        self._RC_min, self._RC_max = RC_min, RC_max
        self._p_keep = p_keep
        self._p_loss_base = p_loss_base
        self._rng = random.Random(seed)
        self._metrics = MACMetrics()
        self._state: Dict[int, Dict] = {}
        self._last_tick: Optional[int] = None
        self._last_result: Dict[int, bool] = {}

    def attempt_tick(self, tick: int,
                     sender_ids: List[int]) -> Dict[int, bool]:
        if self._last_tick == tick:
            # subset query: return cached results
            missing = [s for s in sender_ids if s not in self._last_result]
            if missing:
                raise RuntimeError(
                    f"SbSpsMac: tick {tick} already computed but new senders "
                    f"{missing} appeared. Call once with full sender set.")
            return {s: self._last_result[s] for s in sender_ids}

        # --- fresh tick ---
        # T12 load knob: MAC_BG_SENDERS injects N phantom contenders that
        # compete for the M sidelink resources each tick, raising the
        # collision rate for the real senders (SB-SPS background load). Only
        # real sender_ids are returned; phantoms use negative ids.
        import os as _osm
        _bg = int(_osm.environ.get('MAC_BG_SENDERS', 0) or 0)
        _eff_senders = list(sender_ids) + [-(i + 1) for i in range(_bg)]
        # 1. init new senders
        for sid in _eff_senders:
            if sid not in self._state:
                self._state[sid] = {
                    'resource': self._rng.randint(0, self._M - 1),
                    'rc': self._rng.randint(self._RC_min, self._RC_max),
                }

        # 2. resource -> senders mapping
        res_users: Dict[int, List[int]] = defaultdict(list)
        for sid in _eff_senders:
            res_users[self._state[sid]['resource']].append(sid)

        # 3. collisions + fading
        result = {}
        for sid in sender_ids:
            r = self._state[sid]['resource']
            collided = len(res_users[r]) > 1
            fading = self._rng.random() < self._p_loss_base
            delivered = not collided and not fading
            result[sid] = delivered
            cause = 'collision' if collided else ('fading' if fading else None)
            self._metrics.record(sid, delivered, drop_cause=cause)

        # 4. reselection
        for sid in _eff_senders:
            st = self._state[sid]
            st['rc'] -= 1
            if st['rc'] <= 0:
                if self._rng.random() >= self._p_keep:
                    st['resource'] = self._rng.randint(0, self._M - 1)
                st['rc'] = self._rng.randint(self._RC_min, self._RC_max)

        self._last_tick = tick
        self._last_result = result
        return result

    @property
    def metrics(self):
        return self._metrics


# ──────────────────────────────────────────────────────────────────────
#  Shared registry + factory
# ──────────────────────────────────────────────────────────────────────
_GLOBAL_MACS: Dict[str, MACModel] = {}


def reset_global_macs():
    """Call at start of each run to prevent state leakage."""
    _GLOBAL_MACS.clear()


def create_mac_model(cfg: Dict, latency_model=None,
                     seed: int = 0,
                     channel_id: str = "pc5_uplink") -> MACModel:
    """Factory: one shared instance per (channel_id, mac_config) per run.

    - No ``mac:`` key -> LegacyMacAdapter (wraps should_drop(), exact old behavior)
    - mac.type: null  -> NullMac (explicit no-loss)
    - mac.type: sbsps -> SbSpsMac (shared across all edge managers)
    """
    mac_cfg = cfg.get("mac", None)
    if mac_cfg is None:
        # no mac block -> legacy adapter, NOT shared
        return LegacyMacAdapter(latency_model)

    # build registry key from config content
    mac_sig = json.dumps(mac_cfg, sort_keys=True)
    key = f"{channel_id}:{seed}:{mac_sig}"

    if key in _GLOBAL_MACS:
        return _GLOBAL_MACS[key]

    mac_type = (mac_cfg.get("type", "null") or "null").lower()

    if mac_type in ("null", "none"):
        model = NullMac()
    elif mac_type == "sbsps":
        model = SbSpsMac(
            M=int(mac_cfg.get("M", 20)),
            RC_min=int(mac_cfg.get("RC_min", 5)),
            RC_max=int(mac_cfg.get("RC_max", 15)),
            p_keep=float(mac_cfg.get("p_keep", 0.4)),
            p_loss_base=float(mac_cfg.get("p_loss_base", 0.02)),
            seed=seed,
        )
    else:
        raise ValueError(f"Unknown MAC type: '{mac_type}'")

    _GLOBAL_MACS[key] = model
    logger.info("Created %s MAC model (channel=%s, seed=%d, M=%s)",
                mac_type, channel_id, seed,
                mac_cfg.get("M", "default"))
    return model
