# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Pluggable latency models for edge-server communication simulation.

Each model converts a source tick into an arrival tick, accounting for
network delay (radio link, backhaul, application processing).  The model
also simulates uplink packet loss.

Supported distributions:
    fixed      - constant delay
    normal     - Gaussian jitter around a mean
    lognormal  - heavy-tailed delay (one parameter)
    hybrid     - C-V2X radio RTT (SEE-V2X trace) + backhaul lognormal
                 + configurable base application latency

SEE-V2X trace resolution (priority order):
    1. Explicit path via ``see_v2x_trace`` YAML key or CLI ``--see-v2x-trace``
    2. Environment variable ``SEE_V2X_TRACE_PATH``
    3. Repo-relative ``data/see_v2x/merged_latency.csv``

Per-intersection filtering:
    Set ``see_v2x_filter: "intersection=3"`` in YAML (or CLI) to use only
    samples from a specific intersection.  Pre-defined regimes:
        L = intersection 3  (lowest mean uplink latency)
        M = intersection 4  (moderate)
        H = intersection 5  (highest mean uplink latency)
    Use ``see_v2x_filter: "regime=L"`` as shorthand.

Usage::

    from ecav.core.application.edge.latency import create_latency_model

    model = create_latency_model(edge_yaml_cfg, dt=0.05)
    arrival = model.stamp(source_tick)
    if model.should_drop():
        ...  # simulate uplink packet loss
"""

from __future__ import annotations

import logging
import math
import os
import pathlib
import random
from abc import ABC, abstractmethod
from typing import Dict, List, Optional

import numpy as np

logger = logging.getLogger("LatencyModel")

# Default backhaul log-normal parameters (ms in log-space)
_DEFAULT_BACKHAUL_MU = 2.9957
_DEFAULT_BACKHAUL_SIGMA = 0.6556

# Pre-defined regime -> intersection mapping
_REGIME_MAP = {"L": "3", "M": "4", "H": "5"}


# ──────────────────────────────────────────────────────────────────────
#  SEE-V2X trace loader
# ──────────────────────────────────────────────────────────────────────

def _find_trace_file(explicit_path: Optional[str] = None) -> Optional[pathlib.Path]:
    """Resolve the SEE-V2X trace CSV using priority order.

    1. ``explicit_path`` (from YAML / CLI)
    2. ``SEE_V2X_TRACE_PATH`` environment variable
    3. Repo-relative ``data/see_v2x/merged_latency.csv``

    Returns the Path if found, else None.
    """
    # 1. Explicit path
    if explicit_path:
        p = pathlib.Path(explicit_path)
        if p.is_file():
            return p
        logger.warning("Explicit SEE-V2X trace path '%s' not found", p)

    # 2. Environment variable
    env_path = os.environ.get("SEE_V2X_TRACE_PATH")
    if env_path:
        p = pathlib.Path(env_path)
        if p.is_file():
            return p
        logger.warning("SEE_V2X_TRACE_PATH='%s' not found", p)

    # 3. Repo-relative (walk up from this file to find repo root)
    here = pathlib.Path(__file__).resolve().parent
    # Walk up to find data/see_v2x/merged_latency.csv
    for ancestor in [here] + list(here.parents):
        candidate = ancestor / "data" / "see_v2x" / "merged_latency.csv"
        if candidate.is_file():
            return candidate
        # Also check old location
        candidate = ancestor / "see-v2x-input" / "merged_latency.csv"
        if candidate.is_file():
            return candidate

    return None


def load_trace(explicit_path: Optional[str] = None,
               filter_str: Optional[str] = None) -> Optional[np.ndarray]:
    """Load and optionally filter the SEE-V2X C-V2X trace.

    Parameters
    ----------
    explicit_path : str or None
        Direct path to merged_latency.csv.
    filter_str : str or None
        Filter expression. Supported forms:
        - ``"intersection=3"`` — keep only rows from intersection 3
        - ``"regime=L"`` — shorthand for intersection=3 (L/M/H)
        - ``"uplink"`` — keep only vehicle->RSU rows
        - ``"downlink"`` — keep only RSU->vehicle rows
        Multiple filters can be combined with ``+``:
        ``"intersection=3+uplink"``

    Returns
    -------
    np.ndarray or None
        1-D array of latency_ms values (float32), or None if not found.
    """
    trace_file = _find_trace_file(explicit_path)
    if trace_file is None:
        return None

    try:
        import pandas as pd
        df = pd.read_csv(trace_file)

        # Apply filters
        if filter_str:
            for part in filter_str.split("+"):
                part = part.strip()
                if part.startswith("intersection="):
                    val = part.split("=", 1)[1]
                    if "intersection" in df.columns:
                        df = df[df["intersection"].astype(str) == val]
                    else:
                        logger.warning("Trace has no 'intersection' column; "
                                       "ignoring filter '%s'", part)
                elif part.startswith("regime="):
                    regime = part.split("=", 1)[1].upper()
                    int_id = _REGIME_MAP.get(regime)
                    if int_id and "intersection" in df.columns:
                        df = df[df["intersection"].astype(str) == int_id]
                    else:
                        logger.warning("Unknown regime '%s' or no intersection "
                                       "column; ignoring filter", regime)
                elif part == "uplink":
                    df = df[df["receiver"] == "rsu"]
                elif part == "downlink":
                    df = df[df["sender"] == "rsu"]
                else:
                    logger.warning("Unknown trace filter '%s'; ignoring", part)

        samples = df["latency_ms"].dropna().to_numpy(dtype=np.float32)
        logger.info("Loaded %d C-V2X samples from %s (filter=%s)",
                    samples.size, trace_file, filter_str)
        return samples if samples.size > 0 else None

    except Exception as exc:
        logger.error("Failed to parse %s: %s", trace_file, exc)
        return None


def get_trace_metadata(explicit_path: Optional[str] = None) -> Dict:
    """Return metadata about the SEE-V2X trace for characterization plots.

    Returns dict with per-intersection stats, aggregate stats, and the
    raw per-intersection sample arrays for CDF/PDF plotting.
    """
    trace_file = _find_trace_file(explicit_path)
    if trace_file is None:
        return {}

    try:
        import pandas as pd
        df = pd.read_csv(trace_file)
    except Exception:
        return {}

    meta = {"trace_file": str(trace_file), "total_samples": len(df)}

    # Aggregate stats
    lats = df["latency_ms"].dropna()
    meta["aggregate"] = {
        "mean": float(lats.mean()),
        "median": float(lats.median()),
        "p95": float(lats.quantile(0.95)),
        "p99": float(lats.quantile(0.99)),
        "max": float(lats.max()),
        "samples": lats.to_numpy(dtype=np.float32),
    }

    # Per-intersection stats
    if "intersection" in df.columns:
        meta["per_intersection"] = {}
        for iid, grp in df.groupby("intersection"):
            gl = grp["latency_ms"].dropna()
            meta["per_intersection"][str(iid)] = {
                "n": len(gl),
                "mean": float(gl.mean()),
                "median": float(gl.median()),
                "p95": float(gl.quantile(0.95)),
                "p99": float(gl.quantile(0.99)),
                "max": float(gl.max()),
                "samples": gl.to_numpy(dtype=np.float32),
            }
        # Uplink-only per intersection
        uplink = df[df["receiver"] == "rsu"]
        meta["per_intersection_uplink"] = {}
        for iid, grp in uplink.groupby("intersection"):
            gl = grp["latency_ms"].dropna()
            meta["per_intersection_uplink"][str(iid)] = {
                "n": len(gl),
                "mean": float(gl.mean()),
                "p95": float(gl.quantile(0.95)),
                "p99": float(gl.quantile(0.99)),
                "samples": gl.to_numpy(dtype=np.float32),
            }

    return meta


# ──────────────────────────────────────────────────────────────────────
#  Abstract base
# ──────────────────────────────────────────────────────────────────────
class LatencyModel(ABC):
    """Base class for all latency simulation models."""

    def __init__(self, dt: float, loss_pct: float = 0.0,
                 timestamp_noise_ms: float = 0.0):
        self._dt_ms = dt * 1000.0
        self._loss_pct = loss_pct
        self._timestamp_noise_std_ms = timestamp_noise_ms
        # Per-sample component history (populated by subclasses that
        # override _sample_ms_components).  Each entry is a dict like
        # {'radio_ms': ..., 'backhaul_ms': ..., 'base_ms': ..., 'total_ms': ...}
        self.sample_history: List[Dict[str, float]] = []

    @abstractmethod
    def _sample_ms(self) -> float:
        """Return one random latency sample in milliseconds."""

    def sample_ms(self) -> float:
        """Public API: return one random latency sample in milliseconds.

        Use this when the caller needs raw ms (e.g. cost modelling).
        Use stamp() when the caller needs a tick offset.
        """
        return self._sample_ms()

    def stamp(self, source_tick: int) -> int:
        """Return the arrival tick for a packet sent at *source_tick*.

        If ``timestamp_noise_ms > 0``, Gaussian noise is added to the
        source tick to model imperfect clock synchronization between
        vehicle and edge server.
        """
        # Clock skew: perturb the source timestamp
        if self._timestamp_noise_std_ms > 0:
            noise_ticks = round(
                np.random.normal(0, self._timestamp_noise_std_ms / self._dt_ms))
            source_tick = max(0, source_tick + noise_ticks)
        delay_ms = self._sample_ms()
        delay_ticks = int(round(delay_ms / self._dt_ms))
        return source_tick + max(delay_ticks, 0)

    def should_drop(self) -> bool:
        """Return True if this packet should be dropped (uplink loss)."""
        return random.random() * 100 < self._loss_pct

    @property
    def loss_pct(self) -> float:
        return self._loss_pct


# ──────────────────────────────────────────────────────────────────────
#  Concrete implementations
# ──────────────────────────────────────────────────────────────────────
class FixedLatencyModel(LatencyModel):
    """Constant delay: every packet arrives exactly *mean_ms* later."""

    def __init__(self, mean_ms: float, dt: float, loss_pct: float = 0.0,
                 timestamp_noise_ms: float = 0.0):
        super().__init__(dt, loss_pct, timestamp_noise_ms)
        self._mean_ms = max(mean_ms, 0.0)

    def _sample_ms(self) -> float:
        return self._mean_ms


class NormalJitterModel(LatencyModel):
    """Gaussian jitter: delay ~ N(mean_ms, std_ms), clamped >= 0."""

    def __init__(self, mean_ms: float, std_ms: float,
                 dt: float, loss_pct: float = 0.0,
                 timestamp_noise_ms: float = 0.0):
        super().__init__(dt, loss_pct, timestamp_noise_ms)
        self._mean_ms = mean_ms
        self._std_ms = std_ms

    def _sample_ms(self) -> float:
        return max(0.0, random.gauss(self._mean_ms, self._std_ms))


class LognormalModel(LatencyModel):
    """Log-normal delay: heavy tail, mean ~ *mean_ms*."""

    def __init__(self, mean_ms: float, dt: float, loss_pct: float = 0.0,
                 timestamp_noise_ms: float = 0.0):
        super().__init__(dt, loss_pct, timestamp_noise_ms)
        self._mu = math.log(max(mean_ms, 1e-3))

    def _sample_ms(self) -> float:
        return float(np.random.lognormal(self._mu, 0.5))


class HybridModel(LatencyModel):
    """C-V2X radio link + backhaul + application latency.

    Total = radio_rtt + backhaul_rtt + base_ms

    Radio RTT: sampled from a real C-V2X trace (SEE-V2X dataset)
    if available, otherwise U(30, 80) ms fallback.
    Backhaul RTT: log-normal(mu, sigma) in log-ms space.
    Can be disabled (``disable_backhaul=True``).
    Base: configurable application-layer latency.
    """

    def __init__(self, base_ms: float, dt: float, loss_pct: float = 0.0,
                 timestamp_noise_ms: float = 0.0,
                 trace_samples: Optional[np.ndarray] = None,
                 backhaul_mu: float = _DEFAULT_BACKHAUL_MU,
                 backhaul_sigma: float = _DEFAULT_BACKHAUL_SIGMA,
                 disable_backhaul: bool = False):
        super().__init__(dt, loss_pct, timestamp_noise_ms)
        self._base_ms = base_ms
        self._trace = trace_samples
        self._backhaul_mu = backhaul_mu
        self._backhaul_sigma = backhaul_sigma
        self._disable_backhaul = disable_backhaul

        if self._trace is not None and self._trace.size > 0:
            logger.info("HybridModel: %d trace samples, backhaul=%s "
                        "(mu=%.3f, sigma=%.3f), base=%.1f ms",
                        self._trace.size,
                        "disabled" if disable_backhaul else "enabled",
                        backhaul_mu, backhaul_sigma, base_ms)
        else:
            logger.warning("HybridModel: no trace loaded, using U(30,80) ms "
                          "radio RTT fallback")

    def _sample_ms(self) -> float:
        # Radio link RTT (one-way estimate)
        if self._trace is not None and self._trace.size > 0:
            rtt_radio = float(np.random.choice(self._trace))
        else:
            rtt_radio = random.uniform(30.0, 80.0)
        # Backhaul RTT
        if self._disable_backhaul:
            rtt_back = 0.0
        else:
            rtt_back = float(np.random.lognormal(
                self._backhaul_mu, self._backhaul_sigma))
        total = rtt_radio + rtt_back + self._base_ms
        self.sample_history.append({
            'radio_ms': rtt_radio,
            'backhaul_ms': rtt_back,
            'base_ms': self._base_ms,
            'total_ms': total,
        })
        return total


# ──────────────────────────────────────────────────────────────────────
#  Factory
# ──────────────────────────────────────────────────────────────────────
def create_latency_model(cfg: Dict, dt: float) -> LatencyModel:
    """Create a latency model from a YAML edge config block.

    Expected keys (same as existing YAML schema)::

        latency: 0.2                    # seconds (base for hybrid, mean for others)
        jitter_std: 0.05                # seconds
        latency_distribution: hybrid    # fixed | normal | lognormal | hybrid
        uplink_packet_loss_pct: 10      # percent
        timestamp_noise_ms: 0           # ms

    SEE-V2X trace keys (for hybrid model)::

        see_v2x_trace: /path/to/merged_latency.csv   # explicit path (optional)
        see_v2x_filter: "intersection=3"              # filter expression (optional)
        backhaul_mu: 2.9957                           # log-normal mu (optional)
        backhaul_sigma: 0.6556                        # log-normal sigma (optional)
        disable_backhaul: false                       # skip backhaul component (optional)
    """
    dist = (cfg.get("latency_distribution", "fixed") or "fixed").lower()
    mean_ms = float(cfg.get("latency", 0)) * 1000.0
    std_ms = float(cfg.get("jitter_std", 0)) * 1000.0
    loss_pct = float(cfg.get("uplink_packet_loss_pct", 0))
    ts_noise = float(cfg.get("timestamp_noise_ms", 0))

    if dist in ("fixed", "none"):
        return FixedLatencyModel(mean_ms, dt, loss_pct, ts_noise)

    if dist == "normal":
        return NormalJitterModel(mean_ms, std_ms, dt, loss_pct, ts_noise)

    if dist == "lognormal":
        return LognormalModel(mean_ms, dt, loss_pct, ts_noise)

    if dist == "hybrid":
        # Load trace with filtering
        trace_path = cfg.get("see_v2x_trace")
        trace_filter = cfg.get("see_v2x_filter")
        trace_samples = load_trace(trace_path, trace_filter)

        if trace_samples is None:
            logger.warning("Hybrid model requested but no SEE-V2X trace found. "
                          "Checked: explicit path=%s, env SEE_V2X_TRACE_PATH=%s, "
                          "repo-relative data/see_v2x/merged_latency.csv. "
                          "Using U(30,80ms) radio RTT fallback.",
                          trace_path, os.environ.get("SEE_V2X_TRACE_PATH"))

        backhaul_mu = float(cfg.get("backhaul_mu", _DEFAULT_BACKHAUL_MU))
        backhaul_sigma = float(cfg.get("backhaul_sigma", _DEFAULT_BACKHAUL_SIGMA))
        disable_backhaul = bool(cfg.get("disable_backhaul", False))

        return HybridModel(mean_ms, dt, loss_pct, ts_noise,
                          trace_samples=trace_samples,
                          backhaul_mu=backhaul_mu,
                          backhaul_sigma=backhaul_sigma,
                          disable_backhaul=disable_backhaul)

    raise ValueError(f"unknown latency_distribution '{dist}'")
