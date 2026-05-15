"""ns-3 LUT-backed latency sampler for closed-loop AoI injection.

Loads ns3_uplink_lut.csv and ns3_downlink_lut.csv from this module's data/ dir, and
exposes sample_ms(N, payload_bytes, direction) for the edge manager.  Each
LUT row has mean/p50/p95/p99 ms over n_samples runs at a (N, payload) cell;
we treat (mean, p95) as a Gaussian (approx, since p95 ≈ mean + 1.645 sigma)
and sample, then clamp to [0, max].  Bilinear interpolation across N and
log-payload keeps the sampler well-defined for the values closed-loop sees
(e.g. N=12 between LUT cells 8 and 16; payload sizes that fall between
1000 B and 16896 B).
"""
from __future__ import annotations

import csv
import math
import random
from pathlib import Path
from typing import Dict, List, Tuple


class Ns3LutSampler:
    def __init__(self, ul_csv: str, dl_csv: str):
        # Each cell tuple: (N, payload, mean_ms, p95_ms, max_ms, prr)
        self._cells: Dict[str, List[Tuple[int, int, float, float, float, float]]] = {
            'ul': [], 'dl': [],
        }
        for path, key in [(ul_csv, 'ul'), (dl_csv, 'dl')]:
            with open(path, newline='') as f:
                for row in csv.DictReader(f):
                    self._cells[key].append((
                        int(row['N']),
                        int(row['payload_bytes']),
                        float(row['mean']),
                        float(row['p95']),
                        float(row['max']),
                        float(row.get('prr', 1.0)),
                    ))
            self._cells[key].sort()

    def _bracket(self, sorted_vals: List[float], v: float) -> Tuple[float, float, float]:
        """Return (lo, hi, frac) where frac in [0, 1] interpolates v between lo and hi."""
        if v <= sorted_vals[0]:
            return sorted_vals[0], sorted_vals[0], 0.0
        if v >= sorted_vals[-1]:
            return sorted_vals[-1], sorted_vals[-1], 1.0
        for i in range(len(sorted_vals) - 1):
            if sorted_vals[i] <= v <= sorted_vals[i + 1]:
                lo, hi = sorted_vals[i], sorted_vals[i + 1]
                if hi == lo:
                    return lo, hi, 0.0
                return lo, hi, (v - lo) / (hi - lo)
        return sorted_vals[-1], sorted_vals[-1], 1.0

    def _lookup(self, direction: str, N: int, payload_bytes: int) -> Tuple[float, float, float, float]:
        """Bilinear interp of (mean, p95, max, prr) across N and log-payload."""
        cells = self._cells[direction]
        Ns = sorted({c[0] for c in cells})
        payloads = sorted({c[1] for c in cells})
        N_lo, N_hi, fN = self._bracket([float(x) for x in Ns], float(N))
        log_p = math.log(max(payload_bytes, 1))
        log_payloads = sorted([math.log(p) for p in payloads])
        lp_lo, lp_hi, fp = self._bracket(log_payloads, log_p)
        # Map back to integer payload values for cell lookup.
        p_lo = int(round(math.exp(lp_lo)))
        p_hi = int(round(math.exp(lp_hi)))
        # Snap to nearest LUT payload bin (LUT only has the discrete cells).
        def snap(p: int) -> int:
            return min(payloads, key=lambda x: abs(x - p))
        p_lo, p_hi = snap(p_lo), snap(p_hi)

        def get(N_v: float, p_v: int) -> Tuple[float, float, float, float]:
            # Snap N to the nearest LUT N value pair on the bracket.
            for c in cells:
                if c[0] == int(N_v) and c[1] == p_v:
                    return c[2], c[3], c[4], c[5]
            # Fallback: nearest by N then payload.
            best = min(cells, key=lambda c: (abs(c[0] - N_v), abs(c[1] - p_v)))
            return best[2], best[3], best[4], best[5]

        a = get(N_lo, p_lo)
        b = get(N_hi, p_lo)
        c = get(N_lo, p_hi)
        d = get(N_hi, p_hi)

        def lerp(x, y, f):
            return tuple(xi * (1 - f) + yi * f for xi, yi in zip(x, y))

        ab = lerp(a, b, fN)
        cd = lerp(c, d, fN)
        out = lerp(ab, cd, fp)
        return float(out[0]), float(out[1]), float(out[2]), float(out[3])

    def sample_ms(self, N: int, payload_bytes: int, direction: str) -> float:
        """Return one latency sample in ms for (N, payload, direction).

        Uses the LUT cell's (mean, p95) as a Gaussian approximation
        (p95 ≈ mean + 1.645 sigma).  Clamped to [0, max_observed].
        Does NOT model packet loss; use sample_with_loss to draw both
        a delay and a delivery flag.
        """
        if direction not in ('ul', 'dl'):
            raise ValueError(f"direction must be 'ul' or 'dl', got {direction!r}")
        mean_ms, p95_ms, max_ms, _prr = self._lookup(direction, N, payload_bytes)
        sigma = max(0.0, (p95_ms - mean_ms) / 1.645)
        sample = random.gauss(mean_ms, sigma) if sigma > 0 else mean_ms
        return max(0.0, min(sample, max_ms))

    def sample_with_loss(self, N: int, payload_bytes: int,
                         direction: str) -> Tuple[float, bool]:
        """Return (delay_ms, delivered) for one transmission.

        Draws a Gaussian latency sample as in sample_ms, then a
        Bernoulli on the cell's PRR to decide if the message was
        delivered.  When `delivered` is False, the caller should drop
        the message (do not push it into the jitter buffer / downlink
        queue).  This implements ns-3 LUT-backed loss in the live
        closed-loop pipeline so the controller's safety claim holds
        under realistic uplink/downlink drops.
        """
        if direction not in ('ul', 'dl'):
            raise ValueError(f"direction must be 'ul' or 'dl', got {direction!r}")
        mean_ms, p95_ms, max_ms, prr = self._lookup(direction, N, payload_bytes)
        sigma = max(0.0, (p95_ms - mean_ms) / 1.645)
        delay_ms = random.gauss(mean_ms, sigma) if sigma > 0 else mean_ms
        delay_ms = max(0.0, min(delay_ms, max_ms))
        delivered = random.random() < prr
        return delay_ms, delivered

    def prr(self, N: int, payload_bytes: int, direction: str) -> float:
        """Return PRR for the (N, payload, direction) cell, interpolated."""
        if direction not in ('ul', 'dl'):
            raise ValueError(f"direction must be 'ul' or 'dl', got {direction!r}")
        _, _, _, prr = self._lookup(direction, N, payload_bytes)
        return prr


_DEFAULT: Ns3LutSampler | None = None


def get_default(repo_root: str | None = None) -> Ns3LutSampler:
    """Singleton accessor.  Reads CSVs alongside this module under data/."""
    global _DEFAULT
    if _DEFAULT is None:
        here = Path(__file__).resolve()
        data_dir = here.parent / "data"
        ul = data_dir / "ns3_uplink_lut.csv"
        dl = data_dir / "ns3_downlink_lut.csv"
        _DEFAULT = Ns3LutSampler(str(ul), str(dl))
    return _DEFAULT
