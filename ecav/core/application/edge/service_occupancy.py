# -*- coding: utf-8 -*-
"""Logical service occupancy for edge managers that run a measured service.

The execution model (ISPASS paper, validated against a continuously running
service by scripts/ispass/live_service_bench.py, barrier arm):

  admission   an invocation starts at a step boundary, and only when the
              service is free in logical time.
  occupancy   the invocation occupies the service for its measured duration,
              t_free = t_admit + compute_ms.
  latest      frames that arrive while the service is busy wait, and only the
              newest waiting frame is admitted. The others are superseded.
  visibility  the result is visible at the first step at or after logical
              completion (plus the downlink): the manager's delivery stamp.

Edge option service_occupancy=on|off (test_runner --edge-kv, so the value is in
the run directory name and the reducer kv). The default is off: without the
option a manager admits every arriving frame and only delays its delivery,
which is what the SenSys study's arms and the colleague's evaluation export
ran, and those results must not change.

TimingTrace records the stamps of one run and replays them in another
(edge option timing_mode=live|replay, trace path in timing_trace or the
environment variable ECAV_TIMING_TRACE), so delivery ticks do not depend on
the host that replays.
"""
import json
import logging
import os
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

_EPS = 1e-9


def occupancy_enabled(cfg) -> bool:
    """True when the edge config asks for service occupancy."""
    v = cfg.get("service_occupancy", "off") if hasattr(cfg, "get") else "off"
    return str(v).lower() in ("on", "true", "1", "yes")


class ServiceOccupancy:
    """Admission gate of one service. Call admit() once per step with the
    frames that arrived at that step, then occupy() with the duration of the
    invocation if admit() returned a frame."""

    def __init__(self, enabled: bool, dt_ms: float):
        self.enabled = bool(enabled)
        self.dt_ms = float(dt_ms)
        self.t_free_ms = 0.0
        self._pending: Optional[Tuple[int, Any]] = None   # (source_tick, payload)
        self.offered = 0
        self.admitted = 0
        self.superseded = 0
        self.admit_ticks: List[int] = []

    def admit(self, tick: int, arrived: List[Tuple[int, Any]]) -> Optional[Tuple[int, Any]]:
        """arrived: (source_tick, payload) of the frames that arrived at this
        step. Returns the frame to invoke the service on at this step, or None."""
        for f in arrived:
            self.offered += 1
            if self._pending is None or f[0] > self._pending[0]:
                if self._pending is not None:
                    self.superseded += 1
                self._pending = f
            else:
                self.superseded += 1
        if self._pending is None:
            return None
        if self.t_free_ms > tick * self.dt_ms + _EPS:
            return None
        f, self._pending = self._pending, None
        self.admitted += 1
        self.admit_ticks.append(int(tick))
        return f

    def occupy(self, tick: int, compute_ms: float) -> None:
        self.t_free_ms = tick * self.dt_ms + max(0.0, float(compute_ms))

    def metrics(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "offered": int(self.offered),
            "admitted": int(self.admitted),
            "superseded": int(self.superseded),
            "superseded_fraction": (float(self.superseded) / self.offered) if self.offered else 0.0,
        }


def delivery_ticks(durations_ms: List[float], dt_ms: float, frame_ms: float,
                   n_ticks: int) -> List[Tuple[int, int, int]]:
    """Reference use of the gate on a list of recorded durations: one frame per
    frame_ms, each admitted invocation takes the next duration. Returns
    (admit_tick, source_frame, deliver_tick) per invocation. This is the
    barrier arm of live_service_bench.py written with ServiceOccupancy, and
    tests/test_service_occupancy.py checks the two against each other."""
    import math
    gate = ServiceOccupancy(True, dt_ms)
    out, last, k = [], -1, 0
    for m in range(n_ticks):
        t = m * dt_ms
        newest = int(math.floor(t / frame_ms + _EPS))
        arrived = [(s, None) for s in range(last + 1, newest + 1)]
        last = max(last, newest)
        f = gate.admit(m, arrived)
        if f is None:
            continue
        if k >= len(durations_ms):
            break
        c = float(durations_ms[k]); k += 1
        gate.occupy(m, c)
        out.append((m, f[0], m + int(math.ceil(c / dt_ms))))
    return out


class TimingTrace:
    """Per-tick stamp record (JSON lines) for managers that had none. A row is
    {"tick", "ul_ms", "compute_ms", "dl_ms"}."""

    def __init__(self, cfg, tag: str = ""):
        self.mode = str(cfg.get("timing_mode", os.environ.get("ECAV_TIMING_MODE", "live"))).lower()
        path = cfg.get("timing_trace", os.environ.get("ECAV_TIMING_TRACE", None))
        self.path = (str(path) + tag) if path else None
        self.records: Dict[int, Dict[str, float]] = {}
        if self.mode == "replay" and self.path:
            if os.path.isfile(self.path):
                with open(self.path, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if line:
                            r = json.loads(line)
                            self.records[int(r["tick"])] = r
                logger.info("timing trace: %d stamps replayed from %s", len(self.records), self.path)
            else:
                logger.warning("timing trace %s not found for replay mode", self.path)
        elif self.mode == "live" and self.path:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            open(self.path, "w", encoding="utf-8").close()

    @property
    def replaying(self) -> bool:
        return self.mode == "replay" and bool(self.records)

    def lookup(self, tick: int) -> Optional[Dict[str, float]]:
        return self.records.get(int(tick)) if self.mode == "replay" else None

    def record(self, tick: int, ul_ms: float, compute_ms: float, dl_ms: float) -> None:
        if self.mode != "live" or not self.path:
            return
        with open(self.path, "a", encoding="utf-8") as f:
            f.write(json.dumps({"tick": int(tick), "ul_ms": float(ul_ms),
                                "compute_ms": float(compute_ms), "dl_ms": float(dl_ms)}) + "\n")
