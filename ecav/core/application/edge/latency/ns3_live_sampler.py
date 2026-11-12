# -*- coding: utf-8 -*-
"""Live ns-3 radio for an edge manager (edge option radio=live).

The table sampler (ns3_lut_sampler.py) draws each message's delay from stored
statistics. This sampler asks the running ns-3 5G-LENA process (NS3Bridge, NR
Uu) to resolve the messages of each simulation step from the actors' current
positions. One ns-3 step is run per simulation step, with n_ue senders: the
managed CAVs at their positions relative to the gNB, and background senders
that carry the same payload on the roads around the gNB (modelled load, the
counterpart of the table's lut_n).

The server reports, per step and per sender, the uplink and the downlink
message that completed in that step and its delay. The outcome reported in a
step is applied to the message sent in that step: a step without a completion
is a lost message. Under a standing queue the delay belongs to an earlier
message, so the delay distribution, the delivery ratio and the run lengths of
loss are the live ones, shifted by the delay.
"""
import logging
import math
import os
import random
import time
from typing import List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)


class Ns3LiveSampler:
    def __init__(self, n_ue: int, dl_bytes: int, origin_xy: Tuple[float, float],
                 dt_s: float = 0.05, seed: int = 0, tag: str = ""):
        from ecav.core.networking.ns3_cosim.ns3_bridge import NS3Bridge
        self.n_ue = max(1, int(n_ue))
        self.dl_bytes = int(dl_bytes)
        self.origin = (float(origin_xy[0]), float(origin_xy[1]))
        self.dt_s = float(dt_s)
        os.environ["UU_DL_PAYLOAD_BYTES"] = str(self.dl_bytes)
        os.environ["NS_GLOBAL_VALUE"] = f"RngRun={int(seed) + 1}"
        self._bridge = NS3Bridge(max_vehicles=self.n_ue, mode="uu_uplink",
                                 tick_duration_s=self.dt_s, shm_tag=tag or "edge")
        # one ns-3 step can take minutes of wall time at a saturated load
        self._bridge.first_tick_timeout_s = 3600.0
        self._bridge.tick_timeout_s = 1800.0
        t0 = time.time()
        self._bridge.start()
        self.startup_s = time.time() - t0
        rng = random.Random(4242 + int(seed))
        # background senders: (road, lane offset, start, speed), as radio_fidelity_bench.Layout
        self._bg = []
        for i in range(self.n_ue):
            lane = (-5.25, -1.75, 1.75, 5.25)[(i // 2) % 4]
            self._bg.append((i % 2, lane, rng.uniform(-120.0, 120.0),
                             rng.uniform(8.0, 15.0) * (1.0 if lane > 0 else -1.0)))
        self._tick = None
        self._ul: List[Tuple[float, bool]] = []
        self._dl: List[Tuple[float, bool]] = []
        self.rows: List[Tuple] = []      # (tick, ul_ms, ul_ok, dl_ms, dl_ok, wall_ms) of sender 0
        self.wall_ms_total = 0.0
        self.steps = 0
        logger.info("Ns3LiveSampler: %d senders, DL %d B, gNB at (%.1f, %.1f), start-up %.0f s",
                    self.n_ue, self.dl_bytes, self.origin[0], self.origin[1], self.startup_s)

    def _positions(self, tick: int, managed_xy: Sequence[Tuple[float, float]]):
        t = tick * self.dt_s
        out = []
        for i in range(self.n_ue):
            if i < len(managed_xy):
                out.append([managed_xy[i][0] - self.origin[0], managed_xy[i][1] - self.origin[1], 1.5])
                continue
            road, lane, s0, v = self._bg[i]
            s = ((s0 + v * t + 120.0) % 240.0) - 120.0
            out.append([s, lane, 1.5] if road == 0 else [lane, s, 1.5])
        return out

    def resolve(self, tick: int, managed_xy: Sequence[Tuple[float, float]], ul_bytes: int) -> None:
        """Run the ns-3 step of this simulation step once."""
        if self._tick == tick:
            return
        t0 = time.perf_counter()
        res = self._bridge.compute_tick(self._positions(tick, managed_xy), None,
                                        self.steps, payload_bytes=int(ul_bytes))
        wall = (time.perf_counter() - t0) * 1000.0
        self.steps += 1
        self.wall_ms_total += wall
        self._tick = tick
        self._ul = [(0.0, False)] * self.n_ue
        self._dl = [(0.0, False)] * self.n_ue
        for r in res or []:
            if r.rx_id == 0xFFFF and r.tx_id < self.n_ue:
                self._ul[r.tx_id] = (float(r.delay_ms), bool(r.delivered))
            elif r.tx_id == 0xFFFE and r.rx_id < self.n_ue:
                self._dl[r.rx_id] = (float(r.delay_ms), bool(r.delivered))
        self.rows.append((int(tick), self._ul[0][0], self._ul[0][1],
                          self._dl[0][0], self._dl[0][1], wall))

    def uplink(self, tick, managed_xy, ul_bytes) -> Tuple[float, bool]:
        """Uplink of this step as the table path composes it: the delay of the
        slowest sender whose message completed, and whether the managed CAV's
        (sender 0) message completed."""
        self.resolve(tick, managed_xy, ul_bytes)
        done = [r[0] for r in self._ul if r[1]]
        return (max(done) if done else 0.0), bool(self._ul[0][1])

    def downlink(self, tick, managed_xy, ul_bytes, idx: int = 0) -> Tuple[float, bool]:
        self.resolve(tick, managed_xy, ul_bytes)
        return self._dl[min(idx, self.n_ue - 1)]

    def metrics(self) -> dict:
        import numpy as np
        if not self.rows:
            return {"steps": 0}
        a = np.array([(r[1], r[2], r[3], r[4], r[5]) for r in self.rows], float)
        ul = a[a[:, 1] > 0, 0]; dl = a[a[:, 3] > 0, 2]
        q = lambda x, p: float(np.percentile(x, p)) if x.size else float("nan")
        return {"steps": int(len(a)), "n_ue": self.n_ue, "dl_bytes": self.dl_bytes,
                "ul_delivery": float(a[:, 1].mean()), "dl_delivery": float(a[:, 3].mean()),
                "ul_p50": q(ul, 50), "ul_p95": q(ul, 95), "dl_p50": q(dl, 50), "dl_p95": q(dl, 95),
                "wall_ms_per_step": float(a[:, 4].mean()), "startup_s": float(self.startup_s)}

    def shutdown(self) -> None:
        try:
            self._bridge.shutdown()
        except Exception as exc:   # the run ends either way
            logger.warning("Ns3LiveSampler shutdown: %s", exc)
