# -*- coding: utf-8 -*-
"""Perception error models for a vehicle that reads its objects from the simulator.

With perception off a vehicle receives the simulator's vehicle list (perception_manager
deactivate_mode), optionally limited to the range and line of sight of its sensors. An error
model turns that list into the list a detector would report, at the cost of a table lookup:

  independent   every vehicle in view is dropped with a fixed probability in every tick,
                independently, and a reported vehicle gets white Gaussian position noise
  calibrated    the detection probability depends on the distance, on how much of the vehicle
                is in view (unblocked rays of five), and on whether the vehicle was detected in
                the previous tick, so misses come in runs. The position error has a radial
                bias, a radial and a tangential spread per distance bin, and is correlated
                between ticks. The table is fitted to a detection trace of the full sensor
                pipeline (scripts/ispass/perception_model_fit.py).

Neither model adds objects that do not exist.
"""
import json
import math
import random


class PerceptionErrorModel:
    def __init__(self, cfg, seed=0):
        self.kind = str(cfg.get('type', 'independent'))
        self.rng = random.Random(int(cfg.get('seed', 0)) * 1000003 + int(seed))
        self.prev = {}        # actor id -> (detected in the previous tick, radial error, tangential error)
        self.last_tick = {}   # actor id -> tick of the previous call in which the actor was in view
        if self.kind == 'independent':
            self.p_miss = float(cfg.get('p_miss', 0.0))
            self.sigma = float(cfg.get('pos_sigma_m', 0.0))
        elif self.kind == 'calibrated':
            with open(cfg['file']) as fh:
                t = json.load(fh)
            self.dist_edges = [float(x) for x in t['dist_edges']]          # bin i covers [edges[i], edges[i+1])
            self.vis_bins = [int(x) for x in t['vis_bins']]                # upper bounds of the visibility bins
            self.p_first = t['p_first']                                    # [dist][vis] detection in the first tick in view
            self.p_dd = t['p_det_given_det']                               # [dist][vis]
            self.p_dm = t['p_det_given_miss']                              # [dist][vis]
            self.bias_r = t['radial_bias_m']                               # [dist]
            self.sig_r = t['radial_sigma_m']                               # [dist]
            self.sig_t = t['tangential_sigma_m']                           # [dist]
            self.rho = float(t.get('error_correlation', 0.0))
        else:
            raise ValueError("unknown perception error model: %s" % self.kind)

    def _dbin(self, d):
        e = self.dist_edges
        for i in range(len(e) - 1):
            if d < e[i + 1]:
                return i
        return len(e) - 2

    def _vbin(self, vis):
        for i, hi in enumerate(self.vis_bins):
            if vis <= hi:
                return i
        return len(self.vis_bins) - 1

    def apply(self, tick, items):
        """items: (actor id, distance in m, unblocked rays 1..5) for every vehicle in view.
        Returns {actor id: (radial error, tangential error)} for the vehicles that are reported."""
        out = {}
        for aid, d, vis in items:
            # a vehicle that was out of view for a tick starts again without history
            fresh = self.last_tick.get(aid) != tick - 1
            self.last_tick[aid] = tick
            if self.kind == 'independent':
                if self.rng.random() >= self.p_miss:
                    out[aid] = (self.rng.gauss(0.0, self.sigma), self.rng.gauss(0.0, self.sigma))
                continue
            i, j = self._dbin(d), self._vbin(vis)
            was, er0, et0 = self.prev.get(aid, (None, 0.0, 0.0))
            if fresh or was is None:
                p = self.p_first[i][j]
            else:
                p = self.p_dd[i][j] if was else self.p_dm[i][j]
            det = self.rng.random() < p
            if det:
                # first-order autoregressive error around the radial bias of the distance bin
                k = 0.0 if (fresh or not was) else self.rho
                s = math.sqrt(max(0.0, 1.0 - k * k))
                er = self.bias_r[i] + k * (er0 - self.bias_r[i]) + s * self.rng.gauss(0.0, self.sig_r[i])
                et = k * et0 + s * self.rng.gauss(0.0, self.sig_t[i])
                out[aid] = (er, et)
                self.prev[aid] = (True, er, et)
            else:
                self.prev[aid] = (False, er0, et0)
        return out
