# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V channel and cooperative perception metrics.

Tracks PRR, CBR, AoI, feature delivery fraction, SINR distribution.
"""
from __future__ import annotations

import logging
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Tuple

import numpy as np

logger = logging.getLogger("V2VMetrics")


@dataclass
class V2VTickStats:
    tick: int = 0
    n_vehicles: int = 0
    n_links: int = 0
    n_delivered: int = 0
    prr: float = 0.0
    cbr: float = 0.0
    mean_sinr_db: float = 0.0
    median_sinr_db: float = 0.0
    per_vehicle_received: Dict[int, int] = field(default_factory=dict)


class V2VMetrics:
    def __init__(self, history_len: int = 500):
        self.history: Deque[V2VTickStats] = deque(maxlen=history_len)
        self._last_delivery_tick: Dict[Tuple[int, int], int] = {}

    def record_tick(self, tick, link_results, n_vehicles, subchannel_assignments):
        n_links = len(link_results)
        n_delivered = sum(1 for lr in link_results if lr.delivered)
        prr = n_delivered / max(n_links, 1)

        sinr_values = [lr.sinr_db for lr in link_results]
        mean_sinr = float(np.mean(sinr_values)) if sinr_values else 0.0
        median_sinr = float(np.median(sinr_values)) if sinr_values else 0.0

        sc_counts: Dict[int, int] = defaultdict(int)
        for sc in subchannel_assignments:
            sc_counts[sc] += 1
        num_sc = max(len(sc_counts), 1)
        cbr = sum(1 for c in sc_counts.values() if c > 1) / num_sc

        per_veh: Dict[int, int] = defaultdict(int)
        for lr in link_results:
            if lr.delivered:
                per_veh[lr.rx_id] += 1
                self._last_delivery_tick[(lr.tx_id, lr.rx_id)] = tick

        stats = V2VTickStats(
            tick=tick, n_vehicles=n_vehicles, n_links=n_links,
            n_delivered=n_delivered, prr=prr, cbr=cbr,
            mean_sinr_db=mean_sinr, median_sinr_db=median_sinr,
            per_vehicle_received=dict(per_veh))
        self.history.append(stats)
        return stats

    def get_aoi(self, tx_id, rx_id, current_tick):
        last = self._last_delivery_tick.get((tx_id, rx_id))
        return current_tick if last is None else current_tick - last

    def get_mean_prr(self, last_n=50):
        recent = list(self.history)[-last_n:]
        return float(np.mean([s.prr for s in recent])) if recent else 0.0

    def get_mean_cbr(self, last_n=50):
        recent = list(self.history)[-last_n:]
        return float(np.mean([s.cbr for s in recent])) if recent else 0.0

    def get_feature_delivery_fraction(self, last_n=50):
        recent = list(self.history)[-last_n:]
        if not recent:
            return 0.0
        fracs = []
        for s in recent:
            n_peers = max(s.n_vehicles - 1, 1)
            for vid, count in s.per_vehicle_received.items():
                fracs.append(count / n_peers)
        return float(np.mean(fracs)) if fracs else 0.0

    def summary_dict(self):
        return {
            "total_ticks": len(self.history),
            "mean_prr": self.get_mean_prr(),
            "mean_cbr": self.get_mean_cbr(),
            "mean_feature_delivery_frac": self.get_feature_delivery_fraction(),
            "last_tick": self.history[-1].tick if self.history else -1,
        }
