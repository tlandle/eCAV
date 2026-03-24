# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Per-vehicle feature exchange for V2V cooperative perception.

Each vehicle broadcasts features over C-V2X sidelink. This module
determines which features each vehicle successfully receives based on
channel delivery decisions from the channel engine.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import torch

logger = logging.getLogger("V2VFusion")


@dataclass
class FeatureBudget:
    """Controls feature compression for sidelink transmission."""
    confidence_threshold: float = 0.1
    max_bytes_per_tti: int = 175_000

    def apply_mask(
        self, features: torch.Tensor, confidence_map: Optional[torch.Tensor]
    ) -> torch.Tensor:
        if confidence_map is None:
            return features
        mask = (confidence_map > self.confidence_threshold).float()
        return features * mask


@dataclass
class V2VFeatureExchange:
    """Manages per-vehicle feature collection based on channel delivery."""
    feature_budget: FeatureBudget = field(default_factory=FeatureBudget)

    def collect_received_features(
        self,
        rx_id: int,
        all_features: Dict[int, Dict],
        delivery_map: Dict[Tuple[int, int], bool],
    ) -> List[Dict]:
        """Collect features that vehicle rx_id successfully received."""
        received = []

        # Own features always available
        if rx_id in all_features:
            own = all_features[rx_id].copy()
            own["_source"] = "self"
            received.append(own)

        # Peer features: only if channel delivered
        for tx_id, feat in all_features.items():
            if tx_id == rx_id:
                continue
            if delivery_map.get((tx_id, rx_id), False):
                peer = feat.copy()
                peer["_source"] = "peer"
                if "spatial_features" in peer:
                    peer["spatial_features"] = self.feature_budget.apply_mask(
                        peer["spatial_features"],
                        peer.get("confidence_map"),
                    )
                received.append(peer)

        return received

    def build_delivery_map(self, link_results: list) -> Dict[Tuple[int, int], bool]:
        """Convert channel engine LinkResult list to delivery map."""
        return {(lr.tx_id, lr.rx_id): lr.delivered for lr in link_results}

    @staticmethod
    def count_delivered(
        delivery_map: Dict[Tuple[int, int], bool], rx_id: int
    ) -> int:
        return sum(1 for (tx, rx), d in delivery_map.items()
                   if rx == rx_id and d)
