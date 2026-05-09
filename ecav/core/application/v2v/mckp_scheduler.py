# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
MCKP (Multi-dimensional Continuous Knapsack Problem) greedy scheduler
for bandwidth-aware V2V object sharing.

Based on AutoCast (Qian et al., arXiv:2112.14947). Each vehicle detects
objects locally and shares selected object point clouds with peers under
a per-frame bandwidth budget. Objects are prioritized by their utility
to the receiver: objects that are occluded from the receiver's perspective
have higher value.

The scheduler solves a greedy knapsack: maximize total utility subject to
a transmission time budget (100ms per frame at the configured data rate).
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("MCKPScheduler")

# Default C-V2X Mode 4 data rate: 10 MHz channel, MCS dependent
# Conservative estimate: ~6 Mbps effective after MAC overhead
_DEFAULT_RATE_MBPS = 6.0
_DEFAULT_SLOT_MS = 100.0  # transmission window per frame
_BYTES_PER_POINT = 12  # 3 floats x 4 bytes


@dataclass
class SharedObject:
    """A detected object offered for V2V sharing."""
    sender_id: int
    object_id: int
    points: np.ndarray          # (N, 3) point cloud in sender's local frame
    bbox_center: np.ndarray     # (3,) center in world coordinates
    bbox_extent: np.ndarray     # (3,) half-extents (l/2, w/2, h/2)
    score: float = 1.0
    speed: float = 0.0

    @property
    def n_points(self) -> int:
        return len(self.points) if self.points is not None else 0

    @property
    def size_bytes(self) -> int:
        return self.n_points * _BYTES_PER_POINT


def transmission_time_ms(n_points: int, rate_mbps: float = _DEFAULT_RATE_MBPS) -> float:
    """Time in ms to transmit n_points at given data rate."""
    bits = n_points * _BYTES_PER_POINT * 8
    return (bits / (rate_mbps * 1e6)) * 1000.0


def compute_occlusion_utility(
    obj_world_pos: np.ndarray,
    receiver_pos: np.ndarray,
    receiver_detected_positions: List[np.ndarray],
    occlusion_threshold: float = 5.0,
) -> float:
    """
    Compute utility of sharing an object with a receiver.

    An object has high utility if the receiver has NOT already detected
    something at that position. Objects already visible to the receiver
    have zero utility.

    Parameters
    ----------
    obj_world_pos : (3,) world position of the shared object
    receiver_pos : (3,) world position of the receiver
    receiver_detected_positions : list of (3,) positions the receiver detected
    occlusion_threshold : if the closest receiver detection is farther than
        this from obj_world_pos, the object is considered novel/occluded.

    Returns
    -------
    utility : float in [0, 1]
    """
    if len(receiver_detected_positions) == 0:
        return 1.0

    min_dist = float('inf')
    for det_pos in receiver_detected_positions:
        d = np.linalg.norm(obj_world_pos[:2] - det_pos[:2])
        if d < min_dist:
            min_dist = d

    if min_dist < occlusion_threshold:
        return 0.0  # receiver already sees this object
    return 1.0


def mckp_greedy_schedule(
    offered_objects: List[SharedObject],
    receiver_pos: np.ndarray,
    receiver_detected_positions: List[np.ndarray],
    budget_ms: float = _DEFAULT_SLOT_MS,
    rate_mbps: float = _DEFAULT_RATE_MBPS,
) -> List[SharedObject]:
    """
    Greedy MCKP scheduler: select objects to transmit under bandwidth budget.

    Objects are scored by utility/size ratio and greedily selected until
    the transmission time budget is exhausted.

    Parameters
    ----------
    offered_objects : list of SharedObject candidates from all senders
    receiver_pos : (3,) receiver world position
    receiver_detected_positions : list of (3,) positions receiver already detects
    budget_ms : per-frame transmission time budget in ms
    rate_mbps : effective data rate in Mbps

    Returns
    -------
    selected : list of SharedObject to transmit
    """
    if not offered_objects:
        return []

    # Score each object
    scored = []
    for obj in offered_objects:
        utility = compute_occlusion_utility(
            obj.bbox_center, receiver_pos, receiver_detected_positions)
        tx_ms = transmission_time_ms(obj.n_points, rate_mbps)
        if tx_ms < 1e-6:
            continue
        ratio = utility / tx_ms
        scored.append((ratio, utility, tx_ms, obj))

    # Sort by utility/time ratio, descending
    scored.sort(key=lambda x: x[0], reverse=True)

    selected = []
    remaining_ms = budget_ms
    for ratio, utility, tx_ms, obj in scored:
        if utility <= 0:
            continue
        if tx_ms > remaining_ms:
            continue
        selected.append(obj)
        remaining_ms -= tx_ms

    logger.debug("MCKP: %d/%d objects selected, %.1f/%.1f ms used",
                 len(selected), len(offered_objects),
                 budget_ms - remaining_ms, budget_ms)
    return selected
