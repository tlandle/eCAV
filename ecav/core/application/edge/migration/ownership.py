# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Per-track ownership handshake for cross-locale migration.

Implements the protocol's ownership layer: a per-track monotonically
increasing epoch, a prepare/commit/abort state machine, and the invariant
that at most one epoch is publishable per track at any time. The source
publishes under epoch e; prepare installs e+1 at the destination
unpublished (shadow); commit transfers the authority token, after which
only e+1 is publishable; abort or expiry flushes the shadow. Consumers
reject any epoch older than the highest they have seen.

The invariant must survive lost acknowledgements, duplicated payloads,
late/stale payloads, wrong-destination payloads, and windows in which
both sides hold state. The fault-injection microeval
(tests/test_ownership_faults.py) drives each failure and asserts the
invariant plus the degraded-mode contract: every failure degrades to an
explicit history rebuild or the planner's conservative default, never to
dual authority and never to a dropped track.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

logger = logging.getLogger(__name__)

PREPARED = "prepared"     # shadow installed at destination, unpublished
COMMITTED = "committed"   # authority token transferred
ABORTED = "aborted"       # shadow flushed


@dataclass
class TrackOwnership:
    """Authority record for one track at one node."""

    epoch: int = 0                    # highest epoch this node has seen
    publishable: bool = False         # may this node publish forecasts?
    shadow_epoch: Optional[int] = None  # prepared-but-unpublished epoch
    state: str = ABORTED


@dataclass
class OwnershipManager:
    """Ownership state machine for the tracks a node sources or hosts.

    One instance per edge. All transitions are idempotent: duplicated
    prepares/commits (retries after lost acks) must not violate the
    single-publishable-epoch invariant.
    """

    node_id: str = "node"
    tracks: Dict[int, TrackOwnership] = field(default_factory=dict)

    def _t(self, track_id: int) -> TrackOwnership:
        return self.tracks.setdefault(track_id, TrackOwnership())

    # ── source side ────────────────────────────────────────────────
    def source_owns(self, track_id: int, epoch: int = 0) -> None:
        t = self._t(track_id)
        t.epoch = max(t.epoch, epoch)
        t.publishable = True
        t.state = COMMITTED

    def source_prepare(self, track_id: int) -> Tuple[int, int]:
        """Begin a handoff: returns (payload_epoch, current_epoch).

        The source stays authoritative and publishable until commit; a
        mispredicted crossing costs a wasted transfer, never a track.
        """
        t = self._t(track_id)
        return t.epoch + 1, t.epoch

    def source_commit(self, track_id: int) -> None:
        """Authority leaves this node. Idempotent."""
        t = self._t(track_id)
        t.publishable = False
        t.state = COMMITTED

    # ── destination side ───────────────────────────────────────────
    def dest_prepare(self, track_id: int, payload_epoch: int) -> bool:
        """Install a shadow epoch. Duplicates and stale epochs are no-ops.

        Returns True when the shadow is (already) installed for this epoch.
        """
        t = self._t(track_id)
        if payload_epoch <= t.epoch:
            logger.warning("[OWN %s] stale prepare tid=%d epoch=%d<=%d",
                           self.node_id, track_id, payload_epoch, t.epoch)
            return False
        if t.shadow_epoch == payload_epoch:
            return True  # duplicate prepare (lost-ack retry): idempotent
        t.shadow_epoch = payload_epoch
        t.state = PREPARED
        return True

    def dest_commit(self, track_id: int, payload_epoch: int) -> bool:
        """Accept authority for the shadow epoch. Idempotent on retry."""
        t = self._t(track_id)
        if t.state == COMMITTED and t.epoch == payload_epoch \
                and t.publishable:
            return True  # duplicate commit: already authoritative
        if t.shadow_epoch != payload_epoch:
            logger.warning("[OWN %s] commit without matching shadow tid=%d "
                           "epoch=%d shadow=%s", self.node_id, track_id,
                           payload_epoch, t.shadow_epoch)
            return False  # degraded mode: rebuild path, never dual authority
        t.epoch = payload_epoch
        t.publishable = True
        t.shadow_epoch = None
        t.state = COMMITTED
        return True

    def dest_abort(self, track_id: int) -> None:
        """Crossing never happened (expiry): flush the shadow."""
        t = self._t(track_id)
        t.shadow_epoch = None
        if t.state == PREPARED:
            t.state = ABORTED

    # ── consumer side ──────────────────────────────────────────────
    def consumer_accept(self, track_id: int, msg_epoch: int) -> bool:
        """Reject forecasts older than the highest epoch seen."""
        t = self._t(track_id)
        if msg_epoch < t.epoch:
            return False
        t.epoch = msg_epoch
        return True

    # ── invariant probe (used by the fault-injection eval) ─────────
    def publishable_epochs(self, track_id: int):
        t = self.tracks.get(track_id)
        return [t.epoch] if (t is not None and t.publishable) else []
