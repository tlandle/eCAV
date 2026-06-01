# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""MigrationPayload: the unit migrated between locales.

The payload carries the model's latent state for one or more tracks:
the sequence-model tracker's recurrent hidden state plus the multi-modal
predictor's per-track attention cache, together with track identity and
per-track risk metadata. This is the artifact that lets the destination
locale resume inference without warm-up.

The payload is intentionally minimal. Container-level state (process,
operating system, model weights) is assumed to already exist at the
destination because every locale runs an instance of the same service.
What changes per migration is only the learned per-track state.

The dataclass is serializable via pickle for the parametric inter-locale
link model in :mod:`.link`. A real deployment would replace pickle with a
length-prefixed binary protocol; the serializer interface is the same.
"""
from __future__ import annotations

import io
import logging
import pickle
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TrackLatent:
    """Per-track payload entries that the destination tracker uses to resume."""

    track_id: int
    persistent_vehicle_id: int
    hidden_state: np.ndarray  # tracker's recurrent state, shape (D,) float16
    predictor_cache: Optional[np.ndarray] = None  # predictor's per-track context
    risk_score: float = 0.0
    last_observation_t: float = 0.0  # simulator time of latest input used

    def nbytes(self) -> int:
        n = int(self.hidden_state.nbytes)
        if self.predictor_cache is not None:
            n += int(self.predictor_cache.nbytes)
        # identity + scalars ~ 32 B
        return n + 32


@dataclass
class MigrationPayload:
    """A bundle of TrackLatent records for one source->destination migration.

    The payload is created by the source locale's :class:`MigrationDaemon`
    once the :class:`TrajectoryTrigger` fires. It crosses the inter-locale
    link as a single message and is consumed on the destination side by the
    receiving daemon.
    """

    source_locale_id: str
    destination_locale_id: str
    trigger_time_s: float
    tracks: List[TrackLatent] = field(default_factory=list)
    schema_version: int = 1

    # ------------------------------------------------------------------
    # Size accounting
    # ------------------------------------------------------------------
    def payload_bytes(self) -> int:
        """Total serialized size in bytes (best-effort sum across tracks).

        Used by :class:`InterLocaleLink` to compute transfer time from the
        link's bandwidth parameter without materializing the wire bytes.
        """
        n = 64  # header overhead
        for t in self.tracks:
            n += t.nbytes()
        return n

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------
    def serialize(self) -> bytes:
        return pickle.dumps(self, protocol=pickle.HIGHEST_PROTOCOL)

    @classmethod
    def deserialize(cls, data: bytes) -> "MigrationPayload":
        obj = pickle.loads(data)
        if not isinstance(obj, cls):
            raise TypeError(f"Deserialized object is not MigrationPayload: {type(obj)}")
        return obj
