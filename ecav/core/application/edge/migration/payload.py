# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""MigrationPayload: the unit migrated between locales.

A payload carries the per-track state that lets the destination locale
resume inference without warm-up. For the Mamba3DMOT tracker this is the
input history that the learned motion model conditions on (memo_bank and
diff_memo_bank) plus the bookkeeping the tracker needs to keep the track
identified and associated.

The Mamba motion model itself is shared across locales (same weights, same
architecture); the per-track state is what changes per vehicle and per
tick, and is what gets serialized into a payload.

The factory functions that build a payload from a live MambaTracklet3D and
inject it into a fresh tracker live in :mod:`.factories` to keep this
module free of torch imports.
"""
from __future__ import annotations

import logging
import pickle
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TrackLatent:
    """Per-track payload entries that the destination tracker uses to resume.

    Fields map directly to MambaTracklet3D state. ``memo_bank`` and
    ``diff_memo_bank`` are the input history the learned motion model
    conditions on; the rest is tracker bookkeeping.
    """

    # Identity
    track_id: int
    persistent_vehicle_id: int

    # Mamba motion model input history (the latent surface the model sees)
    memo_bank: np.ndarray         # (K, 7) float32, K up to max_window
    diff_memo_bank: np.ndarray    # (K, 7) float32

    # Current and predicted bbox
    bbox_3d: np.ndarray           # (7,) float32
    predicted_last_bbox: Optional[np.ndarray]  # (7,) or None

    # Track bookkeeping
    frame_id: int
    start_frame: int
    score: float
    is_activated: bool
    state_flag: int
    time_since_update: int

    # Optional predictor cache (e.g. MTR per-track attention context)
    predictor_cache: Optional[np.ndarray] = None

    # Migration metadata
    risk_score: float = 0.0
    last_observation_t: float = 0.0

    def nbytes(self) -> int:
        n = int(self.memo_bank.nbytes) + int(self.diff_memo_bank.nbytes)
        n += int(self.bbox_3d.nbytes)
        if self.predicted_last_bbox is not None:
            n += int(self.predicted_last_bbox.nbytes)
        if self.predictor_cache is not None:
            n += int(self.predictor_cache.nbytes)
        return n + 96  # rough header overhead


@dataclass
class MigrationPayload:
    """A bundle of TrackLatent records for one source -> destination migration."""

    source_locale_id: str
    destination_locale_id: str
    trigger_time_s: float
    tracks: List[TrackLatent] = field(default_factory=list)
    schema_version: int = 1

    def payload_bytes(self) -> int:
        n = 64  # header overhead
        for t in self.tracks:
            n += t.nbytes()
        return n

    def serialize(self) -> bytes:
        return pickle.dumps(self, protocol=pickle.HIGHEST_PROTOCOL)

    @classmethod
    def deserialize(cls, data: bytes) -> "MigrationPayload":
        obj = pickle.loads(data)
        if not isinstance(obj, cls):
            raise TypeError(f"Deserialized object is not MigrationPayload: {type(obj)}")
        return obj
