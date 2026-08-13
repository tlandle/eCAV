# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""MigrationPayload: the unit migrated between locales.

A payload carries the per-track state that lets the destination locale
resume inference without warm-up. Two tracker backends are supported:

* MambaTrack: the input history the learned motion model conditions on
  (``memo_bank`` and ``diff_memo_bank``) plus the tracker bookkeeping needed
  to keep the track identified and associated. The factory functions that
  build this from a live MambaTracklet3D live in :mod:`.factories` to keep
  this module free of torch imports.
* AB3DMOT: an explicit Kalman snapshot (:class:`KFState`) extracted from the
  source edge's filter, injected as a warm KF at the destination.

In both cases the model (weights, architecture) is shared across locales;
only the per-track state changes per vehicle and per tick and is serialized
into a payload. The serialized size here is a model for the simulated
inter-locale link; a real-world deployment would carry the same per-track
state over a real network protocol rather than pickle.
"""
from __future__ import annotations

import logging
import pickle
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class KFState:
    """Kalman filter state snapshot for one AB3DMOT track.

    Extracted from the source edge's KF.kf and carried in TrackLatent so the
    destination edge can inject a warm KF without a cold-start dwell period.
    """

    state_vector: np.ndarray  # (10,) [x,y,z,θ,l,w,h,dx,dy,dz]
    covariance: np.ndarray    # (10,10)
    hits: int                 # set >= tracker.min_hits on import to skip confirmation dwell
    anchoring_age: int        # ticks since last beacon-matched update

    def nbytes(self) -> int:
        return int(self.state_vector.nbytes + self.covariance.nbytes) + 8


@dataclass
class TrackLatent:
    """Per-track payload entries that the destination tracker uses to resume.

    The dataclass carries either tracker backend's per-track state; the two
    field blocks below are mutually exclusive in practice. The MambaTrack
    block (``memo_bank``/``diff_memo_bank`` plus bbox and bookkeeping) is the
    learned-motion-model input history; the AB3DMOT block (``kf_state``) is an
    explicit Kalman snapshot. Only ``track_id`` and ``persistent_vehicle_id``
    are always set; every backend-specific field defaults so a payload can be
    built from either tracker.
    """

    # Identity (always present)
    track_id: int
    persistent_vehicle_id: int

    # --- MambaTrack backend: learned-motion-model input history + bookkeeping ---
    memo_bank: Optional[np.ndarray] = None         # (K, 7) float32, K up to max_window
    diff_memo_bank: Optional[np.ndarray] = None    # (K, 7) float32
    bbox_3d: Optional[np.ndarray] = None           # (7,) float32
    predicted_last_bbox: Optional[np.ndarray] = None  # (7,) or None
    frame_id: int = 0
    start_frame: int = 0
    score: float = 0.0
    is_activated: bool = False
    state_flag: int = 0
    time_since_update: int = 0
    # Ground velocity in m/s [vx, vy], measured at the source. Memo/diff banks
    # are frame-denominated in the SOURCE tracker's cadence; the destination
    # runs at its own cadence, so dead-reckoning from raw diffs mis-scales by
    # the cadence ratio (measured 2x on GT injection). Time-denominated
    # velocity is cadence-independent.
    vel_mps: Optional[np.ndarray] = None           # (2,) float32 or None

    # --- AB3DMOT backend: recurrent placeholder + explicit Kalman snapshot ---
    hidden_state: Optional[np.ndarray] = None      # recurrent tracker state (D,); reserved for sequence models
    kf_state: Optional[KFState] = None             # AB3DMOT KF state; None for sequence-model trackers

    # Shared: predictor cache (e.g. MTR per-track attention context) + migration metadata
    predictor_cache: Optional[np.ndarray] = None
    risk_score: float = 0.0
    last_observation_t: float = 0.0

    def nbytes(self) -> int:
        n = 0
        for arr in (
            self.memo_bank,
            self.diff_memo_bank,
            self.bbox_3d,
            self.predicted_last_bbox,
            self.hidden_state,
            self.predictor_cache,
        ):
            if arr is not None:
                n += int(arr.nbytes)
        if self.kf_state is not None:
            n += self.kf_state.nbytes()
        return n + 96  # rough header + identity/scalars overhead


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
