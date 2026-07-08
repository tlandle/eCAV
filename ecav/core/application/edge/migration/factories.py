# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Conversion between Mamba3DMOT tracklets and TrackLatent payloads.

A live MambaTracklet3D carries the per-track state the migration unit
needs to transport. These helpers extract that state into a TrackLatent
(source side) and re-instantiate a tracklet from a TrackLatent into a
destination Mamba3DTracker (destination side).

Kept in a separate module so :mod:`.payload` stays free of torch and
tracker imports.
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

from ecav.core.tracking.mamba3dmot.basetrack import TrackState
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker
from ecav.core.tracking.mamba3dmot.tracklet import MambaTracklet3D

from .payload import TrackLatent

logger = logging.getLogger(__name__)


def latent_from_tracklet(
    tracklet: MambaTracklet3D,
    *,
    persistent_vehicle_id: int,
    risk_score: float = 0.0,
    last_observation_t: float = 0.0,
    history_depth: Optional[int] = None,
) -> TrackLatent:
    """Snapshot a live tracklet's per-track state into a TrackLatent.

    ``history_depth`` truncates the migrated memo/diff banks to the
    most-recent N frames. This is how the Reactive-Kalman baseline (B1) is
    expressed: ``history_depth=1`` migrates only the latest bbox and the
    latest diff, which is the information a Kalman filter carries. The
    default (``None``) migrates the full history (the proposed design).
    """
    memo = np.asarray(tracklet.memo_bank, dtype=np.float32)
    diff = np.asarray(tracklet.diff_memo_bank, dtype=np.float32)
    if history_depth is not None and history_depth > 0:
        memo = memo[-history_depth:]
        diff = diff[-history_depth:]
    pred = (
        np.asarray(tracklet.predicted_last_bbox, dtype=np.float32).copy()
        if tracklet.predicted_last_bbox is not None
        else None
    )
    return TrackLatent(
        track_id=int(tracklet.track_id),
        persistent_vehicle_id=int(persistent_vehicle_id),
        memo_bank=memo.copy(),
        diff_memo_bank=diff.copy(),
        bbox_3d=np.asarray(tracklet._bbox_3d, dtype=np.float32).copy(),
        predicted_last_bbox=pred,
        frame_id=int(tracklet.frame_id),
        start_frame=int(tracklet.start_frame),
        score=float(tracklet.score),
        is_activated=bool(tracklet.is_activated),
        state_flag=int(tracklet.state_flag),
        time_since_update=int(tracklet.time_since_update),
        risk_score=float(risk_score),
        last_observation_t=float(last_observation_t),
    )


def inject_latent_into_tracker(
    tracker: Mamba3DTracker,
    latent: TrackLatent,
    *,
    preserve_track_id: bool = True,
) -> MambaTracklet3D:
    """Re-instantiate a tracklet from a TrackLatent inside ``tracker``.

    The destination tracker must have been constructed with the same model
    config as the source. The shared motion model weights are already loaded
    by the destination's own ``Mamba3DTracker.__init__``; this function only
    restores the per-track state.

    Returns the inserted tracklet. The tracker's ``tracked_tracklets`` list
    is updated in place. The tracker's frame_id is advanced if the migrated
    track is from a newer frame than the destination currently has, so that
    the destination's clock does not regress.
    """
    # Build a tracklet with the migrated current bbox as init, then restore
    # the memo banks and metadata. The MambaTracklet3D constructor seeds
    # memo_bank with a single entry; we overwrite that with the full history.
    bbox_for_init = latent.bbox_3d.tolist()
    t = MambaTracklet3D(
        cfgs=tracker.cfgs,
        bbox_3d=bbox_for_init,
        score=latent.score,
        device=tracker.device,
    )

    # Restore Mamba motion-model input history
    t.memo_bank = [row.copy() for row in latent.memo_bank]
    t.diff_memo_bank = [row.copy() for row in latent.diff_memo_bank]
    t._bbox_3d = latent.bbox_3d.copy()
    t.predicted_last_bbox = (
        latent.predicted_last_bbox.copy()
        if latent.predicted_last_bbox is not None
        else None
    )

    # Restore identity + bookkeeping. We bypass next_id() because we want to
    # preserve the source-side track_id for cross-locale continuity.
    if preserve_track_id:
        t.track_id = int(latent.track_id)
        # Advance the basetrack id counter so future new tracks don't collide
        # with the migrated id.
        try:
            # MambaTracklet3D._count is a class attr in BaseTrack
            from ecav.core.tracking.mamba3dmot import basetrack as _bt
            _bt.BaseTrack._count = max(_bt.BaseTrack._count, int(latent.track_id))
        except Exception:  # noqa: BLE001
            logger.exception("Failed to advance BaseTrack id counter")

    t.frame_id = int(latent.frame_id)
    t.start_frame = int(latent.start_frame)
    t.score = float(latent.score)
    t.is_activated = bool(latent.is_activated)
    t.state_flag = int(latent.state_flag) if latent.state_flag is not None else TrackState.Tracked
    t.time_since_update = int(latent.time_since_update)

    tracker.tracked_tracklets.append(t)
    if tracker.frame_id < latent.frame_id:
        tracker.frame_id = int(latent.frame_id)
    return t
