# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Synthetic harness for the predictive latent migration mechanism.

Drives a single-vehicle synthetic detection trace through two independent
Mamba3DTracker instances ("edge A" and "edge B") sharing the same trained
MambaTrack motion-model weights. Migration happens at a configurable
handoff frame: the source tracklet's per-track state is snapshotted into a
TrackLatent, serialized + deserialized (pickle round-trip), and injected
into the destination tracker as a fresh tracklet.

Three checks validate the mechanism:

* **State parity** -- after migration, the destination tracklet's
  memo_bank and diff_memo_bank are byte-equal to the source's.
* **Prediction parity** -- on the next ``predict()`` call from the same
  bbox history, the destination tracker emits the same predicted bbox as
  the source would have (within float32 round-trip tolerance).
* **Cold-start delta** -- a control "edge C" runs the same post-handoff
  detections with NO migration. Its per-frame prediction error against
  ground truth is compared to the migrated edge B. The delta is the
  prediction gap the migration closes.

Run:
    python -m ecav.core.application.edge.migration.harness
    python -m ecav.core.application.edge.migration.harness --handoff 8 --quiet
"""
from __future__ import annotations

import argparse
import copy
import logging
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import torch

from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker

from .factories import inject_latent_into_tracker, latent_from_tracklet
from .payload import MigrationPayload, TrackLatent

logger = logging.getLogger(__name__)


# ──────────────────────────────────────────────────────────────────────────────
#  Shared tracker config
# ──────────────────────────────────────────────────────────────────────────────
def _tracker_cfg(weights_path: str) -> dict:
    return {
        "motion_model_path": weights_path,
        "filter_thresh": 0.05,
        "new_track_thresh": 0.2,
        "match_thresh": 5.0,        # generous; only one track in the harness
        "max_time_lost": 60,
        "enable_time_thresh": 5,
        "max_window": 10,
        "d_m": 512,
        "d_state": 16,
        "L": 3,
        "box_dim": 7,
        "avg_pool_out_dim": [1, 128],
        "pred_head_dims": [64, 7],
    }


# ──────────────────────────────────────────────────────────────────────────────
#  Synthetic trajectories
# ──────────────────────────────────────────────────────────────────────────────
def _fill_box(out: np.ndarray) -> None:
    out[:, 3] = 4.5  # l
    out[:, 4] = 2.0  # w
    out[:, 5] = 1.5  # h


def traj_straight(n_frames: int, dt: float = 0.05) -> np.ndarray:
    """Straight at 20 m/s with mild lateral wobble (benign baseline)."""
    v = 20.0
    out = np.zeros((n_frames, 7), dtype=np.float32)
    _fill_box(out)
    for i in range(n_frames):
        t = i * dt
        out[i, 0] = v * t
        out[i, 1] = 0.5 * np.sin(0.3 * t)
        out[i, 6] = 0.05 * np.cos(0.3 * t)
    return out


def traj_turn(n_frames: int, dt: float = 0.05, handoff: int = 10) -> np.ndarray:
    """Straight before handoff, then a sustained right-hand turn after.

    Cold-start sees only post-turn frames and cannot fit the rotating
    velocity vector quickly. Migrated state already encodes the pre-turn
    motion, so the predictor enters the turn with a correct prior.
    """
    v = 20.0           # m/s tangential
    R = 30.0           # turn radius
    omega = v / R      # rad/s yaw rate (entering the turn)
    out = np.zeros((n_frames, 7), dtype=np.float32)
    _fill_box(out)
    # Pre-turn segment: straight along +x
    for i in range(min(handoff, n_frames)):
        out[i, 0] = v * (i * dt)
    # Post-turn segment: circular arc tangent to the straight line at handoff
    if handoff < n_frames:
        x0 = v * (handoff * dt)
        for j in range(n_frames - handoff):
            theta = omega * (j * dt)
            out[handoff + j, 0] = x0 + R * np.sin(theta)
            out[handoff + j, 1] = -R * (1.0 - np.cos(theta))
            out[handoff + j, 6] = -theta
    return out


def traj_brake(n_frames: int, dt: float = 0.05, handoff: int = 10) -> np.ndarray:
    """Cruise at 20 m/s, then hard brake (8 m/s^2) starting at handoff."""
    v0 = 20.0
    a = -8.0  # hard brake
    out = np.zeros((n_frames, 7), dtype=np.float32)
    _fill_box(out)
    x = 0.0
    v = v0
    for i in range(n_frames):
        if i >= handoff:
            v = max(0.0, v + a * dt)
        x += v * dt
        out[i, 0] = x
    return out


def traj_lane_change(n_frames: int, dt: float = 0.05, handoff: int = 10) -> np.ndarray:
    """Cruise straight, then commit a lateral lane change starting at handoff.

    The lane change is a 3.5 m lateral shift over 1.5 s, modeled as a
    sigmoid in y(t). Yaw tracks the lateral velocity.
    """
    v = 20.0
    lane_w = 3.5
    duration = 1.5
    out = np.zeros((n_frames, 7), dtype=np.float32)
    _fill_box(out)
    for i in range(n_frames):
        t = i * dt
        out[i, 0] = v * t
        if i >= handoff:
            tau = (i - handoff) * dt / duration
            tau = min(max(tau, 0.0), 1.0)
            # Smooth-step (cubic) lateral
            s = 3 * tau ** 2 - 2 * tau ** 3
            ds_dt = (6 * tau - 6 * tau ** 2) / duration if 0 < tau < 1 else 0.0
            out[i, 1] = lane_w * s
            out[i, 6] = float(np.arctan2(lane_w * ds_dt, v))
    return out


TRAJECTORIES = {
    "straight": traj_straight,
    "turn": traj_turn,
    "brake": traj_brake,
    "lane_change": traj_lane_change,
}


# Backward-compat alias for the original entry point
def synthetic_trajectory(n_frames: int = 30, dt: float = 0.05) -> np.ndarray:
    return traj_straight(n_frames, dt)


# ──────────────────────────────────────────────────────────────────────────────
#  Per-frame drive
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class FrameResult:
    frame: int
    track_id: Optional[int]
    predicted_bbox: Optional[np.ndarray]
    actual_bbox: np.ndarray

    @property
    def error_xy(self) -> float:
        if self.predicted_bbox is None:
            return float("nan")
        dx = self.predicted_bbox[0] - self.actual_bbox[0]
        dy = self.predicted_bbox[1] - self.actual_bbox[1]
        return float(np.hypot(dx, dy))


def _drive(
    tracker: Mamba3DTracker,
    detections: np.ndarray,
    *,
    start_frame: int = 0,
) -> List[FrameResult]:
    """Feed detections one frame at a time. Returns per-frame results."""
    results: List[FrameResult] = []
    for i, det in enumerate(detections):
        scores = np.array([0.95], dtype=np.float32)
        active = tracker.update(det.reshape(1, -1), scores)
        tracklet = active[0] if len(active) else None
        pred = (
            tracklet.predicted_last_bbox.copy()
            if tracklet is not None and tracklet.predicted_last_bbox is not None
            else None
        )
        results.append(FrameResult(
            frame=start_frame + i,
            track_id=int(tracklet.track_id) if tracklet else None,
            predicted_bbox=pred,
            actual_bbox=det.copy(),
        ))
    return results


# ──────────────────────────────────────────────────────────────────────────────
#  Harness
# ──────────────────────────────────────────────────────────────────────────────
def run(
    weights_path: str,
    *,
    handoff_frame: int = 10,
    total_frames: int = 30,
    trajectory: str = "straight",
    verbose: bool = True,
) -> dict:
    device = "cuda" if torch.cuda.is_available() else "cpu"
    cfg = _tracker_cfg(weights_path)

    if trajectory not in TRAJECTORIES:
        raise ValueError(f"unknown trajectory {trajectory!r}; "
                         f"choices: {sorted(TRAJECTORIES)}")
    gen = TRAJECTORIES[trajectory]
    # The maneuver-style trajectories accept a handoff arg so the
    # event begins exactly where edge A ends.
    if trajectory == "straight":
        traj = gen(total_frames)
    else:
        traj = gen(total_frames, handoff=handoff_frame)
    pre = traj[:handoff_frame]
    post = traj[handoff_frame:]

    if verbose:
        print(f"device={device}  trajectory={trajectory}  "
              f"handoff_frame={handoff_frame}  total_frames={total_frames}")

    # ── Edge A: source ─────────────────────────────────────────────────
    edge_a = Mamba3DTracker(cfg, device=device)
    a_results = _drive(edge_a, pre, start_frame=0)
    src = edge_a.tracked_tracklets[0]
    if verbose:
        print(f"\nedge A after {handoff_frame} frames: track_id={src.track_id} "
              f"memo_len={len(src.memo_bank)} score={src.score:.3f}")

    # ── Snapshot + serialize + deserialize ─────────────────────────────
    latent_a = latent_from_tracklet(src, persistent_vehicle_id=42, last_observation_t=0.5)
    payload = MigrationPayload(
        source_locale_id="A", destination_locale_id="B",
        trigger_time_s=handoff_frame * 0.05, tracks=[latent_a],
    )
    wire_bytes = payload.serialize()
    rx = MigrationPayload.deserialize(wire_bytes)
    latent_b = rx.tracks[0]
    if verbose:
        print(f"payload bytes={len(wire_bytes)} (declared={payload.payload_bytes()})")

    # ── Edge B: destination (receives migration) ───────────────────────
    edge_b = Mamba3DTracker(cfg, device=device)
    dst = inject_latent_into_tracker(edge_b, latent_b)

    # CHECK 1: state parity ─────────────────────────────────────────────
    src_memo = np.asarray(src.memo_bank, dtype=np.float32)
    dst_memo = np.asarray(dst.memo_bank, dtype=np.float32)
    src_diff = np.asarray(src.diff_memo_bank, dtype=np.float32)
    dst_diff = np.asarray(dst.diff_memo_bank, dtype=np.float32)
    assert np.array_equal(src_memo, dst_memo), "memo_bank mismatch"
    assert np.array_equal(src_diff, dst_diff), "diff_memo_bank mismatch"
    assert src.track_id == dst.track_id, "track_id mismatch"
    if verbose:
        print(f"\nCHECK 1 state parity: OK  "
              f"(memo_bank {src_memo.shape}, diff {src_diff.shape})")

    # CHECK 2: prediction parity ────────────────────────────────────────
    # Run predict on a deep copy of each tracker so we don't touch the
    # state needed for downstream drives.
    src_clone = copy.deepcopy(src)
    src_clone.predict()
    dst_clone = copy.deepcopy(dst)
    dst_clone.predict()
    pred_diff = float(np.linalg.norm(src_clone.predicted_last_bbox -
                                     dst_clone.predicted_last_bbox))
    if verbose:
        print(f"CHECK 2 prediction parity: ||pred_A - pred_B|| = {pred_diff:.6e}")
    assert pred_diff < 1e-4, f"prediction parity failed: {pred_diff}"

    # ── Continue both edges through the rest of the trajectory ─────────
    b_results = _drive(edge_b, post, start_frame=handoff_frame)

    # ── Edge C: cold start, no migration ───────────────────────────────
    edge_c = Mamba3DTracker(cfg, device=device)
    c_results = _drive(edge_c, post, start_frame=handoff_frame)

    # CHECK 3: cold-start delta ─────────────────────────────────────────
    b_errors = [r.error_xy for r in b_results if not np.isnan(r.error_xy)]
    c_errors = [r.error_xy for r in c_results if not np.isnan(r.error_xy)]
    # The first few frames after cold start have no prediction at all (until
    # the tracker accumulates enough history). Compare on overlap of frames
    # for which both have a prediction.
    overlap_len = min(len(b_errors), len(c_errors))
    if overlap_len:
        b_mean = float(np.mean(b_errors[:overlap_len]))
        c_mean = float(np.mean(c_errors[:overlap_len]))
        if verbose:
            print(f"\nCHECK 3 cold-start delta over first {overlap_len} comparable post-handoff frames:")
            print(f"  migrated edge B mean xy error: {b_mean:.4f} m")
            print(f"  cold     edge C mean xy error: {c_mean:.4f} m")
            print(f"  prediction gap (C - B): {c_mean - b_mean:+.4f} m")

    return {
        "src_track": src,
        "dst_track": dst,
        "wire_bytes": len(wire_bytes),
        "a_results": a_results,
        "b_results": b_results,
        "c_results": c_results,
        "pred_diff": pred_diff,
    }


def _summary_row(name: str, out: dict, gap_window: int = 5) -> dict:
    """Frame-aligned comparison between migrated (b) and cold-start (c).

    Returns a dict of metrics. Frames where cold-start has no prediction
    yet are counted as missing (and reported as such), not silently aligned
    with a later cold-start prediction.
    """
    n_frames = len(out["b_results"])
    b_full = np.array([r.error_xy for r in out["b_results"]])  # may contain NaN
    c_full = np.array([r.error_xy for r in out["c_results"]])

    # Migrated overall mean (NaN-aware)
    b_mean = float(np.nanmean(b_full)) if np.any(~np.isnan(b_full)) else float("nan")
    c_mean = float(np.nanmean(c_full)) if np.any(~np.isnan(c_full)) else float("nan")

    # First-N-frames gap window: this is where cold-start hurts most
    w = min(gap_window, n_frames)
    b_window = b_full[:w]
    c_window = c_full[:w]
    b_win_mean = float(np.nanmean(b_window)) if np.any(~np.isnan(b_window)) else float("nan")
    c_win_mean = float(np.nanmean(c_window)) if np.any(~np.isnan(c_window)) else float("nan")

    # Fraction of post-handoff frames where each had a prediction
    b_cov = float(np.mean(~np.isnan(b_full)))
    c_cov = float(np.mean(~np.isnan(c_full)))

    ratio = c_mean / b_mean if (b_mean and b_mean > 0) else float("inf")
    return {
        "name": name,
        "bytes": out["wire_bytes"],
        "b_mean": b_mean,
        "c_mean": c_mean,
        "b_win": b_win_mean,
        "c_win": c_win_mean,
        "ratio": ratio,
        "b_cov": b_cov,
        "c_cov": c_cov,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--weights",
        default="ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth",
    )
    ap.add_argument("--handoff", type=int, default=10)
    ap.add_argument("--total", type=int, default=30)
    ap.add_argument("--trajectory", choices=list(TRAJECTORIES) + ["all"], default="all")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.WARNING)
    trajs = list(TRAJECTORIES) if args.trajectory == "all" else [args.trajectory]
    rows = []
    for name in trajs:
        out = run(
            args.weights,
            handoff_frame=args.handoff,
            total_frames=args.total,
            trajectory=name,
            verbose=not args.quiet,
        )
        rows.append(_summary_row(name, out))
        if not args.quiet:
            print()

    print("=" * 96)
    print(f"{'trajectory':<13} {'bytes':>6}  "
          f"{'mig mean':>10} {'cold mean':>10} {'ratio':>6}   "
          f"{'mig 5-frame':>11} {'cold 5-frame':>12}   "
          f"{'mig cov':>7} {'cold cov':>8}")
    print("-" * 96)
    for r in rows:
        print(f"{r['name']:<13} {r['bytes']:>6d}  "
              f"{r['b_mean']:>9.4f}m {r['c_mean']:>9.4f}m {r['ratio']:>5.2f}x   "
              f"{r['b_win']:>10.4f}m {r['c_win']:>11.4f}m   "
              f"{r['b_cov']:>6.0%} {r['c_cov']:>7.0%}")
    print("=" * 96)
    return 0


if __name__ == "__main__":
    sys.exit(main())
