#!/usr/bin/env python3
"""
Detailed latency profiling of AB3DMOT vs Mamba3DMOT.

Measures per-stage timing (predict, match, update) and per-tick total latency
over OPV2V held-out scenarios. Reports p50/p90/p99 and per-stage breakdowns.

Uses the same CoBEVT detections as compare_trackers.py for a fair comparison.

Usage:
    python scripts/benchmark_trackers.py
"""

import glob
import logging
import os
import pickle
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')
logger = logging.getLogger("BenchTrackers")

CMP_ROOT = "/home/atlas/TrafficSimulator_eCloud/SOTA_Predictors/CMP"


# -----------------------------------------------------------------------------
# Stage timing: monkey-patch internal functions to accumulate per-stage time.
# -----------------------------------------------------------------------------

class StageTimer:
    """Accumulates elapsed time per named stage."""

    def __init__(self):
        self.totals = defaultdict(float)
        self.counts = defaultdict(int)
        self.samples = defaultdict(list)

    def reset(self):
        self.totals.clear()
        self.counts.clear()
        self.samples.clear()

    def wrap(self, stage_name, fn):
        """Return a wrapped version of fn that records its runtime."""
        def wrapped(*args, **kwargs):
            t0 = time.perf_counter()
            result = fn(*args, **kwargs)
            dt = time.perf_counter() - t0
            self.totals[stage_name] += dt
            self.counts[stage_name] += 1
            self.samples[stage_name].append(dt * 1000.0)  # ms
            return result
        return wrapped


def instrument_mamba3dmot(timer):
    """Monkey-patch Mamba3DMOT internals to measure per-stage timing."""
    from ecav.core.tracking.mamba3dmot import matching as mamba_matching
    from ecav.core.tracking.mamba3dmot import tracker as mamba_tracker_mod

    orig_iou = mamba_matching.iou_distance_3d
    orig_linasgn = mamba_matching.linear_assignment
    orig_batched_predict = mamba_tracker_mod.Mamba3DTracker._batched_predict

    mamba_matching.iou_distance_3d = timer.wrap('mamba.iou_match', orig_iou)
    mamba_matching.linear_assignment = timer.wrap(
        'mamba.linear_assign', orig_linasgn)
    mamba_tracker_mod.Mamba3DTracker._batched_predict = timer.wrap(
        'mamba.batched_predict', orig_batched_predict)

    # Tracker module holds its own reference to matching functions
    mamba_tracker_mod.iou_distance_3d = mamba_matching.iou_distance_3d
    mamba_tracker_mod.linear_assignment = mamba_matching.linear_assignment


def instrument_ab3dmot(timer):
    """Monkey-patch AB3DMOT internals to measure per-stage timing."""
    from AB3DMOT_libs import model as ab3d_model
    from AB3DMOT_libs import matching as ab3d_matching

    # AB3DMOT's key methods
    orig_prediction = ab3d_model.AB3DMOT.prediction
    orig_data_association = ab3d_matching.data_association

    ab3d_model.AB3DMOT.prediction = timer.wrap(
        'ab3d.kf_predict', orig_prediction)
    ab3d_matching.data_association = timer.wrap(
        'ab3d.association', orig_data_association)

    # Patch the reference in model.py that's used during update
    ab3d_model.data_association = ab3d_matching.data_association


# -----------------------------------------------------------------------------
# Detection loading (same format as compare_trackers.py)
# -----------------------------------------------------------------------------

def dets_for_frame(det_dict, frame_idx):
    """Extract detections for a specific frame in world frame."""
    if frame_idx not in det_dict:
        return np.empty((0, 8)), np.empty((0, 3))

    det = det_dict[frame_idx]
    pred_box3d = det['pred_box3d_np']
    scores = det['pred_score_np']
    lidar_pose = det.get('cur_lidar_pose_np')

    if len(pred_box3d) == 0:
        return np.empty((0, 8)), np.empty((0, 3))

    if lidar_pose is not None:
        ego_x, ego_y, ego_z = float(lidar_pose[0]), float(lidar_pose[1]), float(lidar_pose[2])
        ego_yaw = np.radians(float(lidar_pose[4]))
        cos_y = np.cos(ego_yaw)
        sin_y = np.sin(ego_yaw)

        local_x = pred_box3d[:, 0]
        local_y = pred_box3d[:, 1]
        world_x = local_x * cos_y - local_y * sin_y + ego_x
        world_y = local_x * sin_y + local_y * cos_y + ego_y
        world_z = pred_box3d[:, 2] + ego_z
        world_yaw = pred_box3d[:, 6] + ego_yaw
    else:
        world_x = pred_box3d[:, 0]
        world_y = pred_box3d[:, 1]
        world_z = pred_box3d[:, 2]
        world_yaw = pred_box3d[:, 6]

    dets = np.column_stack([
        pred_box3d[:, 5], pred_box3d[:, 4], pred_box3d[:, 3],
        world_x, world_y, world_z, world_yaw, scores,
    ]).astype(np.float32)

    info = np.zeros((len(dets), 3), dtype=np.int64)
    info[:, 0] = frame_idx
    return dets, info


# -----------------------------------------------------------------------------
# Benchmark runner
# -----------------------------------------------------------------------------

def percentile(samples, p):
    if not samples:
        return 0.0
    return float(np.percentile(samples, p))


def run_benchmark(tracker_name, tracker_factory, scenarios, det_dict_by_scene,
                  stage_timer, warmup_frames=10):
    """Run a tracker through scenarios and collect per-tick latency samples."""
    stage_timer.reset()
    tick_latencies_ms = []
    track_counts = []
    det_counts = []

    total_frames = 0
    for val_file, (scenario_name, cav_id) in scenarios:
        if scenario_name not in det_dict_by_scene:
            continue
        if cav_id not in det_dict_by_scene[scenario_name]:
            continue

        with open(val_file, 'rb') as f:
            data = pickle.load(f)
        timestamps = data['timestamps']

        det_dict = det_dict_by_scene[scenario_name][cav_id]
        tracker = tracker_factory()

        for frame_idx in range(len(timestamps)):
            dets, info = dets_for_frame(det_dict, frame_idx)
            dets_all = {'dets': dets, 'info': info}

            t0 = time.perf_counter()
            tracks_list, _ = tracker.track(dets_all, frame_idx)
            dt = (time.perf_counter() - t0) * 1000.0  # ms

            if total_frames >= warmup_frames:
                tick_latencies_ms.append(dt)
                n_trks = len(tracks_list[0]) if tracks_list and len(tracks_list[0]) > 0 else 0
                track_counts.append(n_trks)
                det_counts.append(len(dets))

            total_frames += 1

    return {
        'name': tracker_name,
        'tick_latencies_ms': tick_latencies_ms,
        'track_counts': track_counts,
        'det_counts': det_counts,
        'stage_totals': dict(stage_timer.totals),
        'stage_counts': dict(stage_timer.counts),
        'stage_samples': {k: list(v) for k, v in stage_timer.samples.items()},
    }


def print_results(result):
    name = result['name']
    latencies = result['tick_latencies_ms']
    print(f"\n=== {name} ===")
    print(f"Total ticks measured: {len(latencies)}")
    print(f"Mean dets/tick: {statistics.mean(result['det_counts']):.1f}")
    print(f"Mean tracks/tick: {statistics.mean(result['track_counts']):.1f}")
    print()
    print("Per-tick total latency (ms):")
    print(f"  mean={statistics.mean(latencies):.3f}")
    print(f"  p50={percentile(latencies, 50):.3f}")
    print(f"  p90={percentile(latencies, 90):.3f}")
    print(f"  p99={percentile(latencies, 99):.3f}")
    print(f"  max={max(latencies):.3f}")
    print()
    print("Per-stage breakdown (ms, accumulated over all ticks):")
    total_stage_time = sum(result['stage_totals'].values()) * 1000.0
    for stage in sorted(result['stage_totals'], key=lambda s: -result['stage_totals'][s]):
        total_ms = result['stage_totals'][stage] * 1000.0
        count = result['stage_counts'][stage]
        mean_ms = total_ms / count if count else 0.0
        samples = result['stage_samples'][stage]
        p50 = percentile(samples, 50)
        p90 = percentile(samples, 90)
        p99 = percentile(samples, 99)
        frac = 100 * total_ms / total_stage_time if total_stage_time else 0.0
        print(f"  {stage:<30s} n={count:>6d}  mean={mean_ms:6.3f}  "
              f"p50={p50:6.3f}  p90={p90:6.3f}  p99={p99:6.3f}  "
              f"frac={frac:5.1f}%")


def main():
    val_files = sorted(glob.glob(os.path.join(
        CMP_ROOT, "preprocessed_data/opv2v/gt_multiego_speedless/cv_val/*.pickle")))
    logger.info("Evaluation scenarios: %d", len(val_files))

    # Pre-parse scenario/cav from filenames
    scenarios = []
    for val_file in val_files:
        basename = os.path.basename(val_file).replace('.pickle', '')
        parts = basename.split('-')
        scenario_name = parts[0]
        cav_id = parts[1]
        scenarios.append((val_file, (scenario_name, cav_id)))

    # Load detections
    det_file = os.path.join(
        CMP_ROOT, "preprocessed_data/opv2v",
        "corpbevtlidar_delay_1_frame_aug_c256_detection_result_for_tracking_test.pkl")
    with open(det_file, 'rb') as f:
        all_dets = pickle.load(f)
    logger.info("Detection results loaded: %d scenarios", len(all_dets))

    from ecav.core.tracking import get_tracker

    mamba_ckpt = 'ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth'

    # --- AB3DMOT ---
    ab3d_timer = StageTimer()
    instrument_ab3dmot(ab3d_timer)
    ab3d_result = run_benchmark(
        'AB3DMOT (batched)',
        lambda: get_tracker('AB3DMOT', {'min_hits': 3, 'max_age': 6}),
        scenarios, all_dets, ab3d_timer)

    # --- Mamba3DMOT ---
    mamba_timer = StageTimer()
    instrument_mamba3dmot(mamba_timer)
    mamba_result = run_benchmark(
        'Mamba3DMOT (BEV IoU)',
        lambda: get_tracker('MAMBA3DMOT', {
            'min_hits': 3, 'max_age': 6, 'match_thresh': 0.7,
            'motion_model_path': mamba_ckpt,
        }),
        scenarios, all_dets, mamba_timer)

    print_results(ab3d_result)
    print_results(mamba_result)

    # Side-by-side summary
    print("\n" + "=" * 70)
    print("SIDE-BY-SIDE LATENCY SUMMARY")
    print("=" * 70)
    print(f"{'Metric':<30s} {'AB3DMOT':>15s} {'Mamba3D':>15s} {'Ratio':>10s}")
    print("-" * 70)
    metrics = [
        ('mean per-tick (ms)', statistics.mean(ab3d_result['tick_latencies_ms']),
         statistics.mean(mamba_result['tick_latencies_ms'])),
        ('p50 per-tick (ms)', percentile(ab3d_result['tick_latencies_ms'], 50),
         percentile(mamba_result['tick_latencies_ms'], 50)),
        ('p99 per-tick (ms)', percentile(ab3d_result['tick_latencies_ms'], 99),
         percentile(mamba_result['tick_latencies_ms'], 99)),
    ]
    for label, a, b in metrics:
        ratio = b / a if a > 0 else float('inf')
        print(f"{label:<30s} {a:>15.3f} {b:>15.3f} {ratio:>9.2f}x")


if __name__ == '__main__':
    main()
