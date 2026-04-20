#!/usr/bin/env python3
"""
Compare AB3DMOT vs SSM3DMOT tracking accuracy on OPV2V held-out scenarios.

Runs both trackers on the same CoBEVT detection results from the cv_val
scenarios and computes MOTA, MOTP, IDS, FRAG against GT annotations.

Usage:
    python scripts/compare_trackers.py
"""

import glob
import logging
import os
import pickle
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')
logger = logging.getLogger("CompareTrackers")

CMP_ROOT = "/home/atlas/TrafficSimulator_eCloud/SOTA_Predictors/CMP"


def load_detections(scenario_name, cav_id):
    """Load CMP's CoBEVT detection results for a scenario/cav pair."""
    det_file = os.path.join(
        CMP_ROOT, "preprocessed_data/opv2v",
        "corpbevtlidar_delay_1_frame_aug_c256_detection_result_for_tracking_test.pkl")
    with open(det_file, 'rb') as f:
        all_dets = pickle.load(f)

    if scenario_name not in all_dets:
        return None
    if cav_id not in all_dets[scenario_name]:
        return None
    return all_dets[scenario_name][cav_id]


def load_gt_trajectories(scenario_file):
    """Load GT trajectories from preprocessed pickle."""
    with open(scenario_file, 'rb') as f:
        data = pickle.load(f)
    return data['data'], data['timestamps']


def dets_for_frame(det_dict, frame_idx):
    """Extract detections for a specific frame from CMP's detection dict.

    Transforms detections from ego-local frame to world frame using
    the lidar pose stored in the detection dict.
    """
    if frame_idx not in det_dict:
        return np.empty((0, 8)), np.empty((0, 3))

    det = det_dict[frame_idx]
    pred_box3d = det['pred_box3d_np']  # (N, 7): x,y,z,l,w,h,ry (ego-local)
    scores = det['pred_score_np']       # (N,)
    lidar_pose = det.get('cur_lidar_pose_np')  # [x,y,z,roll,yaw,pitch]

    if len(pred_box3d) == 0:
        return np.empty((0, 8)), np.empty((0, 3))

    # Transform detection positions from ego-local to world frame
    if lidar_pose is not None:
        ego_x, ego_y, ego_z = float(lidar_pose[0]), float(lidar_pose[1]), float(lidar_pose[2])
        ego_yaw = np.radians(float(lidar_pose[4]))
        cos_y = np.cos(ego_yaw)
        sin_y = np.sin(ego_yaw)

        # Rotate and translate
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

    # Convert to AB3DMOT format: [h, w, l, x, y, z, yaw, score]
    dets = np.column_stack([
        pred_box3d[:, 5],  # h
        pred_box3d[:, 4],  # w
        pred_box3d[:, 3],  # l
        world_x,
        world_y,
        world_z,
        world_yaw,
        scores,
    ]).astype(np.float32)

    info = np.zeros((len(dets), 3), dtype=np.int64)
    info[:, 0] = frame_idx

    return dets, info


def gt_for_frame(gt_trajs, frame_idx):
    """Extract GT positions for a specific frame."""
    positions = {}
    for tid, frames in gt_trajs.items():
        if frame_idx < len(frames):
            state = frames[frame_idx]
            if state[-1] == 1:  # valid
                positions[tid] = np.array(state[:3], dtype=np.float32)  # x, y, z
    return positions


def compute_mot_metrics(track_positions, gt_positions, dist_threshold=4.0):
    """
    Compute per-frame MOT metrics.

    Returns: (tp, fp, fn, id_switches, total_distance)
    """
    if not gt_positions:
        return 0, len(track_positions), 0, 0, 0.0

    if not track_positions:
        return 0, 0, len(gt_positions), 0, 0.0

    # Build cost matrix
    gt_ids = list(gt_positions.keys())
    trk_ids = list(track_positions.keys())
    cost = np.full((len(gt_ids), len(trk_ids)), 1e6, dtype=np.float32)

    for i, gid in enumerate(gt_ids):
        for j, tid in enumerate(trk_ids):
            cost[i, j] = np.linalg.norm(gt_positions[gid] - track_positions[tid])

    # Hungarian matching
    from scipy.optimize import linear_sum_assignment
    row_ind, col_ind = linear_sum_assignment(cost)

    tp = 0
    total_dist = 0.0
    matched_gt = set()
    matched_trk = set()

    for r, c in zip(row_ind, col_ind):
        if cost[r, c] < dist_threshold:
            tp += 1
            total_dist += cost[r, c]
            matched_gt.add(gt_ids[r])
            matched_trk.add(trk_ids[c])

    fp = len(trk_ids) - len(matched_trk)
    fn = len(gt_ids) - len(matched_gt)

    return tp, fp, fn, 0, total_dist


def run_tracker_on_scenario(tracker, det_dict, gt_trajs, timestamps):
    """Run a tracker on one scenario and compute metrics."""
    all_tp, all_fp, all_fn, all_ids = 0, 0, 0, 0
    all_dist = 0.0
    prev_matches = {}

    for frame_idx in range(len(timestamps)):
        dets, info = dets_for_frame(det_dict, frame_idx)
        dets_all = {'dets': dets, 'info': info}

        tracks_list, _ = tracker.track(dets_all, frame_idx)
        tracks = tracks_list[0] if tracks_list and len(tracks_list[0]) > 0 else np.empty((0, 14))

        # Extract track positions: {track_id: [x, y, z]}
        track_positions = {}
        for trk in tracks:
            tid = int(trk[7])
            track_positions[tid] = np.array([trk[3], trk[4], trk[5]])

        # GT positions
        gt_pos = gt_for_frame(gt_trajs, frame_idx)

        # Compute metrics
        tp, fp, fn, ids, dist = compute_mot_metrics(track_positions, gt_pos)
        all_tp += tp
        all_fp += fp
        all_fn += fn
        all_dist += dist

        # ID switch detection
        if prev_matches and track_positions:
            for gid in gt_pos:
                # Find closest track
                if not track_positions:
                    continue
                dists = {tid: np.linalg.norm(gt_pos[gid] - pos)
                         for tid, pos in track_positions.items()}
                best_tid = min(dists, key=dists.get)
                if dists[best_tid] < 4.0:
                    if gid in prev_matches and prev_matches[gid] != best_tid:
                        all_ids += 1
                    prev_matches[gid] = best_tid

        prev_matches = {}
        for gid in gt_pos:
            if not track_positions:
                continue
            dists = {tid: np.linalg.norm(gt_pos[gid] - pos)
                     for tid, pos in track_positions.items()}
            if dists:
                best_tid = min(dists, key=dists.get)
                if dists[best_tid] < 4.0:
                    prev_matches[gid] = best_tid

    return {
        'tp': all_tp, 'fp': all_fp, 'fn': all_fn,
        'ids': all_ids, 'total_dist': all_dist,
    }


def main():
    from ecav.core.tracking import get_tracker

    # Load held-out val scenarios
    val_files = sorted(glob.glob(os.path.join(
        CMP_ROOT, "preprocessed_data/opv2v/gt_multiego_speedless/cv_val/*.pickle")))
    logger.info("Evaluation scenarios: %d files", len(val_files))

    # Load detection results
    det_file = os.path.join(
        CMP_ROOT, "preprocessed_data/opv2v",
        "corpbevtlidar_delay_1_frame_aug_c256_detection_result_for_tracking_test.pkl")
    with open(det_file, 'rb') as f:
        all_dets = pickle.load(f)
    logger.info("Detection results loaded: %d scenarios", len(all_dets))

    # Trackers to compare
    mamba_ckpt = 'ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth'
    trackers = {
        'AB3DMOT': lambda: get_tracker('AB3DMOT', {'min_hits': 3, 'max_age': 6}),
        'Mamba3D (BEV IoU)': lambda: get_tracker('MAMBA3DMOT', {
            'min_hits': 3, 'max_age': 6, 'match_thresh': 0.7,
            'motion_model_path': mamba_ckpt,
        }),
    }

    results = {}

    for tracker_name, tracker_factory in trackers.items():
        logger.info("\n=== %s ===", tracker_name)
        agg = {'tp': 0, 'fp': 0, 'fn': 0, 'ids': 0, 'total_dist': 0.0}

        for val_file in val_files:
            basename = os.path.basename(val_file).replace('.pickle', '')
            parts = basename.split('-')
            scenario_name = parts[0]
            cav_id = parts[1]

            # Load GT
            gt_trajs, timestamps = load_gt_trajectories(val_file)

            # Get detections for this scenario/cav
            if scenario_name not in all_dets:
                logger.warning("  %s: no detections", basename)
                continue
            if cav_id not in all_dets[scenario_name]:
                logger.warning("  %s: no detections for cav %s", basename, cav_id)
                continue

            det_dict = all_dets[scenario_name][cav_id]

            # Run tracker
            tracker = tracker_factory()

            # Load SSM weights if applicable
            if hasattr(tracker, 'load_ssm_weights') and 'ssm_weights' in trackers:
                pass  # handled in factory

            metrics = run_tracker_on_scenario(tracker, det_dict, gt_trajs, timestamps)

            for k in agg:
                agg[k] += metrics[k]

            logger.info("  %s: TP=%d FP=%d FN=%d IDS=%d",
                         basename, metrics['tp'], metrics['fp'],
                         metrics['fn'], metrics['ids'])

        # Compute aggregate metrics
        tp, fp, fn = agg['tp'], agg['fp'], agg['fn']
        mota = 1.0 - (fp + fn + agg['ids']) / max(tp + fn, 1)
        motp = agg['total_dist'] / max(tp, 1)
        precision = tp / max(tp + fp, 1)
        recall = tp / max(tp + fn, 1)

        results[tracker_name] = {
            'MOTA': mota, 'MOTP': motp,
            'Precision': precision, 'Recall': recall,
            'IDS': agg['ids'],
            'TP': tp, 'FP': fp, 'FN': fn,
        }

        logger.info("  MOTA=%.4f MOTP=%.4f Prec=%.4f Rec=%.4f IDS=%d",
                     mota, motp, precision, recall, agg['ids'])

    # Print comparison table
    print("\n" + "=" * 70)
    print("TRACKER COMPARISON (held-out OPV2V scenarios)")
    print("=" * 70)
    print(f"{'Tracker':<25s} {'MOTA':>7s} {'MOTP':>7s} {'Prec':>7s} "
          f"{'Rec':>7s} {'IDS':>5s} {'TP':>6s} {'FP':>6s} {'FN':>6s}")
    print("-" * 70)
    for name, m in results.items():
        print(f"{name:<25s} {m['MOTA']:>6.3f} {m['MOTP']:>6.2f}m "
              f"{m['Precision']:>6.3f} {m['Recall']:>6.3f} "
              f"{m['IDS']:>5d} {m['TP']:>6d} {m['FP']:>6d} {m['FN']:>6d}")


if __name__ == '__main__':
    main()
