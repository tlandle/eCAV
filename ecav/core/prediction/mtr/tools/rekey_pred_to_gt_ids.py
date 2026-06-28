#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Re-key AB3DMOT pred pickles by GT object IDs.

Per frame, Hungarian-match each tracked detection to the closest GT
object (gating distance configurable). Output a new pred pickle whose
top-level keys are GT object IDs, so the MTR loader can directly fetch
matching future states from the GT pickle.

Unmatched tracker outputs become false-positive tracks keyed by
negative integers (-1, -2, ...) — the loader treats them as objects
without future supervision; the model sees their past but loss is
masked.

GT objects with no matched detection at frame t remain absent from
the pred pickle at frame t (valid=0). The training loader still sees
them via the GT pickle when needed.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
from collections import OrderedDict

import numpy as np
from scipy.optimize import linear_sum_assignment


def rekey_one_zone(gt_obj, pred_obj, gate=5.0, fp_id_start=-1):
    gt_data = gt_obj['data']
    gt_ts = gt_obj['timestamps']
    pred_data = pred_obj['data']
    pred_ts = pred_obj['timestamps']

    # Align timestamps: use intersection in pred order
    common_ts = [t for t in pred_ts if t in set(gt_ts)]
    ts_to_pred_idx = {t: pred_ts.index(t) for t in common_ts}
    ts_to_gt_idx = {t: gt_ts.index(t) for t in common_ts}

    # For each frame, build (gt_oid, x, y) and (pred_oid, x, y) lists
    # then Hungarian-match.
    n_t = len(common_ts)
    out_data = {}  # gt_oid -> list of length n_t of state vectors
    fp_data = {}   # negative id -> list

    fp_next_id = fp_id_start

    # Pre-extract per-frame valid lists
    gt_valid_frame = []
    for t in common_ts:
        idx = ts_to_gt_idx[t]
        per_gt = []
        for oid, seq in gt_data.items():
            s = seq[idx]
            if s[7] == 1:
                per_gt.append((oid, s[0], s[1]))
        gt_valid_frame.append(per_gt)

    pred_valid_frame = []
    for t in common_ts:
        idx = ts_to_pred_idx[t]
        per_pr = []
        for oid, seq in pred_data.items():
            s = seq[idx]
            if s[7] == 1:
                per_pr.append((oid, s, s[0], s[1]))
        pred_valid_frame.append(per_pr)

    # Persist trackers across frames to avoid splitting one tracker
    # between two different GT objects.
    tracker_to_gt: dict = {}

    for f, (gt_list, pr_list) in enumerate(zip(gt_valid_frame, pred_valid_frame)):
        if not pr_list:
            continue
        if not gt_list:
            # All predictions are FPs at this frame
            for (tr_oid, state, _, _) in pr_list:
                key = tracker_to_gt.get(tr_oid)
                if key is None:
                    key = fp_next_id
                    fp_next_id -= 1
                    tracker_to_gt[tr_oid] = key
                if key < 0:
                    fp_data.setdefault(key, [None] * n_t)
                    fp_data[key][f] = list(state)
            continue

        # Cost matrix: Euclidean distance pred×gt, infinity beyond gate
        n_gt, n_pr = len(gt_list), len(pr_list)
        cost = np.full((n_pr, n_gt), 1e6, dtype=np.float32)
        for i, (_, _, px, py) in enumerate(pr_list):
            for j, (_, gx, gy) in enumerate(gt_list):
                d = float(np.hypot(px - gx, py - gy))
                if d <= gate:
                    cost[i, j] = d

        # Bias: keep persistent assignment if pred had been matched before
        for i, (tr_oid, _, _, _) in enumerate(pr_list):
            prev = tracker_to_gt.get(tr_oid)
            if prev is not None and prev >= 0:
                for j, (gt_oid, _, _) in enumerate(gt_list):
                    if gt_oid == prev and cost[i, j] < 1e6:
                        cost[i, j] -= 1.0  # slight preference

        row_ind, col_ind = linear_sum_assignment(cost)
        matched_pred = set()
        for i, j in zip(row_ind, col_ind):
            if cost[i, j] >= 1e6:
                continue
            tr_oid = pr_list[i][0]
            gt_oid = gt_list[j][0]
            state = pr_list[i][1]
            out_data.setdefault(gt_oid, [None] * n_t)
            out_data[gt_oid][f] = list(state)
            tracker_to_gt[tr_oid] = gt_oid
            matched_pred.add(i)

        # Unmatched predictions: FP
        for i, (tr_oid, state, _, _) in enumerate(pr_list):
            if i in matched_pred:
                continue
            key = tracker_to_gt.get(tr_oid)
            if key is None or key >= 0:
                key = fp_next_id
                fp_next_id -= 1
                tracker_to_gt[tr_oid] = key
            fp_data.setdefault(key, [None] * n_t)
            fp_data[key][f] = list(state)

    # Fill gaps with invalid placeholder
    invalid = [0.0] * 7 + [0]
    for d in (out_data, fp_data):
        for k in d:
            d[k] = [s if s is not None else invalid for s in d[k]]

    # Merge with FPs negative-keyed
    merged = OrderedDict()
    for k in sorted(out_data.keys()):
        merged[k] = out_data[k]
    for k in sorted(fp_data.keys(), reverse=True):
        merged[k] = fp_data[k]

    return {'data': dict(merged), 'timestamps': common_ts}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--gt_traj_root', required=True)
    p.add_argument('--pred_traj_root', required=True)
    p.add_argument('--manifest', required=True,
                   help='dataset_info.json describing zones + splits')
    p.add_argument('--out_pred_root', required=True,
                   help='Output pred_traj root (split into train/test)')
    p.add_argument('--gate', type=float, default=5.0,
                   help='Max Euclidean (m) for valid pred-GT match')
    return p.parse_args()


def main():
    args = parse_args()
    with open(args.manifest) as f:
        info = json.load(f)
    for split in ('train', 'test'):
        os.makedirs(os.path.join(args.out_pred_root, split), exist_ok=True)

    for r in info['records']:
        scen, rsu, split = r['scenario'], r['rsu'], r['split']
        gt_p = os.path.join(args.gt_traj_root, split,
                            f'{scen}-{rsu}-traj.pickle')
        pr_p = os.path.join(args.pred_traj_root, split,
                            f'{scen}-{rsu}-traj.pickle')
        out_p = os.path.join(args.out_pred_root, split,
                             f'{scen}-{rsu}-traj.pickle')
        if not (os.path.exists(gt_p) and os.path.exists(pr_p)):
            print(f'missing: {scen}/{rsu} [{split}]')
            continue
        with open(gt_p, 'rb') as f:
            gt = pickle.load(f)
        with open(pr_p, 'rb') as f:
            pr = pickle.load(f)
        rekeyed = rekey_one_zone(gt, pr, gate=args.gate)
        with open(out_p, 'wb') as f:
            pickle.dump(rekeyed, f)
        n_gt_keys = sum(1 for k in rekeyed['data'] if k >= 0)
        n_fp_keys = sum(1 for k in rekeyed['data'] if k < 0)
        print(f'{scen[:25]:25}/{rsu:>7} [{split}] '
              f'gt-keyed={n_gt_keys:4} fp={n_fp_keys:3} '
              f'ts={len(rekeyed["timestamps"])}')


if __name__ == '__main__':
    main()
