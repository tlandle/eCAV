#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""K-means on GT trajectory endpoints (in current-heading frame) to
produce multiv2x_cluster_64_center_dict.pkl for MTR.

Matches the OPV2V cluster_64 format used by CMP:
  {'TYPE_VEHICLE': ndarray (K, 2) of (dx, dy) endpoint clusters}

For each (object, t) where a future state at t + FUTURE exists and
is valid, compute endpoint displacement in the object's current
heading frame, then cluster across all such samples.
"""

from __future__ import annotations

import argparse
import json
import os
import pickle
from collections import defaultdict

import numpy as np
from sklearn.cluster import KMeans


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--gt_traj_root', required=True,
                   help='multiv2x_gt_traj/ root')
    p.add_argument('--manifest', required=True,
                   help='dataset_info.json (defines train split records)')
    p.add_argument('--future_frames', type=int, default=25,
                   help='Future horizon (5Hz × 25 = 5s)')
    p.add_argument('--past_frames', type=int, default=10,
                   help='Past history (used to estimate current heading)')
    p.add_argument('--n_clusters', type=int, default=64)
    p.add_argument('--output', required=True,
                   help='Output pkl path')
    p.add_argument('--use_yaw_from_state', action='store_true',
                   help='Use stored yaw (state[6]) instead of finite-diff heading')
    return p.parse_args()


def main():
    args = parse_args()

    with open(args.manifest) as f:
        info = json.load(f)
    train_records = [r for r in info['records'] if r['split'] == 'train']

    endpoints = []  # list of (dx, dy) in current-heading frame
    for r in train_records:
        scen, rsu = r['scenario'], r['rsu']
        pkl_path = os.path.join(args.gt_traj_root, 'train',
                                f'{scen}-{rsu}-traj.pickle')
        if not os.path.exists(pkl_path):
            print(f'missing {pkl_path}, skipping')
            continue
        with open(pkl_path, 'rb') as f:
            obj = pickle.load(f)
        data, timestamps = obj['data'], obj['timestamps']
        n_t = len(timestamps)

        for oid, seq in data.items():
            # seq: list of length n_t, each [x,y,z,l,w,h,yaw,valid]
            for t in range(args.past_frames, n_t - args.future_frames):
                cur = seq[t]
                fut = seq[t + args.future_frames]
                if cur[7] != 1 or fut[7] != 1:
                    continue
                cx, cy = cur[0], cur[1]
                fx, fy = fut[0], fut[1]

                if args.use_yaw_from_state:
                    # pickle yaw is already radians (opencood boxes)
                    yaw = cur[6]
                else:
                    prev = seq[t - 1] if t >= 1 else cur
                    dx_h = cur[0] - prev[0]
                    dy_h = cur[1] - prev[1]
                    if abs(dx_h) < 1e-6 and abs(dy_h) < 1e-6:
                        yaw = cur[6]
                    else:
                        yaw = np.arctan2(dy_h, dx_h)

                # Rotate (fx-cx, fy-cy) into current-heading frame
                dx_w, dy_w = fx - cx, fy - cy
                c, s = np.cos(-yaw), np.sin(-yaw)
                dx_h = c * dx_w - s * dy_w
                dy_h = s * dx_w + c * dy_w
                endpoints.append((dx_h, dy_h))

    print(f'Collected {len(endpoints)} endpoint samples')
    endpoints = np.array(endpoints, dtype=np.float32)

    km = KMeans(n_clusters=args.n_clusters, n_init=10, random_state=42)
    km.fit(endpoints)
    centers = km.cluster_centers_.astype(np.float32)
    print(f'Cluster center stats: min={centers.min(axis=0)}, max={centers.max(axis=0)}')

    out_dict = {'TYPE_VEHICLE': centers}
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    with open(args.output, 'wb') as f:
        pickle.dump(out_dict, f)
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
