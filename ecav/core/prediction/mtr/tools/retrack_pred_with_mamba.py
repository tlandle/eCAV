# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""Regenerate MTR training pred-trajectories with the mamba3dmot tracker.

Decodes per-frame detections from the SAVED WF fused features (cls/reg
heads only; no sensor/fusion re-run), feeds them through Mamba3DMOTWrapper
per zone, associates the resulting tracks to GT object ids (gated
Hungarian per frame + majority vote over the track lifetime, so the
supervision key is stable), normalizes yaw to motion direction for moving
states, and writes pred pickles alongside the existing GT-anchored ones.

Rationale (KB 2026-07-19/21): MTR must be trained on the SAME tracker
output the live system feeds it. GT-anchored association offline created a
train-live distribution skew (dense KF replays vs gappy detections) that
collapsed live prediction magnitude; mamba3dmot is the production tracker
(RELAY migrates its hidden state), so train = live by construction.
"""

import argparse
import json
import logging
import os
import pickle
import sys
from collections import OrderedDict, defaultdict
from typing import Dict, List

import numpy as np
import torch
from scipy.optimize import linear_sum_assignment

logger = logging.getLogger('retrack_mamba')
logging.basicConfig(level=logging.INFO, format='%(message)s')

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), *'../../../../..'.split('/')))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'ecav/worldfusion'))

from opencood.hypes_yaml import yaml_utils          # noqa: E402
from opencood.tools import train_utils              # noqa: E402
from opencood.utils import box_utils                # noqa: E402
from opencood.data_utils.post_processor.world_voxel_postprocessor import (  # noqa: E402
    WorldVoxelPostprocessor)
from ecav.core.tracking import get_tracker          # noqa: E402

STATE_DIM = 8

# Live-tuned mamba3dmot config (openscenario_3_multi_edge_mamba.yaml)
TRACKER_CFG = {
    'motion_model_path': 'ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth',
    'filter_thresh': 0.05,
    'new_track_thresh': 0.2,
    'match_thresh': 5.0,
    'max_time_lost': 60,
    'enable_time_thresh': 5,
    'max_window': 10,
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model_dir', required=True,
                   help='WF checkpoint dir (config.yaml + net_epoch*.pth)')
    p.add_argument('--export_root', required=True,
                   help='Existing export root (features + gt pickles + manifest)')
    p.add_argument('--out_dirname', default='multiv2x_pred_traj_mamba')
    p.add_argument('--eval_epoch', type=int, default=27)
    p.add_argument('--score_threshold', type=float, default=0.20)
    p.add_argument('--match_gate', type=float, default=2.0,
                   help='Max meters to associate a track to a GT object')
    p.add_argument('--min_votes', type=int, default=3,
                   help='Minimum per-frame matches before a track keys a GT id')
    p.add_argument('--motion_yaw_min_step', type=float, default=0.1,
                   help='Per-frame displacement (m) above which yaw is set '
                        'to the motion direction')
    p.add_argument('--limit_zones', type=int, default=None)
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    return p.parse_args()


def build_heads(model_dir, eval_epoch, device):
    """Load the WF model and return (cls_head, reg_head, postprocessor)."""
    hypes = yaml_utils.load_yaml(os.path.join(model_dir, 'config.yaml'))
    hypes['model']['args']['use_camera'] = False
    model = train_utils.create_model(hypes)
    model.to(device)
    _, model = train_utils.load_model(model_dir, model, eval_epoch,
                                      start_from_best=False)
    model.eval()
    post = WorldVoxelPostprocessor(hypes['postprocess'], dataset=None,
                                   train=False)
    post.params['target_args']['score_threshold'] = 0.0  # gate via CLI arg
    return model.cls_head, model.reg_head, post


def decode_frame(feat_np, cls_head, reg_head, post, device, score_threshold):
    """Saved fused feature -> (K, 7) [x,y,z,h,w,l,yaw] ego-frame boxes + scores."""
    feat = torch.from_numpy(feat_np.astype(np.float32)).to(device)
    with torch.inference_mode():
        output = {'psm': cls_head(feat), 'rm': reg_head(feat)}
        anchor_box = torch.from_numpy(post.generate_anchor_box()).to(device)
        data_dict = {'ego': {
            'anchor_box': anchor_box,
            'transformation_matrix': torch.eye(4).to(device),
            'world_anchor': [[0, 0, 0, 0, 0, 0]],
            'lidar_pose': np.array([[0, 0, 0, 0, 0, 0]]),
        }}
        pred_box, pred_score = post.post_process(data_dict, {'ego': output})
    if pred_box is None or len(pred_box) == 0:
        return np.zeros((0, 7), np.float32), np.zeros((0,), np.float32)
    boxes = box_utils.corner_to_center(
        pred_box.cpu().numpy(), order='hwl').astype(np.float32)
    scores = pred_score.cpu().numpy().astype(np.float32)
    keep = scores >= score_threshold
    return boxes[keep], scores[keep]


def normalize_yaw_to_motion(seq: List[List[float]], min_step: float):
    """In-place: yaw := motion direction wherever per-step displacement is
    above min_step (kills the detection 180-degree flip ambiguity)."""
    prev_idx = None
    for i, s in enumerate(seq):
        if s[7] != 1:
            continue
        if prev_idx is not None:
            dx = s[0] - seq[prev_idx][0]
            dy = s[1] - seq[prev_idx][1]
            if (dx * dx + dy * dy) ** 0.5 > min_step:
                s[6] = float(np.arctan2(dy, dx))
        prev_idx = i


def main():
    args = parse_args()
    device = torch.device(args.device)
    cls_head, reg_head, post = build_heads(args.model_dir, args.eval_epoch,
                                           device)

    with open(os.path.join(args.export_root, 'manifest.json')) as f:
        records = json.load(f)['records']
    if args.limit_zones:
        records = records[:args.limit_zones]

    feat_root = os.path.join(args.export_root, 'wf_fused_features')
    gt_root = os.path.join(args.export_root, 'multiv2x_gt_traj')
    out_root = os.path.join(args.export_root, args.out_dirname)
    for split in ('train', 'test'):
        os.makedirs(os.path.join(out_root, split), exist_ok=True)

    for rec in records:
        scen, rsu, split = rec['scenario'], rec['rsu'], rec['split']
        gt_pkl = os.path.join(gt_root, split, f'{scen}-{rsu}-traj.pickle')
        with open(gt_pkl, 'rb') as f:
            gt_obj = pickle.load(f)
        timestamps = gt_obj['timestamps']
        gt_data = gt_obj['data']

        tracker = get_tracker('mamba3dmot', dict(TRACKER_CFG))
        # per-track: {tid: {ts: state}}, votes: {tid: {gt_id: n}}
        track_states: Dict[int, 'OrderedDict[str, List[float]]'] = {}
        votes: Dict[int, Dict[int, int]] = defaultdict(lambda: defaultdict(int))

        for f_idx, ts in enumerate(timestamps):
            feat_path = os.path.join(
                feat_root, split, f'wf_fused_{scen}_{rsu}_{ts}.npy')
            if not os.path.exists(feat_path):
                continue
            boxes, scores = decode_frame(np.load(feat_path), cls_head,
                                         reg_head, post, device,
                                         args.score_threshold)
            # wrapper input: (N, 8) [h,w,l,x,y,z,yaw,score]
            dets = np.zeros((len(boxes), 8), np.float32)
            if len(boxes):
                dets[:, 0] = boxes[:, 3]  # h
                dets[:, 1] = boxes[:, 4]  # w
                dets[:, 2] = boxes[:, 5]  # l
                dets[:, 3] = boxes[:, 0]  # x
                dets[:, 4] = boxes[:, 1]  # y
                dets[:, 5] = boxes[:, 2]  # z
                dets[:, 6] = boxes[:, 6]  # yaw
                dets[:, 7] = scores
            info = np.full((len(boxes), 3), -1, np.float32)
            info[:, 0] = dets[:, 7]
            tracks_list, _ = tracker.track(
                {'dets': dets, 'info': info}, f_idx)
            tracks = tracks_list[0] if len(tracks_list) else np.zeros((0, 14))

            # GT boxes this frame for association
            gt_pos, gt_ids = [], []
            for oid, seq in gt_data.items():
                if f_idx < len(seq) and seq[f_idx][7] == 1:
                    gt_pos.append(seq[f_idx][:2])
                    gt_ids.append(oid)
            gt_pos = np.asarray(gt_pos, np.float32).reshape(-1, 2)

            for trk in np.atleast_2d(tracks):
                if trk.shape[0] == 0:
                    continue
                tid = int(trk[7])
                # track row: [h,w,l,x,y,z,yaw,...] -> state [x,y,z,l,w,h,yaw,1]
                state = [float(trk[3]), float(trk[4]), float(trk[5]),
                         float(trk[2]), float(trk[1]), float(trk[0]),
                         float(trk[6]), 1]
                track_states.setdefault(tid, OrderedDict())[ts] = state

            # gated Hungarian: track positions vs GT positions
            if len(gt_ids) and isinstance(tracks, np.ndarray) and tracks.size:
                t2 = np.atleast_2d(tracks)
                tp = t2[:, 3:5].astype(np.float32)
                cost = np.linalg.norm(tp[:, None, :] - gt_pos[None], axis=2)
                gated = np.where(cost <= args.match_gate, cost, 1e6)
                ri, ci = linear_sum_assignment(gated)
                for i, j in zip(ri, ci):
                    if gated[i, j] >= 1e6:
                        continue
                    votes[int(t2[i, 7])][gt_ids[j]] += 1

        # Track -> GT id by majority vote; on collision keep the track with
        # more votes for that GT id.
        best_for_gt: Dict[int, tuple] = {}
        for tid, v in votes.items():
            gt_id, n = max(v.items(), key=lambda kv: kv[1])
            if n < args.min_votes:
                continue
            if gt_id not in best_for_gt or n > best_for_gt[gt_id][1]:
                best_for_gt[gt_id] = (tid, n)

        invalid = [0.0] * (STATE_DIM - 1) + [0]
        pred_out = OrderedDict()
        for gt_id, (tid, _n) in best_for_gt.items():
            seq = [list(track_states[tid].get(t, invalid))
                   for t in timestamps]
            # Prune misattributed segments: tracker id reuse can stitch two
            # physical vehicles under one tid; the majority vote keys one GT
            # id, so drop states far from that GT when it is visible, and
            # kill residual implausible jumps (>6 m per 0.2 s frame) where
            # GT visibility can't arbitrate.
            gt_seq = gt_data.get(gt_id)
            for f_idx, s in enumerate(seq):
                if s[7] != 1:
                    continue
                if gt_seq is not None and f_idx < len(gt_seq) \
                        and gt_seq[f_idx][7] == 1:
                    d = np.hypot(s[0] - gt_seq[f_idx][0],
                                 s[1] - gt_seq[f_idx][1])
                    if d > 2 * args.match_gate:
                        seq[f_idx] = list(invalid)
            prev = None
            for f_idx, s in enumerate(seq):
                if s[7] != 1:
                    prev = None
                    continue
                if prev is not None and \
                        np.hypot(s[0] - prev[0], s[1] - prev[1]) > 6.0:
                    seq[f_idx] = list(invalid)
                    prev = None
                    continue
                prev = s
            normalize_yaw_to_motion(seq, args.motion_yaw_min_step)
            pred_out[gt_id] = seq

        out_pkl = os.path.join(out_root, split, f'{scen}-{rsu}-traj.pickle')
        with open(out_pkl, 'wb') as f:
            pickle.dump({'data': dict(pred_out),
                         'timestamps': timestamps}, f)
        n_frames_tracked = sum(
            sum(1 for s in seq if s[7] == 1) for seq in pred_out.values())
        logger.info('%s/%s [%s]: %d tracks -> %d keyed GT objs, '
                    '%d tracked states over %d ts',
                    scen, rsu, split, len(track_states), len(pred_out),
                    n_frames_tracked, len(timestamps))

    logger.info('Done. Pred pickles at %s', out_root)


if __name__ == '__main__':
    main()
