#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Export WorldFusion fused features + AB3DMOT tracked trajectories
for Multi-V2X, in the format MTR (CMP-style) expects.

For each (scenario, rsu) zone in dataset_info.json, iterates the GT
trajectory pickle's timestamps. Per timestamp:
  1. Builds the WF intermediate-fusion sample (RSU + connected CAVs)
  2. Runs a single WF forward pass
  3. Dumps fused_feature .npy (float16)
  4. Decodes detections, transforms to world frame, feeds AB3DMOT
After each zone, flushes AB3DMOT tracks into the pred_traj pickle
(matching the GT pickle schema: [x, y, z, l, w, h, yaw, valid]).

Also emits rsu_poses.json (one entry per zone) and a manifest with
feature paths.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import pickle
import sys
from collections import OrderedDict
from typing import Dict, List, Tuple

import numpy as np
import torch
from scipy.optimize import linear_sum_assignment
from torch.utils.data import DataLoader
from tqdm import tqdm

# Make the WF opencood package importable
_THIS = os.path.abspath(__file__)
_REPO_ROOT = '/'.join(_THIS.split('/')[:-5])  # .../ecloudsim_distributed_sandbox
_WF_ROOT = os.path.join(_REPO_ROOT, 'ecav', 'worldfusion')
if _WF_ROOT not in sys.path:
    sys.path.insert(0, _WF_ROOT)

import opencood.hypes_yaml.yaml_utils as yaml_utils  # noqa: E402
from opencood.data_utils.datasets import build_dataset  # noqa: E402
from opencood.tools import train_utils  # noqa: E402
from opencood.utils import box_utils  # noqa: E402
from opencood.hypes_yaml.yaml_utils import load_yaml  # noqa: E402
from opencood.data_utils.post_processor.world_voxel_postprocessor import \
    WorldVoxelPostprocessor  # noqa: E402

from AB3DMOT_libs.model import AB3DMOT  # noqa: E402
from easydict import EasyDict as edict  # noqa: E402


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(name)s: %(message)s',
)
logger = logging.getLogger('export_wf_for_mtr')


# --- Constants ---------------------------------------------------------------

# State vector schema (matches existing multiv2x_gt_traj pickle):
#   [x, y, z, l, w, h, yaw, valid]
STATE_DIM = 8


def hwl_box_to_state(box):
    """[x, y, z, h, w, l, yaw] (postprocessor/object_bbx order 'hwl')
    -> pickle state [x, y, z, l, w, h, yaw, valid=1]."""
    x, y, z, h, w, l, yaw = (float(v) for v in box[:7])
    return [x, y, z, l, w, h, yaw, 1]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model_dir', required=True,
                   help='WF checkpoint dir (contains config.yaml + net_epoch*.pth)')
    p.add_argument('--data_root', required=True,
                   help='Multi-V2X dataset root (contains Town*__... dirs)')
    p.add_argument('--manifest', required=True,
                   help='dataset_info.json from multiv2x_mtr/')
    p.add_argument('--gt_traj_root', required=True,
                   help='Existing multiv2x_gt_traj/ root (defines per-zone timestamps)')
    p.add_argument('--output_dir', required=True,
                   help='Where to write features + pred_traj pickles')
    p.add_argument('--eval_epoch', type=int, default=27,
                   help='Which WF checkpoint to load')
    p.add_argument('--score_threshold', type=float, default=0.20,
                   help='Override postprocess score_threshold for detections')
    p.add_argument('--match_gate', type=float, default=2.0,
                   help='Max Euclidean (m) to associate a detection to a GT object')
    p.add_argument('--limit_zones', type=int, default=None,
                   help='Process only this many zones (smoke test)')
    p.add_argument('--limit_frames', type=int, default=None,
                   help='Process only this many frames per zone (smoke test)')
    p.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    return p.parse_args()


# --- Dataset sample override -------------------------------------------------

def build_zone_samples(dataset, records: List[Dict], gt_traj_root: str
                       ) -> List[Tuple[Dict, str, str]]:
    """For each (scenario, rsu, split) record, expand its GT pickle's
    timestamps into per-frame samples that match the WF dataset's
    expected schema.

    Returns: list of (sample_dict, scenario_name, rsu_name, split, ts)
    where sample_dict is {'scenario_id': int, 'rsu_name': str, 'timestamp': str}.
    Drops timestamps where the RSU yaml is missing.
    """
    # Build scenario_name -> scenario_id from the WF dataset
    name_to_id = {}
    for sid, sc in dataset.scenario_database.items():
        name_to_id[sc['name']] = sid

    out = []
    for r in records:
        scen, rsu, split = r['scenario'], r['rsu'], r['split']
        if scen not in name_to_id:
            logger.warning(f'scenario {scen} not in WF dataset; skipping')
            continue
        sid = name_to_id[scen]
        sc = dataset.scenario_database[sid]
        if rsu not in sc['agents']:
            logger.warning(f'{scen}/{rsu} not in scenario_database agents; skipping')
            continue

        gt_pkl = os.path.join(gt_traj_root, split, f'{scen}-{rsu}-traj.pickle')
        if not os.path.exists(gt_pkl):
            logger.warning(f'missing GT pickle {gt_pkl}; skipping')
            continue
        with open(gt_pkl, 'rb') as f:
            gt_obj = pickle.load(f)
        timestamps = gt_obj['timestamps']

        rsu_dir = os.path.join(sc['path'], rsu)
        for ts in timestamps:
            if not os.path.exists(os.path.join(rsu_dir, f'{ts}.yaml')):
                continue
            out.append(({
                'scenario_id': sid,
                'rsu_name': rsu,
                'timestamp': ts,
            }, scen, rsu, split, ts))
    return out


# --- AB3DMOT plumbing --------------------------------------------------------

_AB3DMOT_DEFAULT_CFG = edict({
    'vis': False,
    'save_path': None,
    'use_3d_iou': False,
    'thres': 2.0,
    'output_dir': None,
    'min_hits': 3,
    'max_age': 6,
    'ego_com': None,
    'affi_pro': False,
    'dataset': 'KITTI',
    'det_name': 'deprecated',
    'anchoring': False,
    'dup_x_max': 8.0,
    'dup_y_max': 2.0,
    'dup_size_ratio': 2.5,
    'cull_consec_ticks': 0,
})


def make_tracker():
    return AB3DMOT(_AB3DMOT_DEFAULT_CFG, 'Car')


# --- Per-frame inference ------------------------------------------------------

def run_one_frame(model, post_processor, item, batch_data, device):
    """Run WF on one frame; return ego-frame pred detections + GT boxes.

    Everything is in the RSU-ego frame. Since the RSU is static, this frame
    is fixed across the whole zone, so trajectories built in it are
    world-consistent without any world transform. This is the same frame
    the dataset's GT (`object_bbx_center`) lives in, so pred and GT are
    directly comparable with no x_to_world reconciliation.

    Returns:
        fused_np:    [1, C, H, W] fused BEV feature
        pred_boxes:  (P, 7) [x, y, z, h, w, l, yaw] pred detections (ego)
        pred_scores: (P,)
        gt_boxes:    (G, 7) [x, y, z, h, w, l, yaw] GT boxes (ego)
        gt_ids:      (G,) CARLA object ids aligned to gt_boxes
    """
    batch_data = train_utils.to_device(batch_data, device)
    cav_content = batch_data['ego']
    output = model(cav_content)
    fused_feature = output['fused_feature']  # [1, C, H, W]

    # --- GT boxes (ego frame) with persistent CARLA ids ---
    ego = item['ego']
    mask = np.asarray(ego['object_bbx_mask']) == 1
    gt_boxes = np.asarray(ego['object_bbx_center'])[mask].astype(np.float32)
    gt_ids = np.asarray(list(ego['object_ids']), dtype=np.int64)

    # --- Pred detections (ego frame), WorldVoxelPostprocessor ---
    anchor_box_np = post_processor.generate_anchor_box()
    data_dict_for_post = {
        'ego': {
            'anchor_box': torch.from_numpy(anchor_box_np).to(device),
            'transformation_matrix': torch.eye(4).to(device),
            'world_anchor': [[0, 0, 0, 0, 0, 0]],
            'lidar_pose': np.array([[0, 0, 0, 0, 0, 0]]),
        }
    }
    pred_box_tensor, pred_score = post_processor.post_process(
        data_dict_for_post, {'ego': output})

    if pred_box_tensor is None or len(pred_box_tensor) == 0:
        pred_boxes = np.zeros((0, 7), dtype=np.float32)
        pred_scores = np.zeros((0,), dtype=np.float32)
    else:
        pred_boxes = box_utils.corner_to_center(
            pred_box_tensor.cpu().detach().numpy(), order='hwl'
        ).astype(np.float32)
        pred_scores = pred_score.cpu().detach().numpy().astype(np.float32)

    return (fused_feature.detach().cpu().numpy(),
            pred_boxes, pred_scores, gt_boxes, gt_ids)


# --- Main --------------------------------------------------------------------

def main():
    args = parse_args()

    # Resolve output structure
    feat_root = os.path.join(args.output_dir, 'wf_fused_features')
    pred_root = os.path.join(args.output_dir, 'multiv2x_pred_traj')
    gt_root = os.path.join(args.output_dir, 'multiv2x_gt_traj')
    for split in ('train', 'test'):
        os.makedirs(os.path.join(feat_root, split), exist_ok=True)
        os.makedirs(os.path.join(pred_root, split), exist_ok=True)
        os.makedirs(os.path.join(gt_root, split), exist_ok=True)

    # Load WF config and override data root
    config_path = os.path.join(args.model_dir, 'config.yaml')
    hypes = yaml_utils.load_yaml(config_path)
    hypes['root_dir'] = args.data_root
    hypes['validate_dir'] = args.data_root
    # Force LiDAR-only mode in case config drifted
    hypes['model']['args']['use_camera'] = False
    # Override detection score threshold passed to postprocessor
    hypes['postprocess']['target_args']['score_threshold'] = args.score_threshold

    logger.info('Building WF dataset...')
    # train=False is REQUIRED: train mode applies per-frame random world
    # flip/rotation/scaling/translation (the translaug augmentors) jointly to
    # lidar + GT boxes. Each frame then sits in an independently randomized
    # frame: self-consistent within the frame (detections still match GT),
    # but positions jump ~30-60 m across frames, which scrambles every
    # exported trajectory. Found 2026-07-06 after MTR plateaued at the
    # static-baseline ADE (~31 m @ 5 s).
    dataset = build_dataset(hypes, visualize=False, train=False)

    # Detection decode must match the live edge manager exactly:
    # WorldVoxelPostprocessor (world-axis-aligned RSU-local frame), not the
    # opv2v VoxelPostprocessor the dataset builds (which ego-recenters).
    post_processor = WorldVoxelPostprocessor(
        hypes['postprocess'], dataset=None, train=False)

    # Load manifest and build per-frame sample list keyed off GT pickles
    with open(args.manifest) as f:
        info = json.load(f)
    records = info['records']
    if args.limit_zones:
        records = records[:args.limit_zones]

    samples = build_zone_samples(dataset, records, args.gt_traj_root)
    logger.info(f'Total frames to process: {len(samples)} '
                f'across {len(set((s[1], s[2]) for s in samples))} zones')

    # Group samples by (scenario, rsu, split) for per-zone AB3DMOT
    zone_samples: "OrderedDict[Tuple[str, str, str], List[Tuple[Dict, str]]]" = \
        OrderedDict()
    for sample_dict, scen, rsu, split, ts in samples:
        zone_samples.setdefault((scen, rsu, split), []).append((sample_dict, ts))

    if args.limit_frames:
        for k in zone_samples:
            zone_samples[k] = zone_samples[k][:args.limit_frames]

    # Build model
    logger.info('Building WF model...')
    model = train_utils.create_model(hypes)
    device = torch.device(args.device)
    if device.type == 'cuda':
        model.cuda()
    _, model = train_utils.load_model(
        args.model_dir, model, args.eval_epoch, start_from_best=False)
    model.zero_grad()
    model.eval()

    # In LiDAR-only mode the model __init__ skips the camera branch and
    # never registers self.sensor.nx, but forward() references it
    # (line 802 in point_pillar_worldfusion.py). Derive nx from voxel_size
    # + lidar_range and inject as a buffer so the forward pass succeeds.
    if not hasattr(model.sensor, 'nx') or model.sensor.nx is None:
        pc_cfg = hypes['model']['args']['pc_params']
        vsz = pc_cfg['voxel_size']
        lrange = pc_cfg['lidar_range']
        nx = torch.tensor([
            int(round((lrange[3] - lrange[0]) / vsz[0])),
            int(round((lrange[4] - lrange[1]) / vsz[1])),
            int(round((lrange[5] - lrange[2]) / vsz[2])),
        ], dtype=torch.long, device=device)
        model.sensor.register_buffer('nx', nx, persistent=False)
        logger.info(f'Registered missing nx buffer on sensor: {nx.tolist()}')

    rsu_poses_out: Dict[str, Dict[str, List[float]]] = {}
    manifest_out = {
        'feature_dir': feat_root,
        'pred_traj_dir': pred_root,
        'gt_traj_dir': gt_root,
        'records': [],
    }

    # Iterate zones
    with torch.inference_mode():
        for (scen, rsu, split), zsamples in zone_samples.items():
            logger.info(f'--- {scen}/{rsu} [{split}] : {len(zsamples)} frames')

            # Pred (observed) and GT both keyed by CARLA object id, ego frame.
            # Detections are associated to GT objects per frame by gated
            # Hungarian matching (offline GT-anchored association, standard for
            # building tracking/prediction training data). The detection
            # positions stay the model's noisy output; only the data
            # association uses GT. No tracker, no id reconciliation.
            tracks: Dict[int, Dict[str, List[float]]] = {}
            gt_tracks: Dict[int, Dict[str, List[float]]] = {}
            timestamps_seen: List[str] = []

            # Cache RSU pose once (reference only; all data is ego-frame).
            sid = zsamples[0][0]['scenario_id']
            sc_path = dataset.scenario_database[sid]['path']
            first_ts = zsamples[0][1]
            rsu_yaml = load_yaml(os.path.join(sc_path, rsu, f'{first_ts}.yaml'))
            rsu_pose = list(map(float, rsu_yaml['lidar_pose']))
            rsu_poses_out.setdefault(scen, {})[rsu] = rsu_pose

            for frame_idx, (sample_dict, ts) in enumerate(tqdm(zsamples,
                                                               desc=f'{scen[:18]}/{rsu}')):
                # Inject sample into dataset and call directly (skip DataLoader)
                dataset.samples = [sample_dict]
                item = dataset[0]
                batch_data = dataset.collate_batch_test([item])

                fused_np, pred_boxes, scores, gt_boxes, gt_ids = run_one_frame(
                    model, post_processor, item, batch_data, device)

                timestamps_seen.append(ts)

                # Apply score threshold (postprocessor gates, but honor the
                # CLI override symmetrically with the live path).
                if pred_boxes.size and args.score_threshold > 0:
                    keep = scores >= args.score_threshold
                    pred_boxes = pred_boxes[keep]
                    scores = scores[keep]

                # Dump fused feature as plain .npy float16
                # (transfer-level compression is plain tar, not gzip/zlib)
                feat_path = os.path.join(
                    feat_root, split,
                    f'wf_fused_{scen}_{rsu}_{ts}.npy')
                np.save(feat_path, fused_np.astype(np.float16))

                # --- GT: record each object by CARLA id (ego frame) ---
                for box, oid in zip(gt_boxes, gt_ids):
                    gt_tracks.setdefault(int(oid), OrderedDict())[ts] = \
                        hwl_box_to_state(box)

                # --- Pred: gated Hungarian match of detections to GT ids ---
                if pred_boxes.size and gt_boxes.size:
                    pxy = pred_boxes[:, :2]
                    gxy = gt_boxes[:, :2]
                    cost = np.linalg.norm(
                        pxy[:, None, :] - gxy[None, :, :], axis=2)
                    cost_gated = np.where(cost <= args.match_gate, cost, 1e6)
                    ri, ci = linear_sum_assignment(cost_gated)
                    for i, j in zip(ri, ci):
                        if cost_gated[i, j] >= 1e6:
                            continue
                        oid = int(gt_ids[j])
                        tracks.setdefault(oid, OrderedDict())[ts] = \
                            hwl_box_to_state(pred_boxes[i])

            # Flush per-zone pickles (pred + gt), padding gaps with valid=0.
            invalid_state = [0.0] * (STATE_DIM - 1) + [0]

            def _align(track_dict):
                out = OrderedDict()
                for oid, ts_to_state in track_dict.items():
                    out[oid] = [ts_to_state.get(t, invalid_state)
                                for t in timestamps_seen]
                return out

            pred_out = _align(tracks)
            gt_out = _align(gt_tracks)

            pred_pkl = os.path.join(pred_root, split,
                                    f'{scen}-{rsu}-traj.pickle')
            gt_pkl = os.path.join(gt_root, split,
                                  f'{scen}-{rsu}-traj.pickle')
            with open(pred_pkl, 'wb') as f:
                pickle.dump({'data': dict(pred_out),
                             'timestamps': timestamps_seen}, f)
            with open(gt_pkl, 'wb') as f:
                pickle.dump({'data': dict(gt_out),
                             'timestamps': timestamps_seen}, f)
            logger.info(f'  wrote pred ({len(pred_out)} tracks) + '
                        f'gt ({len(gt_out)} objs), {len(timestamps_seen)} ts')

            manifest_out['records'].append({
                'scenario': scen,
                'rsu': rsu,
                'split': split,
                'n_timestamps': len(timestamps_seen),
                'n_pred_tracks': len(pred_out),
                'n_gt_objects': len(gt_out),
                'pred_traj_file': pred_pkl,
                'gt_traj_file': gt_pkl,
                'feature_dir': os.path.join(feat_root, split),
                'feature_name_fmt':
                    f'wf_fused_{scen}_{rsu}_{{ts}}.npy',
                'rsu_pose': rsu_pose,
            })

    with open(os.path.join(args.output_dir, 'rsu_poses.json'), 'w') as f:
        json.dump(rsu_poses_out, f, indent=2)
    with open(os.path.join(args.output_dir, 'manifest.json'), 'w') as f:
        json.dump(manifest_out, f, indent=2)
    logger.info('Done.')


if __name__ == '__main__':
    main()
