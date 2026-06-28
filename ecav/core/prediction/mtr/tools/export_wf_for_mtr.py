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
from opencood.utils.transformation_utils import x_to_world  # noqa: E402
from opencood.hypes_yaml.yaml_utils import load_yaml  # noqa: E402

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
                   help='Override postprocess score_threshold for detection input to tracker')
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


# --- Geometry helpers --------------------------------------------------------

def transform_box_centers_to_world(centers_local: np.ndarray,
                                    rsu_pose: List[float]) -> np.ndarray:
    """Transform (N, 7) [x,y,z,h,w,l,yaw] from RSU-local to world.

    The post_processor returns boxes in the ego (RSU) lidar frame at test
    time (transformation_matrix = identity). To match the GT pickle's
    world-frame coordinates, apply the RSU's lidar-to-world transform.

    Yaw is rotated by the RSU's yaw (degrees, CARLA convention).
    """
    if centers_local.size == 0:
        return centers_local
    T_world_rsu = x_to_world(rsu_pose)  # 4x4
    xyz_local = centers_local[:, :3]
    xyz_h = np.concatenate(
        [xyz_local, np.ones((xyz_local.shape[0], 1), dtype=xyz_local.dtype)],
        axis=1,
    )
    xyz_world = (T_world_rsu @ xyz_h.T).T[:, :3]
    out = centers_local.copy()
    out[:, :3] = xyz_world
    rsu_yaw_rad = np.deg2rad(rsu_pose[4])
    out[:, 6] = centers_local[:, 6] + rsu_yaw_rad
    return out


# --- Per-frame inference ------------------------------------------------------

def run_one_frame(model, dataset, batch_data, device, score_threshold
                  ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (fused_feature_np, centers_local, scores_np).

    centers_local: (N, 7) [x, y, z, h, w, l, yaw] in RSU-local frame.
    """
    batch_data = train_utils.to_device(batch_data, device)
    cav_content = batch_data['ego']
    output = model(cav_content)
    fused_feature = output['fused_feature']  # [1, C, H, W]

    pred_box_tensor, pred_score = dataset.post_processor.post_process(
        batch_data, {'ego': output})

    if pred_box_tensor is None or len(pred_box_tensor) == 0:
        return fused_feature.detach().cpu().numpy(), \
               np.zeros((0, 7), dtype=np.float32), \
               np.zeros((0,), dtype=np.float32)

    # post_process returns corner format; convert to center 'hwl'
    centers = box_utils.corner_to_center_torch(pred_box_tensor, order='hwl')
    # centers: (N, 7) [x, y, z, h, w, l, yaw]
    mask = pred_score >= score_threshold
    centers = centers[mask]
    pred_score = pred_score[mask]

    return (fused_feature.detach().cpu().numpy(),
            centers.detach().cpu().numpy().astype(np.float32),
            pred_score.detach().cpu().numpy().astype(np.float32))


# --- Main --------------------------------------------------------------------

def main():
    args = parse_args()

    # Resolve output structure
    feat_root = os.path.join(args.output_dir, 'wf_fused_features')
    pred_root = os.path.join(args.output_dir, 'multiv2x_pred_traj')
    for split in ('train', 'test'):
        os.makedirs(os.path.join(feat_root, split), exist_ok=True)
        os.makedirs(os.path.join(pred_root, split), exist_ok=True)

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
    dataset = build_dataset(hypes, visualize=False, train=True)

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
        'records': [],
    }

    # Iterate zones
    with torch.inference_mode():
        for (scen, rsu, split), zsamples in zone_samples.items():
            logger.info(f'--- {scen}/{rsu} [{split}] : {len(zsamples)} frames')

            tracker = make_tracker()

            # Will hold per-object track {oid: {ts: [x,y,z,l,w,h,yaw,valid]}}
            tracks: Dict[int, Dict[str, List[float]]] = {}
            timestamps_seen: List[str] = []

            # Cache RSU pose once
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

                fused_np, centers_local, scores = run_one_frame(
                    model, dataset, batch_data, device, args.score_threshold)

                # Dump fused feature as plain .npy float16
                # (transfer-level compression is plain tar, not gzip/zlib)
                feat_path = os.path.join(
                    feat_root, split,
                    f'wf_fused_{scen}_{rsu}_{ts}.npy')
                np.save(feat_path, fused_np.astype(np.float16))

                # Transform detections to world frame (matches GT pickle)
                centers_world = transform_box_centers_to_world(
                    centers_local, rsu_pose)

                # AB3DMOT input: dets N x 7 [h,w,l,x,y,z,theta]
                if centers_world.size == 0:
                    dets = np.zeros((0, 7), dtype=np.float64)
                    # AB3DMOT info layout: [score, GUID, carla_id]
                    info_arr = np.zeros((0, 3), dtype=np.float64)
                else:
                    dets = np.stack([
                        centers_world[:, 3],  # h
                        centers_world[:, 4],  # w
                        centers_world[:, 5],  # l
                        centers_world[:, 0],  # x
                        centers_world[:, 1],  # y
                        centers_world[:, 2],  # z
                        centers_world[:, 6],  # theta
                    ], axis=1).astype(np.float64)
                    n = dets.shape[0]
                    info_arr = np.stack([
                        scores.astype(np.float64),
                        -np.ones(n, dtype=np.float64),  # GUID unknown
                        -np.ones(n, dtype=np.float64),  # carla_id unknown
                    ], axis=1)

                results, _ = tracker.track(
                    {'dets': dets, 'info': info_arr}, frame_idx)

                # results: list with one array N x 16: [h,w,l,x,y,z,theta, ID, ..., conf]
                timestamps_seen.append(ts)
                if results and results[0].size > 0:
                    for row in results[0]:
                        h, w, l, x, y, z, theta, oid = row[:8]
                        state = [
                            float(x), float(y), float(z),
                            float(l), float(w), float(h),
                            float(theta), 1,
                        ]
                        oid_int = int(oid)
                        if oid_int not in tracks:
                            tracks[oid_int] = OrderedDict()
                        tracks[oid_int][ts] = state

            # Flush per-zone pickle: align each track to the timestamps list,
            # padding missing timestamps with valid=0 placeholder.
            data_out = OrderedDict()
            invalid_state = [0.0] * (STATE_DIM - 1) + [0]
            for oid, ts_to_state in tracks.items():
                seq = [ts_to_state.get(ts, invalid_state)
                       for ts in timestamps_seen]
                data_out[oid] = seq

            out_pkl = os.path.join(pred_root, split,
                                   f'{scen}-{rsu}-traj.pickle')
            with open(out_pkl, 'wb') as f:
                pickle.dump({'data': dict(data_out),
                             'timestamps': timestamps_seen}, f)
            logger.info(f'  wrote {out_pkl}  '
                        f'({len(data_out)} tracks, {len(timestamps_seen)} ts)')

            manifest_out['records'].append({
                'scenario': scen,
                'rsu': rsu,
                'split': split,
                'n_timestamps': len(timestamps_seen),
                'n_tracks': len(data_out),
                'pred_traj_file': out_pkl,
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
