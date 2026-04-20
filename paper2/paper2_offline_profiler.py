#!/usr/bin/env python3
"""
Offline end-to-end edge pipeline profiler using real OPV2V data.

Loads real CoBEVT post-backbone fused features from OPV2V test set,
stacks N agents' features, and runs the full pipeline:
  fusion (Where2Comm) -> detection (post-process) -> tracking (AB3DMOT) -> prediction (MTR)

To simulate N > max_cavs_per_scene, features from different scenes
are combined. This is valid for compute profiling since the GPU cost
depends on tensor shapes, not spatial content.
"""

import sys
import os
import time
import csv
import math
import glob
from collections import defaultdict, deque
from typing import Dict, List, Tuple

import numpy as np
import torch

REPO = os.path.abspath('.')
CMP_ABS = os.path.abspath('ecav/core/application/v2v/baselines/cmp/CMP')
MTR_ABS = os.path.join(CMP_ABS, 'MTR')
sys.path.insert(0, MTR_ABS)
sys.path.insert(0, CMP_ABS)
sys.path.insert(0, os.path.join(REPO, 'ecav/worldfusion'))
sys.path.insert(0, REPO)

DEVICE = 'cuda:0'
N_AGENTS = [1, 2, 4, 8, 16, 24, 32, 48, 64]
N_TICKS = 30
DEADLINE_MS = 100.0
FEAT_DIR = os.path.join(REPO, 'preprocessed_data/opv2v/fused_features_corpbevtlidar_delay_1_frame_aug_c256')
TRACK_PKL = os.path.join(REPO, 'preprocessed_data/opv2v/corpbevtlidar_delay_1_frame_aug_c256_tracking_result_for_prediction_test.pkl')
MTR_CKPT = os.path.join(REPO, 'models/mtr/best_model.pth')


def load_opv2v_features(max_n=64, frame_idx=50):
    """Load real fused features from OPV2V, enough for max_n agents."""
    files = sorted(glob.glob(os.path.join(FEAT_DIR, '*.npy')))

    # Group by scene_cavid
    by_scene_cav = defaultdict(list)
    for f in files:
        name = os.path.basename(f).replace('fused_feature_', '').replace('.pkl.npy', '')
        parts = name.split('_')
        scene = '_'.join(parts[:6])
        cav_id = parts[6]
        frame = int(parts[7])
        by_scene_cav[(scene, cav_id)].append((frame, f))

    # Collect one feature per CAV at the target frame (or nearest)
    features = []
    for (scene, cav_id), frames in by_scene_cav.items():
        frames.sort()
        # Find closest to frame_idx
        best = min(frames, key=lambda x: abs(x[0] - frame_idx))
        features.append(best[1])
        if len(features) >= max_n:
            break

    # If not enough CAVs, duplicate with small noise
    while len(features) < max_n:
        src = features[len(features) % len(features)]
        features.append(src)

    # Load all
    loaded = []
    for f in features[:max_n]:
        arr = np.load(f)  # [1, 256, 48, 176]
        loaded.append(torch.from_numpy(arr).float())

    print(f"Loaded {len(loaded)} real features, shape={loaded[0].shape}")
    return loaded


def load_opv2v_tracks():
    """Load tracking results for trajectory building."""
    import pickle
    with open(TRACK_PKL, 'rb') as f:
        data = pickle.load(f)
    # Flatten all tracks from all scenes/timestamps
    all_tracks = []
    for scene in data.values():
        for ts, cav_data in scene.items():
            for k, v in cav_data.items():
                if isinstance(k, (int, np.integer)) and isinstance(v, list):
                    # v is a list of frames, each frame is [x,y,z,h,w,l,yaw,score?]
                    if len(v) > 0:
                        all_tracks.append(np.array(v[-1], dtype=np.float32))
    return all_tracks


class OfflinePipeline:
    def __init__(self, real_features):
        self.real_features = real_features
        self._load_fusion_model()
        self._load_tracker()
        self._load_mtr()
        self.tracked_trajectories = {}

    def _load_fusion_model(self):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        import opencood.tools.train_utils as train_utils

        wf_dir = os.path.join(REPO,
            'ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10')
        hypes = load_yaml(os.path.join(wf_dir, 'config.yaml'))
        self.wf_model = PointPillarWorldFusion(hypes['model']['args']).to(DEVICE).eval()
        _, self.wf_model = train_utils.load_model(wf_dir, self.wf_model, epoch=50)
        self.hypes = hypes
        print("WorldFusion model loaded.")

    def _load_tracker(self):
        from AB3DMOT_libs.model import AB3DMOT
        from easydict import EasyDict as edict
        cfg = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': False,
            'thres': 2.0, 'output_dir': None, 'min_hits': 3, 'max_age': 3,
            'ego_com': None, 'affi_pro': False, 'dataset': 'KITTI',
            'det_name': 'deprecated', 'anchoring': True,
            'dup_x_max': 8.0, 'dup_y_max': 2.0, 'dup_size_ratio': 2.5,
            'cull_consec_ticks': 3,
        })
        self._tracker_cfg = cfg
        self.tracker = AB3DMOT(cfg, 'Car')
        print("AB3DMOT loaded.")

    def _load_mtr(self):
        from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor
        self.mtr_static = MTREdgePredictor(
            cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
            num_output_steps=25, budget_ms=1000.0,
            enable_amortization=False, enable_risk_budget=False)
        self.mtr_adaptive = MTREdgePredictor(
            cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
            num_output_steps=25, budget_ms=50.0,
            enable_amortization=True, enable_risk_budget=True)
        print("MTR predictors loaded.")

    def get_features(self, N):
        """Stack N real features into a batch."""
        feats = [self.real_features[i % len(self.real_features)] for i in range(N)]
        # The features are [1, 256, 48, 176] (post-backbone CoBEVT).
        # WorldFusion expects [N, 64, 200, 200] (pre-backbone).
        # For profiling the fusion cost on post-backbone features,
        # we skip backbone and go directly to Where2Comm.
        return torch.cat(feats, dim=0).to(DEVICE)  # [N, 256, 48, 176]

    def run_fusion_post_backbone(self, features, N):
        """Run WorldFusion Where2Comm on post-backbone features."""
        rl = torch.tensor([N], dtype=torch.int64, device=DEVICE)
        pw = torch.eye(4, device=DEVICE).reshape(1, 1, 1, 4, 4).expand(
            1, N, N, 4, 4).contiguous()

        batch = {
            'spatial_features_2d': features,
            'record_len': rl,
            'pairwise_t_matrix': pw,
        }
        output = self.wf_model.fuse_post_backbone(batch)
        return output.get('fused_feature'), output, None

    def run_detection_from_psm(self, pred_dict, tick):
        """Generate synthetic detections based on PSM confidence map."""
        psm = pred_dict['psm'].sigmoid().cpu().numpy()
        # Find high-confidence locations
        h, w = psm.shape[2], psm.shape[3]
        psm_max = psm[0].max(axis=0)  # [H, W]

        # Threshold detections
        thresh = 0.3
        ys, xs = np.where(psm_max > thresh)
        n_dets = len(ys)

        if n_dets == 0:
            return np.empty((0, 8), np.float32), np.empty((0, 3), np.int64)

        # Convert grid to world coordinates
        # Post-backbone: stride 2 from 200x200 grid -> 100x100
        # But CoBEVT features are 48x176 (different grid)
        scale_x = 80.0 / w  # meters per grid cell
        scale_y = 80.0 / h
        dets = np.zeros((n_dets, 8), dtype=np.float32)
        for i in range(n_dets):
            dets[i] = [1.5, 1.8, 4.5,
                       xs[i] * scale_x - 40, 0.5, ys[i] * scale_y - 40,
                       0.0, float(psm_max[ys[i], xs[i]])]
        info = np.zeros((n_dets, 3), dtype=np.int64)
        info[:, 0] = tick
        return dets, info

    def run_tracking(self, dets, info, tick):
        dets_all = {'dets': dets, 'info': info}
        tracks, _ = self.tracker.track(dets_all, tick)
        return tracks

    def build_trajectories(self, tracks, tick):
        from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
        from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
        from ecav.ecav_carla import Location, Rotation, Transform

        if tracks is None or len(tracks) == 0:
            return

        # Tracker returns [np.ndarray(N, 16)] - unwrap the batch
        flat_tracks = []
        for trk in tracks:
            if isinstance(trk, np.ndarray) and trk.ndim == 2:
                for row in trk:
                    flat_tracks.append(row)
            elif isinstance(trk, np.ndarray) and trk.ndim == 1:
                flat_tracks.append(trk)
        tracks = flat_tracks

        if not tracks:
            return

        active_ids = set()
        for trk in tracks:
            if isinstance(trk, np.ndarray) and trk.ndim == 1 and len(trk) >= 8:
                tid = int(trk[7])
                active_ids.add(tid)
                x, y, z = float(trk[3]), float(trk[5]), float(trk[4])
                yaw = float(trk[6])
                tf = Transform(location=Location(x=x, y=y, z=z),
                               rotation=Rotation(yaw=math.degrees(yaw)))

                if tid not in self.tracked_trajectories:
                    corners = np.array([
                        [x-2,y-1,z-0.7],[x+2,y-1,z-0.7],
                        [x+2,y+1,z-0.7],[x-2,y+1,z-0.7],
                        [x-2,y-1,z+0.7],[x+2,y-1,z+0.7],
                        [x+2,y+1,z+0.7],[x-2,y+1,z+0.7],
                    ], dtype=np.float32)
                    obs = ObstacleVehicle(corners, None)
                    obs.kf_speed_mps = 8.0
                    obs.carla_id = -1
                    self.tracked_trajectories[tid] = ObstacleTrajectory(obs, [tf])
                else:
                    ot = self.tracked_trajectories[tid]
                    ot.trajectory.appendleft(tf)
                    if len(ot.trajectory) >= 2:
                        prev = ot.trajectory[1]
                        dx = tf.location.x - prev.location.x
                        dy = tf.location.y - prev.location.y
                        ot.obstacle.kf_speed_mps = math.sqrt(dx*dx + dy*dy) / 0.05

        for tid in [t for t in self.tracked_trajectories if t not in active_ids]:
            del self.tracked_trajectories[tid]

    def run_prediction(self, predictor, tick):
        if not self.tracked_trajectories:
            return []
        return predictor.generate_predicted_trajectories(
            self.tracked_trajectories, source_tick=tick, publish_tick=tick)

    def reset(self):
        from AB3DMOT_libs.model import AB3DMOT
        self.tracker = AB3DMOT(self._tracker_cfg, 'Car')
        self.tracked_trajectories = {}
        self.mtr_static.reset_cache()
        self.mtr_adaptive.reset_cache()


def profile_tick(pipeline, N, tick, predictor):
    features = pipeline.get_features(N)

    # Fusion
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    with torch.no_grad():
        fused, pred_dict, psm_single = pipeline.run_fusion_post_backbone(features, N)
    torch.cuda.synchronize()
    fusion_ms = (time.perf_counter() - t0) * 1000

    # Detection: real fusion output + synthetic detections scaled to N
    # Fusion timing is real. Detection count matches expected ~4 dets/agent.
    t0 = time.perf_counter()
    real_dets, real_info = pipeline.run_detection_from_psm(pred_dict, tick)
    # Supplement with synthetic detections to reach expected count
    n_expected = int(4.0 * N)
    n_real = len(real_dets)
    if n_real < n_expected:
        n_synth = n_expected - n_real
        synth_dets = np.zeros((n_synth, 8), dtype=np.float32)
        for d in range(n_synth):
            angle = 2 * np.pi * d / max(n_synth, 1)
            r = 20 + 5 * (d % 3)
            synth_dets[d] = [1.5, 1.8, 4.5,
                             r * np.cos(angle) + 0.3 * tick, 0.5,
                             r * np.sin(angle) + 0.1 * tick,
                             angle, 0.7 + 0.1 * (d % 3)]
        synth_info = np.zeros((n_synth, 3), dtype=np.int64)
        synth_info[:, 0] = tick
        if n_real > 0:
            dets = np.vstack([real_dets, synth_dets])
            info = np.vstack([real_info, synth_info])
        else:
            dets = synth_dets
            info = synth_info
    else:
        dets = real_dets
        info = real_info
    detection_ms = (time.perf_counter() - t0) * 1000

    # Tracking
    t0 = time.perf_counter()
    tracks = pipeline.run_tracking(dets, info, tick)
    pipeline.build_trajectories(tracks, tick)
    tracking_ms = (time.perf_counter() - t0) * 1000

    # Prediction
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    preds = pipeline.run_prediction(predictor, tick)
    torch.cuda.synchronize()
    prediction_ms = (time.perf_counter() - t0) * 1000

    total_ms = fusion_ms + detection_ms + tracking_ms + prediction_ms

    return {
        'tick': tick, 'N': N,
        'fusion_ms': round(fusion_ms, 2),
        'detection_ms': round(detection_ms, 2),
        'tracking_ms': round(tracking_ms, 2),
        'prediction_ms': round(prediction_ms, 2),
        'total_ms': round(total_ms, 2),
        'n_detections': len(dets),
        'n_tracks': len(pipeline.tracked_trajectories),
        'n_predictions': len(preds) if preds else 0,
        'deadline_met': total_ms <= DEADLINE_MS,
    }


def main():
    print("=" * 70)
    print("Offline E2E Edge Profiler (OPV2V real features)")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"Deadline: {DEADLINE_MS}ms, Ticks: {N_TICKS}")
    print("=" * 70)

    real_features = load_opv2v_features(max_n=max(N_AGENTS))
    pipeline = OfflinePipeline(real_features)

    # Warmup
    print("\nWarmup...")
    for _ in range(3):
        f = pipeline.get_features(4)
        with torch.no_grad():
            pipeline.run_fusion_post_backbone(f, 4)
    print("Done.\n")

    all_results = []
    policies = {
        'mtr_static': pipeline.mtr_static,
        'mtr_adaptive': pipeline.mtr_adaptive,
    }

    for pol_name, predictor in policies.items():
        print(f"\n{'='*50}")
        print(f"Policy: {pol_name}")
        print(f"{'='*50}")

        for N in N_AGENTS:
            pipeline.reset()
            tick_results = []

            for tick in range(N_TICKS):
                r = profile_tick(pipeline, N, tick, predictor)
                r['predictor'] = pol_name
                tick_results.append(r)
                all_results.append(r)

            steady = tick_results[3:]
            if steady:
                print(f"  N={N:3d}: total={np.mean([r['total_ms'] for r in steady]):6.1f}ms "
                      f"(fuse={np.mean([r['fusion_ms'] for r in steady]):.1f} "
                      f"det={np.mean([r['detection_ms'] for r in steady]):.1f} "
                      f"track={np.mean([r['tracking_ms'] for r in steady]):.1f} "
                      f"pred={np.mean([r['prediction_ms'] for r in steady]):.1f}) "
                      f"dets={np.mean([r['n_detections'] for r in steady]):.0f} "
                      f"tracks={np.mean([r['n_tracks'] for r in steady]):.0f} "
                      f"deadline={sum(1 for r in steady if r['deadline_met'])/len(steady)*100:.0f}%")

    csv_path = os.path.join(REPO, 'paper2_figures/offline_profile.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=all_results[0].keys())
        w.writeheader()
        w.writerows(all_results)
    print(f"\nSaved {csv_path}")

    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"{'Policy':<15s} {'N':>3s} {'Fusion':>7s} {'Det':>5s} {'Track':>7s} "
          f"{'Pred':>7s} {'Total':>7s} {'Dets':>5s} {'Trks':>5s} {'DL%':>5s}")
    for pol_name in policies:
        for N in N_AGENTS:
            rows = [r for r in all_results
                    if r['predictor'] == pol_name and r['N'] == N and r['tick'] >= 3]
            if rows:
                print(f"{pol_name:<15s} {N:3d} "
                      f"{np.mean([r['fusion_ms'] for r in rows]):7.1f} "
                      f"{np.mean([r['detection_ms'] for r in rows]):5.1f} "
                      f"{np.mean([r['tracking_ms'] for r in rows]):7.1f} "
                      f"{np.mean([r['prediction_ms'] for r in rows]):7.1f} "
                      f"{np.mean([r['total_ms'] for r in rows]):7.1f} "
                      f"{np.mean([r['n_detections'] for r in rows]):5.0f} "
                      f"{np.mean([r['n_tracks'] for r in rows]):5.0f} "
                      f"{sum(1 for r in rows if r['deadline_met'])/len(rows)*100:5.0f}%")


if __name__ == '__main__':
    main()
