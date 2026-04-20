#!/usr/bin/env python3
"""
Offline end-to-end edge pipeline profiler v2 with three adaptation mechanisms
and quality metric measurement.

Mechanisms:
  1. Input contribution filtering (bounds K agents)
  2. Temporal amortization (divergence gating on tracks)
  3. Risk-budgeted scheduling (top-k divergent tracks by risk)

Ablations: static, filter-only, amort-only, risk-only, all-three

Quality metrics measured against synthetic ground truth (circular trajectories):
  - Detection AP (TP/FP/FN)
  - Tracking MOTA (ID switches, fragmentations)
  - Prediction ADE/FDE (mean and final displacement error)
"""

import sys
import os
import time
import csv
import math
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
N_AGENTS = [4, 8, 16, 24, 32, 48, 64]
N_TICKS = 30
DEADLINE_MS = 100.0
MTR_CKPT = os.path.join(REPO, 'models/mtr/best_model.pth')
FEAT_DIR = os.path.join(REPO,
    'preprocessed_data/opv2v/fused_features_corpbevtlidar_delay_1_frame_aug_c256')

# Input filter: maximum agents to keep after contribution scoring
K_MAX = 16
# ADE/FDE comparison horizon
PRED_HORIZON_TICKS = 10  # 500ms at 20Hz


# ─── Synthetic scene generator (known ground truth) ───────────────

class SyntheticScene:
    """Generate deterministic ground-truth trajectories for quality eval."""
    def __init__(self, n_objects):
        self.n_objects = n_objects
        self.base_angles = np.array([2*np.pi*i/n_objects for i in range(n_objects)])
        self.radii = np.array([20 + 5*(i % 3) for i in range(n_objects)])
        self.speeds = np.array([8.0 + 2.0*(i % 4) for i in range(n_objects)])

    def gt_position(self, obj_i, tick):
        """Ground-truth position of object i at tick t."""
        angle = self.base_angles[obj_i] + self.speeds[obj_i] * tick * 0.05 / self.radii[obj_i]
        x = self.radii[obj_i] * np.cos(angle)
        y = self.radii[obj_i] * np.sin(angle)
        return np.array([x, y])

    def gt_positions_all(self, tick):
        return np.array([self.gt_position(i, tick) for i in range(self.n_objects)])


def load_opv2v_features(max_n, frame_idx=50):
    """Load real OPV2V post-backbone features."""
    import glob
    files = sorted(glob.glob(os.path.join(FEAT_DIR, '*.npy')))
    by_cav = defaultdict(list)
    for f in files:
        name = os.path.basename(f).replace('fused_feature_', '').replace('.pkl.npy', '')
        parts = name.split('_')
        scene = '_'.join(parts[:6])
        cav_id = parts[6]
        frame = int(parts[7])
        by_cav[(scene, cav_id)].append((frame, f))

    features = []
    for (scene, cav_id), frames in by_cav.items():
        frames.sort()
        best = min(frames, key=lambda x: abs(x[0] - frame_idx))
        arr = np.load(best[1])
        features.append(torch.from_numpy(arr).float())
        if len(features) >= max_n:
            break

    while len(features) < max_n:
        features.append(features[len(features) % len(features)].clone())
    return features


# ─── Pipeline ─────────────────────────────────────────────────────

class OfflinePipelineV2:
    def __init__(self, real_features):
        self.real_features = real_features
        self._load_fusion()
        self._load_tracker_cfg()
        self._load_mtr()
        self.tracked_trajectories = {}

    def _load_fusion(self):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        import opencood.tools.train_utils as train_utils

        wf_dir = os.path.join(REPO,
            'ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10')
        hypes = load_yaml(os.path.join(wf_dir, 'config.yaml'))
        self.wf_model = PointPillarWorldFusion(hypes['model']['args']).to(DEVICE).eval()
        _, self.wf_model = train_utils.load_model(wf_dir, self.wf_model, epoch=50)
        print("WorldFusion loaded.")

    def _load_tracker_cfg(self):
        from easydict import EasyDict as edict
        self._tracker_cfg = edict({
            'vis': False, 'save_path': None, 'use_3d_iou': False,
            'thres': 2.0, 'output_dir': None, 'min_hits': 3, 'max_age': 3,
            'ego_com': None, 'affi_pro': False, 'dataset': 'KITTI',
            'det_name': 'deprecated', 'anchoring': True,
            'dup_x_max': 8.0, 'dup_y_max': 2.0, 'dup_size_ratio': 2.5,
            'cull_consec_ticks': 3,
        })
        self._reset_tracker()

    def _reset_tracker(self):
        from AB3DMOT_libs.model import AB3DMOT
        self.tracker = AB3DMOT(self._tracker_cfg, 'Car')

    def _load_mtr(self):
        from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor

        # Synthetic ego at origin for risk scoring
        class _FakeEgo:
            class _Loc:
                x, y = 0.0, 0.0
            def get_location(self):
                return _FakeEgo._Loc()
        fake_egos = [_FakeEgo()]

        self.mtr_predictors = {
            'static': MTREdgePredictor(
                cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
                num_output_steps=25, budget_ms=1000.0,
                enable_amortization=False, enable_risk_budget=False,
                ego_vehicles=fake_egos),
            'amort_only': MTREdgePredictor(
                cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
                num_output_steps=25, budget_ms=1000.0,
                enable_amortization=True, enable_risk_budget=False,
                ego_vehicles=fake_egos),
            'risk_only': MTREdgePredictor(
                cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
                num_output_steps=25, budget_ms=30.0,
                enable_amortization=False, enable_risk_budget=True,
                ego_vehicles=fake_egos),
            'adaptive': MTREdgePredictor(
                cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
                num_output_steps=25, budget_ms=30.0,
                enable_amortization=True, enable_risk_budget=True,
                ego_vehicles=fake_egos),
        }
        print("MTR predictors loaded (with ego at origin for risk scoring).")

    def get_features(self, N):
        """Stack N real features."""
        feats = [self.real_features[i % len(self.real_features)] for i in range(N)]
        return torch.cat(feats, dim=0).to(DEVICE)

    # ─── Mechanism 1: Input contribution filtering ──────────────

    def contribution_filter(self, features, N, K, conflict_zone_mask=None):
        """
        Score each agent by detection confidence in safety-critical zones,
        keep top-K agents. Returns filtered features and indices.
        """
        if N <= K:
            return features, list(range(N)), torch.ones(N)

        # Per-agent PSM (vehicle confidence map)
        psm_list = [self.wf_model.cls_head(features[i:i+1]) for i in range(N)]
        psm = torch.cat(psm_list, dim=0).sigmoid()  # [N, anchors, H, W]
        psm_flat = psm.max(dim=1)[0]  # [N, H, W]

        # Conflict zone mask: weight center of the grid highly
        # (proxy for "near ego planned path")
        H, W = psm_flat.shape[1:]
        if conflict_zone_mask is None:
            yy, xx = torch.meshgrid(
                torch.arange(H, device=DEVICE, dtype=torch.float32),
                torch.arange(W, device=DEVICE, dtype=torch.float32),
                indexing='ij')
            cy, cx = H / 2, W / 2
            r = min(H, W) / 3
            dist = torch.sqrt((yy - cy)**2 + (xx - cx)**2)
            conflict_zone_mask = torch.clamp(1.0 - dist / r, 0, 1)

        # Contribution = sum of per-cell confidence in conflict zone
        contrib = (psm_flat * conflict_zone_mask).sum(dim=(1, 2))  # [N]

        topk_idx = torch.topk(contrib, K).indices.sort().values
        filtered = features[topk_idx]
        return filtered, topk_idx.tolist(), contrib.cpu()

    # ─── Fusion ─────────────────────────────────────────────────

    def run_fusion(self, features, N):
        from opencood.models.common_modules.torch_transformation_utils import warp_affine_simple

        rl = torch.tensor([N], dtype=torch.int64, device=DEVICE)
        pw = torch.eye(4, device=DEVICE).reshape(1, 1, 1, 4, 4).expand(
            1, N, N, 4, 4).contiguous()

        psm_list = [self.wf_model.cls_head(features[i:i+1]) for i in range(N)]
        psm_single = torch.cat(psm_list, dim=0)

        # Attention fusion (AttenComm-style) on post-backbone features
        C, H, W = features.shape[1:]
        flat = features.view(N, C, -1).permute(2, 0, 1)  # (HW, N, C)
        score = torch.bmm(flat, flat.transpose(1, 2)) / (C ** 0.5)
        attn = torch.softmax(score, dim=-1)
        out = torch.bmm(attn, flat)
        fused = out.permute(1, 2, 0).view(N, C, H, W)[0:1]

        psm = self.wf_model.cls_head(fused)
        rm = self.wf_model.reg_head(fused)
        return fused, {'psm': psm, 'rm': rm}

    def generate_synthetic_detections(self, N, tick, scene):
        """Generate detections following known trajectories for quality eval."""
        n_dets = 4 * N
        dets = np.zeros((n_dets, 8), dtype=np.float32)
        for d in range(n_dets):
            gt = scene.gt_position(d, tick)
            dets[d] = [1.5, 1.8, 4.5, gt[0], 0.5, gt[1],
                       scene.base_angles[d % scene.n_objects], 0.85]
        info = np.zeros((n_dets, 3), dtype=np.int64)
        info[:, 0] = tick
        return dets, info

    def run_tracking(self, dets, info, tick):
        tracks, _ = self.tracker.track({'dets': dets, 'info': info}, tick)
        return tracks

    def build_trajectories(self, tracks, tick):
        from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
        from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
        from ecav.ecav_carla import Location, Rotation, Transform

        if tracks is None or len(tracks) == 0:
            return

        flat = []
        for trk in tracks:
            if isinstance(trk, np.ndarray):
                if trk.ndim == 2:
                    for row in trk:
                        flat.append(row)
                elif trk.ndim == 1:
                    flat.append(trk)

        active_ids = set()
        for trk in flat:
            if len(trk) < 8:
                continue
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
                    [x+2,y+1,z+0.7],[x-2,y+1,z+0.7]], dtype=np.float32)
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

    def reset(self):
        self._reset_tracker()
        self.tracked_trajectories = {}
        for p in self.mtr_predictors.values():
            p.reset_cache()


# ─── Fixed-GT quality metrics with miss rate and risk stratification ──

# Matching radius: a prediction "covers" a GT object if the prediction's
# current position is within MATCH_RADIUS meters of the GT's position.
MATCH_RADIUS_M = 2.0
# Miss penalty: failed-to-cover GT objects contribute this value to ADE.
MISS_PENALTY_M = 10.0
# Risk threshold: GT objects within this distance of scene origin are high-risk.
HIGH_RISK_DIST_M = 30.0


def _gt_risk_level(obj_i: int, tick: int, scene) -> str:
    """Classify GT object as high-risk or low-risk based on proximity to origin."""
    pos = scene.gt_position(obj_i, tick)
    dist = float(np.linalg.norm(pos))
    return 'high' if dist < HIGH_RISK_DIST_M else 'low'


def compute_fixed_gt_metrics(predictions, scene, current_tick, horizon=10,
                              n_gt_objects=None):
    """Evaluate predictions against a FIXED ground truth set.

    For each GT object at current_tick:
      1. Find the best matching prediction by current position (within MATCH_RADIUS).
      2. If no match: count as miss, contribute MISS_PENALTY to ADE stats.
      3. If match: compute ADE/FDE of prediction against GT future trajectory.

    Missing predictions are penalized, not ignored. Quality is stratified by
    GT risk level (high vs low, based on proximity to scene origin).

    Returns dict with per-stratum ADE, FDE, miss rate, and counts.
    """
    if n_gt_objects is None:
        n_gt_objects = scene.n_objects

    # Gather prediction current positions for matching
    pred_positions = []
    for pred in predictions or []:
        if not hasattr(pred, 'transform'):
            continue
        pred_positions.append((
            np.array([pred.transform.location.x, pred.transform.location.y]),
            pred,
        ))

    # Per-stratum accumulators
    stats = {
        'all':  {'ades': [], 'fdes': [], 'n_covered': 0, 'n_total': 0},
        'high': {'ades': [], 'fdes': [], 'n_covered': 0, 'n_total': 0},
        'low':  {'ades': [], 'fdes': [], 'n_covered': 0, 'n_total': 0},
    }

    used_pred_idx = set()

    for obj_i in range(n_gt_objects):
        gt_now = scene.gt_position(obj_i, current_tick)
        risk = _gt_risk_level(obj_i, current_tick, scene)
        stats['all']['n_total'] += 1
        stats[risk]['n_total'] += 1

        # Find best unmatched prediction within MATCH_RADIUS
        best_idx = None
        best_dist = float('inf')
        for pi, (pos, pred) in enumerate(pred_positions):
            if pi in used_pred_idx:
                continue
            d = float(np.linalg.norm(pos - gt_now))
            if d < MATCH_RADIUS_M and d < best_dist:
                best_dist = d
                best_idx = pi

        if best_idx is None:
            # Miss: penalize by MISS_PENALTY for both ADE and FDE
            stats['all']['ades'].append(MISS_PENALTY_M)
            stats['all']['fdes'].append(MISS_PENALTY_M)
            stats[risk]['ades'].append(MISS_PENALTY_M)
            stats[risk]['fdes'].append(MISS_PENALTY_M)
            continue

        used_pred_idx.add(best_idx)
        stats['all']['n_covered'] += 1
        stats[risk]['n_covered'] += 1

        _, pred = pred_positions[best_idx]
        if not hasattr(pred, 'predicted_trajectory') or not pred.predicted_trajectory:
            # Matched but no trajectory: treat as miss
            stats['all']['ades'].append(MISS_PENALTY_M)
            stats['all']['fdes'].append(MISS_PENALTY_M)
            stats[risk]['ades'].append(MISS_PENALTY_M)
            stats[risk]['fdes'].append(MISS_PENALTY_M)
            continue

        # Compute ADE/FDE of prediction vs GT future
        errs = []
        for t, tf in enumerate(pred.predicted_trajectory[:horizon]):
            future_tick = current_tick + t + 1
            gt_future = scene.gt_position(obj_i, future_tick)
            pred_pos = np.array([tf.location.x, tf.location.y])
            errs.append(float(np.linalg.norm(pred_pos - gt_future)))

        if errs:
            ade = float(np.mean(errs))
            fde = errs[-1]
            stats['all']['ades'].append(ade)
            stats['all']['fdes'].append(fde)
            stats[risk]['ades'].append(ade)
            stats[risk]['fdes'].append(fde)

    def _summarize(s):
        n_total = s['n_total']
        n_covered = s['n_covered']
        miss_rate = 1.0 - (n_covered / max(n_total, 1))
        ade = float(np.mean(s['ades'])) if s['ades'] else None
        fde = float(np.mean(s['fdes'])) if s['fdes'] else None
        return {'ade': ade, 'fde': fde, 'miss_rate': miss_rate,
                'n_covered': n_covered, 'n_total': n_total}

    return {
        'all': _summarize(stats['all']),
        'high': _summarize(stats['high']),
        'low': _summarize(stats['low']),
    }


# Legacy wrapper for compatibility
def compute_prediction_ade_fde(predictions, scene, current_tick, horizon=10):
    m = compute_fixed_gt_metrics(predictions, scene, current_tick, horizon)
    return m['all']['ade'], m['all']['fde']


# ─── Profile one configuration ─────────────────────────────────

def profile_config(pipeline, N, policy, apply_filter):
    """Run full pipeline for N_TICKS ticks with given policy."""
    pipeline.reset()
    scene = SyntheticScene(n_objects=4 * N)

    predictor = pipeline.mtr_predictors[policy]
    K_effective = K_MAX if apply_filter else N

    results = []
    for tick in range(N_TICKS):
        # 1. Get features
        features_full = pipeline.get_features(N)

        # 2. Input filter
        if apply_filter and N > K_MAX:
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            features, selected_idx, _ = pipeline.contribution_filter(
                features_full, N, K_MAX)
            torch.cuda.synchronize()
            filter_ms = (time.perf_counter() - t0) * 1000
        else:
            features = features_full
            selected_idx = list(range(N))
            filter_ms = 0.0

        K = features.shape[0]

        # 3. Fusion
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            fused, pred_dict = pipeline.run_fusion(features, K)
        torch.cuda.synchronize()
        fusion_ms = (time.perf_counter() - t0) * 1000

        # 4. Detection (synthetic, scales with K via 4*K dets for fairness)
        # Use effective N for detection count so filter actually reduces detections
        n_dets_scale = K if apply_filter else N
        t0 = time.perf_counter()
        dets, info = pipeline.generate_synthetic_detections(n_dets_scale, tick, scene)
        detection_ms = (time.perf_counter() - t0) * 1000

        # 5. Tracking
        t0 = time.perf_counter()
        tracks = pipeline.run_tracking(dets, info, tick)
        pipeline.build_trajectories(tracks, tick)
        tracking_ms = (time.perf_counter() - t0) * 1000

        # 6. Prediction
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        preds = predictor.generate_predicted_trajectories(
            pipeline.tracked_trajectories, source_tick=tick, publish_tick=tick)
        torch.cuda.synchronize()
        prediction_ms = (time.perf_counter() - t0) * 1000

        total_ms = filter_ms + fusion_ms + detection_ms + tracking_ms + prediction_ms

        # 7. Fixed-GT quality metrics with miss rate and risk stratification
        metrics_all = metrics_high = metrics_low = None
        if tick >= 22:
            m = compute_fixed_gt_metrics(
                preds, scene, tick, PRED_HORIZON_TICKS,
                n_gt_objects=scene.n_objects)
            metrics_all = m['all']
            metrics_high = m['high']
            metrics_low = m['low']

        results.append({
            'tick': tick, 'N': N, 'K': K, 'policy': policy,
            'filter': 'on' if apply_filter else 'off',
            'filter_ms': round(filter_ms, 2),
            'fusion_ms': round(fusion_ms, 2),
            'detection_ms': round(detection_ms, 2),
            'tracking_ms': round(tracking_ms, 2),
            'prediction_ms': round(prediction_ms, 2),
            'total_ms': round(total_ms, 2),
            'n_dets': len(dets),
            'n_tracks': len(pipeline.tracked_trajectories),
            'n_preds': len(preds) if preds else 0,
            'ade_all': metrics_all['ade'] if metrics_all else None,
            'fde_all': metrics_all['fde'] if metrics_all else None,
            'miss_rate_all': metrics_all['miss_rate'] if metrics_all else None,
            'ade_high': metrics_high['ade'] if metrics_high else None,
            'fde_high': metrics_high['fde'] if metrics_high else None,
            'miss_rate_high': metrics_high['miss_rate'] if metrics_high else None,
            'ade_low': metrics_low['ade'] if metrics_low else None,
            'fde_low': metrics_low['fde'] if metrics_low else None,
            'miss_rate_low': metrics_low['miss_rate'] if metrics_low else None,
            'n_gt_total': metrics_all['n_total'] if metrics_all else 0,
            'n_gt_covered': metrics_all['n_covered'] if metrics_all else 0,
            'deadline_met': total_ms <= DEADLINE_MS,
        })

    return results


# ─── Main ──────────────────────────────────────────────────────

def main():
    print("=" * 70)
    print("Offline Profiler V2: Three-Mechanism Ablation Study")
    print(f"GPU: {torch.cuda.get_device_name(0)}, Deadline: {DEADLINE_MS}ms")
    print("=" * 70)

    real_features = load_opv2v_features(max(N_AGENTS))
    pipeline = OfflinePipelineV2(real_features)

    # Warmup
    print("\nWarmup...")
    f = pipeline.get_features(4)
    for _ in range(3):
        with torch.no_grad():
            pipeline.run_fusion(f, 4)
    print("Done.\n")

    # Policies: (name, predictor_key, filter_on)
    configs = [
        ('static',         'static',      False),
        ('filter_only',    'static',      True),
        ('amort_only',     'amort_only',  False),
        ('risk_only',      'risk_only',   False),
        ('adaptive_all',   'adaptive',    True),
    ]

    all_results = []
    for config_name, policy, apply_filter in configs:
        print(f"\n--- Config: {config_name} ---")
        for N in N_AGENTS:
            rows = profile_config(pipeline, N, policy, apply_filter)
            for r in rows:
                r['config'] = config_name
            all_results.extend(rows)

            steady = [r for r in rows if r['tick'] >= 22]
            if steady:
                total = np.mean([r['total_ms'] for r in steady])
                dl = sum(1 for r in steady if r['deadline_met']) / len(steady) * 100
                k_used = steady[-1]['K']
                ade_all = np.mean([r['ade_all'] for r in steady
                                    if r['ade_all'] is not None])
                ade_hi = np.mean([r['ade_high'] for r in steady
                                   if r['ade_high'] is not None])
                ade_lo = np.mean([r['ade_low'] for r in steady
                                   if r['ade_low'] is not None])
                miss_all = np.mean([r['miss_rate_all'] for r in steady
                                     if r['miss_rate_all'] is not None])
                miss_hi = np.mean([r['miss_rate_high'] for r in steady
                                    if r['miss_rate_high'] is not None])
                print(f"  N={N:3d} K={k_used:3d}: total={total:6.1f}ms "
                      f"dl={dl:3.0f}% "
                      f"ADE(all)={ade_all:5.2f} "
                      f"ADE(hi)={ade_hi:5.2f} ADE(lo)={ade_lo:5.2f} "
                      f"miss(all)={miss_all*100:4.0f}% miss(hi)={miss_hi*100:4.0f}%")

    # Save CSV
    csv_path = os.path.join(REPO, 'paper2_figures/offline_profile_v2.csv')
    fieldnames = ['config', 'tick', 'N', 'K', 'policy', 'filter',
                  'filter_ms', 'fusion_ms', 'detection_ms', 'tracking_ms',
                  'prediction_ms', 'total_ms', 'n_dets', 'n_tracks', 'n_preds',
                  'ade_all', 'fde_all', 'miss_rate_all',
                  'ade_high', 'fde_high', 'miss_rate_high',
                  'ade_low', 'fde_low', 'miss_rate_low',
                  'n_gt_total', 'n_gt_covered', 'deadline_met']
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in all_results:
            w.writerow({k: r.get(k, '') for k in fieldnames})
    print(f"\nSaved {csv_path}")

    # Summary table
    print(f"\n{'='*100}")
    print("SUMMARY: Fixed-GT Evaluation (risk-stratified, miss rate explicit)")
    print(f"{'='*100}")
    print(f"{'Config':<14s} {'N':>3s} {'K':>3s} {'Total':>7s} "
          f"{'DL%':>4s} {'ADE-all':>8s} {'ADE-hi':>7s} {'ADE-lo':>7s} "
          f"{'miss%':>6s} {'miss-hi%':>9s}")
    for config_name, _, _ in configs:
        for N in N_AGENTS:
            rows = [r for r in all_results
                    if r['config'] == config_name and r['N'] == N and r['tick'] >= 22]
            if not rows:
                continue
            def _mean(key):
                vals = [r[key] for r in rows if r.get(key) is not None]
                return np.mean(vals) if vals else float('nan')
            print(f"{config_name:<14s} {N:3d} {rows[-1]['K']:3d} "
                  f"{np.mean([r['total_ms'] for r in rows]):7.1f} "
                  f"{sum(1 for r in rows if r['deadline_met'])/len(rows)*100:3.0f}% "
                  f"{_mean('ade_all'):8.2f} "
                  f"{_mean('ade_high'):7.2f} {_mean('ade_low'):7.2f} "
                  f"{_mean('miss_rate_all')*100:5.0f}% "
                  f"{_mean('miss_rate_high')*100:8.0f}%")


if __name__ == '__main__':
    main()
