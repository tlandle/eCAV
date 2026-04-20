#!/usr/bin/env python3
"""
Offline profiler v3: Multi-V2X real data evaluation.

Runs the actual WorldFusion edge pipeline (same code path as
edge_manager_worldfusion_ab3dmot_linear_predictor) on Multi-V2X data:

    Vehicle side: pillar_vfe + scatter → spatial_features [N, C*Z, H, W]
    Edge side: N × backbone + Where2Comm fusion + detection heads
               + AB3DMOT tracking + MTR prediction

Usage:
    conda activate opencda310
    python paper2_offline_profiler_v3.py \\
        --data-root /data1/Datasets/Multi-V2X \\
        --town Town10HD__2023_11_15_00_20_38 \\
        --ego-cav cav_105 --max-frames 50
"""

import sys
import os
import time
import csv
import math
import argparse
from collections import defaultdict, deque
from typing import Dict, List, Tuple
from pathlib import Path

import numpy as np
import torch
import yaml as pyyaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CMP_ABS = os.path.join(REPO, 'ecav/core/application/v2v/baselines/cmp/CMP')
MTR_ABS = os.path.join(CMP_ABS, 'MTR')
sys.path.insert(0, MTR_ABS)
sys.path.insert(0, CMP_ABS)
sys.path.insert(0, os.path.join(REPO, 'ecav/worldfusion'))
sys.path.insert(0, REPO)

DEVICE = 'cuda:0'
DEADLINE_MS = 100.0
MTR_CKPT = os.path.join(REPO, 'models/mtr/best_model.pth')
K_MAX = 16
PRED_HORIZON_TICKS = 10


# ─── Multi-V2X scene loader ─────────────────────────────────────

class MultiV2XScene:
    """Load a sequence of Multi-V2X frames for one RSU zone.

    The RSU is the world anchor (ego). Connected CAVs contribute
    features but the coordinate frame is RSU-centric, matching
    the training dataset.
    """

    def __init__(self, data_root: str, town: str, rsu_name: str = None,
                 max_frames: int = 50):
        self.town_dir = Path(data_root) / town
        self._yaml_cache: Dict[str, dict] = {}

        # Find RSUs in this town
        rsu_dirs = sorted([
            d.name for d in self.town_dir.iterdir()
            if d.is_dir() and d.name.startswith('rsu_')
        ])
        if not rsu_dirs:
            raise ValueError(f"No RSU directories in {town}")

        if rsu_name and rsu_name in rsu_dirs:
            self.ego_cav = rsu_name
        else:
            self.ego_cav = rsu_dirs[0]

        self.ego_dir = self.town_dir / self.ego_cav

        self.timestamps = sorted([
            f.stem for f in self.ego_dir.glob('*.yaml')
            if f.stem.isdigit()
        ])
        if max_frames and len(self.timestamps) > max_frames:
            start = (len(self.timestamps) - max_frames) // 2
            self.timestamps = self.timestamps[start:start + max_frames]

        print(f"[MultiV2XScene] {town}/{self.ego_cav}: "
              f"{len(self.timestamps)} frames, {len(rsu_dirs)} RSUs")

    def _load_yaml(self, agent_dir: Path, timestamp: str) -> dict:
        key = f"{agent_dir.name}/{timestamp}"
        if key not in self._yaml_cache:
            with open(agent_dir / f'{timestamp}.yaml') as f:
                self._yaml_cache[key] = pyyaml.safe_load(f)
        return self._yaml_cache[key]

    def ego_yaml(self, tick_idx: int) -> dict:
        return self._load_yaml(self.ego_dir, self.timestamps[tick_idx])

    def connected_agents(self, tick_idx: int) -> List[str]:
        """Return connected agents. RSU (ego) is always included."""
        data = self.ego_yaml(tick_idx)
        conn_ids = data.get('conn_agents', [])
        agents = []
        for aid in conn_ids:
            for prefix in ['cav_', 'rsu_']:
                agent_name = f'{prefix}{aid}'
                d = self.town_dir / agent_name
                if d.is_dir() and agent_name != self.ego_cav:
                    agents.append(agent_name)
                    break
        return agents

    def gt_objects(self, tick_idx: int) -> dict:
        return self.ego_yaml(tick_idx).get('objects', {})

    def agent_lidar_pose(self, agent_name: str, tick_idx: int) -> list:
        return self._load_yaml(
            self.town_dir / agent_name, self.timestamps[tick_idx])['lidar_pose']

    def agent_pcd_path(self, agent_name: str, tick_idx: int) -> str:
        return str(self.town_dir / agent_name / f'{self.timestamps[tick_idx]}.pcd')

    def n_frames(self) -> int:
        return len(self.timestamps)


# ─── Pipeline: uses actual WorldFusion code path ────────────────

class OfflinePipelineV3:
    def __init__(self):
        self._load_model()
        self._load_tracker()
        self._load_mtr()
        self.tracked_trajectories = {}
        self._pre_processor = None

    def _load_model(self):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        from opencood.data_utils.post_processor import build_postprocessor
        import opencood.tools.train_utils as train_utils

        wf_dir = os.environ.get(
            'WF_CKPT_DIR',
            os.path.join(REPO, 'models/worldfusion_multiv2x_caronly_ndm'))
        wf_epoch = int(os.environ.get('WF_CKPT_EPOCH', '5'))
        self.hypes = load_yaml(os.path.join(wf_dir, 'config.yaml'))
        self.model = PointPillarWorldFusion(
            self.hypes['model']['args']).to(DEVICE).eval()
        _, self.model = train_utils.load_model(wf_dir, self.model, epoch=wf_epoch)

        # Post-processor for anchor-based detection
        self.post_processor = build_postprocessor(
            self.hypes['postprocess'], dataset='opv2v', train=False)
        self.anchor_box = self.post_processor.generate_anchor_box()

        print(f"[Pipeline] WorldFusion loaded, anchor shape: {self.anchor_box.shape}")

    def _get_pre_processor(self):
        if self._pre_processor is None:
            from opencood.data_utils.pre_processor import build_preprocessor
            self._pre_processor = build_preprocessor(
                self.hypes['preprocess'], train=False)
        return self._pre_processor

    def _load_tracker(self):
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
        class _FakeEgo:
            class _Loc:
                x, y = 0.0, 0.0
            def get_location(self):
                return _FakeEgo._Loc()
        self._mtr_fake_egos = [_FakeEgo()]
        self._mtr_policy_cfgs = {
            'static':     dict(budget_ms=1000.0, enable_amortization=False, enable_risk_budget=False),
            'amort_only': dict(budget_ms=1000.0, enable_amortization=True,  enable_risk_budget=False),
            'risk_only':  dict(budget_ms=30.0,   enable_amortization=False, enable_risk_budget=True),
            'adaptive':   dict(budget_ms=30.0,   enable_amortization=True,  enable_risk_budget=True),
        }
        self._active_predictor = None
        self._active_policy = None
        print("[Pipeline] MTR policies registered (lazy load).")

    def set_active_predictor(self, policy):
        from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor
        if self._active_policy == policy and self._active_predictor is not None:
            return self._active_predictor
        if self._active_predictor is not None:
            del self._active_predictor
            self._active_predictor = None
            torch.cuda.empty_cache()
        cfg = self._mtr_policy_cfgs[policy]
        self._active_predictor = MTREdgePredictor(
            cmp_root=CMP_ABS, mtr_checkpoint=MTR_CKPT, device=DEVICE,
            num_output_steps=25,
            ego_vehicles=self._mtr_fake_egos,
            **cfg,
        )
        self._active_policy = policy
        print(f"[Pipeline] MTR predictor loaded: {policy}")
        return self._active_predictor

    # ─── Vehicle-side: raw LiDAR → pre-backbone features ──────

    def voxelize_agents(self, scene: MultiV2XScene, tick_idx: int,
                        agent_names: List[str]
                        ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Vehicle-side encoding: voxelize + pillar_vfe + scatter.

        Returns:
          spatial_features: [N, C*Z, H, W] pre-backbone features
          record_len: [1] tensor with value N
          pairwise_t: [1, max_cav, max_cav, 4, 4] transforms
        """
        from opencood.utils.pcd_utils import pcd_to_np, mask_ego_points, \
            mask_points_by_range, shuffle_points
        from opencood.utils.transformation_utils import x1_to_x2
        from opencood.utils import box_utils

        pre_proc = self._get_pre_processor()
        ego_pose = scene.agent_lidar_pose(scene.ego_cav, tick_idx)
        cav_range = self.hypes['preprocess']['cav_lidar_range']

        # PCD files are in agent-local frame (same as OPV2V).
        # Match the dataset's proj_first=False path: mask in agent-local
        # frame, then let fusion handle spatial alignment via pairwise_t.
        processed_list = []
        t_matrices = []
        proj_first = self.hypes.get('fusion', {}).get('args', {}).get('proj_first', False)

        for agent_name in agent_names:
            pcd_path = scene.agent_pcd_path(agent_name, tick_idx)
            if not os.path.exists(pcd_path):
                continue
            lidar_np = pcd_to_np(pcd_path)
            lidar_np = shuffle_points(lidar_np)
            lidar_np = mask_ego_points(lidar_np)

            agent_pose = scene.agent_lidar_pose(agent_name, tick_idx)
            T = x1_to_x2(agent_pose, ego_pose)
            t_matrices.append(T)

            if proj_first:
                # Project to ego frame then mask in ego frame
                lidar_np[:, :3] = box_utils.project_points_by_matrix_torch(
                    lidar_np[:, :3], T)
                lidar_np = mask_points_by_range(lidar_np, cav_range)
            else:
                # Mask in agent-local frame (matches dataset collation)
                lidar_np = mask_points_by_range(lidar_np, cav_range)

            processed = pre_proc.preprocess(lidar_np)
            processed_list.append(processed)

        N = len(processed_list)
        if N == 0:
            sf = torch.zeros(1, 64, 200, 200, device=DEVICE)
            rl = torch.tensor([1], dtype=torch.int64, device=DEVICE)
            pw = torch.eye(4, device=DEVICE).reshape(1,1,1,4,4)
            return sf, rl, pw

        # Pairwise transforms
        max_cav = max(N, 7)
        pw = np.eye(4, dtype=np.float32).reshape(1,1,4,4)
        pw = np.tile(pw, (max_cav, max_cav, 1, 1))
        if proj_first:
            # Points already in ego frame, identity transforms
            pass
        else:
            # Fusion will project features using these transforms
            for i in range(N):
                for j in range(N):
                    if i != j:
                        pw[i, j] = np.linalg.inv(t_matrices[j]) @ t_matrices[i]
        pairwise_t = torch.from_numpy(pw).unsqueeze(0).to(DEVICE)

        # Collate + pillar_vfe + scatter
        merged = {}
        for p in processed_list:
            for k, v in p.items():
                if k not in merged:
                    merged[k] = []
                if isinstance(v, list):
                    merged[k] += v
                else:
                    merged[k].append(v)
        collated = pre_proc.collate_batch(merged)
        for k, v in list(collated.items()):
            if hasattr(v, 'to'):
                collated[k] = v.to(DEVICE)

        record_len = torch.tensor([N], dtype=torch.int64, device=DEVICE)
        sensor = self.model.sensor
        batch = {
            'voxel_features': collated['voxel_features'],
            'voxel_coords': collated['voxel_coords'],
            'voxel_num_points': collated['voxel_num_points'],
            'record_len': record_len,
        }
        with torch.no_grad():
            batch = sensor.scatter(sensor.pillar_vfe(batch))

        return batch['spatial_features'], record_len, pairwise_t

    # ─── Input contribution filter ──────────────────────────────

    def contribution_filter(self, spatial_features, pairwise_t, record_len, k_cav):
        """Score per-CAV contribution by detection confidence in the
        zone. RSU (index 0) is always included. Rank CAVs in indices
        1..N-1, keep top k_cav CAVs. Total kept = 1 + min(k_cav, N-1).

        Returns filtered (spatial_features, pairwise_t, record_len, keep_idx).
        """
        N = spatial_features.shape[0]
        n_cav = N - 1
        if k_cav >= n_cav:
            return spatial_features, pairwise_t, record_len, list(range(N))

        sensor = self.model.sensor
        scores = []
        for i in range(1, N):  # skip RSU (index 0)
            bd = {'spatial_features': spatial_features[i:i+1]}
            bd = sensor.backbone(bd)
            bev2d = bd['spatial_features_2d']
            if sensor.shrink_flag:
                bev2d = sensor.shrink_conv(bev2d)
            psm = self.model.cls_head(bev2d).sigmoid()
            # Conflict zone: weight central region (RSU anchor)
            H, W = psm.shape[2], psm.shape[3]
            yy, xx = torch.meshgrid(
                torch.arange(H, device=DEVICE, dtype=torch.float32),
                torch.arange(W, device=DEVICE, dtype=torch.float32),
                indexing='ij')
            cy, cx = H / 2, W / 2
            r = min(H, W) / 3
            dist = torch.sqrt((yy - cy) ** 2 + (xx - cx) ** 2)
            mask = torch.clamp(1.0 - dist / r, 0, 1)
            contrib = (psm.max(dim=1)[0] * mask).sum().item()
            scores.append((contrib, i))

        scores.sort(reverse=True)
        keep_idx = sorted([0] + [i for _, i in scores[:k_cav]])
        idx_tensor = torch.tensor(keep_idx, dtype=torch.long, device=DEVICE)

        sf_filtered = spatial_features[idx_tensor]
        rl_filtered = torch.tensor([len(keep_idx)], dtype=torch.int64, device=DEVICE)
        pw_filtered = pairwise_t[:, idx_tensor][:, :, idx_tensor]

        return sf_filtered, pw_filtered, rl_filtered, keep_idx

    def contribution_filter_oracle(self, spatial_features, pairwise_t, record_len,
                                    k_cav, ego_pose, gt_inrange, occluded_set,
                                    target='tp_total'):
        """Greedy forward-selection oracle over CAV subsets.

        Starting with the RSU, at each step picks the CAV that most
        increases the chosen target metric when added to the fused set.

        target ∈ {'tp_total', 'tp_occluded'}. The oracle is the (1 - 1/e)
        greedy bound on the true optimal subset under submodularity.

        Expensive: runs fusion + detection (N - selected) times per step.
        Use only offline for reference comparison.
        """
        N = spatial_features.shape[0]
        n_cav = N - 1
        if k_cav >= n_cav:
            return spatial_features, pairwise_t, record_len, list(range(N))

        selected = [0]  # RSU always pinned
        remaining = set(range(1, N))

        def score_subset(idx_list):
            idx_t = torch.tensor(sorted(idx_list), dtype=torch.long, device=DEVICE)
            sf = spatial_features[idx_t]
            pw = pairwise_t[:, idx_t][:, :, idx_t]
            rl = torch.tensor([len(idx_list)], dtype=torch.int64, device=DEVICE)
            fused, pd_ = self.run_edge_fusion(sf, pw, rl)
            dets, _info = self.run_detection(pd_, ego_pose)
            tp, _fp, _fn, matches = self.match_dets_to_gt(dets, gt_inrange)
            tp_occ = sum(1 for gid in matches if gid in occluded_set)
            return tp, tp_occ

        while len(selected) - 1 < k_cav and remaining:
            base_total, base_occ = score_subset(selected)
            best_gain = -1
            best_cav = None
            for cav in list(remaining):
                trial = selected + [cav]
                tp, tp_occ = score_subset(trial)
                gain = (tp_occ - base_occ) if target == 'tp_occluded' \
                       else (tp - base_total)
                if gain > best_gain:
                    best_gain = gain
                    best_cav = cav
            if best_cav is None or best_gain < 0:
                # No CAV improves the score; stop early
                break
            selected.append(best_cav)
            remaining.discard(best_cav)

        keep_idx = sorted(selected)
        idx_tensor = torch.tensor(keep_idx, dtype=torch.long, device=DEVICE)
        sf_filtered = spatial_features[idx_tensor]
        rl_filtered = torch.tensor([len(keep_idx)], dtype=torch.int64, device=DEVICE)
        pw_filtered = pairwise_t[:, idx_tensor][:, :, idx_tensor]
        return sf_filtered, pw_filtered, rl_filtered, keep_idx

    @staticmethod
    def _compute_shadow_mask(observer_xy, blockers, grid_res=2.0, grid_half=70.4,
                              min_dist=3.0):
        """Compute a BEV shadow mask for an observer at observer_xy in the
        RSU-local frame, given a list of blocker objects (each with x, y, radius).

        Returns a 2D grid where True = cell is occluded from observer's view
        by at least one blocker. Grid is centered at the RSU origin, with
        side 2 * grid_half meters, cells of grid_res meters each.

        Angular binning: for each observer, each blocker occupies an angular
        arc; cells in that arc at range > blocker_range are shadowed.
        """
        side = int(2 * grid_half / grid_res)
        shadow = np.zeros((side, side), dtype=bool)
        if not blockers:
            return shadow

        # Precompute cell centers in RSU-local frame
        ij = np.arange(side)
        cell_coords = (ij - side / 2 + 0.5) * grid_res  # center of each cell
        xg, yg = np.meshgrid(cell_coords, cell_coords, indexing='xy')
        # Vector from observer to each cell
        dx = xg - observer_xy[0]
        dy = yg - observer_xy[1]
        cell_range = np.sqrt(dx * dx + dy * dy)
        cell_ang = np.arctan2(dy, dx)

        for blk in blockers:
            bx, by = blk['x'], blk['y']
            r_blk = blk.get('radius', 1.2)
            odx = bx - observer_xy[0]
            ody = by - observer_xy[1]
            blk_range = math.sqrt(odx * odx + ody * ody)
            if blk_range < min_dist:
                continue
            blk_ang = math.atan2(ody, odx)
            half_width = math.atan2(r_blk, blk_range)
            # Angular difference wrapped to [-pi, pi]
            da = (cell_ang - blk_ang + math.pi) % (2 * math.pi) - math.pi
            mask_arc = np.abs(da) <= half_width
            mask_beyond = cell_range > blk_range + r_blk
            shadow |= (mask_arc & mask_beyond)
        return shadow

    def contribution_filter_causal(self, spatial_features, pairwise_t, record_len,
                                    k_cav, scene, tick, agents,
                                    tracked_trajectories=None,
                                    alpha_geom=1.0, alpha_unc=1.0):
        """Causal (present-state only) CAV selector combining three signals:

        1. Geometric occlusion complementarity: fraction of RSU shadow
           cells (projected from previous-tick tracked objects as
           causal blocker proxies) that are visible from this CAV.
        2. Marginal track-uncertainty reduction: sum over tracks of
           (1 / track_age) if the CAV sees the track (track not in
           CAV's shadow).
        3. Distance gate: CAVs > 80m from the RSU anchor are dropped.

        Uses tracked_trajectories from previous tick (causal). If the
        first tick has no tracks, falls back to uniform score (random).

        Cheap: O(N * G^2) grid operations per tick where G ~ 70.
        Target: under 10ms for N<=30.
        """
        N = spatial_features.shape[0]
        n_cav = N - 1
        if k_cav >= n_cav:
            return spatial_features, pairwise_t, record_len, list(range(N))

        # Agent poses relative to RSU (RSU is agents[0])
        ego_pose = scene.agent_lidar_pose(agents[0], tick)
        yaw = math.radians(ego_pose[4])
        c, s = math.cos(yaw), math.sin(yaw)

        cav_xy = []
        for i in range(1, N):
            ap = scene.agent_lidar_pose(agents[i], tick)
            wx, wy = ap[0] - ego_pose[0], ap[1] - ego_pose[1]
            lx = c * wx + s * wy
            ly = -s * wx + c * wy
            cav_xy.append((lx, ly))

        # Blockers from previous-tick tracked trajectories (causal proxy)
        # Trajectories are in ego-local coordinates already (set by
        # build_trajectories which uses tracker output).
        blockers = []
        tracks_with_age = []  # for uncertainty scoring
        if tracked_trajectories:
            for tid, ot in tracked_trajectories.items():
                if not ot.trajectory:
                    continue
                tf = ot.trajectory[0]
                tx, ty = tf.location.x, tf.location.y
                blockers.append({'x': tx, 'y': ty, 'radius': 1.2})
                age = len(ot.trajectory)
                tracks_with_age.append({'x': tx, 'y': ty, 'age': age})

        # RSU shadow mask
        rsu_shadow = self._compute_shadow_mask((0.0, 0.0), blockers)
        n_rsu_shadow = rsu_shadow.sum()

        # Per-CAV scores
        scores = []
        for ci, (lx, ly) in enumerate(cav_xy):
            cav_idx = ci + 1
            # Distance gate
            cav_dist = math.sqrt(lx * lx + ly * ly)
            if cav_dist > 80.0 or cav_dist < 3.0:
                scores.append((-1.0, cav_idx))
                continue
            # Geometric complementarity: CAV-shadow fraction of RSU shadow
            if n_rsu_shadow > 0:
                cav_shadow = self._compute_shadow_mask((lx, ly), blockers)
                # CAV covers a cell iff the cell is in RSU shadow AND not in CAV shadow
                covered = rsu_shadow & ~cav_shadow
                geom_score = float(covered.sum()) / float(n_rsu_shadow)
            else:
                geom_score = 0.0
            # Uncertainty score: sum over tracks of (1/age) for tracks
            # the CAV can see (track not occluded by any blocker from CAV)
            unc_score = 0.0
            for tr in tracks_with_age:
                trx, try_ = tr['x'], tr['y']
                dxx = trx - lx
                dyy = try_ - ly
                tr_range = math.sqrt(dxx * dxx + dyy * dyy)
                if tr_range < 1.0 or tr_range > 80.0:
                    continue
                tr_ang = math.atan2(dyy, dxx)
                visible = True
                for blk in blockers:
                    if blk['x'] == trx and blk['y'] == try_:
                        continue
                    bdx = blk['x'] - lx
                    bdy = blk['y'] - ly
                    blk_range = math.sqrt(bdx * bdx + bdy * bdy)
                    if blk_range >= tr_range:
                        continue
                    blk_ang = math.atan2(bdy, bdx)
                    da = (tr_ang - blk_ang + math.pi) % (2 * math.pi) - math.pi
                    if abs(da) < math.atan2(blk['radius'], blk_range):
                        visible = False
                        break
                if visible:
                    unc_score += 1.0 / max(tr['age'], 1)
            composite = alpha_geom * geom_score + alpha_unc * (unc_score / max(len(tracks_with_age), 1))
            scores.append((composite, cav_idx))

        # Rank CAVs, take top k_cav. If all distance-gated out, fall back
        # to closest-to-RSU ranking.
        valid = [s for s in scores if s[0] >= 0]
        if len(valid) < k_cav:
            # fall back: rank all CAVs by nearness to RSU
            valid = sorted(
                [(-math.sqrt(lx * lx + ly * ly), i + 1) for i, (lx, ly) in enumerate(cav_xy)],
                key=lambda x: x[0], reverse=True)
        valid.sort(key=lambda x: x[0], reverse=True)
        keep_idx = sorted([0] + [i for _, i in valid[:k_cav]])

        idx_tensor = torch.tensor(keep_idx, dtype=torch.long, device=DEVICE)
        sf_filtered = spatial_features[idx_tensor]
        rl_filtered = torch.tensor([len(keep_idx)], dtype=torch.int64, device=DEVICE)
        pw_filtered = pairwise_t[:, idx_tensor][:, :, idx_tensor]
        return sf_filtered, pw_filtered, rl_filtered, keep_idx

    def contribution_filter_lightweight(self, spatial_features, pairwise_t, record_len, k_cav):
        """Score CAVs by L2 norm of spatial_features weighted by a center-zone mask.

        RSU (index 0) is always included. CAVs (indices 1..N-1) are scored
        by the feature norm in the RSU-anchored zone; top k_cav CAVs are
        kept. Total kept = 1 + min(k_cav, N-1).

        No neural network forwards. Cost is O(N * C * H * W) tensor ops,
        typically <5ms.

        Returns filtered (spatial_features, pairwise_t, record_len, keep_idx).
        """
        N = spatial_features.shape[0]
        n_cav = N - 1
        if k_cav >= n_cav:
            return spatial_features, pairwise_t, record_len, list(range(N))

        _, C, H, W = spatial_features.shape

        # Center-zone Gaussian mask on BEV grid (RSU is at grid center)
        yy, xx = torch.meshgrid(
            torch.arange(H, device=DEVICE, dtype=torch.float32),
            torch.arange(W, device=DEVICE, dtype=torch.float32),
            indexing='ij')
        cy, cx = H / 2.0, W / 2.0
        sigma = min(H, W) / 4.0
        zone_mask = torch.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2 * sigma ** 2))

        # Per-agent score: sum of (per-cell L2 norm across channels) * zone_mask
        feature_norm = spatial_features.norm(dim=1)  # [N, H, W]
        scores_t = (feature_norm * zone_mask.unsqueeze(0)).sum(dim=(1, 2))  # [N]
        scores = scores_t.cpu().tolist()

        # Rank CAVs only (skip index 0), take top k_cav
        cav_ranked = sorted(
            [(scores[i], i) for i in range(1, N)],
            key=lambda x: x[0], reverse=True)
        keep_idx = sorted([0] + [i for _, i in cav_ranked[:k_cav]])

        idx_tensor = torch.tensor(keep_idx, dtype=torch.long, device=DEVICE)
        sf_filtered = spatial_features[idx_tensor]
        rl_filtered = torch.tensor([len(keep_idx)], dtype=torch.int64, device=DEVICE)
        pw_filtered = pairwise_t[:, idx_tensor][:, :, idx_tensor]

        return sf_filtered, pw_filtered, rl_filtered, keep_idx

    # ─── Edge-side: exact same path as real edge manager ──────

    def run_edge_fusion(self, spatial_features, pairwise_t, record_len):
        """Run backbone + Where2Comm fusion + detection heads.

        Same code path as edge_manager_worldfusion_ab3dmot_linear_predictor
        _run_fusion(), lines 551-614.
        """
        sensor = self.model.sensor
        N = spatial_features.shape[0]

        # Per-agent: backbone + shrink + cls_head (for PSM)
        psm_single_list = []
        for i in range(N):
            bd = {'spatial_features': spatial_features[i:i+1]}
            bd = sensor.backbone(bd)
            bev2d = bd['spatial_features_2d']
            if sensor.shrink_flag:
                bev2d = sensor.shrink_conv(bev2d)
            psm_single_list.append(self.model.cls_head(bev2d))

        psm_single = torch.cat(psm_single_list, dim=0)

        # Where2Comm fusion (multi-scale, re-runs backbone internally)
        fused_feature, comm_rate, _ = self.model.fusion(
            spatial_features,
            psm_single,
            record_len,
            pairwise_t,
            backbone=sensor.backbone,
            heads=[sensor.shrink_conv if sensor.shrink_flag else None,
                   self.model.cls_head, self.model.reg_head]
        )

        if sensor.shrink_flag:
            fused_feature = sensor.shrink_conv(fused_feature)

        pred_dict = {
            'psm': self.model.cls_head(fused_feature),
            'rm': self.model.reg_head(fused_feature),
        }
        return fused_feature, pred_dict

    def run_detection(self, pred_dict, ego_pose):
        """Post-process fused output using the same path as the real edge manager."""
        from opencood.utils import box_utils

        anchor_box_np = self.post_processor.generate_anchor_box()

        # Use origin as world anchor so decoded boxes stay in ego-centric
        # coordinates within [-40, 40], matching gt_range.
        origin_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        data_dict_for_post = {
            'ego': {
                'anchor_box': torch.from_numpy(anchor_box_np).to(DEVICE),
                'transformation_matrix': torch.eye(4).to(DEVICE),
                'world_anchor': [origin_pose],
                'lidar_pose': np.array([origin_pose]),
            }
        }
        output_dict_for_post = {'ego': pred_dict}

        pred_box_corners, pred_scores = self.post_processor.post_process(
            data_dict_for_post, output_dict_for_post)

        if pred_box_corners is None or len(pred_box_corners) == 0:
            return np.zeros((0, 8), dtype=np.float32), \
                   np.zeros((0, 3), dtype=np.int64)

        box_corners_np = pred_box_corners.cpu().detach().numpy()
        scores_np = pred_scores.cpu().detach().numpy()

        boxes_7dof = box_utils.corner_to_center(box_corners_np, order='hwl')

        n_det = len(boxes_7dof)
        dets = np.zeros((n_det, 8), dtype=np.float32)
        for i in range(n_det):
            b = boxes_7dof[i]  # [x, y, z, h, w, l, yaw]
            dets[i] = [b[3], b[4], b[5], b[0], b[2], b[1], b[6], scores_np[i]]
        info = np.zeros((n_det, 3), dtype=np.int64)
        return dets, info

    def gt_objects_in_range(self, scene, tick_idx, max_range=70.4):
        """Return GT objects within max_range of ego, in ego local frame.

        Detections from run_detection are in ego local frame (rotated
        by ego yaw). GT must be in the same frame for matching.

        Also annotates each GT with its 2D extent radius for downstream
        LOS/occlusion checks.
        """
        ego_pose = scene.agent_lidar_pose(scene.ego_cav, tick_idx)
        yaw = math.radians(ego_pose[4])  # ego yaw in radians
        c, s = math.cos(yaw), math.sin(yaw)
        objects = scene.gt_objects(tick_idx)
        result = []
        for oid, obj in objects.items():
            loc = obj['location']
            # World delta from ego
            wx = loc[0] - ego_pose[0]
            wy = loc[1] - ego_pose[1]
            dist = math.sqrt(wx * wx + wy * wy)
            if dist > max_range:
                continue
            # Rotate into ego local frame
            lx = c * wx + s * wy
            ly = -s * wx + c * wy
            ext = obj.get('extent', [1.5, 0.8, 0.7])
            # Conservative blocking radius: minor semi-axis. The full
            # diagonal over-inflates occlusion since a car is not
            # omnidirectionally wide.
            radius = min(ext[0], ext[1])
            result.append({'oid': oid, 'x': lx, 'y': ly,
                           'z': loc[2] - ego_pose[2], 'dist': dist,
                           'radius': radius,
                           'half_height': ext[2]})
        return result

    @staticmethod
    def rsu_los_occluded(gt_list, self_radius_pad=0.3):
        """Return set of GT oids occluded from RSU by another GT.

        RSU is at origin (x=y=z=0 in local frame). Each GT has
        local-frame center (x, y, z) where z is negative (RSU is
        elevated). For each target, cast a 3D ray from origin to
        its center. A blocker occludes iff:
          (a) blocker is closer to origin in BEV than target,
          (b) blocker's BEV footprint intersects the ray, and
          (c) the ray's elevation at the blocker's BEV projection
              is below the blocker's top (z_blk + half_height).

        The RSU is typically 4-6m above road level; blockers are
        ~1.5m tall, so most occlusions from the RSU only arise for
        close, tall blockers or for targets directly behind them.

        O(M^2) for M GT objects. Causal, present-frame only.
        """
        occluded = set()
        M = len(gt_list)
        for i in range(M):
            tgt = gt_list[i]
            tx, ty, td = tgt['x'], tgt['y'], tgt['dist']
            tz = tgt['z']
            if td <= 1e-3:
                continue
            ux, uy = tx / td, ty / td
            for j in range(M):
                if j == i:
                    continue
                blk = gt_list[j]
                bd = blk['dist']
                if bd >= td - 0.5:
                    continue
                proj = blk['x'] * ux + blk['y'] * uy
                if proj <= 0 or proj >= td:
                    continue
                perp_x = blk['x'] - proj * ux
                perp_y = blk['y'] - proj * uy
                perp = math.sqrt(perp_x * perp_x + perp_y * perp_y)
                if perp >= blk['radius'] + self_radius_pad:
                    continue
                # 3D check: ray elevation at this projection
                # ray goes from (0,0,0) to (tx,ty,tz); at fraction t=proj/td
                ray_z = (proj / td) * tz  # negative (target is below RSU)
                blk_top = blk['z'] + blk['half_height']
                if blk_top >= ray_z:
                    occluded.add(tgt['oid'])
                    break
        return occluded

    @staticmethod
    def match_dets_to_gt(dets, gt_list, match_thresh=4.0):
        """Greedy match detections to GT. Returns tp, fp, fn."""
        if len(dets) == 0:
            return 0, 0, len(gt_list), {}
        matched_gt = set()
        matches = {}  # gt_oid -> det_idx
        for di in range(len(dets)):
            det_x, det_y = dets[di, 3], dets[di, 5]
            best_dist = match_thresh
            best_gid = None
            for g in gt_list:
                if g['oid'] in matched_gt:
                    continue
                dx = det_x - g['x']
                dy = det_y - g['y']
                d = math.sqrt(dx * dx + dy * dy)
                if d < best_dist:
                    best_dist = d
                    best_gid = g['oid']
            if best_gid is not None:
                matched_gt.add(best_gid)
                matches[best_gid] = di
        tp = len(matches)
        fp = len(dets) - tp
        fn = len(gt_list) - tp
        return tp, fp, fn, matches

    @staticmethod
    def match_tracks_to_gt(tracked_trajectories, gt_list, match_thresh=4.0):
        """Match active tracks to GT objects. Returns matches {gt_oid: track_id}."""
        matched_gt = set()
        matches = {}
        for tid, ot in tracked_trajectories.items():
            tf = ot.trajectory[0]
            tx, ty = tf.location.x, tf.location.y
            best_dist = match_thresh
            best_gid = None
            for g in gt_list:
                if g['oid'] in matched_gt:
                    continue
                dx = tx - g['x']
                dy = ty - g['y']
                d = math.sqrt(dx * dx + dy * dy)
                if d < best_dist:
                    best_dist = d
                    best_gid = g['oid']
            if best_gid is not None:
                matched_gt.add(best_gid)
                matches[best_gid] = tid
        return matches

    def oracle_detection(self, scene, tick_idx, max_range=70.4):
        """Convert GT objects to AB3DMOT detection format.

        Skips fusion entirely. Produces perfect detections for all GT
        objects within max_range of the ego, so tracking and prediction
        run on realistic object counts.
        """
        ego_pose = scene.agent_lidar_pose(scene.ego_cav, tick_idx)
        objects = scene.gt_objects(tick_idx)

        dets_list = []
        for oid, obj in objects.items():
            loc = obj['location']
            dx = loc[0] - ego_pose[0]
            dy = loc[1] - ego_pose[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > max_range:
                continue
            ext = obj['extent']
            ang = obj['angle']
            h = ext[2] * 2
            w = ext[1] * 2
            l = ext[0] * 2
            x = dx
            y = dy
            z = loc[2] - ego_pose[2]
            yaw = math.radians(ang[1])
            dets_list.append([h, w, l, x, z, y, yaw, 1.0])

        if not dets_list:
            return np.zeros((0, 8), dtype=np.float32), np.zeros((0, 3), dtype=np.int64)

        dets = np.array(dets_list, dtype=np.float32)
        info = np.zeros((len(dets), 3), dtype=np.int64)
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
                    ot.obstacle.kf_speed_mps = math.sqrt(dx*dx + dy*dy) / 0.1

        for tid in [t for t in self.tracked_trajectories if t not in active_ids]:
            del self.tracked_trajectories[tid]

    def reset(self):
        self._reset_tracker()
        self.tracked_trajectories = {}
        if self._active_predictor is not None:
            self._active_predictor.reset_cache()


# ─── Joint budget allocator ───────────────────────────────────────

class JointBudgetAllocator:
    """Adaptively choose k_cav per tick based on measured fusion cost and
    remaining prediction budget.

    Maintains a rolling estimate of fusion_ms_per_agent (calibrated from
    recent ticks). Each tick:
      1. Estimate fusion cost for k_cav candidates
      2. Pick the largest k_cav that keeps total estimated
         (filter + fusion + detection + tracking) under DEADLINE - pred_reserve
      3. After fusion/detection/tracking run, hand actual remaining budget
         to the adaptive predictor via budget_ms
    """
    def __init__(self, deadline_ms=100.0, pred_reserve_ms=30.0,
                 k_cav_candidates=None):
        self.deadline_ms = deadline_ms
        self.pred_reserve_ms = pred_reserve_ms
        self.candidates = k_cav_candidates or [0, 2, 4, 8, 12, 16]
        self._fusion_history = []  # (k_cav, fusion_ms) recent pairs
        self._filter_cost_est = 14.0  # causal filter cost estimate (ms)
        self._det_trk_est = 40.0  # detection+tracking cost estimate (ms)

    def _estimate_fusion_ms(self, k_cav):
        """Estimate fusion cost at k_cav from recent measurements."""
        if not self._fusion_history:
            # Cold start: linear model from calibration data
            return 11.0 + 8.0 * k_cav  # ~11ms base + 8ms per CAV
        # Linear regression on recent (k, ms) pairs
        ks = np.array([h[0] for h in self._fusion_history[-20:]], dtype=np.float64)
        ms = np.array([h[1] for h in self._fusion_history[-20:]], dtype=np.float64)
        if len(ks) < 3 or ks.std() < 0.5:
            return 11.0 + 8.0 * k_cav
        slope, intercept = np.polyfit(ks, ms, 1)
        return max(5.0, intercept + slope * k_cav)

    def select_k_cav(self, n_cav):
        """Pick the largest feasible k_cav."""
        budget_for_fusion = (self.deadline_ms - self.pred_reserve_ms
                             - self._filter_cost_est - self._det_trk_est)
        best_k = 0
        for k in self.candidates:
            if k > n_cav:
                continue
            est = self._estimate_fusion_ms(k)
            if est <= budget_for_fusion:
                best_k = k
        return best_k

    def update(self, k_cav, fusion_ms, filter_ms, det_trk_ms):
        """Feed back measured costs to calibrate estimator."""
        self._fusion_history.append((k_cav, fusion_ms))
        # Exponential moving average on fixed costs
        alpha = 0.3
        self._filter_cost_est = (1 - alpha) * self._filter_cost_est + alpha * filter_ms
        self._det_trk_est = (1 - alpha) * self._det_trk_est + alpha * det_trk_ms

    def remaining_pred_budget(self, actual_pre_pred_ms):
        """Actual prediction budget after measured pre-prediction stages."""
        return max(5.0, self.deadline_ms - actual_pre_pred_ms)


# ─── Profile joint controller ──────────────────────────────────

def profile_joint(pipeline, scene: MultiV2XScene, warmup_ticks=5,
                  lane_img=None, deadline_ms=None):
    """Profile the joint budget-aware controller: causal filter with
    adaptive k_cav + adaptive predictor (divergence gating + risk budget).

    Each tick:
      1. JointBudgetAllocator picks k_cav based on estimated fusion cost
      2. Causal filter selects k_cav CAVs
      3. Fusion + detection + tracking run
      4. Remaining budget passed to adaptive predictor
      5. Predictor uses divergence gating + risk scheduling within budget
    """
    if deadline_ms is None:
        deadline_ms = DEADLINE_MS
    allocator = JointBudgetAllocator(deadline_ms=deadline_ms)
    predictor = pipeline.set_active_predictor('adaptive')
    if lane_img is not None:
        predictor.set_lane_map(lane_img)
    pipeline.reset()
    n_ticks = scene.n_frames()
    results = []
    prev_trk_matches = {}
    cumulative = {'tp': 0, 'fp': 0, 'fn': 0, 'idsw': 0}

    for tick in range(n_ticks):
        agents = scene.connected_agents(tick)
        if scene.ego_cav not in agents:
            agents = [scene.ego_cav] + agents
        N = len(agents)
        n_cav = N - 1
        torch.cuda.reset_peak_memory_stats()

        # Vehicle side: voxelize
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            spatial_features, record_len, pairwise_t = \
                pipeline.voxelize_agents(scene, tick, agents)
        torch.cuda.synchronize()
        encode_ms = (time.perf_counter() - t0) * 1000

        # Joint allocator: pick k_cav
        k_cav_chosen = allocator.select_k_cav(n_cav)

        # Causal filter at chosen k_cav
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        if k_cav_chosen < n_cav:
            with torch.no_grad():
                spatial_features, pairwise_t, record_len, _ = \
                    pipeline.contribution_filter_causal(
                        spatial_features, pairwise_t, record_len, k_cav_chosen,
                        scene, tick, agents,
                        tracked_trajectories=pipeline.tracked_trajectories)
        torch.cuda.synchronize()
        filter_ms = (time.perf_counter() - t0) * 1000
        K = spatial_features.shape[0]
        n_cav_kept = K - 1

        # Fusion
        ego_pose = scene.agent_lidar_pose(scene.ego_cav, tick)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            fused, pred_dict = pipeline.run_edge_fusion(
                spatial_features, pairwise_t, record_len)
        torch.cuda.synchronize()
        fusion_ms = (time.perf_counter() - t0) * 1000
        predictor.set_fused_feature(fused)

        # Detection
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            dets, info = pipeline.run_detection(pred_dict, ego_pose)
        torch.cuda.synchronize()
        detection_ms = (time.perf_counter() - t0) * 1000
        info[:, 0] = tick

        # Tracking
        t0 = time.perf_counter()
        tracks = pipeline.run_tracking(dets, info, tick)
        pipeline.build_trajectories(tracks, tick)
        tracking_ms = (time.perf_counter() - t0) * 1000

        # Update allocator with measured costs
        det_trk_ms = detection_ms + tracking_ms
        allocator.update(n_cav_kept, fusion_ms, filter_ms, det_trk_ms)

        # Set prediction budget to whatever remains
        pre_pred_ms = filter_ms + fusion_ms + detection_ms + tracking_ms
        pred_budget = allocator.remaining_pred_budget(pre_pred_ms)
        predictor.budget_ms = pred_budget

        # Prediction (adaptive: divergence gating + risk budget)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        preds = predictor.generate_predicted_trajectories(
            pipeline.tracked_trajectories, source_tick=tick, publish_tick=tick)
        torch.cuda.synchronize()
        prediction_ms = (time.perf_counter() - t0) * 1000

        edge_ms = filter_ms + fusion_ms + detection_ms + tracking_ms + prediction_ms

        peak_alloc_mb = torch.cuda.max_memory_allocated() / (1024 ** 2)
        reserved_mb = torch.cuda.memory_reserved() / (1024 ** 2)

        # Quality metrics
        gt_inrange = pipeline.gt_objects_in_range(scene, tick)
        n_gt_inrange = len(gt_inrange)
        occluded_set = pipeline.rsu_los_occluded(gt_inrange)
        n_gt_occluded = len(occluded_set)
        tp, fp, fn, det_matches = pipeline.match_dets_to_gt(dets, gt_inrange)
        cumulative['tp'] += tp
        cumulative['fp'] += fp
        cumulative['fn'] += fn
        tp_occluded = sum(1 for gid in det_matches if gid in occluded_set)
        tp_visible = tp - tp_occluded
        fn_occluded = n_gt_occluded - tp_occluded
        fn_visible = (n_gt_inrange - n_gt_occluded) - tp_visible

        trk_matches = pipeline.match_tracks_to_gt(
            pipeline.tracked_trajectories, gt_inrange)
        idsw = 0
        for gid, tid in trk_matches.items():
            if gid in prev_trk_matches and prev_trk_matches[gid] != tid:
                idsw += 1
        cumulative['idsw'] += idsw
        prev_trk_matches = trk_matches

        # Prediction quality
        pred_ade = float('nan')
        if preds and tick + 5 < n_ticks:
            ades = []
            future_gt = pipeline.gt_objects_in_range(scene, min(tick + 5, n_ticks - 1))
            for pred_obj in preds:
                ptraj = getattr(pred_obj, 'predicted_trajectory', None)
                if ptraj is None or len(ptraj) == 0:
                    continue
                pt = ptraj[min(4, len(ptraj) - 1)]
                px, py = pt.location.x, pt.location.y
                best_d = 50.0
                for g in future_gt:
                    dx = px - g['x']
                    dy = py - g['y']
                    d = math.sqrt(dx * dx + dy * dy)
                    if d < best_d:
                        best_d = d
                if best_d < 50.0:
                    ades.append(best_d)
            if ades:
                pred_ade = sum(ades) / len(ades)

        # Predictor stats
        stats = predictor.get_stats() if hasattr(predictor, 'get_stats') else {}

        results.append({
            'tick': tick, 'timestamp': scene.timestamps[tick],
            'N': N, 'K': K,
            'k_cav_req': k_cav_chosen,
            'n_cav_kept': n_cav_kept,
            'policy': 'joint',
            'filter_on': True,
            'filter_mode': 'causal',
            'encode_ms': round(encode_ms, 2),
            'filter_ms': round(filter_ms, 2),
            'fusion_ms': round(fusion_ms, 2),
            'detection_ms': round(detection_ms, 2),
            'tracking_ms': round(tracking_ms, 2),
            'prediction_ms': round(prediction_ms, 2),
            'edge_ms': round(edge_ms, 2),
            'pred_budget_ms': round(pred_budget, 2),
            'n_cache_hits': stats.get('cache_hits', 0),
            'n_mtr_predicted': stats.get('mtr_predicted', 0),
            'n_linear_fallback': stats.get('linear_fallback', 0),
            'n_risk_pruned': stats.get('risk_pruned', 0),
            'n_dets': len(dets),
            'n_tracks': len(pipeline.tracked_trajectories),
            'n_preds': len(preds) if preds else 0,
            'n_gt': len(scene.gt_objects(tick)),
            'n_gt_inrange': n_gt_inrange,
            'n_gt_occluded': n_gt_occluded,
            'det_tp': tp, 'det_fp': fp, 'det_fn': fn,
            'det_tp_occluded': tp_occluded,
            'det_tp_visible': tp_visible,
            'det_fn_occluded': fn_occluded,
            'det_fn_visible': fn_visible,
            'trk_idsw': idsw,
            'pred_ade': round(pred_ade, 2) if not math.isnan(pred_ade) else '',
            'deadline_met': edge_ms <= deadline_ms,
            'peak_alloc_mb': round(peak_alloc_mb, 1),
            'reserved_mb': round(reserved_mb, 1),
        })

        if tick % 5 == 0:
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            print(f"  tick={tick:3d} N={N:2d} K={K:2d}(k_cav={k_cav_chosen}) "
                  f"edge={edge_ms:6.1f}ms "
                  f"filt={filter_ms:.0f} fuse={fusion_ms:.0f} pred={prediction_ms:.0f} "
                  f"budget={pred_budget:.0f} "
                  f"cache={stats.get('cache_hits',0)} mtr={stats.get('mtr_predicted',0)} "
                  f"lin={stats.get('linear_fallback',0)} "
                  f"recall={recall:.0%}")

    c = cumulative
    total_gt = c['tp'] + c['fn']
    recall = c['tp'] / total_gt if total_gt > 0 else 0
    precision = c['tp'] / (c['tp'] + c['fp']) if (c['tp'] + c['fp']) > 0 else 0
    mota = 1.0 - (c['fp'] + c['fn'] + c['idsw']) / max(total_gt, 1)
    ade_vals = [r['pred_ade'] for r in results if r['pred_ade'] != '']
    mean_ade = sum(ade_vals) / len(ade_vals) if ade_vals else float('nan')
    print(f"  Quality: recall={recall:.3f} prec={precision:.3f} "
          f"MOTA={mota:.3f} IDSW={c['idsw']} "
          f"ADE={mean_ade:.2f}m ({len(ade_vals)} ticks)")
    return results


# ─── Profile one config ─────────────────────────────────────────

def profile_config(pipeline, scene: MultiV2XScene, policy, warmup_ticks=5,
                    apply_filter=False, k_cav=None, lane_img=None,
                    filter_mode='backbone'):
    predictor = pipeline.set_active_predictor(policy)
    if lane_img is not None:
        predictor.set_lane_map(lane_img)
    pipeline.reset()
    n_ticks = scene.n_frames()
    results = []
    prev_trk_matches = {}  # gt_oid -> track_id (for ID switch counting)
    cumulative = {'tp': 0, 'fp': 0, 'fn': 0, 'idsw': 0}
    gt_history = {}  # tick -> list of gt objects in range

    for tick in range(n_ticks):
        agents = scene.connected_agents(tick)
        if scene.ego_cav not in agents:
            agents = [scene.ego_cav] + agents
        N = len(agents)

        # Reset peak-memory stats so each tick is measured independently
        torch.cuda.reset_peak_memory_stats()

        # Vehicle side: voxelize
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            spatial_features, record_len, pairwise_t = \
                pipeline.voxelize_agents(scene, tick, agents)
        torch.cuda.synchronize()
        encode_ms = (time.perf_counter() - t0) * 1000

        # Input contribution filter: RSU always pinned, select top k_cav CAVs
        # K = total kept (1 RSU + up to k_cav CAVs), n_cav_kept = CAVs selected
        n_cav = N - 1
        K = N
        n_cav_kept = n_cav
        filter_ms = 0.0
        needs_filter = (apply_filter and k_cav is not None and k_cav < n_cav
                        and filter_mode != 'none')
        # Oracle filter modes need GT + LOS info precomputed before selection
        oracle_mode = filter_mode in ('oracle_total', 'oracle_occluded')
        ego_pose = scene.agent_lidar_pose(scene.ego_cav, tick)
        if needs_filter and oracle_mode:
            _gt_inrange_pre = pipeline.gt_objects_in_range(scene, tick)
            _occluded_pre = pipeline.rsu_los_occluded(_gt_inrange_pre)
        if needs_filter:
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            with torch.no_grad():
                if filter_mode == 'lightweight':
                    spatial_features, pairwise_t, record_len, _ = \
                        pipeline.contribution_filter_lightweight(
                            spatial_features, pairwise_t, record_len, k_cav)
                elif filter_mode == 'random':
                    # Random baseline: RSU (index 0) + k_cav random CAVs
                    import random as _rnd
                    idx_pool = list(range(1, N))
                    _rnd.shuffle(idx_pool)
                    keep = sorted([0] + idx_pool[:k_cav])
                    idx_tensor = torch.tensor(keep, dtype=torch.long, device=DEVICE)
                    spatial_features = spatial_features[idx_tensor]
                    pairwise_t = pairwise_t[:, idx_tensor][:, :, idx_tensor]
                    record_len = torch.tensor([len(keep)], dtype=torch.int64, device=DEVICE)
                elif filter_mode == 'causal':
                    # Pass current pipeline tracks as causal blocker proxy
                    spatial_features, pairwise_t, record_len, _ = \
                        pipeline.contribution_filter_causal(
                            spatial_features, pairwise_t, record_len, k_cav,
                            scene, tick, agents,
                            tracked_trajectories=pipeline.tracked_trajectories)
                elif filter_mode == 'oracle_total':
                    spatial_features, pairwise_t, record_len, _ = \
                        pipeline.contribution_filter_oracle(
                            spatial_features, pairwise_t, record_len, k_cav,
                            ego_pose, _gt_inrange_pre, _occluded_pre,
                            target='tp_total')
                elif filter_mode == 'oracle_occluded':
                    spatial_features, pairwise_t, record_len, _ = \
                        pipeline.contribution_filter_oracle(
                            spatial_features, pairwise_t, record_len, k_cav,
                            ego_pose, _gt_inrange_pre, _occluded_pre,
                            target='tp_occluded')
                else:  # 'backbone'
                    spatial_features, pairwise_t, record_len, _ = \
                        pipeline.contribution_filter(
                            spatial_features, pairwise_t, record_len, k_cav)
            torch.cuda.synchronize()
            filter_ms = (time.perf_counter() - t0) * 1000
            K = spatial_features.shape[0]
            n_cav_kept = K - 1

        # Edge: fusion (backbone + Where2Comm + heads)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            fused, pred_dict = pipeline.run_edge_fusion(
                spatial_features, pairwise_t, record_len)
        torch.cuda.synchronize()
        fusion_ms = (time.perf_counter() - t0) * 1000
        predictor.set_fused_feature(fused)

        # Edge: detection (post-process fused output → 3D boxes)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            dets, info = pipeline.run_detection(pred_dict, ego_pose)
        torch.cuda.synchronize()
        detection_ms = (time.perf_counter() - t0) * 1000
        info[:, 0] = tick

        # Edge: tracking
        t0 = time.perf_counter()
        tracks = pipeline.run_tracking(dets, info, tick)
        pipeline.build_trajectories(tracks, tick)
        tracking_ms = (time.perf_counter() - t0) * 1000

        # Edge: prediction
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        preds = predictor.generate_predicted_trajectories(
            pipeline.tracked_trajectories, source_tick=tick, publish_tick=tick)
        torch.cuda.synchronize()
        prediction_ms = (time.perf_counter() - t0) * 1000

        edge_ms = filter_ms + fusion_ms + detection_ms + tracking_ms + prediction_ms

        peak_alloc_mb = torch.cuda.max_memory_allocated() / (1024 ** 2)
        reserved_mb = torch.cuda.memory_reserved() / (1024 ** 2)

        # --- Quality metrics ---
        gt_inrange = pipeline.gt_objects_in_range(scene, tick)
        gt_history[tick] = gt_inrange
        n_gt_inrange = len(gt_inrange)

        # RSU line-of-sight occlusion (causal, present-frame only)
        occluded_set = pipeline.rsu_los_occluded(gt_inrange)
        n_gt_occluded = len(occluded_set)

        # Detection quality
        tp, fp, fn, det_matches = pipeline.match_dets_to_gt(dets, gt_inrange)
        cumulative['tp'] += tp
        cumulative['fp'] += fp
        cumulative['fn'] += fn

        # TP recovered on RSU-occluded GT (cooperative complementarity)
        tp_occluded = sum(1 for gid in det_matches if gid in occluded_set)
        tp_visible = tp - tp_occluded
        fn_occluded = n_gt_occluded - tp_occluded
        fn_visible = (n_gt_inrange - n_gt_occluded) - tp_visible

        # Tracking quality (ID switches)
        trk_matches = pipeline.match_tracks_to_gt(
            pipeline.tracked_trajectories, gt_inrange)
        idsw = 0
        for gid, tid in trk_matches.items():
            if gid in prev_trk_matches and prev_trk_matches[gid] != tid:
                idsw += 1
        cumulative['idsw'] += idsw
        prev_trk_matches = trk_matches

        # Prediction quality (ADE against future GT)
        pred_ade = float('nan')
        if preds and tick + 5 < n_ticks:
            ades = []
            future_gt = pipeline.gt_objects_in_range(scene, min(tick + 5, n_ticks - 1))
            for pred_obj in (preds if preds else []):
                ptraj = getattr(pred_obj, 'predicted_trajectory', None)
                if ptraj is None or len(ptraj) == 0:
                    continue
                # Take predicted position at ~5 steps ahead
                pt = ptraj[min(4, len(ptraj) - 1)]
                px, py = pt.location.x, pt.location.y
                best_d = 50.0
                for g in future_gt:
                    dx = px - g['x']
                    dy = py - g['y']
                    d = math.sqrt(dx * dx + dy * dy)
                    if d < best_d:
                        best_d = d
                if best_d < 50.0:
                    ades.append(best_d)
            if ades:
                pred_ade = sum(ades) / len(ades)

        results.append({
            'tick': tick, 'timestamp': scene.timestamps[tick],
            'N': N, 'K': K,
            'k_cav_req': k_cav if k_cav is not None else n_cav,
            'n_cav_kept': n_cav_kept,
            'policy': policy,
            'filter_on': apply_filter,
            'filter_mode': filter_mode if apply_filter else 'none',
            'encode_ms': round(encode_ms, 2),
            'filter_ms': round(filter_ms, 2),
            'fusion_ms': round(fusion_ms, 2),
            'detection_ms': round(detection_ms, 2),
            'tracking_ms': round(tracking_ms, 2),
            'prediction_ms': round(prediction_ms, 2),
            'edge_ms': round(edge_ms, 2),
            'n_dets': len(dets),
            'n_tracks': len(pipeline.tracked_trajectories),
            'n_preds': len(preds) if preds else 0,
            'n_gt': len(scene.gt_objects(tick)),
            'n_gt_inrange': n_gt_inrange,
            'n_gt_occluded': n_gt_occluded,
            'det_tp': tp, 'det_fp': fp, 'det_fn': fn,
            'det_tp_occluded': tp_occluded,
            'det_tp_visible': tp_visible,
            'det_fn_occluded': fn_occluded,
            'det_fn_visible': fn_visible,
            'trk_idsw': idsw,
            'pred_ade': round(pred_ade, 2) if not math.isnan(pred_ade) else '',
            'deadline_met': edge_ms <= DEADLINE_MS,
            'peak_alloc_mb': round(peak_alloc_mb, 1),
            'reserved_mb': round(reserved_mb, 1),
        })

        if tick % 5 == 0:
            recall = tp / (tp + fn) if (tp + fn) > 0 else 0
            print(f"  tick={tick:3d} N={N:2d} K={K:2d} "
                  f"edge={edge_ms:6.1f}ms "
                  f"fuse={fusion_ms:.0f} "
                  f"dets={len(dets)} gt_ir={n_gt_inrange} "
                  f"tp={tp} recall={recall:.0%} "
                  f"trks={len(pipeline.tracked_trajectories)} idsw={idsw} "
                  f"preds={len(preds) if preds else 0} ade={pred_ade:.1f}" if not math.isnan(pred_ade) else
                  f"  tick={tick:3d} N={N:2d} K={K:2d} "
                  f"edge={edge_ms:6.1f}ms "
                  f"fuse={fusion_ms:.0f} "
                  f"dets={len(dets)} gt_ir={n_gt_inrange} "
                  f"tp={tp} recall={recall:.0%} "
                  f"trks={len(pipeline.tracked_trajectories)} idsw={idsw}")

    # Print cumulative quality summary
    c = cumulative
    total_gt = c['tp'] + c['fn']
    recall = c['tp'] / total_gt if total_gt > 0 else 0
    precision = c['tp'] / (c['tp'] + c['fp']) if (c['tp'] + c['fp']) > 0 else 0
    mota = 1.0 - (c['fp'] + c['fn'] + c['idsw']) / max(total_gt, 1)
    ade_vals = [r['pred_ade'] for r in results if r['pred_ade'] != '']
    mean_ade = sum(ade_vals) / len(ade_vals) if ade_vals else float('nan')
    print(f"  Quality: recall={recall:.3f} prec={precision:.3f} "
          f"MOTA={mota:.3f} IDSW={c['idsw']} "
          f"ADE={mean_ade:.2f}m ({len(ade_vals)} ticks)")

    return results


# ─── Oracle profile (GT detections → tracking → prediction) ────

def profile_oracle(pipeline, scene: MultiV2XScene, policy,
                   max_range=70.4):
    predictor = pipeline.set_active_predictor(policy)
    pipeline.reset()
    n_ticks = scene.n_frames()
    results = []

    for tick in range(n_ticks):
        N = len(scene.connected_agents(tick)) + 1
        n_gt_all = len(scene.gt_objects(tick))

        # Oracle detection: GT objects within range
        t0 = time.perf_counter()
        dets, info = pipeline.oracle_detection(scene, tick, max_range)
        detection_ms = (time.perf_counter() - t0) * 1000
        info[:, 0] = tick
        n_in_range = len(dets)

        # Tracking
        t0 = time.perf_counter()
        tracks = pipeline.run_tracking(dets, info, tick)
        pipeline.build_trajectories(tracks, tick)
        tracking_ms = (time.perf_counter() - t0) * 1000

        # Prediction
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        preds = predictor.generate_predicted_trajectories(
            pipeline.tracked_trajectories, source_tick=tick, publish_tick=tick)
        torch.cuda.synchronize()
        prediction_ms = (time.perf_counter() - t0) * 1000

        edge_ms = tracking_ms + prediction_ms

        results.append({
            'tick': tick, 'timestamp': scene.timestamps[tick],
            'N': N, 'K': N,
            'policy': policy,
            'filter_on': False,
            'encode_ms': 0.0,
            'filter_ms': 0.0,
            'fusion_ms': 0.0,
            'detection_ms': round(detection_ms, 2),
            'tracking_ms': round(tracking_ms, 2),
            'prediction_ms': round(prediction_ms, 2),
            'edge_ms': round(edge_ms, 2),
            'n_dets': n_in_range,
            'n_tracks': len(pipeline.tracked_trajectories),
            'n_preds': len(preds) if preds else 0,
            'n_gt': n_gt_all,
            'n_gt_inrange': n_in_range,
            'deadline_met': edge_ms <= DEADLINE_MS,
            'peak_alloc_mb': 0.0,
            'reserved_mb': 0.0,
        })

        if tick % 5 == 0:
            print(f"  tick={tick:3d} N={N:2d} "
                  f"gt_inrange={n_in_range:2d} "
                  f"trks={len(pipeline.tracked_trajectories):2d} "
                  f"preds={len(preds) if preds else 0:2d} "
                  f"trk={tracking_ms:.1f}ms pred={prediction_ms:.1f}ms "
                  f"edge={edge_ms:.1f}ms")

    return results


# ─── Main ────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data-root', default='/data1/Datasets/Multi-V2X')
    parser.add_argument('--town', default=None,
                        help='Single town or "all" for multi-town eval')
    parser.add_argument('--rsu', default=None,
                        help='Specific RSU name (default: first RSU per town)')
    parser.add_argument('--max-frames', type=int, default=100)
    parser.add_argument('--out', default='paper2_figures/offline_profile_v3.csv')
    parser.add_argument('--skip-oracle', action='store_true')
    parser.add_argument('--filter-mode', default='backbone',
                        choices=['backbone', 'lightweight', 'random',
                                 'causal', 'oracle_total', 'oracle_occluded',
                                 'none'],
                        help='backbone: full cls_head score (expensive ~127ms); '
                             'lightweight: feature-norm zone score (<5ms); '
                             'random: uniform random CAV selection (baseline); '
                             'causal: geometric-Voronoi + uncertainty (~10ms); '
                             'oracle_total: greedy oracle on total TP gain; '
                             'oracle_occluded: greedy oracle on RSU-occluded TP; '
                             'none: no CAV filtering, fuse all')
    parser.add_argument('--k-cav', type=int, default=None,
                        help='Number of CAVs to select beyond the RSU. '
                             'RSU is always included. If None, no filter.')
    parser.add_argument('--sweep-k-cav', default=None,
                        help='Comma-separated list of k_cav values to sweep '
                             '(e.g. "0,2,4,8"). Overrides --k-cav.')
    parser.add_argument('--joint', action='store_true',
                        help='Run joint budget-aware controller: causal filter '
                             'with adaptive k_cav + adaptive predictor. '
                             'Runs alongside static baseline for comparison.')
    args = parser.parse_args()

    # Discover towns
    data_root = Path(args.data_root)
    if args.town and args.town != 'all':
        towns = [args.town]
    else:
        towns = sorted([
            d.name for d in data_root.iterdir()
            if d.is_dir() and d.name.startswith('Town')
        ])

    print("=" * 70)
    print("Offline Profiler V3: Multi-V2X Real Data (RSU-centric)")
    print(f"GPU: {torch.cuda.get_device_name(0)}, Deadline: {DEADLINE_MS}ms")
    print(f"Towns: {towns}")
    print("=" * 70)

    LANE_MAP_DIR = os.path.join(REPO, 'models/lane_maps')

    pipeline = OfflinePipelineV3()
    all_results = []

    for town in towns:
        scene = MultiV2XScene(args.data_root, town, rsu_name=args.rsu,
                              max_frames=args.max_frames)

        # Load lane map for this RSU zone
        lane_map_path = os.path.join(LANE_MAP_DIR, f'{town}_{scene.ego_cav}.png')
        if os.path.exists(lane_map_path):
            from PIL import Image
            lane_img = np.array(Image.open(lane_map_path))
            # Set on all predictors that get loaded
            print(f"Lane map loaded: {lane_map_path} ({lane_img.shape})")
        else:
            lane_img = None
            print(f"No lane map found at {lane_map_path}")

        # Warmup
        print(f"\nWarmup ({town}/{scene.ego_cav})...")
        agents = scene.connected_agents(0)
        if scene.ego_cav not in agents:
            agents = [scene.ego_cav] + agents
        agents = agents[:4]
        with torch.no_grad():
            sf, rl, pw = pipeline.voxelize_agents(scene, 0, agents)
            for _ in range(3):
                pipeline.run_edge_fusion(sf, pw, rl)
        print("Done.\n")

        # Build list of (config_label, apply_filter, k_cav) configs to run
        configs = []
        if args.sweep_k_cav:
            k_vals = [int(x) for x in args.sweep_k_cav.split(',')]
            # Always include full-static baseline (RSU + all CAVs)
            configs.append(('static_all', False, None))
            for kv in k_vals:
                configs.append((f'k_cav={kv}_{args.filter_mode}', True, kv))
        elif args.k_cav is not None:
            configs.append((f'k_cav={args.k_cav}_{args.filter_mode}', True, args.k_cav))
        else:
            configs.append(('static_all', False, None))

        for label, do_filter, kv in configs:
            print(f"\n--- {town}/{scene.ego_cav}: {label} ---")
            rows = profile_config(pipeline, scene, 'static',
                                  apply_filter=do_filter, k_cav=kv,
                                  lane_img=lane_img,
                                  filter_mode=args.filter_mode)
            for r in rows:
                r['config'] = label
                r['town'] = town
                r['rsu'] = scene.ego_cav
            all_results.extend(rows)

        # Joint controller profile (runs separately)
        if args.joint:
            print(f"\n--- {town}/{scene.ego_cav}: JOINT (causal+adaptive) ---")
            rows = profile_joint(pipeline, scene, lane_img=lane_img)
            for r in rows:
                r['config'] = 'joint'
                r['town'] = town
                r['rsu'] = scene.ego_cav
            all_results.extend(rows)

            # Also run prediction-only (no filter, adaptive predictor)
            print(f"\n--- {town}/{scene.ego_cav}: pred_only (adaptive) ---")
            rows = profile_config(pipeline, scene, 'adaptive',
                                  apply_filter=False, k_cav=None,
                                  lane_img=lane_img, filter_mode='none')
            for r in rows:
                r['config'] = 'pred_only_adaptive'
                r['town'] = town
                r['rsu'] = scene.ego_cav
            all_results.extend(rows)

            steady = [r for r in rows if r['tick'] >= 5]
            if steady:
                edge_arr = np.array([r['edge_ms'] for r in steady])
                fuse_arr = np.array([r['fusion_ms'] for r in steady])
                edge_mean = edge_arr.mean()
                edge_p95 = np.percentile(edge_arr, 95)
                edge_p99 = np.percentile(edge_arr, 99)
                deadline_miss_rate = (edge_arr > DEADLINE_MS).mean()
                tp_sum = sum(r.get('det_tp', 0) for r in steady)
                fn_sum = sum(r.get('det_fn', 0) for r in steady)
                fp_sum = sum(r.get('det_fp', 0) for r in steady)
                recall = tp_sum / (tp_sum + fn_sum) if (tp_sum + fn_sum) > 0 else 0
                prec = tp_sum / (tp_sum + fp_sum) if (tp_sum + fp_sum) > 0 else 0
                n_mean = np.mean([r['N'] for r in steady])
                kept_mean = np.mean([r['n_cav_kept'] for r in steady])
                print(f"  Steady: N={n_mean:.1f} cav_kept={kept_mean:.1f} "
                      f"recall={recall:.3f} prec={prec:.3f}")
                print(f"  Latency: fusion={fuse_arr.mean():.1f}ms "
                      f"edge mean={edge_mean:.1f}ms "
                      f"p95={edge_p95:.1f}ms p99={edge_p99:.1f}ms")
                print(f"  Deadline ({DEADLINE_MS:.0f}ms): "
                      f"miss_rate={deadline_miss_rate:.1%} "
                      f"({int(deadline_miss_rate*len(steady))}/{len(steady)} over)")

    # Save CSV
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ['config', 'town', 'rsu', 'tick', 'timestamp',
                  'N', 'K', 'k_cav_req', 'n_cav_kept',
                  'policy', 'filter_on', 'filter_mode',
                  'encode_ms', 'filter_ms', 'fusion_ms', 'detection_ms',
                  'tracking_ms', 'prediction_ms', 'edge_ms',
                  'n_dets', 'n_tracks', 'n_preds', 'n_gt', 'n_gt_inrange',
                  'n_gt_occluded',
                  'det_tp', 'det_fp', 'det_fn',
                  'det_tp_occluded', 'det_tp_visible',
                  'det_fn_occluded', 'det_fn_visible',
                  'trk_idsw', 'pred_ade',
                  'deadline_met', 'peak_alloc_mb', 'reserved_mb']
    with open(out_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        for r in all_results:
            w.writerow({k: r.get(k, '') for k in fieldnames})
    print(f"\nSaved {out_path}")


if __name__ == '__main__':
    main()
