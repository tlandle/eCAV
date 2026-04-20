"""WorldFusion edge manager with AB3DMOT tracking and adaptive MTR prediction.

Implements the unified adaptive controller from paper2_controller_unified.md:
  - Input contribution filtering (agent count K chosen by utility search)
  - Temporal amortization with time-domain divergence thresholds
  - Risk-budgeted scheduling over divergent tracks
  - Rate adaptation as fallback only (not in core contribution)

The controller searches candidate K values and selects the one that maximizes
estimated utility, using a rolling M_hat estimator with slack for pre-tracking
budgeting. The final MTR knapsack uses measured residual budget after fusion
and tracking actually run.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>

import math
import time
from collections import deque
from typing import Dict, Any, List, Tuple

import numpy as np
import carla
import torch

from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor
from .edge_manager_worldfusion_ab3dmot_linear_predictor import WorldFusionEdge


class WorldFusionAdaptiveEdge(WorldFusionEdge):
    """WorldFusion + AB3DMOT + MTR with unified adaptive controller.

    Controller variables:
      - K: number of agents kept after input contribution filtering
      - x_i: binary per-object prediction decision (handled inside MTREdgePredictor)

    Rate adaptation is fallback only, triggered when no K value yields a
    feasible estimated allocation under the deadline.
    """

    def __init__(self, world: carla.World, cfg: Dict[str, Any],
                 cav_world, carla_client: carla.Client,
                 *, world_dt: float = 0.05, **kwargs):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        # Adaptive MTR predictor (amortization + risk budget enabled)
        mtr_cfg = cfg.get('mtr_predictor', {})
        adapt_cfg = cfg.get('adaptive_controller', {})

        self.predictor = MTREdgePredictor(
            cmp_root=mtr_cfg.get('cmp_root'),
            mtr_checkpoint=mtr_cfg.get('checkpoint'),
            device=mtr_cfg.get('device', 'cuda'),
            num_output_steps=25,
            budget_ms=mtr_cfg.get('budget_ms', 50.0),
            enable_amortization=mtr_cfg.get('enable_amortization', True),
            enable_risk_budget=mtr_cfg.get('enable_risk_budget', True),
            ego_vehicles=[vm.vehicle for vm in self.vehicle_manager_list],
        )
        self.lin_pred = self.predictor

        # Controller configuration
        self.deadline_ms = adapt_cfg.get('deadline_ms', 100.0)
        # k_cav candidates: number of CAVs beyond the RSU (RSU always pinned)
        self.candidate_k_cavs = adapt_cfg.get('candidate_k_cavs', [0, 2, 4, 8, 12])
        self.overhead_ms = adapt_cfg.get('overhead_ms', 2.0)
        # Cost model coefficients calibrated from Multi-V2X profiler measurements
        self.c_filter_ms = adapt_cfg.get('c_filter_ms', 13.0)  # causal filter cost
        self.c_fusion_base_ms = adapt_cfg.get('c_fusion_base_ms', 11.0)  # RSU-only fusion
        self.c_fusion_per_cav_ms = adapt_cfg.get('c_fusion_per_cav_ms', 8.0)  # per added CAV
        self.c_detection_ms = adapt_cfg.get('c_detection_ms', 20.0)
        self.c_tracking_per_track_ms = adapt_cfg.get('c_tracking_per_track_ms', 0.7)
        self.c_base_mtr_ms = adapt_cfg.get('c_base_mtr_ms', 25.0)
        self.c_marginal_mtr_ms = adapt_cfg.get('c_marginal_mtr_ms', 1.5)
        self.pred_reserve_ms = adapt_cfg.get('pred_reserve_ms', 30.0)

        # Controller state
        self._k_cav_star = None  # selected k_cav this tick (CAVs beyond RSU)
        self._rate_k = 1     # rate multiplier (fallback only)
        self._tick_since_update = 0
        self._compute_history: deque = deque(maxlen=20)
        self._fusion_history: List[Tuple[int, float]] = []  # (k_cav, fusion_ms)
        # Rolling window for M_hat estimator
        self._tracks_per_agent: deque = deque(maxlen=20)

        print(f"[WorldFusion Adaptive Edge] causal controller "
              f"(deadline={self.deadline_ms}ms, "
              f"k_cav candidates={self.candidate_k_cavs})")

    # ── Cost model estimators ──────────────────────────────────────

    def _estimate_fusion_ms(self, k_cav: int) -> float:
        """Estimate fusion cost for RSU + k_cav CAVs.

        Uses linear regression on recent (k_cav, fusion_ms) pairs if
        available. Falls back to calibrated coefficients from Multi-V2X
        profiler measurements.
        """
        if len(self._fusion_history) >= 5:
            ks = np.array([h[0] for h in self._fusion_history[-20:]])
            ms = np.array([h[1] for h in self._fusion_history[-20:]])
            if ks.std() > 0.5:
                slope, intercept = np.polyfit(ks, ms, 1)
                return max(5.0, intercept + slope * k_cav)
        return self.c_fusion_base_ms + self.c_fusion_per_cav_ms * k_cav

    def _estimate_pre_pred_cost(self, k_cav: int) -> float:
        """Estimated filter + fusion + detection + tracking cost."""
        M_hat = self._M_hat(1 + k_cav)
        return (self.c_filter_ms
                + self._estimate_fusion_ms(k_cav)
                + self.c_detection_ms
                + self.c_tracking_per_track_ms * M_hat)

    def _M_hat(self, K_total: int) -> float:
        """Estimate tracked-object count at K_total agents with slack."""
        if not self._tracks_per_agent:
            return 4.0 * K_total + 2.0
        mean_ratio = np.mean(list(self._tracks_per_agent))
        slack = max(0.1 * mean_ratio * K_total, 2.0)
        return mean_ratio * K_total + slack

    def _select_k_cav_star(self, n_cav: int) -> int:
        """Pick the largest k_cav that leaves enough budget for prediction.

        RSU is always included. The controller picks the largest k_cav
        from the candidate set such that estimated pre-prediction cost
        leaves at least pred_reserve_ms for the adaptive predictor.
        """
        budget_for_pre_pred = self.deadline_ms - self.pred_reserve_ms
        best_k = 0
        for k in self.candidate_k_cavs:
            if k > n_cav:
                continue
            est = self._estimate_pre_pred_cost(k)
            if est <= budget_for_pre_pred:
                best_k = k
        return best_k

    # ── Main tick ─────────────────────────────────────────────────

    def run_step(self, tick: int):
        # Rate fallback: skip ticks when rate_k > 1
        if self._rate_k > 1:
            self._tick_since_update += 1
            if self._tick_since_update < self._rate_k:
                return self._update_agents(tick, None)
            self._tick_since_update = 0

        # Select k_cav* for this tick (RSU always pinned)
        n_cav = len(self.vehicle_manager_list)
        self._k_cav_star = self._select_k_cav_star(n_cav) if n_cav > 0 else 0

        t0 = time.perf_counter()
        result = super().run_step(tick)
        compute_ms = (time.perf_counter() - t0) * 1000
        self._compute_history.append(compute_ms)

        # Update M_hat estimator
        k_total = 1 + (self._k_cav_star if self._k_cav_star is not None else n_cav)
        if k_total > 0:
            n_tracks = len(getattr(self, 'tracked_trajectories', {}))
            self._tracks_per_agent.append(n_tracks / max(k_total, 1))

        # Set adaptive predictor budget to remaining time after pre-prediction stages
        pre_pred_ms = compute_ms  # rough; actual breakdown measured inside _run_fusion
        remaining = max(5.0, self.deadline_ms - pre_pred_ms)
        self.predictor.budget_ms = remaining

        # Report to profiler
        if hasattr(self, 'profiler') and hasattr(self.profiler, '_frames') and self.profiler._frames:
            frame = self.profiler._frames[-1]
            frame.adaptive_rate_k = self._rate_k
            frame.adaptive_k_cav = self._k_cav_star
            frame.adaptive_compute_ms = compute_ms
            frame.adaptive_deadline_met = compute_ms <= (self.deadline_ms * self._rate_k)
            if hasattr(self.predictor, 'get_stats'):
                stats = self.predictor.get_stats()
                frame.adaptive_n_cached = stats.get('cache_hits', 0)
                frame.adaptive_n_risk_selected = stats.get('mtr_predicted', 0)
                frame.adaptive_n_linear = stats.get('linear_fallback', 0)
                frame.adaptive_n_divergent = (stats.get('mtr_predicted', 0)
                                              + stats.get('risk_pruned', 0))

        # Rate adaptation fallback
        if len(self._compute_history) >= 5:
            p95 = np.percentile(list(self._compute_history), 95)
            old_k = self._rate_k
            if p95 <= self.deadline_ms:
                self._rate_k = 1
            elif p95 <= self.deadline_ms * 2:
                self._rate_k = 2
            elif p95 <= self.deadline_ms * 4:
                self._rate_k = 4
            elif p95 <= self.deadline_ms * 5:
                self._rate_k = 5
            else:
                self._rate_k = 10
            if self._rate_k != old_k:
                print(f"[Adaptive] Rate fallback: k={old_k}->{self._rate_k} "
                      f"(p95={p95:.0f}ms, deadline={self.deadline_ms:.0f}ms)")

        return result

    def _run_fusion(self, spatial_features, pairwise_t_matrix, record_len):
        """Fusion with causal contributor filter. RSU always pinned (index 0)."""
        sensor = self.model.sensor
        N = spatial_features.shape[0]
        n_cav = N - 1
        device = spatial_features.device

        # Apply causal filter if k_cav_star < n_cav
        t_filt = time.perf_counter()
        if (self._k_cav_star is not None and self._k_cav_star < n_cav
                and n_cav > 0):
            selected_idx = self._causal_filter(
                spatial_features, n_cav, self._k_cav_star)
            idx_tensor = torch.tensor(selected_idx, dtype=torch.long,
                                      device=device)
            spatial_features = spatial_features[idx_tensor]
            pairwise_t_matrix = pairwise_t_matrix[:, idx_tensor][:, :, idx_tensor]
            record_len = torch.tensor([len(selected_idx)], dtype=torch.int64,
                                      device=device)
            N = len(selected_idx)
        filter_ms = (time.perf_counter() - t_filt) * 1000

        # Run backbone + shrink on each agent
        psm_single_list = []
        for i in range(N):
            bd = {'spatial_features': spatial_features[i:i + 1]}
            bd = sensor.backbone(bd)
            bev2d = bd['spatial_features_2d']
            if sensor.shrink_flag:
                bev2d = sensor.shrink_conv(bev2d)
            psm_single_list.append(self.model.cls_head(bev2d))

        psm_single = torch.cat(psm_single_list, dim=0)

        # Where2Comm attention fusion
        fused_feature, _, _ = self.model.fusion(
            spatial_features, psm_single, record_len, pairwise_t_matrix,
            backbone=sensor.backbone,
            heads=[sensor.shrink_conv if sensor.shrink_flag else None,
                   self.model.cls_head, self.model.reg_head])

        if sensor.shrink_flag:
            fused_feature = sensor.shrink_conv(fused_feature)

        pred_dict = {
            'psm': self.model.cls_head(fused_feature),
            'rm': self.model.reg_head(fused_feature),
        }

        # Update fusion cost estimator
        fusion_ms = (time.perf_counter() - t_filt) * 1000 - filter_ms
        self._fusion_history.append((N - 1, fusion_ms))
        if len(self._fusion_history) > 50:
            self._fusion_history = self._fusion_history[-30:]

        return fused_feature, pred_dict

    # ── Causal contributor filter ─────────────────────────────────

    @staticmethod
    def _compute_shadow_mask(observer_xy, blockers, grid_res=2.0,
                              grid_half=70.4, min_dist=3.0):
        """BEV shadow mask for an observer using angular binning."""
        side = int(2 * grid_half / grid_res)
        shadow = np.zeros((side, side), dtype=bool)
        if not blockers:
            return shadow
        ij = np.arange(side)
        cell_coords = (ij - side / 2 + 0.5) * grid_res
        xg, yg = np.meshgrid(cell_coords, cell_coords, indexing='xy')
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
            da = (cell_ang - blk_ang + math.pi) % (2 * math.pi) - math.pi
            mask_arc = np.abs(da) <= half_width
            mask_beyond = cell_range > blk_range + r_blk
            shadow |= (mask_arc & mask_beyond)
        return shadow

    def _causal_filter(self, spatial_features, n_cav, k_cav) -> List[int]:
        """Causal (present-state) CAV selector: geometric occlusion
        complementarity + track-uncertainty reduction.

        RSU (index 0) always included. Ranks CAVs in indices 1..N-1
        by how much they see into the RSU's shadow regions (projected
        from previous-tick tracked objects) and how much they can
        reduce track uncertainty.

        Returns sorted list of selected indices (always includes 0).
        """
        if k_cav >= n_cav:
            return list(range(1 + n_cav))

        # Build blocker list from current tracked trajectories
        tracked = getattr(self, 'tracked_trajectories', {})
        blockers = []
        tracks_with_age = []
        for tid, ot in tracked.items():
            if not ot.trajectory:
                continue
            tf = ot.trajectory[0]
            blockers.append({'x': tf.location.x, 'y': tf.location.y,
                             'radius': 1.2})
            tracks_with_age.append({'x': tf.location.x, 'y': tf.location.y,
                                    'age': len(ot.trajectory)})

        # Get agent poses. RSU is index 0 in the feature stack.
        # Vehicle managers provide localizer positions.
        rsu_managers = getattr(self, 'rsu_manager_list', [])
        veh_managers = getattr(self, 'vehicle_manager_list', [])

        # RSU position (world frame, index 0)
        rsu_pos = np.zeros(2)
        if rsu_managers:
            rsu_loc = rsu_managers[0].localizer.get_ego_pos()
            if rsu_loc is not None:
                rsu_pos = np.array([rsu_loc.location.x,
                                    rsu_loc.location.y])

        # CAV positions relative to RSU (for scoring)
        cav_xy = []
        for vm in veh_managers:
            veh_loc = vm.localizer.get_ego_pos()
            if veh_loc is not None:
                cav_xy.append((veh_loc.location.x - rsu_pos[0],
                               veh_loc.location.y - rsu_pos[1]))
            else:
                cav_xy.append((0.0, 0.0))

        # RSU shadow mask
        rsu_shadow = self._compute_shadow_mask((0.0, 0.0), blockers)
        n_rsu_shadow = rsu_shadow.sum()

        # Score each CAV
        scores = []
        for ci, (lx, ly) in enumerate(cav_xy):
            cav_dist = math.sqrt(lx * lx + ly * ly)
            if cav_dist > 80.0 or cav_dist < 1.0:
                scores.append((-1.0, ci + 1))
                continue

            # Geometric complementarity
            if n_rsu_shadow > 0:
                cav_shadow = self._compute_shadow_mask((lx, ly), blockers)
                covered = rsu_shadow & ~cav_shadow
                geom_score = float(covered.sum()) / float(n_rsu_shadow)
            else:
                geom_score = 0.0

            # Uncertainty reduction
            unc_score = 0.0
            for tr in tracks_with_age:
                dxx = tr['x'] - lx
                dyy = tr['y'] - ly
                tr_range = math.sqrt(dxx * dxx + dyy * dyy)
                if tr_range < 1.0 or tr_range > 80.0:
                    continue
                tr_ang = math.atan2(dyy, dxx)
                visible = True
                for blk in blockers:
                    if blk['x'] == tr['x'] and blk['y'] == tr['y']:
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

            n_tracks = max(len(tracks_with_age), 1)
            composite = geom_score + (unc_score / n_tracks)
            scores.append((composite, ci + 1))

        # Rank, take top k_cav
        valid = [s for s in scores if s[0] >= 0]
        if len(valid) < k_cav:
            valid = sorted(
                [(-math.sqrt(lx*lx + ly*ly), i+1)
                 for i, (lx, ly) in enumerate(cav_xy)],
                key=lambda x: x[0], reverse=True)
        valid.sort(key=lambda x: x[0], reverse=True)
        return sorted([0] + [i for _, i in valid[:k_cav]])
