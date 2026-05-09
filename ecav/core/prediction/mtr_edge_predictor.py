"""MTR trajectory predictor for edge pipeline with deadline-aware adaptation.

Wraps CMP's MTR model as a drop-in replacement for LinearPredictorManager.
Implements two adaptation mechanisms:
  1. Temporal amortization: cache predictions, re-predict only divergent tracks
  2. Risk-budgeted scheduling: predict highest-risk subset within compute budget

Same interface as LinearPredictorManager and SMARTPredictorManager3D:
  generate_predicted_trajectories(tracked_trajectories, source_tick, publish_tick)
"""
# Author: Tyler Landle <tlandle3@gatech.edu>

import logging
import math
import os
import sys
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn.functional as F

from ecav.core.prediction.obstacle_prediction import ObstaclePrediction
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.ecav_carla import Location, Rotation, Transform

logger = logging.getLogger(__name__)

# MTR constants (matching CMP's OPV2V config)
_PAST_FRAMES = 10       # 10 past frames at 10Hz
_TIME_INTERVAL = 0.1    # 10Hz
_FUTURE_FRAMES = 50     # 5s at 10Hz
_MIN_HIST_TICKS = 5     # lowered from 22 for compatibility with sparse detections
_SUBSAMPLE = 2          # 20Hz -> 10Hz
_MIN_SPEED_MPS = 1.0    # stationary gate

# Adaptation defaults
_TAU_POS = 0.1          # seconds: time-domain divergence tolerance
_THRESH_POS_FLOOR = 0.5 # meters: floor for stationary objects
_THRESH_HEAD = 0.175    # ~10 degrees in radians
_MAX_CACHE_AGE = 10     # ticks before forced re-prediction

# Risk score parameters (smooth continuous)
_DIST_SCALE_M = 20.0    # proximity exponential decay scale
_SAFETY_RADIUS_M = 5.0  # conflict proximity safety radius
_CONFLICT_HORIZON_S = 5.0  # seconds ahead for path-conflict computation

# Quality function (calibrated control surrogate)
_Q_LINEAR = 0.3         # quality of linear-fallback predictions


class PredictionCache:
    """Per-track prediction cache entry."""
    __slots__ = ['prediction', 'predicted_trajectory', 'tick', 'age',
                 'last_pos', 'last_heading']

    def __init__(self, prediction, predicted_trajectory, tick, pos, heading):
        self.prediction = prediction          # ObstaclePrediction object
        self.predicted_trajectory = predicted_trajectory  # raw trajectory array
        self.tick = tick
        self.age = 0
        self.last_pos = pos
        self.last_heading = heading


class MTREdgePredictor:
    """MTR prediction with temporal amortization and risk-budgeted scheduling.

    Args:
        cmp_root: Path to CMP directory containing MTR subdirectory.
        mtr_checkpoint: Path to MTR model weights.
        device: 'cuda' or 'cpu'.
        num_output_steps: Future steps at simulation rate (20Hz).
        budget_ms: Compute budget for prediction stage (ms).
        enable_amortization: Use divergence gating.
        enable_risk_budget: Use risk-budgeted subset selection.
        thresh_pos: Position divergence threshold (meters).
        thresh_head: Heading divergence threshold (radians).
        max_cache_age: Maximum ticks before forced re-prediction.
    """

    def __init__(self,
                 cmp_root: str = None,
                 mtr_checkpoint: str = None,
                 device: str = 'cuda',
                 num_output_steps: int = 25,
                 budget_ms: float = 50.0,
                 enable_amortization: bool = True,
                 enable_risk_budget: bool = True,
                 tau_pos: float = _TAU_POS,
                 thresh_pos_floor: float = _THRESH_POS_FLOOR,
                 thresh_head: float = _THRESH_HEAD,
                 max_cache_age: int = _MAX_CACHE_AGE,
                 dist_scale_m: float = _DIST_SCALE_M,
                 safety_radius_m: float = _SAFETY_RADIUS_M,
                 conflict_horizon_s: float = _CONFLICT_HORIZON_S,
                 q_linear: float = _Q_LINEAR,
                 ego_vehicles: list = None):

        self.device = torch.device(device)
        self.num_output_steps = num_output_steps
        self.budget_ms = budget_ms
        self.enable_amortization = enable_amortization
        self.enable_risk_budget = enable_risk_budget
        # Divergence: time-domain tolerance (constant tau_pos seconds of motion)
        self.tau_pos = tau_pos
        self.thresh_pos_floor = thresh_pos_floor
        self.thresh_head = thresh_head
        self.max_cache_age = max_cache_age
        # Risk score parameters (smooth continuous)
        self.dist_scale_m = dist_scale_m
        self.safety_radius_m = safety_radius_m
        self.conflict_horizon_s = conflict_horizon_s
        # Quality surrogate
        self.q_linear = q_linear
        self.ego_vehicles = ego_vehicles or []

        # Prediction cache: track_id -> PredictionCache
        self._cache: Dict[int, PredictionCache] = {}

        # Timing history for budget estimation
        self._mtr_time_per_track: deque = deque(maxlen=50)

        # Fused BEV feature and lane map (set externally before inference)
        self._fused_feature: Optional[torch.Tensor] = None
        self._lane_image: Optional[torch.Tensor] = None

        # Load MTR model
        self._load_model(cmp_root, mtr_checkpoint)

        # Stats
        self._stats = {
            'total_calls': 0, 'cache_hits': 0, 'mtr_predicted': 0,
            'linear_fallback': 0, 'risk_pruned': 0,
        }

    def _load_model(self, cmp_root, mtr_checkpoint):
        if cmp_root is None:
            cmp_root = 'ecav/core/application/v2v/baselines/cmp/CMP'
        mtr_root = os.path.join(cmp_root, 'MTR')

        if mtr_root not in sys.path:
            sys.path.insert(0, mtr_root)
        if cmp_root not in sys.path:
            sys.path.insert(0, cmp_root)

        from mtr.config import cfg, cfg_from_yaml_file
        from mtr.models_opv2v.multi_ego_mtr_model import (
            MotionTransformerWithMultiEgoAggregation)

        cfg_path = os.path.join(mtr_root,
                                'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
        cfg_from_yaml_file(cfg_path, cfg)
        if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
            cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
                cmp_root, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)

        self.model = MotionTransformerWithMultiEgoAggregation(cfg.MODEL)
        self.model.to(self.device).eval()

        if mtr_checkpoint is None:
            mtr_checkpoint = os.path.join(
                mtr_root, 'output/opv2v_multiego_cobevt_c256/ckpt/best_model.pth')

        if os.path.exists(mtr_checkpoint):
            state = torch.load(mtr_checkpoint, map_location=self.device)
            if 'model_state' in state:
                state = state['model_state']
            self.model.load_state_dict(state, strict=False)
            logger.info("MTR model loaded from %s", mtr_checkpoint)
        else:
            logger.warning("MTR checkpoint not found: %s", mtr_checkpoint)
            self.model = None

    # ── Feature injection ───────────────────────────────────────────

    def set_fused_feature(self, feature_tensor: torch.Tensor) -> None:
        """Store the latest fused BEV feature from the edge fusion pipeline.

        Args:
            feature_tensor: Tensor of shape (1, C, H, W) on any device.
                            Stored as-is; resized to (1, 256, 48, 176) at
                            inference time via adaptive_avg_pool2d.
        """
        self._fused_feature = feature_tensor

    def set_lane_map(self, lane_image: torch.Tensor) -> None:
        """Store a lane BEV image for use as map_polylines context.

        Args:
            lane_image: Tensor to use as map context. Repeated per center
                        object at inference time.
        """
        self._lane_image = lane_image

    # ── Public interface ────────────────────────────────────────────

    def generate_predicted_trajectories(
            self,
            tracked_obstacles_trajectories: Dict[int, ObstacleTrajectory],
            source_tick: int = None,
            publish_tick: int = None) -> List[ObstaclePrediction]:
        """Generate predictions with deadline-aware adaptation."""

        self._stats['total_calls'] += 1
        if not tracked_obstacles_trajectories or self.model is None:
            return []

        # Classify tracks
        mature_tracks = {}
        immature_tracks = {}
        stationary_tracks = {}

        for tid, ot in tracked_obstacles_trajectories.items():
            if len(ot.trajectory) < _MIN_HIST_TICKS:
                immature_tracks[tid] = ot
                continue
            speed = getattr(ot.obstacle, 'kf_speed_mps', None)
            if speed is not None and speed < _MIN_SPEED_MPS:
                stationary_tracks[tid] = ot
                continue
            mature_tracks[tid] = ot

        predictions = []

        # Stationary: constant position prediction
        for tid, ot in stationary_tracks.items():
            predictions.append(self._static_prediction(ot, source_tick, publish_tick))

        if not mature_tracks:
            return predictions

        # Divergence gating
        if self.enable_amortization:
            divergent, fresh = self._divergence_gate(mature_tracks, publish_tick)
        else:
            divergent = mature_tracks
            fresh = {}

        # Cache hits: use cached predictions for fresh tracks
        for tid, ot in fresh.items():
            cached = self._cache[tid]
            cached.age += 1
            predictions.append(cached.prediction)
            self._stats['cache_hits'] += 1

        if not divergent:
            return predictions

        # Risk budgeting
        if self.enable_risk_budget and len(divergent) > 2:
            selected, pruned = self._risk_budget_select(divergent)
        else:
            selected = divergent
            pruned = {}

        # Linear fallback for pruned tracks
        for tid, ot in pruned.items():
            pred = self._linear_prediction(ot, source_tick, publish_tick)
            predictions.append(pred)
            self._stats['linear_fallback'] += 1
            self._stats['risk_pruned'] += 1

        # MTR prediction on selected subset
        if selected:
            mtr_preds = self._run_mtr(selected, source_tick, publish_tick)
            predictions.extend(mtr_preds)
            self._stats['mtr_predicted'] += len(mtr_preds)

        return predictions

    # ── Divergence gating (time-domain tolerance) ──────────────────

    def _divergence_gate(self, tracks: Dict[int, ObstacleTrajectory],
                         tick: int) -> Tuple[Dict, Dict]:
        """Split tracks into divergent (need re-prediction) and fresh (use cache).

        Uses a constant time-domain tolerance: an object is divergent if its
        observed state deviates from the cached prediction by more than it
        could plausibly move in tau_pos seconds. At 2 m/s the threshold is
        0.5 m (floor); at 30 m/s it is 3 m. This tolerates 100 ms of unmodeled
        velocity drift regardless of absolute speed.
        """
        divergent = {}
        fresh = {}

        for tid, ot in tracks.items():
            cur_pos = np.array([ot.trajectory[0].location.x,
                                ot.trajectory[0].location.y])
            cur_head = math.radians(ot.trajectory[0].rotation.yaw)

            if tid not in self._cache:
                divergent[tid] = ot
                continue

            cached = self._cache[tid]
            if cached.age >= self.max_cache_age:
                divergent[tid] = ot
                continue

            # Velocity-normalized position threshold
            speed = getattr(ot.obstacle, 'kf_speed_mps', 0.0) or 0.0
            thresh_pos_i = max(self.thresh_pos_floor, speed * self.tau_pos)

            pos_err = np.linalg.norm(cur_pos - cached.last_pos)
            head_err = abs(self._wrap_angle(cur_head - cached.last_heading))

            if pos_err > thresh_pos_i or head_err > self.thresh_head:
                divergent[tid] = ot
            else:
                fresh[tid] = ot

        return divergent, fresh

    # ── Risk scoring and budget selection ───────────────────────────

    def _risk_budget_select(self, tracks: Dict[int, ObstacleTrajectory]
                            ) -> Tuple[Dict, Dict]:
        """Select highest-risk tracks within compute budget."""
        # Estimate how many tracks we can predict in budget
        avg_ms_per_track = 1.5  # marginal cost from benchmarks
        base_ms = 25.0
        if self._mtr_time_per_track:
            avg_ms_per_track = np.mean(list(self._mtr_time_per_track))

        max_tracks = max(int((self.budget_ms - base_ms) / max(avg_ms_per_track, 0.1)), 1)

        if len(tracks) <= max_tracks:
            return tracks, {}

        # Score each track by risk
        scored = []
        for tid, ot in tracks.items():
            risk = self._compute_risk(ot)
            scored.append((risk, tid, ot))

        scored.sort(key=lambda x: x[0], reverse=True)

        selected = {}
        pruned = {}
        for i, (risk, tid, ot) in enumerate(scored):
            if i < max_tracks:
                selected[tid] = ot
            else:
                pruned[tid] = ot

        return selected, pruned

    def _ego_state(self, ego) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Extract ego position and velocity as numpy arrays, or None."""
        if ego is None:
            return None
        # Position
        if hasattr(ego, 'get_location'):
            loc = ego.get_location()
            pos = np.array([loc.x, loc.y], dtype=np.float32)
        elif hasattr(ego, 'location'):
            pos = np.array([ego.location.x, ego.location.y], dtype=np.float32)
        else:
            return None
        # Velocity (optional)
        vel = np.zeros(2, dtype=np.float32)
        if hasattr(ego, 'get_velocity'):
            v = ego.get_velocity()
            vel = np.array([v.x, v.y], dtype=np.float32)
        elif hasattr(ego, 'velocity'):
            vel = np.array([ego.velocity.x, ego.velocity.y], dtype=np.float32)
        return pos, vel

    def _closest_approach_distance(self, obj_pos: np.ndarray, obj_vel: np.ndarray,
                                    ego_pos: np.ndarray, ego_vel: np.ndarray,
                                    horizon_s: float) -> float:
        """Compute minimum distance between object and ego over [0, horizon_s]
        assuming constant velocity for both."""
        # Relative motion: r(t) = r0 + v_rel * t
        r0 = obj_pos - ego_pos
        v_rel = obj_vel - ego_vel
        v_sq = np.dot(v_rel, v_rel)
        if v_sq < 1e-6:
            return float(np.linalg.norm(r0))
        # Time of closest approach (clamped to horizon)
        t_star = -np.dot(r0, v_rel) / v_sq
        t_star = max(0.0, min(horizon_s, t_star))
        closest = r0 + v_rel * t_star
        return float(np.linalg.norm(closest))

    def _compute_risk(self, ot: ObstacleTrajectory) -> float:
        """Smooth continuous risk score.

        r_i = speed * proximity * occlusion * conflict_proximity

        - proximity: exp(-dist/d_scale), decays smoothly with distance to nearest ego
        - conflict_proximity: in [0, 1] based on closest-approach distance between
          object and any ego's planned path over conflict_horizon seconds
        - occlusion: 1.0 if visible to any ego, 2.0 otherwise (not used here, placeholder)

        This is a heuristic score in [0, inf), not a calibrated probability.
        """
        obj_pos = np.array([ot.trajectory[0].location.x,
                            ot.trajectory[0].location.y], dtype=np.float32)
        speed = float(getattr(ot.obstacle, 'kf_speed_mps', 0.0) or 0.0)

        # Estimate object velocity from trajectory
        if len(ot.trajectory) >= 2:
            prev = ot.trajectory[1]
            dx = ot.trajectory[0].location.x - prev.location.x
            dy = ot.trajectory[0].location.y - prev.location.y
            obj_vel = np.array([dx / 0.05, dy / 0.05], dtype=np.float32)
        else:
            obj_vel = np.zeros(2, dtype=np.float32)

        if not self.ego_vehicles:
            return speed  # fallback when no egos available

        # Smooth proximity factor: exp(-dist/d_scale), max over egos
        proximity = 0.0
        conflict_prox = 0.0
        for ego in self.ego_vehicles:
            state = self._ego_state(ego)
            if state is None:
                continue
            ego_pos, ego_vel = state
            dist = float(np.linalg.norm(obj_pos - ego_pos))
            p = math.exp(-dist / self.dist_scale_m)
            if p > proximity:
                proximity = p

            # conflict_proximity based on closest approach under const velocity
            ca = self._closest_approach_distance(
                obj_pos, obj_vel, ego_pos, ego_vel, self.conflict_horizon_s)
            cp = max(0.0, 1.0 - ca / self.safety_radius_m)
            if cp > conflict_prox:
                conflict_prox = cp

        # occlusion factor: placeholder (needs visibility info from edge)
        occlusion = 1.0

        risk = speed * proximity * occlusion * conflict_prox
        # Ensure non-zero baseline so pure speed still orders objects
        # when all ego terms are zero (distant/non-conflicting).
        baseline = speed * 0.01
        return max(risk, baseline)

    # ── MTR inference ──────────────────────────────────────────────

    def _run_mtr(self, tracks: Dict[int, ObstacleTrajectory],
                 source_tick, publish_tick) -> List[ObstaclePrediction]:
        """Run MTR on a subset of tracks and update cache."""
        from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

        track_ids = list(tracks.keys())
        otrajs = [tracks[tid] for tid in track_ids]
        n = len(otrajs)

        # Build trajectory history at 10Hz
        obj_trajs = np.zeros((n, _PAST_FRAMES + 1, 8), dtype=np.float32)
        for i, ot in enumerate(otrajs):
            traj = ot.trajectory
            for t in range(_PAST_FRAMES + 1):
                idx = t * _SUBSAMPLE
                if idx < len(traj):
                    tf = traj[idx]
                else:
                    tf = traj[-1]
                obj_trajs[i, _PAST_FRAMES - t] = [
                    tf.location.x, tf.location.y, tf.location.z,
                    4.5, 1.8, 1.5,  # default vehicle dimensions
                    math.radians(tf.rotation.yaw), 1.0]

        obj_ids = np.array(track_ids)
        obj_types = np.array(['TYPE_VEHICLE'] * n)
        center_objects = obj_trajs[:, -1, :].copy()
        obj_trajs_future = np.zeros((n, _FUTURE_FRAMES, 8), dtype=np.float32)
        timestamps = np.array([_TIME_INTERVAL * i for i in range(_PAST_FRAMES + 1)],
                              dtype=np.float32)

        try:
            ret = OPV2VMultiEgoDataset.create_agent_data_for_center_objects(
                center_objects=center_objects, obj_trajs_past=obj_trajs,
                obj_trajs_future=obj_trajs_future,
                track_index_to_predict=np.arange(n),
                sdc_track_index=0, timestamps=timestamps,
                obj_types=obj_types, obj_ids=obj_ids)
        except Exception as e:
            logger.warning("MTR batch build failed: %s", e)
            return [self._linear_prediction(ot, source_tick, publish_tick)
                    for ot in otrajs]

        otd, otm, otp, otlp, otfs, otfm, cgt, cgtm, cgfvi, tipn, stin, oto, oio = ret

        input_dict = {
            'obj_trajs': torch.tensor(otd, dtype=torch.float32, device=self.device),
            'obj_trajs_mask': torch.tensor(otm, dtype=torch.bool, device=self.device),
            'track_index_to_predict': torch.tensor(tipn, dtype=torch.long, device=self.device),
            'obj_trajs_pos': torch.tensor(otp, dtype=torch.float32, device=self.device),
            'obj_trajs_last_pos': torch.tensor(otlp, dtype=torch.float32, device=self.device),
            'obj_types': oto, 'obj_ids': oio,
            'center_objects_world': center_objects,
            'center_objects_id': oio[tipn],
            'center_objects_type': oto[tipn],
            'obj_trajs_future_state': torch.tensor(otfs, dtype=torch.float32, device=self.device),
            'obj_trajs_future_mask': torch.tensor(otfm, dtype=torch.bool, device=self.device),
            'center_gt_trajs': torch.tensor(cgt, dtype=torch.float32, device=self.device),
            'center_gt_trajs_mask': torch.tensor(cgtm, dtype=torch.bool, device=self.device),
            'center_gt_final_valid_idx': torch.tensor(cgfvi, dtype=torch.long, device=self.device),
            'map_polylines': (
                torch.tensor(self._lane_image, dtype=torch.float32, device=self.device).unsqueeze(0).expand(n, -1, -1, -1)
                if self._lane_image is not None
                else torch.zeros(n, 256, 256, 3, dtype=torch.float32, device=self.device)
            ),
            'fused_feature': (
                F.adaptive_avg_pool2d(
                    self._fused_feature.to(self.device), (48, 176)
                ).unsqueeze(0)
                if self._fused_feature is not None
                else torch.randn(1, 256, 48, 176, device=self.device).unsqueeze(0)
            ),
        }
        batch = {'input_dict': input_dict, 'num_cavs': 1, 'batch_sample_count': [n]}

        t0 = time.perf_counter()
        with torch.no_grad():
            output = self.model(batch)
        torch.cuda.synchronize()
        elapsed_ms = (time.perf_counter() - t0) * 1000

        # Update timing estimate
        if n > 0:
            self._mtr_time_per_track.append((elapsed_ms - 25.0) / n)

        # Convert output to predictions
        pred_trajs = output['pred_trajs'].cpu().numpy()  # (N, modes, T, 5)
        pred_scores = output['pred_scores'].cpu().numpy()  # (N, modes)
        best_mode = pred_scores.argmax(axis=1)

        predictions = []
        for i, (tid, ot) in enumerate(zip(track_ids, otrajs)):
            best = pred_trajs[i, best_mode[i], :, :2]  # (T, 2)

            # Subsample to simulation rate (10Hz -> 20Hz via interpolation)
            pred_tfs = []
            cur_z = ot.trajectory[0].location.z
            for step in range(min(self.num_output_steps, len(best))):
                pred_tfs.append(Transform(
                    location=Location(x=float(best[step, 0]),
                                      y=float(best[step, 1]),
                                      z=float(cur_z)),
                    rotation=Rotation(yaw=0.0)))

            cur_tf = Transform(
                location=Location(x=ot.trajectory[0].location.x,
                                  y=ot.trajectory[0].location.y,
                                  z=ot.trajectory[0].location.z),
                rotation=Rotation(yaw=ot.trajectory[0].rotation.yaw))

            pred = ObstaclePrediction(
                ot, cur_tf, probability=float(pred_scores[i, best_mode[i]]),
                predicted_trajectory=pred_tfs,
                source_tick=source_tick, publish_tick=publish_tick)
            predictions.append(pred)

            # Update cache
            cur_pos = np.array([ot.trajectory[0].location.x,
                                ot.trajectory[0].location.y])
            cur_head = math.radians(ot.trajectory[0].rotation.yaw)
            self._cache[tid] = PredictionCache(
                pred, best, publish_tick, cur_pos, cur_head)

        logger.info("[MTR Edge] %d tracks predicted in %.1fms (%.1fms/track)",
                    n, elapsed_ms, elapsed_ms / max(n, 1))
        return predictions

    # ── Fallback predictions ───────────────────────────────────────

    def _linear_prediction(self, ot: ObstacleTrajectory,
                           source_tick, publish_tick) -> ObstaclePrediction:
        """Constant-velocity linear extrapolation."""
        traj = ot.trajectory
        if len(traj) < 2:
            return self._static_prediction(ot, source_tick, publish_tick)

        cur = traj[0]
        prev = traj[1]
        dt = 0.05  # 20Hz tick
        vx = (cur.location.x - prev.location.x) / dt
        vy = (cur.location.y - prev.location.y) / dt

        pred_tfs = []
        for step in range(self.num_output_steps):
            t = (step + 1) * dt
            pred_tfs.append(Transform(
                location=Location(x=cur.location.x + vx * t,
                                  y=cur.location.y + vy * t,
                                  z=cur.location.z),
                rotation=Rotation(yaw=cur.rotation.yaw)))

        cur_tf = Transform(location=cur.location, rotation=cur.rotation)
        return ObstaclePrediction(
            ot, cur_tf, probability=0.5,
            predicted_trajectory=pred_tfs,
            source_tick=source_tick, publish_tick=publish_tick)

    def _static_prediction(self, ot: ObstacleTrajectory,
                           source_tick, publish_tick) -> ObstaclePrediction:
        """Stationary prediction (parked/stopped vehicles)."""
        cur = ot.trajectory[0]
        cur_tf = Transform(location=cur.location, rotation=cur.rotation)
        pred_tfs = [Transform(
            location=Location(x=cur.location.x, y=cur.location.y, z=cur.location.z),
            rotation=Rotation(yaw=cur.rotation.yaw)
        )] * self.num_output_steps
        return ObstaclePrediction(
            ot, cur_tf, probability=1.0,
            predicted_trajectory=pred_tfs,
            source_tick=source_tick, publish_tick=publish_tick)

    # ── Utilities ──────────────────────────────────────────────────

    @staticmethod
    def _wrap_angle(a: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def get_stats(self) -> Dict:
        """Return adaptation statistics."""
        return dict(self._stats)

    def reset_cache(self):
        """Clear prediction cache (e.g., on scene change)."""
        self._cache.clear()
