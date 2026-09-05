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

# Shared eval-mode MTR instances keyed by (checkpoint, device) — multi-edge
# sims on one GPU reuse rather than duplicate identical weights.
_SHARED_MTR_MODELS = {}

# MTR constants (matching CMP's OPV2V config)
_PAST_FRAMES = 10       # 10 past frames at 10Hz
_TIME_INTERVAL = 0.1    # 10Hz
_FUTURE_FRAMES = 50     # 5s at 10Hz
_MIN_HIST_TICKS = 5     # lowered from 22 for compatibility with sparse detections
# tracked_trajectories is appended on every fusion event; the cadence-debug
# trace shows fusion fires every 2 sim-ticks (= 0.1 s = 10 Hz) on the
# WorldFusion adaptive edge, which already matches MTR's trained
# _TIME_INTERVAL.  Striding over this signal feeds MTR positions 2× too far
# apart, making it interpret all motion as ~2× the true speed (and pushing
# it out of training distribution → it falls back to mean predictions).
_SUBSAMPLE = 1
# Stationary gate: parked cars exhibit Kalman-velocity noise around
# 0.5-1.2 m/s due to detection-center jitter (~0.3 m frame-to-frame).
# A 1.0 m/s gate lets transient noise spikes route a parked track to
# MTR, which then hallucinates 5-8 m/s motion (far enough above
# cross-traffic acceleration profiles in training that the model
# defaults to "drive forward"). Raising the gate to 2.0 m/s keeps
# truly stationary tracks on the static-prediction path while still
# catching real moving obstacles (cars: 5+ m/s, pedestrians: 1.4 m/s
# walking, but pedestrians not in scope for SEC paper scenarios).
_MIN_SPEED_MPS = 2.0

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
                 ego_vehicles: list = None,
                 cfg_yaml: str = None,
                 intention_points_file: str = None,
                 dataset: str = 'opv2v',
                 aggregator: str = None,
                 past_frames: int = _PAST_FRAMES,
                 future_frames: int = _FUTURE_FRAMES,
                 time_interval: float = _TIME_INTERVAL,
                 history_subsample: int = _SUBSAMPLE,
                 output_dt: float = 0.05,
                 lane_map: str = None):

        self.device = torch.device(device)
        self.num_output_steps = num_output_steps
        # Model-config knobs (defaults preserve the original OPV2V wiring)
        self._dataset = dataset
        self._past_frames = int(past_frames)
        self._future_frames = int(future_frames)
        self._dt = float(time_interval)
        self._subsample = int(history_subsample)
        # Simulation tick length. Consumers (behavior agent, ADE/FDE eval,
        # proto distribution) index predicted_trajectory at 0.05 s/step, while
        # the model emits steps at self._dt; _to_world resamples between them.
        self._output_dt = float(output_dt)
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
        self._load_model(cmp_root, mtr_checkpoint, cfg_yaml, intention_points_file, aggregator)

        # RSU-zone lane raster for the Swin lane encoder (same convention as
        # the Multi-V2X training rasters: white drivable lanes on black,
        # 256x256 RGB). Without it the lane input falls back to zeros, which
        # is outside the training distribution.
        if lane_map:
            from PIL import Image
            img = Image.open(lane_map)
            if img.mode != 'RGB':
                img = img.convert('RGB')
            self.set_lane_map(torch.from_numpy(np.array(img, dtype=np.uint8)))
            logger.info("Lane raster loaded: %s", lane_map)
        else:
            logger.warning("No lane_map configured; Swin lane input falls "
                           "back to zeros (out of training distribution)")

        # Stats
        self._stats = {
            'total_calls': 0, 'cache_hits': 0, 'mtr_predicted': 0,
            'linear_fallback': 0, 'risk_pruned': 0,
        }

    def _load_model(self, cmp_root, mtr_checkpoint, cfg_yaml=None,
                    intention_points_file=None, aggregator=None):
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

        if cfg_yaml is None:
            cfg_path = os.path.join(
                mtr_root, 'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
        else:
            cfg_path = cfg_yaml if os.path.isabs(cfg_yaml) else os.path.abspath(cfg_yaml)
        cfg_from_yaml_file(cfg_path, cfg)
        if intention_points_file is not None:
            cfg.MODEL.MOTION_DECODER.INTENTION_POINTS_FILE = os.path.abspath(
                intention_points_file)
        if aggregator is not None:
            # 'None' runs the trained stage-1 decoder output directly and
            # skips constructing the (possibly untrained) stage-2 aggregator.
            cfg.MODEL.MOTION_AGGREGATOR.TYPE = aggregator
        if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
            # Relative lane-encoder paths differ by config generation:
            # opv2v cfgs are relative to cmp_root, multiv2x cfgs to mtr_root.
            for root in (mtr_root, cmp_root):
                cand = os.path.join(root, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)
                if os.path.exists(cand):
                    cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = cand
                    break

        if mtr_checkpoint is None:
            mtr_checkpoint = os.path.join(
                mtr_root, 'output/opv2v_multiego_cobevt_c256/ckpt/best_model.pth')

        # Multi-edge sims load byte-identical checkpoints per edge; share
        # one eval-mode instance per (checkpoint, device). Deployment runs
        # one edge per server, so this is a sim-hosting optimization only
        # (two full copies OOM'd a 16 GB card next to CARLA/Town06).
        _key = (os.path.abspath(mtr_checkpoint), str(self.device))
        _shared = _SHARED_MTR_MODELS.get(_key)
        if _shared is not None:
            self.model = _shared
            logger.info("MTR model reused from shared cache (%s)", _key[0])
            return

        self.model = MotionTransformerWithMultiEgoAggregation(cfg.MODEL)
        self.model.to(self.device).eval()

        if os.path.exists(mtr_checkpoint):
            state = torch.load(mtr_checkpoint, map_location=self.device)
            if 'model_state' in state:
                state = state['model_state']
            self.model.load_state_dict(state, strict=False)
            logger.info("MTR model loaded from %s", mtr_checkpoint)
            _SHARED_MTR_MODELS[_key] = self.model
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

        # Classify tracks. The stationary-skip is engineering common sense
        # (constant-position fallback for parked cars), not part of the
        # adaptive controller's amortization. Apply it universally regardless
        # of policy. The amortization mechanism (cross-cycle cache reuse for
        # non-divergent moving tracks) lives further down and is gated on
        # enable_amortization separately.
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
            p = self._static_prediction(ot, source_tick, publish_tick)
            p.source = 'static'
            predictions.append(p)

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
            p = cached.prediction
            p.source = 'cache'
            predictions.append(p)
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
            pred.source = 'linear'
            predictions.append(pred)
            self._stats['linear_fallback'] += 1
            self._stats['risk_pruned'] += 1

        # MTR prediction on selected subset
        if selected:
            mtr_preds = self._run_mtr(selected, source_tick, publish_tick)
            for p in mtr_preds:
                p.source = 'mtr'
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

        # A vehicle is prediction-worthy if it is EITHER close OR on a
        # collision course — combine proximity and conflict with a max, not
        # a product. The old product let proximity=exp(-dist/d_scale) collapse
        # the score of a far but closing vehicle (e.g. an overtake's oncoming
        # traffic at 50-80 m: conflict_prox high, proximity ~0.08, product
        # ~0), so the budgeted selector only ever forecast vehicles already
        # next to the ego and starved the far approacher that governs the
        # overtake go/no-go. max() keeps the near case unchanged while
        # surfacing collision-course actors regardless of current distance.
        engagement = max(proximity, conflict_prox)
        risk = speed * engagement * occlusion
        # Ensure non-zero baseline so pure speed still orders objects
        # when all ego terms are zero (distant/non-conflicting).
        baseline = speed * 0.01
        return max(risk, baseline)

    # ── MTR inference ──────────────────────────────────────────────

    def _run_mtr(self, tracks: Dict[int, ObstacleTrajectory],
                 source_tick, publish_tick) -> List[ObstaclePrediction]:
        """Run MTR on a subset of tracks and update cache."""
        if self._dataset == 'multiv2x':
            from mtr.datasets.multiv2x_multiego_dataset import (
                MultiV2XMultiEgoDataset as _DS)
        else:
            from mtr.datasets.opv2v_multiego_dataset import (
                OPV2VMultiEgoDataset as _DS)

        track_ids = list(tracks.keys())
        otrajs = [tracks[tid] for tid in track_ids]
        n = len(otrajs)

        # Build trajectory history at the model rate (world ticks subsampled
        # by self._subsample). Frames beyond the available track history are
        # left invalid (valid=0, zero state): the MultiV2X-trained model saw
        # masked gaps (intermittent RSU detections), never a frozen replica
        # of the oldest frame, so padding with valid=1 is out of distribution.
        obj_trajs = np.zeros((n, self._past_frames + 1, 8), dtype=np.float32)
        for i, ot in enumerate(otrajs):
            traj = ot.trajectory
            for t in range(self._past_frames + 1):
                idx = t * self._subsample
                if idx >= len(traj):
                    continue
                tf = traj[idx]
                obj_trajs[i, self._past_frames - t] = [
                    tf.location.x, tf.location.y, tf.location.z,
                    4.5, 1.8, 1.5,  # default vehicle dimensions
                    math.radians(tf.rotation.yaw), 1.0]

        # Normalize yaw to motion direction (same rule as the offline
        # retrack tool): detection/tracker box yaw carries a 180-degree
        # ambiguity; the retrained model expects motion-aligned headings.
        for i in range(n):
            prev = None
            for t in range(self._past_frames + 1):
                if obj_trajs[i, t, 7] != 1:
                    prev = None
                    continue
                if prev is not None:
                    dx = obj_trajs[i, t, 0] - obj_trajs[i, prev, 0]
                    dy = obj_trajs[i, t, 1] - obj_trajs[i, prev, 1]
                    if (dx * dx + dy * dy) ** 0.5 > 0.1:
                        obj_trajs[i, t, 6] = math.atan2(dy, dx)
                prev = t

        obj_ids = np.array(track_ids)
        obj_types = np.array(['TYPE_VEHICLE'] * n)
        center_objects = obj_trajs[:, -1, :].copy()
        obj_trajs_future = np.zeros((n, self._future_frames, 8), dtype=np.float32)
        timestamps = np.array([self._dt * i for i in range(self._past_frames + 1)],
                              dtype=np.float32)

        try:
            ret = _DS.create_agent_data_for_center_objects(
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
                else torch.zeros(1, 256, 48, 176, device=self.device).unsqueeze(0)
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

        # Diagnostic: dump MTR raw output for the fastest input track
        # (proxy for Tesla cross-traffic). best[i, t, :2] is in the
        # center-object's local frame; +x = forward.  Per-step displacement
        # *should* match the model's _TIME_INTERVAL = 0.1 s, so a Tesla at
        # ~13 m/s should show first-step (0.1 s ahead) ~(1.3, 0.0).
        if n > 0:
            speeds = []
            for ii, ot_dbg in enumerate(otrajs):
                kfs = getattr(ot_dbg.obstacle, 'kf_speed_mps', 0.0) or 0.0
                speeds.append((kfs, ii))
            speeds.sort(reverse=True)
            kfs_dbg, i_dbg = speeds[0]
            best_dbg = pred_trajs[i_dbg, best_mode[i_dbg], :, :2]
            ch_dbg = float(center_objects[i_dbg, 6])
            dump = ", ".join(
                f"({float(best_dbg[s,0]):+.2f},{float(best_dbg[s,1]):+.2f})"
                for s in [0, 1, 2, 5, 10, 20])
            logger.info(
                "[MTR RAW] tid=%s kf_speed=%.2fm/s ch=%.2frad mode=%d/%d "
                "best[0,1,2,5,10,20]=%s",
                track_ids[i_dbg], kfs_dbg, ch_dbg, int(best_mode[i_dbg]),
                pred_scores.shape[1], dump)
            # Input-side view of the same track: raw world past
            # (oldest->newest) and the center-frame past the model consumed.
            wp = obj_trajs[i_dbg]           # (T+1, 8) world, oldest->newest
            valid_w = wp[:, 7] > 0
            world_pts = ", ".join(f"({p[0]:.1f},{p[1]:.1f})"
                                  for p in wp[valid_w][:, :2])
            ctr = otd[i_dbg, tipn[i_dbg]]   # center row past, (T+1, 22)
            ctr_m = otm[i_dbg, tipn[i_dbg]] > 0
            ctr_pts = ", ".join(f"({p[0]:+.1f},{p[1]:+.1f})"
                                for p in ctr[ctr_m][:, :2])
            logger.info("[MTR IN] tid=%s yaw=%.2f world_past=[%s] "
                        "center_past=[%s]",
                        track_ids[i_dbg], float(wp[valid_w][-1, 6]),
                        world_pts, ctr_pts)

        predictions = []
        n_modes = pred_trajs.shape[1]
        for i, (tid, ot) in enumerate(zip(track_ids, otrajs)):
            best = pred_trajs[i, best_mode[i], :, :2]  # (T, 2) in center frame

            # MTR output is in the center-object's local frame
            # (translated by -center_xyz, rotated by -center_heading; see
            # OPV2VMultiEgoDataset.transform_trajs_to_center_coords). Invert
            # back to world: rotate by +heading then translate by +center_xy.
            # Stationary tracks output ~(0, 0) in this frame, which is why
            # they look fine without the inverse — the bug only surfaces on
            # moving tracks (e.g. cross-traffic Tesla had future at (4.4, 0.3)
            # instead of world (-55.9, 127.4)).
            cx = float(center_objects[i, 0])
            cy = float(center_objects[i, 1])
            ch = float(center_objects[i, 6])
            cos_h, sin_h = math.cos(ch), math.sin(ch)
            cur_z = ot.trajectory[0].location.z

            # T15: stash ALL modes' world-frame endpoints + probabilities for
            # this track so the migration trigger can consume the multimodal
            # forecast (per-mode boundary crossing, probability summed by
            # destination). Endpoint = last horizon point of each mode.
            if not hasattr(self, 'last_mtr_modes'):
                self.last_mtr_modes = {}
            _modes = []
            _sc = pred_scores[i]
            _sc_sum = float(_sc.sum()) if float(_sc.sum()) > 1e-6 else 1.0
            for _m in range(pred_trajs.shape[1]):
                _end_l = pred_trajs[i, _m, -1, :2]
                _wx = cx + float(_end_l[0]) * cos_h - float(_end_l[1]) * sin_h
                _wy = cy + float(_end_l[0]) * sin_h + float(_end_l[1]) * cos_h
                _modes.append((_wx, _wy, float(_sc[_m]) / _sc_sum))
            self.last_mtr_modes[int(tid)] = _modes

            def _to_world(local_traj):
                # Model steps are self._dt seconds apart; consumers index the
                # returned list at self._output_dt (one entry per sim tick).
                # Linearly interpolate, treating the center-frame origin as
                # the t=0 point and clamping past the model horizon.
                n_model = len(local_traj)

                def _pt(j):
                    if j <= 0:
                        return 0.0, 0.0
                    j = min(j, n_model)
                    return float(local_traj[j - 1, 0]), float(local_traj[j - 1, 1])

                out = []
                for step in range(self.num_output_steps):
                    f = ((step + 1) * self._output_dt) / self._dt
                    j0 = int(math.floor(f))
                    frac = f - j0
                    x0, y0 = _pt(j0)
                    x1, y1 = _pt(j0 + 1)
                    lx = x0 + frac * (x1 - x0)
                    ly = y0 + frac * (y1 - y0)
                    wx = cx + lx * cos_h - ly * sin_h
                    wy = cy + lx * sin_h + ly * cos_h
                    out.append(Transform(
                        location=Location(x=wx, y=wy, z=float(cur_z)),
                        rotation=Rotation(yaw=0.0)))
                return out

            pred_tfs = _to_world(best)

            # All 6 modes for minFDE6 / minADE6 evaluation per CMP's protocol.
            all_modes_world = [_to_world(pred_trajs[i, m, :, :2])
                               for m in range(n_modes)]
            mode_scores = [float(pred_scores[i, m]) for m in range(n_modes)]

            cur_tf = Transform(
                location=Location(x=ot.trajectory[0].location.x,
                                  y=ot.trajectory[0].location.y,
                                  z=ot.trajectory[0].location.z),
                rotation=Rotation(yaw=ot.trajectory[0].rotation.yaw))

            pred = ObstaclePrediction(
                ot, cur_tf, probability=float(pred_scores[i, best_mode[i]]),
                predicted_trajectory=pred_tfs,
                source_tick=source_tick, publish_tick=publish_tick)
            pred.predicted_trajectories_all = all_modes_world
            pred.mode_scores = mode_scores
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
