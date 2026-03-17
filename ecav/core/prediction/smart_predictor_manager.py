"""SMART trajectory predictor for edge pipeline.

Wraps SMART (NeurIPS 2024) as a drop-in replacement for LinearPredictorManager.
Converts AB3DMOT tracked trajectories to SMART's tokenized HeteroData, runs
joint multi-agent autoregressive inference, and converts back to world-frame
ObstaclePrediction objects.

SMART operates at 10Hz (Waymo training data). This wrapper handles frequency
conversion between the 20Hz simulation tick rate and SMART's native rate:
  Input:  subsamples 20Hz trajectory history to 10Hz
  Output: interpolates 10Hz predictions back to 20Hz
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import math
import os
import pickle
import sys
import types
from typing import Dict, List, Optional

import numpy as np
import torch
from torch_geometric.data import HeteroData

from ecav.core.prediction.obstacle_prediction import ObstaclePrediction
from ecav.ecav_carla import Location, Rotation, Transform

logger = logging.getLogger(__name__)

# ── Timing ──────────────────────────────────────────────────────────────────
DT_SMART = 0.1          # SMART native frame interval (Waymo 10Hz)
DT_SIM = 0.05           # Simulation tick interval (20Hz)
SUBSAMPLE = round(DT_SMART / DT_SIM)  # 2

# ── SMART architecture constants ────────────────────────────────────────────
NUM_HIST_RAW = 11        # 10Hz history frames expected by decoder
NUM_FUTURE_RAW = 80      # 10Hz future frames produced by decoder
SHIFT = 5                # frames per token step
TOKEN_SIZE = 2048        # trajectory token vocabulary

# Tokenized history: range(SHIFT, NUM_HIST_RAW, SHIFT) = [5, 10] → 2 steps
NUM_HIST_TOKENS = len(range(SHIFT, NUM_HIST_RAW, SHIFT))
NUM_FUTURE_TOKENS = NUM_FUTURE_RAW // SHIFT          # 16
NUM_TOTAL_TOKENS = NUM_HIST_TOKENS + NUM_FUTURE_TOKENS  # 18

# ── Track maturity ──────────────────────────────────────────────────────────
# SMART needs 11 frames at 10Hz = 22 ticks at 20Hz for proper tokenization.
# CMP (MTR fork) uses the same threshold: 10 past frames at 10Hz = 20 ticks.
# Tracks shorter than this are left to the linear predictor (controller knob).
_MIN_HIST_IDENT = NUM_HIST_RAW * SUBSAMPLE   # 22
_MIN_HIST_ANON = NUM_HIST_RAW * SUBSAMPLE    # 22

# ── Stationary gate (must match LinearPredictorManager) ────────────────────
_MIN_KF_SPEED_MPS = 1.0

# ── Default vehicle dimensions (metres) for tokenization ───────────────────
_VEH_LENGTH = 4.8
_VEH_WIDTH = 2.0


class SMARTPredictorManager3D:
    """Joint multi-agent trajectory prediction using SMART.

    Operates on the same Dict[int, ObstacleTrajectory] interface as
    LinearPredictorManager.  All agents are predicted jointly in a single
    forward pass via SMART's autoregressive token decoder with agent-agent
    and polyline-agent graph attention.

    Args:
        checkpoint_path: Path to SMART checkpoint (e.g. epoch=105.ckpt).
        map_cache_path: Path to pre-tokenized CARLA map polylines (.pkl).
            If None, runs without map context.
        device: 'cuda' or 'cpu'.
        num_output_steps: Prediction steps at simulation rate (default 25).
    """

    def __init__(self, cfg_or_checkpoint=None, **kwargs):
        # Accept either a config dict (from registry) or direct kwargs (legacy)
        if isinstance(cfg_or_checkpoint, dict):
            cfg = cfg_or_checkpoint
            checkpoint_path = cfg['checkpoint']
            map_cache_path = cfg.get('map_cache')
            device = cfg.get('device', 'cuda')
            num_output_steps = cfg.get('num_output_steps', 25)
        else:
            checkpoint_path = cfg_or_checkpoint
            map_cache_path = kwargs.get('map_cache_path')
            device = kwargs.get('device', 'cuda')
            num_output_steps = kwargs.get('num_output_steps', 25)

        self.device = torch.device(device)
        self.num_output_steps = num_output_steps

        self._load_model(checkpoint_path)

        # Extract vehicle token codebook from loaded decoder.
        # Shape: (2048, 4, 2) bounding-box corners in agent-local frame.
        decoder = self.model.encoder.agent_encoder
        veh_tok = decoder.trajectory_token['veh']
        if isinstance(veh_tok, np.ndarray):
            veh_tok = torch.tensor(veh_tok, dtype=torch.float32)
        self.traj_token_veh = veh_tok.to(self.device)

        # Pre-tokenized map cache (created by CARLA map extraction utility).
        self.map_data = None
        if map_cache_path and os.path.exists(map_cache_path):
            with open(map_cache_path, 'rb') as f:
                self.map_data = pickle.load(f)
            logger.info("SMART map cache loaded: %s", map_cache_path)

    # ── Model loading ───────────────────────────────────────────────────────

    def _load_model(self, checkpoint_path: str):
        """Instantiate SMART and load trained weights.

        Imports the smart package from ecav/core/prediction/smart/.
        Mocks waymo_open_dataset and wandb since they are only used
        for Waymo submission and logging, not inference.
        """
        # The smart package lives alongside this file. Add its parent to
        # sys.path so that `import smart` resolves to our vendored copy.
        prediction_dir = os.path.dirname(os.path.abspath(__file__))
        if prediction_dir not in sys.path:
            sys.path.insert(0, prediction_dir)

        # Mock optional dependencies that smart.py imports at module level
        # but does not use during inference.
        if 'waymo_open_dataset' not in sys.modules:
            wod = types.ModuleType('waymo_open_dataset')
            wod_protos = types.ModuleType('waymo_open_dataset.protos')
            wod_pb2 = types.ModuleType(
                'waymo_open_dataset.protos.sim_agents_submission_pb2')
            wod_pb2.JointScene = type('JointScene', (), {})
            wod_pb2.SimulatedTrajectory = type('SimulatedTrajectory', (), {})
            wod.protos = wod_protos
            wod_protos.sim_agents_submission_pb2 = wod_pb2
            sys.modules['waymo_open_dataset'] = wod
            sys.modules['waymo_open_dataset.protos'] = wod_protos
            sys.modules['waymo_open_dataset.protos.sim_agents_submission_pb2'] = wod_pb2
        if 'wandb' not in sys.modules:
            sys.modules['wandb'] = types.ModuleType('wandb')

        from omegaconf import OmegaConf
        from smart.model.smart import SMART

        # Standalone SMART takes a flat model_config (not nested under Model).
        cfg = OmegaConf.create({
            'input_dim': 2,
            'hidden_dim': 128,
            'output_dim': 2,
            'output_head': False,
            'num_historical_steps': NUM_HIST_RAW,
            'num_future_steps': NUM_FUTURE_RAW,
            'num_freq_bands': 64,
            'num_heads': 8,
            'head_dim': 16,
            'dropout': 0.1,
            'use_intention': True,
            'token_size': TOKEN_SIZE,
            'lr': 5e-4,
            'warmup_steps': 0,
            'total_steps': 32,
            'weight_decay': 1e-4,
            'dataset': 'waymo',
            'decoder': {
                'num_future_steps': NUM_FUTURE_RAW,
                'num_historical_steps': NUM_HIST_RAW,
                'num_map_layers': 3,
                'num_agent_layers': 6,
                'a2a_radius': 60.0,
                'pl2a_radius': 30.0,
                'pl2pl_radius': 10.0,
                'time_span': 30,
                'use_intention': True,
                'token_size': TOKEN_SIZE,
            },
        })

        self.model = SMART(cfg)

        # SMART checkpoint was saved with older PyTorch and contains
        # easydict objects. Torch 2.6+ defaults weights_only=True which
        # rejects them. Patch torch.load for this call only.
        _orig_load = torch.load
        torch.load = lambda *a, **kw: _orig_load(
            *a, **{**kw, 'weights_only': False})
        try:
            self.model.load_params_from_file(checkpoint_path, logger)
        finally:
            torch.load = _orig_load

        self.model.to(self.device)
        self.model.eval()
        logger.info("SMART model loaded: %s", checkpoint_path)

    # ── Public interface ────────────────────────────────────────────────────

    def generate_predicted_trajectories(
            self,
            tracked_obstacles_trajectories: Dict,
            source_tick=None,
            publish_tick=None) -> List[ObstaclePrediction]:
        """Generate trajectory predictions for all tracked obstacles.

        Same signature as LinearPredictorManager.generate_predicted_trajectories.
        """
        # Filter by track maturity
        valid = []
        rejected = 0
        for tid, ot in tracked_obstacles_trajectories.items():
            n = len(ot.trajectory)
            cid = getattr(ot.obstacle, 'carla_id', -1)
            min_hist = _MIN_HIST_IDENT if cid != -1 else _MIN_HIST_ANON
            if n >= min_hist:
                valid.append((tid, ot))
            else:
                rejected += 1

        total = len(tracked_obstacles_trajectories)
        hist_lens = [len(ot.trajectory) for ot in tracked_obstacles_trajectories.values()]
        max_hist = max(hist_lens) if hist_lens else 0
        logger.info("[SMART] tick=%s: %d tracks total, %d valid, %d rejected "
                    "(immature, need %d ticks, max_hist=%d, all=%s)",
                    source_tick, total, len(valid), rejected,
                    _MIN_HIST_ANON, max_hist, hist_lens[:8])

        if not valid:
            return []

        # Stationary gate: objects with kf_speed < 1.0 m/s get an
        # all-identical-position prediction (same as LinearPredictorManager).
        # This ensures the behavior agent's TTC=1000 parked-car gate works.
        predictions_out = []
        smart_tracks = []
        for tid, ot in valid:
            kf_speed = getattr(ot.obstacle, 'kf_speed_mps', None)
            if kf_speed is not None and kf_speed < _MIN_KF_SPEED_MPS:
                latest_tf = ot.trajectory[0]
                static_tf = Transform(
                    location=Location(x=latest_tf.location.x,
                                      y=latest_tf.location.y,
                                      z=latest_tf.location.z),
                    rotation=Rotation(roll=0.0, pitch=0.0,
                                      yaw=latest_tf.rotation.yaw))
                predictions_out.append(ObstaclePrediction(
                    ot, latest_tf, probability=1.0,
                    predicted_trajectory=[static_tf] * self.num_output_steps,
                    source_tick=source_tick, publish_tick=publish_tick))
                continue
            smart_tracks.append((tid, ot))

        if not smart_tracks:
            return predictions_out

        track_ids = [t[0] for t in smart_tracks]
        otrajs = [t[1] for t in smart_tracks]

        for tid, ot in smart_tracks:
            obs = ot.obstacle
            pos = ot.trajectory[0].location
            spd = getattr(obs, 'kf_speed_mps', None)
            logger.info("[SMART]   track=%d cid=%s pos=(%.1f,%.1f) hist=%d spd=%s",
                        tid, getattr(obs, 'carla_id', -1),
                        pos.x, pos.y, len(ot.trajectory),
                        f"{spd:.1f}" if spd is not None else "?")

        # Extract 10Hz history from 20Hz trajectories
        positions, headings, cur_z = self._extract_history_10hz(otrajs)

        # Tokenize agent trajectories against codebook
        tok_idx, tok_pos, tok_head = self._tokenize_agents(positions, headings)

        # Build SMART input graph
        data = self._build_hetero_data(tok_idx, tok_pos, tok_head,
                                       positions, headings, len(smart_tracks))

        # Run joint inference
        with torch.no_grad():
            output = self.model.inference(data)

        pred_traj = output['pred_traj'].cpu().numpy()   # (N, 80, 2)
        pred_head = output['pred_head'].cpu().numpy()   # (N, 80)

        # Log predicted trajectories for each agent
        for i, tid in enumerate(track_ids):
            cur = otrajs[i].trajectory[0].location
            p0 = pred_traj[i, 0]   # first predicted position (0.1s ahead)
            p12 = pred_traj[i, 12]  # ~1.3s ahead
            dx = p12[0] - cur.x
            dy = p12[1] - cur.y
            disp = (dx**2 + dy**2)**0.5
            logger.info("[SMART]   pred track=%d: cur=(%.1f,%.1f) -> "
                        "t+0.1s=(%.1f,%.1f) t+1.3s=(%.1f,%.1f) disp=%.2fm",
                        tid, cur.x, cur.y, p0[0], p0[1], p12[0], p12[1], disp)

        smart_preds = self._to_predictions(pred_traj, pred_head, cur_z,
                                           track_ids, otrajs,
                                           source_tick, publish_tick)
        return predictions_out + smart_preds

    # ── History extraction ──────────────────────────────────────────────────

    def _extract_history_10hz(self, otrajs):
        """Subsample 20Hz trajectory deques to 11-frame 10Hz history.

        Returns:
            positions: (N, 11, 2) world-frame XY, chronological (0=oldest).
            headings:  (N, 11) radians, chronological.
            cur_z:     (N,) current z-coordinate.
        """
        N = len(otrajs)
        positions = np.zeros((N, NUM_HIST_RAW, 2), dtype=np.float32)
        headings = np.zeros((N, NUM_HIST_RAW), dtype=np.float32)
        cur_z = np.zeros(N, dtype=np.float32)

        for i, ot in enumerate(otrajs):
            traj = ot.trajectory  # [0]=newest, [-1]=oldest

            # Subsample: indices 0,2,4,...,20 from newest end, then reverse
            frames = []
            for j in range(NUM_HIST_RAW):
                idx = j * SUBSAMPLE
                if idx < len(traj):
                    frames.append(traj[idx])
                else:
                    frames.append(traj[-1])  # pad with oldest
            frames.reverse()  # now 0=oldest, 10=newest

            for j, tf in enumerate(frames):
                positions[i, j, 0] = tf.location.x
                positions[i, j, 1] = tf.location.y
                headings[i, j] = math.radians(tf.rotation.yaw)

            cur_z[i] = traj[0].location.z

        return positions, headings, cur_z

    # ── Agent tokenization ──────────────────────────────────────────────────

    def _tokenize_agents(self, positions, headings):
        """Match trajectory segments to the 2048-token codebook.

        For 11 historical frames with shift=5, produces 2 token steps
        by matching at frames 5 and 10.

        Each token encodes a 5-frame motion segment as 4 bounding-box
        corners in agent-local coordinates. Matching rotates all 2048
        templates into world frame using the previous heading, then
        selects the template whose corners are closest (L2) to the
        agent's actual bounding box.

        Returns:
            token_idx:     (N, 2) int64 codebook indices
            token_pos:     (N, 2, 2) float32 matched centers, world frame
            token_heading: (N, 2) float32 matched headings, radians
        """
        N = positions.shape[0]
        dev = self.device
        token_src = self.traj_token_veh  # (2048, 4, 2) agent-local

        out_idx = np.zeros((N, NUM_HIST_TOKENS), dtype=np.int64)
        out_pos = np.zeros((N, NUM_HIST_TOKENS, 2), dtype=np.float32)
        out_head = np.zeros((N, NUM_HIST_TOKENS), dtype=np.float32)

        # Initialize from oldest frame
        prev_pos = torch.tensor(positions[:, 0], dtype=torch.float32, device=dev)
        prev_head = torch.tensor(headings[:, 0], dtype=torch.float32, device=dev)

        hl = _VEH_LENGTH / 2.0
        hw = _VEH_WIDTH / 2.0

        for step, frame_idx in enumerate(range(SHIFT, NUM_HIST_RAW, SHIFT)):
            cur_pos = torch.tensor(
                positions[:, frame_idx], dtype=torch.float32, device=dev)
            cur_head = torch.tensor(
                headings[:, frame_idx], dtype=torch.float32, device=dev)

            # Rotate token templates from agent-local to world using prev_head
            cos_p = prev_head.cos()
            sin_p = prev_head.sin()
            rot = torch.stack([
                torch.stack([cos_p, sin_p], dim=-1),
                torch.stack([-sin_p, cos_p], dim=-1)
            ], dim=-2)  # (N, 2, 2)

            # (N, 2048*4, 2) = (N, 2048*4, 2) @ (N, 2, 2)
            tok_flat = token_src.unsqueeze(0).expand(N, -1, -1, -1).reshape(N, -1, 2)
            tok_world = torch.bmm(tok_flat, rot).reshape(N, TOKEN_SIZE, 4, 2)
            tok_world = tok_world + prev_pos[:, None, None, :]

            # Compute actual bounding box at current frame
            cos_c = cur_head.cos()
            sin_c = cur_head.sin()
            corners = torch.zeros(N, 4, 2, device=dev)
            # left_front, right_front, right_back, left_back
            corners[:, 0, 0] = cur_pos[:, 0] + hl * cos_c - hw * sin_c
            corners[:, 0, 1] = cur_pos[:, 1] + hl * sin_c + hw * cos_c
            corners[:, 1, 0] = cur_pos[:, 0] + hl * cos_c + hw * sin_c
            corners[:, 1, 1] = cur_pos[:, 1] + hl * sin_c - hw * cos_c
            corners[:, 2, 0] = cur_pos[:, 0] - hl * cos_c + hw * sin_c
            corners[:, 2, 1] = cur_pos[:, 1] - hl * sin_c - hw * cos_c
            corners[:, 3, 0] = cur_pos[:, 0] - hl * cos_c - hw * sin_c
            corners[:, 3, 1] = cur_pos[:, 1] - hl * sin_c + hw * cos_c

            # L2 distance over 4 corners, averaged
            dist = (corners.unsqueeze(1) - tok_world).pow(2).sum(-1).mean(-1)
            best = dist.argmin(dim=-1)  # (N,)

            # Extract matched token contour
            arange_n = torch.arange(N, device=dev)
            matched = tok_world[arange_n, best]       # (N, 4, 2)
            matched_center = matched.mean(dim=1)       # (N, 2)
            matched_heading = torch.atan2(
                matched[:, 0, 1] - matched[:, 3, 1],
                matched[:, 0, 0] - matched[:, 3, 0])  # (N,)

            out_idx[:, step] = best.cpu().numpy()
            out_pos[:, step] = matched_center.cpu().numpy()
            out_head[:, step] = matched_heading.cpu().numpy()

            prev_pos = matched_center
            prev_head = matched_heading

        return out_idx, out_pos, out_head

    # ── HeteroData construction ─────────────────────────────────────────────

    def _build_hetero_data(self, tok_idx, tok_pos, tok_head,
                           raw_pos, raw_head, num_agents):
        """Construct PyG HeteroData for SMART inference.

        Token-level fields (token_pos, token_heading, token_idx,
        agent_valid_mask, token_velocity) are padded to NUM_TOTAL_TOKENS
        (18 = 2 historical + 16 future). The decoder zeros out future
        entries and fills them autoregressively during inference.

        Raw-frame fields (position, valid_mask, shape) stay at 91 frames
        (11 historical + 80 future). The decoder reads shape[10] and
        computes num_recurrent_steps from position.shape[1].
        """
        dev = self.device
        data = HeteroData()
        N = num_agents
        total_raw = NUM_HIST_RAW + NUM_FUTURE_RAW   # 91
        T = NUM_TOTAL_TOKENS                         # 18

        # ── Token-level fields (padded to 18) ───────────────────────────
        # Historical tokens in first NUM_HIST_TOKENS positions, rest zero
        full_tok_pos = np.zeros((N, T, 2), dtype=np.float32)
        full_tok_pos[:, :NUM_HIST_TOKENS] = tok_pos
        data['agent']['token_pos'] = torch.tensor(
            full_tok_pos, dtype=torch.float32, device=dev)

        full_tok_head = np.zeros((N, T), dtype=np.float32)
        full_tok_head[:, :NUM_HIST_TOKENS] = tok_head
        data['agent']['token_heading'] = torch.tensor(
            full_tok_head, dtype=torch.float32, device=dev)

        full_tok_idx = np.zeros((N, T), dtype=np.int64)
        full_tok_idx[:, :NUM_HIST_TOKENS] = tok_idx
        data['agent']['token_idx'] = torch.tensor(
            full_tok_idx, dtype=torch.long, device=dev)

        # Token-level validity (decoder overrides [:, 2:] to True)
        tok_valid = torch.zeros(N, T, dtype=torch.bool, device=dev)
        tok_valid[:, :NUM_HIST_TOKENS] = True
        data['agent']['agent_valid_mask'] = tok_valid

        # Token-level velocity
        dt_tok = DT_SMART * SHIFT  # 0.5s between tokens
        vel = np.zeros((N, T, 3), dtype=np.float32)
        if NUM_HIST_TOKENS > 1:
            vel[:, 1:NUM_HIST_TOKENS, :2] = np.diff(tok_pos, axis=1) / dt_tok
        data['agent']['token_velocity'] = torch.tensor(
            vel, dtype=torch.float32, device=dev)

        # ── Raw-frame fields (91 total) ─────────────────────────────────
        # All vehicles
        data['agent']['type'] = torch.zeros(N, dtype=torch.uint8, device=dev)
        data['agent']['category'] = torch.zeros(N, dtype=torch.long, device=dev)

        # Shape: [length, width, height] per raw frame
        shape = np.broadcast_to(
            np.array([_VEH_LENGTH, _VEH_WIDTH, 1.5], dtype=np.float32),
            (N, total_raw, 3)).copy()
        data['agent']['shape'] = torch.tensor(
            shape, dtype=torch.float32, device=dev)

        # GT position placeholder (historical real, future dummy)
        pos_full = np.zeros((N, total_raw, 3), dtype=np.float32)
        pos_full[:, :NUM_HIST_RAW, :2] = raw_pos
        data['agent']['position'] = torch.tensor(
            pos_full, dtype=torch.float32, device=dev)

        valid_full = np.zeros((N, total_raw), dtype=bool)
        valid_full[:, :NUM_HIST_RAW] = True
        data['agent']['valid_mask'] = torch.tensor(
            valid_full, dtype=torch.bool, device=dev)

        # ── Scalar fields ────────────────────────────────────────────────
        data['agent']['av_index'] = torch.zeros(N, dtype=torch.bool, device=dev)
        data['agent']['batch'] = torch.zeros(N, dtype=torch.long, device=dev)
        data['agent']['num_nodes'] = N

        # Map data
        if self.map_data is not None:
            self._populate_map_cache(data)
        else:
            self._populate_empty_map(data)

        return data

    def _populate_empty_map(self, data):
        """Create empty map tensors for no-map inference."""
        dev = self.device
        z = lambda *s, dt=torch.float32: torch.zeros(*s, dtype=dt, device=dev)
        zl = lambda *s: torch.zeros(*s, dtype=torch.long, device=dev)
        zb = lambda *s: torch.zeros(*s, dtype=torch.bool, device=dev)

        data['pt_token']['position'] = z(0, 3)
        data['pt_token']['orientation'] = z(0)
        data['pt_token']['type'] = zl(0)
        data['pt_token']['pl_type'] = zl(0)
        data['pt_token']['pt_valid_mask'] = zb(0)
        data['pt_token']['pt_pred_mask'] = zb(0)
        data['pt_token']['pt_target_mask'] = zb(0)
        data['pt_token']['batch'] = zl(0)
        data['pt_token']['num_nodes'] = 0
        data['pt_token']['token_idx'] = zl(0)
        data['pt_token']['height'] = z(0)
        data['pt_token']['traj_mask'] = zb(0, 3, 0)

        data[('pt_token', 'to', 'map_polygon')]['edge_index'] = zl(2, 0)

        data['map_polygon']['light_type'] = zl(0)
        data['map_polygon']['num_nodes'] = 0

    def _populate_map_cache(self, data):
        """Load pre-tokenized map data from cache into HeteroData.

        Expected cache format (dict):
            pt_token: dict of tensors/arrays for polyline token fields
            edge_index: (2, E) bipartite edges pt_token→map_polygon
            map_polygon: dict with light_type
        """
        dev = self.device
        mc = self.map_data

        pt = mc.get('pt_token', {})
        for key in ['position', 'orientation', 'type', 'pl_type', 'height',
                     'token_idx', 'pt_valid_mask', 'pt_pred_mask',
                     'pt_target_mask', 'traj_mask']:
            if key in pt:
                v = pt[key]
                if isinstance(v, np.ndarray):
                    v = torch.from_numpy(v)
                data['pt_token'][key] = v.to(dev)

        num_pt = (data['pt_token']['position'].shape[0]
                  if 'position' in pt else 0)
        data['pt_token']['batch'] = torch.zeros(
            num_pt, dtype=torch.long, device=dev)
        data['pt_token']['num_nodes'] = num_pt

        if 'edge_index' in mc:
            ei = mc['edge_index']
            if isinstance(ei, np.ndarray):
                ei = torch.from_numpy(ei)
            data[('pt_token', 'to', 'map_polygon')]['edge_index'] = ei.to(
                dtype=torch.long, device=dev)
        else:
            data[('pt_token', 'to', 'map_polygon')]['edge_index'] = (
                torch.zeros(2, 0, dtype=torch.long, device=dev))

        mp = mc.get('map_polygon', {})
        if 'light_type' in mp:
            lt = mp['light_type']
            if isinstance(lt, np.ndarray):
                lt = torch.from_numpy(lt)
            data['map_polygon']['light_type'] = lt.to(
                dtype=torch.long, device=dev)
            data['map_polygon']['num_nodes'] = lt.shape[0]
        else:
            data['map_polygon']['light_type'] = torch.zeros(
                0, dtype=torch.long, device=dev)
            data['map_polygon']['num_nodes'] = 0

    # ── Output conversion ───────────────────────────────────────────────────

    def _to_predictions(self, pred_traj, pred_head, cur_z,
                        track_ids, otrajs, source_tick, publish_tick):
        """Convert SMART output to ObstaclePrediction list.

        Interpolates 10Hz SMART trajectories to 20Hz simulation rate and
        truncates to num_output_steps.
        """
        predictions = []
        N = pred_traj.shape[0]

        # Time grids for interpolation
        t_smart = np.arange(1, NUM_FUTURE_RAW + 1) * DT_SMART   # 0.1..8.0
        t_sim = np.arange(1, self.num_output_steps + 1) * DT_SIM  # 0.05..1.25

        for i in range(N):
            z = float(cur_z[i])

            # Interpolate XY from 10Hz to 20Hz
            x_i = np.interp(t_sim, t_smart, pred_traj[i, :, 0])
            y_i = np.interp(t_sim, t_smart, pred_traj[i, :, 1])

            # Interpolate heading (unwrap to avoid discontinuity at +/-pi)
            h_unwrap = np.unwrap(pred_head[i])
            h_i = np.interp(t_sim, t_smart, h_unwrap)

            pred_transforms = [
                Transform(
                    location=Location(x=float(x_i[j]),
                                      y=float(y_i[j]),
                                      z=z),
                    rotation=Rotation(roll=0.0, pitch=0.0,
                                      yaw=math.degrees(h_i[j])))
                for j in range(self.num_output_steps)
            ]

            ot = otrajs[i]
            latest_tf = ot.trajectory[0]  # newest
            predictions.append(ObstaclePrediction(
                ot, latest_tf,
                probability=1.0,
                predicted_trajectory=pred_transforms,
                source_tick=source_tick,
                publish_tick=publish_tick))

        return predictions
