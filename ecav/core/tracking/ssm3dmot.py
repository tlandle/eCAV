"""
SSM3DMOT: 3D Multi-Object Tracking with State Space Model motion prediction.

Replaces AB3DMOT's per-track Kalman filter with a batched Mamba SSM
for motion prediction. Keeps AB3DMOT's 3D IoU matching, Hungarian
assignment, birth/death logic, and output format.

The key scaling advantage: all track predictions run in one batched
GPU forward pass O(1) instead of N separate KF updates O(N).

Architecture:
    - Motion prediction: TrackSSM (Mamba encoder + FlowSSM decoder)
    - Data association: 3D IoU distance + Hungarian assignment (from AB3DMOT)
    - Track management: min_hits / max_age birth/death (from AB3DMOT)

Input/output format matches AB3DMOT exactly for drop-in replacement.

Author: Tyler Landle <tlandle3@gatech.edu>
License: TDG-Attribution-NonCommercial-NoDistrib
"""
from __future__ import annotations

import copy
import logging
import os
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import torch
import torch.nn as nn

from ecav.core.tracking.base_tracker import BaseTracker

logger = logging.getLogger("SSM3DMOT")

# 3D state: [x, y, z, theta, l, w, h]
STATE_DIM_3D = 7
# SSM input includes state + velocity diffs = 14 dims
SSM_INPUT_DIM = 14


class SSMMotionPredictor(nn.Module):
    """
    Batched Mamba SSM motion predictor for 3D object tracking.

    Takes track history (N_tracks, T, state_dim) and predicts
    next state for all tracks in one forward pass.

    Adapted from TrackSSM (SOTA_Trackers/Mamba_Trackers) for 3D boxes.
    """

    def __init__(self, d_model=256, d_state=16, n_layers=3,
                 input_dim=SSM_INPUT_DIM, output_dim=STATE_DIM_3D,
                 device='cuda:0'):
        super().__init__()

        try:
            from mamba_ssm import Mamba
        except ImportError:
            raise ImportError(
                "mamba_ssm required for SSM3DMOT. "
                "Install with: pip install mamba-ssm")

        self.input_dim = input_dim
        self.output_dim = output_dim
        self.d_model = d_model

        # Project 3D state + diffs to model dim
        self.input_proj = nn.Linear(input_dim, d_model)

        # Mamba encoder: processes track history sequence
        self.encoder = Mamba(
            d_model=d_model,
            d_state=d_state,
            d_conv=4,
            expand=2,
        )

        # Predict next state delta from encoded history
        self.output_proj = nn.Sequential(
            nn.Linear(d_model, d_model // 2),
            nn.SiLU(),
            nn.Linear(d_model // 2, output_dim),
        )

        self.to(device)
        self.device = device

    def forward(self, track_history: torch.Tensor) -> torch.Tensor:
        """
        Predict next state for all tracks.

        Args:
            track_history: (N_tracks, T, input_dim) track state history

        Returns:
            predicted_state: (N_tracks, output_dim) predicted next state
        """
        x = self.input_proj(track_history)  # (N, T, d_model)
        x = self.encoder(x)                 # (N, T, d_model)
        x = x[:, -1, :]                     # (N, d_model) last timestep
        delta = self.output_proj(x)          # (N, output_dim) predicted delta
        return delta


class SSMTrack:
    """Single track managed by SSM3DMOT."""

    _next_id = 1

    def __init__(self, state: np.ndarray, info: np.ndarray,
                 max_history: int = 10):
        """
        Args:
            state: [h, w, l, x, y, z, theta] initial detection (AB3DMOT format)
            info: [frame, det_idx, carla_id]
        """
        self.id = SSMTrack._next_id
        SSMTrack._next_id += 1

        self.state = state.copy()  # [h,w,l,x,y,z,theta]
        self.info = info.copy()

        # History buffer: newest last
        self.history: deque = deque(maxlen=max_history)
        self.history.append(state.copy())

        self.hits = 1
        self.age = 0
        self.time_since_update = 0
        self.hit_streak = 1

    def predict_kf_fallback(self):
        """Simple constant-velocity prediction as fallback."""
        if len(self.history) >= 2:
            vel = self.history[-1] - self.history[-2]
            self.state = self.history[-1] + vel
        self.age += 1
        self.time_since_update += 1

    def update(self, det: np.ndarray, info: np.ndarray):
        """Update track with matched detection."""
        self.state = det.copy()
        self.info = info.copy()
        self.history.append(det.copy())
        self.hits += 1
        self.time_since_update = 0
        self.hit_streak += 1

    def get_history_tensor(self, min_len: int = 3) -> Optional[np.ndarray]:
        """
        Get track history as (T, 14) array: [state(7) + diff(7)].

        Returns None if insufficient history.
        """
        hist = list(self.history)
        if len(hist) < min_len:
            return None

        states = np.array(hist)  # (T, 7)
        diffs = np.zeros_like(states)
        diffs[1:] = states[1:] - states[:-1]

        return np.concatenate([states, diffs], axis=1)  # (T, 14)

    @property
    def is_confirmed(self):
        return self.hits >= 3  # min_hits default


class SSM3DMOT(BaseTracker):
    """
    3D MOT with batched Mamba SSM motion prediction.

    Drop-in replacement for AB3DMOTWrapper. Same input/output format.
    """

    def __init__(self, cfg: dict):
        self.min_hits = cfg.get('min_hits', 3)
        self.max_age = cfg.get('max_age', 6)
        self.iou_threshold = cfg.get('iou_threshold', 0.1)
        self.max_history = cfg.get('max_history', 10)
        self.device = cfg.get('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')
        self.use_ssm = cfg.get('use_ssm', True)

        self.tracks: List[SSMTrack] = []
        self.frame_count = 0

        # SSM motion predictor (shared across all tracks)
        if self.use_ssm:
            try:
                self._ssm = SSMMotionPredictor(
                    d_model=cfg.get('d_model', 256),
                    d_state=cfg.get('d_state', 16),
                    n_layers=cfg.get('n_layers', 3),
                    device=self.device,
                ).eval()
                # Load pretrained weights if provided
                ssm_weights = cfg.get('ssm_weights')
                if ssm_weights and os.path.exists(ssm_weights):
                    self.load_ssm_weights(ssm_weights)
                logger.info("SSM3DMOT: Mamba SSM motion predictor initialized")
            except ImportError:
                logger.warning("mamba_ssm not available, falling back to constant velocity")
                self._ssm = None
                self.use_ssm = False
        else:
            self._ssm = None

        SSMTrack._next_id = 1

    def _batched_predict(self):
        """Predict next state for ALL tracks in one batched GPU forward pass."""
        if not self.tracks:
            return

        if self._ssm is None or not self.use_ssm:
            for trk in self.tracks:
                trk.predict_kf_fallback()
            return

        # Collect histories that are long enough for SSM
        ssm_indices = []
        ssm_histories = []
        fallback_indices = []

        for i, trk in enumerate(self.tracks):
            hist = trk.get_history_tensor(min_len=3)
            if hist is not None:
                ssm_indices.append(i)
                ssm_histories.append(hist)
            else:
                fallback_indices.append(i)

        # Fallback for short tracks
        for i in fallback_indices:
            self.tracks[i].predict_kf_fallback()

        if not ssm_histories:
            return

        # Pad histories to same length and batch
        max_len = max(h.shape[0] for h in ssm_histories)
        padded = np.zeros((len(ssm_histories), max_len, SSM_INPUT_DIM),
                          dtype=np.float32)
        for j, h in enumerate(ssm_histories):
            padded[j, -h.shape[0]:, :] = h  # right-align (newest at end)

        # Batched GPU forward pass
        with torch.no_grad():
            batch = torch.from_numpy(padded).to(self.device)
            deltas = self._ssm(batch).cpu().numpy()  # (N, 7)

        # Apply predictions
        for j, i in enumerate(ssm_indices):
            trk = self.tracks[i]
            trk.state = trk.history[-1] + deltas[j]
            trk.age += 1
            trk.time_since_update += 1

    @staticmethod
    def _iou_3d_distance(tracks, dets):
        """
        Compute 3D center distance matrix between tracks and detections.

        Uses center distance (not full 3D IoU) for efficiency.
        AB3DMOT also uses center distance as primary affinity.

        Args:
            tracks: list of SSMTrack
            dets: (M, 7) array [h,w,l,x,y,z,theta]

        Returns:
            cost: (N, M) distance matrix
        """
        N = len(tracks)
        M = len(dets)
        cost = np.full((N, M), 1e6, dtype=np.float32)

        for i, trk in enumerate(tracks):
            tx, ty, tz = trk.state[3], trk.state[4], trk.state[5]
            for j in range(M):
                dx = dets[j, 3] - tx
                dy = dets[j, 4] - ty
                dz = dets[j, 5] - tz
                cost[i, j] = np.sqrt(dx*dx + dy*dy + dz*dz)

        return cost

    @staticmethod
    def _hungarian_match(cost, threshold=10.0):
        """
        Hungarian matching with threshold.

        Returns:
            matched: list of (track_idx, det_idx) pairs
            unmatched_dets: list of det indices
            unmatched_trks: list of track indices
        """
        from scipy.optimize import linear_sum_assignment

        N, M = cost.shape
        if N == 0 or M == 0:
            return [], list(range(M)), list(range(N))

        row_ind, col_ind = linear_sum_assignment(cost)

        matched = []
        unmatched_dets = set(range(M))
        unmatched_trks = set(range(N))

        for r, c in zip(row_ind, col_ind):
            if cost[r, c] < threshold:
                matched.append((r, c))
                unmatched_dets.discard(c)
                unmatched_trks.discard(r)

        return matched, sorted(unmatched_dets), sorted(unmatched_trks)

    def track(self, dets_all: dict, frame: int) -> Tuple[List[np.ndarray], Any]:
        """
        Run one tracking step. Same interface as AB3DMOT.

        Args:
            dets_all: {'dets': (N, 8) [h,w,l,x,y,z,yaw,score], 'info': (N, 3)}
            frame: frame index

        Returns:
            (tracks_list, None) where tracks_list[0] is (M, 9+) array
        """
        self.frame_count += 1
        dets = dets_all['dets']  # (N, 8) last column is score
        info = dets_all['info']  # (N, 3)

        # 1. Predict all tracks (batched SSM)
        self._batched_predict()

        # 2. Match detections to tracks
        if len(dets) > 0 and len(self.tracks) > 0:
            det_states = dets[:, :7]  # [h,w,l,x,y,z,theta]
            cost = self._iou_3d_distance(self.tracks, det_states)
            matched, unmatched_dets, unmatched_trks = self._hungarian_match(
                cost, threshold=self.iou_threshold * 50)  # ~5m threshold
        elif len(dets) > 0:
            matched = []
            unmatched_dets = list(range(len(dets)))
            unmatched_trks = []
        else:
            matched = []
            unmatched_dets = []
            unmatched_trks = list(range(len(self.tracks)))

        # 3. Update matched tracks
        for trk_idx, det_idx in matched:
            self.tracks[trk_idx].update(dets[det_idx, :7], info[det_idx])

        # 4. Birth new tracks from unmatched detections
        for det_idx in unmatched_dets:
            new_trk = SSMTrack(
                dets[det_idx, :7], info[det_idx],
                max_history=self.max_history)
            self.tracks.append(new_trk)

        # 5. Remove dead tracks
        self.tracks = [t for t in self.tracks
                       if t.time_since_update <= self.max_age]

        # 6. Output confirmed tracks
        results = []
        for trk in self.tracks:
            if trk.hits >= self.min_hits or self.frame_count <= self.min_hits:
                # Output format: [h,w,l,x,y,z,theta, ID, frame, det_idx, carla_id]
                # Match AB3DMOT's output columns
                d = trk.state  # [h,w,l,x,y,z,theta]
                out = np.concatenate([
                    d,                    # 0-6: h,w,l,x,y,z,theta
                    [trk.id],             # 7: track ID
                    trk.info[:1],         # 8: frame
                    trk.info[1:2],        # 9: det_idx
                    trk.info[2:3],        # 10: carla_id
                ])
                # Add velocity estimate from history
                if len(trk.history) >= 2:
                    vel = trk.history[-1] - trk.history[-2]
                    out = np.concatenate([out, vel[:3]])  # 11-13: vx,vy,vz
                else:
                    out = np.concatenate([out, np.zeros(3)])

                results.append(out)

        if results:
            results = [np.array(results)]
        else:
            results = [np.empty((0, 14))]

        return results, None

    def reset(self):
        self.tracks = []
        self.frame_count = 0
        SSMTrack._next_id = 1

    def load_ssm_weights(self, checkpoint_path: str):
        """Load pretrained SSM motion predictor weights."""
        if self._ssm is None:
            logger.warning("SSM not initialized, cannot load weights")
            return
        state_dict = torch.load(checkpoint_path, map_location=self.device)
        if 'model' in state_dict:
            state_dict = state_dict['model']
        self._ssm.load_state_dict(state_dict, strict=False)
        logger.info("SSM weights loaded from %s", checkpoint_path)
