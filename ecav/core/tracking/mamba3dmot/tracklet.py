"""
MambaTracklet3D: 3D tracklet with Mamba motion prediction.

Adapted from MambaTracklet (2D) to 3D bounding boxes.
3D bbox format: [x, y, z, l, w, h, yaw] (7-dim)

The motion model predicts only [dx, dy, dz, dyaw] (4-dim).
l/w/h are preserved from the last observation.
"""
import numpy as np
from .basetrack import BaseTrack, TrackState
import torch
import logging

logger = logging.getLogger("MambaTracklet3D")

from .MambaTrack import MambaTrack

BOX_DIM = 7  # 3D: x, y, z, l, w, h, yaw
DEFAULT_MOTION_INDICES = [0, 1, 2, 6]  # x, y, z, yaw


class MambaTracklet3D(BaseTrack):
    motion_predictor = None
    norm_scale = None
    clamp_val = 1.0
    motion_indices = None

    @staticmethod
    def set_motion_predictor(cfgs, device, ckpt_path=None):
        logger.info('Initializing MambaTrack3D motion predictor')
        model_cfgs = dict(cfgs)
        MambaTracklet3D.norm_scale = None
        MambaTracklet3D.clamp_val = 5.0
        MambaTracklet3D.motion_indices = DEFAULT_MOTION_INDICES

        if ckpt_path:
            ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
            # Use checkpoint's model architecture config
            if 'cfgs' in ckpt:
                for k in ['box_dim', 'd_m', 'd_state', 'L',
                           'avg_pool_out_dim', 'pred_head_dims']:
                    if k in ckpt['cfgs']:
                        model_cfgs[k] = ckpt['cfgs'][k]
            MambaTracklet3D.norm_scale = ckpt.get('norm_scale')
            MambaTracklet3D.clamp_val = ckpt.get('clamp_val', 5.0)
            MambaTracklet3D.motion_indices = ckpt.get(
                'motion_indices', DEFAULT_MOTION_INDICES)
            logger.info('Checkpoint norm_scale=%s, clamp_val=%s, motion_indices=%s',
                        MambaTracklet3D.norm_scale,
                        MambaTracklet3D.clamp_val,
                        MambaTracklet3D.motion_indices)

        model = MambaTrack(model_cfgs).to(device)
        model.eval()

        if ckpt_path:
            state = ckpt.get('model', ckpt)
            model.load_state_dict(state, strict=False)
            logger.info('Loaded MambaTrack3D checkpoint: %s', ckpt_path)

        MambaTracklet3D.motion_predictor = model

    def __init__(self, cfgs, bbox_3d, score, device):
        """
        Args:
            cfgs: config dict
            bbox_3d: [x, y, z, l, w, h, yaw] (7-dim)
            score: detection confidence
            device: torch device
        """
        self.cfgs = cfgs

        self._bbox_3d = np.asarray(bbox_3d[:BOX_DIM], dtype=np.float32)
        self.is_activated = False
        self.score = score

        self.predicted_last_bbox = None

        # History buffers
        self.memo_bank = [self._bbox_3d.copy()]
        self.diff_memo_bank = [np.zeros(BOX_DIM, dtype=np.float32)]

        self.device = device

    @staticmethod
    def _wrap_yaw_diff(diff):
        """Wrap yaw component of a 7-dim diff to [-pi, pi]."""
        diff[6] = np.arctan2(np.sin(diff[6]), np.cos(diff[6]))
        return diff

    @property
    def center(self):
        """Get 3D center [x, y, z]."""
        if self.predicted_last_bbox is not None:
            return self.predicted_last_bbox[:3]
        return self._bbox_3d[:3]

    @property
    def state(self):
        """Current 3D bbox state."""
        if self.predicted_last_bbox is not None:
            return self.predicted_last_bbox
        return self._bbox_3d

    def get_bbox(self):
        if len(self.memo_bank):
            return self.memo_bank[-1].copy()
        return self._bbox_3d.copy()

    @torch.no_grad()
    def predict(self):
        """Predict next state using MambaTrack motion model."""
        enable_thresh = self.cfgs.get('enable_time_thresh', 5)
        if len(self.memo_bank) < enable_thresh:
            # Short history: constant-velocity extrapolation. Use the MEAN of
            # the last few inter-frame deltas, not just the last one: a single
            # delta is jitter-dominated and, for a fast vehicle re-acquired
            # after a gap, the last delta can be a small acceleration-phase
            # step, making the coast under-shoot and the track fall behind
            # (measured: 8 m/s oncoming coasted at ~1/4 speed -> fragmented).
            pred = self.memo_bank[-1].copy()
            if len(self.memo_bank) >= 2:
                # Velocity from the memo-bank ENDPOINTS (net displacement over
                # the window), not the last few inter-frame diffs. WorldFusion
                # detections are jitter-dominated, so a mean of the last 3
                # diffs can be near-zero or even wrong-signed for a steadily
                # moving vehicle; averaging over the whole window recovers the
                # true velocity because real displacement dominates jitter.
                first = np.asarray(self.memo_bank[0], dtype=np.float32)
                last = np.asarray(self.memo_bank[-1], dtype=np.float32)
                span = max(len(self.memo_bank) - 1, 1)
                vel = (last - first) / span
                # Wrap yaw delta to avoid wrap-around artifacts
                vel[6] = np.arctan2(np.sin(vel[6]), np.cos(vel[6]))
                # Dead-reckon through a detection gap: advance by vel * the
                # number of steps since the last OBSERVATION, not a single
                # step. Without this a coasting track freezes one step ahead
                # of its last detection (predicted_last_bbox recomputed from
                # memo_bank[-1] every tick), so an obstacle that is occluded
                # after a locale handoff never moves downtrack and the
                # migrated prediction is useless. steps = tsu+1 because tsu is
                # incremented at the end of this call.
                steps = self.time_since_update + 1
                pred[0] += vel[0] * steps
                pred[1] += vel[1] * steps
                pred[2] += vel[2] * steps
                pred[6] += vel[6]
        else:
            hist_diff = np.array(self.diff_memo_bank[1:], dtype=np.float32)

            indices = MambaTracklet3D.motion_indices
            norm_scale = MambaTracklet3D.norm_scale
            clamp_val = MambaTracklet3D.clamp_val

            # Select motion dimensions and normalize
            if indices is not None:
                hist_motion = hist_diff[:, indices]
            else:
                hist_motion = hist_diff

            if norm_scale is not None:
                hist_motion = np.clip(
                    hist_motion / norm_scale, -clamp_val, clamp_val)
            else:
                scale_factor = self.cfgs.get('scale_factor', 1.0)
                hist_motion = hist_motion * scale_factor

            hist_tensor = torch.tensor(
                hist_motion, dtype=torch.float32
            ).unsqueeze(0).to(self.device)

            out = MambaTracklet3D.motion_predictor(hist_tensor).squeeze()
            out = out.detach().cpu().numpy()

            # Denormalize
            if norm_scale is not None:
                out = out * norm_scale
            else:
                out /= self.cfgs.get('scale_factor', 1.0)

            # Apply predicted delta to last known state
            pred = self.memo_bank[-1].copy()
            if indices is not None:
                for i, idx in enumerate(indices):
                    pred[idx] += out[i]
            else:
                pred += out

        self.time_since_update += 1
        self.predicted_last_bbox = pred

    def activate(self, frame_id):
        self.track_id = self.next_id()
        self.state_flag = TrackState.Tracked
        if frame_id == 1:
            self.is_activated = True
        self.frame_id = frame_id
        self.start_frame = frame_id

    def re_activate(self, new_track, frame_id, new_id=False):
        diff = new_track._bbox_3d - self.memo_bank[-1]
        self.diff_memo_bank.append(self._wrap_yaw_diff(diff))
        self.memo_bank.append(new_track._bbox_3d.copy())
        self._bbox_3d = new_track._bbox_3d.copy()  # last-observed box

        max_window = self.cfgs.get('max_window', 10)
        if len(self.memo_bank) > max_window:
            self.memo_bank = self.memo_bank[1:]
            self.diff_memo_bank = self.diff_memo_bank[1:]

        self.state_flag = TrackState.Tracked
        self.is_activated = True
        self.frame_id = frame_id
        self.time_since_update = 0  # observed this frame; reset coast counter
        if new_id:
            self.track_id = self.next_id()
        self.score = new_track.score

    def update(self, new_track, frame_id):
        if new_track is None:
            diff = self.predicted_last_bbox - self.memo_bank[-1]
            self.diff_memo_bank.append(self._wrap_yaw_diff(diff))
            self.memo_bank.append(self.predicted_last_bbox.copy())
        else:
            self.frame_id = frame_id
            diff = new_track._bbox_3d - self.memo_bank[-1]
            self.diff_memo_bank.append(self._wrap_yaw_diff(diff))
            self.memo_bank.append(new_track._bbox_3d.copy())
            # Keep the last-observed box current: _bbox_3d was set once at
            # creation and never refreshed, so distance-based association
            # (center_distance_3d) matched against a stale birth position.
            self._bbox_3d = new_track._bbox_3d.copy()
            self.score = new_track.score

        max_window = self.cfgs.get('max_window', 10)
        if len(self.memo_bank) > max_window:
            self.memo_bank = self.memo_bank[1:]
            self.diff_memo_bank = self.diff_memo_bank[1:]

        self.state_flag = TrackState.Tracked
        self.is_activated = True
        self.time_since_update = 0
