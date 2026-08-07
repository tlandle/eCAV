"""
Mamba3DTracker: MambaTrack-based 3D multi-object tracker.

Two-round association (active + lost tracklets) with MTP predictions,
following MambaTrack (Xiao et al., ACM MM 2024) Algorithm 1.

Adapted from ByteTrack/MambaTrack 2D to 3D bounding boxes.
"""

import numpy as np
import torch
import logging
from .basetrack import BaseTrack, TrackState
from .tracklet import MambaTracklet3D
from .matching import center_distance_3d, iou_distance_3d, linear_assignment

logger = logging.getLogger("Mamba3DTracker")


def joint_tracklets(t1, t2):
    exists = set()
    res = []
    for t in t1:
        exists.add(t.track_id)
        res.append(t)
    for t in t2:
        if t.track_id not in exists:
            exists.add(t.track_id)
            res.append(t)
    return res


class Mamba3DTracker:
    """
    3D MOT using MambaTrack motion prediction with two-round association.

    Round 1: active tracklets vs all detections (MTP predicted positions)
    Round 2: lost tracklets vs remaining detections (autoregressive MTP)
    """

    def __init__(self, cfgs, device):
        self.tracked_tracklets = []
        self.lost_tracklets = []
        self.removed_tracklets = []

        self.frame_id = 0
        self.cfgs = cfgs
        self.device = device

        self.det_thresh = self.cfgs.get('filter_thresh', 0.2)
        self.new_track_thresh = self.cfgs.get('new_track_thresh', 0.3)
        self.max_time_lost = self.cfgs.get('max_time_lost', 30)
        self.match_thresh = self.cfgs.get('match_thresh', 0.7)

        # Initialize shared motion predictor
        ckpt_path = self.cfgs.get('motion_model_path', None)
        MambaTracklet3D.set_motion_predictor(
            cfgs=self.cfgs, device=device, ckpt_path=ckpt_path)

    def update(self, dets_3d, scores):
        """
        Update tracker with new detections.

        Args:
            dets_3d: (N, 7) array [x, y, z, l, w, h, yaw]
            scores: (N,) confidence scores

        Returns:
            list of active tracklets
        """
        self.frame_id += 1
        activated_tracklets = []
        refind_tracklets = []
        lost_tracklets = []
        removed_tracklets = []

        # Filter low-confidence detections
        remain_inds = scores > self.det_thresh
        dets = dets_3d[remain_inds]
        scores_keep = scores[remain_inds]

        # Create detection tracklets
        if len(dets) > 0:
            detections = [
                MambaTracklet3D(self.cfgs, bbox, s, self.device)
                for bbox, s in zip(dets, scores_keep)
            ]
        else:
            detections = []

        # --- Round 1: match active tracklets with detections ---

        # Predict positions for all active and lost tracklets in one batched pass
        tracklet_pool = joint_tracklets(self.tracked_tracklets, self.lost_tracklets)
        self._batched_predict(tracklet_pool)

        # Compute BEV IoU cost matrix (1 - IoU)
        dists = iou_distance_3d(self.tracked_tracklets, detections)
        matches, u_track, u_detection = linear_assignment(
            dists, thresh=self.match_thresh)

        # Update matched active tracklets
        for itracked, idet in matches:
            track = self.tracked_tracklets[itracked]
            det = detections[idet]
            track.update(det, self.frame_id)
            activated_tracklets.append(track)

        # --- Round 2: match lost tracklets with remaining detections ---

        detections_remain = [detections[i] for i in u_detection]
        # Lost-pool recapture uses CENTER DISTANCE, not IoU: after multi-
        # frame detection dropouts the coasted prediction has drifted past
        # box overlap (movers decelerate; CV overshoots), and IoU-only
        # matching can never recapture them -> permanent fragmentation.
        # Same principle as AB3DMOT's ground-plane distance gate.
        lost_gate_m = float(self.cfgs.get('lost_match_dist_m', 5.0))
        dists_lost = center_distance_3d(self.lost_tracklets, detections_remain)
        matches_lost, u_lost, u_det_remain = linear_assignment(
            dists_lost, thresh=min(lost_gate_m / 20.0, 0.999))

        for ilost, idet in matches_lost:
            track = self.lost_tracklets[ilost]
            det = detections_remain[idet]
            track.re_activate(det, self.frame_id, new_id=False)
            refind_tracklets.append(track)

        # Mark unmatched active tracklets as lost
        for it in u_track:
            track = self.tracked_tracklets[it]
            # TEMP DEBUG: why did this track miss? min cost to any det and
            # whether its best det went to another track (competition).
            if dists.shape[1] > 0:
                from ecav.core.tracking.mamba3dmot.matching import _extract_bev_box
                tb = _extract_bev_box(track)
                dboxes = [_extract_bev_box(d) for d in detections]
                dcent = [((db[0]-tb[0])**2 + (db[1]-tb[1])**2)**0.5 for db in dboxes]
                jn = int(np.argmin(dcent))
                db = dboxes[jn]
                mb = track.memo_bank[-1] if len(track.memo_bank) else None
                print(f"[MAMBA LOST] tid={track.track_id} "
                      f"min_cost={dists[it].min():.3f} "
                      f"trk_bev=({tb[0]:.1f},{tb[1]:.1f},l{tb[2]:.1f},w{tb[3]:.1f},yaw{tb[4]:.2f}) "
                      f"near_det=({db[0]:.1f},{db[1]:.1f},l{db[2]:.1f},w{db[3]:.1f},yaw{db[4]:.2f}) "
                      f"cdist={dcent[jn]:.2f} "
                      f"memo_last=({mb[0]:.1f},{mb[1]:.1f})" if mb is not None else "no-memo")
            else:
                print(f"[MAMBA LOST] tid={track.track_id} no_dets")
            if not track.state_flag == TrackState.Lost:
                track.state_flag = TrackState.Lost
                lost_tracklets.append(track)

        # Remove long-lost tracklets
        for track in self.lost_tracklets:
            if self.frame_id - track.frame_id > self.max_time_lost:
                track.state_flag = TrackState.Removed
                removed_tracklets.append(track)

        # Initialize new tracklets from unmatched high-confidence detections
        for idet in u_det_remain:
            det_idx = u_detection[idet]
            det = detections[det_idx]
            if det.score >= self.new_track_thresh:
                det.activate(self.frame_id)
                activated_tracklets.append(det)

        # Update tracker state
        self.tracked_tracklets = [
            t for t in joint_tracklets(activated_tracklets, refind_tracklets)
            if t.state_flag == TrackState.Tracked
        ]
        self.lost_tracklets = [
            t for t in joint_tracklets(
                [t for t in self.lost_tracklets
                 if t.state_flag != TrackState.Removed],
                lost_tracklets)
        ]
        self.removed_tracklets.extend(removed_tracklets)

        return self.tracked_tracklets

    @torch.no_grad()
    def _batched_predict(self, tracklets):
        """Run motion prediction for all tracklets in a single batched Mamba forward pass.

        Tracklets with insufficient history fall back to linear extrapolation
        (handled individually, cheap). The rest are batched into one GPU call.
        """
        if not tracklets:
            return

        enable_thresh = self.cfgs.get('enable_time_thresh', 5)
        norm_scale = MambaTracklet3D.norm_scale
        clamp_val = MambaTracklet3D.clamp_val
        indices = MambaTracklet3D.motion_indices
        motion_dim = len(indices) if indices is not None else 7

        short_hist = []  # fallback to linear extrapolation
        long_hist = []   # batched Mamba inference

        for t in tracklets:
            if len(t.memo_bank) < enable_thresh:
                short_hist.append(t)
            else:
                long_hist.append(t)

        # Fallback path: linear extrapolation (cheap, per-tracklet)
        for t in short_hist:
            t.predict()

        if not long_hist:
            return

        # Build batched input: pad at the start (older frames) with zeros
        max_len = max(len(t.diff_memo_bank) - 1 for t in long_hist)
        batch = np.zeros((len(long_hist), max_len, motion_dim), dtype=np.float32)

        for i, t in enumerate(long_hist):
            hist = np.array(t.diff_memo_bank[1:], dtype=np.float32)
            if indices is not None:
                hist = hist[:, indices]
            if norm_scale is not None:
                hist = np.clip(hist / norm_scale, -clamp_val, clamp_val)
            seq_len = len(hist)
            batch[i, max_len - seq_len:, :] = hist

        # Single batched forward pass
        batch_tensor = torch.from_numpy(batch).to(self.device)
        out = MambaTracklet3D.motion_predictor(batch_tensor)
        # out shape: (batch, 1, motion_dim) from AdaptiveAvgPool2d
        out = out.squeeze(1) if out.dim() == 3 else out
        out_np = out.detach().cpu().numpy()

        # Denormalize
        if norm_scale is not None:
            out_np = out_np * norm_scale

        # Assign predictions back to tracklets
        for i, t in enumerate(long_hist):
            delta = out_np[i]
            pred = t.memo_bank[-1].copy()
            if indices is not None:
                for j, idx in enumerate(indices):
                    pred[idx] += delta[j]
            else:
                pred += delta
            t.predicted_last_bbox = pred
            t.time_since_update += 1
