# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Tracking Metrics

Tracks multi-object tracking quality metrics: MOTA, MOTP, IDF1, ID switches,
fragmentations. Used by edge managers to evaluate tracking performance.
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Set, Tuple
from collections import deque, defaultdict


@dataclass
class TrackingFrame:
    """Metrics for a single tracking frame"""
    frame_id: int
    num_tracks: int = 0
    num_ground_truth: int = 0
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    id_switches: int = 0
    fragmentations: int = 0
    mota: float = 0.0
    motp: float = 0.0  # Average localization error (meters)


class TrackingMetrics:
    """
    Accumulates multi-object tracking metrics over time.

    Implements standard MOT metrics:
    - MOTA: Multi-Object Tracking Accuracy = 1 - (FN + FP + IDSW) / GT
    - MOTP: Multi-Object Tracking Precision = avg localization error of matched tracks
    - IDF1: ID F1 Score = 2*IDTP / (2*IDTP + IDFP + IDFN)
    - ID Switches: Number of times a track ID changes for same GT object
    - Fragmentations: Number of times a track is interrupted

    Parameters
    ----------
    history_size : int
        Number of frames to keep in history
    distance_threshold : float
        Maximum distance (meters) for track-to-GT matching
    """

    def __init__(
        self,
        history_size: int = 1000,
        distance_threshold: float = 2.0
    ):
        self.history_size = history_size
        self.distance_threshold = distance_threshold
        self.frame_history: deque = deque(maxlen=history_size)

        # Track-to-GT mapping persistence (for ID switch detection)
        self._prev_track_to_gt: Dict[int, int] = {}
        self._gt_to_track_history: Dict[int, List[int]] = defaultdict(list)

        # Accumulated counts
        self._total_gt = 0
        self._total_tp = 0
        self._total_fp = 0
        self._total_fn = 0
        self._total_id_switches = 0
        self._total_fragmentations = 0
        self._total_distance = 0.0
        self._total_matches = 0

        # For IDF1 calculation
        self._id_tp = 0  # Correctly identified detections
        self._id_fp = 0  # Detections with wrong ID
        self._id_fn = 0  # Missed GT with known ID

        self.total_frames = 0

    def update(
        self,
        tracks: List[Tuple[int, np.ndarray]],  # [(track_id, position), ...]
        ground_truth: List[Tuple[int, np.ndarray]],  # [(gt_id, position), ...]
        frame_id: int = 0
    ) -> TrackingFrame:
        """
        Update metrics with a new frame of tracks.

        Parameters
        ----------
        tracks : List[Tuple[int, np.ndarray]]
            List of (track_id, position) tuples where position is [x, y, z, ...]
        ground_truth : List[Tuple[int, np.ndarray]]
            List of (gt_id, position) tuples
        frame_id : int
            Frame identifier

        Returns
        -------
        TrackingFrame
            Metrics for this frame
        """
        self.total_frames += 1

        # Extract IDs and positions
        track_ids = [t[0] for t in tracks]
        track_positions = [np.array(t[1][:3]) for t in tracks]
        gt_ids = [g[0] for g in ground_truth]
        gt_positions = [np.array(g[1][:3]) for g in ground_truth]

        num_tracks = len(tracks)
        num_gt = len(ground_truth)
        self._total_gt += num_gt

        # Match tracks to ground truth
        track_to_gt, distances = self._match_tracks(
            track_ids, track_positions,
            gt_ids, gt_positions
        )

        # Count TP, FP, FN
        tp = len(track_to_gt)
        fp = num_tracks - tp
        fn = num_gt - tp

        self._total_tp += tp
        self._total_fp += fp
        self._total_fn += fn

        # Calculate MOTP (average localization error)
        if distances:
            motp = np.mean(distances)
            self._total_distance += sum(distances)
            self._total_matches += len(distances)
        else:
            motp = 0.0

        # Detect ID switches
        id_switches = self._detect_id_switches(track_to_gt, gt_ids)
        self._total_id_switches += id_switches

        # Detect fragmentations
        fragmentations = self._detect_fragmentations(track_to_gt, gt_ids)
        self._total_fragmentations += fragmentations

        # Update IDF1 accumulators
        self._update_idf1(track_to_gt, track_ids, gt_ids)

        # Calculate MOTA for this frame
        if num_gt > 0:
            mota = 1.0 - (fn + fp + id_switches) / num_gt
        else:
            mota = 1.0 if num_tracks == 0 else 0.0

        # Update track-to-GT mapping for next frame
        self._prev_track_to_gt = track_to_gt.copy()

        # Update GT history
        for gt_id in gt_ids:
            matched_track = None
            for tid, gid in track_to_gt.items():
                if gid == gt_id:
                    matched_track = tid
                    break
            self._gt_to_track_history[gt_id].append(matched_track)

        frame = TrackingFrame(
            frame_id=frame_id,
            num_tracks=num_tracks,
            num_ground_truth=num_gt,
            true_positives=tp,
            false_positives=fp,
            false_negatives=fn,
            id_switches=id_switches,
            fragmentations=fragmentations,
            mota=mota,
            motp=motp,
        )

        self.frame_history.append(frame)
        return frame

    def _match_tracks(
        self,
        track_ids: List[int],
        track_positions: List[np.ndarray],
        gt_ids: List[int],
        gt_positions: List[np.ndarray]
    ) -> Tuple[Dict[int, int], List[float]]:
        """
        Match tracks to ground truth using Hungarian-like greedy matching.

        Returns
        -------
        tuple
            (track_to_gt_mapping, list of match distances)
        """
        track_to_gt = {}
        distances = []

        if not track_positions or not gt_positions:
            return track_to_gt, distances

        # Build cost matrix
        cost_matrix = np.zeros((len(track_positions), len(gt_positions)))
        for i, t_pos in enumerate(track_positions):
            for j, g_pos in enumerate(gt_positions):
                cost_matrix[i, j] = np.linalg.norm(t_pos[:2] - g_pos[:2])

        # Greedy matching (prefer maintaining previous assignments)
        gt_matched = set()
        track_matched = set()

        # First pass: try to maintain previous mappings
        for track_idx, track_id in enumerate(track_ids):
            if track_id in self._prev_track_to_gt:
                prev_gt_id = self._prev_track_to_gt[track_id]
                if prev_gt_id in gt_ids:
                    gt_idx = gt_ids.index(prev_gt_id)
                    if gt_idx not in gt_matched:
                        dist = cost_matrix[track_idx, gt_idx]
                        if dist < self.distance_threshold:
                            track_to_gt[track_id] = prev_gt_id
                            distances.append(dist)
                            gt_matched.add(gt_idx)
                            track_matched.add(track_idx)

        # Second pass: match remaining
        for track_idx, track_id in enumerate(track_ids):
            if track_idx in track_matched:
                continue

            best_dist = float('inf')
            best_gt_idx = -1

            for gt_idx in range(len(gt_positions)):
                if gt_idx in gt_matched:
                    continue
                dist = cost_matrix[track_idx, gt_idx]
                if dist < best_dist and dist < self.distance_threshold:
                    best_dist = dist
                    best_gt_idx = gt_idx

            if best_gt_idx >= 0:
                track_to_gt[track_id] = gt_ids[best_gt_idx]
                distances.append(best_dist)
                gt_matched.add(best_gt_idx)

        return track_to_gt, distances

    def _detect_id_switches(
        self,
        track_to_gt: Dict[int, int],
        gt_ids: List[int]
    ) -> int:
        """Count ID switches from previous frame."""
        id_switches = 0

        # Check each GT that was matched in previous frame
        for prev_track_id, prev_gt_id in self._prev_track_to_gt.items():
            if prev_gt_id not in gt_ids:
                continue  # GT no longer visible

            # Find current track matched to this GT
            current_track_id = None
            for tid, gid in track_to_gt.items():
                if gid == prev_gt_id:
                    current_track_id = tid
                    break

            # ID switch if different track now matches same GT
            if current_track_id is not None and current_track_id != prev_track_id:
                id_switches += 1

        return id_switches

    def _detect_fragmentations(
        self,
        track_to_gt: Dict[int, int],
        gt_ids: List[int]
    ) -> int:
        """Count fragmentations (track gaps)."""
        fragmentations = 0

        for gt_id in gt_ids:
            history = self._gt_to_track_history.get(gt_id, [])
            if len(history) >= 2:
                # Fragmentation: was tracked, then lost, now tracked again
                prev_track = history[-1] if history else None
                was_matched = any(gid == gt_id for gid in track_to_gt.values())

                if prev_track is None and was_matched:
                    # Check if there was a gap (lost then found)
                    if len(history) >= 2 and history[-2] is not None:
                        fragmentations += 1

        return fragmentations

    def _update_idf1(
        self,
        track_to_gt: Dict[int, int],
        track_ids: List[int],
        gt_ids: List[int]
    ):
        """Update IDF1 accumulators."""
        # IDTP: Correct associations
        for track_id, gt_id in track_to_gt.items():
            if track_id in self._prev_track_to_gt:
                if self._prev_track_to_gt[track_id] == gt_id:
                    self._id_tp += 1
                else:
                    self._id_fp += 1  # Wrong ID
            else:
                self._id_tp += 1  # New correct association

        # IDFN: Missed GT objects
        matched_gts = set(track_to_gt.values())
        for gt_id in gt_ids:
            if gt_id not in matched_gts:
                self._id_fn += 1

    def get_metrics(self) -> Dict:
        """
        Get aggregated tracking metrics.

        Returns
        -------
        dict
            Dictionary with all tracking metrics
        """
        if not self.frame_history:
            return {
                'total_frames': 0,
                'mota': 0.0,
                'motp': 0.0,
                'idf1': 0.0,
                'total_id_switches': 0,
                'total_fragmentations': 0,
                'total_tp': 0,
                'total_fp': 0,
                'total_fn': 0,
                'precision': 0.0,
                'recall': 0.0,
            }

        # MOTA = 1 - (FN + FP + IDSW) / total_GT
        if self._total_gt > 0:
            mota = 1.0 - (self._total_fn + self._total_fp + self._total_id_switches) / self._total_gt
        else:
            mota = 0.0

        # MOTP = avg localization error
        motp = self._total_distance / self._total_matches if self._total_matches > 0 else 0.0

        # IDF1 = 2 * IDTP / (2 * IDTP + IDFP + IDFN)
        idf1_denom = 2 * self._id_tp + self._id_fp + self._id_fn
        idf1 = 2 * self._id_tp / idf1_denom if idf1_denom > 0 else 0.0

        # Precision/Recall
        precision = self._total_tp / (self._total_tp + self._total_fp) if (self._total_tp + self._total_fp) > 0 else 0.0
        recall = self._total_tp / (self._total_tp + self._total_fn) if (self._total_tp + self._total_fn) > 0 else 0.0

        return {
            'total_frames': len(self.frame_history),
            'mota': mota,
            'motp': motp,
            'idf1': idf1,
            'total_id_switches': self._total_id_switches,
            'total_fragmentations': self._total_fragmentations,
            'total_tp': self._total_tp,
            'total_fp': self._total_fp,
            'total_fn': self._total_fn,
            'precision': precision,
            'recall': recall,
        }

    def evaluate(self):
        """
        Generate evaluation figure and text summary.

        Returns
        -------
        tuple
            (matplotlib.figure.Figure, str, dict)
        """
        import matplotlib.pyplot as plt
        import warnings
        warnings.filterwarnings('ignore')

        metrics = self.get_metrics()
        frames = list(self.frame_history)

        fig, axes = plt.subplots(2, 2, figsize=(12, 10))

        if frames:
            frame_ids = [f.frame_id for f in frames]

            # 1. MOTA over time
            motas = [f.mota for f in frames]
            axes[0, 0].plot(frame_ids, motas, 'g-', alpha=0.7)
            axes[0, 0].axhline(y=metrics['mota'], color='darkgreen', linestyle='--',
                              label=f"Avg MOTA: {metrics['mota']:.3f}")
            axes[0, 0].set_xlabel('Frame')
            axes[0, 0].set_ylabel('MOTA')
            axes[0, 0].set_title('Multi-Object Tracking Accuracy')
            axes[0, 0].set_ylim([-1, 1.1])
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

            # 2. MOTP over time (localization error)
            motps = [f.motp for f in frames]
            axes[0, 1].plot(frame_ids, motps, 'b-', alpha=0.7)
            axes[0, 1].axhline(y=metrics['motp'], color='darkblue', linestyle='--',
                              label=f"Avg MOTP: {metrics['motp']:.2f}m")
            axes[0, 1].set_xlabel('Frame')
            axes[0, 1].set_ylabel('MOTP (meters)')
            axes[0, 1].set_title('Multi-Object Tracking Precision (Localization Error)')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)

            # 3. ID Switches and Fragmentations
            id_switches = [f.id_switches for f in frames]
            frags = [f.fragmentations for f in frames]
            axes[1, 0].bar(frame_ids, id_switches, alpha=0.7, label='ID Switches', color='red')
            axes[1, 0].bar(frame_ids, frags, bottom=id_switches, alpha=0.7,
                          label='Fragmentations', color='orange')
            axes[1, 0].set_xlabel('Frame')
            axes[1, 0].set_ylabel('Count')
            axes[1, 0].set_title(f"ID Switches: {metrics['total_id_switches']}, Frags: {metrics['total_fragmentations']}")
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)

            # 4. Summary metrics bar chart
            metric_names = ['MOTA', 'IDF1', 'Precision', 'Recall']
            metric_values = [max(0, metrics['mota']), metrics['idf1'],
                           metrics['precision'], metrics['recall']]
            colors = ['green', 'blue', 'purple', 'orange']
            bars = axes[1, 1].bar(metric_names, metric_values, color=colors, alpha=0.8)
            axes[1, 1].set_ylabel('Score')
            axes[1, 1].set_title('Tracking Performance Summary')
            axes[1, 1].set_ylim([0, 1.0])
            for bar, val in zip(bars, metric_values):
                axes[1, 1].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                               f'{val:.3f}', ha='center', va='bottom', fontsize=10)
            axes[1, 1].grid(True, alpha=0.3, axis='y')

        fig.suptitle('Tracking Metrics Evaluation')
        fig.tight_layout()

        perform_txt = f"""
Tracking Metrics Summary:
  Total Frames: {metrics['total_frames']}
  MOTA: {metrics['mota']:.4f}
  MOTP: {metrics['motp']:.4f} meters
  IDF1: {metrics['idf1']:.4f}
  ID Switches: {metrics['total_id_switches']}
  Fragmentations: {metrics['total_fragmentations']}
  True Positives: {metrics['total_tp']}
  False Positives: {metrics['total_fp']}
  False Negatives: {metrics['total_fn']}
  Precision: {metrics['precision']:.4f}
  Recall: {metrics['recall']:.4f}
"""

        return fig, perform_txt, metrics

    def reset(self):
        """Reset all accumulated metrics."""
        self.frame_history.clear()
        self._prev_track_to_gt.clear()
        self._gt_to_track_history.clear()
        self._total_gt = 0
        self._total_tp = 0
        self._total_fp = 0
        self._total_fn = 0
        self._total_id_switches = 0
        self._total_fragmentations = 0
        self._total_distance = 0.0
        self._total_matches = 0
        self._id_tp = 0
        self._id_fp = 0
        self._id_fn = 0
        self.total_frames = 0
