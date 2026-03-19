# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Detection Metrics

Tracks detection quality metrics: TP/FP/FN, precision, recall, AP at multiple
IoU thresholds. Used by edge managers to evaluate detection performance.
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple
from collections import deque


@dataclass
class DetectionFrame:
    """Metrics for a single detection frame"""
    frame_id: int
    true_positives: int = 0
    false_positives: int = 0
    false_negatives: int = 0
    num_detections: int = 0
    num_ground_truth: int = 0
    precision: float = 0.0
    recall: float = 0.0
    # Per-threshold AP (computed over accumulated frames)
    ap_30: float = 0.0
    ap_50: float = 0.0
    ap_70: float = 0.0


class DetectionMetrics:
    """
    Accumulates detection metrics over time for evaluation.

    Tracks true positives, false positives, and false negatives using
    distance-based matching between detections and ground truth.
    Computes AP at multiple IoU thresholds (0.3, 0.5, 0.7).

    Parameters
    ----------
    history_size : int
        Number of frames to keep in history
    distance_thresholds : dict
        Distance thresholds for matching at different "IoU" levels
        Default: {0.3: 4.0m, 0.5: 2.0m, 0.7: 1.0m}
    """

    def __init__(
        self,
        history_size: int = 1000,
        distance_thresholds: Optional[Dict[float, float]] = None
    ):
        self.history_size = history_size
        self.frame_history: deque = deque(maxlen=history_size)

        # Distance thresholds approximate IoU thresholds for 3D detection
        # Lower distance = stricter matching = higher effective IoU
        # Relaxed thresholds for urban driving scenarios with localization uncertainty
        self.distance_thresholds = distance_thresholds or {
            0.3: 8.0,   # Loose matching (~IoU 0.3)
            0.5: 5.0,   # Standard matching (~IoU 0.5)
            0.7: 2.5,   # Strict matching (~IoU 0.7)
        }

        # Accumulated counts for AP calculation
        self._accumulated = {
            0.3: {'tp': [], 'fp': [], 'num_gt': 0},
            0.5: {'tp': [], 'fp': [], 'num_gt': 0},
            0.7: {'tp': [], 'fp': [], 'num_gt': 0},
        }

        self.total_frames = 0

    def update(
        self,
        detections: List,
        ground_truth: List,
        frame_id: int = 0,
        confidences: Optional[List[float]] = None
    ) -> DetectionFrame:
        """
        Update metrics with a new frame of detections.

        Parameters
        ----------
        detections : List
            List of detected objects. Each can be:
            - (x, y, z, ...) tuple/list
            - Object with .location attribute
            - numpy array [x, y, z, ...]
        ground_truth : List
            List of ground truth objects (same format as detections)
        frame_id : int
            Frame identifier
        confidences : List[float], optional
            Confidence scores for each detection (for AP calculation)

        Returns
        -------
        DetectionFrame
            Metrics for this frame
        """
        self.total_frames += 1

        # Extract positions
        det_positions = self._extract_positions(detections)
        gt_positions = self._extract_positions(ground_truth)

        # Default confidences if not provided
        if confidences is None:
            confidences = [1.0] * len(det_positions)

        # Match at standard threshold (0.5) for frame metrics
        tp, fp, fn, matched_gt = self._match_detections(
            det_positions, gt_positions,
            self.distance_thresholds[0.5]
        )

        # Calculate precision/recall
        num_det = len(det_positions)
        num_gt = len(gt_positions)
        precision = tp / num_det if num_det > 0 else 0.0
        recall = tp / num_gt if num_gt > 0 else 0.0

        # Update accumulated counts for each threshold
        for iou_thresh, dist_thresh in self.distance_thresholds.items():
            tp_t, fp_t, fn_t, _ = self._match_detections(
                det_positions, gt_positions, dist_thresh, confidences
            )
            self._accumulated[iou_thresh]['num_gt'] += num_gt
            # Store per-detection TP/FP for AP calculation
            for i, conf in enumerate(confidences):
                if i < len(det_positions):
                    # Check if this detection matched GT at this threshold
                    is_tp = i < tp_t  # Simplified: assume sorted by confidence
                    self._accumulated[iou_thresh]['tp'].append((conf, is_tp))
                    self._accumulated[iou_thresh]['fp'].append((conf, not is_tp))

        # Compute current AP values
        ap_30 = self._compute_ap(0.3)
        ap_50 = self._compute_ap(0.5)
        ap_70 = self._compute_ap(0.7)

        frame = DetectionFrame(
            frame_id=frame_id,
            true_positives=tp,
            false_positives=fp,
            false_negatives=fn,
            num_detections=num_det,
            num_ground_truth=num_gt,
            precision=precision,
            recall=recall,
            ap_30=ap_30,
            ap_50=ap_50,
            ap_70=ap_70,
        )

        self.frame_history.append(frame)
        return frame

    def _extract_positions(self, objects: List) -> List[np.ndarray]:
        """Extract (x, y, z) positions from various object formats."""
        positions = []
        for obj in objects:
            if obj is None:
                continue
            if hasattr(obj, 'location'):
                loc = obj.location
                positions.append(np.array([loc.x, loc.y, loc.z]))
            elif hasattr(obj, 'get_location'):
                loc = obj.get_location()
                positions.append(np.array([loc.x, loc.y, loc.z]))
            elif isinstance(obj, np.ndarray):
                if len(obj) >= 3:
                    positions.append(obj[:3].astype(float))
            elif isinstance(obj, (list, tuple)):
                if len(obj) >= 3:
                    positions.append(np.array(obj[:3], dtype=float))
        return positions

    def _match_detections(
        self,
        det_positions: List[np.ndarray],
        gt_positions: List[np.ndarray],
        distance_threshold: float,
        confidences: Optional[List[float]] = None
    ) -> Tuple[int, int, int, set]:
        """
        Match detections to ground truth using greedy distance-based matching.

        Returns
        -------
        tuple
            (true_positives, false_positives, false_negatives, matched_gt_indices)
        """
        if not det_positions and not gt_positions:
            return 0, 0, 0, set()
        if not gt_positions:
            return 0, len(det_positions), 0, set()
        if not det_positions:
            return 0, 0, len(gt_positions), set()

        # Sort detections by confidence if provided
        if confidences:
            sorted_indices = np.argsort(confidences)[::-1]
            det_positions = [det_positions[i] for i in sorted_indices if i < len(det_positions)]

        gt_matched = set()
        det_matched = set()

        # Greedy matching: highest confidence first
        for det_idx, det_pos in enumerate(det_positions):
            best_dist = float('inf')
            best_gt_idx = -1

            for gt_idx, gt_pos in enumerate(gt_positions):
                if gt_idx in gt_matched:
                    continue
                dist = np.linalg.norm(det_pos[:2] - gt_pos[:2])  # 2D distance
                if dist < best_dist and dist < distance_threshold:
                    best_dist = dist
                    best_gt_idx = gt_idx

            if best_gt_idx >= 0:
                gt_matched.add(best_gt_idx)
                det_matched.add(det_idx)

        tp = len(det_matched)
        fp = len(det_positions) - tp
        fn = len(gt_positions) - len(gt_matched)

        return tp, fp, fn, gt_matched

    def _compute_ap(self, iou_threshold: float) -> float:
        """
        Compute Average Precision at given IoU threshold.
        Uses 11-point interpolation (VOC style).
        """
        data = self._accumulated.get(iou_threshold)
        if not data or data['num_gt'] == 0:
            return 0.0

        # Sort by confidence
        tp_list = sorted(data['tp'], key=lambda x: x[0], reverse=True)

        if not tp_list:
            return 0.0

        # Compute precision-recall curve
        tp_cumsum = 0
        fp_cumsum = 0
        precisions = []
        recalls = []

        for conf, is_tp in tp_list:
            if is_tp:
                tp_cumsum += 1
            else:
                fp_cumsum += 1

            precision = tp_cumsum / (tp_cumsum + fp_cumsum)
            recall = tp_cumsum / data['num_gt']
            precisions.append(precision)
            recalls.append(recall)

        # 11-point interpolation
        ap = 0.0
        for t in np.arange(0, 1.1, 0.1):
            precisions_at_recall = [p for p, r in zip(precisions, recalls) if r >= t]
            if precisions_at_recall:
                ap += max(precisions_at_recall)
        ap /= 11.0

        return ap

    def get_metrics(self) -> Dict:
        """
        Get aggregated detection metrics.

        Returns
        -------
        dict
            Dictionary with all detection metrics
        """
        if not self.frame_history:
            return {
                'total_frames': 0,
                'total_tp': 0,
                'total_fp': 0,
                'total_fn': 0,
                'precision': 0.0,
                'recall': 0.0,
                'f1': 0.0,
                'ap_30': 0.0,
                'ap_50': 0.0,
                'ap_70': 0.0,
                'map': 0.0,
            }

        frames = list(self.frame_history)
        total_tp = sum(f.true_positives for f in frames)
        total_fp = sum(f.false_positives for f in frames)
        total_fn = sum(f.false_negatives for f in frames)

        precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) > 0 else 0.0
        recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) > 0 else 0.0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

        ap_30 = self._compute_ap(0.3)
        ap_50 = self._compute_ap(0.5)
        ap_70 = self._compute_ap(0.7)
        map_score = (ap_30 + ap_50 + ap_70) / 3.0

        return {
            'total_frames': len(frames),
            'total_tp': total_tp,
            'total_fp': total_fp,
            'total_fn': total_fn,
            'precision': precision,
            'recall': recall,
            'f1': f1,
            'ap_30': ap_30,
            'ap_50': ap_50,
            'ap_70': ap_70,
            'map': map_score,
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

            # 1. Precision/Recall over time
            precisions = [f.precision for f in frames]
            recalls = [f.recall for f in frames]
            axes[0, 0].plot(frame_ids, precisions, 'g-', label='Precision', alpha=0.7)
            axes[0, 0].plot(frame_ids, recalls, 'b-', label='Recall', alpha=0.7)
            axes[0, 0].axhline(y=metrics['precision'], color='darkgreen', linestyle='--', alpha=0.5)
            axes[0, 0].axhline(y=metrics['recall'], color='darkblue', linestyle='--', alpha=0.5)
            axes[0, 0].set_xlabel('Frame')
            axes[0, 0].set_ylabel('Score')
            axes[0, 0].set_title(f"P={metrics['precision']:.3f}, R={metrics['recall']:.3f}, F1={metrics['f1']:.3f}")
            axes[0, 0].set_ylim([0, 1.1])
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

            # 2. TP/FP/FN over time
            tps = [f.true_positives for f in frames]
            fps = [f.false_positives for f in frames]
            fns = [f.false_negatives for f in frames]
            axes[0, 1].stackplot(frame_ids, tps, fps, fns,
                                  labels=['TP', 'FP', 'FN'],
                                  colors=['green', 'red', 'orange'], alpha=0.7)
            axes[0, 1].set_xlabel('Frame')
            axes[0, 1].set_ylabel('Count')
            axes[0, 1].set_title(f"TP={metrics['total_tp']}, FP={metrics['total_fp']}, FN={metrics['total_fn']}")
            axes[0, 1].legend(loc='upper left')
            axes[0, 1].grid(True, alpha=0.3)

            # 3. AP at different thresholds (bar chart)
            thresholds = ['AP@0.3', 'AP@0.5', 'AP@0.7', 'mAP']
            ap_values = [metrics['ap_30'], metrics['ap_50'], metrics['ap_70'], metrics['map']]
            colors = ['lightgreen', 'green', 'darkgreen', 'blue']
            bars = axes[1, 0].bar(thresholds, ap_values, color=colors, alpha=0.8)
            axes[1, 0].set_ylabel('Average Precision')
            axes[1, 0].set_title('Detection AP at Different IoU Thresholds')
            axes[1, 0].set_ylim([0, 1.0])
            for bar, val in zip(bars, ap_values):
                axes[1, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02,
                               f'{val:.3f}', ha='center', va='bottom', fontsize=10)
            axes[1, 0].grid(True, alpha=0.3, axis='y')

            # 4. Detection count histogram
            num_dets = [f.num_detections for f in frames]
            num_gts = [f.num_ground_truth for f in frames]
            axes[1, 1].hist(num_dets, bins=20, alpha=0.7, label='Detections', color='blue')
            axes[1, 1].hist(num_gts, bins=20, alpha=0.7, label='Ground Truth', color='green')
            axes[1, 1].set_xlabel('Count per Frame')
            axes[1, 1].set_ylabel('Frequency')
            axes[1, 1].set_title('Detection vs Ground Truth Distribution')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)

        fig.suptitle('Detection Metrics Evaluation')
        fig.tight_layout()

        perform_txt = f"""
Detection Metrics Summary:
  Total Frames: {metrics['total_frames']}
  True Positives: {metrics['total_tp']}
  False Positives: {metrics['total_fp']}
  False Negatives: {metrics['total_fn']}
  Precision: {metrics['precision']:.4f}
  Recall: {metrics['recall']:.4f}
  F1 Score: {metrics['f1']:.4f}
  AP@0.3: {metrics['ap_30']:.4f}
  AP@0.5: {metrics['ap_50']:.4f}
  AP@0.7: {metrics['ap_70']:.4f}
  mAP: {metrics['map']:.4f}
"""

        return fig, perform_txt, metrics

    def reset(self):
        """Reset all accumulated metrics."""
        self.frame_history.clear()
        self.total_frames = 0
        for iou in self._accumulated:
            self._accumulated[iou] = {'tp': [], 'fp': [], 'num_gt': 0}
