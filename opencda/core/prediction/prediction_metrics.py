# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Prediction Metrics

Tracks trajectory prediction quality metrics: ADE at multiple horizons, FDE,
miss rate, minADE/minFDE for multi-modal predictions.
"""
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from collections import deque


@dataclass
class PredictionFrame:
    """Metrics for a single prediction evaluation frame"""
    frame_id: int
    num_predictions: int = 0
    num_ground_truth: int = 0
    ade_1s: float = 0.0  # Average Displacement Error at 1 second
    ade_2s: float = 0.0  # Average Displacement Error at 2 seconds
    ade_3s: float = 0.0  # Average Displacement Error at 3 seconds
    fde: float = 0.0     # Final Displacement Error
    miss_rate: float = 0.0  # Fraction of predictions > miss_threshold from GT
    min_ade: float = 0.0  # Best ADE across modes (for multi-modal)
    min_fde: float = 0.0  # Best FDE across modes (for multi-modal)
    num_modes: int = 1    # Number of prediction modes


class PredictionMetrics:
    """
    Accumulates trajectory prediction metrics over time.

    Tracks:
    - ADE (Average Displacement Error) at 1s, 2s, 3s horizons
    - FDE (Final Displacement Error)
    - Miss Rate: fraction of predictions > threshold from ground truth
    - minADE/minFDE: best performance across multiple prediction modes

    Parameters
    ----------
    history_size : int
        Number of frames to keep in history
    dt : float
        Simulation timestep in seconds (default 0.05 = 20Hz)
    horizons_s : List[float]
        Prediction horizons to evaluate in seconds
    miss_threshold : float
        Distance threshold for miss rate calculation (meters)
    """

    def __init__(
        self,
        history_size: int = 1000,
        dt: float = 0.05,
        horizons_s: Optional[List[float]] = None,
        miss_threshold: float = 4.0
    ):
        self.history_size = history_size
        self.dt = dt
        self.horizons_s = horizons_s or [1.0, 2.0, 3.0]
        self.miss_threshold = miss_threshold
        self.frame_history: deque = deque(maxlen=history_size)

        # Accumulated metrics
        self._ade_by_horizon: Dict[float, List[float]] = {h: [] for h in self.horizons_s}
        self._fde_list: List[float] = []
        self._min_ade_list: List[float] = []
        self._min_fde_list: List[float] = []
        self._miss_count = 0
        self._total_predictions = 0

        self.total_frames = 0

    def update(
        self,
        predictions: Dict[int, List[np.ndarray]],
        ground_truth: Dict[int, List[np.ndarray]],
        frame_id: int = 0
    ) -> PredictionFrame:
        """
        Update metrics with new predictions.

        Parameters
        ----------
        predictions : Dict[int, List[np.ndarray]]
            Dictionary mapping track_id -> list of predicted positions.
            Each position is [x, y, z] or [x, y].
            For multi-modal: Dict[int, List[List[np.ndarray]]] where inner list is modes.
        ground_truth : Dict[int, List[np.ndarray]]
            Dictionary mapping track_id -> list of actual future positions.
        frame_id : int
            Frame identifier

        Returns
        -------
        PredictionFrame
            Metrics for this frame
        """
        self.total_frames += 1

        ade_1s_list = []
        ade_2s_list = []
        ade_3s_list = []
        fde_list = []
        min_ade_list = []
        min_fde_list = []
        miss_count = 0
        total_preds = 0
        num_modes = 1

        for track_id, pred_traj in predictions.items():
            if track_id not in ground_truth:
                continue

            gt_traj = ground_truth[track_id]
            if not pred_traj or not gt_traj:
                continue

            # Handle multi-modal predictions
            if isinstance(pred_traj[0], list) or (isinstance(pred_traj[0], np.ndarray) and pred_traj[0].ndim > 1):
                # Multi-modal: pred_traj is list of trajectories
                modes = pred_traj
                num_modes = max(num_modes, len(modes))

                mode_ades = []
                mode_fdes = []

                for mode_traj in modes:
                    ade, fde = self._compute_trajectory_errors(mode_traj, gt_traj)
                    if ade is not None:
                        mode_ades.append(ade)
                        mode_fdes.append(fde)

                if mode_ades:
                    min_ade = min(mode_ades)
                    min_fde = min(mode_fdes)
                    min_ade_list.append(min_ade)
                    min_fde_list.append(min_fde)

                    # Use best mode for horizon-specific ADE
                    best_mode_idx = mode_ades.index(min_ade)
                    best_traj = modes[best_mode_idx]
                    horizon_errors = self._compute_horizon_errors(best_traj, gt_traj)

                    if 1.0 in horizon_errors:
                        ade_1s_list.append(horizon_errors[1.0])
                    if 2.0 in horizon_errors:
                        ade_2s_list.append(horizon_errors[2.0])
                    if 3.0 in horizon_errors:
                        ade_3s_list.append(horizon_errors[3.0])

                    fde_list.append(min_fde)
                    total_preds += 1

                    if min_fde > self.miss_threshold:
                        miss_count += 1

            else:
                # Single mode prediction
                ade, fde = self._compute_trajectory_errors(pred_traj, gt_traj)

                if ade is not None:
                    min_ade_list.append(ade)
                    min_fde_list.append(fde)
                    fde_list.append(fde)
                    total_preds += 1

                    horizon_errors = self._compute_horizon_errors(pred_traj, gt_traj)

                    if 1.0 in horizon_errors:
                        ade_1s_list.append(horizon_errors[1.0])
                    if 2.0 in horizon_errors:
                        ade_2s_list.append(horizon_errors[2.0])
                    if 3.0 in horizon_errors:
                        ade_3s_list.append(horizon_errors[3.0])

                    if fde > self.miss_threshold:
                        miss_count += 1

        # Aggregate frame metrics
        ade_1s = np.mean(ade_1s_list) if ade_1s_list else 0.0
        ade_2s = np.mean(ade_2s_list) if ade_2s_list else 0.0
        ade_3s = np.mean(ade_3s_list) if ade_3s_list else 0.0
        fde = np.mean(fde_list) if fde_list else 0.0
        min_ade = np.mean(min_ade_list) if min_ade_list else 0.0
        min_fde = np.mean(min_fde_list) if min_fde_list else 0.0
        miss_rate = miss_count / total_preds if total_preds > 0 else 0.0

        # Update accumulated lists
        self._ade_by_horizon[1.0].extend(ade_1s_list)
        self._ade_by_horizon[2.0].extend(ade_2s_list)
        self._ade_by_horizon[3.0].extend(ade_3s_list)
        self._fde_list.extend(fde_list)
        self._min_ade_list.extend(min_ade_list)
        self._min_fde_list.extend(min_fde_list)
        self._miss_count += miss_count
        self._total_predictions += total_preds

        frame = PredictionFrame(
            frame_id=frame_id,
            num_predictions=total_preds,
            num_ground_truth=len(ground_truth),
            ade_1s=ade_1s,
            ade_2s=ade_2s,
            ade_3s=ade_3s,
            fde=fde,
            miss_rate=miss_rate,
            min_ade=min_ade,
            min_fde=min_fde,
            num_modes=num_modes,
        )

        self.frame_history.append(frame)
        return frame

    def _compute_trajectory_errors(
        self,
        pred_traj: List[np.ndarray],
        gt_traj: List[np.ndarray]
    ) -> Tuple[Optional[float], Optional[float]]:
        """
        Compute ADE and FDE for a single trajectory.

        Returns
        -------
        tuple
            (ADE, FDE) or (None, None) if trajectories are empty
        """
        if not pred_traj or not gt_traj:
            return None, None

        min_len = min(len(pred_traj), len(gt_traj))
        if min_len == 0:
            return None, None

        errors = []
        for i in range(min_len):
            pred_pos = np.array(pred_traj[i])[:2]  # x, y only
            gt_pos = np.array(gt_traj[i])[:2]
            error = np.linalg.norm(pred_pos - gt_pos)
            errors.append(error)

        ade = np.mean(errors)
        fde = errors[-1]

        return ade, fde

    def _compute_horizon_errors(
        self,
        pred_traj: List[np.ndarray],
        gt_traj: List[np.ndarray]
    ) -> Dict[float, float]:
        """
        Compute errors at specific time horizons.

        Returns
        -------
        dict
            Mapping from horizon (seconds) to error (meters)
        """
        horizon_errors = {}

        if not pred_traj or not gt_traj:
            return horizon_errors

        min_len = min(len(pred_traj), len(gt_traj))

        for horizon in self.horizons_s:
            idx = int(horizon / self.dt)
            if idx < min_len:
                pred_pos = np.array(pred_traj[idx])[:2]
                gt_pos = np.array(gt_traj[idx])[:2]
                error = np.linalg.norm(pred_pos - gt_pos)
                horizon_errors[horizon] = error

        return horizon_errors

    def get_metrics(self) -> Dict:
        """
        Get aggregated prediction metrics.

        Returns
        -------
        dict
            Dictionary with all prediction metrics
        """
        if not self.frame_history:
            return {
                'total_frames': 0,
                'total_predictions': 0,
                'ade_1s': 0.0,
                'ade_2s': 0.0,
                'ade_3s': 0.0,
                'fde': 0.0,
                'miss_rate': 0.0,
                'min_ade': 0.0,
                'min_fde': 0.0,
            }

        ade_1s = np.mean(self._ade_by_horizon[1.0]) if self._ade_by_horizon[1.0] else 0.0
        ade_2s = np.mean(self._ade_by_horizon[2.0]) if self._ade_by_horizon[2.0] else 0.0
        ade_3s = np.mean(self._ade_by_horizon[3.0]) if self._ade_by_horizon[3.0] else 0.0
        fde = np.mean(self._fde_list) if self._fde_list else 0.0
        min_ade = np.mean(self._min_ade_list) if self._min_ade_list else 0.0
        min_fde = np.mean(self._min_fde_list) if self._min_fde_list else 0.0
        miss_rate = self._miss_count / self._total_predictions if self._total_predictions > 0 else 0.0

        return {
            'total_frames': len(self.frame_history),
            'total_predictions': self._total_predictions,
            'ade_1s': ade_1s,
            'ade_2s': ade_2s,
            'ade_3s': ade_3s,
            'fde': fde,
            'miss_rate': miss_rate,
            'min_ade': min_ade,
            'min_fde': min_fde,
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

            # 1. ADE at different horizons over time
            ade_1s = [f.ade_1s for f in frames]
            ade_2s = [f.ade_2s for f in frames]
            ade_3s = [f.ade_3s for f in frames]

            axes[0, 0].plot(frame_ids, ade_1s, 'b-', label='ADE @1s', alpha=0.7)
            axes[0, 0].plot(frame_ids, ade_2s, 'orange', label='ADE @2s', alpha=0.7)
            axes[0, 0].plot(frame_ids, ade_3s, 'r-', label='ADE @3s', alpha=0.7)
            axes[0, 0].set_xlabel('Frame')
            axes[0, 0].set_ylabel('Error (meters)')
            axes[0, 0].set_title('Average Displacement Error by Horizon')
            axes[0, 0].legend()
            axes[0, 0].grid(True, alpha=0.3)

            # 2. FDE over time
            fdes = [f.fde for f in frames]
            axes[0, 1].plot(frame_ids, fdes, 'g-', alpha=0.7)
            axes[0, 1].axhline(y=metrics['fde'], color='darkgreen', linestyle='--',
                              label=f"Avg FDE: {metrics['fde']:.2f}m")
            axes[0, 1].axhline(y=self.miss_threshold, color='red', linestyle=':',
                              label=f"Miss Threshold: {self.miss_threshold}m")
            axes[0, 1].set_xlabel('Frame')
            axes[0, 1].set_ylabel('FDE (meters)')
            axes[0, 1].set_title('Final Displacement Error')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)

            # 3. Summary bar chart
            metric_names = ['ADE@1s', 'ADE@2s', 'ADE@3s', 'FDE', 'minADE', 'minFDE']
            metric_values = [
                metrics['ade_1s'], metrics['ade_2s'], metrics['ade_3s'],
                metrics['fde'], metrics['min_ade'], metrics['min_fde']
            ]
            colors = ['lightblue', 'blue', 'darkblue', 'green', 'orange', 'red']
            bars = axes[1, 0].bar(metric_names, metric_values, color=colors, alpha=0.8)
            axes[1, 0].set_ylabel('Error (meters)')
            axes[1, 0].set_title('Prediction Error Summary')
            axes[1, 0].tick_params(axis='x', rotation=45)
            for bar, val in zip(bars, metric_values):
                axes[1, 0].text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.05,
                               f'{val:.2f}', ha='center', va='bottom', fontsize=9)
            axes[1, 0].grid(True, alpha=0.3, axis='y')

            # 4. Miss rate over time
            miss_rates = [f.miss_rate for f in frames]
            axes[1, 1].plot(frame_ids, miss_rates, 'r-', alpha=0.7)
            axes[1, 1].axhline(y=metrics['miss_rate'], color='darkred', linestyle='--',
                              label=f"Avg: {metrics['miss_rate']:.1%}")
            axes[1, 1].set_xlabel('Frame')
            axes[1, 1].set_ylabel('Miss Rate')
            axes[1, 1].set_title(f"Miss Rate (>{self.miss_threshold}m threshold)")
            axes[1, 1].set_ylim([0, 1.0])
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)

        fig.suptitle('Prediction Metrics Evaluation')
        fig.tight_layout()

        perform_txt = f"""
Prediction Metrics Summary:
  Total Frames: {metrics['total_frames']}
  Total Predictions: {metrics['total_predictions']}
  ADE @1s: {metrics['ade_1s']:.4f} meters
  ADE @2s: {metrics['ade_2s']:.4f} meters
  ADE @3s: {metrics['ade_3s']:.4f} meters
  FDE: {metrics['fde']:.4f} meters
  Miss Rate: {metrics['miss_rate']:.2%}
  minADE: {metrics['min_ade']:.4f} meters
  minFDE: {metrics['min_fde']:.4f} meters
"""

        return fig, perform_txt, metrics

    def reset(self):
        """Reset all accumulated metrics."""
        self.frame_history.clear()
        self._ade_by_horizon = {h: [] for h in self.horizons_s}
        self._fde_list.clear()
        self._min_ade_list.clear()
        self._min_fde_list.clear()
        self._miss_count = 0
        self._total_predictions = 0
        self.total_frames = 0
