# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Edge Resource Profiler

Measures resource utilization for edge servers running ML perception pipelines.
Designed for capacity planning: can one edge handle one intersection, multiple
intersections, or do we need multiple edges per intersection?

Metrics tracked:
- GPU memory (allocated, cached, peak)
- GPU utilization %
- CPU utilization %
- System memory
- Per-component timing breakdown
- Throughput (frames/sec, vehicles/sec)
"""
import time
import threading
import json
import os
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Deque
from datetime import datetime

import numpy as np
import torch
import psutil

# Optional: pynvml for GPU utilization (nvidia-smi Python bindings)
try:
    import pynvml
    PYNVML_AVAILABLE = True
except ImportError:
    PYNVML_AVAILABLE = False


@dataclass
class FrameMetrics:
    """Metrics for a single frame/tick"""
    tick: int
    timestamp: float

    # Timing (ms)
    total_ms: float = 0.0
    feature_collection_ms: float = 0.0
    fusion_ms: float = 0.0
    detection_ms: float = 0.0
    tracking_ms: float = 0.0
    prediction_ms: float = 0.0
    distribution_ms: float = 0.0

    # Counts
    num_agents: int = 0  # vehicles + RSUs sending data
    num_detections: int = 0
    num_tracks: int = 0
    num_predictions: int = 0

    # GPU Memory (MB)
    gpu_memory_allocated_mb: float = 0.0
    gpu_memory_cached_mb: float = 0.0
    gpu_memory_peak_mb: float = 0.0

    # GPU Utilization (%)
    gpu_utilization_pct: float = 0.0
    gpu_memory_utilization_pct: float = 0.0

    # CPU/System
    cpu_percent: float = 0.0
    system_memory_percent: float = 0.0
    process_memory_mb: float = 0.0

    # Detection metrics (separate from tracking)
    detection_true_positives: int = 0
    detection_false_positives: int = 0
    detection_false_negatives: int = 0
    detection_precision: float = 0.0
    detection_recall: float = 0.0
    detection_ap_30: float = 0.0  # AP at IoU=0.3 (V2Xverse standard)
    detection_ap_50: float = 0.0  # AP at IoU=0.5
    detection_ap_70: float = 0.0  # AP at IoU=0.7 (V2Xverse standard)
    detection_ap_75: float = 0.0  # AP at IoU=0.75
    detection_map: float = 0.0    # mAP averaged over IoU thresholds [0.5:0.95:0.05]

    # Tracking metrics
    track_id_switches: int = 0
    track_fragmentations: int = 0
    track_mostly_tracked: int = 0
    track_mostly_lost: int = 0
    track_mota: float = 0.0  # Multi-Object Tracking Accuracy
    track_motp: float = 0.0  # Multi-Object Tracking Precision (avg localization error)
    track_idf1: float = 0.0  # ID F1 Score (identity preservation)

    # Prediction metrics (at various horizons)
    prediction_error_1s_m: float = 0.0  # ADE at 1 second
    prediction_error_2s_m: float = 0.0  # ADE at 2 seconds
    prediction_error_3s_m: float = 0.0  # ADE at 3 seconds
    prediction_fde_m: float = 0.0  # Final Displacement Error
    prediction_miss_rate: float = 0.0  # % predictions > threshold from GT

    # Multi-modal prediction metrics (best of K trajectories)
    prediction_min_ade_m: float = 0.0  # Minimum ADE across all predicted modes
    prediction_min_fde_m: float = 0.0  # Minimum FDE across all predicted modes
    prediction_num_modes: int = 1       # Number of prediction modes generated

    # Closed-loop safety metrics
    collision_pedestrian: int = 0   # Collisions with pedestrians
    collision_vehicle: int = 0      # Collisions with vehicles
    collision_static: int = 0       # Collisions with static objects/layout
    red_light_infraction: int = 0   # Red light violations
    stop_sign_infraction: int = 0   # Stop sign violations
    outside_route_lanes_pct: float = 0.0  # Percentage of time outside route lanes
    route_completion_pct: float = 0.0     # Percentage of route completed (0-100)
    vehicle_blocked: bool = False         # Whether vehicle got blocked/stuck

    # Driving score (composite metric)
    score_route: float = 0.0        # Route completion score (0-100)
    score_penalty: float = 1.0      # Penalty multiplier (starts at 1.0, decreases with infractions)
    score_composed: float = 0.0     # score_route * score_penalty

    # Ego-Uniqueness metrics
    ego_uniqueness_violations: int = 0   # identities with >1 active track this tick
    duplicate_track_count: int = 0       # total extra tracks (sum of |tracks|-1 per dup identity)
    ego_ghost_tracks: int = 0            # duplicates involving managed/ego vehicles
    geom_dup_clusters: int = 0           # geometric duplicate clusters (always defined)
    geom_dup_tracks: int = 0             # tracks participating in geometric duplicate clusters

    # Age-of-Information at publish boundary (ticks)
    aoi_ticks: int = 0  # current_tick - latest_source_tick

    # Compute-contention metrics
    contention_vehicles_affected: int = 0   # vehicles that got stale predictions
    contention_budget_exceeded_ms: float = 0.0  # simulated compute overshoot

    # Additional timing detail
    timing_detail: Dict[str, float] = field(default_factory=dict)


@dataclass
class IntersectionMetrics:
    """Aggregate metrics for an intersection over time"""
    intersection_id: str
    start_time: float = 0.0
    end_time: float = 0.0
    total_frames: int = 0

    # Timing stats (ms)
    avg_total_ms: float = 0.0
    max_total_ms: float = 0.0
    min_total_ms: float = 0.0
    p95_total_ms: float = 0.0
    p99_total_ms: float = 0.0

    # Throughput
    frames_per_second: float = 0.0
    avg_agents_per_frame: float = 0.0
    avg_detections_per_frame: float = 0.0

    # Resource usage (averages)
    avg_gpu_memory_mb: float = 0.0
    peak_gpu_memory_mb: float = 0.0
    avg_gpu_utilization_pct: float = 0.0
    avg_cpu_percent: float = 0.0
    avg_process_memory_mb: float = 0.0

    # Detection metrics (aggregate)
    total_detection_tp: int = 0
    total_detection_fp: int = 0
    total_detection_fn: int = 0
    detection_precision: float = 0.0
    detection_recall: float = 0.0
    detection_f1: float = 0.0
    detection_ap_30: float = 0.0  # AP at IoU=0.3
    detection_ap_50: float = 0.0  # AP at IoU=0.5
    detection_ap_70: float = 0.0  # AP at IoU=0.7
    detection_ap_75: float = 0.0  # AP at IoU=0.75
    detection_map: float = 0.0    # mAP averaged over IoU thresholds

    # Tracking metrics (aggregate)
    total_id_switches: int = 0
    total_fragmentations: int = 0
    avg_mota: float = 0.0
    avg_motp: float = 0.0  # Average localization precision (lower is better)
    avg_idf1: float = 0.0  # Average ID F1 score

    # Prediction metrics (aggregate)
    avg_prediction_error_1s_m: float = 0.0
    avg_prediction_error_2s_m: float = 0.0
    avg_prediction_error_3s_m: float = 0.0
    avg_prediction_fde_m: float = 0.0
    avg_miss_rate: float = 0.0

    # Multi-modal prediction metrics (aggregate)
    avg_min_ade_m: float = 0.0
    avg_min_fde_m: float = 0.0
    avg_num_modes: float = 1.0

    # Ego-Uniqueness metrics (aggregate)
    total_ego_uniqueness_violations: int = 0
    ticks_with_violations: int = 0
    violation_tick_fraction: float = 0.0
    total_duplicate_tracks: int = 0
    total_ego_ghost_tracks: int = 0
    # Geometric duplicates (aggregate, always defined)
    total_geom_dup_clusters: int = 0
    total_geom_dup_tracks: int = 0
    ticks_with_geom_dups: int = 0
    geom_dup_tick_fraction: float = 0.0

    # AoI at publish boundary (aggregate)
    aoi_mean_ticks: float = 0.0
    aoi_p50_ticks: float = 0.0
    aoi_p95_ticks: float = 0.0
    aoi_p99_ticks: float = 0.0
    aoi_max_ticks: int = 0

    # Compute-contention metrics (aggregate)
    total_contention_vehicles_affected: int = 0
    ticks_with_contention: int = 0
    contention_tick_fraction: float = 0.0
    avg_contention_budget_exceeded_ms: float = 0.0


class EdgeProfiler:
    """
    Resource profiler for edge perception pipelines.

    Usage:
        profiler = EdgeProfiler(intersection_id="intersection_001")

        # In your processing loop:
        with profiler.profile_frame(tick) as frame:
            with frame.time("feature_collection"):
                # collect features
            with frame.time("fusion"):
                # run fusion
            frame.set_counts(num_agents=2, num_detections=5)

        # At end:
        summary = profiler.get_summary()
        profiler.save_report("edge_metrics.json")
    """

    def __init__(
        self,
        intersection_id: str,
        history_size: int = 1000,
        sample_gpu_utilization: bool = True,
        gpu_device: int = 0
    ):
        self.intersection_id = intersection_id
        self.history_size = history_size
        self.sample_gpu_utilization = sample_gpu_utilization
        self.gpu_device = gpu_device

        self.frame_history: Deque[FrameMetrics] = deque(maxlen=history_size)
        self.start_time = time.time()
        self.process = psutil.Process(os.getpid())

        # Initialize pynvml if available
        self._nvml_initialized = False
        if PYNVML_AVAILABLE and sample_gpu_utilization:
            try:
                pynvml.nvmlInit()
                self._nvml_handle = pynvml.nvmlDeviceGetHandleByIndex(gpu_device)
                self._nvml_initialized = True
            except Exception as e:
                print(f"[EdgeProfiler] Failed to initialize NVML: {e}")

        # Current frame being profiled
        self._current_frame: Optional[FrameMetrics] = None
        self._frame_start_time: float = 0.0
        self._timer_stack: List[tuple] = []

    def profile_frame(self, tick: int) -> 'FrameProfileContext':
        """Start profiling a frame. Use as context manager."""
        return FrameProfileContext(self, tick)

    def _start_frame(self, tick: int):
        """Internal: start frame profiling"""
        # Reset GPU peak memory tracking
        if torch.cuda.is_available():
            torch.cuda.reset_peak_memory_stats(self.gpu_device)

        self._frame_start_time = time.perf_counter()
        self._current_frame = FrameMetrics(
            tick=tick,
            timestamp=time.time()
        )
        self._timer_stack = []

    def _end_frame(self):
        """Internal: end frame profiling and collect metrics"""
        if self._current_frame is None:
            return

        # Total time
        self._current_frame.total_ms = (time.perf_counter() - self._frame_start_time) * 1000.0

        # GPU memory
        if torch.cuda.is_available():
            self._current_frame.gpu_memory_allocated_mb = (
                torch.cuda.memory_allocated(self.gpu_device) / 1024 / 1024
            )
            self._current_frame.gpu_memory_cached_mb = (
                torch.cuda.memory_reserved(self.gpu_device) / 1024 / 1024
            )
            self._current_frame.gpu_memory_peak_mb = (
                torch.cuda.max_memory_allocated(self.gpu_device) / 1024 / 1024
            )

        # GPU utilization via NVML
        if self._nvml_initialized:
            try:
                util = pynvml.nvmlDeviceGetUtilizationRates(self._nvml_handle)
                self._current_frame.gpu_utilization_pct = util.gpu
                self._current_frame.gpu_memory_utilization_pct = util.memory
            except Exception:
                pass

        # CPU/Memory
        self._current_frame.cpu_percent = self.process.cpu_percent()
        self._current_frame.system_memory_percent = psutil.virtual_memory().percent
        self._current_frame.process_memory_mb = self.process.memory_info().rss / 1024 / 1024

        # Store frame
        self.frame_history.append(self._current_frame)
        self._current_frame = None

    def time_component(self, name: str) -> 'ComponentTimerContext':
        """Time a specific component. Use as context manager."""
        return ComponentTimerContext(self, name)

    def _start_component(self, name: str):
        """Internal: start component timing"""
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        self._timer_stack.append((name, time.perf_counter()))

    def _end_component(self, name: str):
        """Internal: end component timing"""
        if torch.cuda.is_available():
            torch.cuda.synchronize()

        if not self._timer_stack or self._timer_stack[-1][0] != name:
            return

        _, start_time = self._timer_stack.pop()
        elapsed_ms = (time.perf_counter() - start_time) * 1000.0

        if self._current_frame is not None:
            # Map to known fields or add to detail
            field_map = {
                'feature_collection': 'feature_collection_ms',
                'fusion': 'fusion_ms',
                'detection': 'detection_ms',
                'tracking': 'tracking_ms',
                'prediction': 'prediction_ms',
                'distribution': 'distribution_ms',
            }

            if name in field_map:
                setattr(self._current_frame, field_map[name], elapsed_ms)
            else:
                self._current_frame.timing_detail[name] = elapsed_ms

    def set_counts(
        self,
        num_agents: Optional[int] = None,
        num_detections: Optional[int] = None,
        num_tracks: Optional[int] = None,
        num_predictions: Optional[int] = None
    ):
        """Set count metrics for current frame"""
        if self._current_frame is None:
            return

        if num_agents is not None:
            self._current_frame.num_agents = num_agents
        if num_detections is not None:
            self._current_frame.num_detections = num_detections
        if num_tracks is not None:
            self._current_frame.num_tracks = num_tracks
        if num_predictions is not None:
            self._current_frame.num_predictions = num_predictions

    def set_detection_metrics(
        self,
        true_positives: int = 0,
        false_positives: int = 0,
        false_negatives: int = 0,
        ap_50: Optional[float] = None,
        ap_75: Optional[float] = None,
        map_score: Optional[float] = None
    ):
        """
        Set detection metrics for current frame.

        Parameters
        ----------
        true_positives : int
            Number of correct detections matched to ground truth
        false_positives : int
            Number of detections not matched to any ground truth
        false_negatives : int
            Number of ground truth objects not detected
        ap_50 : float, optional
            Average Precision at IoU=0.5
        ap_75 : float, optional
            Average Precision at IoU=0.75
        map_score : float, optional
            Mean Average Precision over IoU thresholds [0.5:0.95:0.05]
        """
        if self._current_frame is None:
            return

        self._current_frame.detection_true_positives = true_positives
        self._current_frame.detection_false_positives = false_positives
        self._current_frame.detection_false_negatives = false_negatives

        # Calculate precision and recall
        total_pred = true_positives + false_positives
        total_gt = true_positives + false_negatives

        precision = true_positives / total_pred if total_pred > 0 else 0.0
        recall = true_positives / total_gt if total_gt > 0 else 0.0

        self._current_frame.detection_precision = precision
        self._current_frame.detection_recall = recall

        # Set AP metrics if provided, otherwise estimate from P/R
        if ap_50 is not None:
            self._current_frame.detection_ap_50 = ap_50
        else:
            # Simple approximation: AP ≈ precision * recall (very rough)
            self._current_frame.detection_ap_50 = precision * recall

        if ap_75 is not None:
            self._current_frame.detection_ap_75 = ap_75
        else:
            # AP at higher IoU is typically lower
            self._current_frame.detection_ap_75 = precision * recall * 0.7

        if map_score is not None:
            self._current_frame.detection_map = map_score
        else:
            # Estimate mAP as average of AP@50 and AP@75
            self._current_frame.detection_map = (
                self._current_frame.detection_ap_50 + self._current_frame.detection_ap_75
            ) / 2

    def set_tracking_metrics(
        self,
        id_switches: int = 0,
        fragmentations: int = 0,
        mostly_tracked: int = 0,
        mostly_lost: int = 0,
        mota: Optional[float] = None,
        motp: Optional[float] = None,
        idf1: Optional[float] = None
    ):
        """
        Set tracking metrics for current frame.

        Parameters
        ----------
        id_switches : int
            Number of times a track ID changed for same object
        fragmentations : int
            Number of times a track was interrupted
        mostly_tracked : int
            Number of objects tracked for >= 80% of their lifetime
        mostly_lost : int
            Number of objects tracked for <= 20% of their lifetime
        mota : float, optional
            Multi-Object Tracking Accuracy (if pre-computed)
        motp : float, optional
            Multi-Object Tracking Precision - average localization error (meters)
        idf1 : float, optional
            ID F1 Score - measures identity preservation (0-1)
        """
        if self._current_frame is None:
            return

        self._current_frame.track_id_switches = id_switches
        self._current_frame.track_fragmentations = fragmentations
        self._current_frame.track_mostly_tracked = mostly_tracked
        self._current_frame.track_mostly_lost = mostly_lost

        if mota is not None:
            self._current_frame.track_mota = mota
        else:
            # Simple MOTA calculation: 1 - (FN + FP + IDS) / GT
            tp = self._current_frame.detection_true_positives
            fp = self._current_frame.detection_false_positives
            fn = self._current_frame.detection_false_negatives
            gt = tp + fn
            if gt > 0:
                self._current_frame.track_mota = 1.0 - (fn + fp + id_switches) / gt
            else:
                self._current_frame.track_mota = 0.0

        # MOTP: Multi-Object Tracking Precision (lower is better)
        if motp is not None:
            self._current_frame.track_motp = motp

        # IDF1: ID F1 Score - computed from IDTP, IDFP, IDFN
        if idf1 is not None:
            self._current_frame.track_idf1 = idf1
        else:
            # Estimate IDF1 from available metrics
            # IDF1 = 2 * IDTP / (2 * IDTP + IDFP + IDFN)
            # Approximate: IDTP ≈ TP, IDFP ≈ FP + ID_switches, IDFN ≈ FN + ID_switches
            tp = self._current_frame.detection_true_positives
            fp = self._current_frame.detection_false_positives
            fn = self._current_frame.detection_false_negatives
            idtp = tp
            idfp = fp + id_switches
            idfn = fn + id_switches
            denom = 2 * idtp + idfp + idfn
            self._current_frame.track_idf1 = 2 * idtp / denom if denom > 0 else 0.0

    def set_prediction_metrics(
        self,
        error_1s_m: float = 0.0,
        error_2s_m: float = 0.0,
        error_3s_m: float = 0.0,
        fde_m: float = 0.0,
        miss_rate: float = 0.0,
        min_ade_m: Optional[float] = None,
        min_fde_m: Optional[float] = None,
        num_modes: int = 1
    ):
        """
        Set prediction quality metrics for current frame.

        Parameters
        ----------
        error_1s_m : float
            Average Displacement Error at 1 second horizon (meters)
        error_2s_m : float
            Average Displacement Error at 2 second horizon (meters)
        error_3s_m : float
            Average Displacement Error at 3 second horizon (meters)
        fde_m : float
            Final Displacement Error at end of prediction horizon (meters)
        miss_rate : float
            Percentage of predictions > 2m from ground truth (0-1)
        min_ade_m : float, optional
            Minimum ADE across all prediction modes (for multi-modal predictors)
        min_fde_m : float, optional
            Minimum FDE across all prediction modes (for multi-modal predictors)
        num_modes : int
            Number of prediction modes/hypotheses generated per target
        """
        if self._current_frame is None:
            return

        self._current_frame.prediction_error_1s_m = error_1s_m
        self._current_frame.prediction_error_2s_m = error_2s_m
        self._current_frame.prediction_error_3s_m = error_3s_m
        self._current_frame.prediction_fde_m = fde_m
        self._current_frame.prediction_miss_rate = miss_rate
        self._current_frame.prediction_num_modes = num_modes

        # For multi-modal predictions, minADE/minFDE are the best across modes
        # For single-mode predictors (like linear predictor), minADE = ADE
        if min_ade_m is not None:
            self._current_frame.prediction_min_ade_m = min_ade_m
        else:
            # Default: use average of ADE across horizons
            self._current_frame.prediction_min_ade_m = (error_1s_m + error_2s_m + error_3s_m) / 3 if any([error_1s_m, error_2s_m, error_3s_m]) else 0.0

        if min_fde_m is not None:
            self._current_frame.prediction_min_fde_m = min_fde_m
        else:
            self._current_frame.prediction_min_fde_m = fde_m

    def set_ego_uniqueness_metrics(
        self,
        violations: int = 0,
        duplicate_tracks: int = 0,
        ego_ghosts: int = 0,
        geom_dup_clusters: int = 0,
        geom_dup_tracks: int = 0,
    ):
        """Set Ego-Uniqueness metrics for current frame."""
        if self._current_frame is None:
            return
        self._current_frame.ego_uniqueness_violations = violations
        self._current_frame.duplicate_track_count = duplicate_tracks
        self._current_frame.ego_ghost_tracks = ego_ghosts
        self._current_frame.geom_dup_clusters = geom_dup_clusters
        self._current_frame.geom_dup_tracks = geom_dup_tracks

    def set_aoi_ticks(self, aoi_ticks: int):
        """Set Age-of-Information for current frame (ticks since source)."""
        if self._current_frame is None:
            return
        self._current_frame.aoi_ticks = aoi_ticks

    def set_contention_metrics(
        self,
        vehicles_affected: int = 0,
        budget_exceeded_ms: float = 0.0
    ):
        """Set compute-contention metrics for current frame.

        Parameters
        ----------
        vehicles_affected : int
            Number of vehicles that received stale (previous-tick) predictions
            because the simulated compute budget was exceeded.
        budget_exceeded_ms : float
            Amount (ms) by which the simulated per-vehicle compute sum exceeded
            the configured budget.
        """
        if self._current_frame is None:
            return
        self._current_frame.contention_vehicles_affected = vehicles_affected
        self._current_frame.contention_budget_exceeded_ms = budget_exceeded_ms

    def get_summary(self) -> IntersectionMetrics:
        """Get aggregate metrics summary"""
        if not self.frame_history:
            return IntersectionMetrics(intersection_id=self.intersection_id)

        frames = list(self.frame_history)
        total_times = [f.total_ms for f in frames]
        total_times_sorted = sorted(total_times)

        elapsed_time = time.time() - self.start_time

        # Detection aggregates
        total_tp = sum(f.detection_true_positives for f in frames)
        total_fp = sum(f.detection_false_positives for f in frames)
        total_fn = sum(f.detection_false_negatives for f in frames)
        total_pred = total_tp + total_fp
        total_gt = total_tp + total_fn

        det_precision = total_tp / total_pred if total_pred > 0 else 0.0
        det_recall = total_tp / total_gt if total_gt > 0 else 0.0
        det_f1 = (2 * det_precision * det_recall / (det_precision + det_recall)
                  if (det_precision + det_recall) > 0 else 0.0)

        # AP/mAP aggregates
        ap_50_values = [f.detection_ap_50 for f in frames if f.detection_ap_50 > 0]
        ap_75_values = [f.detection_ap_75 for f in frames if f.detection_ap_75 > 0]
        map_values = [f.detection_map for f in frames if f.detection_map > 0]
        avg_ap_50 = sum(ap_50_values) / len(ap_50_values) if ap_50_values else 0.0
        avg_ap_75 = sum(ap_75_values) / len(ap_75_values) if ap_75_values else 0.0
        avg_map = sum(map_values) / len(map_values) if map_values else 0.0

        # Tracking aggregates
        total_ids = sum(f.track_id_switches for f in frames)
        total_frags = sum(f.track_fragmentations for f in frames)
        mota_values = [f.track_mota for f in frames if f.track_mota != 0]
        avg_mota = sum(mota_values) / len(mota_values) if mota_values else 0.0
        motp_values = [f.track_motp for f in frames if f.track_motp > 0]
        avg_motp = sum(motp_values) / len(motp_values) if motp_values else 0.0
        idf1_values = [f.track_idf1 for f in frames if f.track_idf1 > 0]
        avg_idf1 = sum(idf1_values) / len(idf1_values) if idf1_values else 0.0

        # Prediction aggregates
        pred_1s = [f.prediction_error_1s_m for f in frames if f.prediction_error_1s_m > 0]
        pred_2s = [f.prediction_error_2s_m for f in frames if f.prediction_error_2s_m > 0]
        pred_3s = [f.prediction_error_3s_m for f in frames if f.prediction_error_3s_m > 0]
        pred_fde = [f.prediction_fde_m for f in frames if f.prediction_fde_m > 0]
        miss_rates = [f.prediction_miss_rate for f in frames if f.num_predictions > 0]

        # Multi-modal prediction aggregates
        min_ade_values = [f.prediction_min_ade_m for f in frames if f.prediction_min_ade_m > 0]
        min_fde_values = [f.prediction_min_fde_m for f in frames if f.prediction_min_fde_m > 0]
        num_modes_values = [f.prediction_num_modes for f in frames if f.num_predictions > 0]
        avg_min_ade = sum(min_ade_values) / len(min_ade_values) if min_ade_values else 0.0
        avg_min_fde = sum(min_fde_values) / len(min_fde_values) if min_fde_values else 0.0
        avg_num_modes = sum(num_modes_values) / len(num_modes_values) if num_modes_values else 1.0

        # AoI aggregates (only count ticks with valid source_tick)
        aoi_vals = [f.aoi_ticks for f in frames if f.aoi_ticks > 0]

        summary = IntersectionMetrics(
            intersection_id=self.intersection_id,
            start_time=self.start_time,
            end_time=time.time(),
            total_frames=len(frames),

            # Timing stats
            avg_total_ms=sum(total_times) / len(total_times),
            max_total_ms=max(total_times),
            min_total_ms=min(total_times),
            p95_total_ms=total_times_sorted[int(len(total_times_sorted) * 0.95)] if len(total_times_sorted) > 20 else max(total_times),
            p99_total_ms=total_times_sorted[int(len(total_times_sorted) * 0.99)] if len(total_times_sorted) > 100 else max(total_times),

            # Throughput
            frames_per_second=len(frames) / elapsed_time if elapsed_time > 0 else 0,
            avg_agents_per_frame=sum(f.num_agents for f in frames) / len(frames),
            avg_detections_per_frame=sum(f.num_detections for f in frames) / len(frames),

            # Resource usage
            avg_gpu_memory_mb=sum(f.gpu_memory_allocated_mb for f in frames) / len(frames),
            peak_gpu_memory_mb=max(f.gpu_memory_peak_mb for f in frames),
            avg_gpu_utilization_pct=sum(f.gpu_utilization_pct for f in frames) / len(frames),
            avg_cpu_percent=sum(f.cpu_percent for f in frames) / len(frames),
            avg_process_memory_mb=sum(f.process_memory_mb for f in frames) / len(frames),

            # Detection metrics
            total_detection_tp=total_tp,
            total_detection_fp=total_fp,
            total_detection_fn=total_fn,
            detection_precision=det_precision,
            detection_recall=det_recall,
            detection_f1=det_f1,
            detection_ap_50=avg_ap_50,
            detection_ap_75=avg_ap_75,
            detection_map=avg_map,

            # Tracking metrics
            total_id_switches=total_ids,
            total_fragmentations=total_frags,
            avg_mota=avg_mota,
            avg_motp=avg_motp,
            avg_idf1=avg_idf1,

            # Prediction metrics
            avg_prediction_error_1s_m=sum(pred_1s) / len(pred_1s) if pred_1s else 0.0,
            avg_prediction_error_2s_m=sum(pred_2s) / len(pred_2s) if pred_2s else 0.0,
            avg_prediction_error_3s_m=sum(pred_3s) / len(pred_3s) if pred_3s else 0.0,
            avg_prediction_fde_m=sum(pred_fde) / len(pred_fde) if pred_fde else 0.0,
            avg_miss_rate=sum(miss_rates) / len(miss_rates) if miss_rates else 0.0,

            # Multi-modal prediction metrics
            avg_min_ade_m=avg_min_ade,
            avg_min_fde_m=avg_min_fde,
            avg_num_modes=avg_num_modes,

            # Ego-Uniqueness metrics
            total_ego_uniqueness_violations=sum(f.ego_uniqueness_violations for f in frames),
            ticks_with_violations=sum(1 for f in frames if f.ego_uniqueness_violations > 0),
            violation_tick_fraction=(
                sum(1 for f in frames if f.ego_uniqueness_violations > 0) / len(frames)
                if frames else 0.0
            ),
            total_duplicate_tracks=sum(f.duplicate_track_count for f in frames),
            total_ego_ghost_tracks=sum(f.ego_ghost_tracks for f in frames),
            total_geom_dup_clusters=sum(f.geom_dup_clusters for f in frames),
            total_geom_dup_tracks=sum(f.geom_dup_tracks for f in frames),
            ticks_with_geom_dups=sum(1 for f in frames if f.geom_dup_clusters > 0),
            geom_dup_tick_fraction=(
                sum(1 for f in frames if f.geom_dup_clusters > 0) / len(frames)
                if frames else 0.0
            ),

            # AoI at publish boundary
            aoi_mean_ticks=float(np.mean(aoi_vals)) if aoi_vals else 0.0,
            aoi_p50_ticks=float(np.median(aoi_vals)) if aoi_vals else 0.0,
            aoi_p95_ticks=float(np.percentile(aoi_vals, 95)) if aoi_vals else 0.0,
            aoi_p99_ticks=float(np.percentile(aoi_vals, 99)) if aoi_vals else 0.0,
            aoi_max_ticks=int(max(aoi_vals)) if aoi_vals else 0,

            # Compute-contention metrics
            total_contention_vehicles_affected=sum(f.contention_vehicles_affected for f in frames),
            ticks_with_contention=sum(1 for f in frames if f.contention_vehicles_affected > 0),
            contention_tick_fraction=(
                sum(1 for f in frames if f.contention_vehicles_affected > 0) / len(frames)
                if frames else 0.0
            ),
            avg_contention_budget_exceeded_ms=(
                np.mean([f.contention_budget_exceeded_ms for f in frames
                         if f.contention_budget_exceeded_ms > 0])
                if any(f.contention_budget_exceeded_ms > 0 for f in frames)
                else 0.0
            ),
        )

        return summary

    def get_frame_history(self) -> List[Dict]:
        """Get all frame metrics as list of dicts"""
        return [asdict(f) for f in self.frame_history]

    def save_report(self, filepath: str):
        """Save profiling report to JSON file"""
        report = {
            'intersection_id': self.intersection_id,
            'generated_at': datetime.now().isoformat(),
            'summary': asdict(self.get_summary()),
            'frames': self.get_frame_history()
        }

        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)

        print(f"[EdgeProfiler] Report saved to {filepath}")

    def print_summary(self):
        """Print summary to console"""
        summary = self.get_summary()
        print(f"\n{'='*60}")
        print(f"Edge Profiler Summary: {summary.intersection_id}")
        print(f"{'='*60}")
        print(f"Total Frames: {summary.total_frames}")
        print(f"Duration: {summary.end_time - summary.start_time:.1f}s")
        print(f"Throughput: {summary.frames_per_second:.2f} fps")
        print(f"\nTiming (ms):")
        print(f"  Avg: {summary.avg_total_ms:.2f}")
        print(f"  Min: {summary.min_total_ms:.2f}")
        print(f"  Max: {summary.max_total_ms:.2f}")
        print(f"  P95: {summary.p95_total_ms:.2f}")
        print(f"  P99: {summary.p99_total_ms:.2f}")
        print(f"\nGPU Memory:")
        print(f"  Avg: {summary.avg_gpu_memory_mb:.1f} MB")
        print(f"  Peak: {summary.peak_gpu_memory_mb:.1f} MB")
        print(f"  Utilization: {summary.avg_gpu_utilization_pct:.1f}%")
        print(f"\nCPU/System:")
        print(f"  CPU: {summary.avg_cpu_percent:.1f}%")
        print(f"  Process Memory: {summary.avg_process_memory_mb:.1f} MB")
        print(f"\nWorkload:")
        print(f"  Avg Agents/Frame: {summary.avg_agents_per_frame:.1f}")
        print(f"  Avg Detections/Frame: {summary.avg_detections_per_frame:.1f}")
        print(f"{'='*60}\n")

    def cleanup(self):
        """Cleanup resources"""
        if self._nvml_initialized:
            try:
                pynvml.nvmlShutdown()
            except Exception:
                pass

    def get_evaluation_result(self):
        """
        Return evaluation results in the format expected by EvaluationManager.

        Returns:
            Tuple[matplotlib.figure.Figure, str, Dict]:
                - figure: Matplotlib figure with timing breakdown
                - perform_txt: Text summary for log file
                - metrics: Dict of metrics for global_metrics
        """
        import matplotlib.pyplot as plt
        import numpy as np

        summary = self.get_summary()
        frames = list(self.frame_history)

        # Build metrics dict
        metrics = {
            'intersection_id': summary.intersection_id,
            'total_frames': summary.total_frames,
            'duration_s': summary.end_time - summary.start_time,
            'frames_per_second': summary.frames_per_second,

            # Timing (ms)
            'timing_avg_ms': summary.avg_total_ms,
            'timing_min_ms': summary.min_total_ms,
            'timing_max_ms': summary.max_total_ms,
            'timing_p95_ms': summary.p95_total_ms,
            'timing_p99_ms': summary.p99_total_ms,

            # Component timing averages
            'feature_collection_avg_ms': np.mean([f.feature_collection_ms for f in frames]) if frames else 0,
            'fusion_avg_ms': np.mean([f.fusion_ms for f in frames]) if frames else 0,
            'detection_avg_ms': np.mean([f.detection_ms for f in frames]) if frames else 0,
            'tracking_avg_ms': np.mean([f.tracking_ms for f in frames]) if frames else 0,
            'prediction_avg_ms': np.mean([f.prediction_ms for f in frames]) if frames else 0,
            'distribution_avg_ms': np.mean([f.distribution_ms for f in frames]) if frames else 0,

            # GPU
            'gpu_memory_avg_mb': summary.avg_gpu_memory_mb,
            'gpu_memory_peak_mb': summary.peak_gpu_memory_mb,
            'gpu_utilization_avg_pct': summary.avg_gpu_utilization_pct,

            # CPU/System
            'cpu_avg_pct': summary.avg_cpu_percent,
            'process_memory_avg_mb': summary.avg_process_memory_mb,

            # Workload
            'agents_per_frame_avg': summary.avg_agents_per_frame,
            'detections_per_frame_avg': summary.avg_detections_per_frame,

            # Detection metrics
            'detection_total_tp': summary.total_detection_tp,
            'detection_total_fp': summary.total_detection_fp,
            'detection_total_fn': summary.total_detection_fn,
            'detection_precision': summary.detection_precision,
            'detection_recall': summary.detection_recall,
            'detection_f1': summary.detection_f1,
            'detection_ap_50': summary.detection_ap_50,
            'detection_ap_75': summary.detection_ap_75,
            'detection_map': summary.detection_map,

            # Tracking metrics
            'tracking_total_id_switches': summary.total_id_switches,
            'tracking_total_fragmentations': summary.total_fragmentations,
            'tracking_avg_mota': summary.avg_mota,
            'tracking_avg_motp': summary.avg_motp,
            'tracking_avg_idf1': summary.avg_idf1,

            # Prediction metrics
            'prediction_ade_1s_m': summary.avg_prediction_error_1s_m,
            'prediction_ade_2s_m': summary.avg_prediction_error_2s_m,
            'prediction_ade_3s_m': summary.avg_prediction_error_3s_m,
            'prediction_fde_m': summary.avg_prediction_fde_m,
            'prediction_miss_rate': summary.avg_miss_rate,

            # Multi-modal prediction metrics
            'prediction_min_ade_m': summary.avg_min_ade_m,
            'prediction_min_fde_m': summary.avg_min_fde_m,
            'prediction_avg_num_modes': summary.avg_num_modes,

            # Ego-Uniqueness metrics
            'ego_uniqueness_total_violations': summary.total_ego_uniqueness_violations,
            'ego_uniqueness_ticks_with_violations': summary.ticks_with_violations,
            'ego_uniqueness_violation_tick_fraction': summary.violation_tick_fraction,
            'ego_uniqueness_total_duplicate_tracks': summary.total_duplicate_tracks,
            'ego_uniqueness_total_ego_ghost_tracks': summary.total_ego_ghost_tracks,
            'geom_dup_total_clusters': summary.total_geom_dup_clusters,
            'geom_dup_total_tracks': summary.total_geom_dup_tracks,
            'geom_dup_ticks_with': summary.ticks_with_geom_dups,
            'geom_dup_tick_fraction': summary.geom_dup_tick_fraction,

            # AoI at publish boundary
            'aoi_mean_ticks': summary.aoi_mean_ticks,
            'aoi_p50_ticks': summary.aoi_p50_ticks,
            'aoi_p95_ticks': summary.aoi_p95_ticks,
            'aoi_p99_ticks': summary.aoi_p99_ticks,
            'aoi_max_ticks': summary.aoi_max_ticks,

            # Compute-contention metrics
            'contention_total_vehicles_affected': summary.total_contention_vehicles_affected,
            'contention_ticks_with_contention': summary.ticks_with_contention,
            'contention_tick_fraction': summary.contention_tick_fraction,
            'contention_avg_budget_exceeded_ms': summary.avg_contention_budget_exceeded_ms,
        }

        # Build text summary
        perform_txt = f"""
Edge Profiler: {summary.intersection_id}
  Frames: {summary.total_frames}, FPS: {summary.frames_per_second:.2f}
  Timing (ms): avg={summary.avg_total_ms:.2f}, p95={summary.p95_total_ms:.2f}, p99={summary.p99_total_ms:.2f}
  GPU Memory: avg={summary.avg_gpu_memory_mb:.1f}MB, peak={summary.peak_gpu_memory_mb:.1f}MB
  GPU Util: {summary.avg_gpu_utilization_pct:.1f}%, CPU: {summary.avg_cpu_percent:.1f}%
  Workload: {summary.avg_agents_per_frame:.1f} agents/frame, {summary.avg_detections_per_frame:.1f} dets/frame

Detection Metrics:
  Precision: {summary.detection_precision:.3f}, Recall: {summary.detection_recall:.3f}, F1: {summary.detection_f1:.3f}
  AP@50: {summary.detection_ap_50:.3f}, AP@75: {summary.detection_ap_75:.3f}, mAP: {summary.detection_map:.3f}
  TP: {summary.total_detection_tp}, FP: {summary.total_detection_fp}, FN: {summary.total_detection_fn}

Tracking Metrics:
  MOTA: {summary.avg_mota:.3f}, MOTP: {summary.avg_motp:.2f}m, IDF1: {summary.avg_idf1:.3f}
  ID Switches: {summary.total_id_switches}, Fragmentations: {summary.total_fragmentations}

Prediction Metrics:
  ADE @1s: {summary.avg_prediction_error_1s_m:.2f}m, @2s: {summary.avg_prediction_error_2s_m:.2f}m, @3s: {summary.avg_prediction_error_3s_m:.2f}m
  FDE: {summary.avg_prediction_fde_m:.2f}m, Miss Rate: {summary.avg_miss_rate:.1%}
  minADE: {summary.avg_min_ade_m:.2f}m, minFDE: {summary.avg_min_fde_m:.2f}m (avg {summary.avg_num_modes:.1f} modes)

Ego-Uniqueness:
  Violations: {summary.total_ego_uniqueness_violations}, Ticks w/ violations: {summary.ticks_with_violations} ({summary.violation_tick_fraction:.1%})
  Duplicate tracks: {summary.total_duplicate_tracks}, Ego ghosts: {summary.total_ego_ghost_tracks}
  Geom duplicates: {summary.total_geom_dup_clusters} clusters / {summary.total_geom_dup_tracks} tracks ({summary.geom_dup_tick_fraction:.1%} ticks)

Compute Contention:
  Vehicles affected: {summary.total_contention_vehicles_affected}, Ticks w/ contention: {summary.ticks_with_contention} ({summary.contention_tick_fraction:.1%})
  Avg budget exceeded: {summary.avg_contention_budget_exceeded_ms:.2f}ms
"""

        # Create figure with subplots (3x2 for more comprehensive view)
        fig, axes = plt.subplots(3, 2, figsize=(14, 14))

        # 1. Timing breakdown bar chart
        if frames:
            component_names = ['Feature\nCollection', 'Fusion', 'Detection', 'Tracking', 'Prediction', 'Distribution']
            component_avgs = [
                metrics['feature_collection_avg_ms'],
                metrics['fusion_avg_ms'],
                metrics['detection_avg_ms'],
                metrics['tracking_avg_ms'],
                metrics['prediction_avg_ms'],
                metrics['distribution_avg_ms'],
            ]
            axes[0, 0].bar(component_names, component_avgs, color='steelblue')
            axes[0, 0].set_ylabel('Time (ms)')
            axes[0, 0].set_title('Average Component Timing')
            axes[0, 0].tick_params(axis='x', rotation=45)

        # 2. Total timing over time
        if frames:
            ticks = [f.tick for f in frames]
            total_times = [f.total_ms for f in frames]
            axes[0, 1].plot(ticks, total_times, 'b-', alpha=0.7)
            axes[0, 1].axhline(y=summary.avg_total_ms, color='r', linestyle='--', label=f'Avg: {summary.avg_total_ms:.1f}ms')
            axes[0, 1].axhline(y=summary.p95_total_ms, color='orange', linestyle='--', label=f'P95: {summary.p95_total_ms:.1f}ms')
            axes[0, 1].set_xlabel('Tick')
            axes[0, 1].set_ylabel('Total Time (ms)')
            axes[0, 1].set_title('Frame Processing Time')
            axes[0, 1].legend()

        # 3. Detection metrics (precision/recall over time)
        if frames:
            precisions = [f.detection_precision for f in frames]
            recalls = [f.detection_recall for f in frames]
            axes[1, 0].plot(ticks, precisions, 'g-', label='Precision', alpha=0.7)
            axes[1, 0].plot(ticks, recalls, 'b-', label='Recall', alpha=0.7)
            axes[1, 0].axhline(y=summary.detection_precision, color='darkgreen', linestyle='--', alpha=0.5)
            axes[1, 0].axhline(y=summary.detection_recall, color='darkblue', linestyle='--', alpha=0.5)
            axes[1, 0].set_xlabel('Tick')
            axes[1, 0].set_ylabel('Score')
            axes[1, 0].set_title(f'Detection: P={summary.detection_precision:.2f}, R={summary.detection_recall:.2f}, F1={summary.detection_f1:.2f}')
            axes[1, 0].set_ylim([0, 1.1])
            axes[1, 0].legend()

        # 4. Tracking metrics (MOTA over time + ID switches)
        if frames:
            motas = [f.track_mota for f in frames]
            id_switches = [f.track_id_switches for f in frames]
            ax1 = axes[1, 1]
            ax2 = ax1.twinx()

            ax1.plot(ticks, motas, 'g-', label='MOTA', alpha=0.7)
            ax1.axhline(y=summary.avg_mota, color='darkgreen', linestyle='--', alpha=0.5)
            ax1.set_xlabel('Tick')
            ax1.set_ylabel('MOTA', color='g')
            ax1.set_ylim([-1, 1.1])

            ax2.bar(ticks, id_switches, alpha=0.3, color='red', label='ID Switches', width=1.0)
            ax2.set_ylabel('ID Switches', color='r')

            ax1.set_title(f'Tracking: MOTA={summary.avg_mota:.2f}, IDS={summary.total_id_switches}')
            ax1.legend(loc='upper left')
            ax2.legend(loc='upper right')

        # 5. Prediction error over time
        if frames:
            pred_1s = [f.prediction_error_1s_m for f in frames]
            pred_2s = [f.prediction_error_2s_m for f in frames]
            pred_3s = [f.prediction_error_3s_m for f in frames]
            axes[2, 0].plot(ticks, pred_1s, 'b-', label='ADE @1s', alpha=0.7)
            axes[2, 0].plot(ticks, pred_2s, 'orange', label='ADE @2s', alpha=0.7)
            axes[2, 0].plot(ticks, pred_3s, 'r-', label='ADE @3s', alpha=0.7)
            axes[2, 0].set_xlabel('Tick')
            axes[2, 0].set_ylabel('Error (m)')
            axes[2, 0].set_title(f'Prediction: ADE@1s={summary.avg_prediction_error_1s_m:.2f}m, FDE={summary.avg_prediction_fde_m:.2f}m')
            axes[2, 0].legend()

        # 6. GPU/CPU resource usage
        if frames:
            gpu_mem = [f.gpu_memory_allocated_mb for f in frames]
            cpu_pct = [f.cpu_percent for f in frames]
            ax1 = axes[2, 1]
            ax2 = ax1.twinx()

            ax1.plot(ticks, gpu_mem, 'g-', label='GPU Memory (MB)', alpha=0.7)
            ax1.axhline(y=summary.avg_gpu_memory_mb, color='darkgreen', linestyle='--', alpha=0.5)
            ax1.set_xlabel('Tick')
            ax1.set_ylabel('GPU Memory (MB)', color='g')

            ax2.plot(ticks, cpu_pct, 'b-', label='CPU %', alpha=0.5)
            ax2.set_ylabel('CPU %', color='b')

            ax1.set_title(f'Resources: GPU={summary.avg_gpu_memory_mb:.0f}MB, CPU={summary.avg_cpu_percent:.0f}%')
            ax1.legend(loc='upper left')
            ax2.legend(loc='upper right')

        fig.suptitle(f'Edge Performance: {summary.intersection_id}', fontsize=14)
        fig.tight_layout()

        self.cleanup()

        return fig, perform_txt, metrics


class FrameProfileContext:
    """Context manager for frame profiling"""

    def __init__(self, profiler: EdgeProfiler, tick: int):
        self.profiler = profiler
        self.tick = tick

    def __enter__(self) -> 'FrameProfileContext':
        self.profiler._start_frame(self.tick)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.profiler._end_frame()
        return False

    def time(self, name: str) -> 'ComponentTimerContext':
        """Time a component within this frame"""
        return ComponentTimerContext(self.profiler, name)

    def set_counts(self, **kwargs):
        """Set count metrics"""
        self.profiler.set_counts(**kwargs)

    def set_detection_metrics(self, **kwargs):
        """Set detection metrics (TP, FP, FN)"""
        self.profiler.set_detection_metrics(**kwargs)

    def set_tracking_metrics(self, **kwargs):
        """Set tracking metrics (ID switches, MOTA, etc.)"""
        self.profiler.set_tracking_metrics(**kwargs)

    def set_prediction_metrics(self, **kwargs):
        """Set prediction quality metrics (ADE, FDE, miss rate)"""
        self.profiler.set_prediction_metrics(**kwargs)

    def set_ego_uniqueness_metrics(self, **kwargs):
        """Set Ego-Uniqueness metrics (violations, duplicates, ghosts)"""
        self.profiler.set_ego_uniqueness_metrics(**kwargs)

    def set_contention_metrics(self, **kwargs):
        """Set compute-contention metrics (vehicles affected, budget exceeded)"""
        self.profiler.set_contention_metrics(**kwargs)

    def set_aoi_ticks(self, aoi_ticks: int):
        """Set Age-of-Information for this frame."""
        self.profiler.set_aoi_ticks(aoi_ticks)


class ComponentTimerContext:
    """Context manager for component timing"""

    def __init__(self, profiler: EdgeProfiler, name: str):
        self.profiler = profiler
        self.name = name

    def __enter__(self):
        self.profiler._start_component(self.name)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.profiler._end_component(self.name)
        return False


def calculate_prediction_errors(
    predictions: Dict[int, List],  # track_id -> list of predicted (x,y,z) positions
    ground_truth: Dict[int, List],  # track_id -> list of actual (x,y,z) positions
    dt: float = 0.05,  # timestep in seconds
    horizons_s: List[float] = [1.0, 2.0, 3.0]  # prediction horizons to evaluate
) -> Dict[str, float]:
    """
    Calculate prediction quality metrics by comparing predictions to ground truth.

    Parameters
    ----------
    predictions : Dict[int, List]
        Dictionary mapping track IDs to lists of predicted (x,y,z) positions
    ground_truth : Dict[int, List]
        Dictionary mapping track IDs to lists of actual (x,y,z) positions
    dt : float
        Simulation timestep in seconds
    horizons_s : List[float]
        Prediction horizons to evaluate in seconds

    Returns
    -------
    Dict[str, float]
        Dictionary with:
        - error_{horizon}s_m: ADE at each horizon
        - fde_m: Final displacement error
        - miss_rate: Fraction of predictions > 2m from GT
    """
    errors_by_horizon = {h: [] for h in horizons_s}
    fde_list = []
    miss_count = 0
    total_predictions = 0
    miss_threshold_m = 2.0  # Standard miss threshold

    for track_id in predictions:
        if track_id not in ground_truth:
            continue

        pred_traj = predictions[track_id]
        gt_traj = ground_truth[track_id]

        if not pred_traj or not gt_traj:
            continue

        # Calculate errors at each timestep
        min_len = min(len(pred_traj), len(gt_traj))
        if min_len == 0:
            continue

        for i in range(min_len):
            pred_pos = np.array(pred_traj[i][:2])  # x, y only
            gt_pos = np.array(gt_traj[i][:2])
            error = np.linalg.norm(pred_pos - gt_pos)

            # Check which horizon this timestep corresponds to
            time_s = i * dt
            for horizon in horizons_s:
                if abs(time_s - horizon) < dt / 2:
                    errors_by_horizon[horizon].append(error)

            # Track miss rate
            total_predictions += 1
            if error > miss_threshold_m:
                miss_count += 1

        # FDE is error at last timestep
        if min_len > 0:
            pred_final = np.array(pred_traj[min_len - 1][:2])
            gt_final = np.array(gt_traj[min_len - 1][:2])
            fde_list.append(np.linalg.norm(pred_final - gt_final))

    # Calculate averages
    result = {}
    for horizon in horizons_s:
        key = f'error_{int(horizon)}s_m'
        if errors_by_horizon[horizon]:
            result[key] = float(np.mean(errors_by_horizon[horizon]))
        else:
            result[key] = 0.0

    result['fde_m'] = float(np.mean(fde_list)) if fde_list else 0.0
    result['miss_rate'] = miss_count / total_predictions if total_predictions > 0 else 0.0

    return result


def calculate_detection_metrics(
    detections: List,  # List of detected bounding boxes (x, y, z, ...)
    ground_truth: List,  # List of ground truth bounding boxes
    iou_threshold: float = 0.5,
    distance_threshold: float = 2.0  # meters
) -> Dict[str, int]:
    """
    Calculate detection metrics by matching detections to ground truth.

    Uses distance-based matching (simpler than full IoU for 3D).

    Parameters
    ----------
    detections : List
        List of detected positions/boxes
    ground_truth : List
        List of ground truth positions/boxes
    iou_threshold : float
        IoU threshold for matching (not used currently, uses distance)
    distance_threshold : float
        Maximum distance for a detection to match ground truth

    Returns
    -------
    Dict[str, int]
        Dictionary with true_positives, false_positives, false_negatives
    """
    if not detections and not ground_truth:
        return {'true_positives': 0, 'false_positives': 0, 'false_negatives': 0}

    if not ground_truth:
        return {'true_positives': 0, 'false_positives': len(detections), 'false_negatives': 0}

    if not detections:
        return {'true_positives': 0, 'false_positives': 0, 'false_negatives': len(ground_truth)}

    # Simple greedy matching based on distance
    gt_matched = set()
    det_matched = set()

    # Extract positions from detections and ground truth
    det_positions = []
    for d in detections:
        if hasattr(d, 'location'):
            det_positions.append(np.array([d.location.x, d.location.y, d.location.z]))
        elif hasattr(d, 'bounding_box'):
            loc = d.bounding_box.location
            det_positions.append(np.array([loc.x, loc.y, loc.z]))
        elif isinstance(d, (list, tuple, np.ndarray)) and len(d) >= 3:
            det_positions.append(np.array(d[:3]))
        else:
            continue

    gt_positions = []
    for g in ground_truth:
        if hasattr(g, 'location'):
            gt_positions.append(np.array([g.location.x, g.location.y, g.location.z]))
        elif hasattr(g, 'get_location'):
            loc = g.get_location()
            gt_positions.append(np.array([loc.x, loc.y, loc.z]))
        elif isinstance(g, (list, tuple, np.ndarray)) and len(g) >= 3:
            gt_positions.append(np.array(g[:3]))
        else:
            continue

    # Greedy matching: match each detection to closest unmatched GT
    for det_idx, det_pos in enumerate(det_positions):
        best_dist = float('inf')
        best_gt_idx = -1

        for gt_idx, gt_pos in enumerate(gt_positions):
            if gt_idx in gt_matched:
                continue
            dist = np.linalg.norm(det_pos - gt_pos)
            if dist < best_dist and dist < distance_threshold:
                best_dist = dist
                best_gt_idx = gt_idx

        if best_gt_idx >= 0:
            gt_matched.add(best_gt_idx)
            det_matched.add(det_idx)

    true_positives = len(det_matched)
    false_positives = len(det_positions) - true_positives
    false_negatives = len(gt_positions) - len(gt_matched)

    return {
        'true_positives': true_positives,
        'false_positives': false_positives,
        'false_negatives': false_negatives
    }
