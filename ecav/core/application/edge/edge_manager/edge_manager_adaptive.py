"""Adaptive edge manager: runtime controller with multiple predictor tiers.

Loads multiple predictors at startup (heavy + light). The EdgeController
selects which tier to use per-tick based on queue depth, deadline
pressure, and scene density. Can skip frames and degrade fusion fidelity.

This is the contribution that shifts the deadline cliff rightward
(Paper 2).

YAML:
    manager_type: adaptive_edge
    fusion_backend: worldfusion
    tracker: ab3dmot
    predictors:
      heavy:
        type: smart
        checkpoint: ecav/core/prediction/smart/checkpoints/epoch=105.ckpt
        device: cuda
      light:
        type: linear
        num_future_steps: 25
    controller:
      policy: adaptive
      deadline_ms: 200
      queue_depth_thresh: 3
    tracker_cfg:
      min_hits: 3
      max_age: 6
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
from typing import Any, Dict

from ecav.core.application.edge.edge_manager.edge_manager_pluggable_base import (
    _PluggableEdgeBase)
from ecav.core.application.edge.edge_controller import EdgeController
from ecav.core.prediction import get_predictor

logger = logging.getLogger(__name__)


class AdaptiveEdge(_PluggableEdgeBase):
    """Adaptive pipeline with runtime controller and multi-tier predictors."""

    def __init__(self, world, cfg, cav_world, carla_client, *,
                 world_dt=0.05, **kwargs):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        # Multiple predictor tiers
        predictors_cfg = cfg.get('predictors', {})
        self.predictors: Dict[str, Any] = {}
        for tier_name, pred_cfg in predictors_cfg.items():
            pred_type = pred_cfg.pop('type', 'linear')
            self.predictors[tier_name] = get_predictor(pred_type, pred_cfg)
            logger.info("[AdaptiveEdge] Predictor '%s': %s",
                        tier_name, pred_type)

        # Ensure fallbacks exist
        if 'light' not in self.predictors:
            self.predictors['light'] = get_predictor(
                'linear', {'num_future_steps': 25})
        if 'heavy' not in self.predictors:
            self.predictors['heavy'] = self.predictors['light']

        # Controller
        ctrl_cfg = cfg.get('controller', {'policy': 'adaptive'})
        self.controller = EdgeController(ctrl_cfg)
        logger.info("[AdaptiveEdge] Controller: %s",
                    ctrl_cfg.get('policy', 'adaptive'))

        self._last_predictions = []

    def run_step(self, tick):
        with self.profiler.profile_frame(tick) as frame:
            with frame.time("feature_collection"):
                self.update_information(tick)
                queue_depth = len(self._jitter_buffer)

            # Update rate knob: skip frame?
            if self.controller.should_skip_frame(queue_depth):
                self._advance_vehicles(tick, self._last_predictions)
                return None

            with frame.time("tracking"):
                num_tracks = self._drain_and_track(tick, frame)

            # Predictor tier knob
            with frame.time("prediction"):
                stage_latencies = frame.get_stage_times()
                tier = self.controller.select_predictor(
                    queue_depth, stage_latencies, num_tracks)

                predictions = self.predictors[tier].generate_predicted_trajectories(
                    self.tracked_trajectories,
                    source_tick=self._latest_source_tick,
                    publish_tick=tick)

                logger.debug("[AdaptiveEdge] tick=%d tier=%s tracks=%d preds=%d",
                             tick, tier, num_tracks, len(predictions))

            self._last_predictions = predictions

            # Record for controller
            self.controller.record_tick(queue_depth, frame.get_stage_times())

            self._advance_vehicles(tick, predictions)

            frame.set_counts(
                num_agents=(len(self.vehicle_manager_list)
                            + len(self.rsu_manager_list)),
                num_detections=0,
                num_tracks=num_tracks,
                num_predictions=len(predictions))

            return None
