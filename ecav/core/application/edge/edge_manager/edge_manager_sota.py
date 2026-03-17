"""SOTA edge manager: static pipeline, no adaptation.

Always runs the configured fusion + tracker + predictor at full
fidelity. No frame skipping, no tier switching, no degradation.

This is the baseline for characterizing the deadline cliff under
dense scale (Paper 2).

YAML:
    manager_type: sota_edge
    fusion_backend: worldfusion
    tracker: ab3dmot
    predictor: smart
    predictor_cfg:
      checkpoint: ecav/core/prediction/smart/checkpoints/epoch=105.ckpt
      device: cuda
    tracker_cfg:
      min_hits: 3
      max_age: 6
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging

from ecav.core.application.edge.edge_manager.edge_manager_pluggable_base import (
    _PluggableEdgeBase)
from ecav.core.prediction import get_predictor

logger = logging.getLogger(__name__)


class SOTAEdge(_PluggableEdgeBase):
    """Static SOTA pipeline: always runs the configured predictor."""

    def __init__(self, world, cfg, cav_world, carla_client, *,
                 world_dt=0.05, **kwargs):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        pred_type = cfg.get('predictor', 'linear')
        pred_cfg = cfg.get('predictor_cfg', {})
        self.predictor = get_predictor(pred_type, pred_cfg)
        logger.info("[SOTAEdge] Predictor: %s", pred_type)

    def run_step(self, tick):
        with self.profiler.profile_frame(tick) as frame:
            with frame.time("feature_collection"):
                self.update_information(tick)

            with frame.time("tracking"):
                num_tracks = self._drain_and_track(tick, frame)

            with frame.time("prediction"):
                predictions = self.predictor.generate_predicted_trajectories(
                    self.tracked_trajectories,
                    source_tick=self._latest_source_tick,
                    publish_tick=tick)

            self._advance_vehicles(tick, predictions)

            frame.set_counts(
                num_agents=(len(self.vehicle_manager_list)
                            + len(self.rsu_manager_list)),
                num_detections=0,
                num_tracks=num_tracks,
                num_predictions=len(predictions))

            return None
