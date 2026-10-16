# -*- coding: utf-8 -*-
"""V2X2V object sharing with VIPS time rectification, on the late-fusion
pipeline.

Identical to PredictionLateFusionEdge in every step but one: each drained
frame's detections are moved from their observation tick to the tick being
processed by the velocity of the track they associate with, before
association (VIPS; arrival order). Nothing about the tracker's past state is
rewritten, so this is time rectification, not out-of-sequence retrodiction,
and the age the planner sees still follows the observation (obs_tick is the
frame's tick): rectification predicts, it does not refresh.

The earlier standalone VIPS manager (edge_manager_vips_temporal_alignment,
still registered as VIPS_TEMPORAL) carried its own detection, publish and
identity paths and at zero delay published the vehicle's own track at rest
behind the vehicle (Oct 16, 2 of 5 runs braked on it); the arm must differ
from the naive arm by rectification alone, which this class guarantees.
"""
from .edge_manager_prediction_late_fusion_ab3dmot_linear_predictor import (
    PredictionLateFusionEdge,
)
from .edge_manager_vips_temporal_alignment import _time_rectify_detections


class VIPSLateFusionEdge(PredictionLateFusionEdge):

    def _collect_detections_for_frame(self, objects, beacons, source_tick):
        dets_all = super()._collect_detections_for_frame(objects, beacons, source_tick)
        now = getattr(self, "_drain_tick", None)
        if now is None or len(dets_all.get("dets", ())) == 0:
            return dets_all
        return _time_rectify_detections(dets_all, int(source_tick), int(now), self.dt, self.tracker)
