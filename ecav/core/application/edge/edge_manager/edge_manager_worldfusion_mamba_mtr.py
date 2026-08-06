"""WorldFusion edge manager with mamba3dmot tracking, MTR prediction, and
the adaptive deadline controller.

Combination: Where2Comm fusion + Mamba3DMOT + MTR (stage-1, no aggregator)
+ adaptive controller. This is the RELAY production pipeline: the MTR
checkpoint is trained on mamba3dmot-produced pasts (train = live), and the
migration daemon hands off mamba3dmot hidden state across locales.

Differences from the AB3DMOT parent:
- tracker: Mamba3DMOTWrapper via the tracker registry (yaml `tracker_cfg`).
- detections are fed in PLAIN axes ([h,w,l,x,y,z,yaw,score]) — the offline
  retrack tool trains the tracker on plain-axis boxes, so the live feed
  must match; the parent's KITTI swap is undone in the hook.
- replay parses plain-axis rows back to boxes.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>

from typing import Dict, Any
import carla
import numpy as np

from ecav.core.tracking import get_tracker
from .edge_manager_worldfusion_ab3dmot_mtr_adaptive import (
    WorldFusionAdaptiveEdge)
from .edge_manager_pluggable_base import _PluggableEdgeBase


class WorldFusionMambaAdaptiveEdge(WorldFusionAdaptiveEdge):
    """WorldFusion + Mamba3DMOT + MTR + adaptive controller."""

    # Tracker-agnostic migration surface borrowed from _PluggableEdgeBase:
    # duck-typed dispatch exports the Mamba memo-bank latent (or an AB3DMOT
    # KF snapshot) instead of the AB3DMOT-only mixin the WF chain inherits.
    # Prerequisites (tracker wrapper, beacon_id_mgr, track_to_carla,
    # _vm_by_carla_id) all exist on this chain.
    _raw_tracker = _PluggableEdgeBase._raw_tracker
    _is_mamba = staticmethod(_PluggableEdgeBase._is_mamba)
    _resolved_carla_id = _PluggableEdgeBase._resolved_carla_id
    _export_track_latent = _PluggableEdgeBase._export_track_latent
    _import_track_latent = _PluggableEdgeBase._import_track_latent
    export_vehicle_state = _PluggableEdgeBase.export_vehicle_state
    import_vehicle_state = _PluggableEdgeBase.import_vehicle_state
    export_tracked_obstacle_state = \
        _PluggableEdgeBase.export_tracked_obstacle_state
    import_tracked_obstacle_state = \
        _PluggableEdgeBase.import_tracked_obstacle_state

    _DEFAULT_TRACKER_CFG = {
        'motion_model_path':
            'ecav/core/tracking/mamba3dmot/mamba3dmot_weights.pth',
        'filter_thresh': 0.05,
        'new_track_thresh': 0.2,
        'match_thresh': 5.0,
        'max_time_lost': 60,
        'enable_time_thresh': 5,
        'max_window': 10,
    }

    def __init__(self, world: carla.World, cfg: Dict[str, Any],
                 cav_world, carla_client: carla.Client,
                 *, world_dt: float = 0.05, **kwargs):
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        tracker_cfg = {**self._DEFAULT_TRACKER_CFG,
                       **cfg.get('tracker_cfg', {})}
        self.tracker = get_tracker('mamba3dmot', tracker_cfg)
        self.mot_tracker = self.tracker
        print("[WorldFusion Mamba Edge] Using mamba3dmot tracker "
              f"(match_thresh={tracker_cfg['match_thresh']}, "
              f"max_time_lost={tracker_cfg['max_time_lost']})")

    def _format_dets_for_tracker(self, det_results):
        """Undo the WF pipeline's KITTI swap: cols 4/5 back to plain
        [h,w,l,x,y_world,z,yaw,score], matching the offline retrack feed."""
        dets = det_results.get('dets')
        if dets is not None and len(dets):
            dets = np.asarray(dets, dtype=np.float32).copy()
            dets[:, [4, 5]] = dets[:, [5, 4]]
            det_results = dict(det_results)
            det_results['dets'] = dets
        return det_results

    def _track_row_to_box(self, trk):
        """Plain-axis tracker row [h,w,l,x,y,z,yaw,...] -> [x,y,z,h,w,l,yaw]."""
        return np.array([trk[3], trk[4], trk[5], trk[0], trk[1], trk[2],
                         trk[6]])
