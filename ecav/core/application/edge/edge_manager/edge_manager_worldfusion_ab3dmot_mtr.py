"""WorldFusion edge manager with AB3DMOT tracking and MTR prediction.

Inherits all fusion, tracking, detection, and distribution logic from
WorldFusionEdge (linear predictor variant). Overrides only the predictor
initialization to use MTR instead of linear prediction.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>

from typing import Dict, Any
import carla

from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor
from .edge_manager_worldfusion_ab3dmot_linear_predictor import WorldFusionEdge


class WorldFusionMTREdge(WorldFusionEdge):
    """WorldFusion + AB3DMOT + MTR prediction (no adaptation)."""

    def __init__(self, world: carla.World, cfg: Dict[str, Any],
                 cav_world, carla_client: carla.Client,
                 *, world_dt: float = 0.05, **kwargs):
        # Let parent init everything including linear predictor
        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kwargs)

        # Replace the predictor with MTR
        mtr_cfg = cfg.get('mtr_predictor', {})
        self.predictor = MTREdgePredictor(
            cmp_root=mtr_cfg.get('cmp_root'),
            mtr_checkpoint=mtr_cfg.get('checkpoint'),
            cfg_yaml=mtr_cfg.get('cfg_yaml'),
            intention_points_file=mtr_cfg.get('intention_points_file'),
            dataset=mtr_cfg.get('dataset', 'opv2v'),
            aggregator=mtr_cfg.get('aggregator'),
            past_frames=mtr_cfg.get('past_frames', 10),
            future_frames=mtr_cfg.get('future_frames', 50),
            time_interval=mtr_cfg.get('time_interval', 0.1),
            history_subsample=mtr_cfg.get('history_subsample', 1),
            device=mtr_cfg.get('device', 'cuda'),
            # One entry per sim tick (predictor resamples model steps to
            # output_dt); cover the full model horizon by default.
            num_output_steps=mtr_cfg.get('num_output_steps', 100),
            output_dt=world_dt,
            lane_map=(mtr_cfg.get('lane_map')
                      if mtr_cfg.get('lane_map') != 'auto' else None),
            budget_ms=1000.0,           # no budget constraint
            enable_amortization=False,  # no adaptation
            enable_risk_budget=False,   # no adaptation
            ego_vehicles=[vm.vehicle for vm in self.vehicle_manager_list],
        )
        self.lin_pred = self.predictor

        if mtr_cfg.get('lane_map') == 'auto':
            self._set_auto_lane_raster(world, mtr_cfg)
        print("[WorldFusion MTR Edge] Using MTR predictor (no adaptation)")
