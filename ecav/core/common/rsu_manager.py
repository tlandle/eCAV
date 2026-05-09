# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Road-Side-Unit (RSU) manager.

Loads either the classic eCAV perception pipeline, BM2CP, or
WorldFusion, depending on the YAML sensing.perception.backend entry.
"""

import logging

from ecav.core.common.data_dumper import DataDumper
from ecav.core.sensing.localization.rsu_localization_manager import \
    LocalizationManager
from ecav.core.sensing.tracking.tracking_manager import TrackingManager

logger = logging.getLogger(__name__)

# ------------------------------------------------------------------ #
#  Runtime backend selector
# ------------------------------------------------------------------ #
def _pick_perception_class(percep_yaml: dict):
    backend = str(percep_yaml.get("backend", "default")).lower()
    if backend in ("bm2cp", "fusion"):
        from ecav.core.sensing.perception.bm2cp_perception_manager import (
            BM2CPPerceptionManager,
        )
        return BM2CPPerceptionManager

    if backend == "worldfusion":
        from ecav.core.sensing.perception.worldfusion_perception_manager import (
            WorldFusionPerceptionManager,
        )
        return WorldFusionPerceptionManager

    from ecav.core.sensing.perception.perception_manager import (
        PerceptionManager,
    )
    return PerceptionManager


# ------------------------------------------------------------------ #
#  RSU Manager
# ------------------------------------------------------------------ #
class RSUManager:
    def __init__(
        self,
        carla_world,
        config_yaml,
        carla_map,
        cav_world,
        current_time="",
        data_dumping=False,
        run_distributed=False,
        is_proxy=False,
    ):
        self.rid = -abs(config_yaml.get("id", -1))

        # ------------------- build local copies -------------------- #
        sensing_cfg = config_yaml["sensing"]
        spawn_pos   = config_yaml["spawn_position"]
        self.spawn_position = spawn_pos

        # absolute world position for this fixed RSU
        sensing_cfg.setdefault("localization", {})["global_position"] = spawn_pos
        sensing_cfg.setdefault("perception",   {})["global_position"] = spawn_pos

        # ------------------- localisation -------------------------- #
        self.localizer = LocalizationManager(
            carla_world,
            sensing_cfg["localization"],
            carla_map,
        )

        # ------------------- tracking ------------------------------ #
        self.tracking_manager = TrackingManager(
            None,
            cav_world,
            data_dump=data_dumping,
            carla_world=carla_world,
            infra_id=self.rid,
            tracker_type="SORT",
        )

        # ------------------- perception ---------------------------- #
        # Proxy RSUs (root process in distributed mode) receive features via gRPC
        # and skip local perception. run_distributed is passed explicitly by the caller
        # (same pattern as VehicleManager) — do NOT read cav_world.run_distributed,
        # which reflects the YAML 'distributed' key and stays False even with -d active.
        is_server_proxy = is_proxy or run_distributed

        if is_server_proxy:
            from ecav.core.sensing.perception.perception_manager import PerceptionManager
            PercepCls = PerceptionManager
            # Suppress all sensor spawning — proxy RSUs receive detections via gRPC
            # and never run local perception. Must use nested keys; PerceptionManager
            # reads config_yaml['camera']['visualize'], not a flat 'camera_visualize'.
            sensing_cfg["perception"]["activate"] = False
            sensing_cfg["perception"]["camera"]["visualize"] = 0
            sensing_cfg["perception"]["lidar"]["visualize"] = False
            logger.info("[RSUManager] Distributed proxy — using base PerceptionManager")
        else:
            PercepCls = _pick_perception_class(sensing_cfg["perception"])

        self.perception_manager = PercepCls(
            vehicle=None,                         # RSU is static
            config_yaml=sensing_cfg["perception"],
            cav_world=cav_world,
            carla_world=carla_world,
            data_dump=data_dumping,
            infra_id=self.rid,
            tracking_manager=self.tracking_manager,
        )
     
        # ------------------- misc ---------------------------------- #
        self.objects = {}
        self.data_dumper = (
            DataDumper(self.perception_manager, self.rid, save_time=current_time)
            if data_dumping
            else None
        )
        cav_world.update_rsu_manager(self)

    # ------------------- public API -------------------------------- #
    def update_info(self):
        import time as _t
        t0 = _t.time()
        self.objects.clear()
        self.localizer.localize()
        t_loc = _t.time()
        ego_pos = self.localizer.get_ego_pos()
        self.objects = self.perception_manager.detect(ego_pos)
        t_det = _t.time()
        logger.debug("[RSU update_info] localize=%.0fms | detect=%.0fms | total=%.0fms",
                     (t_loc - t0) * 1000, (t_det - t_loc) * 1000, (t_det - t0) * 1000)

    def run_step(self):
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      planner=None)

    def destroy(self):
        self.perception_manager.destroy()
        self.localizer.destroy()

