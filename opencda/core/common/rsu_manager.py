

# -*- coding: utf-8 -*-
"""
Road-Side-Unit (RSU) manager.

• Loads either the classic OpenCDA perception pipeline **or** the BM2CP
  multi-modal network, depending on the YAML entry:

    sensing:
      perception:
        backend: bm2cp     #  "bm2cp"  or  "default" (classic)

"""

# Author: Tyler Landle <tlandle3@gatech.edu> for BM2CP integration
# License: TDG-Attribution-NonCommercial-NoDistrib
# -----------------------------------------------------------------------------

from opencda.core.common.data_dumper import DataDumper
from opencda.core.sensing.localization.rsu_localization_manager import \
    LocalizationManager
from opencda.core.sensing.tracking.tracking_manager import TrackingManager

# opencda/core/common/rsu_manager.py
# … imports unchanged …

# ------------------------------------------------------------------ #
#  Runtime backend selector
# ------------------------------------------------------------------ #
def _pick_perception_class(percep_yaml: dict):
    backend = str(percep_yaml.get("backend", "default")).lower()
    if backend in ("bm2cp", "fusion"):
        from opencda.core.sensing.perception.bm2cp_perception_manager import (
            BM2CPPerceptionManager,
        )
        return BM2CPPerceptionManager

    if backend == "worldfusion":
        from opencda.core.sensing.perception.worldfusion_perception_manager import (
            WorldFusionPerceptionManager,
        )
        return WorldFusionPerceptionManager

    from opencda.core.sensing.perception.perception_manager import (
        PerceptionManager,
    )
    return PerceptionManager


# ------------------------------------------------------------------ #
#  RSU Manager
# ------------------------------------------------------------------ #
class RSUManager:
    # … docstring & __init__ header unchanged …
    def __init__(
        self,
        carla_world,
        config_yaml,
        carla_map,
        cav_world,
        current_time="",
        data_dumping=False,
    ):
        self.rid = -abs(config_yaml.get("id", -1))

        # ------------------- build local copies -------------------- #
        sensing_cfg = config_yaml["sensing"]
        spawn_pos   = config_yaml["spawn_position"]

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
        PercepCls  = _pick_perception_class(sensing_cfg["perception"])
        backend_id = PercepCls.__name__


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

    # ------------------- public API (unchanged) ------------------- #
    def update_info(self):
        self.objects.clear()
        self.localizer.localize()
        ego_pos = self.localizer.get_ego_pos()
        self.objects = self.perception_manager.detect(ego_pos)

    def run_step(self):
        if self.data_dumper:
            self.data_dumper.run_step(self.perception_manager,
                                      self.localizer,
                                      planner=None)

    def destroy(self):
        self.perception_manager.destroy()
        self.localizer.destroy()

