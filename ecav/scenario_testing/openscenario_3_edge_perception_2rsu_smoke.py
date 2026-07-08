# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Object-sharing I2V smoke runner (PerceptionEdge, 2 RSU).

Edge ships detections only; the vehicle tracks + predicts + plans locally
(VehicleSideTracker). Delegates to the base late-fusion runner; the edge
class is chosen from the YAML manager_type (perception).
"""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner,
    run_vehicle,
    run_scenario,
)

SCENARIO_NAME = 'openscenario_3_edge_perception_2rsu_smoke'
