# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Late-fusion smoke runner for Scenario 3 (LTAP).

Delegates to the base late-fusion runner; edge-manager class and
anchoring/SBA are selected from the YAML. Exists so that
``-t openscenario_3_edge_late_fusion_smoke`` resolves to a module and
loads the matching config of the same name.
"""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner,
    run_vehicle,
    run_scenario,
)

SCENARIO_NAME = 'openscenario_3_edge_late_fusion_smoke'
