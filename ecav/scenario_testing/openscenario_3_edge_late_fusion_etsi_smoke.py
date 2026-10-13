# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""ETSI self-announcement smoke runner for Scenario 3 (LTAP).

Delegates to the base late-fusion runner; the edge-manager class
(late_fusion_etsi_self_announcement) is selected from the YAML of the
same name. Exists so that ``-t openscenario_3_edge_late_fusion_etsi_smoke``
resolves to a module and loads the matching config.
"""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner,
    run_vehicle,
    run_scenario,
)

SCENARIO_NAME = 'openscenario_3_edge_late_fusion_etsi_smoke'
