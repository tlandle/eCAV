# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Infra-only (I2V) smoke runner for Scenario 3 (LTAP).

The edge-manager class is selected from the YAML ``manager_type`` field
(``infra_only`` here), so the run loop is identical to the late-fusion
runner. This module only delegates so that ``-t
openscenario_3_edge_infra_only_smoke`` resolves to a module and loads the
matching config of the same name.
"""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner,
    run_vehicle,
    run_scenario,
)

SCENARIO_NAME = 'openscenario_3_edge_infra_only_smoke'
