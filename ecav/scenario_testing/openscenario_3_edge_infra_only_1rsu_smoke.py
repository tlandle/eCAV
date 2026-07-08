# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Single-RSU infra-only (I2V) smoke runner for Scenario 3 (LTAP).

Ladder rung 2: direct infrastructure-only envelope without multi-source
ambiguity. Delegates to the late-fusion runner; the edge-manager class is
chosen from the YAML ``manager_type`` field (``infra_only``).
"""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner,
    run_vehicle,
    run_scenario,
)

SCENARIO_NAME = 'openscenario_3_edge_infra_only_1rsu_smoke'
