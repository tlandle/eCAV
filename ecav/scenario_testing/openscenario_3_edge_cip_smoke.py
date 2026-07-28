# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""CIP-style edge smoke runner (edge plans, vehicle actuates). Delegates to the
late-fusion runner; edge class chosen from YAML manager_type (cip)."""
from ecav.scenario_testing.openscenario_3_edge_late_fusion import (
    exec_scenario_runner, run_vehicle, run_scenario,
)
SCENARIO_NAME = 'openscenario_3_edge_cip_smoke'
