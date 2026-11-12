#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""Scale scenario on Town06: N ego vehicles on the straight carriageways, no other actors.

The ego entries (positions, role names hero, hero1, ...) come from
scenario_scale_town06.xml, written by scripts/ispass/gen_scale_town06.py. In
distributed mode the client with vehicle index 0 spawns every ego and each client
k drives ego k; the run ends when the orchestrator sends END (its step limit), so
the behaviour tree only has to stay alive.
"""
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenarios.basic_scenario import BasicScenario

TAG = "[SCALE_TOWN06]"


class Scenario_ScaleTown06(BasicScenario):
    """N egos cruising on Town06; no scripted actors."""

    timeout = 100000

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=100000, vehicle_index=-1,
                 scenario_params=None, distributed=False):
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        print(f"{TAG} {len(ego_vehicles)} egos, vehicle_index {vehicle_index}, distributed {distributed}", flush=True)
        super(Scenario_ScaleTown06, self).__init__(
            "Scenario_ScaleTown06", ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable, vehicle_index=vehicle_index,
            scenario_params=scenario_params)

    def _initialize_actors(self, config):
        return

    def _create_behavior(self):
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(Idle())
        return root

    def _create_test_criteria(self):
        k = self.vehicle_index if (self.distributed and self.vehicle_index >= 0) else 0
        return [CollisionTest(self.ego_vehicles[k])]

    def __del__(self):
        self.remove_all_actors()
