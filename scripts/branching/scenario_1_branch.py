#!/usr/bin/env python
# Branching-junction scenario (NSDI experiment 1). One road user (the NPC,
# other_actors[0]) approaches a k-way junction and takes the branch named by
# BRANCH_TAKEN (a locale id from branching_sites.json), routed with the CARLA
# GlobalRoutePlanner so the turn is committed late and is not inferable from
# the approach geometry. Each branch holds a recipient CAV (ego_vehicles[i],
# XML order == branch order) behind a stopped blocker (other_actors[1+i]); the
# CAV must overtake the blocker into the NPC's lane at the branch's conflict
# point, so the NPC is the oncoming for whichever branch it takes and the
# overtake gate / first-use / collision machinery of the flow scenario is
# reused unchanged. Blockers are static from spawn; only the NPC is hidden
# (z-500 in the XML) and released by a trigger on the taken branch's CAV.
import json
import logging
import math
import os
import random

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter, Idle, WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance, InTriggerDistanceToLocation)
from srunner.scenarios.basic_scenario import BasicScenario

logger = logging.getLogger(__name__)

SITES_JSON = os.environ.get(
    'BRANCH_SITES_JSON', 'ecav/scenario_testing/scenarios/branching_sites.json')


class Scenario_1(BasicScenario):

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=180):
        self._map = CarlaDataProvider.get_map()
        self.timeout = timeout
        self.site_key = os.environ['BRANCH_SITE']            # k2 | k3 | k4
        self.site = json.load(open(SITES_JSON))[self.site_key]
        self.branches = [l for l in self.site['locales']
                         if l['id'].startswith('locale_branch_')]
        self.taken = os.environ['BRANCH_TAKEN']              # locale id
        self.taken_idx = [b['id'] for b in self.branches].index(self.taken)
        self.npc_speed = float(os.environ.get('ONCOMING_SPEED', '12') or 12.0)
        self._trigger_distance = float(os.environ.get('TRIGGER_DIST', '300') or 300.0)
        self.agents = []
        self.npc_plan = None
        super(Scenario_1, self).__init__("Scenario_1", ego_vehicles, config,
                                         world, debug_mode,
                                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        npc = self.other_actors[0]
        t = npc.get_transform()
        seed_idx = int(os.environ.get('KHONSU_SEED', '0') or 0)
        band = float(os.environ.get('KHONSU_SEED_BAND_M', '4.0') or 4.0)
        off = random.Random(seed_idx).uniform(-band, band) if seed_idx > 0 else 0.0
        fwd = t.get_forward_vector()
        # spawn-phase seed: shift the NPC back along its own heading by `off`
        self.npc_visible = carla.Transform(
            carla.Location(t.location.x - fwd.x * off, t.location.y - fwd.y * off,
                           t.location.z + 501.0), t.rotation)
        logger.info("[SEEDROW] seed_idx=%d band_m=%.2f spawn_offset_m=%.3f "
                    "site=%s taken=%s taken_idx=%d",
                    seed_idx, band, off, self.site_key, self.taken, self.taken_idx)
        self.npc_trigger_location = carla.Location(
            self.npc_visible.location.x, self.npc_visible.location.y,
            self.npc_visible.location.z)

        # route: visible spawn -> the taken branch's route end (outward lane,
        # min(L-3, 45) m past the mouth, from branching_sites.json)
        br = self.branches[self.taken_idx]
        ex, ey, ez = br['npc_route_end']
        end = carla.Location(ex, ey, ez)
        from agents.navigation.global_route_planner import GlobalRoutePlanner
        grp = GlobalRoutePlanner(self._map, 2.0)
        self.npc_plan = grp.trace_route(self.npc_visible.location, end)
        n_turn = sum(1 for _, ro in self.npc_plan if str(ro).endswith(('LEFT', 'RIGHT')))
        logger.info("[BRANCHROUTE] taken=%s plan_len=%d turn_steps=%d "
                    "end=(%.1f,%.1f)", self.taken, len(self.npc_plan), n_turn,
                    end.x, end.y)
        self.npc_route_end = end

    def _create_behavior(self):
        npc = self.other_actors[0]
        taken_ego = self.ego_vehicles[self.taken_idx]
        seq = py_trees.composites.Sequence("branching_npc")
        seq.add_child(ActorTransformSetter(npc, self.npc_visible))
        # release the NPC when the taken branch's CAV is within TRIGGER_DIST of
        # the NPC's spawn; tuned per site in the smoke (Oracle ~100% ready).
        seq.add_child(InTriggerDistanceToLocation(
            taken_ego, self.npc_trigger_location, self._trigger_distance))
        drive = WaypointFollower(npc, self.npc_speed, plan=self.npc_plan)
        seq.add_child(drive)
        seq.add_child(Idle())
        self.agents.append(drive)

        root = py_trees.composites.Parallel(
            "Parallel Behavior",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(seq)
        # end when the NPC reaches its route end, or the taken branch's CAV has
        # driven through the junction (it starts ~79 m before the mouth).
        root.add_child(InTriggerDistanceToLocation(npc, self.npc_route_end, 4.0))
        root.add_child(DriveDistance(taken_ego, 130.0))
        return root

    def _create_test_criteria(self):
        # collision on every CAV; the lander scores the taken branch's CAV
        return [CollisionTest(ego) for ego in self.ego_vehicles]

    def __del__(self):
        self.remove_all_actors()
