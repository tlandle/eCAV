#!/usr/bin/env python

"""Scenario B — Town06 right-merge / obstacle hand-off.

Ego drives east in the LEFTMOST eastbound lane -3 (y~136.5). The lane's left
boundary is a solid line onto the shoulder, so no left escape exists. A
stationary emergency vehicle blocks lane -3 ahead: ego's only way onward is a
RIGHT merge into lane -4 (y~140) — the lane a fast NPC approaches in from
behind. The NPC lane-follows an explicit waypoint plan at constant speed
(the proven scenario_3 Lincoln pattern; a plan-less WaypointFollower stalled,
and SyncArrival steers 0 and drifts out of lane) timed to threaten the merge
zone while ego is stopped behind the emergency vehicle. Once the NPC passes,
lane -4 clears and ego merges. As the NPC crosses the locale 0 -> locale 1
boundary, the ecav loop fires an obstacle hand-off so locale 1 gets the NPC's
KF history before its own RSU can see it.

Actor layout:
    ego_vehicles[0]    = CAV (lane -3, ~43 km/h east)
    other_actors[0]    = emergency vehicle (stationary, blocks lane -3)
    other_actors[1]    = fast NPC (lane -4, constant speed east)
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    WaypointFollower,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
)
from srunner.scenarios.basic_scenario import BasicScenario


class Scenario_MultiEdgeRightMerge(BasicScenario):
    """Town06 forced-right-merge with a fast NPC crossing the locale boundary."""

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=600, vehicle_index=-1,
                 scenario_params=None, distributed=False):
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # scenario_params is a list of "key=value" strings (same convention as scenario_3).
        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 43))

        # ── NPC timing (tunable) ──────────────────────────────────────
        # Constant-speed lane follower along lane -4. At 18 m/s the NPC:
        #   crosses the locale boundary overlap (x~156)  at ~tick 190
        #   passes ego's stop zone (x~270)               at ~tick 315
        # while ego (43 km/h from x=100 in lane -3) stops behind the emergency
        # vehicle around tick 350 — so lane -4 is threatened exactly during
        # ego's merge decision, then clears as the NPC drives on.
        self.npc_speed_mps = 18.0
        # Waypoints along lane -4 (y from map probe; projected to lane center
        # by the follower). The NPC keeps going past the scene so the lane
        # clears behind it.
        self.npc_plan = [
            carla.Location(x=50.0, y=139.7, z=0.5),
            carla.Location(x=150.0, y=140.1, z=0.5),
            carla.Location(x=250.0, y=140.5, z=0.5),
            carla.Location(x=350.0, y=140.9, z=0.5),
            carla.Location(x=450.0, y=141.2, z=0.5),
        ]
        self.terminate_drive_distance = 300.0

        super(Scenario_MultiEdgeRightMerge, self).__init__(
            "Scenario_MultiEdgeRightMerge",
            ego_vehicles,
            config,
            world,
            debug_mode,
            criteria_enable=criteria_enable,
            vehicle_index=vehicle_index,
            scenario_params=scenario_params)

    def _initialize_actors(self, config):
        # Ego/RSU containers (distributed) don't spawn background actors.
        if self.distributed and self.vehicle_index >= 0:
            return

        # Spawn all other_actors at the XML transforms (z=-500 keeps them
        # underground); ActorTransformSetter teleports them up in the tree.
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            if actor is None:
                raise RuntimeError(
                    f"Failed to spawn {actor_config.model} at {actor_config.transform}")
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=True)
            actor.set_autopilot(False)

        # Visible transforms: same x/y, lifted +501 back to ground level (z~1).
        self.visible_transforms = []
        for actor in self.other_actors:
            t = actor.get_transform()
            self.visible_transforms.append(carla.Transform(
                carla.Location(t.location.x, t.location.y, t.location.z + 501),
                t.rotation))

    def _create_behavior(self):
        # Ego/RSU containers (distributed): minimal tree, terminate on ego progress.
        if self.distributed and self.vehicle_index >= 0:
            root = py_trees.composites.Parallel(
                "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            root.add_child(DriveDistance(self.ego_vehicles[self.vehicle_index],
                                         self.terminate_drive_distance))
            return root

        emrg = self.other_actors[0]
        npc = self.other_actors[1]

        # Emergency vehicle: teleport up, then hold position (blocks lane -3).
        emrg_seq = py_trees.composites.Sequence("EmrgVehicle")
        emrg_seq.add_child(ActorTransformSetter(emrg, self.visible_transforms[0]))
        emrg_seq.add_child(Idle())

        # Fast NPC: teleport up, then follow the lane -4 plan at constant
        # speed for the whole run — it passes the merge zone and keeps going,
        # so the lane clears behind it.
        npc_seq = py_trees.composites.Sequence("FastNPC")
        npc_seq.add_child(ActorTransformSetter(npc, self.visible_transforms[1]))
        npc_seq.add_child(WaypointFollower(npc, self.npc_speed_mps, plan=self.npc_plan))
        npc_seq.add_child(Idle())

        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(emrg_seq)
        root.add_child(npc_seq)
        root.add_child(DriveDistance(self.ego_vehicles[0], self.terminate_drive_distance))
        return root

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
