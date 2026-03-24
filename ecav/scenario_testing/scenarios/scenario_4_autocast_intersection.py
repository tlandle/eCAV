#!/usr/bin/env python
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Scenario 4: AutoCast-style occluded intersection.

Replicates the geometry from AutoCast10 (Qian et al., arXiv:2112.14947)
for direct comparison across cooperative perception architectures.

Ego drives north. A red-light violator approaches from the east on the
cross-street. Dense parked trucks in the left lane create full occlusion.
The violator is invisible to the ego until it clears the truck wall.

Actor layout:
  other_actors[0]    = collider (Audi TT, from east, yaw=180)
  other_actors[1:8]  = occlusion wall (7 carlacola trucks, left lane)
  other_actors[8:14] = parked cars at intersection

The collider uses SyncArrival to time its approach so it reaches the
intersection simultaneously with the ego, creating a realistic collision
scenario. This matches AutoCast's evaluation methodology.
"""

import py_trees
import carla
import numpy as np

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    WaypointFollower,
    KeepVelocity,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance,
    InTriggerDistanceToLocation,
)
from srunner.scenarios.basic_scenario import BasicScenario


class Scenario_4(BasicScenario):
    """
    AutoCast-style occluded intersection scenario.

    Configurable collider speed via scenario params:
      ego_vehicle_max_speed=43  (km/h)
      collider_speed=30         (km/h, default matches AutoCast)
    """

    timeout = 600

    # Actor indices in other_actors list
    _COLLIDER_IDX = 0
    _TRUCK_START_IDX = 1
    _TRUCK_END_IDX = 8    # exclusive (7 trucks)
    _PARKED_START_IDX = 8

    def __init__(self, world, ego_vehicles, config,
                 randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, vehicle_index=-1, scenario_params=None,
                 distributed=False):
        print("Running Scenario 4: AutoCast-style occluded intersection")
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # Parse scenario params
        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 43))
        self.collider_speed_kmph = float(kv.get("collider_speed", 30))
        self.collider_speed_mps = self.collider_speed_kmph / 3.6

        # Total actors: 1 collider + 7 trucks + 6 parked = 14
        self.num_actors = 14
        self._trigger_distance = 80

        print(f"Ego max speed: {self.ego_max_speed_kmh} km/h")
        print(f"Collider speed: {self.collider_speed_kmph} km/h "
              f"({self.collider_speed_mps:.1f} m/s)")

        super().__init__(
            "Scenario_4", ego_vehicles, config, world,
            debug_mode, criteria_enable=criteria_enable,
            vehicle_index=vehicle_index,
            scenario_params=scenario_params)

    def _initialize_actors(self, config):
        if self.distributed and self.vehicle_index >= 0:
            return

        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=True)
            actor.set_autopilot(False)

        # Compute visible transforms (z + 501 to teleport from underground)
        self._visible_transforms = []
        for i in range(len(self.other_actors)):
            t = self.other_actors[i].get_transform()
            self._visible_transforms.append(carla.Transform(
                carla.Location(t.location.x, t.location.y,
                               t.location.z + 501),
                t.rotation))

    def _create_behavior(self):
        if self.distributed and self.vehicle_index >= 0:
            termination = DriveDistance(
                self.ego_vehicles[self.vehicle_index], 200)
            root = py_trees.composites.Parallel(
                "Parallel Behavior",
                policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            root.add_child(termination)
            return root

        root = py_trees.composites.Parallel(
            "Parallel Behavior",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # --- Collider behavior (index 0) ---
        collider = self.other_actors[self._COLLIDER_IDX]
        collider_seq = py_trees.composites.Sequence("Collider")

        # Teleport collider to visible position
        collider_seq.add_child(ActorTransformSetter(
            collider, self._visible_transforms[self._COLLIDER_IDX]))

        # Wait until ego is close enough to intersection
        trigger_loc = carla.Location(
            self._visible_transforms[self._COLLIDER_IDX].location.x,
            self._visible_transforms[self._COLLIDER_IDX].location.y,
            self._visible_transforms[self._COLLIDER_IDX].location.z)
        collider_seq.add_child(InTriggerDistanceToLocation(
            self.ego_vehicles[0], trigger_loc, self._trigger_distance))

        # Drive collider through intersection at fixed speed
        # Waypoints: from east, through intersection, continuing west
        collider_waypoints = [
            carla.Location(x=-88.0, y=131.5, z=0.5),   # intersection center
            carla.Location(x=-110.0, y=131.5, z=0.5),   # past intersection
            carla.Location(x=-130.0, y=131.5, z=0.5),   # well past
        ]
        collider_seq.add_child(WaypointFollower(
            collider, self.collider_speed_mps, plan=collider_waypoints))
        collider_seq.add_child(Idle())
        root.add_child(collider_seq)

        # --- Occlusion wall: teleport trucks into position ---
        for i in range(self._TRUCK_START_IDX, self._TRUCK_END_IDX):
            if i >= len(self.other_actors):
                break
            truck_seq = py_trees.composites.Sequence(f"Truck_{i}")
            truck_seq.add_child(ActorTransformSetter(
                self.other_actors[i], self._visible_transforms[i]))
            truck_seq.add_child(Idle())
            root.add_child(truck_seq)

        # --- Parked cars: teleport into position ---
        for i in range(self._PARKED_START_IDX, len(self.other_actors)):
            park_seq = py_trees.composites.Sequence(f"Parked_{i}")
            park_seq.add_child(ActorTransformSetter(
                self.other_actors[i], self._visible_transforms[i]))
            park_seq.add_child(Idle())
            root.add_child(park_seq)

        # --- End condition ---
        root.add_child(DriveDistance(self.ego_vehicles[0], 200))

        return root

    def _create_test_criteria(self):
        criteria = []
        criteria.append(CollisionTest(self.ego_vehicles[0]))
        return criteria

    def __del__(self):
        self.remove_all_actors()
