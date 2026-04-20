"""
Scenario 21: Fast Approach from RSU Blind Side

Ego vehicle heads east on the main road toward an intersection.
A fast vehicle (Dodge Charger) approaches from the south on a side
street, partially occluded from the RSU by parked trucks near the
intersection corner.

Six CAVs: two on the side street heading north (they can see the
fast approaching vehicle early) and four behind the ego on the main
road (they see only what the ego already sees).

The scenario tests whether the selector picks the side-street CAVs
that provide early warning of the fast approach, versus the
trailing CAVs that add no new information.

Expected outcome with correct selection: ego brakes before entering
the conflict zone.  With wrong selection (or deadline miss): ego
enters the intersection and collides with the fast approach.
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter, WaypointFollower, Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance, InTriggerDistanceToLocation)
from srunner.scenarios.basic_scenario import BasicScenario


class Scenario_21(BasicScenario):
    """Fast approach from RSU blind side with 6 CAVs."""

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, timeout=600,
                 vehicle_index=-1, scenario_params=None, distributed=False):
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # 1 fast approach + 2 occluders + 2 side-street CAVs + 4 trailing CAVs
        self.num_vehicle = 9
        self._trigger_distance = 100
        self.fast_approach_speed_mps = 16.0  # ~58 km/h

        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 50))
        self.fast_approach_speed_mps = float(
            kv.get("fast_approach_speed", self.fast_approach_speed_mps))

        super().__init__("Scenario_21", ego_vehicles, config, world,
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

        for i in range(self.num_vehicle):
            t = self.other_actors[i].get_transform()
            setattr(self, f"visible_{i}", carla.Transform(
                carla.Location(t.location.x, t.location.y,
                               t.location.z + 501),
                t.rotation))

    def _create_behavior(self):
        if self.distributed and self.vehicle_index >= 0:
            termination = DriveDistance(
                self.ego_vehicles[self.vehicle_index], 200)
            root = py_trees.composites.Parallel(
                "Parallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            root.add_child(termination)
            return root

        sequences = []
        for i in range(self.num_vehicle):
            seq = py_trees.composites.Sequence(f"Vehicle_{i}")
            actor = self.other_actors[i]
            vis = getattr(self, f"visible_{i}")

            trigger_loc = carla.Location(
                vis.location.x, vis.location.y, vis.location.z)
            trigger = InTriggerDistanceToLocation(
                self.ego_vehicles[0], trigger_loc, self._trigger_distance)
            seq.add_child(ActorTransformSetter(actor, vis))
            seq.add_child(trigger)

            if i == 0:
                # Fast approach: drive north through intersection
                waypoints = [
                    carla.Location(x=-84.8, y=127.7, z=0.5),
                    carla.Location(x=-84.8, y=160.0, z=0.5),
                ]
                seq.add_child(WaypointFollower(
                    actor, self.fast_approach_speed_mps, plan=waypoints))
            elif i < 3:
                # Occluders: stationary parked trucks
                seq.add_child(Idle())
            elif i < 5:
                # Side-street CAVs: drive north slowly
                waypoints = [
                    carla.Location(x=-84.8, y=120.0, z=0.5),
                    carla.Location(x=-84.8, y=150.0, z=0.5),
                ]
                seq.add_child(WaypointFollower(actor, 6.0, plan=waypoints))
            else:
                # Trailing CAVs: follow ego direction
                waypoints = [
                    carla.Location(x=-84.8, y=127.7, z=0.5),
                    carla.Location(x=-50.0, y=127.7, z=0.5),
                ]
                seq.add_child(WaypointFollower(actor, 8.0, plan=waypoints))

            seq.add_child(Idle())
            sequences.append(seq)

        termination = DriveDistance(self.ego_vehicles[0], 200)
        root = py_trees.composites.Parallel(
            "Parallel", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for s in sequences:
            root.add_child(s)
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
