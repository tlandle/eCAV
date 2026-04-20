"""
Scenario 22: Dense Intersection with Conflicting Intents

16 vehicles converge on a Town03 intersection from all four approach
directions (4 per approach).  This scenario tests deadline compliance
under high N rather than occlusion complementarity.

With N=17 (16 vehicles + 1 RSU), the static pipeline exceeds the
100 ms deadline.  The adaptive controller must select a manageable
subset while preserving the ego's ability to detect vehicles on
conflicting trajectories (especially east and west approaches that
cross the ego's northbound path).

Expected outcome:
  - Static (all 16): accurate detection but 100% deadline miss
  - Joint controller: selects ~2-4 CAVs from cross-traffic approaches,
    meets deadline, preserves safety on conflict trajectories
  - RSU-only: meets deadline, catches most vehicles from elevation,
    but may miss fast approaches at zone boundary
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


class Scenario_22(BasicScenario):
    """Dense 16-vehicle intersection convergence."""

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

        self.num_vehicle = 16
        self._trigger_distance = 120
        self.approach_speed_mps = 10.0  # ~36 km/h

        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 50))
        self.approach_speed_mps = float(
            kv.get("approach_speed", self.approach_speed_mps))

        super().__init__("Scenario_22", ego_vehicles, config, world,
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

        # Intersection center for waypoint targets
        cx, cy = -84.8, 127.7
        # Each approach drives through the intersection center
        approach_targets = {
            'north': [carla.Location(x=cx, y=cy, z=0.5),
                      carla.Location(x=cx, y=80.0, z=0.5)],
            'east':  [carla.Location(x=cx, y=cy, z=0.5),
                      carla.Location(x=-140.0, y=cy, z=0.5)],
            'south': [carla.Location(x=cx, y=cy, z=0.5),
                      carla.Location(x=cx, y=170.0, z=0.5)],
            'west':  [carla.Location(x=cx, y=cy, z=0.5),
                      carla.Location(x=-20.0, y=cy, z=0.5)],
        }
        approach_order = ['north'] * 4 + ['east'] * 4 + ['south'] * 4 + ['west'] * 4

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

            approach = approach_order[i]
            waypoints = approach_targets[approach]
            seq.add_child(WaypointFollower(
                actor, self.approach_speed_mps, plan=waypoints))
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
