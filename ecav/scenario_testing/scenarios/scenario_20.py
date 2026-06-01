"""
Scenario 20: LTAP with Dense Occlusion and CAV Selection

Ego vehicle heads north through a signalized intersection in Town03.
Four firetrucks line the right side of the road, blocking the ego's
view of cross-traffic approaching from the east.  Two obstacle
vehicles (Lincoln and Tesla) approach from the east at speed.

Eight CAVs are present: four trailing the ego on the same lane (low
complementarity value, they see what the ego sees) and four on the
eastern approach road (high complementarity value, they see the
cross-traffic that the ego and same-lane CAVs cannot).

The RSU sits on the traffic light above the intersection and can see
both approach roads.  The scenario tests whether the contributor
selector picks the eastern CAVs (which see the occluded threat) over
the same-lane CAVs (which duplicate the ego/RSU view).

Expected outcome:
  - RSU-only: detects cross-traffic, ego brakes in time
  - Causal filter (k_cav=2): picks eastern CAVs, detects cross-traffic
  - Random (k_cav=2): 50% chance of picking same-lane CAVs, may miss
  - Feature-norm: picks same-lane CAVs (stronger signal in zone center),
    misses cross-traffic until late
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


class Scenario_20(BasicScenario):
    """LTAP with dense occlusion and 8 CAVs (4 same-lane, 4 eastern)."""

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

        # XML has 2 cross-traffic + N stationary occluders. Read N from
        # the config so both scenario_20.xml (occluded) and
        # scenario_20_deoc.xml (no occluders) work without code changes.
        # Real cooperating CAVs are defined in the YAML vehicles: list.
        self.num_cross_traffic = 2
        self.num_vehicle = len(config.other_actors)
        self._trigger_distance = 90

        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 50))
        self.cross_traffic_speed_mps = float(kv.get("cross_traffic_speed", 12.0))
        self.cav_speed_mps = float(kv.get("cav_speed", 8.0))

        super().__init__("Scenario_20", ego_vehicles, config, world,
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

        # Compute visible transforms (raise from z=-500 to road level)
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

            # Make visible when ego approaches
            trigger_loc = carla.Location(vis.location.x, vis.location.y,
                                         vis.location.z)
            trigger = InTriggerDistanceToLocation(
                self.ego_vehicles[0], trigger_loc, self._trigger_distance)
            seq.add_child(ActorTransformSetter(actor, vis))
            seq.add_child(trigger)

            if i < self.num_cross_traffic:
                # Cross-traffic: drive westward through intersection
                waypoints = [
                    carla.Location(x=-84.8, y=127.9, z=0.5),
                    carla.Location(x=-120.0, y=129.5, z=0.5),
                    carla.Location(x=-150.0, y=115.0, z=0.5),
                ]
                seq.add_child(WaypointFollower(
                    actor, self.cross_traffic_speed_mps, plan=waypoints))
            else:
                # Stationary occluder cars (parked along the eastern curb)
                seq.add_child(Idle())

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
