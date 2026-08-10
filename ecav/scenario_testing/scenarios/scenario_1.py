#!/usr/bin/env python

"""
Overtake Scenario:

The scripts simulate a scenario where an ego vehicle has to overtake a background vehicle
that is ahead of the ego vehicle and at a lower speed. There are two fearless pedestrians
that suddenly appear in front of the ego vehicle and the ego vehicle has to avoid a collision
"""

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


class Scenario_1(BasicScenario):
    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600, vehicle_index=-1, scenario_params=None, distributed=False):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Overtake Scenario")
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        # Vehicle count follows the XML actor list so denser oncoming
        # streams (two-locale variant) need only more <other_actor> rows.
        # First actor is the stationary blocker (velocity 0); the rest are
        # oncoming traffic. The single-locale XML (4 actors) keeps its
        # original [0, 8, 6, 6] profile exactly.
        self.num_vehicle = len(config.other_actors)
        _base = [0, 8, 6, 6]
        self.vehicle_velocities = [
            (_base[i] if i < len(_base) else 8)
            for i in range(self.num_vehicle)]
        # ONCOMING_SPEED overrides every oncoming actor's constant speed (all
        # but the stationary blocker at index 0). A fast, constant oncoming is
        # tracked with good velocity SNR upstream and handed off at its true
        # speed, which the timing-necessity arms (predictive vs reactive
        # migration) need; the default profile is unchanged when unset.
        import os as _os
        _onc_spd = _os.environ.get('ONCOMING_SPEED')
        if _onc_spd is not None:
            _v = float(_onc_spd)
            self.vehicle_velocities = [
                (0 if i == 0 else _v) for i in range(self.num_vehicle)]
        self._trigger_distance = 150
        self.agents = []

        super(Scenario_1, self).__init__("Scenario_1",
                                         ego_vehicles,
                                         config,
                                         world,
                                         debug_mode,
                                         criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        # Spawn vehicles
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(
                actor_config.model, actor_config.transform)
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=False)

        # Transformation that renders the vehicle visible
        for i in range(self.num_vehicle):
            car_transform = self.other_actors[i].get_transform()
            setattr(self, f"car_0{i + 1}_visible", carla.Transform(
                carla.Location(car_transform.location.x,
                               car_transform.location.y,
                               car_transform.location.z + 501),
                car_transform.rotation))

            # Trigger location for the actors
            setattr(self, f"vehicle_0{i + 1}_trigger_location", carla.Location(
                car_transform.location.x,
                car_transform.location.y,
                car_transform.location.z + 501, ))

    def _create_behavior(self):

        sequence_vehicle = []

        # Vehicle behaviors
        for i in range(self.num_vehicle):
            sequence_vehicle.append(py_trees.composites.Sequence(f"Vehicle_0{i + 1}"))
            trigger_location = getattr(self, f"vehicle_0{i + 1}_trigger_location")
            actor = self.other_actors[i]
            transform = getattr(self, f"car_0{i + 1}_visible")
            velocity = self.vehicle_velocities[i]

            trigger_behavior = InTriggerDistanceToLocation(self.ego_vehicles[0], trigger_location,
                                                           self._trigger_distance)
            set_transform_behavior = ActorTransformSetter(actor, transform)

            import os as _os
            _accel = _os.environ.get('ONCOMING_ACCEL', '0') == '1'
            if _accel and i >= 1 and velocity > 0:
                # Aggressive oncoming: cruise slow, then floor it near the
                # conflict. A box+velocity SNAPSHOT migrated upstream (kf arm)
                # captures only the slow cruise speed, so the destination
                # predicts constant velocity and the ego thinks it has a gap;
                # the actor then accelerates and arrives early -> collision.
                # The full memo-bank latent carries the motion history, so the
                # destination predictor sees the acceleration and the ego waits.
                car_transform = actor.get_transform()
                conflict_loc = carla.Location(
                    278.0, car_transform.location.y,
                    car_transform.location.z + 501)
                cruise_v = 5.0     # slow approach; a snapshot sees this speed
                fast_v = 16.0      # floor it as the ego reaches the conflict
                # The oncoming cruises upstream (tracked and migrated by the
                # previous locale) and accelerates when the EGO nears the
                # conflict, so the fast approach overlaps the overtake window
                # regardless of the ego's exact deceleration profile. This is
                # the aggressive-driver case: a full-latent migration recovers
                # the accelerating speed and the ego waits; a snapshot
                # under-predicts it and the ego commits into a closing gap.
                phase1 = py_trees.composites.Parallel(
                    f"accel_gate_0{i + 1}",
                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                phase1.add_child(WaypointFollower(actor, cruise_v))  # cruise
                phase1.add_child(InTriggerDistanceToLocation(
                    self.ego_vehicles[0], conflict_loc, 30.0))       # ego near conflict
                fast = WaypointFollower(actor, fast_v)               # floor it
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(phase1)
                sequence_vehicle[i].add_child(fast)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(fast)
            else:
                drive_behavior = WaypointFollower(actor, velocity)
                sequence_vehicle[i].add_child(set_transform_behavior)
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(drive_behavior)
                sequence_vehicle[i].add_child(Idle())
                self.agents.append(drive_behavior)

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 100)

        # Build composite behavior tree
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        for i in range(self.num_vehicle):
            root.add_child(sequence_vehicle[i])
        root.add_child(termination)
        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
