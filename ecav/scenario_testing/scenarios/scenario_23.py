"""
Scenario 23: Stale Prediction Under Deadline Pressure

Ego vehicle heads north.  A lead vehicle (Lincoln) drives ahead at
moderate speed and then brakes hard.  11 background CAVs create high
N, pushing the static pipeline past the 100 ms deadline.

When the pipeline misses deadline, the edge prediction goes stale.
The ego's planner receives an outdated trajectory showing the lead
vehicle still moving forward, and fails to brake in time.

The adaptive controller should cap fusion cost (select only the
relevant CAVs, drop the background ones), leaving budget for timely
divergence-gated prediction of the braking lead vehicle.  The
divergence gate should fire when the cached prediction (still moving
forward) diverges from the observed state (decelerating), triggering
a fresh MTR pass that correctly predicts the stop.

Expected outcome:
  - Static (N=12): deadline miss, stale prediction, rear-end collision
  - Joint controller: caps fusion, timely re-prediction, ego brakes
  - RSU-only: meets deadline, timely prediction, ego brakes (but
    loses context from other CAVs)
"""

import py_trees
import carla
import random
import math

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter, WaypointFollower, Idle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (
    DriveDistance, InTriggerDistanceToLocation)
from srunner.scenarios.basic_scenario import BasicScenario


class DelayedHardBrake(py_trees.behaviour.Behaviour):
    """Drive at target speed, then brake hard after delay seconds."""

    def __init__(self, actor, target_speed_mps=10.0, delay_s=4.0,
                 brake_duration_s=3.0, name="DelayedHardBrake"):
        super().__init__(name)
        self._actor = actor
        self._speed = target_speed_mps
        self._delay = delay_s
        self._brake_dur = brake_duration_s
        self._t0 = None

    def initialise(self):
        self._t0 = self._actor.get_world().get_snapshot().timestamp.elapsed_seconds

    def update(self):
        t = self._actor.get_world().get_snapshot().timestamp.elapsed_seconds
        elapsed = t - self._t0

        if elapsed < self._delay:
            # Drive forward at target speed
            self._actor.apply_control(carla.VehicleControl(
                throttle=0.7, steer=0.0, brake=0.0))
            return py_trees.common.Status.RUNNING
        elif elapsed < self._delay + self._brake_dur:
            # Hard brake
            self._actor.apply_control(carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=1.0))
            return py_trees.common.Status.RUNNING
        else:
            # Stay stopped
            self._actor.apply_control(carla.VehicleControl(
                throttle=0.0, steer=0.0, brake=0.5))
            return py_trees.common.Status.SUCCESS


class Scenario_23(BasicScenario):
    """Stale prediction under deadline pressure with 12 agents."""

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

        # 1 lead braker + 11 background CAVs
        self.num_vehicle = 12
        self._trigger_distance = 100

        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 50))

        super().__init__("Scenario_23", ego_vehicles, config, world,
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
                # Lead vehicle: drive then brake hard
                seq.add_child(DelayedHardBrake(
                    actor, target_speed_mps=10.0,
                    delay_s=3.0, brake_duration_s=4.0))
            else:
                # Background CAVs: drive slowly in their lanes
                waypoints = [
                    carla.Location(
                        x=vis.location.x,
                        y=vis.location.y + 60,
                        z=0.5),
                ]
                seq.add_child(WaypointFollower(actor, 6.0, plan=waypoints))

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
