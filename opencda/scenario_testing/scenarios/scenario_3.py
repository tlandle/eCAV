#!/usr/bin/env python

"""
Overtake Scenario:

The scripts simulate a scenario where an ego vehicle has to overtake a background vehicle
that is ahead of the ego vehicle and at a lower speed. There are two fearless pedestrians
that suddenly appear in front of the ego vehicle and the ego vehicle has to avoid a collision
"""

import py_trees
import carla
import math
import random

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      WaypointFollower,
                                                                      Idle,
                                                                      SyncArrival)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance, InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario


class SteeringJitter(py_trees.behaviour.Behaviour):
    """
    Inject random ±amplitude steering offsets every `period` seconds.
    Works while another behaviour (e.g. WaypointFollower) is sending throttle.
    """
    def __init__(self,
                 actor: carla.Vehicle,
                 amplitude_deg: float = 5.0,
                 period: float = 0.3,
                 dur: float = 4.0,
                 name="SteeringJitter"):
        super().__init__(name)
        self._actor = actor
        self._amp   = math.radians(amplitude_deg)   # convert to rad
        self._period = period
        self._dur   = dur
        self._start_t = 0.0
        self._next_t  = 0.0

    def initialise(self):
        world = self._actor.get_world()
        self._start_t = world.get_snapshot().timestamp.elapsed_seconds
        self._next_t  = self._start_t

        #self._actor.set_simulate_physics(True)

    def update(self):
        sim_t = self._actor.get_world().get_snapshot().timestamp.elapsed_seconds
        if sim_t - self._start_t >= self._dur:
            return py_trees.common.Status.SUCCESS

        # time to send a new random steering offset?
        if sim_t >= self._next_t:
            offset = random.uniform(-self._amp, self._amp)
            # convert rad offset to CARLA steering in [-1,1] (approx linear)
            steering_cmd = max(-1.0, min(1.0, offset / (math.pi/4)))
            current = self._actor.get_control()
            self._actor.apply_control(
                carla.VehicleControl(
                    throttle=current.throttle,
                    brake=current.brake,
                    steer=steering_cmd,
                    hand_brake=False,
                    reverse=current.reverse)
            )
            self._next_t += self._period

        # keep running
        return py_trees.common.Status.RUNNING

class RandomHardBrake(py_trees.behaviour.Behaviour):
    """
    1) Wait `start_delay` sec
    2) Apply FULL THROTTLE until the moment we decide to brake
    3) Brake hard for a random duration in [min_dur, max_dur]
    4) Resume FULL THROTTLE and finish

    Tick every frame: when active it always sets the final control for the
    current tick, so it wins over WaypointFollower.
    """

    def __init__(self,
                 actor: carla.Vehicle,
                 start_delay: float = 2.0,
                 p_brake: float = 1.0,
                 min_dur: float = 1.0,
                 max_dur: float = 3.0,
                 full_throttle: float = 0.8,
                 name: str = "RandomHardBrake"):
        super().__init__(name)
        self._actor   = actor
        self._world   = actor.get_world()

        self._start_delay = start_delay
        self._p_brake     = p_brake
        self._min_dur     = min_dur
        self._max_dur     = max_dur
        self._full_thr    = full_throttle

        self._t_start       = 0.0   # sim-time when node first ticks
        self._brake_start   = None  # sim-time brake starts
        self._brake_end     = None  # sim-time brake ends
        self._state         = "delay"   # delay | throttle | braking | done

    def _sim_time(self):
        return self._world.get_snapshot().timestamp.elapsed_seconds

    # --------------- Py-Trees interface ----------------
    def initialise(self):
        self._t_start = self._sim_time()
        self._state   = "delay"
        self._brake_start = self._brake_end = None

    def update(self):
        t = self._sim_time()

        # phase 1 ───── delay before anything happens
        if self._state == "delay":
            if t - self._t_start >= self._start_delay:
                # decide whether we will brake this run
                if random.random() < self._p_brake:
                    dur = random.uniform(self._min_dur, self._max_dur)
                    self._brake_start = t + random.uniform(0.1, 0.5)  # tiny random offset
                    self._brake_end   = self._brake_start + dur
                    self._state = "throttle"
                else:
                    self._state = "done"
            else:
                return py_trees.common.Status.RUNNING

        # phase 2 ───── full throttle until brake_start
        if self._state == "throttle":
            if t >= self._brake_start:
                self._state = "braking"
            else:
                # overwrite follower with full throttle, zero brake
                cur = self._actor.get_control()
                self._actor.apply_control(carla.VehicleControl(
                    throttle=self._full_thr,
                    steer=cur.steer,
                    brake=0.0))
                return py_trees.common.Status.RUNNING

        # phase 3 ───── hard brake window
        if self._state == "braking":
            if t >= self._brake_end:
                self._state = "after"
            else:
                # full brake each tick
                cur = self._actor.get_control()
                self._actor.apply_control(carla.VehicleControl(
                    throttle=0.0,
                    steer=cur.steer,
                    brake=0.5))
                return py_trees.common.Status.RUNNING

        # phase 4 ───── resume full throttle and finish
        if self._state == "after":
            cur = self._actor.get_control()
            self._actor.apply_control(carla.VehicleControl(
                throttle=self._full_thr,
                steer=cur.steer,
                brake=0.0))
            self._state = "done"
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS


class Scenario_3(BasicScenario):
    """
    The class spawns two background vehicles and two pedestrians in front of the ego vehicle.
    The ego vehicle is driving behind and overtaking the fast vehicle ahead

    self.other_actors[0] = fast car
    self.other_actors[1] = slow car
    """

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=600):
        """
        Setup all relevant parameters and create scenario
        """
        print("Running Unprotected Red-light Violation Scenario")
        self.timeout = timeout
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(
            config.trigger_points[0].location)

        self.num_vehicle = 6
        self.vehicle_01_velocity = 25  # Violated vehicle
        self.vehicle_02_velocity = 0  # Large vehicles from 02 to 06
        self.vehicle_03_velocity = 0
        self.vehicle_04_velocity = 0
        self.vehicle_05_velocity = 0
        self.vehicle_06_velocity = 0
        self._trigger_distance = 75
        self.agents = []

        super(Scenario_3, self).__init__("Scenario_3",
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
            actor.set_simulate_physics(enabled=True)
            actor.set_autopilot(False)           #  follower will drive via apply_control

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

        # Vehicle behavior
        for i in range(self.num_vehicle):
            sequence_vehicle.append(py_trees.composites.Sequence(f"Vehicle_0{i + 1}"))
            trigger_location = getattr(self, f"vehicle_0{i + 1}_trigger_location")
            actor = self.other_actors[i]
            transform = getattr(self, f"car_0{i + 1}_visible")
            velocity = getattr(self, f"vehicle_0{i + 1}_velocity")

            sync_arrival = SyncArrival(actor, self.ego_vehicles[0] , carla.Location(x=-83.55, y=127.9, z=0.5))

            trigger_behavior = InTriggerDistanceToLocation(self.ego_vehicles[0], trigger_location,
                                                           self._trigger_distance)
            set_transform_behavior = ActorTransformSetter(actor, transform)
            if i == 0:
                waypoint = [carla.Location(x=-108.6, y=129.5, z=0.5), carla.Location(x=-120.6, y=129.5, z=0.5), carla.Location(x=-140.6, y=115.2, z=0.5), carla.Location(x=-142.0, y=87.6, z=0.5)]
                drive_behavior = WaypointFollower(actor, velocity, plan=waypoint)
            else:
                drive_behavior = WaypointFollower(actor, velocity)

            if (i == 0):
                sync_arrival_parallel = py_trees.composites.Parallel(
                    f"SyncArrival_{i}", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
                waypoint_follower_parallel = py_trees.composites.Parallel(
                    f"WaypointFollower_{i}", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

            sequence_vehicle[i].add_child(set_transform_behavior)
            #if i == 0:
            #    sequence_vehicle[i].add_child(trigger_behavior)
            #    sequence_vehicle[i].add_child(sync_arrival_parallel)
            #    sequence_vehicle[i].add_child(waypoint_follower_parallel)
            #    waypoint_follower_parallel.add_child(drive_behavior)
            #    sync_arrival_parallel.add_child(sync_arrival)
            #if i == 0:
                #sequence_vehicle[i].add_child(trigger_behavior)
                #sequence_vehicle[i].add_child(sync_arrival)
                #sequence_vehicle[i].add_child(drive_behavior)
            #else:

        # Run brake and follower in *parallel* so the follower pauses
        # automatically while RandomBrake is RUNNING
            if i == 0:
                parallel = py_trees.composites.Parallel(
                "Brake+Follow", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)

                
                brake_behavior = RandomHardBrake(actor,
                                 start_delay=2.0,
                                 p_brake=1.0,      # 1.0 = always brake
                                 min_dur=1.0,
                                 max_dur=5.0,
                                 full_throttle=1.0)

                jitter_behavior = SteeringJitter(actor,
                                     amplitude_deg=4.0,
                                     period=0.25,
                                     dur=3.0)

                parallel.add_child(drive_behavior)
                parallel.add_child(brake_behavior)
                parallel.add_child(jitter_behavior)

                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(parallel)
            else:
                sequence_vehicle[i].add_child(trigger_behavior)
                sequence_vehicle[i].add_child(drive_behavior)
            sequence_vehicle[i].add_child(Idle())
            self.agents.append(drive_behavior)

        # End condition
        termination = DriveDistance(self.ego_vehicles[0], 200)

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
