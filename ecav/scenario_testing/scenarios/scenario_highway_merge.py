#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""Highway merge (cut-in) geometry for the SenSys safety-envelope harness, Town06.

The focal CAV (ego) cruises east in lane -4 of Town06's eastbound carriageway at
its commanded speed (97 km/h in the paper). A slower vehicle (the intruder, a
Tesla Model 3 so the per-tick logger's conflict-actor rule finds it) cruises in
the adjacent lane -5 at ego speed minus the closing speed. When the bumper gap
between them has closed to the design gap, the intruder changes lanes into the
ego's lane over CUTIN_DURATION_S seconds and holds its speed. An optional truck
(the occluder) cruises in lane -5 behind the intruder, so the CAV's own sensors
see the intruder only around the cut-in while the roadside unit sees it earlier.

Design (highway_merge_design.csv of the evaluation export), all in SI:
    initial_gap        = clearance + ttc * closing_speed            (bumper gap at the cut-in)
    rel_stop_distance  = closing_speed^2 / (2 * ego_braking)
    kinematic_budget   = ttc - non_age_response - closing_speed / (2 * ego_braking)
with ttc 1.9 s, clearance 3.0 m, non-age response 0.28 s, ego braking 6 m/s^2.

Environment knobs (test_runner --scenario-env KEY=VALUE):
    CLOSING_SPEED_MPS   closing speed at the cut-in, m/s (default 10)
    CUTIN_TTC_S         time to the clearance at the cut-in (1.9)
    CLEARANCE_M         clearance distance (3.0)
    EGO_BRAKING_MPS2    design deceleration (6.0), reported only
    NON_AGE_RESPONSE_S  design response time (0.28), reported only
    CUTIN_DURATION_S    lane-change duration (2.0): lateral path length = speed * duration
    PRE_TRIGGER_S       cruise before the gap closes to the design gap (4.0): the start
                        gap is initial_gap + closing_speed * PRE_TRIGGER_S
    EGO_INITIAL_SPEED_MPS  ego speed at spawn (default: commanded speed), so the
                        encounter starts with both vehicles cruising
    EGO_DRIVE_DISTANCE  scenario ends when the ego has driven this far (250: the cut-in happens
                        about 110 to 140 m in and an emergency stop from 27 m/s ends within 60 m)
    OCCLUDER            1 places the truck behind the intruder in lane -5 (default 0)
    OCCLUDER_GAP_M      bumper gap truck to intruder (12.0)
"""
import math
import os

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (
    ActorTransformSetter,
    WaypointFollower,
    Idle,
)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance
from srunner.scenarios.basic_scenario import BasicScenario

try:
    from agents.navigation.local_planner import RoadOption
except Exception:  # pragma: no cover - scenario_runner's own copy
    from srunner.scenariomanager.scenarioatomics.atomic_behaviors import RoadOption  # type: ignore

TAG = "[HIGHWAY_MERGE]"

# Town06 eastbound lane centres (offline map probe, 2026-11-09): lane -3 at
# y = 135.93 + 0.0041 x, each further lane 3.5 m to +y, heading yaw 0.2 deg.
LANE_Y0 = {-3: 135.93, -4: 139.43, -5: 142.93, -6: 146.43, -7: 149.93}
LANE_SLOPE = 0.0041
EGO_LANE = -4
INTRUDER_LANE = -5
ROAD_YAW = 0.2
PLAN_X_END = 900.0


def _env_float(name, default):
    v = os.environ.get(name)
    return float(v) if v not in (None, "") else float(default)


def lane_y(lane_id, x):
    return LANE_Y0[lane_id] + LANE_SLOPE * x


def design(closing, ttc, clearance, braking, response):
    gap = clearance + ttc * closing
    rel_stop = closing * closing / (2.0 * braking)
    budget_s = ttc - response - closing / (2.0 * braking)
    return gap, rel_stop, budget_s


class SetInitialVelocity(py_trees.behaviour.Behaviour):
    """Give an actor a cruising velocity along the road on its first tick above
    ground, so the encounter starts with the vehicles already at speed."""

    def __init__(self, actor, speed_mps, yaw_deg=ROAD_YAW, name="SetInitialVelocity"):
        super().__init__(name)
        self._actor = actor
        self._v = float(speed_mps)
        self._yaw = math.radians(yaw_deg)

    def update(self):
        self._actor.set_target_velocity(carla.Vector3D(
            self._v * math.cos(self._yaw), self._v * math.sin(self._yaw), 0.0))
        return py_trees.common.Status.SUCCESS


class CutIn(py_trees.behaviour.Behaviour):
    """RUNNING while the bumper gap from the ego's front to the intruder's rear
    exceeds the design gap; then replaces the intruder's lane plan with a lane
    change into the ego's lane and returns SUCCESS. The follower keeps the
    intruder's speed; only its path changes."""

    def __init__(self, ego, npc, follower, gap_m, duration_s, speed_mps, scenario,
                 name="CutIn"):
        super().__init__(name)
        self._ego = ego
        self._npc = npc
        self._follower = follower
        self._gap = float(gap_m)
        self._L = float(duration_s) * float(speed_mps)
        self._sc = scenario
        self._done = False

    def update(self):
        if self._done:
            return py_trees.common.Status.SUCCESS
        e, n = self._ego.get_location(), self._npc.get_location()
        gap = (n.x - e.x) - (self._ego.bounding_box.extent.x + self._npc.bounding_box.extent.x)
        if gap > self._gap:
            return py_trees.common.Status.RUNNING
        planner = getattr(self._follower, "_local_planner_dict", {}).get(self._npc)
        if planner is None:
            return py_trees.common.Status.RUNNING
        plan = self._sc.lane_change_plan(n.x, self._L)
        try:
            planner.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
        except TypeError:
            planner.set_global_plan(plan)
        self._done = True
        ev, nv = self._ego.get_velocity(), self._npc.get_velocity()
        print(f"{TAG} cut-in at tick {GameTimeTick.now()} x={n.x:.1f}: bumper gap {gap:.1f} m "
              f"(design {self._gap:.1f}), ego {math.hypot(ev.x, ev.y):.1f} m/s, "
              f"intruder {math.hypot(nv.x, nv.y):.1f} m/s, lateral path {self._L:.0f} m", flush=True)
        return py_trees.common.Status.SUCCESS


class GameTimeTick:
    """Tick counter for the log lines (the scenario tree has no tick index)."""
    _frame0 = None

    @classmethod
    def now(cls):
        try:
            f = CarlaDataProvider.get_world().get_snapshot().frame
        except Exception:
            return -1
        if cls._frame0 is None:
            cls._frame0 = f
        return int(f - cls._frame0)


class Scenario_HighwayMerge(BasicScenario):
    """Town06 highway cut-in at a configured closing speed."""

    timeout = 1200

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False,
                 criteria_enable=True, timeout=600, vehicle_index=-1,
                 scenario_params=None, distributed=False):
        self.timeout = timeout
        self.vehicle_index = vehicle_index
        self.distributed = distributed
        self._map = CarlaDataProvider.get_map()
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        kv = dict(p.split("=", 1) for p in (scenario_params or []))
        self.ego_max_speed_kmh = float(kv.get("ego_vehicle_max_speed", 97))
        self.ego_speed = self.ego_max_speed_kmh / 3.6
        self.closing = _env_float("CLOSING_SPEED_MPS", 10.0)
        self.ttc = _env_float("CUTIN_TTC_S", 1.9)
        self.clearance = _env_float("CLEARANCE_M", 3.0)
        self.braking = _env_float("EGO_BRAKING_MPS2", 6.0)
        self.response = _env_float("NON_AGE_RESPONSE_S", 0.28)
        self.cutin_duration = _env_float("CUTIN_DURATION_S", 2.0)
        self.pre_trigger = _env_float("PRE_TRIGGER_S", 4.0)
        self.ego_initial_speed = _env_float("EGO_INITIAL_SPEED_MPS", self.ego_speed)
        self.drive_distance = _env_float("EGO_DRIVE_DISTANCE", 250.0)
        self.occluder = os.environ.get("OCCLUDER", "0") == "1"
        self.occluder_gap = _env_float("OCCLUDER_GAP_M", 12.0)
        self.intruder_speed = max(self.ego_speed - self.closing, 1.0)
        self.gap, self.rel_stop, self.budget_s = design(
            self.closing, self.ttc, self.clearance, self.braking, self.response)
        self.start_gap = self.gap + self.closing * self.pre_trigger
        print(f"{TAG} design: ego {self.ego_speed:.2f} m/s, closing {self.closing:.1f} m/s, "
              f"intruder {self.intruder_speed:.2f} m/s, cut-in gap {self.gap:.1f} m "
              f"(start gap {self.start_gap:.1f}), rel. stop {self.rel_stop:.2f} m, "
              f"kinematic budget {self.budget_s*1000:.0f} ms, lane change {self.cutin_duration:.1f} s, "
              f"occluder {'on' if self.occluder else 'off'}", flush=True)
        self.agents = []
        GameTimeTick.now()  # tick 0 of the log lines is the scenario construction
        super(Scenario_HighwayMerge, self).__init__(
            "Scenario_HighwayMerge", ego_vehicles, config, world, debug_mode,
            criteria_enable=criteria_enable, vehicle_index=vehicle_index,
            scenario_params=scenario_params)
        if not (self.distributed and self.vehicle_index >= 0):
            try:
                ego = ego_vehicles[0]
                yaw = math.radians(ego.get_transform().rotation.yaw)
                ego.set_target_velocity(carla.Vector3D(
                    self.ego_initial_speed * math.cos(yaw), self.ego_initial_speed * math.sin(yaw), 0.0))
                print(f"{TAG} ego starts at {self.ego_initial_speed:.1f} m/s from "
                      f"({ego.get_location().x:.1f}, {ego.get_location().y:.1f})", flush=True)
            except Exception as exc:  # the run goes on from rest
                print(f"{TAG} ego initial velocity not applied: {exc}", flush=True)

    # ------------------------------------------------------------------
    def _lane_waypoint(self, x, lane_id):
        """Driving waypoint of lane_id nearest (x, fitted y); None past the road end."""
        y0 = lane_y(lane_id, x)
        for dy in (0.0, 0.8, -0.8, 1.6, -1.6):
            wp = self._map.get_waypoint(carla.Location(x=x, y=y0 + dy, z=0.5),
                                        project_to_road=True, lane_type=carla.LaneType.Driving)
            if wp is not None and wp.lane_id == lane_id and abs(wp.transform.location.x - x) < 5.0:
                return wp
        return None

    def _lane_plan(self, x_start, lane_id, step=25.0):
        plan, x = [], x_start
        while x <= PLAN_X_END:
            wp = self._lane_waypoint(x, lane_id)
            if wp is None:
                break
            plan.append((wp, RoadOption.LANEFOLLOW))
            x += step
        return plan

    def lane_change_plan(self, x_now, lateral_len):
        """Intruder path from its lane into the ego's lane: one more point in its
        own lane 5 m ahead, the ego-lane centre lateral_len further on, then the
        ego lane to the end of the straight."""
        plan = []
        wp = self._lane_waypoint(x_now + 5.0, INTRUDER_LANE)
        if wp is not None:
            plan.append((wp, RoadOption.LANEFOLLOW))
        plan += self._lane_plan(x_now + 5.0 + lateral_len, EGO_LANE)
        return plan

    # ------------------------------------------------------------------
    def _initialize_actors(self, config):
        if self.distributed and self.vehicle_index >= 0:
            return
        for actor_config in config.other_actors:
            actor = CarlaDataProvider.request_new_actor(actor_config.model, actor_config.transform)
            if actor is None:
                raise RuntimeError(f"Failed to spawn {actor_config.model} at {actor_config.transform}")
            self.other_actors.append(actor)
            actor.set_simulate_physics(enabled=True)
            actor.set_autopilot(False)
        ego = self.ego_vehicles[0]
        ego_loc = ego.get_location()
        intruder = self.other_actors[0]
        half = ego.bounding_box.extent.x + intruder.bounding_box.extent.x
        x_i = ego_loc.x + self.start_gap + half
        wp_i = self._lane_waypoint(x_i, INTRUDER_LANE)
        y_i = wp_i.transform.location.y if wp_i is not None else lane_y(INTRUDER_LANE, x_i)
        self.visible_transforms = [carla.Transform(carla.Location(x=x_i, y=y_i, z=0.5),
                                                   carla.Rotation(yaw=ROAD_YAW))]
        if len(self.other_actors) > 1:
            truck = self.other_actors[1]
            x_t = x_i - (intruder.bounding_box.extent.x + truck.bounding_box.extent.x) - self.occluder_gap
            wp_t = self._lane_waypoint(x_t, INTRUDER_LANE)
            y_t = wp_t.transform.location.y if wp_t is not None else lane_y(INTRUDER_LANE, x_t)
            self.visible_transforms.append(carla.Transform(carla.Location(x=x_t, y=y_t, z=0.5),
                                                           carla.Rotation(yaw=ROAD_YAW)))
        print(f"{TAG} intruder starts at x={x_i:.1f} (lane {INTRUDER_LANE}), ego at x={ego_loc.x:.1f} "
              f"(lane {EGO_LANE}); start bumper gap {self.start_gap:.1f} m", flush=True)

    def _create_behavior(self):
        if self.distributed and self.vehicle_index >= 0:
            root = py_trees.composites.Parallel(
                "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
            root.add_child(DriveDistance(self.ego_vehicles[self.vehicle_index], self.drive_distance))
            return root
        ego = self.ego_vehicles[0]
        intruder = self.other_actors[0]
        x_i = self.visible_transforms[0].location.x
        follower = WaypointFollower(intruder, self.intruder_speed,
                                    plan=self._lane_plan(x_i + 5.0, INTRUDER_LANE))
        self.agents.append(follower)
        seq = py_trees.composites.Sequence("Intruder")
        seq.add_child(ActorTransformSetter(intruder, self.visible_transforms[0]))
        seq.add_child(SetInitialVelocity(intruder, self.intruder_speed))
        both = py_trees.composites.Parallel("Intruder_drive_and_cut_in",
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        both.add_child(follower)
        both.add_child(CutIn(ego, intruder, follower, self.gap, self.cutin_duration,
                             self.intruder_speed, self))
        seq.add_child(both)
        seq.add_child(Idle())
        root = py_trees.composites.Parallel(
            "Parallel Behavior", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        root.add_child(seq)
        if self.occluder and len(self.other_actors) > 1:
            truck = self.other_actors[1]
            x_t = self.visible_transforms[1].location.x
            tfollow = WaypointFollower(truck, self.intruder_speed,
                                       plan=self._lane_plan(x_t + 5.0, INTRUDER_LANE))
            tseq = py_trees.composites.Sequence("Occluder")
            tseq.add_child(ActorTransformSetter(truck, self.visible_transforms[1]))
            tseq.add_child(SetInitialVelocity(truck, self.intruder_speed))
            tseq.add_child(tfollow)
            tseq.add_child(Idle())
            root.add_child(tseq)
        root.add_child(DriveDistance(ego, self.drive_distance))
        return root

    def _create_test_criteria(self):
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self.remove_all_actors()
