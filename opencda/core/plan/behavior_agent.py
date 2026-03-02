# -*- coding: utf-8 -*-

"""Behavior planning module
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib


import math
import random
import sys
import time
import logging
import opencda.logging_ecloud
import numpy as np
import carla
import copy

from opencda.core.common.misc import get_speed, positive, cal_distance_angle
from opencda.core.plan.collision_check import CollisionChecker
from opencda.core.plan.local_planner_behavior import LocalPlanner, RoadOption
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.core.plan.planning_metrics import PlanningMetrics
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory

from opencda.core.common.misc import distance_vehicle, draw_trajetory_points
from opencda.core.prediction.linear_predictor_manager import LinearPredictorManager

logger = logging.getLogger(__name__)

SET_DESTINATION_WAYPOINT_LIMIT = 16 # TODO: move to config

PATH_RESOLUTION = 0.1  # meters per path point (ds=0.1 in local_planner)

def will_prediction_collide_with_ego(
    prediction,
    ego_path_x,
    ego_path_y,
    ego_path_yaw,
    ego_speed_mps,
    time_step=0.05,
    lateral_threshold=2.0,
    max_steps=25
):
    """
    Checks if the predicted trajectory intersects ego path, and returns TTC.

    Time-synchronizes both paths: at each future time step, compare the
    obstacle's predicted position with the ego's expected position along
    its planned path.

    Returns:
        (bool, float): (True, time_to_collision_in_sec) if collision likely, else (False, None).
    """
    pred_traj = prediction.predicted_trajectory[:max_steps]
    if not pred_traj or len(ego_path_x) < 2:
        return False, None

    for i, pred_transform in enumerate(pred_traj):
        pred_time = i * time_step
        ego_distance_ahead = ego_speed_mps * pred_time
        ego_idx = min(int(ego_distance_ahead / PATH_RESOLUTION), len(ego_path_x) - 1)

        ego_x = ego_path_x[ego_idx]
        ego_y = ego_path_y[ego_idx]

        pred_loc = pred_transform.location
        dx = pred_loc.x - ego_x
        dy = pred_loc.y - ego_y
        lateral_dist = (dx**2 + dy**2) ** 0.5

        if lateral_dist < lateral_threshold:
            return True, pred_time

    return False, None

class BehaviorAgent(object):
    """
    A modulized version of carla BehaviorAgent.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    carla_map : carla.map
        The carla HD map for simulation world.

    config_yaml : dict
        The configuration dictionary of the localization module.

    Attributes
    ----------
    _ego_pos : carla.position
        Posiion of the ego vehicle.

    _ego_speed : float
        Speed of the ego vehicle.

    _map : carla.map
        The HD map of the current simulation world.

    max_speed : float
        The current speed limit of the ego vehicles.

    break_distance : float
        The current distance needed for ego vehicle to reach a steady stop.

    _collision_check : collisionchecker
        A collision check class to estimate the collision with front obstacle.

    ignore_traffic_light : boolean
        Boolean indicator of whether to ignore traffic light.

    overtake_allowed : boolean
        Boolean indicator of whether to allow overtake.

    _local_planner : LocalPlanner
        A carla local planner class for behavior planning.

    lane_change_allowed : boolean
        Boolean indicator of whether the lane change is allowed.

    white_list : list
        The white list contains all position of target
        platoon member for joining.

    obstacle_vehicles : list
        The list contains all obstacle vehicles nearby.

    objects : dict
        The dictionary that contains all kinds of objects nearby.

    debug_helper : PlanningMetrics
        The helper class that help with the debug functions.
    """

    def __init__(self, vehicle, carla_map, config_yaml, is_dist=False):

        self.vehicle = vehicle
        # ego pos(transform) and speed(km/h) retrieved from localization module
        self._ego_pos = None
        self._ego_speed = 0.0
        self._map = carla_map
        self._is_dist = is_dist

        # speed related, check yaml file to see the meaning
        self.max_speed = config_yaml['max_speed']
        self.tailgate_speed = config_yaml['tailgate_speed']
        self.speed_lim_dist = config_yaml['speed_lim_dist']
        self.speed_decrease = config_yaml['speed_decrease']

        # safety related
        self.safety_time = config_yaml['safety_time']
        self.emergency_param = config_yaml['emergency_param']
        self.break_distance = 0
        self.ttc = 1000
        # collision checker
        time_ahead = config_yaml['collision_time_ahead']
        self._collision_check = CollisionChecker(
            time_ahead=time_ahead)
        self.ignore_traffic_light = config_yaml['ignore_traffic_light']
        self.overtake_allowed = config_yaml['overtake_allowed']
        self.overtake_allowed_origin = config_yaml['overtake_allowed']
        self.overtake_counter = 0
        self.overtake_other_direction = False
        self.overtake_end_wpts = []
        self.ego_location_buffer = []
        # used to indicate whether a vehicle is on the planned path
        self.hazard_flag = False
        # RSS-inspired proper response state
        self._committed_brake_ttl = 0
        self._rss_threat_carla_id = -1
        self._rss_last_lateral_dist = 0.0
        self._rss_lateral_growing_ticks = 0

        # route planner related
        self._global_planner = None
        self.start_waypoint = None
        self.end_waypoint = None
        self._sampling_resolution = config_yaml['sample_resolution']

        # intersection agent related
        self.light_state = "Red"
        self.light_id_to_ignore = -1
        self.stop_sign_wait_count = 0
        self.left_turn = False

        # trajectory planner
        self._local_planner = LocalPlanner(
            self, carla_map, config_yaml['local_planner'])

        # special behavior rlated
        self.car_following_flag = False
        # lane change allowed flag
        self.lane_change_allowed = True
        # destination temp push flag
        self.destination_push_flag = 0

        # overtaking
        self.overtake_wait_time = 18    # TODO: make this a parameter
        self.overtake_wait_counter = self.overtake_wait_time
        self.do_overtake = False
        self.num_overtake_collisions = 0

        # white list of vehicle managers that the cav does not consider as
        # obstacles
        self.white_list = []
        self.obstacle_vehicles = []
        self.static_obstacles = []
        self.objects = {}

        # debug helper
        self.planning_metrics = PlanningMetrics(self.vehicle.id)
        # braking attribution: shared list between behavior_agent and planning_metrics
        self.brake_attributions = self.planning_metrics.brake_attributions
        # logger.debug message in debug mode
        self.debug = False if 'debug' not in \
                              config_yaml else config_yaml['debug']

        # Other Car Trajecories Dictionary
        self.other_car_trajectories = {}
        self.other_car_speeds = {}
        self.generated_predictions = [] # ObstaclePrediction list

        # Local+Edge prediction pipelines
        self.tracked_obstacles   = {}   # {track_id: ObstacleTrajectory}
        self._num_future_steps = 25 # number of future steps to predict
        self._linear_predictor   = LinearPredictorManager(num_future_steps=self._num_future_steps)
        self.local_predictions   = []   # produced on-board each tick
        self.edge_predictions    = []   # filled when the edge sends predictions


    def _maintain_tracks_and_predict(self, dt: float):
        """
        • Updates/creates ObstacleTrajectory objects for every currently
          detected local obstacle vehicle.
        • Removes trajectories unseen for >2 s.
        • Runs the linear predictor to populate self.local_predictions.
        """
        # 1. Update or create trajectories
        logger.debug("Obstacle Vehicles: %s", self.obstacle_vehicles)
        for obs in self.obstacle_vehicles:
            tid = getattr(obs, "carla_id", None) or obs.id      # robust id
            if tid not in self.tracked_obstacles:
                self.tracked_obstacles[tid] = ObstacleTrajectory(obs, [])
                self.tracked_obstacles[tid].time_since_update = 0.0
            self.tracked_obstacles[tid].update(obs.get_transform())

        # 2. Prune stale trajectories (>2 s since last update)
        stale_ids = []
        for tid, traj in self.tracked_obstacles.items():
            if hasattr(traj, "time_since_update"):
                traj.step(dt)          # advance internal timer
                if traj.time_since_update > 1.0:
                    stale_ids.append(tid)
        for tid in stale_ids:
            self.tracked_obstacles.pop(tid, None)

        # 3. Generate local predictions
        self.local_predictions = self._linear_predictor.generate_predicted_trajectories(
            self.tracked_obstacles
        )

    def update_information(self, ego_pos, ego_speed, objects):
        """
        Update the perception and localization information
        to the behavior agent.

        Parameters
        ----------
        ego_pos : carla.Transform
            Ego position from localization module.

        ego_speed : float
            km/h, ego speed.

        objects : dict
            Objects detection results from perception module.
        """
        if hasattr(self.vehicle, 'is_proxy'):
            return

        # update localization information
        self._ego_speed = ego_speed
        self._ego_pos = ego_pos
        self.break_distance = self._ego_speed / 3.6 * self.emergency_param
        # update the localization info to trajectory planner
        self.get_local_planner().update_information(ego_pos, ego_speed)
        self.objects = objects


        # ─── 1. Cache any edge-supplied predictions (may be an empty list) ────────
        #self.edge_predictions = list(self.generated_predictions)     # shallow copy

        # ─── 2. Decide which pipeline to use ──────────────────────────────────────
        if self.edge_predictions:            # → edge is active; trust it exclusively
            self.generated_predictions = self.edge_predictions.copy()

        else:                                # → no edge data; run local predictor
            #self._maintain_tracks_and_predict(dt=0.05)   # ≈ sim-step seconds
            #self.generated_predictions = self.local_predictions
            self.generated_predictions = []

        # current version only consider about vehicles
        obstacle_vehicles = objects['vehicles']
        self.obstacle_vehicles = self.white_list_match(obstacle_vehicles)
        
        #if 'static' in objects:
            #self.static_obstacles = objects['static']
        #logger.debug(self.obstacle_vehicles)

        # update the debug helper
        self.planning_metrics.update(ego_speed, self.ttc)

        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

    def add_white_list(self, vm):
        """
        Add vehicle manager to white list.
        """
        self.white_list.append(vm)

    def white_list_match(self, obstacles):
        """
        Match the detected obstacles with the white list.
        Remove the obstacles that are in white list.
        The white list contains all position of target platoon
        member for joining.

        Parameters
        ----------
        obstacles : list
            A list of carla.Vehicle or ObstacleVehicle

        Returns
        -------
        new_obstacle_list : list
            The new list of obstacles.
        """
        new_obstacle_list = []

        for o in obstacles:
            flag = False
            o_x = o.get_location().x
            o_y = o.get_location().y

            o_waypoint = self._map.get_waypoint(o.get_location())
            o_lane_id = o_waypoint.lane_id

            for vm in self.white_list:
                pos = vm.v2x_manager.get_ego_pos()
                vm_x = pos.location.x
                vm_y = pos.location.y

                w_waypoint = self._map.get_waypoint(pos.location)
                w_lane_id = w_waypoint.lane_id

                # if the id is different, then not matched for sure
                if o_lane_id != w_lane_id:
                    continue

                if abs(vm_x - o_x) <= 3.0 and abs(vm_y - o_y) <= 3.0:
                    flag = True
                    break
            if not flag:
                new_obstacle_list.append(o)

        return new_obstacle_list

    def overtake_other_lane(
            self,
            waypoint_list):

        self.get_local_planner().get_waypoints_queue().clear()
        self.get_local_planner().get_trajectory().clear()
        self.get_local_planner().get_waypoint_buffer().clear()


    def set_destination(
            self,
            start_location,
            end_location,
            clean=False,
            end_reset=True,
            clean_history=False,
            waypoint_limit=SET_DESTINATION_WAYPOINT_LIMIT):
        """
        This method creates a list of waypoints from agent's
        position to destination location based on the route returned
        by the global router.

        Parameters
        ----------
        end_reset : boolean
            Flag to reset the waypoint queue.

        start_location : carla.location
            Initial position.

        end_location : carla.location
            Final position.

        clean : boolean
            Flag to clean the waypoint queue.

        clean_history : boolean
            Flag to clean the waypoint history.
        """
        if clean:
            self.get_local_planner().get_waypoints_queue().clear()
            self.get_local_planner().get_trajectory().clear()
            self.get_local_planner().get_waypoint_buffer().clear()
        if clean_history:
            self.get_local_planner().get_history_buffer().clear()

        self.start_waypoint = self._map.get_waypoint(start_location)
        logger.debug("Start Location: (%s, %s, %s)" %(start_location.x, start_location.y, start_location.z))
        logger.debug("Start Location Waypoint: (%s, %s, %s) (%s)" %(self.start_waypoint.transform.location.x, self.start_waypoint.transform.location.y, self.start_waypoint.transform.location.z, self.start_waypoint.transform.rotation.yaw))
        """
        # make sure the start waypoint is behind the vehicle
        waypoint_attempts = 0
        unable_to_find_wp = False
        if self._ego_pos:
            cur_loc = self._ego_pos.location
            cur_yaw = self._ego_pos.rotation.yaw
            _, angle = cal_distance_angle(
                self.start_waypoint.transform.location, cur_loc, cur_yaw)

            while angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, angle = cal_distance_angle(
                    self.start_waypoint.transform.location, cur_loc, cur_yaw)
                waypoint_attempts += 1
                logger.debug("%s is %s degrees from current loc %s | %s" % (self.start_waypoint.transform.location, angle, cur_loc, cur_yaw))
                if waypoint_attempts == waypoint_limit:
                    unable_to_find_wp = True
                    logger.error("unable to find valid start waypoint based on current location of %s, yaw = %s", cur_loc, cur_yaw)
                    break

        if unable_to_find_wp:
            return -1
        """
        end_waypoint = self._map.get_waypoint(end_location)
        logger.debug("End Location: (%s, %s, %s)" %(end_location.x, end_location.y, end_location.z))
        logger.debug("End Location Waypoint: (%s, %s, %s)" %(end_waypoint.transform.location.x, end_waypoint.transform.location.y, end_waypoint.transform.location.z))
        if end_reset:
            self.end_waypoint = end_waypoint

        route_trace = self._trace_route(self.start_waypoint, end_waypoint)

        # DEBUG: Conditional breakpoint - triggers if route seems to go backwards (y decreasing significantly)
        # Set BEHAVIOR_DEBUG=1 env var to enable: BEHAVIOR_DEBUG=1 python opencda.py ...
        import os
        if os.environ.get('BEHAVIOR_DEBUG') and route_trace and len(route_trace) > 1:
            first_wp = route_trace[0][0].transform.location
            last_wp = route_trace[-1][0].transform.location
            # Trigger breakpoint if route goes significantly backwards in Y (potential U-turn)
            if last_wp.y < first_wp.y - 10:
                print(f"[ROUTE DEBUG] Suspicious route: start=({first_wp.x:.1f}, {first_wp.y:.1f}), end=({last_wp.x:.1f}, {last_wp.y:.1f})")
                breakpoint()  # Will pause here - use 'c' to continue, 'n' for next, 'p var' to print

        self._local_planner.set_global_plan(route_trace, clean)

        return 0

    def get_local_planner(self):
        """
        return the local planner
        """
        return self._local_planner

    def reroute(self, spawn_points):
        """
        This method implements re-routing for vehicles
        approaching its destination.  It finds a new target and
         computes another path to reach it.

        Parameters
        ----------
        spawn_points : list
            List of possible destinations for the agent.
        """

        logger.debug("Target almost reached, setting new destination...")
        random.shuffle(spawn_points)
        new_start = \
            self._local_planner.waypoints_queue[-1][0].transform.location
        destination = spawn_points[0].location if \
            spawn_points[0].location != new_start else spawn_points[1].location
        logger.debug("New destination: " + str(destination))
        #input("New Destination set - why?")

        self.set_destination(new_start, destination)

    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the
        optimal route from start_waypoint to end_waypoint.

        Parameters
        ----------
        start_waypoint : carla.waypoint
            Initial position.

        end_waypoint : carla.waypoint
            Final position.
        """
        # Setting up global router
        if self._global_planner is None:
            wld = self.vehicle.get_world()
            dao = GlobalRoutePlannerDAO(
                wld.get_map(), sampling_resolution=self._sampling_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._global_planner = grp

        # Obtain route plan
        route = self._global_planner.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)
        
        #input("Route : %s" %(route))

        #draw_trajetory_points(self.vehicle.get_world(),
        #                          route,
        #                          z=0.1,
        #                          size=.1,
        #                          color=carla.Color(0, 0, 255),
        #                          lt=0.2)

        return route

    def traffic_light_manager(self, waypoint):
        """
        This method is in charge of behaviors for red lights and stops.
        WARNING: What follows is a proxy to avoid having a car brake after
        running a yellow light. This happens because the car is still under
        the influence of the semaphore, even after passing it.
        So, the semaphore id is temporarely saved to ignore it and go around
        this issue, until the car is near a new one.

        Parameters
        ----------
        waypoint : carla.waypoint
            Current waypoint of the agent.

        """

        light_id = self.vehicle.get_traffic_light(
        ).id if self.vehicle.get_traffic_light() is not None else -1

        # this is the case where the vehicle just pass a stop sign, and won't
        # stop at any stop sign in the next 4 seconds.
        if 60 <= self.stop_sign_wait_count < 240:
            self.stop_sign_wait_count += 1
        elif self.stop_sign_wait_count >= 240:
            self.stop_sign_wait_count = 0

        if self.light_state == "Red":
            # when light state is red and light id is -1, it means the vehicle
            # is near a stop sign.
            if light_id == -1:
                # we force the vehicle wait for 2 sceconds in front of the
                # stop sign
                if self.stop_sign_wait_count < 60:
                    self.stop_sign_wait_count += 1
                    # indicate emergent stop needed
                    return 1
                # After pass a stop sign, the vehicle shouldn't stop at
                # the stop sign in the opposite direction
                else:
                    # indicate no need to stop
                    return 0

            if not waypoint.is_junction and (
                    self.light_id_to_ignore != light_id or light_id == -1):
                return 1
            elif waypoint.is_junction and light_id != -1:
                self.light_id_to_ignore = light_id
        if self.light_id_to_ignore != light_id:
            self.light_id_to_ignore = -1
        return 0

    def collision_manager(self, rx, ry, ryaw, waypoint, adjacent_check=False, is_left_turn_at_intersection=False, check_full_path=False):
        """
        This module is in charge of warning in case of a collision.

        Parameters
        ----------
        rx : float
            x coordinates of plan path.

        ry : float
            y coordinates of plan path.

        ryaw : float
            yaw angle.

        waypoint : carla.waypoint
            current waypoint of the agent.

        adjacent_check : boolean
            Whether it is a check for adjacent lane.
        """

        def dist(v):
            return v.get_location().distance(waypoint.transform.location)

        vehicle_state = False
        min_distance = 1000
        target_vehicle = None

        #logger.debug(adjacent_check)
        #logger.debug("generated predictions: %s" %self.generated_predictions)

        for vehicle in self.obstacle_vehicles:
            logger.debug("Self Vehicle Location: (%s, %s, %s)" %(self.vehicle.get_location().x, self.vehicle.get_location().y, self.vehicle.get_location().z))
            logger.debug("Vehicle Id: %s" %vehicle.carla_id)
         
            collision_free = self._collision_check.collision_circle_check(
                rx, ry, ryaw, vehicle, self._ego_speed / 3.6, self._map,
                adjacent_check=adjacent_check, world=self.vehicle.get_world())

            if not collision_free:
                vehicle_state = True

                # the vehicle length is typical 3 meters,
                # so we need to consider that when calculating the distance
                distance = positive(dist(vehicle) - 3)
                # if distance > 10:
                #     vehicle_state = False
                logger.debug("Vehicle non trajectory potential collision Distance: %s" %distance)
                
                if distance < min_distance:
                    min_distance = distance
                    target_vehicle = vehicle

        # Pass 2: Edge/local prediction collision check
        ego_speed_mps = self._ego_speed / 3.6
        ego_loc = self.vehicle.get_location()
        time_step = 0.05
        world = self.vehicle.get_world()

        # Detailed prediction inventory logging
        n_total = len(self.generated_predictions)
        n_ego = sum(1 for p in self.generated_predictions
                    if p.obstacle_trajectory.obstacle.carla_id == self.vehicle.id)
        logger.debug("[PRED INVENTORY] total=%d, ego_filtered=%d, to_check=%d | ego_pos=(%.1f,%.1f) ego_speed=%.1f m/s",
                     n_total, n_ego, n_total - n_ego, ego_loc.x, ego_loc.y, ego_speed_mps)
        for i, pred in enumerate(self.generated_predictions):
            obs = pred.obstacle_trajectory.obstacle
            traj = pred.predicted_trajectory
            start_loc = traj[0].location if traj else None
            end_loc = traj[-1].location if traj else None
            logger.debug("[PRED %d/%d] carla_id=%s track_id=%s start=(%.1f,%.1f) end=(%.1f,%.1f) traj_len=%d",
                         i+1, n_total,
                         obs.carla_id, obs.track_id,
                         start_loc.x if start_loc else 0, start_loc.y if start_loc else 0,
                         end_loc.x if end_loc else 0, end_loc.y if end_loc else 0,
                         len(traj))

        for pred in self.generated_predictions:
            # Identity filter via anchoring protocol
            if pred.obstacle_trajectory.obstacle.carla_id == self.vehicle.id:
                continue

            # Derive obstacle speed from predicted trajectory
            pred_traj = pred.predicted_trajectory
            if len(pred_traj) >= 2:
                p0, p1 = pred_traj[0].location, pred_traj[1].location
                obs_speed = ((p1.x - p0.x)**2 + (p1.y - p0.y)**2)**0.5 / time_step
            else:
                obs_speed = 0.0

            # First pass: check collision WITHOUT drawing (world=None)
            collision, ttc = self._collision_check.trajectory_collision_check(
                rx, ry, ego_speed_mps,
                pred.predicted_trajectory, obs_speed,
                time_step=time_step,
                world=None)

            # Only act on time-synchronized collisions (valid TTC).
            # TTC=1000 means only the spatial-overlap fallback triggered
            # (e.g. parked cars near the path) — not a real collision course.
            if collision and ttc < self._collision_check.time_ahead:
                # Re-run WITH drawing for confirmed collisions only
                self._collision_check.trajectory_collision_check(
                    rx, ry, ego_speed_mps,
                    pred.predicted_trajectory, obs_speed,
                    time_step=time_step,
                    world=world)

                self.ttc = ttc
                self.planning_metrics.update(ego_speed_mps, ttc)
                vehicle_state = True

                # TTC-derived distance for downstream car-following logic
                ttc_distance = max(ego_speed_mps * ttc, 0.5)
                if ttc_distance < min_distance:
                    min_distance = ttc_distance
                    target_vehicle = pred.obstacle_trajectory.obstacle

                obs = pred.obstacle_trajectory.obstacle
                logger.debug("[PRED COLLISION] carla_id=%s track_id=%s TTC=%.2fs dist=%.2fm obs_speed=%.1f m/s",
                             obs.carla_id, obs.track_id, ttc, ttc_distance, obs_speed)

                # Braking attribution
                trigger_cid = pred.obstacle_trajectory.obstacle.carla_id
                self.brake_attributions.append({
                    'trigger_track_id': pred.obstacle_trajectory.obstacle.track_id,
                    'trigger_carla_id': trigger_cid,
                    'is_ego_ghost': (trigger_cid == self.vehicle.id),
                    'obstacle_speed': obs_speed,
                    'ttc': ttc,
                })

        return vehicle_state, target_vehicle, min_distance

    def _find_threat_distance(self, ego_loc, threat_carla_id):
        """Return Euclidean distance to the tracked threat obstacle, or None."""
        if threat_carla_id < 0:
            return None
        for pred in self.generated_predictions:
            obs = pred.obstacle_trajectory.obstacle
            if getattr(obs, 'carla_id', -1) == threat_carla_id:
                obs_loc = obs.get_location()
                return ((obs_loc.x - ego_loc.x)**2 +
                        (obs_loc.y - ego_loc.y)**2)**0.5
        return None

    def overtake_management(self, obstacle_vehicle, set_destination=True):
        """
        Overtake behavior.

        Parameters
        ----------
        obstacle_vehicle : carla.vehicle
            The obstacle vehicle.

        Return
        ------
        vehicle_state : boolean
            Flag indicating whether the vehicle is in dangerous state.
        """
        # obstacle vehicle's location
        obstacle_vehicle_loc = obstacle_vehicle.get_location()
        logger.debug("obstacle vehicle loc: %.1f, %.1f", obstacle_vehicle_loc.x, obstacle_vehicle_loc.y)
        obstacle_vehicle_wpt = self._map.get_waypoint(obstacle_vehicle_loc)

        # whether a lane change is allowed
        left_turn = obstacle_vehicle_wpt.left_lane_marking.lane_change
        right_turn = obstacle_vehicle_wpt.right_lane_marking.lane_change
        #logger.debug("Left Lane Change: %s" %left_turn)
        #logger.debug("Right lane change: %s" %right_turn)

        #logger.debug("Lane Type: %s" %obstacle_vehicle_wpt.left_lane_marking.type)

        # left and right waypoint of the obstacle vehicle
        left_wpt = obstacle_vehicle_wpt.get_left_lane()
        right_wpt = obstacle_vehicle_wpt.get_right_lane()

        #logger.debug("Left Waypoint: %s" %left_wpt)
        #logger.debug("Right waypoint: %s" %right_wpt)

        #logger.debug("Left Waypoint LAne Id: %s" %left_wpt.lane_id)

        # if the vehicle is able to operate left overtake
        #if (left_turn == carla.LaneChange.Left or left_turn ==
        #    carla.LaneChange.Both or obstacle_vehicle_wpt.left_lane_marking.type == carla.LaneMarkingType.Broken) and \
        #        left_wpt and \
                #obstacle_vehicle_wpt.lane_id * left_wpt.lane_id > 0 and \
        #        left_wpt.lane_type == carla.LaneType.Driving or left_wpt.lane_type == carla.LaneType.Bidirectional:qq
        if (left_turn == carla.LaneChange.Left or left_turn ==
            carla.LaneChange.Both or obstacle_vehicle_wpt.left_lane_marking.type == carla.LaneMarkingType.Broken) and \
                left_wpt and \
                left_wpt.lane_type == carla.LaneType.Driving or left_wpt.lane_type == carla.LaneType.Bidirectional:

            self.vehicle.get_world().debug.draw_point(left_wpt.transform.location, size=.1, life_time=2.0)

            # this not the real plan path, but just a quick path to check
            # collision
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
                ego_loc=self._ego_pos.location, target_wpt=left_wpt,
                carla_map=self._map,
                overtake=True, world=self.vehicle.get_world(), oncoming_lane=True)
            vehicle_state, _, _ = self.collision_manager(
                rx, ry, ryaw, self._map.get_waypoint(
                    self._ego_pos.location), True)
            logger.debug("VehicleState: %s" %vehicle_state)
            #logger.debug("Checked for overtake but possibly saw collision")
            if not vehicle_state:
                logger.debug("left overtake is operated")
                if set_destination:
                    self.overtake_counter = 100
                #next_wpt_list = left_wpt.next(15)
                if left_turn == carla.LaneChange.NONE and obstacle_vehicle_wpt.left_lane_marking.type == carla.LaneMarkingType.Broken and self._ego_speed < 20:
                    logger.debug("performing overtake into opposing flow of traffic")
                    import os
                    if os.environ.get('BEHAVIOR_DEBUG'):
                        print(f"[OVERTAKE DEBUG] !!! OPPOSING TRAFFIC OVERTAKE !!! ego=({self._ego_pos.location.x:.1f}, {self._ego_pos.location.y:.1f})")
                        breakpoint()  # This is the U-turn code path - inspect left_wpt, obstacle_vehicle
                    # self.overtake_counter = 200
                    if set_destination:
                        self.overtake_counter = 50  # just enough to be able to change lanes
                        self.overtake_other_direction = True
                    next_wpt_list = []
                    next_wpt_list.append((left_wpt.previous(5)[0], RoadOption.CHANGELANELEFT))
                    next_wpt_list.append((left_wpt.previous(7)[0], RoadOption.LANEFOLLOW))
                    next_wpt_list.append((left_wpt.previous(10)[0], RoadOption.LANEFOLLOW))
                    next_wpt_list.append((left_wpt.previous(13)[0], RoadOption.LANEFOLLOW))
                    next_wpt_list.append((left_wpt.previous(15)[0], RoadOption.LANEFOLLOW))
                    # input(next_wpt_list)
                    if set_destination:
                        self.overtake_end_wpts.append((obstacle_vehicle_wpt.next(30)[0], RoadOption.CHANGELANERIGHT))
                        self.overtake_end_wpts.append((obstacle_vehicle_wpt.next(32)[0], RoadOption.LANEFOLLOW))
                        self.overtake_end_wpts.append((obstacle_vehicle_wpt.next(34)[0], RoadOption.LANEFOLLOW))
                        self.overtake_end_wpts.append((obstacle_vehicle_wpt.next(36)[0], RoadOption.LANEFOLLOW))
                    # next_wpt_list.extend(self.overtake_end_wpts)
                elif left_turn != carla.LaneChange.NONE:
                    if set_destination:
                        self.overtake_other_direction = False
                    next_wpt_list = left_wpt.next(self._ego_speed / 3.6 * 6)
                    if len(next_wpt_list) == 0:
                        return True

                    next_wpt = next_wpt_list[0]
                    left_wpt = left_wpt.next(5)[0]
                    if set_destination:
                        self.set_destination(
                            left_wpt.transform.location,
                            next_wpt.transform.location,
                            clean=True,
                            end_reset=False)
                    return vehicle_state
                else:
                    return True
                
                if len(next_wpt_list) == 0:
                    #input("Next Waypoint empty")
                    return True
                #self.vehicle.get_world().debug.draw_point(left_wpt.transform.location, color=carla.Color(255,255,0), size=.1, life_time=2.0)

                #input("Next waypoint list size : %s"%len(next_wpt_list))
                #input("Next waypoint calculated")
                next_wpt = next_wpt_list[0]
                #input("Left waypoint next")
                left_wpt = left_wpt.previous(5)[0]
                #input("Drawing Point")

                # for wpt in next_wpt_list:
                #     if isinstance(wpt, tuple):
                #         wpt = wpt[0]
                #     self.vehicle.get_world().debug.draw_point(wpt.transform.location, size=.1, life_time=2.0, color=carla.Color(255,255,0))

                #input("Setting Destination")
                # self.set_destination(
                #     next_wpt.transform.location,
                #     left_wpt.transform.location,
                #     clean=True,
                #     end_reset=False)
                
                if set_destination:
                    self.get_local_planner().get_waypoints_queue().clear()
                    self.get_local_planner().get_trajectory().clear()
                    self.get_local_planner().get_waypoint_buffer().clear()

                    # DEBUG: Breakpoint when overtake path is set
                    import os
                    if os.environ.get('BEHAVIOR_DEBUG'):
                        print(f"[OVERTAKE DEBUG] Setting overtake path with {len(next_wpt_list)} waypoints")
                        breakpoint()  # Inspect: next_wpt_list, self._ego_pos, obstacle_vehicle

                    self._local_planner.set_global_plan(next_wpt_list, clean=True)
                    rx, ry, rk, ryaw = self._local_planner.generate_path()
                    vehicle_state, _, _ = self.collision_manager(
                        rx, ry, ryaw, self._map.get_waypoint(
                            self._ego_pos.location), True, check_full_path=True)
                    
                    return vehicle_state
                else:
                    logger.debug("checking for collisions along overtake path")
                    dt = 0.05
                    for pred in self.generated_predictions:
                        # Identity filter via anchoring protocol
                        if pred.obstacle_trajectory.obstacle.carla_id == self.vehicle.id:
                            continue

                        # Derive obstacle speed from predicted trajectory
                        pred_traj = pred.predicted_trajectory
                        if len(pred_traj) >= 2:
                            p0, p1 = pred_traj[0].location, pred_traj[1].location
                            obstacle_speed = ((p1.x - p0.x)**2 + (p1.y - p0.y)**2)**0.5 / dt
                        else:
                            obstacle_speed = 0.0

                        collision, ttc = self._collision_check.waypoint_collision_check(
                                next_wpt_list, self._ego_pos.location, self._ego_speed / 3.6,
                                pred.predicted_trajectory, obstacle_speed,
                                world=self.vehicle.get_world())

                        if collision:
                            self.planning_metrics.update(self._ego_speed / 3.6, ttc)
                            return True

                    return False

        if (right_turn == carla.LaneChange.Right or right_turn ==
            carla.LaneChange.Both) and \
                right_wpt and \
                obstacle_vehicle_wpt.lane_id * right_wpt.lane_id > 0 \
                and right_wpt.lane_type == carla.LaneType.Driving:
            rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
                ego_loc=self._ego_pos.location,
                target_wpt=right_wpt,
                overtake=True,
                carla_map=self._map,
                world=self.vehicle.get_world())

            vehicle_state, _, _ = self.collision_manager(
                rx, ry, ryaw, self._map.get_waypoint(
                    self._ego_pos.location), True)
            if not vehicle_state:
                logger.debug("right overtake is operated")
                next_wpt_list = right_wpt.next(self._ego_speed / 3.6 * 6)
                if len(next_wpt_list) == 0:
                    return True

                next_wpt = next_wpt_list[0]
                right_wpt = right_wpt.next(5)[0]
                if set_destination:
                    self.overtake_other_direction = False
                    self.overtake_counter = 100
                    self.set_destination(
                        right_wpt.transform.location,
                        next_wpt.transform.location,
                        clean=True,
                        end_reset=False)
                #input("Destination Reset due to right turn or overtake")
                return vehicle_state

        return True

    def lane_change_management(self):
        """
        Identify whether a potential hazard exits if operating lane change.

        Returns
        -------
        vehicle_state : boolean
            Whether the lane change is dangerous.
        """
        ego_wpt = self._map.get_waypoint(self._ego_pos.location)
        ego_lane_id = ego_wpt.lane_id
        target_wpt = None

        # check the closest waypoint on the adjacent lane
        for wpt in self.get_local_planner().get_waypoint_buffer():
            if wpt[0].lane_id != ego_lane_id:
                target_wpt = wpt[0]
                break
        if not target_wpt:
            return True

        rx, ry, ryaw = self._collision_check.adjacent_lane_collision_check(
            ego_loc=self._ego_pos.location,
            target_wpt=target_wpt,
            overtake=False,
            carla_map=self._map,
            world=self.vehicle.get_world())
        vehicle_state, _, _ = self.collision_manager(
            rx, ry, ryaw, self._map.get_waypoint(
                self._ego_pos.location), adjacent_check=True)
        return not vehicle_state

    def car_following_manager(self, vehicle, distance, target_speed=None):
        """
        Module in charge of car-following behaviors when there's
        someone in front of us.

        Parameters
        ----------
        vehicle : carla.vehicle)
            Leading vehicle to follow.

        distance : float
            distance from leading vehicle.

        target_speed : float
            The target car following speed.

        Returns
        -------
        target_speed : float
            The target speed for the next step.

        target_loc : carla.Location
            The target location.
        """
        if not isinstance(vehicle, ObstacleVehicle):
            return
        if not target_speed:
            target_speed = self.max_speed - self.speed_lim_dist

        vehicle_speed = get_speed(vehicle)

        delta_v = max(1, (self._ego_speed - vehicle_speed) / 3.6)
        ttc = distance / delta_v if delta_v != 0 else distance / \
                                                      np.nextafter(0., 1.)

        if self.ttc is not None:
            self.ttc = ttc
        # Under safety time distance, slow down.
        if self.safety_time > ttc > 0.0:
            target_speed = min(positive(vehicle_speed - self.speed_decrease),
                               target_speed)

        # Actual safety distance area, try to follow the speed of the vehicle
        # in front.
        else:
            target_speed = 0 if vehicle_speed == 0 else \
                min(vehicle_speed + 1,
                    target_speed)
        return target_speed

    def left_turn_at_intersection(self, waypoint_buffer):
        """
        Check the next waypoints to see if it is a left turn at the intersection.

        Parameters
        ----------
        objects : dict
            The dictionary contains all objects info.

        waypoint_buffer : deque
            The waypoint buffer.

        Returns
        -------
        is_junc : boolean
            Whether there is any future waypoint in the junction shortly.
        """
        
        yaw_change = 0
        starting_yaw = waypoint_buffer[0][0].transform.rotation.yaw
        #logger.debug("Waypoint buffer size: %s" %len(waypoint_buffer))
        for i, (wpt, _) in enumerate(waypoint_buffer):
            #logger.debug("Waypoint is junction: %s" %wpt.is_junction)
            if wpt.is_junction and i < 3:
                for wpt, _ in waypoint_buffer:
                    yaw_change = wpt.transform.rotation.yaw - starting_yaw
                    #logger.debug("Yaw Change: %s" %yaw_change)
                    #logger.debug("Waypoint road id: %s Waypoint start road id: %s" %(wpt.road_id, waypoint_buffer[0][0].road_id))
                    
                    #self.vehicle.get_world().debug.draw_point(wpt.transform.location, size=.1, life_time=2.0)
                    #if wpt.road_id != waypoint_buffer[0][0].road_id and \
                    if yaw_change < -60 and \
                            yaw_change > -120:
                        #logger.debug("Making a left turn at intersection")
                        return True
        return False

    def is_intersection(self, objects, waypoint_buffer):
        """
        Check the next waypoints is near the intersection. This is done by
        check the distance between the waypoints and the traffic light.

        Parameters
        ----------
        objects : dict
            The dictionary contains all objects info.

        waypoint_buffer : deque
            The waypoint buffer.

        Returns
        -------
        is_junc : boolean
            Whether there is any future waypoint in the junction shortly.
        """
        #logger.debug("Check intersection")
        for tl in objects['traffic_lights']:
            for wpt, _ in waypoint_buffer:
                distance = \
                    tl.get_location().distance(wpt.transform.location)
                #logger.debug(distance)
                if distance < 15:
                    #logger.debug("is Intersection")
                    return True
        return False

    def is_close_to_destination(self):
        """
        Check if the current ego vehicle's position is close to destination

        Returns
        -------
        flag : boolean
            It is True if the current ego vehicle's position is close to destination

        """
        flag = abs(self._ego_pos.location.x - self.end_waypoint.transform.location.x) <= 10 and \
               abs(self._ego_pos.location.y - self.end_waypoint.transform.location.y) <= 10
        return flag

    def check_lane_change_permission(self, lane_change_allowed, collision_detector_enabled, rk):
        """
        Check if lane change is allowed.
        Several conditions will influence the result such as the road curvature, collision detector, overtake and push status.
        Please refer to the code for complete conditions.

        Parameters
        ----------
        lane_change_allowed : boolean
            Previous lane change permission.

        collision_detector_enabled : boolean
            True if collision detector is enabled.

        rk : list
            List of planned path points' curvatures.

        Returns
        -------
        lane_change_enabled : boolean
            True if lane change is allowed


        """
        # the lane change is forbidden if driving within a large curve
        if len(rk) > 2 and np.mean(np.abs(np.array(rk))) > 0.04:
            lane_change_allowed = False
        # change the lane change permission only when all of the following conditions are satisfied:
        # * collision detector is enabled : otherwise we won't perform collision check for lane change.
        # * lane id changes and lane lateral changes : makes sure it is indeed a lane change happen in our planned route.
        # * overtake hasn't happened : if previously we have been doing an overtake, then lane change should not be allowed.
        # * destination is not pushed : if we have been doing destination pushed, then lane change should not be allowed.
        lane_change_enabled_flag = collision_detector_enabled and \
               self.get_local_planner().lane_id_change and \
               self.get_local_planner().lane_lateral_change and \
               self.overtake_counter <= 0 and \
               not self.destination_push_flag
        if lane_change_enabled_flag:
            lane_change_allowed = lane_change_allowed and self.lane_change_management()
            if not lane_change_allowed:
                logger.debug("lane change not allowed")

        return lane_change_allowed

    def get_push_destination(self, ego_vehicle_wp, is_intersection):
        """
        Get the destination for push operation.

        Parameters
        ----------
        ego_vehicle_wp : carla.waypoint
            Ego vehicle's waypoint.

        is_intersection : boolean
            True if in the intersection.

        Returns
        -------
        reset_target : carla.waypoint
            Temporal push destination.

        """
        waypoint_buffer = self.get_local_planner().get_waypoint_buffer()
        reset_index = len(waypoint_buffer) // 2

        # when it comes to the intersection, we need to use the future
        # waypoint to make sure the next waypoint is at the same lane
        if is_intersection:
            reset_target = waypoint_buffer[reset_index][0].next(
                max(self._ego_speed / 3.6, 10.0))[0]
        else:
            reset_target = \
                ego_vehicle_wp.next(max(self._ego_speed / 3.6 * 3,
                                        10.0))[0]
        logger.debug(
            'Vehicle id: %d :destination pushed forward because of '
            'potential collision, reset destination :%f. %f, %f' %
            (self.vehicle.id, reset_target.transform.location.x,
             reset_target.transform.location.y,
             reset_target.transform.location.z))
        return reset_target

    def run_step(
            self,
            target_speed=None,
            collision_detector_enabled=True,
            lane_change_allowed=True):
        """
        Execute one step of navigation

        Parameters
        __________
        collision_detector_enabled : boolean
            Whether to enable collision detection.

        target_speed : float
            A manual order to achieve certain speed.

        lane_change_allowed : boolean
            Whether lane change is allowed. This is passed from
            platoon behavior agent.

        Returns
        -------
        control : carla.VehicleControl
            Vehicle control of the next step.
        """

        agent_step_start_time = time.time()
        # retrieve ego location
        ego_vehicle_loc = self._ego_pos.location
        if len(self.ego_location_buffer) == 10:
            self.ego_location_buffer.pop(0)
        self.ego_location_buffer.append(ego_vehicle_loc)
        ego_vehicle_wp = self._map.get_waypoint(ego_vehicle_loc)
        waipoint_buffer = self.get_local_planner().get_waypoint_buffer()
        #logger.debug(waipoint_buffer)
        # ttc reset to 1000 at the beginning
        self.ttc = 1000
        # when overtake_counter > 0, another overtake/lane change is forbidden
        if self.overtake_counter > 0 and not self.overtake_other_direction:
            self.overtake_counter -= 1

        # we reset destination push flag for every n rounds
        if self.destination_push_flag > 0:
            self.destination_push_flag -= 1
        
        #logger.debug(self.objects)
        # use traffic light to detect intersection
        is_intersection = self.is_intersection(self.objects, waipoint_buffer)
        #logger.debug("Is Intersection: %s" %is_intersection)

        start_time = time.time()
        # 0. Simulation ends condition
        if self.is_close_to_destination():

            # eCLOUD
            if self._is_dist:
                return -1, None # eCloud: Use -1 to indicate simulation end. Need a better way than this.
            else:
                sys.exit(0)
        end_time = time.time()
        self.planning_metrics.update_agent_step_list(0, end_time-start_time)
        logger.debug("step 0 complete")

        start_time = time.time()
        # 1. Traffic light management
        if self.traffic_light_manager(ego_vehicle_wp) != 0:
            # TODO - eCLOUD: (we have no traffic lights in sims yet)
            #logger.debug("Traffic light manager returned 1, stopping")
            return 0, None
        end_time = time.time()
        self.planning_metrics.update_agent_step_list(1, end_time-start_time)
        logger.debug("step 1 complete")

        start_time = time.time()
        # 2. when the temporary route is finished, we return to the global route
        if len(self.get_local_planner().get_waypoints_queue()) == 0 \
                and len(self.get_local_planner().get_waypoint_buffer()) <= 1:
            logger.debug('Global Destination Reset!')
            #input("Waypoint Buffer Size: %s"  %len(self.get_local_planner().get_waypoint_buffer()))
            # in case the vehicle is disabled overtaking function
            # at the beginning
            self.overtake_allowed = True and self.overtake_allowed_origin
            self.lane_change_allowed = True
            self.destination_push_flag = 0
            rerouted = self.set_destination(
                ego_vehicle_loc,
                self.end_waypoint.transform.location,
                clean=True,
                clean_history=True)
            
            if rerouted == -1:
                logger.debug("Trying to reroute but failed, stopping")
                return 0, None

        end_time = time.time()
        logger.debug("Local planner destination reached block: %s" %(end_time - start_time))
        self.planning_metrics.update_agent_step_list(2, end_time-start_time)
        logger.debug("step 2 complete")

        # intersection behavior. if the car is near a intersection, no overtake is allowed
        if is_intersection:
            logger.debug("Overake not allowed because of intersection")
            self.overtake_allowed = False
        else:
            logger.debug("Overtake is allowed because not in intersection")
            self.overtake_allowed = True and self.overtake_allowed_origin

        start_time = time.time()
        # 3. Path generation based on the global route
        rx, ry, rk, ryaw = self._local_planner.generate_path()
        end_time = time.time()
        logger.debug("Local planner path generation time: %s" %(end_time - start_time))
        self.planning_metrics.update_agent_step_list(3, end_time-start_time)
        logger.debug("step 3 complete")

        # 4. check whether lane change is allowed
        start_time = time.time()
        self.lane_change_allowed = self.check_lane_change_permission(lane_change_allowed, collision_detector_enabled, rk)
        end_time = time.time()
        self.planning_metrics.update_agent_step_list(4, end_time-start_time)
        logger.debug("step 4 complete")
        logger.debug("Lane change Allowed: %s" %self.lane_change_allowed)

        # 5. Check if left turn at intersection
        if is_intersection:
            left_turn = self.left_turn_at_intersection(waipoint_buffer)
            #logger.debug("Left Turn at Intersection: %s" %left_turn)
        else:
            left_turn = False

        # 5. Collision check
        start_time = time.time()
        is_hazard = False
        if collision_detector_enabled:
            is_hazard, obstacle_vehicle, distance = self.collision_manager(
                rx, ry, ryaw, ego_vehicle_wp, is_left_turn_at_intersection=left_turn)

        # RSS-inspired proper response: once a prediction collision is
        # detected, the ego must execute a "proper response" (braking) until
        # a provably safe state is reached.  Without this, decelerating
        # changes the ego's projected path → collision check says "safe" →
        # ego accelerates → collision reappears (oscillation).
        #
        # Enter condition:  prediction collision with TTC < time_ahead
        # Proper response:  emergency stop (target_speed = 0)
        # Exit conditions:  (a) ego has stopped (speed < 2 km/h), OR
        #                   (b) obstacle has cleared — its lateral distance
        #                       to ego is growing for consecutive ticks, OR
        #                   (c) safety TTL expires (fallback upper bound)
        ego_loc = self._ego_pos.location if self._ego_pos else None

        if is_hazard and self.ttc < self._collision_check.time_ahead:
            # Enter proper response — record the threatening obstacle
            # and force emergency stop immediately (distance=0 bypasses
            # car_following → local_planner → PID, which only gives
            # gentle braking; emergency stop gives brake=1.0)
            self._committed_brake_ttl = max(
                self._committed_brake_ttl,
                max(20, int(self.ttc / 0.05)))
            distance = 0  # force emergency stop on FIRST tick too
            if obstacle_vehicle is not None and ego_loc is not None:
                obs_loc = obstacle_vehicle.get_location()
                self._rss_threat_carla_id = getattr(
                    obstacle_vehicle, 'carla_id', -1)
                self._rss_last_lateral_dist = (
                    (obs_loc.x - ego_loc.x)**2 +
                    (obs_loc.y - ego_loc.y)**2)**0.5
                self._rss_lateral_growing_ticks = 0
            logger.debug("[RSS] Enter proper response — TTC=%.2fs, "
                         "ego_speed=%.1f km/h",
                         self.ttc, self._ego_speed)

        elif not is_hazard and self._committed_brake_ttl > 0:
            # Collision check says safe, but we are in proper response.
            # Check formal exit conditions before releasing.
            safe = False

            # (a) Ego has stopped
            if self._ego_speed < 2.0:
                safe = True
                logger.debug("[RSS] Exit: ego stopped (%.1f km/h)",
                             self._ego_speed)

            # (b) Obstacle lateral distance growing (cleared the path)
            if not safe and ego_loc is not None:
                threat_dist = self._find_threat_distance(
                    ego_loc, self._rss_threat_carla_id)
                if threat_dist is not None:
                    if threat_dist > self._rss_last_lateral_dist + 0.5:
                        self._rss_lateral_growing_ticks += 1
                    else:
                        self._rss_lateral_growing_ticks = 0
                    self._rss_last_lateral_dist = threat_dist
                    if self._rss_lateral_growing_ticks >= 3:
                        safe = True
                        logger.debug(
                            "[RSS] Exit: obstacle clearing (dist=%.1f, "
                            "growing %d ticks)", threat_dist,
                            self._rss_lateral_growing_ticks)

            # (c) TTL fallback (upper bound safety net)
            self._committed_brake_ttl -= 1
            if self._committed_brake_ttl <= 0:
                safe = True
                logger.debug("[RSS] Exit: TTL expired")

            if not safe:
                is_hazard = True
                distance = 0  # force emergency stop
                logger.debug(
                    "[RSS] Proper response active — TTL=%d, "
                    "ego_speed=%.1f km/h",
                    self._committed_brake_ttl, self._ego_speed)
            else:
                self._committed_brake_ttl = 0
                self._rss_lateral_growing_ticks = 0

        car_following_flag = False
        end_time = time.time()
        self.planning_metrics.update_agent_step_list(5, end_time-start_time)
        logger.debug("step 5 complete")

        if not is_hazard:
            if self.overtake_counter > 0 and self.overtake_other_direction:
                self.overtake_counter -= 1
            self.hazard_flag = False

        # 6. composite steps 7 - 9
        # 7. push case. Push the car to a temporary destination when original lane change action can't be executed
        # The case that the vehicle is doing lane change as planned
        # but found vehicle blocking on the other lane
        start_time = time.time()
        end_time_7 = start_time
        end_time_8 = start_time
        end_time_9 = start_time
        logger.debug("Hazard: %s" %(is_hazard))
        # if is_hazard:
        #     input("Hazard: %s" %(is_hazard))
        if not self.lane_change_allowed and \
                self.get_local_planner().potential_curved_road \
                and not self.destination_push_flag and \
                self.overtake_counter <= 0 and \
                not self.overtake_other_direction:
            self.overtake_allowed = False
            # get push destination based on intersection flag and current waypoint (rule-based)
            reset_target = self.get_push_destination(ego_vehicle_wp, is_intersection)
            # set the flag, so the push operation is not allowed for the next few frames.
            self.destination_push_flag = 90
            # DEBUG: Breakpoint when push destination is set
            import os
            if os.environ.get('BEHAVIOR_DEBUG'):
                print(f"[PUSH DEBUG] Push destination: ego=({ego_vehicle_loc.x:.1f}, {ego_vehicle_loc.y:.1f}), target=({reset_target.transform.location.x:.1f}, {reset_target.transform.location.y:.1f})")
                breakpoint()
            self.set_destination(
                ego_vehicle_loc,
                reset_target.transform.location,
                clean=True,
                end_reset=False)
            logger.debug("Doing lane change as planned but found vehicle blocking other lane")
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            end_time_7 = time.time()

        # 8. the case that vehicle is blocking in front and overtake not
        # allowed or it is doing overtaking the second condition is to
        # prevent successive overtaking
        elif is_hazard and (not left_turn) and (not self.overtake_allowed or
                self.overtake_counter > 0 or self.get_local_planner().potential_curved_road): #TL - Why is this logic here?
            #logger.debug("Vehicle is blocking in front or overtake is not allowed")
            #logger.debug("Overtake Allowed: %s" %self.overtake_allowed)
            #logger.debug("Overtake Counter: %s" %self.overtake_counter)
            #logger.debug("Curved Road: %s" %self.get_local_planner().potential_curved_road)
            car_following_flag = True
            end_time_8 = time.time()
        # 9. overtake handeling
        elif is_hazard and self.overtake_allowed and \
                self.overtake_counter <= 0  and obstacle_vehicle != None:
            logger.debug("Overtake Allowed and overtake counter is 0")
            import os
            if os.environ.get('BEHAVIOR_DEBUG'):
                print(f"[OVERTAKE DEBUG] Hazard detected! obstacle_vehicle={obstacle_vehicle}, ego_pos=({self._ego_pos.location.x:.1f}, {self._ego_pos.location.y:.1f})")
            if isinstance(obstacle_vehicle, ObstacleVehicle):
                obstacle_speed = get_speed(obstacle_vehicle)
            obstacle_lane_id = self._map.get_waypoint(obstacle_vehicle.get_location()).lane_id
            ego_lane_id = self._map.get_waypoint(
                self._ego_pos.location).lane_id
            #logger.debug("Ego Lane Id: %s" %ego_lane_id)
            #logger.debug("Obstacle Lane ID: %s" %obstacle_lane_id)
            # overtake the obstacle vehicle only when speed is bigger and the
            # lane id is the same
            if ego_lane_id == obstacle_lane_id:
                # this flag is used for transition from cut-in joining to back
                # joining
                self.hazard_flag = is_hazard
                # we only consider overtaking when speed is faster than the
                # front obstacle
                if self._ego_speed >= obstacle_speed - 5:
                    # we want to perform an overtake, but we have to wait first
                    if self.overtake_wait_counter > 0 and not self.do_overtake:
                        logger.debug("overtake wait counter: %s", self.overtake_wait_counter)
                        self.overtake_wait_counter -= 1
                        collision = self.overtake_management(obstacle_vehicle, set_destination=False)
                        if collision:
                            self.num_overtake_collisions += 1
                            logger.debug("num collisions in overtake: %s", self.num_overtake_collisions)
                        car_following_flag = True  
                    elif self.overtake_wait_counter <= 0 and not self.do_overtake:
                        car_following_flag = self.overtake_management(obstacle_vehicle, set_destination=False)
                        if self.num_overtake_collisions > 1 or car_following_flag:
                            # we saw too many potential collisions, wait a little bit
                            logger.debug("Saw too many collisions, restarting overtake timer")
                            self.overtake_wait_counter = self.overtake_wait_time / 2
                        else:
                            self.do_overtake = True
                            car_following_flag = self.overtake_management(obstacle_vehicle, set_destination=True)
                            logger.debug("vehicle state in overtake %s", car_following_flag)
                        self.num_overtake_collisions = 0

                    rx, ry, rk, ryaw = self._local_planner.generate_path()
                else:
                    car_following_flag = True
                end_time_9 = time.time()
        # return to other lane if overtaking
        elif self.overtake_counter <= 0 and self.overtake_other_direction and len(self.overtake_end_wpts) > 0:
            self.overtake_counter = 100 # perform another lane change

            # DEBUG: Breakpoint when returning from overtake
            import os
            if os.environ.get('BEHAVIOR_DEBUG'):
                print(f"[OVERTAKE DEBUG] Returning from overtake with {len(self.overtake_end_wpts)} waypoints")
                breakpoint()

            self._local_planner.set_global_plan(self.overtake_end_wpts)
            self.overtake_end_wpts.clear()
            rx, ry, rk, ryaw = self._local_planner.generate_path()
            car_following_flag, _, _ = self.collision_manager(
                    rx, ry, ryaw, self._map.get_waypoint(
                        self._ego_pos.location), True)
        elif self.overtake_counter <= 0 and self.overtake_other_direction and len(self.overtake_end_wpts) == 0:
            self.overtake_other_direction = False
        elif is_hazard and left_turn:
            if distance < max(self.break_distance, 3):
                logger.debug("Car Entering Intersection and break distance is closer than 3 meters")
                return 0, None
        
        if self.overtake_counter <= 0 and not self.overtake_other_direction and self.do_overtake:
            self.do_overtake = False
            self.num_overtake_collisions = 0
            self.overtake_wait_counter = self.overtake_wait_time

        end_time = time.time()

        self.planning_metrics.update_agent_step_list(6, end_time-start_time)
        self.planning_metrics.update_agent_step_list(7, end_time_7-start_time)
        self.planning_metrics.update_agent_step_list(8, end_time_8-start_time)
        self.planning_metrics.update_agent_step_list(9, end_time_9-start_time)
        logger.debug("steps 6, 7, 8, 9 complete")

        # 10. Car following behavior
        start_time = time.time()
        if car_following_flag:
            #logger.debug("Distance: %s" %distance)
            if distance < max(self.break_distance, 3):
                logger.debug("Car Following/Hazard in front and break distance is closer than 3 meters, stopping")
                logger.debug("Current Speed: %s" %self._ego_speed)
                end_time = time.time()
                self.planning_metrics.update_agent_step_list(10, end_time-start_time)
                self.planning_metrics.update_agent_step_list(11, 0)
                return 0, None

            target_speed = self.car_following_manager(obstacle_vehicle, distance, target_speed)
            target_speed, target_loc = self._local_planner.run_step(
                rx, ry, rk, target_speed=target_speed)
            end_time = time.time()
            self.planning_metrics.update_agent_step_list(10, end_time-start_time)
            logger.debug("step 10 complete - following and exiting")
            self.planning_metrics.update_agent_step_list(11, 0)
            return target_speed, target_loc
        end_time = time.time()
        #self.planning_metrics.update_agent_step_list(10, end_time-start_time)
        
        # 11. Normal behavior
        start_time = time.time()
        target_speed, target_loc = self._local_planner.run_step(
            rx, ry, rk, target_speed=self.max_speed - self.speed_lim_dist
            if not target_speed else target_speed)
        logger.debug("Target Speed: %s" %target_speed)
        logger.debug("Target Loc: %s" %target_loc)
        logger.debug("Current Speed: %s" %self._ego_speed)
        end_time = time.time()
        logger.debug("Local planner run step time: %s" %(end_time - start_time))
        self.planning_metrics.update_agent_step_list(11, end_time-start_time)
        logger.debug("step 11 complete")
        #if(self.overtake_counter == 100):
                #input("Check logs, check trajectory")


        return target_speed, target_loc


