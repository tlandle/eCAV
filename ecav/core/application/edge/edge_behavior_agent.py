# -*- coding: utf-8 -*-

"""Behavior manager for platooning specifically
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import weakref
from collections import deque

import carla
import numpy as np

from ecav.core.application.platooning.fsm import FSM
from ecav.core.application.edge.edge_metrics import \
    EdgeMetrics
from ecav.core.common.misc import \
    compute_distance, get_speed, cal_distance_angle
from ecav.core.plan.behavior_agent import BehaviorAgent


class EdgeBehaviorAgent(BehaviorAgent):
    """
    Edge behavior agent that inherits the single vehicle behavior agent.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla vehicle.

    vehicle_manager : ecav object
        The vehicle manager, used when joining platoon finished.

    v2x_manager : ecav object
        Used to received and deliver information.

    behavior_yaml : dict
        The configuration dictionary for BehaviorAgent.

    carla_map : carla.Map
        The HD Map used in the simulation.

    Attributes
    ----------
    vehicle_manager : ecav object
        The weak reference of the vehicle manager, used when joining platoon
        finished.

    v2x_manager : ecav object
        The weak reference of the v2x_manager

    debug_helper : ecav Object
        A debug helper used to record the driving performance
         during platooning

    """

    def __init__(
            self,
            vehicle,
            vehicle_manager,
            v2x_manager,
            behavior_yaml,
            carla_map):

        super(
            EdgeBehaviorAgent,
            self).__init__(
            vehicle,
            carla_map,
            behavior_yaml)

        self.vehicle_manager = weakref.ref(vehicle_manager)()
        # communication manager
        self.v2x_manager = weakref.ref(v2x_manager)()

        # used to calculate performance
        self.edge_metrics = EdgeMetrics(self.vehicle.id)

    def run_step(
            self,
            target_speed=None,
            collision_detector_enabled=True,
            lane_change_allowed=True):
        """
        Run a single step for navigation under platooning agent.
        Finite state machine is used to switch between different
        platooning states.

        Parameters
        ----------
        target_speed : float
            Target speed in km/h

        collision_detector_enabled : bool
            Whether collision detection enabled.

        lane_change_allowed : bool
            Whether lane change is allowed.
        """

        return super().run_step(target_speed, collision_detector_enabled)

    def update_information(self, ego_pos, ego_speed, objects):
        """
        Update the perception and localization
        information to the behavior agent.

        Parameters
        ----------
        ego_pos : carla.Transform
            Ego position from localization module.

        ego_speed : float
            km/h, ego speed.

        objects : dict
            Objects detection results from perception module.
        """
        # update localization information
        self._ego_speed = ego_speed
        self._ego_pos = ego_pos
        self.break_distance = self._ego_speed / 3.6 * self.emergency_param
        # update the localization info to trajectory planner
        self.get_local_planner().update_information(ego_pos, ego_speed)

        # current version only consider about vehicles
        self.objects = objects
        obstacle_vehicles = objects['vehicles']
        self.obstacle_vehicles = self.white_list_match(obstacle_vehicles)

        # update the debug helper
        self.edge_metrics.update(
            ego_speed,
            self.ttc)

        if self.ignore_traffic_light:
            self.light_state = "Green"
        else:
            # This method also includes stop signs and intersections.
            self.light_state = str(self.vehicle.get_traffic_light_state())

      
