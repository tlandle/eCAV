# -*- coding: utf-8 -*-
"""
Extensible modular class for Network modeling. Calculates latency between individual Vehicle client and a networked control plane such as an Edge or Cloud

- track world time and increment it each time it ticks
- use world time and latency to evaluate SET_DESTINATION commands in queue
- Sim API *should* manage this and not send SET_DESTINATION to a specific vehicle until it's ready, but it's also fine to just broadcast them all and for individual vehicles to manage the queue 
- it's possibly a bit easier if Sim API keeps queue since it can manage latencies centrally; otherwise, we need a way for vehicles to get latencies - which is fine? - but it does require support for a PING command
"""
# Author: Jordan Rapp <jrapp7@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import math
import importlib

import numpy as np
import carla

from opencda.core.common.misc import compute_distance
from opencda.core.application.edge.transform_utils import *
from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.scenario_testing.utils.yaml_utils import load_yaml

import coloredlogs, logging
logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)

cloud_config = load_yaml("cloud_config.yaml")
if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as ecloud_rpc

class NetworkEmulator:

    def __init__(self,
                 vehicle_manager,
                 edge_sets_destination=False) -> None:
        self.wp_list = []
        self.vehicle_manager = vehicle_manager
        self.edge_sets_destination = edge_sets_destination
        self.latest_wp = None

    def set_latency_factor(self, latency_factor):
        '''
        should be something like time-per-meter
        '''
        pass

    def get_latency_between_nodes(self, node_1=None, node_2=None):
        '''
        the two nodes in this case can be two vehicles or a vehicle and an edge. maybe should be less generic...
        '''
        return -2 # just a constant offset into the WP list for now. 

    def enqueue_wp(self, wpb):
        new_wpb = ecloud.WaypointBuffer()
        new_wpb.CopyFrom(wpb)
        logger.info("received new WaypointBuffer: %s", new_wpb)
        self.wp_list.append(new_wpb)

    def fetch_wp(self):
        latency_offset = self.get_latency_between_nodes()

        if len(self.wp_list) >= abs(latency_offset) and \
            self.latest_wp != self.wp_list[latency_offset]:
            self.latest_wp = self.wp_list[latency_offset] 
            logger.info("fetching new WPB: %s", self.latest_wp)
            return self.latest_wp
        
        return None
    
    def update_waypoints(self):
        is_wp_valid = False
        waypoint_proto = self.fetch_wp()
        if waypoint_proto != None:
            '''
            world = self.vehicle_manager_list[0].vehicle.get_world()
            self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
            location = self._dao.get_waypoint(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0))
            '''
            world = self.vehicle_manager.vehicle.get_world()
            dao = GlobalRoutePlannerDAO(world.get_map(), 2)
            for swp in waypoint_proto.waypoint_buffer:
                logger.info("Override Waypoint x:%s, y:%s, z:%s, rl:%s, pt:%s, yw:%s", swp.transform.location.x, swp.transform.location.y, swp.transform.location.z, swp.transform.rotation.roll, swp.transform.rotation.pitch, swp.transform.rotation.yaw)
                wp = deserialize_waypoint(swp, dao)
                logger.info("DAO Waypoint x:%s, y:%s, z:%s, rl:%s, pt:%s, yw:%s", wp.transform.location.x, wp.transform.location.y, wp.transform.location.z, wp.transform.rotation.roll, wp.transform.rotation.pitch, wp.transform.rotation.yaw)
                is_wp_valid = self.vehicle_manager.agent.get_local_planner().is_waypoint_valid(waypoint=wp)

                if self.edge_sets_destination and is_wp_valid:
                    cur_location = self.vehicle_manager.vehicle.get_location()
                    start_location = carla.Location(x=cur_location.x, y=cur_location.y, z=cur_location.z)
                    end_location = carla.Location(x=wp.transform.location.x, y=wp.transform.location.y, z=wp.transform.location.z)
                    self.vehicle_manager.set_destination(start_location, end_location, clean=True, end_reset=True)
                    logger.info("edge set destination to %s", end_location)

                elif is_wp_valid:
                    # override waypoints
                    waypoint_buffer = self.vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
                    # print(waypoint_buffer)
                    # for waypoints in waypoint_buffer:
                    #   print("Waypoints transform for Vehicle Before Clearing: " + str(i) + " : ", waypoints[0].transform)
                    waypoint_buffer.clear() #EDIT MADE
                    waypoint_buffer.append((wp, RoadOption.STRAIGHT))
                    logger.info("cleared WP buffer and set next WP to %s", wp)

        cur_location = self.vehicle_manager.vehicle.get_location()
        logger.debug("location is - x: %s, y: %s", cur_location.x, cur_location.y)

        waypoints_buffer_printer = self.vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
        for waypoints in waypoints_buffer_printer:
            logger.debug("waypoint_proto: waypoints transform for Vehicle: %s", waypoints[0].transform)
