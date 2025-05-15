# -*- coding: utf-8 -*-
"""
Tracking module base
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import weakref
import sys
import time

import carla
import cv2
import numpy as np
import open3d as o3d
import scipy as sc

import opencda.core.sensing.perception.sensor_transformation as st
from opencda.core.common.misc import \
    cal_distance_angle, get_speed, get_speed_sumo
from opencda.core.sensing.perception.obstacle_vehicle import \
    ObstacleVehicle
from opencda.core.sensing.perception.static_obstacle import TrafficLight
from opencda.core.sensing.perception.o3d_lidar_libs import \
    o3d_visualizer_init, o3d_pointcloud_encode, o3d_visualizer_show, \
    o3d_camera_lidar_fusion
from opencda.client_debug_helper import ClientDebugHelper

class TrackingManager:
    """
    Default perception module. Currenly only used to detect vehicles.

    Parameters
    ----------
    vehicle : carla.Vehicle
        carla Vehicle, we need this to spawn sensors.

    config_yaml : dict
        Configuration dictionary for perception.

    cav_world : opencda object
        CAV World object that saves all cav information, shared ML model,
         and sumo2carla id mapping dictionary.

    data_dump : bool
        Whether dumping data, if true, semantic lidar will be spawned.

    carla_world : carla.world
        CARLA world, used for rsu.

    Attributes
    ----------
    lidar : opencda object
        Lidar sensor manager.

    rgb_camera : opencda object
        RGB camera manager.

    o3d_vis : o3d object
        Open3d point cloud visualizer.
    """

    def __init__(self, vehicle, cav_world,
                 data_dump=False, carla_world=None, infra_id=None, tracker_type=None):
        self.vehicle = vehicle
 
        if hasattr(vehicle, 'get_world'): 
          self.carla_world = carla_world if carla_world is not None \
            else self.vehicle.get_world()
        else:
          self.carla_world = carla_world
        if self.carla_world is not None:
            self._map = self.carla_world.get_map()
        self.id = infra_id if infra_id is not None else vehicle.id
        #print(carla_world)

        self.cav_world = weakref.ref(cav_world)()
        ml_manager = cav_world.ml_manager

        # count how many steps have been passed
        self.count = 0
        # ego position
        self.ego_pos = None

        # the dictionary contains all objects
        self.data_dump = data_dump

        # TODO: make this a config
        #self.activate_mode = True

        if tracker_type == 'SORT':
            from opencda.core.sensing.tracking.sort_tracker import MultiObjectSORTTracker
            self.tracker = MultiObjectSORTTracker(max_age=5, min_matching_iou=0.3)

        self.debug_helper = ClientDebugHelper(0)

    def dist(self, a):
        """
        A fast method to retrieve the obstacle distance the ego
        vehicle from the server directly.

        Parameters
        ----------
        a : carla.actor
            The obstacle vehicle.

        Returns
        -------
        distance : float
            The distance between ego and the target actor.
        """
        return a.get_location().distance(self.ego_pos.location)

    def track(self, objects):
        #if self.activate_mode:
        return self.activate_mode(objects)
        #else:
        #    return self.deactivate_mode(objects)

    def deactivate_mode(self, objects, ego_pos):
        """
        Object detection using server information directly.

        Parameters
        ----------
        objects : dict
            The dictionary that contains all category of detected objects.
            The key is the object category name and value is its 3d coordinates
            and confidence.

        Returns
        -------
         objects: dict
            Updated object dictionary.
        """
        #perception_start_time = time.time()
        matches = []
        self.ego_pos = ego_pos
        world = self.carla_world

        vehicle_list = world.get_actors().filter("*vehicle*")
        # todo: hard coded
        thresh = 10000 if not self.data_dump else 100000

        vehicle_list = [v for v in vehicle_list if self.dist(v) < thresh and
                        v.id != self.id]

        
        for i, detected_vehicle in enumerate(objects['vehicles']):
            # Remove duplicate vehicles by iterating over list twice
            for j, vehicle in enumerate(objects['vehicles']):
                if i == j:
                    continue
                if detected_vehicle.get_location().distance(vehicle.get_location()) < 3:
                    objects['vehicles'].remove(vehicle)
                    
        matrix = np.zeros((len(objects['vehicles']), len(vehicle_list)))
    

        for i, detected_vehicle in enumerate(objects['vehicles']):
            print("Location: (%s, %s, %s)" %(detected_vehicle.get_location().x, detected_vehicle.get_location().y, detected_vehicle.get_location().z))
            world.debug.draw_string(detected_vehicle.get_location(), "%s"%(i))
        
            for j, vehicle in enumerate(vehicle_list):
                print("j: %s" %j)
                print("VID: %s Location: (%s, %s, %s)" %(vehicle.id, vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z))
                world.debug.draw_string(vehicle.get_location(), "V %s"%(vehicle.id))
                matrix[i][j] = detected_vehicle.get_location().distance(vehicle.get_location())
                print("Distance: %s" %matrix[i][j])



        print(matrix)

        row, col = sc.optimize.linear_sum_assignment(matrix)

        #print(row)
        print(col)

        #perception_end_time = time.time()
        #self.debug_helper.update_perception_time((perception_end_time - perception_start_time)*1000)

        for i, vehicle_index in enumerate(col):
            matches.append(vehicle_list[vehicle_index].id)

        #matches = [(detected_vehicles[i], queried_vehicles[j]) for i, j in zip(row, col)]

        for i, j in zip(row, col):
            objects['vehicles'][i].set_carla_id(vehicle_list[j].id)

        return matches

    def activate_mode(self, objects):
        return self.tracker.track(objects)
