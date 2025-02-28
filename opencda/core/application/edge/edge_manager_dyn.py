# -*- coding: utf-8 -*-

"""Platooning Manager
"""

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import uuid
import weakref

import carla
import matplotlib.pyplot as plt
import numpy as np
import time
import opencda.logging_ecloud
import coloredlogs, logging
import sys

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

sys.path.append("/home/chattsgpu/Documents/Carla_opencda/TrafficSimulator_eCloud/OpenCDA/")

import opencda.core.plan.drive_profile_plotting as open_plt
from opencda.core.application.edge.astar_test_groupcaps_transform import *
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.application.edge.transform_utils import *

import grpc
import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as rpc

class EdgeManager(object):
    """
    Edge manager. Used to manage all vehicle managers under control of the edge

    Parameters
    ----------
    config_yaml : dict
        The configuration dictionary for edge.

    cav_world : opencda object
        CAV world that stores all CAV information.

    Attributes
    ----------
    pmid : int
        The  platooning manager ID.
    vehicle_manager_list : list
        A list of all vehciel managers within the platoon.
    destination : carla.location
        The destiantion of the current plan.
    """

    def __init__(self, config_yaml, cav_world):

        self.edgeid = str(uuid.uuid1())
        self.vehicle_manager_list = []
        self.target_speed = config_yaml['target_speed']
        self.numcars = 8
        # len(config_yaml['members'])
        logger.error("Num Cars: %d", (self.numcars))  # TODO - set edge_index
        #self.locations = []
        self.destination = None
        # Query the vehicle locations and velocities + target velocities
        self.spawn_x = []
        self.spawn_y = []
        self.spawn_v = [] # probably 0s but can be target vel too
        self.xcars = np.empty((int(self.numcars), 0))
        self.ycars = np.empty((int(self.numcars), 0))
        self.x_states = None
        self.y_states = None
        self.tv = None
        self.v = None
        self.target_velocities = np.empty((int(self.numcars), 0))
        self.velocities = np.empty((int(self.numcars),0))
        self.Traffic_Tracker = None
        self.waypoints_dict = {}
        self.cav_world = weakref.ref(cav_world)()
        self.ov, self.oy = generate_limits_grid()
        self.grid_size = 1.0
        self.robot_radius = 1.0
        self.processor = None
        self.secondary_offset=0

    def start_edge(self):
      self.get_four_lane_waypoints_dict()
      self.processor = transform_processor(self.waypoints_dict)
      _, _ = self.processor.process_waypoints_bidirectional(0)
      inverted = self.processor.process_forward(0)
      logger.debug(len(inverted))
      i = 0

      # for k in inverted:
      #     if k[0,0] <= 0 and k[0,0] < -self.secondary_offset:
      #       print("Current indice is: ", k[0,0])
      #       self.secondary_offset = -k[0,0]

      for vehicle_manager in self.vehicle_manager_list:
          spawn_coords = vehicle_manager.vehicle.get_location()
          spawn_coords = np.array([spawn_coords.x,spawn_coords.y]).reshape((2,1))
          # print(spawn_coords)
          spawn_coords = self.processor.process_single_waypoint_forward(spawn_coords[0,0],spawn_coords[1,0])
          # print(spawn_coords)
          # sys.exit()
          # self.spawn_x.append(vehicle_manager.vehicle.get_location().x)
          # self.spawn_y.append(vehicle_manager.vehicle.get_location().y)
          #self.spawn_v.append(vehicle_manager.vehicle.get_velocity())
          ## THIS IS TEMPORARY ##
          # print("inverted is: ", inverted[i][0,0])
          # print("revised x is: ", self.secondary_offset)
          self.spawn_x.append(spawn_coords[0]) # inverted[i][0,0]+self.secondary_offset)
          #self.spawn_v.append(5*(i+1))
          self.spawn_v.append(0)
          self.spawn_y.append(spawn_coords[1])# inverted[i][1,0])
          i += 1

          # TODO: DIST --> do we need to clear at start in containers?
          #vehicle_manager.agent.get_local_planner().get_waypoint_buffer().clear() # clear waypoint buffer at start
      self.dt = .200
      self.numlanes = 4
      self.Traffic_Tracker = Traffic(self.dt,self.numlanes,numcars=int(self.numcars),map_length=200,x_initial=self.spawn_x,y_initial=self.spawn_y,v_initial=self.spawn_v)

    def get_four_lane_waypoints_dict(self):
      world = self.vehicle_manager_list[0].vehicle.get_world()
      self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
      grp = GlobalRoutePlanner(self._dao)
      grp.setup()
      waypoints = world.get_map().generate_waypoints(10)

      indices_source = np.load('Indices_start.npy')
      indices_dest = np.load('Indices_dest.npy')

      indices_source = indices_source.astype(int)
      indices_dest = indices_dest.astype(int)

      #print("Source Shape: ", indices_source.shape)

      a = carla.Location(waypoints[indices_source[0,1]].transform.location)
      b = carla.Location(waypoints[indices_dest[0,1]].transform.location)
      c = carla.Location(waypoints[indices_source[1,1]].transform.location)
      d = carla.Location(waypoints[indices_dest[1,1]].transform.location)
      e = carla.Location(waypoints[indices_source[2,1]].transform.location)
      f = carla.Location(waypoints[indices_dest[2,1]].transform.location)
      g = carla.Location(waypoints[indices_source[3,1]].transform.location)
      j = carla.Location(waypoints[indices_dest[3,1]].transform.location)

      w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w2 = grp.trace_route(c, d) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w3 = grp.trace_route(e, f) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
      w4 = grp.trace_route(g, j) # there are other funcations can be used to generate a route in GlobalRoutePlanner.

      logger.debug(a)
      logger.debug(b)
      logger.debug(c)
      logger.debug(d)

      i = 0
      for w in w1:
        #print(w)
        mark=str(i)
        if i % 10 == 0:
            world.debug.draw_string(w[0].transform.location,mark, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0, persistent_lines=True)
        else:
            world.debug.draw_string(w[0].transform.location, mark, draw_shadow=False,
            color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
            persistent_lines=True)
        i += 1
      i = 0
      for w in w2:
        #print(w)
        mark=str(i)
        if i % 10 == 0:
            world.debug.draw_string(w[0].transform.location,mark, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0, persistent_lines=True)
        else:
            world.debug.draw_string(w[0].transform.location, mark, draw_shadow=False,
            color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
            persistent_lines=True)
        i += 1
      i = 0
      for w in w3:
        #print(w)
        mark=str(i)
        if i % 10 == 0:
            world.debug.draw_string(w[0].transform.location,mark, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0, persistent_lines=True)
        else:
            world.debug.draw_string(w[0].transform.location, mark, draw_shadow=False,
            color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
            persistent_lines=True)
        i += 1
      i = 0
      for w in w4:
        #print(w)
        mark=str(i)
        if i % 10 == 0:
            world.debug.draw_string(w[0].transform.location,mark, draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=120.0, persistent_lines=True)
        else:
            world.debug.draw_string(w[0].transform.location, mark, draw_shadow=False,
            color = carla.Color(r=0, g=0, b=255), life_time=1000.0,
            persistent_lines=True)
        i += 1
      # i = 0

      # while True:
      #   world.tick()

      self.waypoints_dict[1] = {}
      self.waypoints_dict[2] = {}
      self.waypoints_dict[3] = {}
      self.waypoints_dict[4] = {}
      self.waypoints_dict[1]['x'] = []
      self.waypoints_dict[2]['x'] = []
      self.waypoints_dict[3]['x'] = []
      self.waypoints_dict[4]['x'] = []
      self.waypoints_dict[1]['y'] = []
      self.waypoints_dict[2]['y'] = []
      self.waypoints_dict[3]['y'] = []
      self.waypoints_dict[4]['y'] = []


      for waypoint in w1:
        self.waypoints_dict[1]['x'].append(waypoint[0].transform.location.x)
        self.waypoints_dict[1]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w2:
        self.waypoints_dict[2]['x'].append(waypoint[0].transform.location.x)
        self.waypoints_dict[2]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w3:
        self.waypoints_dict[3]['x'].append(waypoint[0].transform.location.x)
        self.waypoints_dict[3]['y'].append(waypoint[0].transform.location.y)

      for waypoint in w4:
        self.waypoints_dict[4]['x'].append(waypoint[0].transform.location.x)
        self.waypoints_dict[4]['y'].append(waypoint[0].transform.location.y)



    def add_member(self, vehicle_manager, leader=False):
        """
        Add memeber to the current platooning

        Parameters
        __________
        leader : boolean
            Indicator of whether this cav is a leader.

        vehicle_manager : opencda object
            The vehicle manager class.
        """
        self.vehicle_manager_list.append(vehicle_manager)

    def get_route_waypoints(self, destination):
        self.start_waypoint = self._map.get_waypoint(start_location)

        # make sure the start waypoint is behind the vehicle
        if self._ego_pos:
            cur_loc = self._ego_pos.location
            cur_yaw = self._ego_pos.rotation.yaw
            _, angle = cal_distance_angle(
                self.start_waypoint.transform.location, cur_loc, cur_yaw)

            while angle > 90:
                self.start_waypoint = self.start_waypoint.next(1)[0]
                _, angle = cal_distance_angle(
                    self.start_waypoint.transform.location, cur_loc, cur_yaw)

        end_waypoint = self._map.get_waypoint(end_location)
        if end_reset:
            self.end_waypoint = end_waypoint

        route_trace = self._trace_route(self.start_waypoint, end_waypoint)


    def set_destination(self, destination):
        """
        Set destination of the vehicle managers in the platoon.
        """
        self.destination = destination
        # for i in range(len(self.vehicle_manager_list)):
        #     self.vehicle_manager_list[i].set_destination(
        #         self.vehicle_manager_list[i].vehicle.get_location(),
        #         destination, clean=True)

    def update_information(self):
        """
        Update CAV world information for every member in the list.
        """

        self.spawn_x.clear()
        self.spawn_y.clear()
        self.spawn_v.clear()
        start_time = time.time()
        for i in range(len(self.vehicle_manager_list)):
            self.vehicle_manager_list[i].update_info()
            print(f"Updated location for vehicle {i} - x:{self.vehicle_manager_list[i].vehicle.get_location().x}, y:{self.vehicle_manager_list[i].vehicle.get_location().y}")
        end_time = time.time()
        logger.debug("Vehicle Manager Update Info Time: %s", (end_time - start_time))
        start_time = time.time()
        for i in range(len(self.vehicle_manager_list)):
            x,y = self.processor.process_single_waypoint_forward(self.vehicle_manager_list[i].vehicle.get_location().x, self.vehicle_manager_list[i].vehicle.get_location().y)
            v = self.vehicle_manager_list[i].vehicle.get_velocity()
            v_scalar = math.sqrt(v.x**2 + v.y**2 + v.z**2)
            self.spawn_x.append(x)
            self.spawn_y.append(y)
            self.spawn_v.append(v_scalar)
            print(f"update_information for vehicle_{i} - x:{x}, y:{y}")
        end_time = time.time()
        logger.debug("Update Info Transform Forward Time: %s", (end_time - start_time))
        #print(self.spawn_x)
        #print(self.spawn_y)
        #print(self.spawn_v)

        start_time = time.time()
        #Added in to check if traffic tracker updating would fix waypoint deque issue
        # TODO: data drive num cars
        self.Traffic_Tracker = Traffic(self.dt,self.numlanes,numcars=int(self.numcars),map_length=200,x_initial=self.spawn_x,y_initial=self.spawn_y,v_initial=self.spawn_v)
        end_time = time.time()
        logger.debug("Traffic Tracker Time: %s", (end_time - start_time))

        for car in self.Traffic_Tracker.cars_on_road:
            car.target_velocity = 15
        # sys.exit()

        #print("Updated Info")

    def algorithm_step(self):
        self.locations = []
        #print("started Algo step")

        #DEBUGGING: Bypass algo and simply move cars forward to solve synch and transform issues
        #Bypassed as of 14/3/2022

        slice_list, vel_array, lanechange_command = get_slices_clustered(self.Traffic_Tracker, int(self.numcars))

        for i in range(len(slice_list)-1,-1,-1): #Iterate through all slices
            if len(slice_list[i]) >= 2: #If the slice has more than one vehicle, run the graph planner. Else it'll move using existing
            #responses - slow down on seeing a vehicle ahead that has slower velocities, else hit target velocity.
            #Somewhat suboptimal, ideally the other vehicle would be
            #folded into existing groups. No easy way to do that yet.
                #print("Slicing")
                a_star = AStarPlanner(slice_list[i], self.ov, self.oy, self.grid_size, self.robot_radius, self.Traffic_Tracker.cars_on_road, i)
                rv, ry, rx_tracked = a_star.planning()
                if len(ry) >= 2: #If there is some planner result, then we move ahead on using it
                    lanechange_command[i] = ry[-2]
                    vel_array[i] = rv[-2]
                else: #If the planner returns an empty list, continue as before - use emergency responses.
                    lanechange_command[i] = ry[0]
                    vel_array[i] = ry[0]

        #print("Sliced")
        for i in range(len(slice_list)-1,-1,-1): #Relay lane change commands and new velocities to vehicles where needed
            if len(slice_list[i]) >= 1 and len(lanechange_command[i]) >= 1:
                carnum = 0
                for car in slice_list[i]:
                    if lanechange_command[i][carnum] > car.lane:
                        car.intentions = "Lane Change 1"
                    elif lanechange_command[i][carnum] < car.lane:
                        car.intentions = "Lane Change -1"
                    car.v = vel_array[i][carnum]
                    carnum += 1

        self.Traffic_Tracker.time_tick(mode='Graph') #Tick the simulation

        #print("Success capsule")

        #Recording location and state
        x_states, y_states, tv, v = self.Traffic_Tracker.ret_car_locations() # Commented out for bypassing algo
        # x_states, y_states, v = [], [], [] #Algo bypass begins
        self.xcars = np.empty((int(self.numcars), 0))
        self.ycars = np.empty((int(self.numcars), 0))

        # for i in range(0,4):
        #     x_states.append([self.Traffic_Tracker.cars_on_road[i].pos_x+4])
        #     y_states.append([self.Traffic_Tracker.cars_on_road[i].lane])
        #     v.append([self.Traffic_Tracker.cars_on_road[i].v])
        # x_states = np.array(x_states).reshape((4,1))
        # y_states = np.array(y_states).reshape((4,1))
        # v = np.array(v).reshape((4,1)) #Algo bypass ends

        ###Begin waypoint transform process, algo ended###
        self.xcars = np.hstack((self.xcars, x_states))
        self.ycars = np.hstack((self.ycars, y_states))
        self.target_velocities = np.hstack((self.target_velocities,tv)) #Commented out for bypassing algo, comment back in if algo present
        self.velocities = np.hstack((self.velocities,v)) #Was just v, v_states for the skipping-planner debugging

        #print("Returned X: ", self.xcars)
        #print("Returned Y: ", self.ycars)

        self.xcars = self.xcars - self.secondary_offset

        #print("Revised Returned X: ", self.xcars)

        ###########################################

        # waypoints_rev = {1 : np.empty((2,0)), 2 : np.empty((2,0)), 3 : np.empty((2,0)), 4 : np.empty((2,0)), 5 : np.empty((2,0)), 6 : np.empty((2,0)), 7 : np.empty((2,0)), 8 : np.empty((2,0))}
        # for i in range(0,self.xcars.shape[1]):
        #   processed_array = []
        #   for j in range(0,8):
        #     x_res = self.xcars[j,i]
        #     y_res = self.ycars[j,i]
        #     processed_array.append(np.array([[x_res],[y_res]]))
        #     print("Appending to waypoints_rev")
        #   back = self.processor.process_back(processed_array)
        #   waypoints_rev[1] = np.hstack((waypoints_rev[1],back[0]))
        #   waypoints_rev[2] = np.hstack((waypoints_rev[2],back[1]))
        #   waypoints_rev[3] = np.hstack((waypoints_rev[3],back[2]))
        #   waypoints_rev[4] = np.hstack((waypoints_rev[4],back[3]))
        #   waypoints_rev[5] = np.hstack((waypoints_rev[5],back[4]))
        #   waypoints_rev[6] = np.hstack((waypoints_rev[6],back[5]))
        #   waypoints_rev[7] = np.hstack((waypoints_rev[7],back[6]))
        #   waypoints_rev[8] = np.hstack((waypoints_rev[8],back[7]))

        waypoints_rev = {}
        car_locations = {}
        for cars in range(1,self.numcars+1):
            waypoints_rev[str(cars)] = np.empty((2,0))
            car_locations[str(cars)] = []

        for i in range(0,self.xcars.shape[1]):
          processed_array = []
          for j in range(0,self.numcars):
            x_res = self.xcars[j,i]
            y_res = self.ycars[j,i]
            processed_array.append(np.array([[x_res],[y_res]]))
            #print("Appending to waypoints_rev")
          #print(processed_array)
          back = self.processor.process_back(processed_array)

          #print(waypoints_rev)
          #print(waypoints_rev.keys())
          for j in range(0,self.numcars):
            #print(len(back))
            #print(j)
            # print(waypoints_rev[str(j+1)])
            # print(back[str(j)])
            # print(np.hstack((waypoints_rev[str(j+1)],back[str(j)])))
            waypoints_rev[str(j+1)] = np.hstack((waypoints_rev[str(j+1)],back[j]))

        # processed_array = []
        # for k in range(0,4): #Added 16/03 outer loop to check if waypoint horizon influenced things, it did not seem to.
        #     for j in range(0,8):
        #         x_res = self.xcars[j,-1]
        #         y_res = self.ycars[j,-1]
        #         processed_array.append(np.array([[x_res],[y_res]]))
        #         self.xcars[j,-1] += 4 #Increment by +4, just adding another waypoint '4m' ahead of this one, until horizon 3 steps ahead
        #     print("Appending to waypoints_rev: ", self.xcars)
        #     back = self.processor.process_back(processed_array)
        #     waypoints_rev[1] = np.hstack((waypoints_rev[1],back[0]))
        #     waypoints_rev[2] = np.hstack((waypoints_rev[2],back[1]))
        #     waypoints_rev[3] = np.hstack((waypoints_rev[3],back[2]))
        #     waypoints_rev[4] = np.hstack((waypoints_rev[4],back[3]))

        #print(waypoints_rev)
        # car_locations = {1 : [], 2 : [], 3 : [], 4 : [], 5 : [], 6 : [], 7 : [], 8 : []}

        for car, car_array in waypoints_rev.items():
          for i in range(0,len(car_array[0])):
            location = self._dao.get_waypoint(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0))
            print(f"algorith_step: vehicle_{i} location - {location}")
            self.locations.append(location)

        #print("Locations appended: ", self.locations)

    def run_step(self):
        """
        Run one control step for each vehicles.

        Returns
        -------
        control_list : list
            The control command list for all vehicles.
        """

        # TODO: make a dist version...

        # run algorithm
        pre_algo_time = time.time()
        logger.error("Num cars: %s", (self.numcars))
        self.algorithm_step()
        logger.error("Num cars: %s", (self.numcars))
        post_algo_time = time.time()
        logger.debug("Algorithm completion time: %s", (post_algo_time - pre_algo_time))
        all_waypoint_buffers = []
        #print("completed Algorithm Step")
        # output algorithm waypoints to waypoint buffer of each vehicle
        for idx, vehicle_manager in enumerate(self.vehicle_manager_list):
        #   # print(i)
        #   waypoint_buffer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
        #   # print(waypoint_buffer)
        #   # for waypoints in waypoint_buffer:
        #   #   print("Waypoints transform for Vehicle Before Clearing: " + str(i) + " : ", waypoints[0].transform)
        #   waypoint_buffer.clear() #EDIT MADE 16/03
            waypoint_buffer_proto = ecloud.WaypointBuffer()
            waypoint_buffer_proto.vehicle_index = idx

            for k in range(0,1):
                waypoint_buffer_proto.waypoint_buffer.extend([serialize_waypoint(self.locations[idx*1+k])])#, RoadOption.STRAIGHT)) #Accounting for horizon of 4 here. To generate a waypoint _buffer_

            #logger.debug(waypoint_buffer_proto.SerializeToString())

            all_waypoint_buffers.append(waypoint_buffer_proto)
          # for waypoints in waypoint_buffer:
          #   print("Waypoints transform for Vehicle After Clearing: " + str(i) + " : ", waypoints[0].transform)
          # sys.exit()
          # # print(waypoint_buffer)

        return all_waypoint_buffers

        # #print("\n ########################\n")
        # #print("Length of vehicle manager list: ", len(self.vehicle_manager_list))

        # control_list = []
        # for i in range(len(self.vehicle_manager_list)):
        #     waypoints_buffer_printer = self.vehicle_manager_list[i].agent.get_local_planner().get_waypoint_buffer()
        #     #for waypoints in waypoints_buffer_printer:
        #         #print("Waypoints transform for Vehicle: " + str(i) + " : ", waypoints[0].transform)
        #     # print(self.vehicle_manager_list[i].agent.get_local_planner().get_waypoint_buffer().transform())
        #     control = self.vehicle_manager_list[i].run_step(self.target_speed)
        #     control_list.append(control)

        # for (i, control) in enumerate(control_list):
        #     self.vehicle_manager_list[i].vehicle.apply_control(control)

        # return control_list

    def evaluate(self):
        """
        Used to save all members' statistics.

        Returns
        -------
        figure : matplotlib.figure
            The figure drawing performance curve passed back to save to
            the disk.

        perform_txt : str
            The string that contains all evaluation results to print out.
        """

        velocity_list = []
        acceleration_list = []
        time_gap_list = []
        distance_gap_list = []

        perform_txt = ''

        for i in range(len(self.vehicle_manager_list)):
            vm = self.vehicle_manager_list[i]
            debug_helper = vm.agent.debug_helper

            # we need to filter out the first 100 data points
            # since the vehicles spawn at the beginning have
            # no velocity and thus make the time gap close to infinite

            velocity_list += debug_helper.speed_list
            acceleration_list += debug_helper.acc_list
            time_gap_list += debug_helper.time_gap_list
            distance_gap_list += debug_helper.dist_gap_list

            time_gap_list_tmp = \
                np.array(debug_helper.time_gap_list)
            time_gap_list_tmp = \
                time_gap_list_tmp[time_gap_list_tmp < 100]
            distance_gap_list_tmp = \
                np.array(debug_helper.dist_gap_list)
            distance_gap_list_tmp = \
                distance_gap_list_tmp[distance_gap_list_tmp < 100]

            perform_txt += '\n Platoon member ID:%d, Actor ID:%d : \n' % (
                i, vm.vehicle.id)
            perform_txt += 'Time gap mean: %f, std: %f \n' % (
                np.mean(time_gap_list_tmp), np.std(time_gap_list_tmp))
            perform_txt += 'Distance gap mean: %f, std: %f \n' % (
                np.mean(distance_gap_list_tmp), np.std(distance_gap_list_tmp))

        figure = plt.figure()

        plt.subplot(411)
        open_plt.draw_velocity_profile_single_plot(velocity_list)

        plt.subplot(412)
        open_plt.draw_acceleration_profile_single_plot(acceleration_list)

        plt.subplot(413)
        open_plt.draw_time_gap_profile_singel_plot(time_gap_list)

        plt.subplot(414)
        open_plt.draw_dist_gap_profile_singel_plot(distance_gap_list)

        label = []
        for i in range(1, len(velocity_list) + 1):
            label.append('Leading Vehicle, id: %d' %
                         int(i - 1) if i == 1 else 'Platoon member, id: %d' %
                         int(i - 1))

        figure.legend(label, loc='upper right')

        return figure, perform_txt

    def destroy(self):
        """
        Destroy edge vehicles actors inside simulation world.
        """
        for vm in self.vehicle_manager_list:
            vm.destroy()
