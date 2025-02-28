import math
import logging

import matplotlib.pyplot as plt

from opencda.core.application.edge.collab_sandbox import Traffic
from opencda.core.application.edge.collab_sandbox import Car

import numpy as np
import itertools

import scipy.io
import matplotlib.pyplot as plt
import matplotlib
import time
import timeit
from sklearn.cluster import AgglomerativeClustering
# from pypapi import events, papi_high as high

from k_means_constrained import KMeansConstrained

from opencda.core.application.edge.transform_utils import transform_processor

import pickle
import carla
from opencda.core.application.edge.tools.carla_data_provider import *
show_animation = True

logger = logging.getLogger(__name__)

#class AStarPlannerClient:
#
#    def __init__(self):
#      self.client = carla.Client('localhost', 2000)
#      self.client.set_timeout(4.0)
#      self.world = self.client.load_world('Town06')
#      self.algorithm_latency = []
#
#    def get_actors(self):
#      return self._world.get_actors()
#
#    def get_vehicles(self):
#      return self.world.get_actors().filter("*vehicle*")
#
#    def get_waypoints(self):
#      return self.world.get_map().generate_waypoints(10)
#
#    def get_four_lane_waypoints(self):
#      dao = GlobalRoutePlannerDAO(self.world.get_map(), 2)
#      grp = GlobalRoutePlanner(dao)
#      grp.setup()
#      waypoints = self.world.get_map().generate_waypoints(10)
#      a = carla.Location(waypoints[343].transform.location)
#      b = carla.Location(waypoints[1116].transform.location)
#      c = carla.Location(waypoints[344].transform.location)
#      d = carla.Location(waypoints[1117].transform.location)
#      e = carla.Location(waypoints[345].transform.location)
#      f = carla.Location(waypoints[1118].transform.location)
#      g = carla.Location(waypoints[346].transform.location)
#      j = carla.Location(waypoints[1119].transform.location)
#
#      w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w2 = grp.trace_route(c, d) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w3 = grp.trace_route(e, f) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w4 = grp.trace_route(g, j) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      waypoints = w1+w2+w3+w4
#      return waypoints
#
#    def get_four_lane_waypoints_dict(self):
#      dao = GlobalRoutePlannerDAO(self.world.get_map(), 2)
#      grp = GlobalRoutePlanner(dao)
#      grp.setup()
#      waypoints_dict = {}
#      waypoints = self.world.get_map().generate_waypoints(10)
#      a = carla.Location(waypoints[343].transform.location)
#      b = carla.Location(waypoints[1116].transform.location)
#      c = carla.Location(waypoints[344].transform.location)
#      d = carla.Location(waypoints[1117].transform.location)
#      e = carla.Location(waypoints[345].transform.location)
#      f = carla.Location(waypoints[1118].transform.location)
#      g = carla.Location(waypoints[346].transform.location)
#      j = carla.Location(waypoints[1119].transform.location)
#
#      w1 = grp.trace_route(a, b) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w2 = grp.trace_route(c, d) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w3 = grp.trace_route(e, f) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      w4 = grp.trace_route(g, j) # there are other funcations can be used to generate a route in GlobalRoutePlanner.
#      waypoints_dict[1] = {}
#      waypoints_dict[2] = {}
#      waypoints_dict[3] = {}
#      waypoints_dict[4] = {}
#      waypoints_dict[1]['x'] = []
#      waypoints_dict[2]['x'] = []
#      waypoints_dict[3]['x'] = []
#      waypoints_dict[4]['x'] = []
#      waypoints_dict[1]['y'] = []
#      waypoints_dict[2]['y'] = []
#      waypoints_dict[3]['y'] = []
#      waypoints_dict[4]['y'] = []
#      for waypoint in w1:
#        waypoints_dict[1]['x'].append(waypoint[0].transform.location.x)
#        waypoints_dict[1]['y'].append(waypoint[0].transform.location.y)
#
#      for waypoint in w2:
#        waypoints_dict[2]['x'].append(waypoint[0].transform.location.x)
#        waypoints_dict[2]['y'].append(waypoint[0].transform.location.y)
#
#      for waypoint in w3:
#        waypoints_dict[3]['x'].append(waypoint[0].transform.location.x)
#        waypoints_dict[3]['y'].append(waypoint[0].transform.location.y)
#
#      for waypoint in w4:
#        waypoints_dict[4]['x'].append(waypoint[0].transform.location.x)
#        waypoints_dict[4]['y'].append(waypoint[0].transform.location.y)
#
#
#      return waypoints_dict
#
#
#    def set_vehicle_position(vehicle_id, location):
#      actor = self.world.get_actors().filter(id)
#      actor.set_location(location)
#
class AStarPlanner:

    def __init__(self, cars, ov, oy, resolution, rr=1, cars_on_road=None, slicenum=0):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        v = []
        x_start = []
        vt = []
        y = []

        for i in cars:
            v.append(i.v)
            vt.append(i.target_velocity)
            x_start.append(i.pos_x)
            y.append(i.lane)

        self.v = np.array(v)
        self.vt = np.array(vt)
        self.x_start = np.array(x_start)
        self.y = np.array(y)

        self.resolution = resolution
        self.rr = rr
        self.min_v, self.min_y = 0, 0
        self.max_v, self.max_y = 0, 0
        self.obstacle_map = None
        self.v_width, self.y_width = 0, 0
        self.motion_v, self.motion_y = self.get_motion_model(len(cars))
        self.calc_obstacle_map(ov, oy)

        self.cars_on_road = cars_on_road
        self.slicenum=slicenum

    class Node:
        def __init__(self, sv, sy, x_start, vt, cost, parent_index):

            self.v = np.array(sv)  # index of grid
            self.y = np.array(sy)  # index of grid
            self.x_start = np.array(x_start)
            self.x_tracked = None
            self.vt = np.array(vt)

            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.v) + "," + str(self.y) + "," + str(
                self.x_tracked) + "," + str(self.cost)

        def length_of_path(self,node_set,pathlen=0):
            if self.parent_index == -1:
                pathlen += 1
                return pathlen
            else:
                pathlen += 1
                return node_set[self.parent_index].length_of_path(node_set=node_set,pathlen=pathlen)

    def planning(self):
        """
        A star path search
        input:
            s_v: start v position [m/s]
            s_y: start y position [m]
            gv: goal v position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        sv = self.v
        sy = self.y
        gy = self.y
        gv = self.vt

        start_node = self.Node(self.calc_xy_index(sv, self.min_v),
                               self.calc_xy_index(sy, self.min_y), self.x_start, self.vt, 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gv, self.min_v),
                              self.calc_xy_index(gy, self.min_y), self.x_start, self.vt, 0.0, -1)

        start_node.x_tracked = self.x_start

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        empty_flag = 0

        while 1:
            if len(open_set) == 0:
                logger.warning("Open set is empty..")
                empty_flag = 1
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.v = current.v
                goal_node.y = current.y
                goal_node.x_tracked = current.x_tracked
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(open_set[
                                                                         o],goal_node))
            current = open_set[c_id]

            # show graph
            # if show_animation:  # pragma: no cover
            #     plt.plot(self.calc_grid_position(current.x, self.min_x),
            #              self.calc_grid_position(current.y, self.min_y), "xc")
            #     # for stopping simulation with the esc key.
            #     plt.gcf().canvas.mpl_connect('key_release_event',
            #                                  lambda event: [exit(
            #                                      0) if event.key == 'escape' else None])
            #     if len(closed_set.keys()) % 10 == 0:
            #         plt.pause(0.001)

            if (current.length_of_path(node_set=closed_set) >= 4): #Was 4 #current.x == goal_node.x and current.y == goal_node.y:
                logger.warning("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.v = current.v
                goal_node.y = current.y
                goal_node.x_tracked = current.x_tracked
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            # for i, _ in enumerate(self.motion):
            for i in range(0,len(self.motion_v)):
                for j in range(0,len(self.motion_y)):
                    node = self.Node(current.v + np.array(self.motion_v[i]),
                                     current.y + np.array(self.motion_y[j]), self.x_start, self.vt,
                                     current.cost, c_id)
                    node.cost = self.calc_heuristic(node,current) #+ np.sum(abs(closed_set[c_id].y - node.y))
                    node.x_tracked = current.x_tracked + (node.v * 0.2)
                    node.x_tracked = node.x_tracked.astype(int)

                    n_id = self.calc_grid_index(node)

                    # If the node is not safe, do nothing
                    if not self.verify_node(node,current):
                        if node.y.any() > 1:
                            logger.warning("Node Not Viable: ", node.__str__())
                        continue

                    if n_id in closed_set:
                        continue

                    if n_id not in open_set:
                        open_set[n_id] = node  # discovered a new node
                        # print(node.__str__())
                        #print(np.sum(abs(node.v - node.vt)))
                    else:
                        if open_set[n_id].cost > node.cost:
                            # This path is the best until now. record it
                            open_set[n_id] = node

        rv, ry, rx = self.calc_final_path(goal_node, closed_set)

        return rv, ry, rx

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rv, ry = [self.calc_grid_position(goal_node.v, self.min_v)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        rx = []
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rv.append(self.calc_grid_position(n.v, self.min_v))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            rx.append(n.x_tracked)
            parent_index = n.parent_index

        return rv, ry, rx

    @staticmethod
    def calc_heuristic(n1,n2=0):
        w = 10.0  # weight of heuristic
        w_lane = 0.5 # weight of lane changes (for now), was 0.5 earlier.
        d = 0
        for i in range(0,len(n1.v)):
            d += w * abs(n1.v[i] - n1.vt[i]) + w_lane*abs(n1.y[i]-n2.y[i])# math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution # + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        for i in range(0,len(position)):
            position[i] = round((position[i]) / self.resolution)
        return position

    def calc_grid_index(self, node):
        coord_tracked = []
        for i in range(0,len(node.x_tracked)):
            coord_tracked.append(node.x_tracked[i])
            coord_tracked.append(node.y[i])
        str1 = ''.join(str(e) for e in coord_tracked)
        return hash(str1)

    def verify_node(self, node, current=None):

        for i in range(0,len(node.v)): #Check to see if within lane and velocity limits
            px = self.calc_grid_position(node.v[i], self.min_v)
            py = self.calc_grid_position(node.y[i], self.min_y)

            if px < self.min_v:
                return False
            elif py < self.min_y:
                return False
            elif px >= self.max_v:
                return False
            elif py >= self.max_y:
                return False

        #collision check: For all pairs in slice, check collisions
        for i in range(0,len(node.v)):
            for j in range(i+1,len(node.v)):
                if node.y[i] == node.y[j] and abs(node.x_tracked[j]-node.x_tracked[i]) <= 10:
                    # print("False for constraint")
                    return False
        #Added to prevent easy swapping of lanes in a single iteration, requires sufficient clearance between vehicles now: Added on 05/05/22
        if current is not None:
            for i in range(0,len(node.v)):
                for j in range(i+1,len(node.v)):
                    if node.y[i] == current.y[j] and abs(node.x_tracked[j]-node.x_tracked[i]) <= 10:
                        return False

        # collision check: Other cars: For every vehicle in slice (i in range), check all other non slice vehicles (j + j.slice condition)
        if self.cars_on_road is not None:
            for i in range(0,len(node.v)):
                for j in self.cars_on_road:
                    if j.slice != self.slicenum:
                        if node.y[i] == j.lane and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False
                        if (j.intentions == "Lane Change -1" and node.y[i] == j.lane-1) and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False
                        if (j.intentions == "Lane Change 1" and node.y[i] == j.lane+1) and abs(node.x_tracked[i]-j.pos_x) <= 10:
                            return False

        return True

    def calc_obstacle_map(self, ov, oy):

        self.min_v = round(min(ov))
        self.min_y = round(min(oy))
        self.max_v = round(max(ov))
        self.max_y = round(max(oy))
        # print("min_x:", self.min_v)
        # print("min_y:", self.min_y)
        # print("max_x:", self.max_v)
        # print("max_y:", self.max_y)

        self.v_width = round((self.max_v - self.min_v) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.v_width)]
        for iv in range(self.v_width):
            v = self.calc_grid_position(iv, self.min_v)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iov, ioy in zip(ov, oy):
                    d = math.hypot(iov - v, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[iv][iy] = True
                        break

    @staticmethod
    def get_motion_model(numcars):

        motion_v_atomic = [-1,0,1] #np.arange(-1,1,step=1)
        motion_y_atomic = [-1,0,1]
        motion_y = []
        motion_v = []
        for element in itertools.product(motion_y_atomic, motion_y_atomic):
            motion_y.append(element)
        for element in itertools.product(motion_v_atomic, motion_v_atomic):
            motion_v.append(element)

        motion_y_final = motion_y_atomic
        motion_v_final = motion_v_atomic

        for i in range(0,numcars-1):
            motion_y_interim = []
            for j in motion_y_final:
                for k in motion_y_atomic:
                    motion_y_interim.append(np.append(j,k))
            motion_y_final = motion_y_interim

        for i in range(0,numcars-1):
            motion_v_interim = []
            for j in motion_v_final:
                for k in motion_v_atomic:
                    motion_v_interim.append(np.append(j,k))
            motion_v_final = motion_v_interim

        return motion_v_final, motion_y_final

def get_states_carlist(car_list):
    carnum = 0
    for i in car_list:
        logger.info("X Coordinate of ", carnum, " Is: ", i.pos_x)
        logger.info("Y Coordinate of ", carnum, " Is: ", i.lane)
        logger.info("Velocity of ", carnum, " Is: ", i.v)
        carnum += 1

def get_slice_plans(Traffic_Tracker, ov, oy, slice_length=15, map_length=1000):

    slice_list = []
    lanechange_command = []
    vel_array = []

    for i in range(0,int(map_length/slice_length) + 1):
        slice_list.append([])
        lanechange_command.append([])
        vel_array.append([])

    for i in Traffic_Tracker.cars_on_road:
        i.slice = int(i.pos_x/slice_length)
        slice_list[i.slice].append(i)

    for i in range(int(map_length/slice_length)-1,-1,-1):
        if len(slice_list[i]) >= 2:
            a_star = AStarPlanner(slice_list[i], ov, oy, 1, 1.0, Traffic_Tracker.cars_on_road, i)
            rv, ry, rx_tracked = a_star.planning()

            lanechange_command[i] = ry[-2]
            vel_array[i] = rv[-2]

    return slice_list, lanechange_command, vel_array

def find_nearest(array, value):
    array = np.asarray(array)
    idx = np.nanargmin((np.abs(array - value)))
    return array[idx], idx

def save_obj(obj, name ):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)

def generate_limits_grid(v_min=0.0, v_max=25.0, lane_num=4.0): #Used to generate the grid 'box' initially
    #Lanes assumed from one to max lane - the maximum is unreachable.
    ov, oy = [], [] #Limits for velocity (ov) and lane (y), alter for different lane limits/velocity limits
    for i in range(int(v_min), int(v_max)):
        ov.append(i)
        oy.append(v_min)
    for i in range(0, int(lane_num)):
        ov.append(v_max)
        oy.append(i)
    for i in range(int(v_min), int(v_max)+1):
        ov.append(i)
        oy.append(lane_num)
    for i in range(0, int(lane_num)+1):
        ov.append(v_min)
        oy.append(i)

    return ov, oy

def get_slices_clustered(Traffic_Tracker,numcars):
    slice_list = []
    vel_array = []
    lanechange_command = []

    carlist_posx = []
    position_features = np.empty((0,2))
    lane_weight = 1

    for i in Traffic_Tracker.cars_on_road:
        carlist_posx.append(i.pos_x)
        position_features = np.vstack((position_features,np.array([i.pos_x,lane_weight*i.lane]).reshape((1,2))))
        # print("Velocity: ", i.v)
        # print("Target Velocity: ", i.target_velocity)
        # print("Lane: ", i.lane)
    argsort_indices = np.argsort(np.array(carlist_posx))
    argsort_indices = argsort_indices.tolist()

    cluster_list = KMeansConstrained(n_clusters=int(numcars/2 + 1), size_min=None, size_max=2,random_state=0).fit_predict(position_features)
    group_number = np.amax(cluster_list)+1

    for i in range(0,group_number):
        slice_list.append([])
        lanechange_command.append([])
        vel_array.append([])

    count = 0
    for i in range(0,numcars): #Assign clusters/slice IDs to vehicles. Assign vehicles to the slice list.
        Traffic_Tracker.cars_on_road[i].slice = cluster_list[i]
        slice_list[Traffic_Tracker.cars_on_road[i].slice].append(Traffic_Tracker.cars_on_road[i])
        count += 1
    # print("Slices", slice_list)
    return slice_list, vel_array, lanechange_command


#def main(): #Example scenario test for CARLA waypoints
#    print(__file__ + " start!!")
#
#    grid_size = 1.0  # [m]
#    robot_radius = 1.0  # [m]
#    dt = 0.2
#    numlanes = 4
#    numcars = 4
#
#    ov, oy = generate_limits_grid()
#
#    #Turning test waypoints into a carlist, input reformatting is needed - discussion needed.
#    #Velocities are required for a car list, which is why this test code remains - to make up some and make a list of Car() objects for input
#
#    waypoints = load_obj('waypoints')
#    print(waypoints.keys())
#
#    processor = transform_processor(waypoints)
#    _, _ = processor.process_waypoints_bidirectional(0)
#    inverted = processor.process_forward(0)
#    spawn_x = []
#    spawn_y = []
#    spawn_v = []
#    for i in range(0,4):
#        spawn_x.append(inverted[i][0,0])
#        spawn_v.append(5*(i+1))
#        spawn_y.append(inverted[i][1,0])
#
#    Traffic_Tracker = Traffic(dt,numlanes,numcars=4,map_length=200,x_initial=spawn_x,y_initial=spawn_y,v_initial=spawn_v)
#
#    #Recording states
#    xcars = np.empty((numcars,0))
#    ycars = np.empty((numcars,0))
#    target_velocities = np.empty((numcars,0))
#    velocities = np.empty((numcars,0))
#    x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
#    x_initial, y_initial, v_initial = x_states, y_states, v
#
#    xcars = np.hstack((xcars, x_states))
#    ycars = np.hstack((ycars, y_states))
#    target_velocities = np.hstack((target_velocities,tv))
#    velocities = np.hstack((velocities,v))
#
#    #Set control flags and iteration count
#    lane_change_flag = 0
#    itercount = 0
#    maxtime=10
#
#    while Traffic_Tracker.time < maxtime:
#
#        slice_list, vel_array, lanechange_command = get_slices_clustered(Traffic_Tracker, numcars)
#
#        for i in range(len(slice_list)-1,-1,-1): #Iterate through all slices
#            if len(slice_list[i]) >= 2: #If the slice has more than one vehicle, run the graph planner. Else it'll move using existing
#            #responses - slow down on seeing a vehicle ahead that has slower velocities, else hit target velocity.
#            #Somewhat suboptimal, ideally the other vehicle would be
#            #folded into existing groups. No easy way to do that yet.
#                a_star = AStarPlanner(slice_list[i], ov, oy, grid_size, robot_radius, Traffic_Tracker.cars_on_road, i)
#                rv, ry, rx_tracked = a_star.planning()
#                if len(ry) >= 2: #If there is some planner result, then we move ahead on using it
#                    lanechange_command[i] = ry[-2]
#                    vel_array[i] = rv[-2]
#                else: #If the planner returns an empty list, continue as before - use emergency responses.
#                    lanechange_command[i] = ry[0]
#                    vel_array[i] = ry[0]
#
#        for i in range(len(slice_list)-1,-1,-1): #Relay lane change commands and new velocities to vehicles where needed
#            if len(slice_list[i]) >= 1 and len(lanechange_command[i]) >= 1:
#                carnum = 0
#                for car in slice_list[i]:
#                    if lanechange_command[i][carnum] > car.lane:
#                        car.intentions = "Lane Change 1"
#                    elif lanechange_command[i][carnum] < car.lane:
#                        car.intentions = "Lane Change -1"
#                    car.v = vel_array[i][carnum]
#                    carnum += 1
#
#        Traffic_Tracker.time_tick(mode='Graph') #Tick the simulation
#
#        print("Success capsule")
#
#        #Recording location and state
#        x_states, y_states, tv, v = Traffic_Tracker.ret_car_locations()
#        xcars = np.hstack((xcars, x_states))
#        ycars = np.hstack((ycars, y_states))
#        target_velocities = np.hstack((target_velocities,tv))
#        velocities = np.hstack((velocities,v))
#        ###########################################
#
#        #Check for collisions in simulation, triggering termination early - that's a failstate
#        collisions_rec = Traffic_Tracker.check_collisions()
#        if np.sum(collisions_rec) > 0:
#            print("Collided: ", collisions_rec)
#            print("Time: ", Traffic_Tracker.time)
#            Traffic_Tracker.printstates()
#            sys.exit()
#
#        # plot_name = '/home/chandramouli/Documents/collab-sandbox/comparison_three_test/states_optimal_allocation/' + 'snapshot' + str(Traffic_Tracker.time/0.2) + '.jpg'
#
#        plt.cla()
#        # for i in range(0,Traffic_Tracker.numcars):
#        #   # plt.plot(xcars[i,:],ycars[i,:])
#        #   plt.scatter(x_states,y_states,marker='o',s=20)
#            # print(plot_name)
#        plt.scatter(x_states,y_states,marker='o',s=20)
#        plt.xlim([0,1000])
#        # plt.savefig(plot_name)
#        plt.pause(0.001)
#
#    print("Terminated Successfully")
#
#    #Back conversion of waypoints. Currently somewhat hardcoded, will need revision as we iron out I/O format.
#    #Specifically requires the format for sending to CARLA.
#    print(xcars)
#    print(ycars)
#    print(velocities)
#    waypoints_rev = {1 : np.empty((2,0)), 2 : np.empty((2,0)), 3 : np.empty((2,0)), 4 : np.empty((2,0))}
#    for i in range(1,xcars.shape[1]):
#        processed_array = []
#        for j in range(0,numcars):
#            x_res = xcars[j,i]
#            y_res = ycars[j,i]
#            processed_array.append(np.array([[x_res],[y_res]]))
#        back = processor.process_back(processed_array)
#        waypoints_rev[1] = np.hstack((waypoints_rev[1],back[0]))
#        waypoints_rev[2] = np.hstack((waypoints_rev[2],back[1]))
#        waypoints_rev[3] = np.hstack((waypoints_rev[3],back[2]))
#        waypoints_rev[4] = np.hstack((waypoints_rev[4],back[3]))
#
#    print(waypoints_rev)
#    car_locations = {1 : [], 2 : [], 3 : [], 4 : []}
#
#    for car, car_array in waypoints_rev.items():
#      for i in range(0,len(car_array[0])):
#        location = carla.Transform(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0), carla.Rotation())
#        car_locations[car].append(location)
#      #cars[0].set_location()
#
#    print(car_locations)
#
#    client = AStarPlannerClient()
#    CarlaDataProvider.set_client(client)
#    CarlaDataProvider.set_world(client.world)
#
#    cars = []
#    cars.append(CarlaDataProvider.request_new_actor('vehicle.tesla.model3', car_locations[1][0], rolename='hero'))
#    cars.append(CarlaDataProvider.request_new_actor('vehicle.tesla.model3', car_locations[2][0], rolename='hero'))
#    cars.append(CarlaDataProvider.request_new_actor('vehicle.tesla.model3', car_locations[3][0], rolename='hero'))
#    cars.append(CarlaDataProvider.request_new_actor('vehicle.tesla.model3', car_locations[4][0], rolename='hero'))
#
#    for i in range(0, len(car_locations[1])):
#      time.sleep(1)
#      cars[0].set_location(car_locations[1][i].location)
#      cars[1].set_location(car_locations[2][i].location)
#      cars[2].set_location(car_locations[3][i].location)
#      cars[3].set_location(car_locations[4][i].location)
#      client.world.tick()
#
#    #blueprint = []
#    #blueprint.append(random.choice(self.world.get_blueprint_library().filter(self._actor_filter)))
#    #blueprint.set_attribute('role_name', 'hero')
#    #spawn_point = get_
#
#
#
#if __name__ == '__main__':
#    main()
#
