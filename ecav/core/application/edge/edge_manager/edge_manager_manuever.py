# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Implements the MANEUVER backend of the EdgeManager.
"""
from __future__ import annotations

import time, uuid, weakref, math, logging, random, sys
from collections import deque, defaultdict

import numpy as np
import carla
from easydict import EasyDict as edict

from .edge_manager_base import _BaseEdgeManager, logger
import ecav.core.plan.drive_profile_plotting as open_plt
from ecav.core.application.edge.astar_test_groupcaps_transform import (
        generate_limits_grid, get_slices_clustered, Traffic,
        AStarPlanner, transform_processor)
from ecav.core.plan.global_route_planner import GlobalRoutePlanner
from ecav.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from ecav.core.plan.local_planner_behavior import RoadOption
from ecav.core.application.edge.transform_utils import serialize_waypoint
from ecav.core.application.edge.edge_metrics import EdgeMetrics

import ecloud_pb2 as ecloud          # gRPC waypoint buffer proto
# ----------------------------------------------------------------------


class ManeuverEdge(_BaseEdgeManager):
    """
    Lane-graph manoeuvring edge (previous *MANEUVER* mode).

    Public constructor signature is identical to the old giant file so nothing
    else in your scenario scripts changes.
    """

    # ------------------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 *, world_dt=0.03, **kw):

        super().__init__(world, cfg, cav_world, carla_client,
                         world_dt=world_dt, **kw)

        self.numcars  = len(cfg.get("vehicles", []))
        self.numlanes = cfg.get("num_lanes", 4)
        self.search_dt= cfg.get("search_dt", 2.0)

        # coordinate transform helpers ---------------------------------
        self.waypoints_dict  : dict[int,dict[str,list[float]]] = {}
        self.processor       = None
        self.secondary_offset= 0.0

        # traffic model -------------------------------------------------
        self.spawn_x, self.spawn_y, self.spawn_v = [], [], []
        self.Traffic_Tracker : Traffic|None = None
        self.ov, self.oy = generate_limits_grid()
        self.grid_size   = 1.0
        self.robot_radius= 1.0

        # runtime-buffers ----------------------------------------------
        self.xcars  = np.empty((self.numcars, 0))
        self.ycars  = np.empty((self.numcars, 0))
        self.locations = []                     # override way-points

        # keep a pointer so we can draw strings etc.
        self._dao  = None
        self.grp   = None
        self.carla_client = carla_client

        logger.debug("Initialised ManeuverEdge with %d cars", self.numcars)

    # ------------------------------------------------------------------
    #  High-level API required by _BaseEdgeManager
    # ------------------------------------------------------------------
    def start_edge(self):
        """Pre-compute four reference lanes & coordinate transform."""
        self._build_reference_routes()
        self.processor = transform_processor(self.waypoints_dict)
        self.processor.process_waypoints_bidirectional(0)   # pre-cache
        self._init_spawn_lists()
        self.Traffic_Tracker = Traffic(self.search_dt,
                                       self.numlanes,
                                       numcars=self.numcars,
                                       map_length=200,
                                       x_initial=self.spawn_x,
                                       y_initial=self.spawn_y,
                                       v_initial=self.spawn_v)

    def update_information(self, frame_idx: int = 0):
        """
        Refresh Traffic() from current ego vehicle poses so that the A* planner
        has up-to-date positions & velocities.
        """
        self.spawn_x.clear(); self.spawn_y.clear(); self.spawn_v.clear()

        for vm in self.vehicle_manager_list:
            loc = vm.vehicle.get_location()
            x, y = self.processor.process_single_waypoint_forward(loc.x, loc.y)
            vel  = vm.vehicle.get_velocity()
            self.spawn_x.append(x)
            self.spawn_y.append(y)
            self.spawn_v.append(math.sqrt(vel.x**2 + vel.y**2 + vel.z**2))

        self.Traffic_Tracker = Traffic(self.search_dt, self.numlanes,
                                       numcars   = self.numcars,
                                       map_length=200,
                                       x_initial = self.spawn_x,
                                       y_initial = self.spawn_y,
                                       v_initial = self.spawn_v)

        # set target speed for each Traffic car to its VehicleManager max_speed
        for vm, car in zip(self.vehicle_manager_list,
                           self.Traffic_Tracker.cars_on_road):
            car.target_velocity = vm.agent.max_speed * 0.277778  # km/h→m/s

    def run_step(self, tick: int):
        """
        1) update world-state
        2) run lane-graph algorithm
        3) push new waypoint buffers to every VehicleManager
        """
        self.update_information(tick)
        self._algorithm_step()           # fills self.locations

        # pack to gRPC proto and deliver
        for idx, vm in enumerate(self.vehicle_manager_list):
            buf = ecloud.WaypointBuffer()
            buf.vehicle_index = idx
            buf.waypoint_buffer.extend(
                [serialize_waypoint(self.locations[idx])])
            # send over network here if you have a channel …

        # finally advance vehicle controllers
        if not self.run_distributed:
            for vm in self.vehicle_manager_list:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())

    # ------------------------------------------------------------------
    #  Internal helpers
    # ------------------------------------------------------------------
    def _build_reference_routes(self):
        """Creates four lane centre-lines and fills self.waypoints_dict."""
        world   = self.carla_client.get_world()
        self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
        self.grp  = GlobalRoutePlanner(self._dao); self.grp.setup()

        # pick waypoints from stored .npy (same as before)
        indices_src = np.load('Indices_start.npy').astype(int)
        indices_dst = np.load('Indices_dest.npy').astype(int)
        wps         = world.get_map().generate_waypoints(10)

        pairs = [(indices_src[i,1], indices_dst[i,1]) for i in range(4)]
        routes= [self.grp.trace_route(carla.Location(wps[s].transform.location),
                                      carla.Location(wps[d].transform.location))
                 for s,d in pairs]

        for lane_idx, route in enumerate(routes, 1):
            self.waypoints_dict[lane_idx] = {'x':[], 'y':[]}
            for wp,_ in route:
                self.waypoints_dict[lane_idx]['x'].append(wp.transform.location.x)
                self.waypoints_dict[lane_idx]['y'].append(wp.transform.location.y)

    # ------------------------------------------------------------------
    def _init_spawn_lists(self):
        """Translate initial ego locations to lane frame for Traffic()."""
        self.spawn_x.clear(); self.spawn_y.clear(); self.spawn_v.clear()

        for vm in self.vehicle_manager_list:
            loc = vm.vehicle.get_location()
            x, y = self.processor.process_single_waypoint_forward(loc.x, loc.y)
            self.spawn_x.append(x)
            self.spawn_y.append(y)
            self.spawn_v.append(0.0)

    # ------------------------------------------------------------------
    def _algorithm_step(self):
        """
        Original graph-planner logic: slices traffic, runs A* per slice,
        writes back `self.locations` list of CARLA waypoints (one per car).
        """
        t0 = time.time()
        slice_list, vel_array, lane_cmd = get_slices_clustered(
                self.Traffic_Tracker, self.numcars)

        # run A* whenever a slice has >1 vehicles
        for i in range(len(slice_list)-1, -1, -1):
            if len(slice_list[i]) < 2:
                continue
            a_star = AStarPlanner(slice_list[i], self.ov, self.oy,
                                  self.grid_size, self.robot_radius,
                                  self.Traffic_Tracker.cars_on_road, i)
            rv, ry, _ = a_star.planning()
            if len(ry) >= 2:
                lane_cmd[i] = ry[-2]
                vel_array[i] = rv[-2]

        # propagate planned lane/velocity to Traffic() cars
        for i in range(len(slice_list)-1, -1, -1):
            if not slice_list[i]: continue
            for car, v_new, l_new in zip(slice_list[i],
                                         vel_array[i], lane_cmd[i]):
                if l_new > car.lane:   car.intentions = "Lane Change 1"
                elif l_new < car.lane: car.intentions = "Lane Change -1"
                car.v = v_new

        self.Traffic_Tracker.time_tick(mode='Graph')

        # convert back to world coords
        x_mat, y_mat, _, _ = self.Traffic_Tracker.ret_car_locations()
        self.locations.clear()
        for j in range(self.numcars):
            pt = self.processor.process_back([np.array([[x_mat[j,0]],
                                                        [y_mat[j,0]]])])[0]
            self.locations.append(
                self._dao.get_waypoint(carla.Location(x=pt[0,0],
                                                      y=pt[1,0], z=0.0)))

        self.debug.algorithm_time_list.append((time.time()-t0)*1000)
        # optional: draw debug markers if desired
