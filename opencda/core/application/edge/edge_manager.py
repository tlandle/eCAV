# -*- coding: utf-8 -*-

"""Edge Manager
"""

# Author: Tyler Landle <tlandle3@gatech.edu>
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
from collections import deque, defaultdict

from opencda.scenario_testing.utils.yaml_utils import load_yaml
from opencda.core.prediction.linear_predictor_manager import LinearPredictorManager
from opencda.core.prediction.obstacle_prediction import ObstaclePrediction
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle, BoundingBox
from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT
from opencda.core.common.misc import distance_vehicle, draw_trajetory_points
import open3d as o3d

logger = logging.getLogger(__name__)
coloredlogs.install(level='DEBUG', logger=logger)
logger.setLevel(logging.DEBUG)

cloud_config = load_yaml("cloud_config.yaml")

if cloud_config["log_level"] == "error":
    logger.setLevel(logging.ERROR)
elif cloud_config["log_level"] == "warning":
    logger.setLevel(logging.WARNING)
elif cloud_config["log_level"] == "info":
    logger.setLevel(logging.INFO)

sys.path.append("/home/chattsgpu/Documents/Carla_opencda/TrafficSimulator_eCloud/OpenCDA/")

import opencda.core.plan.drive_profile_plotting as open_plt
from opencda.core.application.edge.astar_test_groupcaps_transform import *
from opencda.core.plan.global_route_planner import GlobalRoutePlanner
from opencda.core.plan.global_route_planner_dao import GlobalRoutePlannerDAO
from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.application.edge.transform_utils import *
from opencda.core.application.edge.edge_debug_helper import \
    EdgeDebugHelper

import grpc
import ecloud_pb2 as ecloud
import ecloud_pb2_grpc as rpc


def create_waypoint_roadoption_tuple(location, carla_map):
    # Get the waypoint closest to the vehicle's location
    waypoint = carla_map.get_waypoint(location)
    
    # Determine the road option (e.g., straight, left, right, etc.)
    # For simplicity, assume a default road option
    road_option = RoadOption.LANEFOLLOW
    
    return (waypoint, road_option)

def _ab3d_row_from_bbox(obj, frame_idx: int):
    """
    ObstacleVehicle →  det_row, info_row
        det_row  = [h, w, l, x, y, z, ry]             (float)
        info_row = [frame_idx, guid/track_id, carla_id]  (int)
    """
    # ── geometry ─────────────────────────────────────────────────────
    bbx = obj.bounding_box
    h, w, l = bbx.extent.z * 2, bbx.extent.y * 2, bbx.extent.x * 2
    loc     = obj.location
    #ry      = math.radians(obj.transform.rotation.yaw)
    ry = 0.0  # AB3DMOT does not use yaw, so we set it to 0.0

    # ── ids ──────────────────────────────────────────────────────────
    track_id = getattr(obj, "track_id", -1)    # use as guid if nothing else
    carla_id = getattr(obj, "carla_id", -1)    # –1 if unknown

    det_row  = [loc.x, loc.y, loc.z, h, w, l, ry]  # AB3DMOT format
    info_row = [frame_idx, track_id, carla_id]
    return det_row, info_row

def collect_ab3d_detections(edge,
                            objects_dict: dict,
                            frame_idx: int):
    """
    Builds dets_all = {'dets': (N,7) float32, 'info': (N,3) int64}
    for AB3DMOT from:
        • every ObstacleVehicle in *objects_dict* (camera/LiDAR) and
        • one self-beacon per VehicleManager in edge.vehicle_manager_list.
    """
    det_rows, info_rows = [], []

    # 1. synthetic obstacles from perception
    for obj in objects_dict.get("vehicles", []):
        det, info = _ab3d_row_from_bbox(obj, frame_idx)
        det_rows.append(det);  info_rows.append(info)

    # 2. one self beacon per ego vehicle  (ground truth pose, real carla_id)
    for vm in edge.vehicle_manager_list:
        veh = vm.vehicle
        bbx = veh.bounding_box.extent
        h,w,l = bbx.z*2, bbx.y*2, bbx.x*2
        loc   = veh.get_location()
        ry = 0.0  # AB3DMOT does not use yaw, so we set it to 0.0
        #ry    = math.radians(veh.get_transform().rotation.yaw)
        det_rows.append([loc.x, loc.y, loc.z, h, w, l, ry])
        info_rows.append([frame_idx, veh.id, veh.id])        # guid = carla_id

    dets_np  = np.asarray(det_rows,  dtype=np.float32)
    info_np  = np.asarray(info_rows, dtype=np.int64)
    return {'dets': dets_np, 'info': info_np}

def _belongs_to_vehicle(pred, vm, track_to_carla):
    """
    Return True if this ObstaclePrediction is the ego vehicle 'vm'.
    Works with either carla_id (if present) or track→carla mapping.
    """
    cid = getattr(pred, "carla_id", -1)                 # if you added it
    if cid == -1:                                       # fall-back via track id
        trk = pred.obstacle_trajectory.obstacle.track_id
        cid = track_to_carla.get(trk, -1)
    return cid == vm.vehicle.id



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

    def __init__(self, world, config_yaml, cav_world, carla_client, world_dt=0.03, edge_dt=0.20, search_dt=2.00, mode=None, other_vehicles=None):

        self.edgeid = str(uuid.uuid1())
        self.world = world
        self.vehicle_manager_list = []
        self.rsu_manager_list = []
        #self.target_speed = config_yaml['target_speed'] # kph
        #self.traffic_velocity = self.target_speed * 0.277778 # convert to m/s! NOT kph
        logger.debug(config_yaml)
        if 'vehicles' in config_yaml:
            self.numcars = len(config_yaml['vehicles']) # TODO - set edge_index
        if 'rsus' in config_yaml:
            self.numrsus = len(config_yaml['rsus'])
        self.latency = config_yaml['latency'] if 'latency' in config_yaml else 0
        self.activate = config_yaml["mode"]
        #self.locations = []
        self.destination = None
        # Hold all vehicle trajectories in dictionary of list:qs
        self.traj_dict = {}
        self.vehicle_speeds = {}
        # Query the vehicle locations and velocities + target velocities:wq
        self.spawn_x = []
        self.spawn_y = []
        self.spawn_v = [] # probably 0s but can be target vel too
        self.xcars = np.empty((self.numcars, 0))
        self.ycars = np.empty((self.numcars, 0))
        self.x_states = None
        self.y_states = None
        self.tv = None
        self.v = None
        self.target_velocities = np.empty((self.numcars, 0))
        self.velocities = np.empty((self.numcars,0))
        self.Traffic_Tracker = None
        self.waypoints_dict = {}
        self.cav_world = weakref.ref(cav_world)()
        self.ov, self.oy = generate_limits_grid()
        self.grid_size = 1.0
        self.robot_radius = 1.0
        self.processor = None
        self.secondary_offset=0
        cav_world.update_edge(self)
        self.carla_client = carla_client
        self.objects_deque = deque()
        self.tracked_trajectories = {}
        self.objects = {}
        self.mode = mode
        self.dt = world_dt
        self.other_vehicles = other_vehicles
        # TODO make this a parameter
        self.num_future_steps = 25
        self.linear_predictor_manager = LinearPredictorManager(num_future_steps=self.num_future_steps)
        self.track_to_carla = {}
        self.carla_to_track = {}

        self.debug_helper = EdgeDebugHelper(0)

        self.search_dt = config_yaml['search_dt'] if 'search_dt' in config_yaml else 2.00
        self.numlanes = config_yaml['num_lanes'] if 'num_lanes' in config_yaml else 4

        self.ab3dmot_tracker = None
        self.obstacle_trajectories = []

        
    def start_edge(self):
      if(self.activate == "MANEUVER"):
        self.get_four_lane_waypoints_dict()
        logger.debug("Got Waypoints")
        self.processor = transform_processor(self.waypoints_dict)
        logger.debug("Edge: Waypoints transformed")
        _, _ = self.processor.process_waypoints_bidirectional(0)
        logger.debug("Edge: Waypoints processed")
        inverted = self.processor.process_forward(0)
        logger.debug(len(inverted))
        i = 0

        # for k in inverted:
        #     if k[0,0] <= 0 and k[0,0] < -self.secondary_offset:
        #       logger.debug("Current indice is: ", k[0,0])
        #       self.secondary_offset = -k[0,0]

        for vehicle_manager in self.vehicle_manager_list:
            spawn_coords = vehicle_manager.vehicle.get_location()
            spawn_coords = np.array([spawn_coords.x,spawn_coords.y]).reshape((2,1))
            logger.debug("Spawn Coords before transform")
            logger.debug(spawn_coords)
            spawn_coords = self.processor.process_single_waypoint_forward(spawn_coords[0,0],spawn_coords[1,0])
            logger.debug("Spawn Coords after Transform")
            logger.debug(spawn_coords)
            # sys.exit()
            # self.spawn_x.append(vehicle_manager.vehicle.get_location().x)
            # self.spawn_y.append(vehicle_manager.vehicle.get_location().y)
            #self.spawn_v.append(vehicle_manager.vehicle.get_velocity())
            ## THIS IS TEMPORARY ##
            # logger.debug("inverted is: ", inverted[i][0,0])
            # logger.debug("revised x is: ", self.secondary_offset)
            self.spawn_x.append(spawn_coords[0]) # inverted[i][0,0]+self.secondary_offset)
            #self.spawn_v.append(5*(i+1))
            self.spawn_v.append(0)
            self.spawn_y.append(spawn_coords[1])# inverted[i][1,0])
            i += 1

            # TODO: DIST --> do we need to clear at start in containers?
            #vehicle_manager.agent.get_local_planner().get_waypoint_buffer().clear() # clear waypoint buffer at start
        self.Traffic_Tracker = Traffic(self.search_dt,self.numlanes,numcars=self.numcars,map_length=200,x_initial=self.spawn_x,y_initial=self.spawn_y,v_initial=self.spawn_v)
      else:
        for vehicle_manager in self.vehicle_manager_list:
            self.traj_dict[vehicle_manager.vehicle.id] = []
        for sequenced_vehicle in self.other_vehicles:
            self.traj_dict[sequenced_vehicle._actor.id] = []
        # Start 3d tracker
        self.start_ab3dmot()


    def start_ab3dmot(self):

        # Minimal config
        cfg = edict()
        cfg.vis = False               # No visualization
        cfg.save_path = None          # Don't save output
        cfg.use_3d_iou = False        # Use distance matching instead of 3D IoU
        cfg.thres = 2.0               # Matching threshold in meters
        cfg.output_dir = None         # Not saving results
        cfg.min_hits = 3              # Frames required to confirm a track
        cfg.max_age = 2               # Frames to keep unmatched tracks alive
        cfg.ego_com = None          # No ego compensation
        cfg.affi_pro = False        # No affinity post-processing
        cfg.dataset = "KITTI"     # Dataset type
        cfg.det_name = "deprecated"

        # Choose category
        cat = 'Car'  # or 'Truck', 'Pedestrian', etc.

        # Initialize the tracker
        self.ab3dmot_tracker = AB3DMOT(cfg, cat)

    def get_four_lane_waypoints_dict(self):
      world = self.carla_client.get_world()
      self._dao = GlobalRoutePlannerDAO(world.get_map(), 2)
      grp = GlobalRoutePlanner(self._dao)
      grp.setup()
      waypoints = world.get_map().generate_waypoints(10)

      indices_source = np.load('Indices_start.npy')
      indices_dest = np.load('Indices_dest.npy')

      indices_source = indices_source.astype(int)
      indices_dest = indices_dest.astype(int)

      #logger.debug("Source Shape: ", indices_source.shape)

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
        #logger.debug(w)
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
        #logger.debug(w)
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
        #logger.debug(w)
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
        #logger.debug(w)
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



    def add_member(self, vehicle_manager):
        """
        Add memeber to the current edge

        Parameters
        __________
        vehicle_manager : opencda object
            The vehicle manager class.
        """
        self.vehicle_manager_list.append(vehicle_manager)

    def add_rsu(self, rsu_manager):
        self.rsu_manager_list.append(rsu_manager)

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

    def transform_object_track_ids_to_global_ids(self):
        """
        Transform the object track ids to global ids.
        """
        # format : key = vehicle_id, value = list of locations
        trajectory = {}
        for obj in self.objects_deque[0]:
            if 'vehicles' in obj:
                for vehicle in obj['vehicles']:
                    if vehicle.id == self.vehicle.id:
                        vehicle.track_id = vehicle.id
                        #logger.debug("Transforming Object Track IDs to Global IDs")
                        #logger.debug(trajectory)
                        #logger.debug(vehicle.id)
                        #logger.debug(vehicle.get_transform().location)
        return trajectory


    def filter_stale_tracks(self, mot_output_deque, max_age=5):
        """
        Filters out stale tracks from each frame in a deque of AB3DMOT outputs.

        Args:
            mot_output_deque (deque): deque of np.ndarray, each (N, 15) AB3DMOT output for one frame.
            max_age (int): maximum allowed age (column 13) before a track is considered stale.

        Returns:
            deque: filtered deque of np.ndarrays, same format but with stale tracks removed.
        """
        filtered_list = []
        for frame_tracks in mot_output_deque:
            if isinstance(frame_tracks, list):
                frame_tracks = np.array(frame_tracks)

            if frame_tracks.shape[0] == 0:
                filtered_list.append(frame_tracks)
                continue

            if frame_tracks.shape[1] >= 14:
                active_tracks = frame_tracks[frame_tracks[:, 13] <= max_age]
            else:
                active_tracks = frame_tracks

            filtered_list.append(active_tracks)

        return filtered_list

    def get_vehicle_trajectories_from_object_history(self):
        """
        Get the vehicle trajectories from the object history.
        """
        # format : key = vehicle_id, value = list of locations
        trajectory = {}
        for obj in self.objects_deque:
            if 'vehicles' in obj:
                for vehicle in obj['vehicles']:
                    if vehicle.id == self.vehicle.id:
                        trajectory.append(vehicle.get_transform().location)

    def convert_boundingbox_to_ab3dmot_dict(self, bbox: BoundingBox,
                                        score: float = 1.0,
                                        obj_type: str = 'Car',
                                        frame_idx: int = 0) -> dict:
        """
        Converts a BoundingBox object to AB3DMOT-compatible detection dictionary.

        Args:
            bbox (BoundingBox): Custom bounding box with location and extent.
            score (float): Detection confidence.
            obj_type (str): Object type label.
            frame_idx (int): Frame number.

        Returns:
            dict: Detection dictionary for AB3DMOT.
        """
        # Extract dimensions (extent is half-size)
        h = bbox.extent.z * 2
        w = bbox.extent.x * 2
        l = bbox.extent.y * 2

        # Get location as [x, y, z]
        loc = bbox.location
        location = [loc.x, loc.y, loc.z]

        return {
            'frame': frame_idx,
            'id': -1,
            'type': obj_type,
            'truncated': 0.0,
            'occluded': 0,
            'alpha': 0.0,
            'bbox': [0, 0, 50, 50],  # dummy 2D image box (not used)
            'location': location,     # [x, y, z]
            'dimensions': [h, w, l],  # [height, width, length]
            'rotation_y': 0.0,        # placeholder yaw
            'score': score
        }

    
    def convert_aabb_to_ab3dmot_dict(self,aabb: o3d.geometry.AxisAlignedBoundingBox, 
                                  frame_idx: int = 0, 
                                  obj_type: str = 'Car',
                                  score: float = 1.0) -> dict:
        """
        Convert an Open3D AxisAlignedBoundingBox to AB3DMOT detection format.

        Parameters:
            aabb (o3d.geometry.AxisAlignedBoundingBox): The Open3D AABB object.
            frame_idx (int): The frame number.
            obj_type (str): Class label (e.g., 'Car').
            score (float): Detection confidence score.

        Returns:
            dict: Detection dictionary compatible with AB3DMOT.
        """
        # Get center and extent
        center = aabb.get_center()          # (x, y, z)
        extent = aabb.get_extent()          # (dx, dy, dz) → width, height, length (in global axes)

        # Reorder dimensions for AB3DMOT: [height, width, length]
        dimensions = [extent[1], extent[0], extent[2]]

        # AB3DMOT uses yaw (rotation_y), but AABB has no orientation → set 0
        yaw = 0.0

        detection = {
            'frame': frame_idx,
            'id': -1,
            'type': obj_type,
            'truncated': 0.0,
            'occluded': 0,
            'alpha': 0.0,
            'bbox': [0, 0, 50, 50],          # Dummy 2D image box
            'location': list(center),        # Bottom center of 3D box
            'dimensions': dimensions,        # [h, w, l]
            'rotation_y': yaw,
            'score': score,
            'info': []
        }

        return detection


    def set_destination(self, destination):
        """
        Set destination of the vehicle managers in the platoon.
        """
        self.destination = destination
        # for i in range(len(self.vehicle_manager_list)):
        #     self.vehicle_manager_list[i].set_destination(
        #         self.vehicle_manager_list[i].vehicle.get_location(),
        #         destination, clean=True)

    def calculate_gap(self, distance):
        """
        Calculate the current vehicle and frontal vehicle's time/distance gap.
        Note: please use groundtruth position of the frontal vehicle to
        calculate the correct distance.

        Parameters
        ----------
        distance : float
            Distance between the ego vehicle and frontal vehicle.
        """
        # we need to count the vehicle length in to calculate the gap
        boundingbox = self.vehicle.bounding_box
        veh_length = 2 * abs(boundingbox.location.y - boundingbox.extent.y)

        delta_v = self._ego_speed / 3.6
        time_gap = distance / delta_v
        self.time_gap = time_gap
        self.dist_gap = distance - veh_length

    def _merge_objects(self,
                       dest: dict,
                       src:  dict,
                       owner_id: int,
                       track_to_carla: dict):
        """
        Append objects from *src* into *dest* **except** those that belong to
        the vehicle whose CARLA id == owner_id.

        track_to_carla : {track_id → carla_id}  (possibly empty)
        """
        for otype, olist in src.items():
            if otype not in dest:
                dest[otype] = []

            for obj in olist:
                # 1) explicit obj.carla_id  (if your class has it)
                cid = getattr(obj, "carla_id", -1)

                # 2) otherwise try the mapping table from AB3DMOT
                if cid == -1 and hasattr(obj, "track_id"):
                    cid = track_to_carla.get(obj.track_id, -1)

                if cid == owner_id:           # <-- skip self
                    continue

                dest[otype].append(obj)

    def update_information(self):
        """Collect world-state for the edge; never store ‘self’ objects."""

        self.objects       = {}          # reset global object dict
        self.spawn_x.clear(); self.spawn_y.clear(); self.spawn_v.clear()

        # ──────────────────────────────────────────────────────────────
        #  Loop over every VehicleManager
        # ──────────────────────────────────────────────────────────────
        for vm in self.vehicle_manager_list:

            if self.activate == "MANEUVER":
                loc = vm.vehicle.get_location()
                x, y = self.processor.process_single_waypoint_forward(loc.x, loc.y)
                v    = vm.vehicle.get_velocity()
                v_mag = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                self.spawn_x.append(x); self.spawn_y.append(y); self.spawn_v.append(v_mag)

            elif self.activate == "PERCEPTION":
                self.traj_dict[vm.vehicle.id]     = vm.agent.get_local_planner().get_waypoint_buffer().copy()
                self.vehicle_speeds[vm.vehicle.id] = vm.vehicle.get_velocity()

            elif self.activate == "PREDICTION":
                self._merge_objects(self.objects,
                               vm.agent.objects,
                               owner_id       = vm.vehicle.id,
                               track_to_carla = self.track_to_carla)

        # ──────────────────────────────────────────────────────────────
        #  Merge RSU objects (RSU never equals a vehicle, so no filter)
        # ──────────────────────────────────────────────────────────────
        for rsu in self.rsu_manager_list:
            for otype, olist in rsu.objects.items():
                self.objects.setdefault(otype, []).extend(olist)

        # keep a history deque if you use latency compensation elsewhere
        self.objects_deque.appendleft(self.objects.copy())

        # ──────────────────────────────────────────────────────────────
        #  (optional) build Traffic() etc. – unchanged from your code
        # ──────────────────────────────────────────────────────────────
        if self.activate == "MANEUVER":
            self.Traffic_Tracker = Traffic(self.search_dt,
                                           self.numlanes,
                                           numcars   = self.numcars,
                                           map_length=200,
                                           x_initial = self.spawn_x,
                                           y_initial = self.spawn_y,
                                           v_initial = self.spawn_v)

            for vm, car in zip(self.vehicle_manager_list, self.Traffic_Tracker.cars_on_road):
                car.target_velocity = vm.agent.max_speed * 0.277778  # km/h → m/s

    def update_information_old(self):
        """
        Update CAV world information for every member in the list.
        """

        self.spawn_x.clear()
        self.spawn_y.clear()
        self.spawn_v.clear()
        self.objects.clear()
        # start_time = time.time()
        # # for i in range(len(self.vehicle_manager_list)):
        # #     self.vehicle_manager_list[i].update_info()
        # #     logger.info("Updated location for vehicle %s - x:%s, y:%s", i, self.vehicle_manager_list[i].vehicle.get_location().x, self.vehicle_manager_list[i].vehicle.get_location().y)
        # end_time = time.time()
        # logger.debug("Vehicle Manager Update Info Time: %s", (end_time - start_time))
        start_time = time.time()
        for i in range(len(self.vehicle_manager_list)):
            if(self.activate == "MANEUVER"):
              x,y = self.processor.process_single_waypoint_forward(self.vehicle_manager_list[i].vehicle.get_location().x, self.vehicle_manager_list[i].vehicle.get_location().y)
              v = self.vehicle_manager_list[i].vehicle.get_velocity()
              v_scalar = math.sqrt(v.x**2 + v.y**2 + v.z**2)
              self.spawn_x.append(x)
              self.spawn_y.append(y)
              self.spawn_v.append(v_scalar)
            elif(self.activate == "PERCEPTION"):
                self.traj_dict[self.vehicle_manager_list[i].vehicle.id] = self.vehicle_manager_list[i].agent.get_local_planner().get_waypoint_buffer().copy()
                self.vehicle_speeds[self.vehicle_manager_list[i].vehicle.id] = self.vehicle_manager_list[i].vehicle.get_velocity()
                logger.debug("Vehicle id: %s" %self.vehicle_manager_list[i].vehicle.id)
            elif(self.activate == "PREDICTION"):
                self.objects = {**self.objects, **self.vehicle_manager_list[i].agent.objects}
        if(self.activate == "PERCEPTION"):
            for sequenced_vehicle in self.other_vehicles:
                #logger.debug("Sequenced Vehicle: ", sequenced_vehicle._actor.id)
                #logger.debug("Sequenced Vehicle Local Planner: ", sequenced_vehicle._local_planner_dict)
                if sequenced_vehicle._actor in sequenced_vehicle._local_planner_dict and sequenced_vehicle._local_planner_dict[sequenced_vehicle._actor] is not None:
                    #logger.debug("Sequenced Vehicle in planner dict: ", sequenced_vehicle._actor.id)
                    self.traj_dict[sequenced_vehicle._actor.id] = sequenced_vehicle._local_planner_dict[sequenced_vehicle._actor]._waypoints_queue.copy()
                    waypoint_roadoption_tuple = create_waypoint_roadoption_tuple(sequenced_vehicle._actor.get_location(), sequenced_vehicle._local_planner_dict[sequenced_vehicle._actor]._map)
                    logger.debug("Waypoint Roadoption Tuple: ", waypoint_roadoption_tuple)
                    logger.debug("Waypoint Roadoption Tuple Location (%s, %s): ", waypoint_roadoption_tuple[0].transform.location.x, waypoint_roadoption_tuple[0].transform.location.y)
                    self.traj_dict[sequenced_vehicle._actor.id].appendleft(waypoint_roadoption_tuple)
                    #logger.debug("Traj Dict: ", self.traj_dict[sequenced_vehicle._actor.id])
                    self.vehicle_speeds[sequenced_vehicle._actor.id] = sequenced_vehicle._actor.get_velocity()
                    #logger.debug("Sequenced Velocity: ", self.vehicle_speeds[sequenced_vehicle._actor.id])
                    #logger.debug("Waypoint Queue: ", self.traj_dict[sequenced_vehicle._actor.id])
                #logger.debug(self.vehicle_manager_list[i].agent.objects)
                #self.objects =  {**self.objects,  **self.vehicle_manager_list[i].agent.objects}
                #logger.debug(self.objects)
                #logger.info("update_information for vehicle_%s - x:%s, y:%s", i, x, y)
        
        #logger.debug("Traj Dict: ", self.traj_dict)
        end_time = time.time()
        logger.debug("Update Info Transform Forward Time: %s", (end_time - start_time))
        #logger.debug(self.spawn_x)
        #logger.debug(self.spawn_y)
        #logger.debug(self.spawn_v)
        for i in range(len(self.rsu_manager_list)):
            #if len(self.objects_deque) == 10:
                #self.objects_deque.pop()
            self.objects = {**self.objects, **self.rsu_manager_list[i].objects}
            #logger.debug("RSU Objects: ", self.rsu_manager_list[i].objects)
        self.objects_deque.appendleft(self.objects.copy())


        logger.debug(self.objects)
          

        start_time = time.time()
        #Added in to check if traffic tracker updating would fix waypoint deque issue
        # TODO: data drive num cars
        if(self.activate == "MANEUVER"):
          self.Traffic_Tracker = Traffic(self.search_dt,self.numlanes,numcars=self.numcars,map_length=200,x_initial=self.spawn_x,y_initial=self.spawn_y,v_initial=self.spawn_v)
          end_time = time.time()
          logger.debug("Traffic Tracker Time: %s", (end_time - start_time))

          for i, car in enumerate(self.Traffic_Tracker.cars_on_road):
            logger.debug(i)
            logger.debug(self.vehicle_manager_list[i].agent.max_speed)
            car.target_velocity = self.vehicle_manager_list[i].agent.max_speed * 0.277778 # convert to m/s! NOT kph

        # sys.exit()

        #logger.debug("Updated Info")

    def algorithm_step(self):
        self.locations = []
        #logger.debug("started Algo step")

        #DEBUGGING: Bypass algo and simply move cars forward to solve synch and transform issues
        #Bypassed as of 14/3/2022

        slice_list, vel_array, lanechange_command = get_slices_clustered(self.Traffic_Tracker, self.numcars)

        for i in range(len(slice_list)-1,-1,-1): #Iterate through all slices
            if len(slice_list[i]) >= 2: #If the slice has more than one vehicle, run the graph planner. Else it'll move using existing
            #responses - slow down on seeing a vehicle ahead that has slower velocities, else hit target velocity.
            #Somewhat suboptimal, ideally the other vehicle would be
            #folded into existing groups. No easy way to do that yet.
                a_star = AStarPlanner(slice_list[i], self.ov, self.oy, self.grid_size, self.robot_radius, self.Traffic_Tracker.cars_on_road, i)
                rv, ry, rx_tracked = a_star.planning()
                if len(ry) >= 2: #If there is some planner result, then we move ahead on using it
                    lanechange_command[i] = ry[-2]
                    vel_array[i] = rv[-2]
                else: #If the planner returns an empty list, continue as before - use emergency responses.
                    lanechange_command[i] = ry[0]
                    vel_array[i] = ry[0]

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

        #logger.debug("Success capsule")

        #Recording location and state
        x_states, y_states, tv, v = self.Traffic_Tracker.ret_car_locations() # Commented out for bypassing algo
        # x_states, y_states, v = [], [], [] #Algo bypass begins
        self.xcars = np.empty((self.numcars, 0))
        self.ycars = np.empty((self.numcars, 0))

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

        #logger.debug("Returned X: ", self.xcars)
        #logger.debug("Returned Y: ", self.ycars)

        self.xcars = self.xcars - self.secondary_offset

        #logger.debug("Revised Returned X: ", self.xcars)

        ###########################################

        # waypoints_rev = {1 : np.empty((2,0)), 2 : np.empty((2,0)), 3 : np.empty((2,0)), 4 : np.empty((2,0)), 5 : np.empty((2,0)), 6 : np.empty((2,0)), 7 : np.empty((2,0)), 8 : np.empty((2,0))}
        # for i in range(0,self.xcars.shape[1]):
        #   processed_array = []
        #   for j in range(0,self.numcars):
        #     x_res = self.xcars[j,i]
        #     y_res = self.ycars[j,i]
        #     processed_array.append(np.array([[x_res],[y_res]]))
        #     logger.debug("Appending to waypoints_rev")
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
            #logger.debug("Appending to waypoints_rev")
          #logger.debug(processed_array)
          back = self.processor.process_back(processed_array)

          #logger.debug(waypoints_rev)
          #logger.debug(waypoints_rev.keys())
          for j in range(0,self.numcars):
            #logger.debug(len(back))
            #logger.debug(j)
            # logger.debug(waypoints_rev[str(j+1)])
            # logger.debug(back[str(j)])
            # logger.debug(np.hstack((waypoints_rev[str(j+1)],back[str(j)])))
            waypoints_rev[str(j+1)] = np.hstack((waypoints_rev[str(j+1)],back[j]))

        # processed_array = []
        # for k in range(0,4): #Added 16/03 outer loop to check if waypoint horizon influenced things, it did not seem to.
        #     for j in range(0,self.numcars):
        #         x_res = self.xcars[j,-1]
        #         y_res = self.ycars[j,-1]
        #         processed_array.append(np.array([[x_res],[y_res]]))
        #         self.xcars[j,-1] += 4 #Increment by +4, just adding another waypoint '4m' ahead of this one, until horizon 3 steps ahead
        #     logger.debug("Appending to waypoints_rev: ", self.xcars)
        #     back = self.processor.process_back(processed_array)
        #     waypoints_rev[1] = np.hstack((waypoints_rev[1],back[0]))
        #     waypoints_rev[2] = np.hstack((waypoints_rev[2],back[1]))
        #     waypoints_rev[3] = np.hstack((waypoints_rev[3],back[2]))
        #     waypoints_rev[4] = np.hstack((waypoints_rev[4],back[3]))

        #logger.debug(waypoints_rev)
        # car_locations = {1 : [], 2 : [], 3 : [], 4 : [], 5 : [], 6 : [], 7 : [], 8 : []}

        logger.warning("CREATING OVERRIDE WAYPOINTS")
        for car, car_array in waypoints_rev.items():
          for i in range(0,len(car_array[0])):
            location = self._dao.get_waypoint(carla.Location(x=car_array[0][i], y=car_array[1][i], z=0.0))
            logger.info("algorithm_step: car_%s location - %s", car, location)
            self.locations.append(location)

            logger.warning("car_%s - (x: %s, y: %s)", car, location.transform.location.x, location.transform.location.x)

        #logger.debug("Locations appended: ", self.locations)

    def run_step(self, step_id=0):
      if(self.activate == "PERCEPTION"):
        logger.debug("running perception_step edge")
        self.run_step_perception(step_id)
      elif(self.activate == "PREDICTION"):
        #logger.debug("running prediction_step edge")
        self.run_step_prediction(step_id)
      elif(self.activate == "MANEUVER"):
        self.run_step_maneuver(step_id) 

    
    def run_step_prediction(self, step_id):
        # ------------------------------------------------ latency filter ---
        print("Running prediction step for edge", self.edgeid, flush=True)
        if step_id < int(self.latency / self.dt):
            return
        lag_steps = int(self.latency / self.dt)

        # ------------------------------------------------ collect detections
        det_rows, info_rows = [], []

        # 1. all vehicle obstacles from the lagged deque snapshot

        objects_snapshot = self.objects_deque[lag_steps].copy()

        dets_all = collect_ab3d_detections(self,
                                   objects_snapshot,
                                   frame_idx=step_id)

        # ------------------------------------------------ AB3DMOT tracking
        t0 = time.perf_counter()
        tracks_np, _ = self.ab3dmot_tracker.track(dets_all, step_id)
        tracker_ms = (time.perf_counter() - t0) * 1000.0

        print("Tracks after AB3DMOT:", tracks_np, flush=True)

        for frame_tracks in tracks_np:
            if frame_tracks is None or len(frame_tracks) == 0:
                continue

            for track in frame_tracks:
                track_id = int(track[7])
                carla_id  = int(track[8])
                if carla_id != -1:
                    self.track_to_carla[track_id]  = carla_id
                    self.carla_to_track[carla_id]  = track_id



        print("Track to CARLA mapping:", self.track_to_carla, flush=True)
        print("CARLA to Track mapping:", self.carla_to_track, flush=True)

        # ------------------------------------------------ convert to trajectories
        tracks_np = self.filter_stale_tracks(tracks_np, max_age=5)
        print("Tracks after filtering:", tracks_np, flush=True)
        self.convert_ab3dmot_history_to_trajectories(
            tracks_np, trajectory_length=10, dt=self.dt)

        # ------------------------------------------------ prediction
        t1 = time.perf_counter()
        preds = self.linear_predictor_manager.generate_predicted_trajectories(
                     self.tracked_trajectories)
        predict_ms = (time.perf_counter() - t1) * 1000.0

        # ------------------------------------------------ debug/telemetry
        self.debug_helper.update_edge(0,
                                      tracking_time=tracker_ms,
                                      prediction_time=predict_ms)

        # ------------------------------------------------ forward to vehicles

        #print("Predictions generated:", preds, flush=True)

        for vm in self.vehicle_manager_list:
            vm.agent.generated_predictions[:] = [
                p for p in preds
                if not _belongs_to_vehicle(p, vm, self.track_to_carla)
            ]
            #print("Vehicle manager", vm, "has predictions:", vm.agent.generated_predictions, flush=True)

        print("Completed prediction step for edge", self.edgeid, flush=True)

        # ------------------------------------------------ apply control
        for vm in self.vehicle_manager_list:
            print("Running step for vehicle manager:", vm, flush=True)
            vm.update_info(step_id)
            control = vm.run_step()
            print("Applying control for vehicle manager:", vm, flush=True)
            vm.vehicle.apply_control(control)

        for rsu in self.rsu_manager_list:
            rsu.update_info()
            rsu.run_step()

    def convert_ab3dmot_history_to_trajectories(self, mot_output_deque, trajectory_length=10, dt=0.05):
        if not hasattr(self, "tracked_trajectories"):
            self.tracked_trajectories = {}

        updated_ids = set()

        for frame_tracks in mot_output_deque:
            if frame_tracks is None or len(frame_tracks) == 0:
                continue

            for track in frame_tracks:
                track_id = int(track[7])
                location = track[0:3]
                rotation_y = track[6]
                carla_id  = int(track[8])
                guid = int(track[9])

                loc = carla.Location(x=location[0], y=location[1], z=location[2])
                rot = carla.Rotation(yaw=np.degrees(rotation_y))
                transform = carla.Transform(location=loc, rotation=rot)

                updated_ids.add(track_id)

                if track_id in self.tracked_trajectories:
                    traj = self.tracked_trajectories[track_id]
                    traj.update(transform)
                    traj.trajectory = deque(traj.trajectory, maxlen=trajectory_length)
                    traj.obstacle.transform = transform
                    traj.obstacle.location = transform.location
                    traj.obstacle.carla_id = carla_id
                else:
                    dummy_obstacle = ObstacleVehicle(
                        corners=np.zeros((8, 3)),
                        o3d_bbx=None,
                        track_id=track_id,
                        tick_id=0
                    )
                    dummy_obstacle.transform = transform
                    dummy_obstacle.location = transform.location
                    dummy_obstacle.carla_id = carla_id
                    self.tracked_trajectories[track_id] = ObstacleTrajectory(
                        dummy_obstacle,
                        deque([transform], maxlen=trajectory_length)
                    )

        # Advance time for all tracked objects, and filter stale ones
        to_remove = []
        for track_id, traj in self.tracked_trajectories.items():
            if track_id not in updated_ids:
                traj.step(dt)
                if traj.time_since_update >= 2.0:
                    to_remove.append(track_id)

        for tid in to_remove:
            del self.tracked_trajectories[tid]


    def run_step_perception(self, step_id):
        if step_id < int(self.latency/self.dt):
            return
        #logger.debug("running perception_step edge")
        #logger.debug("Step ID: ", step_id)
        #logger.debug("Vehicle Manager List: ", self.vehicle_manager_list)
        
        number_of_steps = int(self.latency/self.dt)
        logger.debug("Number of Steps for Latency: ", number_of_steps)
        for idx, vehicle_manager in enumerate(self.vehicle_manager_list):
            #prnt("Objects Deque: ", self.objects_deque)
            objects_to_send = self.objects_deque[number_of_steps].copy()
            #logger.debug("Objects Deque After Copy: ", self.objects_deque)
            logger.debug("Objects to send: ", objects_to_send)
            #logger.debug("Vehicle %s" %idx)
            for object_type, object_list in objects_to_send.items():
                for obj in object_list:
                    if obj.get_location().distance(vehicle_manager.vehicle.get_location()) < 3:
                        object_list.remove(obj)
            vehicle_manager.edge_objects.clear()
            vehicle_manager.edge_objects = objects_to_send.copy()

            # Update vehicle manager agent trajectory dictionary
            vehicle_manager.agent.other_car_trajectories = self.traj_dict.copy()

            # Update vehicle manager agent vehicle speeds
            vehicle_manager.agent.other_car_speeds = self.vehicle_speeds.copy()

            # clear predictions
            # vehicle_manager.agent.generated_predictions.clear()

            # Apply Control; Run Step 
            vehicle_manager.update_info(step_id)
            control = vehicle_manager.run_step()
            vehicle_manager.vehicle.apply_control(control)
            #logger.debug("Applied control")
        for rsu in self.rsu_manager_list:
            rsu.update_info()
            rsu.run_step()
        #logger.debug("Objects Deque Before After Run Step: ", self.objects_deque)
        
        #for idx, vehicle_manager in enumerate(self.vehicle_manager_list):
        #  objects_to_send = self.objects.copy()
        #  if 'vehicles' in objects_to_send:
        #    logger.debug(len(objects_to_send['vehicles']))
        #  logger.debug("Vehicle %s" %idx)
        #  for object_type, object_list in objects_to_send.items():
        #    for obj in object_list:
        #      logger.debug("Object Distance: %s"%obj.get_location().distance(vehicle_manager.vehicle.get_location()))
        #      if obj.get_location().distance(vehicle_manager.vehicle.get_location()) < 3:
        #        object_list.remove(obj)
        #  vehicle_manager.edge_objects.clear()
        #  vehicle_manager.edge_objects = objects_to_send
        #  logger.debug(objects_to_send)
        #  if 'vehicles' in objects_to_send:
        #    logger.debug(len(objects_to_send['vehicles']))
        #  vehicle_manager.update_info(step_id)
        #  control = vehicle_manager.run_step()
        #  vehicle_manager.vehicle.apply_control(control)
        #  logger.debug("Applied control")
        #for rsu in self.rsu_manager_list:
        #  rsu.update_info()
        # rsu.run_step() 
          
          
          
    def run_step_maneuvering(self, step_id):
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
        self.algorithm_step()
        post_algo_time = time.time()
        logger.debug("Algorithm completion time: %s", (post_algo_time - pre_algo_time))
        self.debug_helper.update_edge((post_algo_time - pre_algo_time)*1000)
        all_waypoint_buffers = []
        #logger.debug("completed Algorithm Step")
        # output algorithm waypoints to waypoint buffer of each vehicle
        for idx, vehicle_manager in enumerate(self.vehicle_manager_list):
        #   # logger.debug(i)
        #   waypoint_buffer = vehicle_manager.agent.get_local_planner().get_waypoint_buffer()
        #   # logger.debug(waypoint_buffer)
        #   # for waypoints in waypoint_buffer:
        #   #   logger.debug("Waypoints transform for Vehicle Before Clearing: " + str(i) + " : ", waypoints[0].transform)
        #   waypoint_buffer.clear() #EDIT MADE 16/03
            waypoint_buffer_proto = ecloud.WaypointBuffer()
            waypoint_buffer_proto.vehicle_index = idx

            for k in range(0,1):
                waypoint_buffer_proto.waypoint_buffer.extend([serialize_waypoint(self.locations[idx*1+k])])#, RoadOption.STRAIGHT)) #Accounting for horizon of 4 here. To generate a waypoint _buffer_

            #logger.debug(waypoint_buffer_proto.SerializeToString())

            all_waypoint_buffers.append(waypoint_buffer_proto)
          # for waypoints in waypoint_buffer:
          #   logger.debug("Waypoints transform for Vehicle After Clearing: " + str(i) + " : ", waypoints[0].transform)
          # sys.exit()
          # # logger.debug(waypoint_buffer)

        return all_waypoint_buffers

        # #logger.debug("\n ########################\n")
        # #logger.debug("Length of vehicle manager list: ", len(self.vehicle_manager_list))

        # control_list = []
        # for i in range(len(self.vehicle_manager_list)):
        #     waypoints_buffer_logger.debuger = self.vehicle_manager_list[i].agent.get_local_planner().get_waypoint_buffer()
        #     #for waypoints in waypoints_buffer_logger.debuger:
        #         #logger.debug("Waypoints transform for Vehicle: " + str(i) + " : ", waypoints[0].transform)
        #     # logger.debug(self.vehicle_manager_list[i].agent.get_local_planner().get_waypoint_buffer().transform())
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
            The string that contains all evaluation results to logger.debug out.
        """

        #velocity_list = []
        #time_gap_list = []
        #distance_gap_list = []
        algorithm_time_list = []
        debug_helper = self.debug_helper

        perform_txt = ''

        for i in range(len(self.vehicle_manager_list)):
            vm = self.vehicle_manager_list[i]
            debug_helper = vm.agent.debug_helper

            # we need to filter out the first 100 data points
            # since the vehicles spawn at the beginning have
            # no velocity and thus make the time gap close to infinite

            #velocity_list += debug_helper.speed_list
            #time_gap_list += debug_helper.time_gap_list
            #distance_gap_list += debug_helper.dist_gap_list

            #time_gap_list_tmp = \
            #    np.array(debug_helper.time_gap_list)
            #time_gap_list_tmp = \
            #    time_gap_list_tmp[time_gap_list_tmp < 100]
            #distance_gap_list_tmp = \
            #    np.array(debug_helper.dist_gap_list)
            #distance_gap_list_tmp = \
            #    distance_gap_list_tmp[distance_gap_list_tmp < 100]

            #perform_txt += '\n Platoon member ID:%d, Actor ID:%d : \n' % (
            #    i, vm.vehicle.id)
            #perform_txt += 'Time gap mean: %f, std: %f \n' % (
            #    np.mean(time_gap_list_tmp), np.std(time_gap_list_tmp))
            #perform_txt += 'Distance gap mean: %f, std: %f \n' % (
            #    np.mean(distance_gap_list_tmp), np.std(distance_gap_list_tmp))


        algorithm_time_list += self.debug_helper.algorithm_time_list
        algorithm_time_list_tmp = \
                np.array(self.debug_helper.algorithm_time_list)
        algorithm_time_list_tmp = \
                algorithm_time_list_tmp[algorithm_time_list_tmp < 100]

        tracking_time_list = self.debug_helper.tracking_time_list
        tracking_time_list_mean = np.mean(tracking_time_list)
        tracking_time_list_std = np.std(tracking_time_list)

        prediction_time_list = self.debug_helper.prediction_time_list
        prediction_time_list_mean = np.mean(prediction_time_list)
        prediction_time_list_std = np.std(prediction_time_list)


        perform_txt += 'Algorithm time mean: %f, std: %f \n' % (
                np.mean(algorithm_time_list_tmp), np.std(algorithm_time_list_tmp))

        perform_txt += 'Tracking time mean: %f, std: %f \n' % (
                tracking_time_list_mean, tracking_time_list_std)

        perform_txt += 'Prediction time mean: %f, std: %f \n' % (
                prediction_time_list_mean, prediction_time_list_std)

        metrics = {
            'algorithm_time_mean': np.mean(algorithm_time_list_tmp),
            'algorithm_time_std': np.std(algorithm_time_list_tmp),
            'tracking_time_mean': tracking_time_list_mean,
            'tracking_time_std': tracking_time_list_std,
            'prediction_time_mean': prediction_time_list_mean,
            'prediction_time_std': prediction_time_list_std
        }


        figure = plt.figure()

        #plt.subplot(411)
        #open_plt.draw_velocity_profile_single_plot(velocity_list)

        plt.subplot(412)
        open_plt.draw_algorithm_time_profile_single_plot(algorithm_time_list)

        #plt.subplot(413)
        #open_plt.draw_time_gap_profile_singel_plot(time_gap_list)

        #plt.subplot(414)
        #open_plt.draw_dist_gap_profile_singel_plot(distance_gap_list)



        return figure, perform_txt, metrics

    def destroy(self):
        """
        Destroy edge vehicles actors inside simulation world.
        """
        for vm in self.vehicle_manager_list:
            vm.destroy()
