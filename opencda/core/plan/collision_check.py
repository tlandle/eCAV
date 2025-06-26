""" -*- coding: utf-8 -*- """
""" This module is used to check collision possibility """

# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
import math
from math import sin, cos
from scipy import spatial
from scipy.interpolate import interp1d
import coloredlogs, logging

import carla
import numpy as np

from opencda.core.plan.local_planner_behavior import RoadOption
from opencda.core.common.misc import cal_distance_angle, draw_trajetory_points
from opencda.core.plan.spline import Spline2D


logger = logging.getLogger(__name__)
coloredlogs.install(level='ERROR', logger=logger)

def create_waypoint_roadoption_tuple(vehicle, carla_map):
    # Get the current location of the vehicle
    location = vehicle.get_location()
    
    # Get the waypoint closest to the vehicle's location
    waypoint = carla_map.get_waypoint(location)
    
    # Determine the road option (e.g., straight, left, right, etc.)
    # For simplicity, we'll assume a default road option
    road_option = RoadOption.LANEFOLLOW
    
    return (waypoint, road_option)

def get_next_waypoints_positions(waypoints_deque, num_waypoints=10):
    positions = []
    for i in range(min(num_waypoints, len(waypoints_deque))):
        waypoint = waypoints_deque[i][0]
        x = waypoint.transform.location.x
        y = waypoint.transform.location.y
        positions.append((x, y))
    return positions

def get_positions_from_deque(waypoints_deque, velocity_vector, time_intervals):
    positions = []
    for t in time_intervals:
        distance = np.linalg.norm([velocity_vector.x, velocity_vector.y, velocity_vector.z]) * t
        for i in range(len(waypoints_deque) - 1):
            waypoint1 = waypoints_deque[i][0]
            waypoint2 = waypoints_deque[i+1][0]
            segment = waypoint2.transform.location.distance(waypoint1.transform.location)
            if distance <= segment:
                ratio = distance / segment
                x = waypoint1.transform.location.x + ratio * (waypoint2.transform.location.x - waypoint1.transform.location.x)
                y = waypoint1.transform.location.y + ratio * (waypoint2.transform.location.y - waypoint1.transform.location.y)
                positions.append((x, y))
                break
            else:
                distance -= segment
    return positions


def interpolate_positions(waypoints_deque, num_waypoints=10, interval=3.0):
    positions = []
    distance_covered = 0.0

    for i in range(len(waypoints_deque) - 1):
        if len(positions) >= num_waypoints:
            break

        waypoint1 = waypoints_deque[i][0]
        waypoint2 = waypoints_deque[i+1][0]
        segment_length = waypoint2.transform.location.distance(waypoint1.transform.location)

        while distance_covered + interval <= segment_length:
            ratio = (distance_covered + interval) / segment_length
            x = waypoint1.transform.location.x + ratio * (waypoint2.transform.location.x - waypoint1.transform.location.x)
            y = waypoint1.transform.location.y + ratio * (waypoint2.transform.location.y - waypoint1.transform.location.y)
            positions.append((x, y))
            distance_covered += interval

        distance_covered -= segment_length

    return positions

def linear_interp_trajectory(points):
    x = points[:, 0]
    y = points[:, 1]

    # compute arc lengths
    diffs = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    s = np.insert(np.cumsum(segment_lengths), 0, 0.0)

    # parametric interpolators x(s) and y(s)
    xs = interp1d(s, x, kind='linear')
    ys = interp1d(s, y, kind='linear')

    # get arc points
    ds = 0.1
    total_length = s[-1]
    num_steps = int(total_length / ds)
    sp = np.linspace(0, num_steps * ds, num_steps + 1)

    # get path
    rx = xs(sp)
    ry = ys(sp)
    return sp, np.stack((rx, ry), axis=1)

def time_reparametrize(points, sp, speed, min_speed=3):
    tp = sp / max(speed, min_speed)
    x, y = points[:, 0], points[:, 1]
    xt = interp1d(tp, x, kind='linear')
    yt = interp1d(tp, y, kind='linear')
    return xt, yt, tp

def lookahead_interp(xt, yt, time_ahead, dt=0.05, t_max=None):
    t_end = min(time_ahead, t_max)
    num_steps = int(t_end / dt)
    tp = np.linspace(0, num_steps * dt, num_steps + 1)
    x = xt(tp)
    y = yt(tp)
    return x, y

def check_paths_within_radius(path_a, path_b, r=1, dt=0.05):
    diffs = path_a - path_b
    dists = np.linalg.norm(diffs, axis=1)
    is_collision = np.any(dists < r)

    collision_indices = np.where(dists < r)[0]
    if is_collision:
        ttc = collision_indices[0] * dt
    else:
        ttc = None

    return is_collision, ttc

def interpolate_positions_points(waypoints_deque, num_points=10, num_waypoints=10):

    positions = []
    total_length = 0.0

    # Calculate the total length of the specified number of waypoints
    for i in range(min(num_waypoints, len(waypoints_deque)) - 1):
        waypoint1 = waypoints_deque[i][0]
        waypoint2 = waypoints_deque[i+1][0]
        total_length += waypoint2.transform.location.distance(waypoint1.transform.location)

    interval = total_length / (num_points - 1)
    distance_covered = 0.0

    for i in range(min(num_waypoints, len(waypoints_deque)) - 1):
        if len(positions) >= num_points:
            break

        waypoint1 = waypoints_deque[i][0]
        waypoint2 = waypoints_deque[i+1][0]
        segment_length = waypoint2.transform.location.distance(waypoint1.transform.location)

        while distance_covered + interval <= segment_length:
            ratio = (distance_covered + interval) / segment_length
            x = waypoint1.transform.location.x + ratio * (waypoint2.transform.location.x - waypoint1.transform.location.x)
            y = waypoint1.transform.location.y + ratio * (waypoint2.transform.location.y - waypoint1.transform.location.y)
            positions.append((x, y))
            distance_covered += interval

        distance_covered -= segment_length

    # Ensure the last point is exactly at the end of the trajectory
    if len(positions) < num_points:
        last_waypoint = waypoints_deque[min(num_waypoints, len(waypoints_deque)) - 1][0]
        positions.append((last_waypoint.transform.location.x, last_waypoint.transform.location.y))

    return positions

def check_intersection_with_path(positions, path_x, path_y, path_yaw):
    logger.debug("Positions length: %s" %len(positions))
    logger.debug("Path X length: %s" %len(path_x))
    for pos, (px, py, pyaw) in zip(positions, zip(path_x, path_y, path_yaw)):
        #logger.debug("Obstacle Vehicle Position: %s" %pos)
        #logger.debug("Ego Vehicle Position: %s" %(px, py))
        logger.debug("Distance: %s" %np.linalg.norm(np.array(pos) - np.array([px, py])))
        if np.linalg.norm(np.array(pos) - np.array([px, py])) < 3.0:  # Assuming a threshold for intersection
            return True
    return False

def check_intersection_with_path_v2(positions, path_x, path_y, path_yaw):
    for pos, (px, py, pyaw) in zip(positions, zip(path_x, path_y, path_yaw)):
        logger.debug("Distance: %s" %np.linalg.norm(np.array(pos) - np.array([px, py])))
        if carla.Location(x=pos[0], y=pos[1]).distance(carla.Location(x=px, y=py)) < 6.0:
            return True
    return False

def perpendicular_line_arrays(x, y, yaw, num_points=10, distance=1.0):
    """
    Calculates points along a perpendicular line and returns separate arrays for x, y, and yaw.

    Args:
        x: x-coordinate of the original point.
        y: y-coordinate of the original point.
        yaw: Yaw angle (in radians) of the original point.
        num_points: Number of points to generate along the perpendicular line.
        distance: Distance between each point on the perpendicular line.

    Returns:
        A tuple containing three arrays: (x_array, y_array, yaw_array).
        Returns None if num_points is not a positive integer.
    """

    if not isinstance(num_points, int) or num_points <= 0:
        logger.debug("num_points must be a positive integer.")
        return None

    perpendicular_yaw = yaw + math.pi / 2  # Calculate perpendicular yaw
    perpendicular_yaw = math.atan2(math.sin(perpendicular_yaw), math.cos(perpendicular_yaw)) # Normalize

    x_array = []
    y_array = []
    yaw_array = []

    for i in range(num_points):
        offset = (i - (num_points - 1) / 2) * distance # offset to center the points around the original point.
        px = x + offset * math.cos(perpendicular_yaw)
        py = y + offset * math.sin(perpendicular_yaw)
        x_array.append(px)
        y_array.append(py)
        yaw_array.append(perpendicular_yaw)  # Yaw remains constant along the perpendicular

    return np.array(x_array), np.array(y_array), np.array(yaw_array)

class CollisionChecker:
    """
    The default collision checker module.

    Parameters
    ----------
    time_ahead : float
        how many seconds we look ahead in advance for collision check.
    circle_radius : float
        The radius of the collision checking circle.
    circle_offsets : float
        The offset between collision checking circle and the trajectory point.
    """

    def __init__(self, time_ahead=1.2, circle_radius=1.0, circle_offsets=None):

        self.time_ahead = time_ahead
        self._circle_offsets = [-.8,
                                0,
                                .8] \
            if circle_offsets is None else circle_offsets
        self._circle_radius = circle_radius


    def is_in_range(
            self,
            ego_pos,
            target_vehicle,
            candidate_vehicle,
            carla_map):
        """
        Check whether there is a obstacle vehicle between target_vehicle
        and ego_vehicle during back_joining.

        Parameters
        ----------
        carla_map : carla.map
            Carla map  of the current simulation world.

        ego_pos : carla.transform
            Ego vehicle position.

        target_vehicle : carla.vehicle
            The target vehicle that ego vehicle trying to catch up with.

        candidate_vehicle : carla.vehicle
            The possible obstacle vehicle blocking the ego vehicle
            and target vehicle.

        Returns
        -------
        detection result : boolean
        Indicator of whether the target vehicle is in range.
        """
        ego_loc = ego_pos.location
        target_loc = target_vehicle.get_location()
        candidate_loc = candidate_vehicle.get_location()

        # set the checking rectangle
        min_x, max_x = min(
            ego_loc.x, target_loc.x), max(
            ego_loc.x, target_loc.x)
        min_y, max_y = min(
            ego_loc.y, target_loc.y), max(
            ego_loc.y, target_loc.y)

        # give a small buffer of 2 meters
        if candidate_loc.x <= min_x - 2 or candidate_loc.x >= max_x + 2 or \
                candidate_loc.y <= min_y - 2 or candidate_loc.y >= max_y + 2:
            return False

        candidate_wpt = carla_map.get_waypoint(candidate_loc)
        target_wpt = carla_map.get_waypoint(target_loc)

        # if the candidate vehicle is right behind the target vehicle, then it
        # is blocking
        if target_wpt.lane_id == candidate_wpt.lane_id:
            return True

        # in case they are in the same lane but section id is different which
        # changes their id
        if target_wpt.section_id == candidate_wpt.section_id:
            return False

        # check the angle
        distance, angle = cal_distance_angle(
            target_wpt.transform.location, candidate_wpt.transform.location,
            candidate_wpt.transform.rotation.yaw)

        return True if angle <= 3 else False

    def adjacent_lane_collision_check(
            self, ego_loc, target_wpt, overtake, carla_map, world, oncoming_lane=False):
        """
        Generate a straight line in the adjacent lane for collision detection
        during overtake/lane change.

        Args:
            -ego_loc (carla.Location): Ego Location.
            -target_wpt (carla.Waypoint): the check point in the adjacent
             at a far distance.
            -overtake (bool): indicate whether this is an overtake or normal
             lane change behavior.
            -world (carla.World): CARLA Simulation world,
             used to draw debug lines.

        Returns:
            -rx (list): the x coordinates of the collision check line in
             the adjacent lane
            -ry (list): the y coordinates of the collision check line in
             the adjacent lane
            -ryaw (list): the yaw angle of the the collision check line in
             the adjacent lane
        """
        # we first need to consider the vehicle on the other lane in front
        if overtake:
            if oncoming_lane:
                logger.debug("Oncoming lane true")
                target_wpt_next = target_wpt.previous(6)[0]
            else:
                target_wpt_next = target_wpt.next(6)[0]
        else:
            target_wpt_next = target_wpt

        # Next we consider the vehicle behind us
        diff_x = target_wpt_next.transform.location.x - ego_loc.x
        diff_y = target_wpt_next.transform.location.y - ego_loc.y
        diff_s = np.hypot(diff_x, diff_y) + 3

        if oncoming_lane:
            target_wpt_previous = target_wpt.next(diff_s)
        else:
            target_wpt_previous = target_wpt.previous(diff_s)

        while len(target_wpt_previous) == 0:
            diff_s -= 2
            if oncoming_lane:
                target_wpt_previous = target_wpt.next(diff_s)
            else:
                target_wpt_previous = target_wpt.previous(diff_s)


        target_wpt_previous = target_wpt_previous[0]
        if oncoming_lane:
            target_wpt_middle = target_wpt_previous.previous(diff_s/2)[0]
        else:
            target_wpt_middle = target_wpt_previous.next(diff_s/2)[0]

        x, y = [target_wpt_next.transform.location.x,
                target_wpt_middle.transform.location.x,
                target_wpt_previous.transform.location.x], \
               [target_wpt_next.transform.location.y,
                target_wpt_middle.transform.location.y,
                target_wpt_previous.transform.location.y]
        ds = 0.1

        sp = Spline2D(x, y)
        s = np.arange(sp.s[0], sp.s[-1], ds)

        debug_tmp = []

        # calculate interpolation points
        rx, ry, ryaw = [], [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(sp.calc_yaw(i_s))
            debug_tmp.append(carla.Transform(carla.Location(ix, iy, 0)))

        #draw yellow line for overtaking, white line for lane change
        draw_trajetory_points(
             world, debug_tmp, color=carla.Color(
                 255, 255, 0) if overtake else carla.Color(
                 255, 255, 255), size=0.05, lt=0.2)
    
        #world.tick()
        #input()
        return rx, ry, ryaw

    def collision_circle_check(
            self,
            path_x,
            path_y,
            path_yaw,
            obstacle_vehicle,
            speed,
            carla_map,
            adjacent_check=False,
            is_left_turn_at_intersection=False,
            world=None):
        """
        Use circled collision check to see whether potential hazard on
        the forwarding path.

        Args:
            -adjacent_check (boolean): Indicator of whether do adjacent check.
             Note: always give full path for adjacent lane check.
            -speed (float): ego vehicle speed in m/s.
            -path_yaw (float): a list of yaw angles
            -path_x (list): a list of x coordinates
            -path_y (list): a list of y coordinates
            -obstacle_vehicle (carla.vehicle): potention hazard vehicle
             on the way
        Returns:
            -collision_free (boolean): Flag indicate whether the
             current range is collision free.
        """
        collision_free = True
        # detect x second ahead. in case the speed is very slow,
        # there is some minimum threshold for the check distance
        distance_check = min(max(int(self.time_ahead * speed / 0.1), 90),
                             len(path_x)) \
            if not adjacent_check else len(path_x)

        #logger.debug("Distance Check: %s" %distance_check)

        #create path for intersection check that is perpendicular to the path of the ego vehicle

        obstacle_vehicle_loc = obstacle_vehicle.get_location()
        logger.debug("Obstacle_vehicle Location (%s, %s, %s)" %(obstacle_vehicle_loc.x, obstacle_vehicle_loc.y, obstacle_vehicle_loc.z))
        #logger.debug("Self Location (%s, %s, %s)" %(
        obstacle_vehicle_yaw = \
            carla_map.get_waypoint(obstacle_vehicle_loc).transform.rotation.yaw


        # every step is 0.1m, so we check every 10 points
        for i in range(0, distance_check, 10):
            ptx, pty, yaw = path_x[i], path_y[i], path_yaw[i]
            #if is_left_turn_at_intersection:
                #yaw = yaw - 90

            circle_locations = np.zeros((len(self._circle_offsets), 2))
            circle_offsets = np.array(self._circle_offsets)
            circle_locations[:, 0] = ptx + circle_offsets * cos(yaw)
            circle_locations[:, 1] = pty + circle_offsets * sin(yaw)

            # calculate bbx coords under world coordinate system
            corrected_extent_x = obstacle_vehicle.bounding_box.extent.x * \
                                 math.cos(math.radians(obstacle_vehicle_yaw))
            corrected_extent_y = obstacle_vehicle.bounding_box.extent.y * \
                                 math.sin(math.radians(obstacle_vehicle_yaw))

            # we need compute the four corner of the bbx
            obstacle_vehicle_bbx_array = \
                np.array([[obstacle_vehicle_loc.x -
                           corrected_extent_x,
                           obstacle_vehicle_loc.y -
                           corrected_extent_y],
                          [obstacle_vehicle_loc.x -
                           corrected_extent_x,
                           obstacle_vehicle_loc.y +
                           corrected_extent_y],
                          [obstacle_vehicle_loc.x,
                           obstacle_vehicle_loc.y],
                          [obstacle_vehicle_loc.x +
                           corrected_extent_x,
                           obstacle_vehicle_loc.y -
                           corrected_extent_y],
                          [obstacle_vehicle_loc.x +
                           corrected_extent_x,
                           obstacle_vehicle_loc.y +
                           corrected_extent_y]])

            #logger.debug(obstacle_vehicle_bbx_array)

            # compute whether the distance between the four corners of the
            # vehicle to the trajectory point
            collision_dists = spatial.distance.cdist(
                obstacle_vehicle_bbx_array, circle_locations)

            collision_dists = np.subtract(collision_dists, self._circle_radius)
            collision_free = collision_free and not np.any(collision_dists < 0)

            if not collision_free:
                #world.debug.draw_point(obstacle_vehicle_loc, size=1, life_time=1.0)
                logger.debug(collision_dists)
                #break

        return collision_free
    
    def waypoint_collision_check(
            self,
            ego_waypoints,
            ego_loc,
            ego_speed,
            obstacle_trajectory,
            obstacle_speed,
            world=None):
        """
        Check whether the vehicle's path along desired waypoints intersects
        with the predicted path of an obstacle vehicle.
        """
        # get ego points
        ego_points = []
        ego_points.append([ego_loc.x, ego_loc.y])
        for wpt, _ in ego_waypoints:
            x = wpt.transform.location.x
            y = wpt.transform.location.y
            ego_points.append([x, y])
        ego_points = np.asarray(ego_points)
        ego_x = ego_points[:, 0]
        ego_y = ego_points[:, 1]

        print("ego waypoints: %s" %ego_points)

        # get spline
        ds = 0.1
        sp = Spline2D(ego_x, ego_y)
        s = np.arange(sp.s[0], sp.s[-1], ds)

        # calculate interpolation points
        rx, ry = [], []
        for i_s in s:
            ix, iy = sp.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)

        is_collision, ttc = self.trajectory_collision_check(rx, ry, ego_speed, obstacle_trajectory, obstacle_speed, check_full_path=True, world=world)
        return is_collision, ttc

    def trajectory_collision_check(
            self,
            ego_path_x, 
            ego_path_y,
            ego_speed,
            obstacle_trajectory,
            obstacle_speed,
            time_step=0.05,
            world=None,
            check_full_path=False):
        """
        Check whether the vehicle will collide with the obstacle vehicle
        in the future.
        """
        is_collision = False
        ttc = 1000

        # get interpolation of obstacle trajectory
        obs_points = []
        for i, pred_transform in enumerate(obstacle_trajectory):
            x = pred_transform.location.x
            y = pred_transform.location.y
            obs_points.append([x, y])
        obs_points = np.asarray(obs_points)
        obs_sp, obs_path = linear_interp_trajectory(obs_points)

        # print("Obstacle Path Length: %s" %len(obs_path))

        if check_full_path or ego_speed < 3:
            logger.debug("check full paths")

            ego_x_points = ego_path_x
            ego_y_points = ego_path_y
            ego_path = np.stack((ego_x_points, ego_y_points), axis=1)

            # get lookahead for obstacle path
            obs_path_x = obs_path[:, 0]
            obs_path_y = obs_path[:, 1]
            obs_distance_check = min(max(50, int(self.time_ahead * obstacle_speed / 0.1)), len(obs_path_x))
            obs_x_points = obs_path_x[:obs_distance_check]
            obs_y_points = obs_path_y[:obs_distance_check]
            obs_path = np.stack((obs_x_points, obs_y_points), axis=1)

            # check collision (naive arc-based)
            dists = spatial.distance.cdist(ego_path, obs_path)
            collision_dists = np.subtract(dists, self._circle_radius * 2)
            is_collision = np.any(collision_dists < 0)
        else:
            # naively resample the ego path
            ego_distance_check = min(max(int(self.time_ahead * ego_speed / 0.1), 50), len(ego_path_x))    # minimum number of points can be tuned
            ego_x_points = ego_path_x[:ego_distance_check]
            ego_y_points = ego_path_y[:ego_distance_check]
            ego_points = np.stack((ego_x_points, ego_y_points), axis=1)
            ego_sp, ego_path = linear_interp_trajectory(ego_points)

            # check for intersection point
            dists = spatial.distance.cdist(ego_path, obs_path)
            intersection = np.any(dists < 2)
            if intersection:
                # time reparameterize
                ego_xt, ego_yt, ego_tp = time_reparametrize(ego_path, ego_sp, ego_speed)
                obs_xt, obs_yt, obs_tp = time_reparametrize(obs_path, obs_sp, obstacle_speed)

                # get lookahead paths
                ego_x_points, ego_y_points = lookahead_interp(ego_xt, ego_yt, self.time_ahead, t_max=ego_tp[-1], dt=time_step)
                obs_x_points, obs_y_points = lookahead_interp(obs_xt, obs_yt, self.time_ahead, t_max=obs_tp[-1], dt=time_step)

                # check collision (time-based)
                length = min(len(ego_x_points), len(obs_x_points))
                ego_path = np.stack((ego_x_points[:length], ego_y_points[:length]), axis=1)
                obs_path = np.stack((obs_x_points[:length], obs_y_points[:length]), axis=1)
                proximity_check = min(10, ego_speed / 1.6)
                is_collision, ttc = check_paths_within_radius(ego_path, obs_path, r=proximity_check, dt=time_step)
            else:
                obs_x_points = obs_path[:, 0]
                obs_y_points = obs_path[:, 1]

        if world is not None:
            for i in range(len(ego_x_points)):
                #print("Ego Vehicle Point: (%s, %s)" %(ego_x_points[i], ego_y_points[i]))
                world.debug.draw_point(carla.Location(x=ego_x_points[i], y=ego_y_points[i], z=.5), color=carla.Color(0,255,255), size=0.1, life_time=0.25)
            for i in range(len(obs_x_points)):
                #print("Obstacle Vehicle Point: (%s, %s)" %(obs_x_points[i], obs_y_points[i]))
                world.debug.draw_point(carla.Location(x=obs_x_points[i], y=obs_y_points[i], z=.5), color=carla.Color(255,0,0), size=0.1, life_time=0.25)

        return is_collision, ttc


