# -*- coding: utf-8 -*-
"""
Localization module
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

import random
import weakref
from collections import deque

import carla
import numpy as np

from ecav.core.common.misc import get_speed
from ecav.core.sensing.localization.localization_metrics \
    import LocalizationMetrics
from ecav.core.sensing.localization.kalman_filter import KalmanFilter
from ecav.core.sensing.localization.coordinate_transform \
    import geo_to_transform


class GnssSensor(object):
    """
    The default GNSS sensor module.

    Parameters
    ----------
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    config : dict
        The configuration dictionary of the localization module.

    Attributes
    ----------
    sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle, config):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.gnss')

        # set the noise for gps
        blueprint.set_attribute(
            'noise_alt_stddev', str(
                config['noise_alt_stddev']))
        blueprint.set_attribute(
            'noise_lat_stddev', str(
                config['noise_lat_stddev']))
        blueprint.set_attribute(
            'noise_lon_stddev', str(
                config['noise_lon_stddev']))
        # Do not seed GNSS noise — let CARLA randomize each run.
        # spawn the sensor
        self.sensor = world.spawn_actor(
            blueprint,
            carla.Transform(
                carla.Location(
                    x=0.0,
                    y=0.0,
                    z=0.0)),
            attach_to=vehicle,
            attachment_type=carla.AttachmentType.Rigid)

        # latitude and longitude at current timestamp
        self.lat, self.lon, self.alt, self.timestamp = 0.0, 0.0, 0.0, 0.0
        self._vehicle_id = vehicle.id
        _parent = self.sensor.parent
        print(f"[GNSS INIT] vehicle={vehicle.id} sensor={self.sensor.id} "
              f"parent={_parent.id if _parent else 'NONE'} "
              f"veh_loc=({vehicle.get_transform().location.x:.1f},"
              f"{vehicle.get_transform().location.y:.1f})", flush=True)

        # ----- Bursty GPS dropout simulation -----
        # gnss_dropout_pct: probability (0-100) per tick of entering dropout
        # gnss_dropout_duration_ticks: mean burst length (exponential dist)
        self._dropout_prob = config.get('gnss_dropout_pct', 0.0) / 100.0
        self._dropout_mean_ticks = config.get(
            'gnss_dropout_duration_ticks', 10)
        # Remaining ticks in current dropout burst (0 = not in dropout)
        self._dropout_remaining = 0

        # create weak reference to avoid circular reference
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: GnssSensor._on_gnss_event(
                weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        """GNSS method that returns the current geo location.

        When bursty GPS dropout is enabled (gnss_dropout_pct > 0), this
        callback may suppress the incoming GNSS reading and hold the
        last known position instead -- simulating signal loss in tunnels
        or urban canyons.

        Dropout model:
        - Each tick while *not* in dropout, a Bernoulli trial with
          probability ``gnss_dropout_pct / 100`` decides whether a new
          dropout burst begins.
        - When a burst starts its duration (in ticks) is drawn from an
          exponential distribution with mean
          ``gnss_dropout_duration_ticks`` (clamped to >= 1).
        - While the burst countdown is positive the GNSS update is
          suppressed (lat/lon/alt/timestamp retain their previous
          values, i.e. position-hold).
        """
        self = weak_self()
        if not self:
            return

        # --- Bursty dropout state machine ---
        if self._dropout_remaining > 0:
            # Currently inside a dropout burst -- suppress this reading
            self._dropout_remaining -= 1
            return

        if self._dropout_prob > 0.0 and random.random() < self._dropout_prob:
            # Start a new dropout burst
            duration = int(
                max(1, random.expovariate(1.0 / self._dropout_mean_ticks)))
            self._dropout_remaining = duration
            # Suppress this tick's reading as well (first tick of burst)
            return

        # Normal operation -- accept the GNSS reading
        self.lat = event.latitude
        self.lon = event.longitude
        self.alt = event.altitude
        self.timestamp = event.timestamp
        if not hasattr(self, '_cb_count'):
            self._cb_count = 0
        self._cb_count += 1
        if self._cb_count <= 5:
            print(f"[GNSS CB] veh={self._vehicle_id} cb#{self._cb_count} "
                  f"lat={event.latitude:.8f} lon={event.longitude:.8f} "
                  f"ts={event.timestamp:.3f}", flush=True)


class ImuSensor(object):
    """
    Default ImuSensor module.

    Parameters
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.

    Attributes
    world : carla.world
        The caral world of the current vehicle.

    blueprint : carla.blueprint
        The current blueprint of the sensor actor.

    weak_self : ecav Object
        A weak reference point to avoid circular reference.

    sensor : CARLA actor
        The current sensor actors that will be attach to the vehicles.
    """

    def __init__(self, vehicle):
        world = vehicle.get_world()
        blueprint = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            blueprint, carla.Transform(), attach_to=vehicle)

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: ImuSensor._IMU_callback(
                weak_self, sensor_data))
        self.gyroscope = None

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        """
        IMU method that returns the 3-D (x,y,z)
        acceleration and gyroscope values.
        """
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        # m/s^2
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        # rad/s
        self.gyroscope = (
            max(limits[0], min(limits[1], sensor_data.gyroscope.x)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.y)),
            max(limits[0], min(limits[1], sensor_data.gyroscope.z)))
        self.compass = sensor_data.compass


class LocalizationManager(object):
    """
    Default localization module.

    Parameters
    vehicle : carla.Vehicle
        The carla.Vehicle. We need this class to spawn our gnss and imu sensor.
    config_yaml : dict
        The configuration dictionary of the localization module.
    carla_map : carla.Map
        The carla HDMap. We need this to find the map origin to
        convert wg84 to enu coordinate system.

    Attributes
    gnss : ecav object
        GNSS sensor manager for spawning gnss sensor and listen to the data
        transmission.
    ImuSensor : ecav object
        Imu sensor manager for spawning gnss sensor and listen to the data
        transmission.
    kf : ecav object
        The filter used to fuse different sensors.
    debug_helper : ecav object
        The debug helper is used to visualize the accuracy of
        the localization and provide evaluation functions.
    """

    def __init__(self, vehicle, config_yaml, carla_map):

        self.vehicle = vehicle

        if hasattr(vehicle, 'get_world'):
            self.activate = config_yaml['activate']
            self.map = carla_map
            self.geo_ref = self.map.transform_to_geolocation(
                carla.Location(x=0, y=0, z=0))

            # speed and transform and current timestamp
            self._ego_pos = None
            self._speed = 0

            # history track
            self._ego_pos_history = deque(maxlen=100)
            self._timestamp_history = deque(maxlen=100)

            if self.activate:
                self.gnss = GnssSensor(vehicle, config_yaml['gnss'])
                self.imu = ImuSensor(vehicle)

            # heading direction noise
            self.heading_noise_std = \
                config_yaml['gnss']['heading_direction_stddev']
            self.speed_noise_std = config_yaml['gnss']['speed_stddev']

            self.dt = config_yaml['dt']
            #print("Kalman Value: %s",self.dt)
            # Kalman Filter
            self.kf = KalmanFilter(self.dt)

        # DebugHelper
        self.localization_metrics = LocalizationMetrics(
            config_yaml['debug_helper'], self.vehicle.id)

    def localize(self):
        """
        Currently implemented in a naive way.
        """

        if not self.activate:
            self._ego_pos = self.vehicle.get_transform()
            self._speed = get_speed(self.vehicle)
        else:
            speed_true = get_speed(self.vehicle)
            speed_noise = self.add_speed_noise(speed_true)

            # gnss coordinates under ESU(Unreal coordinate system)
            # Skip if GNSS hasn't initialized yet (returns 0,0,0)
            if self.gnss.lat == 0.0 and self.gnss.lon == 0.0:
                self._ego_pos = self.vehicle.get_transform()
                self._speed = get_speed(self.vehicle)
                return

            x, y, z = geo_to_transform(self.gnss.lat,
                                       self.gnss.lon,
                                       self.gnss.alt,
                                       self.geo_ref.latitude,
                                       self.geo_ref.longitude, 0.0)

            # Get ground truth location and convert to same coordinate frame as GNSS
            # CRITICAL: Must apply same geo_to_transform conversion to ensure
            # localization metrics compare apples-to-apples (both in Web Mercator)
            location = self.vehicle.get_transform().location
            gt_geo = self.map.transform_to_geolocation(location)
            gt_x, gt_y, gt_z = geo_to_transform(gt_geo.latitude,
                                                 gt_geo.longitude,
                                                 gt_geo.altitude,
                                                 self.geo_ref.latitude,
                                                 self.geo_ref.longitude, 0.0)

            # Diagnostic: every tick, compact format
            _n = len(self._ego_pos_history)
            _xerr = abs(x - gt_x)
            _yerr = abs(y - gt_y)
            _stuck = (hasattr(self, '_prev_gnss_lat') and
                      self.gnss.lat == self._prev_gnss_lat and
                      self.gnss.lon == self._prev_gnss_lon)
            print(f"[LOC] v={self.vehicle.id} t={_n} "
                  f"gnss=({x:.1f},{y:.1f}) gt=({gt_x:.1f},{gt_y:.1f}) "
                  f"carla=({location.x:.1f},{location.y:.1f}) "
                  f"err=({_xerr:.1f},{_yerr:.1f}) "
                  f"ts={self.gnss.timestamp:.3f} stuck={_stuck}", flush=True)
            self._prev_gnss_lat = self.gnss.lat
            self._prev_gnss_lon = self.gnss.lon


            # We add synthetic noise to the heading direction
            rotation = self.vehicle.get_transform().rotation
            heading_angle = self.add_heading_direction_noise(rotation.yaw)

            # assume the initial position is accurate
            if len(self._ego_pos_history) == 0:
                x_kf, y_kf, heading_angle_kf = x, y, heading_angle
                self._speed = speed_true
                self.kf.run_step_init(
                    x, y, np.deg2rad(heading_angle), self._speed / 3.6)
            else:
                x_kf, y_kf, heading_angle_kf, speed_kf = self.kf.run_step(
                    x, y, np.deg2rad(heading_angle),
                    speed_noise / 3.6,
                    self.imu.gyroscope[2])
                self._speed = speed_kf * 3.6
                heading_angle_kf = np.rad2deg(heading_angle_kf)
            #print("Calculated speed: ", self._speed)
            #print("Actual Speed: " , get_speed(self.vehicle))

            # add data to debug helper (both GNSS and ground truth now in same frame)
            self.localization_metrics.run_step(x,
                                       y,
                                       heading_angle,
                                       speed_noise,
                                       x_kf,
                                       y_kf,
                                       heading_angle_kf,
                                       self._speed,
                                       gt_x,
                                       gt_y,
                                       rotation.yaw,
                                       speed_true)

            # the final pose of the vehicle
            self._ego_pos = carla.Transform(
                carla.Location(
                    x=x_kf, y=y_kf, z=z), carla.Rotation(
                    pitch=0, yaw=rotation.yaw, roll=0))
            #print("Final Pose(GNSS): (%s, %s, %s)" %(x_kf, y_kf, z))

            # save the track for future use
            self._ego_pos_history.append(self._ego_pos)
            self._timestamp_history.append(self.gnss.timestamp)

    def add_heading_direction_noise(self, heading_direction):
        """
        Add synthetic noise to heading direction.

        Parameters
        __________
        heading_direction : float
            groundtruth heading_direction obtained from the server.

        Returns
        -------
        heading_direction : float
            heading direction with noise.
        """
        return heading_direction + np.random.normal(0, self.heading_noise_std)

    def add_speed_noise(self, speed):
        """
        Add gaussian white noise to the current speed.

        Parameters
        __________
        speed : float
            m/s, current speed.

        Returns
        -------
        speed : float
            the speed with noise.
        """
        return speed + np.random.normal(0, self.speed_noise_std)

    def get_ego_pos(self):
        """
        Retrieve ego vehicle position
        """
        return self._ego_pos

    def get_ego_spd(self):
        """
        Retrieve ego vehicle speed
        """
        return self._speed

    def destroy(self):
        """
        Destroy the sensors
        """
        if hasattr(self, 'gnss'):
            self.gnss.sensor.destroy()
        if hasattr(self, 'imu'):
            self.imu.sensor.destroy()
