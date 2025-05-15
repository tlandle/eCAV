from collections import defaultdict, deque

#import erdos

import pylot.utils
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory


class ObstacleLocationHistoryManager:
    def __init__(self, vehicle, world):
        self._flags = flags
        # Queues in which received messages are stored.
        self._obstacles = deque()
        self._depth = deque()
        self._pose = deque()
        self._obstacle_history = defaultdict(deque)
        self._timestamp_history = deque()
        # Stores the id of obstacles that have values for a given timestamp.
        # This is used to GC the state from timestamp_history.
        self._timestamp_to_id = defaultdict(list)
        if hasattr(vehicle, 'get_world'):
            self.world = world if world is not None else vehicle.get_world()
        else:
            self.world = world

        self._map = self.world.get_map()

        self.cav_world = weakref.ref(cav_world)



    def destroy(self):
        self._logger.warn('destroying {}'.format(self.config.name))


    def track_obstacles(self, timestamp, tracked_obstacles):
        for obstacle_vehicle in tracked_obstacles:
            if (vehicle_transform.location.distance(
                    obstacle_vehicle.transform.location) >
                    self._flags.dynamic_obstacle_distance_threshold):
                continue
            # Store the obstacle history for each vehicle.
            self._obstacle_history[obstacle_vehicle.id].append(obstacle_vehicle)
            # Transform obstacle location from global world coordinates to
            # ego-centric coordinates.
            cur_obstacle_trajectory = []
            for obstacle_vehicle in self._obstacle_history[
                    obstacle_vehicle.id]:
                new_location = \
                    vehicle_transform.inverse_transform_locations(
                        [obstacle_vehicle.transform.location])[0]
                cur_obstacle_trajectory.append(
                    carla.Transform(new_location,
                                          carla.Rotation()))

                # Draw the trajectory of the obstacle.
            # The trajectory is relative to the current location.
            obstacle_trajectories.append(
                ObstacleTrajectory(obstacle_vehicle, cur_obstacle_trajectory))

            return obstacle_trajectories



