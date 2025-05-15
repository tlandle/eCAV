import carla

#from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory
#from pylot.utils import Transform
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory


class ObstaclePrediction(object):
    """Class storing info about an obstacle prediction.

    Args:
        obstacle_trajectory (:py:class:`~pylot.perception.tracking.obstacle_trajectory.ObstacleTrajectory`):  # noqa: E501
            Trajectory of the obstacle.
        transform (:py:class:`~pylot.utils.Transform`): The current transform
            of the obstacle.
        probability (:obj: `float`): The probability of the prediction.
        predicted_trajectory (list(:py:class:`~pylot.utils.Transform`)): The
            predicted future trajectory.
    """
    def __init__(self, obstacle_trajectory: ObstacleTrajectory,
                 transform: carla.Transform, probability: float,
                 predicted_trajectory):
        # Trajectory in ego frame of coordinates.
        self.obstacle_trajectory = obstacle_trajectory
        # The transform is in world coordinates.
        self.transform = transform
        self.probability = probability
        # Predicted trajectory in ego frame of coordinates.
        self.predicted_trajectory = predicted_trajectory

    @property
    def id(self):
        return self.obstacle_trajectory.obstacle.id

    @property
    def label(self):
        return self.obstacle_trajectory.obstacle.label

    def is_animal(self):
        return self.obstacle_trajectory.obstacle.is_animal()

    def is_person(self):
        return self.obstacle_trajectory.obstacle.is_person()

    def is_speed_limit(self):
        return self.obstacle_trajectory.obstacle.is_speed_limit()

    def is_stop_sign(self):
        return self.obstacle_trajectory.obstacle.is_stop_sign()

    def is_traffic_light(self):
        return self.obstacle_trajectory.obstacle.is_traffic_light()

    def is_vehicle(self):
        return self.obstacle_trajectory.obstacle.is_vehicle()

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return ('Prediction for obstacle {}, probability {}, '
                'predicted trajectory {}'.format(
                    self.obstacle_trajectory.obstacle, self.probability,
                    self.predicted_trajectory))
