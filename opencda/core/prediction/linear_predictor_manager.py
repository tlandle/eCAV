"""Implements an operator that fits a linear model to predict trajectories."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import carla
import numpy as np

from opencda.core.prediction.obstacle_prediction import ObstaclePrediction


class LinearPredictorManager():
    """Operator that implements a linear predictor.

    It takes (x,y) locations of agents in past, and fits a linear model to
    these locations.

    Args:
        tracking_stream (:py:class:`erdos.ReadStream`): The stream on which
            :py:class:`~pylot.perception.messages.ObstacleTrajectoriesMessage`
            are received.
        linear_prediction_stream (:py:class:`erdos.WriteStream`): Stream on
            which the operator sends
            :py:class:`~pylot.prediction.messages.PredictionMessage` messages.
        flags (absl.flags): Object to be used to access absl flags.
    """
    def __init__(self, num_future_steps=25):
        self.num_predicted_steps = num_future_steps

    def generate_predicted_trajectories(self, tracked_obstacles_trajectories):
        """
        Generate linear predicted trajectories for tracked obstacles in world coordinates.

        Args:
            tracked_obstacles_trajectories (Dict[int, ObstacleTrajectory]):
                Dictionary mapping track_id to ObstacleTrajectory.

        Returns:
            List[ObstaclePrediction]: Future trajectory predictions per obstacle.
        """
        obstacle_predictions_list = []

        for obstacle_trajectory in tracked_obstacles_trajectories.values():
            trajectory = obstacle_trajectory.trajectory
            num_steps = len(trajectory)
            if num_steps < 2:
                continue  # Not enough history to predict

            # Time matrices
            ts = np.zeros((num_steps, 2))
            future_ts = np.zeros((self.num_predicted_steps, 2))
            for t in range(num_steps):
                ts[t] = [-t, 1]  # regression over time
            for i in range(self.num_predicted_steps):
                future_ts[i] = [i + 1, 1]

            # Position matrix: xy[t] = [x, y]
            xy = np.array([[tf.location.x, tf.location.y] for tf in reversed(trajectory)])

            # Linear regression for x and y
            linear_model_params = np.linalg.lstsq(ts, xy, rcond=None)[0]
            predict_array = future_ts @ linear_model_params

            # Use latest yaw angle
            latest_transform = trajectory[-1]
            rotation = latest_transform.rotation

            # Convert predicted points to future Transforms
            predictions = [
                carla.Transform(location=carla.Location(x=pt[0], y=pt[1], z=latest_transform.location.z),
                                rotation=rotation)
                for pt in predict_array
            ]

            # Create prediction object
            obstacle_predictions_list.append(
                ObstaclePrediction(obstacle_trajectory,
                                   latest_transform,
                                   probability=1.0,
                                   predicted_trajectory=predictions)
            )

        return obstacle_predictions_list

