"""Implements an operator that predicts trajectories using KF velocity."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import numpy as np

from opencda.core.prediction.obstacle_prediction import ObstaclePrediction
from opencda.opencda_carla import Location, Rotation, Transform

logger = logging.getLogger(__name__)

# Minimum track history required before generating a prediction.
_MIN_HISTORY = 3

# Minimum KF-estimated speed (m/s) to generate a prediction.
# Objects below this are considered stationary by the tracker's own
# Kalman filter velocity estimate.
_MIN_KF_SPEED_MPS = 1.0


class LinearPredictorManager():
    """Predicts future trajectories using Kalman filter velocity.

    Uses the tracker's KF velocity estimate (kf_vx, kf_vy) for constant-
    velocity extrapolation from the current position.  Falls back to
    least-squares regression on historical positions when KF velocity
    components are unavailable.

    KF velocity is preferred over regression because regression fits to
    ALL historical positions equally — including early frames with large
    monocular-depth errors.  As depth estimates improve over time, the
    regression interprets depth convergence as real motion, producing
    diagonal predicted trajectories.  The KF velocity reflects the
    filter's current best estimate and is not biased by old positions.
    """
    def __init__(self, num_future_steps=60):
        # 60 steps at 0.05s/step = 3 seconds prediction horizon
        self.num_predicted_steps = num_future_steps

    def generate_predicted_trajectories(self, tracked_obstacles_trajectories):
        """
        Generate predicted trajectories for tracked obstacles in world coordinates.

        Args:
            tracked_obstacles_trajectories (Dict[int, ObstacleTrajectory]):
                Dictionary mapping track_id to ObstacleTrajectory.

        Returns:
            List[ObstaclePrediction]: Future trajectory predictions per obstacle.
        """
        obstacle_predictions_list = []

        for obstacle_trajectory in tracked_obstacles_trajectories.values():
            # Gate on KF velocity: skip objects the tracker considers stationary
            kf_speed = getattr(obstacle_trajectory.obstacle, 'kf_speed_mps', None)
            if kf_speed is not None and kf_speed < _MIN_KF_SPEED_MPS:
                logger.debug("[PRED SKIP] track_id=%s kf_speed=%.2f m/s (stationary)",
                             obstacle_trajectory.obstacle.track_id, kf_speed)
                continue

            trajectory = obstacle_trajectory.trajectory
            num_steps = len(trajectory)
            if num_steps < _MIN_HISTORY:
                continue

            # Use latest position and orientation (trajectory[0] is newest)
            latest_transform = trajectory[0]
            rotation = latest_transform.rotation
            cur_x = latest_transform.location.x
            cur_y = latest_transform.location.y
            cur_z = latest_transform.location.z

            # Prefer KF velocity extrapolation over regression
            kf_vx = getattr(obstacle_trajectory.obstacle, 'kf_vx', None)
            kf_vy = getattr(obstacle_trajectory.obstacle, 'kf_vy', None)

            if kf_vx is not None and kf_vy is not None:
                # KF velocity (m/tick): extrapolate from current position
                predictions = [
                    Transform(
                        location=Location(
                            x=cur_x + kf_vx * (i + 1),
                            y=cur_y + kf_vy * (i + 1),
                            z=cur_z),
                        rotation=Rotation(
                            roll=rotation.roll,
                            pitch=rotation.pitch,
                            yaw=rotation.yaw))
                    for i in range(self.num_predicted_steps)
                ]
            else:
                # Fallback: least-squares regression on historical positions
                ts = np.zeros((num_steps, 2))
                future_ts = np.zeros((self.num_predicted_steps, 2))
                for t in range(num_steps):
                    ts[t] = [-t, 1]
                for i in range(self.num_predicted_steps):
                    future_ts[i] = [i + 1, 1]

                xy = np.array([[tf.location.x, tf.location.y] for tf in trajectory])
                linear_model_params = np.linalg.lstsq(ts, xy, rcond=None)[0]
                predict_array = future_ts @ linear_model_params

                predictions = [
                    Transform(
                        location=Location(x=pt[0], y=pt[1], z=cur_z),
                        rotation=Rotation(
                            roll=rotation.roll,
                            pitch=rotation.pitch,
                            yaw=rotation.yaw))
                    for pt in predict_array
                ]

            obstacle_predictions_list.append(
                ObstaclePrediction(obstacle_trajectory,
                                   latest_transform,
                                   probability=1.0,
                                   predicted_trajectory=predictions)
            )

        return obstacle_predictions_list
