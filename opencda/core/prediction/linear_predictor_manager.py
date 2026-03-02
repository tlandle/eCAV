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

# Displacement consistency check parameters.
# P[7:]*=10000 in the KF causes aggressive velocity convergence from the
# first position delta, and R*=10 + Q*=0.01 prevent correction.  Even a
# 0.1m LiDAR noise blip at birth gets locked in as ~1.7 m/s phantom
# velocity.  We verify the KF velocity against actual observed displacement:
# if the object hasn't actually moved at least this fraction of what the KF
# predicts, the velocity estimate is an initialization artifact.
_DISPLACEMENT_CHECK_MIN_FRAMES = 5   # need enough history for reliable measurement
_DISPLACEMENT_CONSISTENCY_RATIO = 0.5  # actual / expected displacement
_DT = 0.05  # simulation timestep (seconds)


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

    def generate_predicted_trajectories(self, tracked_obstacles_trajectories,
                                        source_tick=None, publish_tick=None):
        """
        Generate predicted trajectories for tracked obstacles in world coordinates.

        Args:
            tracked_obstacles_trajectories (Dict[int, ObstacleTrajectory]):
                Dictionary mapping track_id to ObstacleTrajectory.
            source_tick (int, optional): Simulation tick of the latest detection
                frame used by the tracker. Attached to each ObstaclePrediction
                for AoI (Age of Information) tracking.

        Returns:
            List[ObstaclePrediction]: Future trajectory predictions per obstacle.
        """
        obstacle_predictions_list = []

        for obstacle_trajectory in tracked_obstacles_trajectories.values():
            kf_speed = getattr(obstacle_trajectory.obstacle, 'kf_speed_mps', None)

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

            # Stationary gate: objects below _MIN_KF_SPEED_MPS get a
            # stationary prediction (all points at current location) so the
            # collision check can still detect stopped vehicles in the path.
            if kf_speed is not None and kf_speed < _MIN_KF_SPEED_MPS:
                logger.debug("[PRED STATIONARY] track_id=%s kf_speed=%.2f m/s",
                             obstacle_trajectory.obstacle.track_id, kf_speed)
                static_tf = Transform(
                    location=Location(x=cur_x, y=cur_y, z=cur_z),
                    rotation=Rotation(
                        roll=rotation.roll,
                        pitch=rotation.pitch,
                        yaw=rotation.yaw))
                predictions = [static_tf] * self.num_predicted_steps
                obstacle_predictions_list.append(
                    ObstaclePrediction(obstacle_trajectory,
                                       latest_transform,
                                       probability=1.0,
                                       predicted_trajectory=predictions,
                                       source_tick=source_tick,
                                       publish_tick=publish_tick)
                )
                continue

            # Prefer KF velocity extrapolation over regression
            kf_vx = getattr(obstacle_trajectory.obstacle, 'kf_vx', None)
            kf_vy = getattr(obstacle_trajectory.obstacle, 'kf_vy', None)

            obs = obstacle_trajectory.obstacle
            logger.debug("[PRED PASS] track_id=%s carla_id=%s kf_speed=%s kf_vx=%s kf_vy=%s "
                          "pos=(%.1f,%.1f) hist=%d method=%s",
                          obs.track_id, obs.carla_id,
                          f"{kf_speed:.2f}" if kf_speed is not None else "None",
                          f"{kf_vx:.4f}" if kf_vx is not None else "None",
                          f"{kf_vy:.4f}" if kf_vy is not None else "None",
                          cur_x, cur_y, num_steps,
                          "KF" if (kf_vx is not None and kf_vy is not None) else "REGRESSION")

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
                                   predicted_trajectory=predictions,
                                   source_tick=source_tick,
                                   publish_tick=publish_tick)
            )

        return obstacle_predictions_list
