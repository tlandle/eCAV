"""Implements an operator that predicts trajectories using KF velocity."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
import numpy as np

from ecav.core.prediction.obstacle_prediction import ObstaclePrediction
from ecav.ecav_carla import Location, Rotation, Transform

logger = logging.getLogger(__name__)

# ── Track maturity gates ──
# Anonymous tracks (carla_id == -1) need more frames for KF velocity to
# converge past initialization artifacts.  At dt=50ms, ±1-3m position
# jitter in the first 3 frames maps to 20-60 m/s phantom velocity.
# 10 frames (500ms) is enough for the KF to converge to <1 m/s on a
# truly stationary object.
_MIN_HISTORY_ANONYMOUS = 10
# Identity-bound tracks (carla_id != -1) have stable association and are
# not fighting duplicate births, so a shorter maturity window is safe.
_MIN_HISTORY_IDENTIFIED = 3

# Minimum KF-estimated speed (m/s) to generate a moving prediction.
# Objects below this get a stationary prediction (all points at current
# location) so the collision check can still detect stopped vehicles in
# the path, but TTC=1000 from spatial-overlap means no braking trigger.
_MIN_KF_SPEED_MPS = 1.0

# ── Displacement consistency check ──
# Distinguishes real motion from pose jitter.  For true motion
# (consistency ≈ 1) net displacement tracks path length.  For jitter
# (consistency ≈ 0) the object oscillates in place.
_CONSISTENCY_MIN_FRAMES = 5
_CONSISTENCY_RATIO = 0.6     # net_disp / path_disp threshold
_MIN_NET_DISPLACEMENT = 2.0  # metres — large actual movement always passes


class LinearPredictorManager():
    """Predicts future trajectories using Kalman filter velocity.

    Uses the tracker's KF velocity estimate (kf_vx, kf_vy) for constant-
    velocity extrapolation from the current position.  Falls back to
    least-squares regression on historical positions when KF velocity
    components are unavailable.

    KF velocity is preferred over regression because regression fits to
    ALL historical positions equally — including early frames with large
    position errors.  As estimates improve over time, the regression
    interprets convergence as real motion, producing diagonal predicted
    trajectories.  The KF velocity reflects the filter's current best
    estimate and is not biased by old positions.
    """
    def __init__(self, cfg_or_steps=60):
        # Accept either a config dict (from registry) or raw int (legacy)
        if isinstance(cfg_or_steps, dict):
            self.num_predicted_steps = cfg_or_steps.get('num_future_steps', 25)
        else:
            self.num_predicted_steps = cfg_or_steps

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

            # ── Track maturity gate ──
            # Anonymous tracks need longer history for KF velocity to
            # converge past initialization artifacts from position jitter.
            cid = getattr(obstacle_trajectory.obstacle, 'carla_id', -1)
            min_hist = (_MIN_HISTORY_IDENTIFIED if cid != -1
                        else _MIN_HISTORY_ANONYMOUS)
            if num_steps < min_hist:
                continue

            # Use latest position and orientation (trajectory[0] is newest)
            latest_transform = trajectory[0]
            rotation = latest_transform.rotation
            cur_x = latest_transform.location.x
            cur_y = latest_transform.location.y
            cur_z = latest_transform.location.z

            # ── Displacement consistency check ──
            # Catch tracks where pose jitter masquerades as motion.
            # net_disp / path_disp ≈ 1 for real motion, ≈ 0 for jitter.
            force_stationary = False
            if (num_steps >= _CONSISTENCY_MIN_FRAMES
                    and kf_speed is not None
                    and kf_speed >= _MIN_KF_SPEED_MPS):
                first = trajectory[-1].location   # oldest
                last  = trajectory[0].location     # newest
                net_disp = ((last.x - first.x)**2 +
                            (last.y - first.y)**2)**0.5
                path_disp = sum(
                    ((trajectory[j].location.x - trajectory[j+1].location.x)**2 +
                     (trajectory[j].location.y - trajectory[j+1].location.y)**2)**0.5
                    for j in range(num_steps - 1))
                consistency = net_disp / max(path_disp, 1e-6)

                if (consistency < _CONSISTENCY_RATIO
                        and net_disp < _MIN_NET_DISPLACEMENT):
                    logger.debug(
                        "[PRED JITTER] track_id=%s cid=%s kf_speed=%.1f m/s "
                        "consistency=%.2f net_disp=%.2fm — forcing stationary",
                        obstacle_trajectory.obstacle.track_id, cid,
                        kf_speed, consistency, net_disp)
                    force_stationary = True

            # Stationary gate: objects below _MIN_KF_SPEED_MPS or flagged
            # as jitter get a stationary prediction (all points at current
            # location).  TTC=1000 from spatial-overlap means no braking.
            if force_stationary or (kf_speed is not None and kf_speed < _MIN_KF_SPEED_MPS):
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
