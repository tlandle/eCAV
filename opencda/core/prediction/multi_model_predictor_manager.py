###############################################################################
# MultiPathPPPredictorManager3D  –  works on ObstacleTrajectory objects
###############################################################################

import math, numpy as np, carla
from pathlib import Path
from typing import Dict, List

from opencda.core.prediction.obstacle_prediction import ObstaclePrediction
from multipathpp.tf2_inference import MPPPEngine     # thin wrapper

class MultiPathPPPredictorManager3D:
    """
    Multi-modal predictor using the official Waymo MultiPath++ (no-map)
    checkpoint.  Input = Dict[int, ObstacleTrajectory] (same as LinearPredictor).
    Output = List[ObstaclePrediction].
    """

    def __init__(self,
                 checkpoint_dir: str | Path,
                 num_future_steps: int = 30,
                 device: str = 'cuda'):
        self.num_predicted_steps = num_future_steps
        self.engine = MPPPEngine(Path(checkpoint_dir), device=device)

    # ───────────── helper: build agent-centric features ──────────────
    @staticmethod
    def _traj_to_features(traj: 'ObstacleTrajectory') -> dict:
        """
        Convert list[Transform] (oldest→latest, len ≤10) into the tensor fields
        expected by MultiPath++ (no-map).  Outputs np.arrays.
        """
        history = traj.trajectory[-10:]                        # keep ≤10
        xy_world = np.array([[tf.location.x, tf.location.y] for tf in history],
                            dtype=np.float32)
        yaw_world = np.radians([tf.rotation.yaw for tf in history])

        # agent frame: translate & rotate s.t. last pose = (0,0,0)
        x0, y0   = xy_world[-1]
        yaw0     = yaw_world[-1]
        cos0, sin0 = math.cos(-yaw0), math.sin(-yaw0)

        delta = xy_world - np.array([x0, y0], dtype=np.float32)
        xy_agent = np.empty_like(delta)
        xy_agent[:, 0] = delta[:,0]*cos0 - delta[:,1]*sin0
        xy_agent[:, 1] = delta[:,0]*sin0 + delta[:,1]*cos0

        vel_agent = np.diff(xy_agent, axis=0, prepend=xy_agent[0:1]) / 0.1
        yaw_agent = np.unwrap(yaw_world - yaw0)

        return {
            'state/past/x'       : xy_agent[:,0],
            'state/past/y'       : xy_agent[:,1],
            'state/past/vel_x'   : vel_agent[:,0],
            'state/past/vel_y'   : vel_agent[:,1],
            'state/past/heading' : yaw_agent,
            'state/valid'        : np.ones((len(history),), np.int64),
            'state/type'         : np.array([1], np.int64)      # 1 = vehicle
        }

    # ───────────── public API – identical to LinearPredictor ──────────
    def generate_predicted_trajectories(
        self,
        tracked_obstacles_trajectories: Dict[int, 'ObstacleTrajectory']
    ) -> List[ObstaclePrediction]:

        predictions: List[ObstaclePrediction] = []

        for traj in tracked_obstacles_trajectories.values():
            if len(traj.trajectory) < 2:
                continue

            # 1) build features & run network
            feats = self._traj_to_features(traj)
            modes_xy, probs = self.engine.predict_one_features(feats)
            # modes_xy shape: [K, T_f, 2]

            last_tf = traj.trajectory[-1]
            base_x, base_y = last_tf.location.x, last_tf.location.y
            z_lvl  = last_tf.location.z
            yaw_tf = last_tf.rotation

            # 2) back-project modes to world 3-D
            cos0, sin0 = math.cos(yaw_tf.yaw * math.pi/180), math.sin(yaw_tf.yaw * math.pi/180)
            world_modes: List[List[carla.Transform]] = []

            for mode in modes_xy:             # loop over K modes
                path = []
                for p in mode:
                    # rotate back
                    xw = p[0]*cos0 - p[1]*sin0 + base_x
                    yw = p[0]*sin0 + p[1]*cos0 + base_y
                    path.append(carla.Transform(
                        location=carla.Location(x=float(xw), y=float(yw), z=z_lvl),
                        rotation=yaw_tf))
                world_modes.append(path)

            predictions.append(ObstaclePrediction(
                obstacle_trajectory=traj,
                transform=last_tf,
                probability=probs,              # np.ndarray[K]
                predicted_trajectory=world_modes))
        return predictions

