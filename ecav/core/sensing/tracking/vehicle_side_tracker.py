# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Vehicle-side 3D tracker + linear predictor.

The on-vehicle equivalent of the edge's ``track + predict`` stages,
reusing the same AB3DMOT tracker and LinearPredictorManager so the
edge-vs-vehicle comparison isolates placement, not algorithm.

Used by the standard cooperative-perception baseline: edge ships raw
multi-source detections, vehicle tracks them locally, vehicle's planner
consumes the same prediction format it already expects from the edge.

Row-to-ObstacleTrajectory conversion mirrors the edge's
``_ab3d_history_to_trajs`` exactly: trajectory is a deque with newest at
index 0, ``obstacle`` is a real ``ObstacleVehicle`` carrying ``carla_id``,
``track_id``, ``kf_vx``, ``kf_vy``, and ``kf_speed_mps`` so the linear
predictor's track-maturity and stationarity gates evaluate identically.
"""
from __future__ import annotations

import logging
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper

logger = logging.getLogger(__name__)


def _box_to_transform(box):
    """Convert AB3DMOT box format [h, w, l, x, y, z, yaw] to picklable Transform.

    Matches the edge's coordinate handling: KITTI camera convention
    inside the tracker (KITTI x = CARLA x, KITTI y = CARLA z, KITTI z =
    CARLA y), swap y<->z back to CARLA world coordinates on output.
    """
    from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf

    h, w, l, x, ky, kz, yaw = box  # ky=CARLA_z, kz=CARLA_y
    loc = _Loc(x=float(x), y=float(kz), z=float(ky))
    rot = _Rot(yaw=np.degrees(float(yaw)))
    return _Tf(location=loc, rotation=rot)


class VehicleSideTracker:
    """Per-vehicle AB3DMOT + linear predictor for the standard
    cooperative-perception baseline.

    Construction is config-driven so a single YAML knob can toggle the
    baseline on without changing planner code paths.

    Parameters
    ----------
    cfg : dict, optional
        ``tracker`` -- passed straight through to AB3DMOTWrapper.
        ``predictor`` -- LinearPredictorManager config.
        ``history_len`` -- per-track history depth retained for the
        predictor (default 10 frames, matching the edge).
        ``dt`` -- simulator tick in seconds (default 0.05), used to
        convert KF m/tick velocities to m/s.
    """

    def __init__(self, cfg: Optional[dict] = None) -> None:
        cfg = cfg or {}
        tracker_cfg = cfg.get("tracker", {})
        predictor_cfg = cfg.get("predictor", {})
        self._history_len = int(cfg.get("history_len", 10))
        self._dt = float(cfg.get("dt", 0.05))

        self._tracker = AB3DMOTWrapper(tracker_cfg)
        self._predictor = LinearPredictorManager(
            cfg_or_steps=predictor_cfg.get("num_future_steps", 10),
        )
        self._tracked: Dict[int, ObstacleTrajectory] = {}
        self._frame: int = 0

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def process_detections(
        self,
        dets_all: Dict[str, np.ndarray],
        tick: int,
    ) -> List[Any]:
        """Run one tick of the local tracker on the edge-published
        detection list and return short-horizon predictions in the same
        format the planner already consumes from the edge.

        Parameters
        ----------
        dets_all : dict
            AB3DMOT-format detection bundle: ``'dets'`` shape ``(N, 8)``
            ``[h, w, l, x, y, z, yaw, score]`` and ``'info'`` shape
            ``(N, 3)`` ``[frame, det_idx, source_id]``. Empty bundle is
            permitted.
        tick : int
            Simulator tick of the detection batch. Used as the
            ``source_tick`` for the predictor's AoI metadata.
        """
        self._frame = int(tick)
        results, _ = self._tracker.track(dets_all, self._frame)
        self._update_trajectory_cache(results)
        return self._predictor.generate_predicted_trajectories(
            self._tracked,
            source_tick=self._frame,
            publish_tick=self._frame,
        )

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget all tracker state."""
        self._tracker.reset()
        self._tracked.clear()
        self._frame = 0

    # ------------------------------------------------------------------
    # Internals -- mirror edge _ab3d_history_to_trajs exactly
    # ------------------------------------------------------------------
    def _update_trajectory_cache(self, tracker_rows: List[np.ndarray]) -> None:
        updated: set[int] = set()
        for frame in tracker_rows:
            if frame is None or len(frame) == 0:
                continue
            for trk in np.atleast_2d(frame):
                if len(trk) < 9:
                    continue
                tid = int(trk[7])
                cid = int(trk[8])
                tf = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self._tracked:
                    dummy = ObstacleVehicle(
                        corners=np.zeros((8, 3)),
                        o3d_bbx=None,
                        track_id=tid,
                        tick_id=0,
                    )
                    self._tracked[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=self._history_len),
                    )

                traj = self._tracked[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid

                # KF velocity (m/tick) -> m/s for downstream prediction gating.
                # KITTI dx(idx 10) = CARLA vx, KITTI dz(idx 12) = CARLA vy.
                if len(trk) > 12:
                    kf_vx = float(trk[10])
                    kf_vy = float(trk[12])
                    traj.obstacle.kf_vx = kf_vx
                    traj.obstacle.kf_vy = kf_vy
                    traj.obstacle.kf_speed_mps = (
                        (kf_vx ** 2 + kf_vy ** 2) ** 0.5
                    ) / self._dt

        # Drop tracks the tracker no longer reports this tick.
        for tid in list(self._tracked):
            if tid not in updated:
                del self._tracked[tid]
