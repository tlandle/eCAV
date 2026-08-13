"""Shared utilities for converting AB3DMOT tracker output to ObstacleTrajectory.

Used by both the monolithic edge managers and the composable edge manager.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from collections import deque
from typing import Dict, Optional, Set

import numpy as np

from ecav.ecav_carla import Location as _Loc, Rotation as _Rot, Transform as _Tf
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory


def box_to_transform_late_fusion(box: np.ndarray) -> _Tf:
    """Convert AB3DMOT track output [h,w,l,x,ky,kz,yaw] to world Transform.

    KITTI camera convention inside tracker:
        KITTI x = CARLA x, KITTI y = CARLA z (height), KITTI z = CARLA y
    Swaps y/z back to CARLA world coordinates.
    """
    h, w, l, x, ky, kz, yaw = box
    return _Tf(
        location=_Loc(x=float(x), y=float(kz), z=float(ky)),
        rotation=_Rot(yaw=np.degrees(float(yaw))))


def box_to_transform_worldfusion(trk: np.ndarray) -> _Tf:
    """Convert AB3DMOT track output to world Transform (WorldFusion order).

    WorldFusion tracker output: [h,w,l,x,y,z,yaw,...].
    Reorder to [x,y,z,h,w,l,yaw] then extract.
    """
    return _Tf(
        location=_Loc(x=float(trk[3]), y=float(trk[5]), z=float(trk[4])),
        rotation=_Rot(yaw=np.degrees(float(trk[6]))))


def ab3d_tracks_to_trajectories(
        hist,
        tracked_trajectories: Dict[int, ObstacleTrajectory],
        track_to_carla: Dict[int, int],
        horizon: int = 30,
        dt: float = 0.05,
        box_to_transform=box_to_transform_late_fusion,
        beacon_id_mgr=None,
        anchoring: bool = True,
) -> Set[int]:
    """Rebuild ObstacleTrajectory dict from AB3DMOT track history.

    Clears existing trajectory deques and replays from the history buffer.
    Extracts KF velocity from tracker state indices 10 (vx) and 12 (vy).

    Args:
        hist: Iterable of np.ndarray tracker output frames.
        tracked_trajectories: Dict to update in-place (track_id -> ObstacleTrajectory).
        track_to_carla: Dict to update in-place (track_id -> carla_id).
        horizon: Maximum trajectory deque length.
        dt: Simulation tick interval (seconds) for speed conversion.
        box_to_transform: Function to convert track row to Transform.
        beacon_id_mgr: Optional BeaconIdManager for temp_id reverse lookup.
        anchoring: Whether beacon anchoring is active.

    Returns:
        Set of track IDs that were updated this call.
    """
    updated: Set[int] = set()

    # Clear trajectory deques
    for traj in tracked_trajectories.values():
        traj.trajectory.clear()

    for frame in hist:
        if frame is None or len(frame) == 0:
            continue
        for trk in frame:
            tid = int(trk[7])
            cid_raw = int(trk[8]) if len(trk) > 8 else -1

            if anchoring and beacon_id_mgr is not None:
                real_cid = beacon_id_mgr.get_carla_id_for_temp(cid_raw)
                cid = real_cid if real_cid is not None else cid_raw
            else:
                cid = cid_raw

            tf = box_to_transform(trk[:7])
            updated.add(tid)

            if tid not in tracked_trajectories:
                dummy = ObstacleVehicle(
                    corners=np.zeros((8, 3)),
                    o3d_bbx=None,
                    track_id=tid,
                    tick_id=0)
                tracked_trajectories[tid] = ObstacleTrajectory(
                    dummy, deque(maxlen=horizon))

            traj = tracked_trajectories[tid]
            traj.trajectory.appendleft(tf)
            traj.obstacle.transform = tf
            traj.obstacle.location = tf.location
            traj.obstacle.carla_id = cid
            track_to_carla[tid] = cid

            # Velocity from tracker state: dx=index 10 (CARLA vx), dz=index 12
            # (CARLA vy). Col 13 flags the unit: Mamba rows carry m/s directly
            # (cadence-independent, measured from source-tick stride); AB3DMOT
            # KF rows carry m per dt-frame and keep the historical divide.
            if len(trk) > 12:
                kf_vx = float(trk[10])
                kf_vy = float(trk[12])
                vel_is_mps = len(trk) > 13 and float(trk[13]) == 1.0
                speed_mps = ((kf_vx**2 + kf_vy**2)**0.5
                             if vel_is_mps else
                             ((kf_vx**2 + kf_vy**2)**0.5) / dt)
                traj.obstacle.kf_speed_mps = speed_mps
                # kf_vx/kf_vy keep the per-dt-frame convention for their
                # consumers regardless of source row unit.
                traj.obstacle.kf_vx = kf_vx * dt if vel_is_mps else kf_vx
                traj.obstacle.kf_vy = kf_vy * dt if vel_is_mps else kf_vy

    # Prune stale tracks
    for tid in list(tracked_trajectories):
        if tid not in updated:
            del tracked_trajectories[tid]
            track_to_carla.pop(tid, None)

    return updated
