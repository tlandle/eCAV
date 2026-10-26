# -*- coding: utf-8 -*-
"""Re-index a consumed forecast by its age at use.

The edge publishes a constant-velocity forecast whose first point refers to
the tick after the forecast's origin: the observation tick for object
sharing and the exact-state reference, the ingest tick for an arm that
rectifies its detections before association (VIPS). The planner consumes a
forecast some ticks later. Without re-indexing, the planner reads point 0 as
the obstacle's present position, which lies age/dt ticks in the past, and
its hazard check compares the obstacle's past against the ego's future: a
crossing vehicle that has already cleared the conflict point reads as still
approaching it (false brake at 1.5 s and above on the junction geometries),
and one that has moved on reads as further away than it is (collision at
the 2.0 m proximity check at 500 ms, SenSys j3 campaign, 2026-10-24/26).

reindex_forecast_by_age() drops the points that lie in the past and extends
the end by the last segment's velocity, so the consumed forecast starts at
the present and keeps its length. Age at use is unchanged: it is still
measured from the observation, and the metrics read source_tick.
"""
import copy
import logging

logger = logging.getLogger(__name__)


def shift_trajectory(traj, k):
    """Return traj shifted by k points: traj[k:] plus a constant-velocity
    extension of the last segment so the length is preserved. k <= 0 returns
    traj unchanged. Needs at least two points to extend."""
    n = len(traj)
    if k <= 0 or n < 2:
        return traj
    import carla
    last, prev = traj[-1], traj[-2]
    dx = last.location.x - prev.location.x
    dy = last.location.y - prev.location.y
    dz = last.location.z - prev.location.z
    rot = last.rotation
    head = list(traj[k:]) if k < n else []
    # The extension continues from the last point: point n-1+i, i >= 1
    first_missing = max(k, n)          # index of the first point to synthesise
    need = n - len(head)
    out = head
    for i in range(need):
        j = first_missing + i          # absolute index of the synthesised point
        step = j - (n - 1)             # steps beyond the last published point
        out.append(carla.Transform(
            carla.Location(x=last.location.x + dx * step,
                           y=last.location.y + dy * step,
                           z=last.location.z + dz * step),
            carla.Rotation(pitch=rot.pitch, yaw=rot.yaw, roll=rot.roll)))
    return out


def reindex_forecast_by_age(pred, now_tick):
    """Return a shallow copy of pred whose predicted_trajectory (and all
    modes, if present) start at now_tick. pred itself is not modified, so a
    prediction held across ticks is re-indexed from its original each tick.
    Returns pred unchanged when the age is unknown or not positive."""
    if now_tick is None:
        return pred
    origin = getattr(pred, 'origin_tick', None)
    if origin is None:
        origin = getattr(pred, 'source_tick', None)
    if origin is None:
        return pred
    traj = getattr(pred, '_orig_predicted_trajectory', None) or pred.predicted_trajectory
    if not traj or len(traj) < 2:
        return pred
    # point j of the forecast refers to tick origin + j + 1
    k = int(now_tick) - int(origin) - 1
    if k <= 0:
        return pred
    p = copy.copy(pred)
    p._orig_predicted_trajectory = traj
    p.predicted_trajectory = shift_trajectory(traj, k)
    p.age_reindex_steps = k
    modes = getattr(pred, 'predicted_trajectories_all', None)
    if modes:
        p.predicted_trajectories_all = [
            shift_trajectory(m, k) if (m and len(m) >= 2) else m for m in modes]
    return p
