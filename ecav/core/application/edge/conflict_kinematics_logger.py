# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Per-tick conflict-kinematics logger for the LTAP safety-envelope fixture.

Records the closing geometry between the managed ego and the cross-traffic
actor at every edge tick, plus a conservative braking-margin residual. The
goal is a pinned kinematic reference: at zero edge delay a correct obstacle
track must leave positive braking margin, and increasing AoI must monotonically
consume that margin until the physics-limited boundary. This separates timing
artifacts from logic-level self-ghosting.

Conflict point is the intersection of the ego's turning path with the
cross-traffic path. Distance-to-conflict for the ego is measured along its
planned path (arc length), not Euclidean, because the ego is turning.

One CSV row per tick. Self-contained: the caller passes the ego VehicleManager,
the per-tick GT snapshot the edge already builds, and the tick index.

Braking-margin residual (conservative, RSS-style longitudinal stopping):
    d_stop(v) = v * rho + v^2 / (2 * a_brake)
    M(t)      = d_conflict_ego(t) - d_stop(v_ego(t))
A collision while M > 0 is suspicious (timing artifact or controller bug).
A collision after M < 0 is a genuine physics-limited failure.
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, Optional

import numpy as np


# Conservative defaults; override via edge cfg. rho = reaction/actuation delay,
# a_brake = guaranteed deceleration. IEEE 2846 longitudinal defaults are in this
# range; values are reported in the paper, not hidden.
_DEFAULT_RHO_S = 0.1
_DEFAULT_A_BRAKE = 6.0  # m/s^2


def _stop_distance(v_mps: float, response_s: float, a_brake: float,
                   d_buf: float = 0.0) -> float:
    """Conservative stopping distance.

    d_stop = v * response + v^2 / (2 a_brake) + d_buf
    where response = rho + Delta_use (actuation delay plus edge-state age).
    """
    if v_mps <= 0.0:
        return d_buf
    return v_mps * response_s + (v_mps * v_mps) / (2.0 * a_brake) + d_buf


def tau_max(d_e: float, v_mps: float, a_brake: float, rho_s: float,
            d_buf: float) -> float:
    """Maximum admissible edge-state age at this decision point.

    tau_max = (d_e - d_buf - v^2/(2 a_brake)) / v - rho
    A correct edge track is admissible iff Delta_use <= tau_max. Decreases with
    speed and weaker braking; the physics boundary is where tau_max -> 0.
    """
    if v_mps <= 0.1:
        return float("inf")
    return (d_e - d_buf - (v_mps * v_mps) / (2.0 * a_brake)) / v_mps - rho_s


class ConflictKinematicsLogger:
    """Writes one CSV row per tick describing the ego/cross-traffic conflict.

    Parameters
    ----------
    out_path : str
        CSV destination. Parent dir is created if missing.
    conflict_xy : tuple(float, float)
        World (x, y) of the LTAP conflict point (path intersection).
    cross_traffic_type : str
        Substring matched against actor type_id to pick the cross-traffic
        actor from the GT snapshot (e.g. 'tesla'). The nearest matching actor
        to the conflict point is used if several match.
    rho_s, a_brake : float
        Braking-margin model parameters.
    """

    _COLUMNS = [
        "tick",
        "ego_x", "ego_y", "ego_yaw_deg", "ego_speed_mps", "ego_throttle", "ego_brake",
        "cross_x", "cross_y", "cross_yaw_deg", "cross_speed_mps",
        "ego_dist_conflict_arc_m", "ego_dist_conflict_euclid_m",
        "cross_dist_conflict_m",
        "ego_ttc_conflict_s", "cross_ttc_conflict_s", "delta_ttc_s",
        "delta_use_s",            # edge-state age at planner use (AoI-at-use)
        "ego_stop_dist_m",        # v*(rho+dUse) + v^2/2a + d_buf
        "brake_margin_m",         # d_e - ego_stop_dist  (AoI-aware residual M)
        "brake_margin_no_aoi_m",  # d_e - d_stop with dUse=0 (zero-latency floor)
        "tau_max_s",              # max admissible edge age at this point
        "collision_flag",
    ]

    def __init__(
        self,
        out_path: str,
        conflict_xy=(-84.8, 127.7),
        cross_traffic_type: str = "tesla",
        rho_s: float = _DEFAULT_RHO_S,
        a_brake: float = _DEFAULT_A_BRAKE,
        d_buf: float = 1.0,
    ) -> None:
        self._path = out_path
        self._cx, self._cy = float(conflict_xy[0]), float(conflict_xy[1])
        self._cross_type = cross_traffic_type.lower()
        self._rho = float(rho_s)
        self._a_brake = float(a_brake)
        self._d_buf = float(d_buf)
        self._fh = None
        self._collision_seen = False

    # ------------------------------------------------------------------
    def _ensure_open(self) -> None:
        if self._fh is not None:
            return
        d = os.path.dirname(self._path)
        if d:
            os.makedirs(d, exist_ok=True)
        # Auto-uniquify so sequential/sweep runs never clobber each other's
        # trace. base.csv -> base.csv, base_1.csv, base_2.csv, ...
        path = self._path
        if os.path.exists(path):
            stem, ext = os.path.splitext(self._path)
            n = 1
            while os.path.exists(f"{stem}_{n}{ext}"):
                n += 1
            path = f"{stem}_{n}{ext}"
        self._path = path
        self._fh = open(path, "w")
        self._fh.write(",".join(self._COLUMNS) + "\n")
        self._fh.flush()
        print(f"[CONFLICT_KIN] writing trace to {path}", flush=True)

    # ------------------------------------------------------------------
    def _ego_arc_distance_to_conflict(self, ego_vm) -> float:
        """Arc length along the ego's planned path to the conflict point.

        Walks the local planner's waypoint buffer, summing segment lengths
        until the waypoint closest to the conflict point, so a turning ego is
        measured along its route rather than as-the-crow-flies. Falls back to
        Euclidean if the buffer is unavailable.
        """
        try:
            lp = ego_vm.agent.get_local_planner()
            buf = lp.get_waypoint_buffer()
        except Exception:
            buf = None

        ego_loc = ego_vm.vehicle.get_location()
        if not buf:
            return math.hypot(ego_loc.x - self._cx, ego_loc.y - self._cy)

        # Build the polyline ego -> buffered waypoints, find the vertex nearest
        # the conflict point, return cumulative length up to it.
        pts = [(ego_loc.x, ego_loc.y)]
        for entry in buf:
            wp = entry[0] if isinstance(entry, (tuple, list)) else entry
            try:
                t = wp.transform.location
                pts.append((t.x, t.y))
            except Exception:
                continue

        best_i, best_d = 0, float("inf")
        for i, (px, py) in enumerate(pts):
            d = math.hypot(px - self._cx, py - self._cy)
            if d < best_d:
                best_d, best_i = d, i

        arc = 0.0
        for i in range(1, best_i + 1):
            arc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        # add the residual gap from the nearest vertex to the conflict point
        arc += best_d
        return arc

    # ------------------------------------------------------------------
    def _pick_cross_traffic(self, gt_snapshot: Dict[int, dict]) -> Optional[dict]:
        """Pick the cross-traffic violator.

        The LTAP scene has near-stationary occluders (TM global_speed_perc=-95)
        and one moving violator. Prefer a MOVING vehicle of the configured type
        near the conflict point; occluders are stationary so a speed gate
        separates the violator even when several share the type. Fall back to
        nearest-of-type if none are moving (e.g. before the violator triggers).
        """
        moving, any_type = [], []
        for aid, vd in (gt_snapshot or {}).items():
            if self._cross_type not in str(vd.get("type", "")).lower():
                continue
            d = math.hypot(vd["x"] - self._cx, vd["y"] - self._cy)
            spd = float(vd.get("speed", 0.0))
            any_type.append((d, vd))
            if spd > 2.0:
                moving.append((d, vd))
        pool = moving or any_type
        if not pool:
            return None
        pool.sort(key=lambda t: t[0])
        return pool[0][1]

    # ------------------------------------------------------------------
    def log_tick(
        self,
        tick: int,
        ego_vm,
        gt_snapshot: Dict[int, dict],
        collision_flag: bool = False,
        delta_use_s: float = 0.0,
    ) -> None:
        """Append one row for this tick. Safe to call every edge step.

        delta_use_s is the age of the edge state the planner is acting on
        (Delta_use). It enters the stopping distance as part of the response
        time (rho + Delta_use), so the margin shrinks as edge state ages.
        """
        try:
            self._ensure_open()
            ego_tf = ego_vm.vehicle.get_transform()
            ego_loc = ego_tf.location
            ego_vel = ego_vm.vehicle.get_velocity()
            ego_speed = math.hypot(ego_vel.x, ego_vel.y)
            try:
                ctrl = ego_vm.vehicle.get_control()
                throttle, brake = float(ctrl.throttle), float(ctrl.brake)
            except Exception:
                throttle, brake = float("nan"), float("nan")

            ego_arc = self._ego_arc_distance_to_conflict(ego_vm)
            ego_euclid = math.hypot(ego_loc.x - self._cx, ego_loc.y - self._cy)
            ego_ttc = ego_arc / ego_speed if ego_speed > 0.1 else float("inf")

            cross = self._pick_cross_traffic(gt_snapshot)
            if cross is not None:
                cross_speed = float(cross.get("speed",
                    math.hypot(cross.get("vx", 0.0), cross.get("vy", 0.0))))
                cross_dist = math.hypot(cross["x"] - self._cx, cross["y"] - self._cy)
                cross_ttc = cross_dist / cross_speed if cross_speed > 0.1 else float("inf")
                cross_x, cross_y = cross["x"], cross["y"]
                cross_yaw = cross.get("yaw", float("nan"))
            else:
                cross_speed = cross_dist = cross_ttc = float("nan")
                cross_x = cross_y = cross_yaw = float("nan")

            delta_ttc = (ego_ttc - cross_ttc
                         if math.isfinite(ego_ttc) and math.isfinite(cross_ttc)
                         else float("nan"))
            # AoI-aware stopping distance: response = rho + Delta_use.
            stop_d = _stop_distance(ego_speed, self._rho + delta_use_s,
                                    self._a_brake, self._d_buf)
            margin = ego_arc - stop_d
            # Zero-latency floor: same formula with Delta_use = 0.
            stop_d0 = _stop_distance(ego_speed, self._rho,
                                     self._a_brake, self._d_buf)
            margin_no_aoi = ego_arc - stop_d0
            tmax = tau_max(ego_arc, ego_speed, self._a_brake,
                           self._rho, self._d_buf)

            if collision_flag:
                self._collision_seen = True

            row = [
                tick,
                f"{ego_loc.x:.3f}", f"{ego_loc.y:.3f}",
                f"{ego_tf.rotation.yaw:.2f}", f"{ego_speed:.3f}",
                f"{throttle:.3f}", f"{brake:.3f}",
                f"{cross_x:.3f}", f"{cross_y:.3f}", f"{cross_yaw:.2f}",
                f"{cross_speed:.3f}",
                f"{ego_arc:.3f}", f"{ego_euclid:.3f}", f"{cross_dist:.3f}",
                f"{ego_ttc:.3f}", f"{cross_ttc:.3f}", f"{delta_ttc:.3f}",
                f"{delta_use_s:.3f}",
                f"{stop_d:.3f}", f"{margin:.3f}", f"{margin_no_aoi:.3f}",
                f"{tmax:.3f}",
                int(bool(self._collision_seen)),
            ]
            self._fh.write(",".join(str(c) for c in row) + "\n")
            self._fh.flush()
        except Exception:
            # Logging must never break the sim.
            pass

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None
