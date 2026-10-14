# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Per-tick, per-object age-at-use logger for the safety-envelope runs.

The SenSys runs recorded one simulation_metrics.json per run with per-vehicle
and per-edge aggregates. That cannot answer three questions the reviewers
asked: the age of the conflicting actor's state at the braking decision
(conflict-time age), the tail age over the conflicting actor alone
(critical-object age), and how many consecutive edge publishes a vehicle
missed (burst loss). This logger writes one CSV row per planner tick per
object the planner consumed, so those reductions become a script over the
run directory instead of a re-run.

Wiring: _BaseEdgeManager constructs the logger when the edge config block has

    per_tick_age_log:
      enabled: true
      match_radius_m: 5.0      # anonymous track counts as the conflict actor
                               # when within this distance of its GT position
      conflict_xy: [...]       # defaults to the conflict_kinematics block
      cross_traffic_type: tesla

and calls log_tick() from _record_time_to_events(), which every SenSys edge
manager invokes once per managed vehicle per tick after the planner step.
The publish sequence comes from note_publish(), which the base manager calls
around the shared linear predictor so every edge publish gets a sequence
number regardless of manager subclass.

Row semantics
-------------
One row per (tick, focal vehicle, consumed prediction). When the planner
consumed nothing that tick, one row with blank object fields is written so
the decision timeline stays complete.

provenance: 'self' when the object's carla_id is the focal vehicle itself
(a self-ghost by identity); 'anchored' when the id is a real actor id and
anchoring is on (beacon-bound); 'identified' when the id is a real actor id
with anchoring off (oracle front-end); 'anonymous' otherwise (-1 or an edge
guid above 10000, the planner's own rule).

age_at_use_ms = (consume_tick - gen_tick) * dt, where gen_tick is the
prediction's source_tick (the detection frame the tracker last ingested) and
consume_tick is the planner tick that acted on it.

The file is small: no boxes, no trajectories, about 25 short fields per row.
"""
from __future__ import annotations

import csv
import logging
import math
import os
import time
from typing import Any, Dict, Optional


COLUMNS = [
    "tick", "sim_time_s", "wall_time_s", "focal_vehicle_id",
    "track_id", "carla_id", "source", "provenance",
    "gen_tick", "publish_tick", "publish_seq", "consume_tick",
    "age_at_use_ms", "publish_age_ms",
    "obs_tick", "frame_age_ms", "lineage_ok",
    "obs_x", "obs_y", "obs_speed_mps",
    "conflict_actor_id", "is_conflict_actor", "conflict_dist_m",
    "ego_speed_mps", "ctrl_throttle", "ctrl_brake", "decision",
    "brake_event", "brake_reason", "gt_brake_class", "ttc_s",
]

_ANON_GUID_FLOOR = 10000   # planner rule: carla_id > 10000 is an edge guid


def _fmt(v: Any) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        if math.isnan(v) or math.isinf(v):
            return ""
        return f"{v:.3f}"
    return str(v)


class PerTickAgeLogger:
    """Writes one CSV row per planner tick per consumed object."""

    def __init__(
        self,
        out_path: str,
        conflict_xy=(-84.8, 127.7),
        cross_traffic_type: str = "tesla",
        match_radius_m: float = 5.0,
    ) -> None:
        self._path = out_path
        self._cx, self._cy = float(conflict_xy[0]), float(conflict_xy[1])
        self._cross_type = str(cross_traffic_type).lower()
        self._match_r = float(match_radius_m)
        self._fh = None
        self._writer = None
        # publish_tick -> sequence number, assigned in publish order
        self._publish_seq: Dict[int, int] = {}
        # rows whose obs <= frame <= publish <= consume ordering failed
        self._lineage_violations = 0
        self._next_seq = 0
        # per-vehicle count of brake attributions seen, to detect a new one
        self._brake_seen: Dict[int, int] = {}

    # ------------------------------------------------------------------
    def note_publish(self, publish_tick: Optional[int]) -> None:
        """Record that the edge published at publish_tick (idempotent)."""
        if publish_tick is None or publish_tick in self._publish_seq:
            return
        self._publish_seq[int(publish_tick)] = self._next_seq
        self._next_seq += 1

    # ------------------------------------------------------------------
    def _ensure_open(self) -> None:
        if self._fh is not None:
            return
        d = os.path.dirname(self._path)
        if d:
            os.makedirs(d, exist_ok=True)
        self._fh = open(self._path, "w", newline="")
        self._writer = csv.writer(self._fh)
        self._writer.writerow(COLUMNS)

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
                self._fh.close()
            except Exception:
                pass
            self._fh = None
            self._writer = None
            if self._lineage_violations:
                logging.getLogger(__name__).warning(
                    "[PTAGE] %d rows violated obs<=frame<=publish<=consume in %s",
                    self._lineage_violations, self._path)

    @property
    def lineage_violations(self) -> int:
        return self._lineage_violations

    # ------------------------------------------------------------------
    def _pick_conflict_actor(self, gt_snapshot: Dict[int, dict]) -> Optional[tuple]:
        """Same rule as ConflictKinematicsLogger: a moving vehicle of the
        configured type nearest the conflict point; nearest-of-type if none
        is moving yet. Returns (actor_id, x, y) or None."""
        moving, any_type = [], []
        for aid, vd in (gt_snapshot or {}).items():
            if self._cross_type not in str(vd.get("type", "")).lower():
                continue
            d = math.hypot(vd["x"] - self._cx, vd["y"] - self._cy)
            any_type.append((d, aid, vd))
            if float(vd.get("speed", 0.0)) > 2.0:
                moving.append((d, aid, vd))
        pool = moving or any_type
        if not pool:
            return None
        pool.sort(key=lambda t: t[0])
        _, aid, vd = pool[0]
        return int(aid), float(vd["x"]), float(vd["y"])

    @staticmethod
    def _provenance(cid: Optional[int], focal_id: int, anchoring: bool) -> str:
        if cid is None or cid < 0 or cid > _ANON_GUID_FLOOR:
            return "anonymous"
        if cid == focal_id:
            return "self"
        return "anchored" if anchoring else "identified"

    @staticmethod
    def _obs_xy_speed(pred) -> tuple:
        obs = pred.obstacle_trajectory.obstacle
        loc = getattr(obs, "location", None)
        if loc is None:
            tf = getattr(pred, "transform", None)
            loc = getattr(tf, "location", None)
        x = float(getattr(loc, "x", float("nan"))) if loc is not None else float("nan")
        y = float(getattr(loc, "y", float("nan"))) if loc is not None else float("nan")
        spd = getattr(obs, "kf_speed_mps", None)
        if spd is None:
            vel = getattr(obs, "velocity", None)
            if vel is not None:
                spd = math.hypot(float(getattr(vel, "x", 0.0)), float(getattr(vel, "y", 0.0)))
        return x, y, (float(spd) if spd is not None else float("nan"))

    # ------------------------------------------------------------------
    def log_tick(self, tick: int, vm, gt_snapshot: Dict[int, dict], dt: float) -> None:
        """Append the rows for one managed vehicle at one tick. Safe to call
        every tick; never raises into the edge loop."""
        try:
            self._ensure_open()
            agent = vm.agent
            focal_id = int(vm.vehicle.id)
            wall = time.time()
            sim_t = tick * dt

            try:
                ctrl = vm.vehicle.get_control()
                throttle, brake = float(ctrl.throttle), float(ctrl.brake)
            except Exception:
                throttle, brake = float("nan"), float("nan")
            try:
                vel = vm.vehicle.get_velocity()
                ego_speed = math.hypot(vel.x, vel.y)
            except Exception:
                ego_speed = float("nan")
            if not math.isnan(brake) and brake >= 0.5:
                decision = "brake"
            elif not math.isnan(brake) and brake > 0.0:
                decision = "yield"
            else:
                decision = "go"

            # Brake event this tick: a new attribution appended by the planner.
            attrs = getattr(getattr(agent, "planning_metrics", None),
                            "brake_attributions", None) or []
            seen = self._brake_seen.get(focal_id, 0)
            brake_event = 1 if len(attrs) > seen else 0
            self._brake_seen[focal_id] = len(attrs)
            brake_reason = attrs[-1].get("ghost_reason") if (brake_event and attrs) else ""
            gt_class = attrs[-1].get("gt_brake_class") if (brake_event and attrs) else ""
            ttc = getattr(agent, "ttc", None)
            ttc_s = float(ttc) if ttc is not None and float(ttc) < 999 else float("nan")

            conflict = self._pick_conflict_actor(gt_snapshot)
            c_id = conflict[0] if conflict else None
            anchoring = bool(getattr(agent, "_anchoring", False))

            common_head = [tick, _fmt(sim_t), _fmt(wall), focal_id]
            common_tail = [
                _fmt(ego_speed), _fmt(throttle), _fmt(brake), decision,
                brake_event, brake_reason or "", gt_class or "", _fmt(ttc_s),
            ]

            preds = list(getattr(agent, "generated_predictions", None) or [])
            if not preds:
                self._writer.writerow(
                    common_head
                    + ["", "", "none", "", "", "", "", tick, "", "", "", "", "", "", "", "",
                       _fmt(c_id), 0, ""]
                    + common_tail)
                self._fh.flush()
                return

            for pred in preds:
                obs = pred.obstacle_trajectory.obstacle
                tid = getattr(obs, "track_id", None)
                cid = getattr(obs, "carla_id", None)
                cid_i = int(cid) if cid is not None else None
                frame_tick = getattr(pred, "source_tick", None)
                obs_tick = getattr(pred, "obs_tick", None)
                # age at use follows the observation (last matched detection of
                # the track); frame_age_ms is the tracker frame clock for reference
                gen_tick = obs_tick if obs_tick is not None else frame_tick
                pub_tick = getattr(pred, "publish_tick", None)
                source = "edge" if pub_tick is not None else "local"
                prov = self._provenance(cid_i, focal_id, anchoring)
                age_ms = ((tick - int(gen_tick)) * dt * 1000.0) if gen_tick is not None else None
                frame_age_ms = ((tick - int(frame_tick)) * dt * 1000.0) if frame_tick is not None else None
                pub_age_ms = ((tick - int(pub_tick)) * dt * 1000.0) if pub_tick is not None else None
                # lineage: obs <= frame <= publish <= consume
                chain = [int(x) for x in (obs_tick, frame_tick, pub_tick) if x is not None] + [int(tick)]
                lineage_ok = 1 if all(a <= b for a, b in zip(chain, chain[1:])) else 0
                if not lineage_ok:
                    self._lineage_violations += 1
                seq = self._publish_seq.get(int(pub_tick)) if pub_tick is not None else None
                ox, oy, ospd = self._obs_xy_speed(pred)
                if conflict is not None and not math.isnan(ox):
                    cdist = math.hypot(ox - conflict[1], oy - conflict[2])
                else:
                    cdist = float("nan")
                is_conflict = 0
                if c_id is not None:
                    if cid_i == c_id:
                        is_conflict = 1
                    elif prov == "anonymous" and not math.isnan(cdist) and cdist <= self._match_r:
                        is_conflict = 1
                self._writer.writerow(
                    common_head
                    + [_fmt(tid), _fmt(cid_i), source, prov,
                       _fmt(gen_tick), _fmt(pub_tick), _fmt(seq), tick,
                       _fmt(age_ms), _fmt(pub_age_ms),
                       _fmt(obs_tick), _fmt(frame_age_ms), lineage_ok,
                       _fmt(ox), _fmt(oy), _fmt(ospd),
                       _fmt(c_id), is_conflict, _fmt(cdist)]
                    + common_tail)
            self._fh.flush()
        except Exception:
            # Diagnostics must never take the edge loop down.
            return
