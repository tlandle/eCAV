# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
BSM J2945-inspired temporary ID rotation for the eCloud anchoring protocol.

Each vehicle broadcasts a beacon containing a *temporary ID* (temp_id) instead
of its true CARLA actor id.  The temp_id is rotated periodically based on:

* **Time threshold** -- every ``rotation_interval_ticks`` simulation ticks
  (default 200 = 10 s at 0.05 s/tick).
* **Distance threshold** -- after travelling ``rotation_distance_m`` metres
  since the last rotation (default 100 m).

The edge's anchoring protocol must detect ID changes and reconcile them using
spatiotemporal continuity so that the tracker can maintain consistent track
identity despite the pseudonymous ID changes.

Usage
-----
Instantiate one ``BeaconIdManager`` per edge manager.  On every tick, call
``get_temp_id(carla_id, position, tick)`` for each managed vehicle to obtain its
current temp_id (which may rotate).  When a rotation occurs the manager logs
the event.  Call ``reconcile_id_change(...)`` to verify that a previously-known
temp_id and a newly-appeared temp_id plausibly belong to the same physical
vehicle (spatiotemporal consistency check).  If reconciliation succeeds, call
``remap_tracker_identity(tracker, old_temp_id, new_temp_id)`` to update the
AB3DMOT tracker's internal state.
"""

from __future__ import annotations

import logging
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("BeaconIdManager")


# ──────────────────────────────────────────────────────────────────────────────
#  Per-vehicle record
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class _VehicleRecord:
    """Internal bookkeeping for one managed vehicle."""
    carla_id: int
    temp_id: int = 0
    last_rotation_tick: int = 0
    last_rotation_position: Optional[np.ndarray] = None
    last_position: Optional[np.ndarray] = None
    last_velocity: Optional[np.ndarray] = None
    last_tick: int = 0
    # Heading (rad) and velocity (m/s) reported by the last ACCEPTED beacon,
    # for the ValidBeacon residual checks; None until a beacon carried them.
    last_yaw: Optional[float] = None
    last_meas_velocity: Optional[np.ndarray] = None


# ──────────────────────────────────────────────────────────────────────────────
#  Rotation-event log entry
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class RotationEvent:
    """Immutable record of a single temp_id rotation."""
    carla_id: int
    old_temp_id: int
    new_temp_id: int
    tick: int
    position: np.ndarray
    velocity: Optional[np.ndarray]
    reason: str            # "time", "distance", or "both"


# ──────────────────────────────────────────────────────────────────────────────
#  Main class
# ──────────────────────────────────────────────────────────────────────────────
class BeaconIdManager:
    """Manage pseudonymous temporary IDs for BSM-style beacon rotation.

    Parameters
    ----------
    rotation_interval_ticks : int
        Number of simulation ticks between forced rotations (default 200).
    rotation_distance_m : float
        Distance in metres between forced rotations (default 100.0).
    world_dt : float
        Duration of one simulation tick in seconds (default 0.05).
    reconciliation_tolerance : float
        Multiplier on the velocity-based distance gate used in
        ``reconcile_id_change`` (default 1.5 -- allows 50 % slack).
    """

    def __init__(
        self,
        rotation_interval_ticks: int = 200,
        rotation_distance_m: float = 100.0,
        world_dt: float = 0.05,
        reconciliation_tolerance: float = 1.5,
        validity_checks: bool = False,
        max_accel_mps2: float = 8.0,
        max_yaw_rate_rps: float = 1.0,
        max_speed_mps: float = 40.0,
        max_beacon_age_ticks: Optional[int] = None,
    ):
        self.rotation_interval_ticks = rotation_interval_ticks
        self.rotation_distance_m = rotation_distance_m
        self.world_dt = world_dt
        self.reconciliation_tolerance = reconciliation_tolerance
        # ValidBeacon (paper Sec. 4.3): a beacon updates the anchored track
        # only if it is recent and its displacement, acceleration, yaw-rate,
        # and speed residuals are physically reachable. Off by default so
        # existing arms are unchanged; the bounds are the paper's defaults:
        #   max_accel_mps2       8.0  (hard braking / strong launch, about 0.8 g)
        #   max_yaw_rate_rps     1.0  (tight urban turn at low speed)
        #   max_speed_mps       40.0  (144 km/h, above any urban scenario)
        #   max_beacon_age_ticks None (recency check off unless set)
        self.validity_checks = bool(validity_checks)
        self.max_accel_mps2 = float(max_accel_mps2)
        self.max_yaw_rate_rps = float(max_yaw_rate_rps)
        self.max_speed_mps = float(max_speed_mps)
        self.max_beacon_age_ticks = (None if max_beacon_age_ticks is None
                                     else int(max_beacon_age_ticks))
        self._validity_keys = ('accepted', 'rejected_displacement', 'rejected_accel',
                               'rejected_yaw_rate', 'rejected_speed', 'rejected_recency')
        self.validity_counts: Dict[str, int] = {k: 0 for k in self._validity_keys}
        self._validity_tick_counts: Dict[str, int] = {k: 0 for k in self._validity_keys}

        # carla_id -> _VehicleRecord
        self._records: Dict[int, _VehicleRecord] = {}
        # temp_id  -> carla_id  (reverse lookup for reconciliation)
        self._temp_to_carla: Dict[int, int] = {}
        # Chronological log of all rotations (bounded)
        self._rotation_log: List[RotationEvent] = []
        # Pending rotations that the edge manager has not yet reconciled
        self._pending_rotations: List[RotationEvent] = []

    # ── helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _random_temp_id() -> int:
        """Return a random 32-bit positive integer suitable as a temp_id."""
        return random.randint(1, 0x7FFF_FFFF)

    @staticmethod
    def _position_array(position) -> np.ndarray:
        """Coerce *position* to a (3,) float32 array.

        Accepts ``carla.Location``, a 3-tuple/list, or an existing ndarray.
        """
        if hasattr(position, "x") and hasattr(position, "y"):
            z = getattr(position, "z", 0.0)
            return np.asarray([position.x, position.y, z], dtype=np.float32)
        return np.asarray(position, dtype=np.float32).ravel()[:3]

    # ── public API ────────────────────────────────────────────────────────

    def get_temp_id(self, carla_id: int, position, tick: int,
                    yaw: Optional[float] = None, velocity=None) -> int:
        """Return the current temporary ID for *carla_id*, rotating if needed.

        Parameters
        ----------
        carla_id : int
            The true CARLA actor id of the vehicle.
        position : carla.Location | array-like
            Current world position of the vehicle.
        tick : int
            Current simulation tick.
        yaw, velocity : optional
            Heading (rad) and velocity (m/s) the beacon reported; stored
            on the record for the ValidBeacon residual checks.

        Returns
        -------
        int
            The (possibly freshly rotated) temporary ID.
        """
        pos = self._position_array(position)
        velocity_in = velocity

        # --- first encounter: initialise record ---
        if carla_id not in self._records:
            tid = self._random_temp_id()
            rec = _VehicleRecord(
                carla_id=carla_id,
                temp_id=tid,
                last_rotation_tick=tick,
                last_rotation_position=pos.copy(),
                last_position=pos.copy(),
                last_velocity=np.zeros(3, dtype=np.float32),
                last_tick=tick,
            )
            self._records[carla_id] = rec
            self._temp_to_carla[tid] = carla_id
            self._store_meas(rec, yaw, velocity)
            return tid

        rec = self._records[carla_id]

        # --- compute velocity from position delta ---
        dt_ticks = max(tick - rec.last_tick, 1)
        velocity = (pos - rec.last_position) / (dt_ticks * self.world_dt) \
            if rec.last_position is not None else np.zeros(3, dtype=np.float32)
        rec.last_velocity = velocity
        rec.last_position = pos.copy()
        rec.last_tick = tick
        self._store_meas(rec, yaw, velocity_meas=velocity_in)

        # --- check rotation triggers ---
        time_triggered = (tick - rec.last_rotation_tick) >= self.rotation_interval_ticks
        dist_triggered = False
        if rec.last_rotation_position is not None:
            dist_since = float(np.linalg.norm(pos[:2] - rec.last_rotation_position[:2]))
            dist_triggered = dist_since >= self.rotation_distance_m

        if time_triggered or dist_triggered:
            reason = ("both" if (time_triggered and dist_triggered)
                      else ("time" if time_triggered else "distance"))
            old_tid = rec.temp_id
            new_tid = self._random_temp_id()

            # Update mappings — keep old temp_id in reverse dict so that
            # get_carla_id_for_temp() still works for tracker output arrays
            # generated before remap_tracker_identity() runs (same-tick race).
            self._temp_to_carla[new_tid] = carla_id

            event = RotationEvent(
                carla_id=carla_id,
                old_temp_id=old_tid,
                new_temp_id=new_tid,
                tick=tick,
                position=pos.copy(),
                velocity=velocity.copy(),
                reason=reason,
            )
            self._rotation_log.append(event)
            self._pending_rotations.append(event)

            # Trim rotation log to last 1000 entries
            if len(self._rotation_log) > 1000:
                self._rotation_log = self._rotation_log[-500:]

            logger.info(
                "Temp-ID rotation: carla_id=%d  %d -> %d  reason=%s  tick=%d",
                carla_id, old_tid, new_tid, reason, tick,
            )

            rec.temp_id = new_tid
            rec.last_rotation_tick = tick
            rec.last_rotation_position = pos.copy()

        return rec.temp_id

    # ------------------------------------------------------------------
    def _store_meas(self, rec: _VehicleRecord, yaw, velocity=None, velocity_meas=None) -> None:
        v = velocity_meas if velocity_meas is not None else velocity
        if yaw is not None:
            rec.last_yaw = float(yaw)
        if v is not None:
            rec.last_meas_velocity = self._position_array(v)

    def _count(self, key: str) -> None:
        self.validity_counts[key] += 1
        self._validity_tick_counts[key] += 1

    def pop_validity_counts(self) -> Dict[str, int]:
        """Return and reset the per-tick ValidBeacon counters, keyed
        beacon_<key> for the edge profiler."""
        out = {'beacon_' + k: v for k, v in self._validity_tick_counts.items()}
        self._validity_tick_counts = {k: 0 for k in self._validity_keys}
        return out

    def validate_beacon(
        self,
        carla_id: int,
        position,
        tick: int,
        yaw: Optional[float] = None,
        velocity=None,
        stamp_tick: Optional[int] = None,
    ) -> Tuple[bool, str]:
        """ValidBeacon: accept a beacon as an anchor update only if it is
        recent and physically plausible relative to the last accepted beacon
        of the same vehicle. Returns (accepted, reason). With
        ``validity_checks`` off every beacon is accepted (today's behaviour).
        The first beacon of a vehicle is always accepted: it is the bootstrap
        that creates the anchored track. On rejection the record is left
        untouched, so the next beacon is judged against the last accepted
        state and the vehicle is tracked anonymously until a valid beacon
        arrives (the same path as a failed rotation reconcile).
        """
        if not self.validity_checks:
            self._count('accepted')
            return True, 'off'
        rec = self._records.get(carla_id)
        if rec is None or rec.last_position is None:
            self._count('accepted')
            return True, 'bootstrap'
        if (self.max_beacon_age_ticks is not None and stamp_tick is not None
                and abs(int(tick) - int(stamp_tick)) > self.max_beacon_age_ticks):
            self._count('rejected_recency')
            return False, 'recency'
        pos = self._position_array(position)[:2]
        elapsed_s = max(tick - rec.last_tick, 1) * self.world_dt
        # Predict from the last accepted beacon's reported velocity when it
        # carried one, else from the position-delta velocity (as reconcile).
        _v_src = (rec.last_meas_velocity if rec.last_meas_velocity is not None
                  else rec.last_velocity)
        vel_prev = (np.asarray(_v_src, dtype=np.float32).ravel()[:2]
                    if _v_src is not None else np.zeros(2, np.float32))
        predicted = rec.last_position[:2] + vel_prev * elapsed_s
        gate = max(self.reconciliation_tolerance * float(np.linalg.norm(vel_prev)) * elapsed_s, 5.0)
        if float(np.linalg.norm(pos - predicted)) > gate:
            self._count('rejected_displacement')
            return False, 'displacement'
        if velocity is not None:
            v_now = self._position_array(velocity)[:2]
            if float(np.linalg.norm(v_now)) > self.max_speed_mps:
                self._count('rejected_speed')
                return False, 'speed'
            if rec.last_meas_velocity is not None:
                accel = float(np.linalg.norm(v_now - rec.last_meas_velocity[:2])) / elapsed_s
                if accel > self.max_accel_mps2:
                    self._count('rejected_accel')
                    return False, 'accel'
        if yaw is not None and rec.last_yaw is not None:
            d = float(yaw) - float(rec.last_yaw)
            d = (d + np.pi) % (2 * np.pi) - np.pi
            if abs(d) / elapsed_s > self.max_yaw_rate_rps:
                self._count('rejected_yaw_rate')
                return False, 'yaw_rate'
        self._count('accepted')
        return True, 'ok'

    # ------------------------------------------------------------------

    def reconcile_id_change(
        self,
        old_temp_id: int,
        new_temp_id: int,
        old_position: np.ndarray,
        new_position: np.ndarray,
        old_velocity: np.ndarray,
        elapsed_ticks: int,
    ) -> bool:
        """Check whether *old_temp_id* and *new_temp_id* plausibly belong to
        the same physical vehicle based on spatiotemporal continuity.

        The test is: the new position must be within
        ``reconciliation_tolerance * ||velocity|| * elapsed_time`` of the
        predicted position from the old state.

        Parameters
        ----------
        old_temp_id : int
            Previously known temporary ID.
        new_temp_id : int
            Newly observed temporary ID.
        old_position : array-like, shape (2,) or (3,)
            Last known position of the old temp_id.
        new_position : array-like, shape (2,) or (3,)
            Observed position of the new temp_id.
        old_velocity : array-like, shape (2,) or (3,)
            Velocity vector (m/s) of the old temp_id.
        elapsed_ticks : int
            Number of ticks between old and new observations.

        Returns
        -------
        bool
            True if the two IDs are consistent with being the same vehicle.
        """
        old_pos = np.asarray(old_position, dtype=np.float32).ravel()[:2]
        new_pos = np.asarray(new_position, dtype=np.float32).ravel()[:2]
        vel = np.asarray(old_velocity, dtype=np.float32).ravel()[:2]

        elapsed_s = elapsed_ticks * self.world_dt

        # Predicted position under constant velocity
        predicted_pos = old_pos + vel * elapsed_s
        displacement = float(np.linalg.norm(new_pos - predicted_pos))

        # Gate: allow reconciliation_tolerance * (speed * elapsed_time)
        speed = float(np.linalg.norm(vel))
        gate = self.reconciliation_tolerance * speed * elapsed_s
        # Minimum gate of 5 m to handle near-stationary vehicles
        gate = max(gate, 5.0)

        ok = displacement <= gate
        if ok and self.validity_checks and speed > self.max_speed_mps:
            ok = False
        logger.debug(
            "Reconcile %d->%d: displacement=%.2f  gate=%.2f  %s",
            old_temp_id, new_temp_id, displacement, gate,
            "PASS" if ok else "FAIL",
        )
        return ok

    # ------------------------------------------------------------------

    def pop_pending_rotations(self) -> List[RotationEvent]:
        """Return and clear the list of rotations not yet handled by the edge.

        The edge manager should call this once per tick, iterate through the
        events, and for each successful ``reconcile_id_change`` call
        ``remap_tracker_identity``.
        """
        events = self._pending_rotations
        self._pending_rotations = []
        return events

    # ------------------------------------------------------------------

    @staticmethod
    def remap_tracker_identity(tracker, old_temp_id: int, new_temp_id: int) -> bool:
        """Update the AB3DMOT tracker so that the track currently associated
        with *old_temp_id* is reassigned to *new_temp_id*.

        This modifies ``tracker.trackers[i].carla_id`` in-place.  The tracker
        uses ``carla_id`` (which now stores a temp_id) for identity-aware
        association in ``matching.py``.

        Parameters
        ----------
        tracker : AB3DMOT
            The tracker instance whose internal state should be updated.
        old_temp_id : int
            The temp_id value currently stored on the KF track.
        new_temp_id : int
            The replacement temp_id value.

        Returns
        -------
        bool
            True if a matching track was found and updated, False otherwise.
        """
        # Track pool resolution is tracker-agnostic: AB3DMOT exposes
        # .trackers; Mamba3DMOTWrapper exposes .tracker.tracked_tracklets.
        # Both track objects carry the carla_id attribute rewritten here.
        if hasattr(tracker, 'trackers'):
            pool = tracker.trackers
        else:
            pool = getattr(getattr(tracker, 'tracker', None),
                           'tracked_tracklets', [])
        for kf_track in pool:
            if getattr(kf_track, "carla_id", -1) == old_temp_id:
                kf_track.carla_id = new_temp_id
                kf_track.anchoring_age = 0  # reset epoch on successful remap
                logger.info(
                    "Tracker remap: track %d  carla_id %d -> %d",
                    getattr(kf_track, 'id', getattr(kf_track, 'track_id', -1)),
                    old_temp_id, new_temp_id,
                )
                return True
        logger.warning(
            "Tracker remap FAILED: no track with carla_id=%d", old_temp_id,
        )
        return False

    # ------------------------------------------------------------------

    def get_carla_id_for_temp(self, temp_id: int) -> Optional[int]:
        """Reverse-lookup: return the carla_id that currently owns *temp_id*,
        or None if unknown."""
        return self._temp_to_carla.get(temp_id)

    def get_record(self, carla_id: int) -> Optional[_VehicleRecord]:
        """Return the internal record for *carla_id* (for testing/debug)."""
        return self._records.get(carla_id)

    @property
    def rotation_log(self) -> List[RotationEvent]:
        """Full chronological rotation history."""
        return self._rotation_log

    def reset(self) -> None:
        """Clear all state (useful between scenarios)."""
        self._records.clear()
        self._temp_to_carla.clear()
        self._rotation_log.clear()
        self._pending_rotations.clear()
