# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Vehicle-side 3D tracker + linear predictor.

This module wraps the same AB3DMOT tracker the edge uses, but instantiated
on the vehicle. It is the on-vehicle equivalent of the edge's
``track + predict`` stages. The intended consumer is the standard
cooperative-perception baseline, in which the edge publishes raw
multi-source detections (no tracking, no prediction) and the vehicle does
its own tracking and short-horizon prediction locally.

Without this module the vehicle's planner has nowhere to consume edge
detections from, because the planner expects future-trajectory predictions
in ``BehaviorAgent.edge_predictions``. With this module, edge-published
detections enter the local tracker, get associated to persistent vehicle-
local tracks, and are turned into per-track short-horizon predictions in
exactly the format the planner already consumes.

Reuses the production tracker and predictor:
    * AB3DMOT (``ecav.core.tracking.ab3dmot_wrapper.AB3DMOTWrapper``)
    * LinearPredictorManager
      (``ecav.core.prediction.linear_predictor_manager``)

No new ML; only placement. The point of the module is to make the
``edge publishes detections, vehicle tracks locally'' baseline runnable
without modifying the vehicle's planner.
"""
from __future__ import annotations

import logging
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper

logger = logging.getLogger(__name__)


class VehicleSideTracker:
    """Per-vehicle AB3DMOT + linear predictor for the standard
    cooperative-perception baseline.

    Construction is config-driven so a single YAML knob can toggle the
    baseline on without changing planner code paths.

    Parameters
    ----------
    cfg : dict
        ``tracker`` -- passed straight through to AB3DMOTWrapper.
        ``predictor`` -- LinearPredictorManager config (defaults to 10
        future steps).
        ``history_len`` -- per-track history depth retained for the
        predictor (default 10 frames).
    """

    def __init__(self, cfg: Optional[dict] = None) -> None:
        cfg = cfg or {}
        tracker_cfg = cfg.get("tracker", {})
        predictor_cfg = cfg.get("predictor", {})
        self._history_len = int(cfg.get("history_len", 10))

        self._tracker = AB3DMOTWrapper(tracker_cfg)
        self._predictor = LinearPredictorManager(
            cfg_or_steps=predictor_cfg.get("num_future_steps", 10),
        )

        # per-track history buffer of recent ObstacleTrajectory snapshots,
        # used by the linear predictor to fit a velocity.
        self._tracked: Dict[int, ObstacleTrajectory] = {}
        self._frame: int = 0

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------
    def process_detections(
        self,
        dets_all: Dict[str, np.ndarray],
        tick: int,
    ) -> List[ObstacleTrajectory]:
        """Run one tick of the local tracker on the edge-published
        detection list and return the resulting short-horizon predictions.

        Parameters
        ----------
        dets_all : dict
            AB3DMOT-format detection bundle. Keys: ``'dets'`` shape
            ``(N, 8)`` ``[h, w, l, x, y, z, yaw, score]`` and ``'info'``
            shape ``(N, 3)`` ``[frame, det_idx, source_id]``. Empty dict
            (no detections this tick) is permitted.
        tick : int
            Simulator tick at which the detections were generated. Used
            as the ``source_tick`` for the predictor's AoI metadata.

        Returns
        -------
        list of ObstacleTrajectory
            Per-track short-horizon predictions in world coordinates,
            ready to be assigned to ``vm.agent.edge_predictions``.
        """
        self._frame = int(tick)
        results, _ = self._tracker.track(dets_all, self._frame)
        # Rebuild the tracked-trajectory cache from the tracker's output.
        self._update_trajectory_cache(results)
        preds = self._predictor.generate_predicted_trajectories(
            self._tracked,
            source_tick=self._frame,
            publish_tick=self._frame,
        )
        return preds

    # ------------------------------------------------------------------
    def reset(self) -> None:
        """Forget all tracker state. Used between scenarios or on lost
        connection beyond the holdover budget."""
        self._tracker.reset()
        self._tracked.clear()
        self._frame = 0

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------
    def _update_trajectory_cache(self, tracker_rows: List[np.ndarray]) -> None:
        """Convert per-frame AB3DMOT output into the tracked-trajectory
        dict the LinearPredictor expects.

        Each row from ``AB3DMOT.track`` carries
        ``[h, w, l, x, y, z, yaw, id, ...]``. We append a fresh waypoint
        to the matching ObstacleTrajectory and prune unmatched tracks.
        """
        live_ids: set[int] = set()
        for row in tracker_rows:
            if row is None or len(row) == 0:
                continue
            for trk in np.atleast_2d(row):
                if len(trk) < 8:
                    continue
                h, w, l, x, y, z, yaw, tid = trk[:8]
                tid = int(tid)
                live_ids.add(tid)
                # Lazy create an ObstacleTrajectory the first time we see a track.
                if tid not in self._tracked:
                    obstacle = _make_obstacle_vehicle(tid, x, y, z, yaw, l, w, h)
                    self._tracked[tid] = ObstacleTrajectory(obstacle, [])
                self._tracked[tid].update(_make_transform(x, y, z, yaw))

        # Drop tracks the tracker no longer reports.
        for tid in list(self._tracked):
            if tid not in live_ids:
                # Mild aging instead of immediate drop: the predictor can still
                # extrapolate a couple of ticks before we discard the track.
                self._tracked[tid].age(dt=0.05)
                if self._tracked[tid].time_since_update > 0.5:
                    del self._tracked[tid]


# ──────────────────────────────────────────────────────────────────────────────
#  Lightweight helpers (avoid pulling carla.* into a non-CARLA codepath)
# ──────────────────────────────────────────────────────────────────────────────
def _make_obstacle_vehicle(tid: int, x: float, y: float, z: float,
                            yaw: float, l: float, w: float, h: float):
    """Construct a minimal ObstacleVehicle stand-in.

    The downstream predictor + planner read .id and .get_transform()-ish
    fields; full CARLA Actor methods are not required for the
    linear-predictor path. We construct a lightweight duck-typed object
    that exposes only what the predictor consumes.
    """
    return _LightweightObstacle(
        id=tid,
        x=float(x), y=float(y), z=float(z),
        yaw=float(yaw),
        length=float(l), width=float(w), height=float(h),
    )


def _make_transform(x: float, y: float, z: float, yaw: float):
    """Build a transform-like object exposing .location.x/y/z and
    .rotation.yaw, matching what ObstacleTrajectory.append() consumes."""
    return _LightweightTransform(x=float(x), y=float(y), z=float(z), yaw=float(yaw))


class _LightweightObstacle:
    """Duck-typed obstacle exposing the surface the predictor needs."""

    __slots__ = ("id", "x", "y", "z", "yaw", "length", "width", "height")

    def __init__(self, *, id: int, x: float, y: float, z: float, yaw: float,
                 length: float, width: float, height: float) -> None:
        self.id = id
        self.x = x; self.y = y; self.z = z
        self.yaw = yaw
        self.length = length; self.width = width; self.height = height


class _LightweightTransform:
    """Duck-typed transform with .location and .rotation accessors."""

    __slots__ = ("location", "rotation")

    def __init__(self, *, x: float, y: float, z: float, yaw: float) -> None:
        self.location = _LightweightLocation(x, y, z)
        self.rotation = _LightweightRotation(yaw)


class _LightweightLocation:
    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float) -> None:
        self.x = x; self.y = y; self.z = z

    def as_vector_2D(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=np.float32)


class _LightweightRotation:
    __slots__ = ("yaw",)

    def __init__(self, yaw: float) -> None:
        self.yaw = yaw
