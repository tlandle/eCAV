# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Locale: a coverage region served by one or more RSUs and a radio cell.

A locale is the spatial unit across which the predictive latent migration
protocol operates. Locales are distinct from edge servers: one edge server
may host multiple locales, and two adjacent locales may live on different
edge servers. The protocol described in
``ecav.core.application.edge.migration`` operates between locales and is
agnostic to whether the source and destination share a host.

The Locale geometry is a closed polygon in the shared map frame. The class
exposes containment, signed distance to boundary, and exit-prediction tests
used by the trajectory trigger.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class Locale:
    """A coverage region identified by ``locale_id`` and a polygon boundary.

    Polygons are stored as an ordered sequence of ``(x, y)`` vertices in the
    shared map frame. The polygon is assumed simple and closed (the closing
    edge from the last vertex back to the first is implicit).
    """

    locale_id: str
    polygon: np.ndarray  # shape (N, 2), float64
    rsu_ids: Tuple[int, ...] = field(default_factory=tuple)
    edge_host_id: Optional[str] = None  # which edge server hosts this locale

    def __post_init__(self) -> None:
        arr = np.asarray(self.polygon, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] < 3:
            raise ValueError(
                f"Locale polygon must have shape (N>=3, 2); got {arr.shape}"
            )
        self.polygon = arr

    # ------------------------------------------------------------------
    # Geometry
    # ------------------------------------------------------------------
    def contains(self, point: Sequence[float]) -> bool:
        """Return True iff ``point`` lies inside the polygon (ray casting)."""
        x, y = float(point[0]), float(point[1])
        inside = False
        n = len(self.polygon)
        j = n - 1
        for i in range(n):
            xi, yi = self.polygon[i]
            xj, yj = self.polygon[j]
            intersect = ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            )
            if intersect:
                inside = not inside
            j = i
        return inside

    def signed_distance(self, point: Sequence[float]) -> float:
        """Negative inside, positive outside; magnitude is Euclidean distance
        to the nearest polygon edge."""
        x, y = float(point[0]), float(point[1])
        d = self._distance_to_polygon(x, y)
        return -d if self.contains(point) else d

    def _distance_to_polygon(self, x: float, y: float) -> float:
        dists = []
        n = len(self.polygon)
        for i in range(n):
            a = self.polygon[i]
            b = self.polygon[(i + 1) % n]
            dists.append(_point_segment_distance(x, y, a[0], a[1], b[0], b[1]))
        return float(min(dists))

    # ------------------------------------------------------------------
    # Crossing prediction
    # ------------------------------------------------------------------
    def predicted_to_exit_within(
        self,
        trajectory: np.ndarray,
        horizon_s: float,
        tick_dt_s: float,
    ) -> bool:
        """Return True iff any waypoint of ``trajectory`` (shape (T, 2)) within
        the first ``horizon_s`` seconds falls outside the locale.

        ``trajectory[k]`` is the predicted ``(x, y)`` position at time
        ``k * tick_dt_s`` from now.
        """
        if trajectory.size == 0:
            return False
        horizon_steps = int(horizon_s / tick_dt_s)
        for k in range(min(horizon_steps, len(trajectory))):
            if not self.contains(trajectory[k]):
                return True
        return False


# ──────────────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────────────
def _point_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    """Shortest distance from point (px,py) to segment (ax,ay)-(bx,by)."""
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return float(np.hypot(px - ax, py - ay))
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    cx, cy = ax + t * dx, ay + t * dy
    return float(np.hypot(px - cx, py - cy))
