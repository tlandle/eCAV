# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Ego-Uniqueness Monitor for edge-assisted driving.

Detects when the edge tracker maintains multiple active tracks for the
same physical vehicle — a distributed correctness failure called
"self-ghosting."

Three orthogonal metrics:

1. **IdDup** (identity-layer, anchoring-dependent):
   Tracks with known carla_id — violation when >1 track shares an ID.

2. **GT-IdDup** (oracle-assisted identity, anchoring-agnostic but GT-
   dependent):  Position-match anonymous tracks to GT vehicles; violation
   when >1 track maps to the same GT actor.

3. **GeomDup** (purely geometric, always defined):
   For each ego, cluster nearby tracks by pairwise distance; violation
   when a cluster has ≥2 members.  No carla_id or GT required.

Together these let the paper report:
    - GeomDup / km  (physical NMS leak rate, fair across anchoring modes)
    - IdDup / km    (identity-layer failure, only when anchoring ON)
    - id_coverage   (fraction of tracks with carla_id != -1)
"""

from __future__ import annotations

import math
import numpy as np
from collections import defaultdict, Counter
from dataclasses import dataclass, field
from typing import Dict, List, Set, Optional, Tuple


@dataclass
class ViolationEvent:
    """A period during which a single vehicle had multiple active tracks."""
    carla_id: int
    start_tick: int
    end_tick: int = -1          # -1 while ongoing
    track_ids: Set[int] = field(default_factory=set)
    is_ego: bool = False        # True if this identity is a managed vehicle

    @property
    def duration(self) -> int:
        if self.end_tick == -1:
            return 0
        return self.end_tick - self.start_tick


@dataclass
class TickRecord:
    """Per-tick snapshot of identity → track mapping."""
    tick: int
    identity_map: Dict[int, Set[int]] = field(default_factory=dict)
    num_duplicates: int = 0
    duplicate_identities: List[int] = field(default_factory=list)
    # Geometric duplicates (always defined)
    geom_dup_clusters: int = 0
    geom_dup_tracks: int = 0
    # Identity coverage
    id_coverage_count: int = 0
    total_tracks: int = 0


class EgoUniquenessMonitor:
    """
    Monitors AB3DMOT tracker output for Ego-Uniqueness violations.

    A violation occurs when more than one active track corresponds to the
    same physical vehicle at the same tick. Detection uses three sources:

    1. Beacon identity: tracks with known carla_id (from self-beacon anchoring)
    2. Position matching: tracks with carla_id=-1 matched to GT vehicles
       within a distance threshold
    3. Geometric clustering: pairwise proximity of track centers near each
       ego (always defined, no identity needed)
    """

    GT_MATCH_THRESHOLD = 3.0  # meters — max distance for position-based identity

    def __init__(self, geom_roi_radius: float = 8.0,
                 geom_eps: float = 1.0):
        """
        Args:
            geom_roi_radius: radius (m) around each ego for geometric
                duplicate ROI.  Tracks beyond this are ignored.
            geom_eps: pairwise distance threshold (m) for clustering
                two tracks as geometric duplicates.
        """
        self.violation_events: List[ViolationEvent] = []
        self.per_tick_records: List[TickRecord] = []
        self._active_violations: Dict[int, ViolationEvent] = {}
        self._tick_count = 0
        self._total_tracks = 0
        self._total_duplicates = 0
        # Geometric duplicate detector config
        self._geom_roi_radius = geom_roi_radius
        self._geom_eps = geom_eps
        # Geometric aggregate counters
        self._total_geom_dup_clusters = 0
        self._total_geom_dup_tracks = 0
        self._total_id_coverage = 0

    def update(self, tick: int, tracks: np.ndarray,
               gt_snapshot: Optional[Dict] = None,
               managed_vehicle_ids: Optional[Set[int]] = None,
               ego_poses: Optional[Dict[int, Tuple[float, float, float]]] = None):
        """
        Analyze tracks for Ego-Uniqueness violations.

        Args:
            tick: current simulation tick
            tracks: AB3DMOT output array, shape (N, 13+)
                    columns: [h,w,l,x,y,z,yaw, track_id, carla_id, guid, ...]
            gt_snapshot: {carla_actor_id: {'x','y','z','speed','type',...}}
            managed_vehicle_ids: set of CARLA IDs for ego/managed vehicles
            ego_poses: {vehicle_id: (x, y, yaw_rad)} for geometric dup
                       detection.  If None, geometric check is skipped.
        """
        if managed_vehicle_ids is None:
            managed_vehicle_ids = set()

        self._tick_count += 1

        # Build identity_map: carla_id → set of track_ids
        identity_map: Dict[int, Set[int]] = defaultdict(set)

        if tracks is None or len(tracks) == 0:
            self._close_all_violations(tick)
            self.per_tick_records.append(TickRecord(tick=tick))
            return

        n_tracks = len(tracks)
        self._total_tracks += n_tracks

        # === Collect per-track data ===
        track_data = []  # list of (track_id, carla_id, x, y)
        id_coverage_count = 0
        for trk in tracks:
            track_id = int(trk[7])
            carla_id = int(trk[8])
            # KITTI x(idx 3) = CARLA x, KITTI z(idx 5) = CARLA y
            tx, ty = float(trk[3]), float(trk[5])
            track_data.append((track_id, carla_id, tx, ty))

            if carla_id != -1:
                id_coverage_count += 1
                # Known identity from beacon
                identity_map[carla_id].add(track_id)
            elif gt_snapshot is not None:
                # Try to match to GT vehicle by position
                best_dist, best_id = self.GT_MATCH_THRESHOLD, None
                for vid, vdata in gt_snapshot.items():
                    dist = math.sqrt(
                        (tx - vdata['x'])**2 + (ty - vdata['y'])**2)
                    if dist < best_dist:
                        best_dist = dist
                        best_id = vid
                if best_id is not None:
                    identity_map[best_id].add(track_id)

        self._total_id_coverage += id_coverage_count

        # === Detect identity violations: any identity with >1 track ===
        duplicate_identities = []
        num_duplicates = 0
        for cid, tids in identity_map.items():
            if len(tids) > 1:
                duplicate_identities.append(cid)
                num_duplicates += len(tids) - 1
                self._total_duplicates += len(tids) - 1

                # Start or continue violation event
                if cid in self._active_violations:
                    self._active_violations[cid].track_ids.update(tids)
                else:
                    event = ViolationEvent(
                        carla_id=cid,
                        start_tick=tick,
                        track_ids=set(tids),
                        is_ego=(cid in managed_vehicle_ids),
                    )
                    self._active_violations[cid] = event
            else:
                # No violation for this identity — close any active event
                if cid in self._active_violations:
                    self._active_violations[cid].end_tick = tick
                    self.violation_events.append(
                        self._active_violations.pop(cid))

        # Close violations for identities no longer present
        current_ids = set(identity_map.keys())
        for cid in list(self._active_violations.keys()):
            if cid not in current_ids:
                self._active_violations[cid].end_tick = tick
                self.violation_events.append(
                    self._active_violations.pop(cid))

        # === Geometric duplicate detector (always-on) ===
        # Global check: collect all tracks within roi_radius of ANY
        # managed ego, then cluster by pairwise distance.  This gives
        # one count per tick that is comparable across anchoring modes.
        geom_dup_clusters = 0
        geom_dup_tracks = 0

        if ego_poses and n_tracks >= 2:
            roi_r2 = self._geom_roi_radius ** 2

            # Gather indices of tracks within ROI of any managed ego
            roi_set = set()
            for ex, ey, _ in ego_poses.values():
                for idx, (_, _, tx, ty) in enumerate(track_data):
                    if (tx - ex)**2 + (ty - ey)**2 <= roi_r2:
                        roi_set.add(idx)

            roi_idx = sorted(roi_set)
            n = len(roi_idx)

            if n >= 2:
                # Union-find clustering by pairwise distance
                parent = list(range(n))

                def _find(i, p=parent):
                    while p[i] != i:
                        p[i] = p[p[i]]
                        i = p[i]
                    return i

                def _union(i, j, p=parent):
                    pi, pj = _find(i), _find(j)
                    if pi != pj:
                        p[pi] = pj

                eps2 = self._geom_eps ** 2
                for i in range(n):
                    _, _, xi, yi = track_data[roi_idx[i]]
                    for j in range(i + 1, n):
                        _, _, xj, yj = track_data[roi_idx[j]]
                        if (xi - xj)**2 + (yi - yj)**2 <= eps2:
                            _union(i, j)

                cluster_sizes = Counter(_find(i) for i in range(n))
                for size in cluster_sizes.values():
                    if size >= 2:
                        geom_dup_clusters += 1
                        geom_dup_tracks += size

        self._total_geom_dup_clusters += geom_dup_clusters
        self._total_geom_dup_tracks += geom_dup_tracks

        self.per_tick_records.append(TickRecord(
            tick=tick,
            identity_map=dict(identity_map),
            num_duplicates=num_duplicates,
            duplicate_identities=duplicate_identities,
            geom_dup_clusters=geom_dup_clusters,
            geom_dup_tracks=geom_dup_tracks,
            id_coverage_count=id_coverage_count,
            total_tracks=n_tracks,
        ))

    def _close_all_violations(self, tick: int):
        for cid, event in self._active_violations.items():
            event.end_tick = tick
            self.violation_events.append(event)
        self._active_violations.clear()

    def get_metrics(self) -> Dict:
        """Return aggregate Ego-Uniqueness metrics."""
        # Close any still-active violations for accounting
        all_events = list(self.violation_events)
        for event in self._active_violations.values():
            all_events.append(event)

        durations = [e.duration for e in all_events if e.duration > 0]
        ego_violations = [e for e in all_events if e.is_ego]
        ticks_with_violations = sum(
            1 for r in self.per_tick_records if r.num_duplicates > 0
        )
        ticks_with_geom_dups = sum(
            1 for r in self.per_tick_records if r.geom_dup_clusters > 0
        )

        # Compute violations per minute (at 0.05s per tick = 20 ticks/s)
        total_seconds = self._tick_count * 0.05
        total_minutes = total_seconds / 60.0 if total_seconds > 0 else 1.0

        return {
            # --- Identity-layer duplicates (anchoring-dependent) ---
            'total_violations': len(all_events),
            'violations_per_min': len(all_events) / total_minutes,
            'ticks_with_violations': ticks_with_violations,
            'violation_tick_fraction': (
                ticks_with_violations / max(1, self._tick_count)
            ),
            'duplicate_track_rate': (
                self._total_duplicates / max(1, self._total_tracks)
            ),
            'duplicate_lifetime_mean_ticks': (
                float(np.mean(durations)) if durations else 0.0
            ),
            'duplicate_lifetime_p95_ticks': (
                float(np.percentile(durations, 95)) if durations else 0.0
            ),
            'ego_ghost_violation_count': len(ego_violations),
            # --- Geometric duplicates (always defined) ---
            'geom_dup_total_clusters': self._total_geom_dup_clusters,
            'geom_dup_total_tracks': self._total_geom_dup_tracks,
            'geom_dup_ticks_with': ticks_with_geom_dups,
            'geom_dup_tick_fraction': (
                ticks_with_geom_dups / max(1, self._tick_count)
            ),
            'geom_dup_clusters_per_min': (
                self._total_geom_dup_clusters / total_minutes
            ),
            # --- Identity coverage ---
            'id_coverage_fraction': (
                self._total_id_coverage / max(1, self._total_tracks)
            ),
        }
