# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""AB3DMOT detection-bundle helpers.

Single source of truth for converting per-tick perception output into the
``(dets, info)`` bundle ``AB3DMOTWrapper.track()`` expects. Used by both the
edge's sensor-detection branch and the vehicle-side tracker. Anywhere a list
of ``ObstacleVehicle``-like objects feeds the tracker, route through here so
the KITTI/CARLA coordinate convention and column layout stay in one place.

Conventions
-----------
KITTI camera frame is used inside AB3DMOT:
    KITTI_x = CARLA_x, KITTI_y = CARLA_z (height), KITTI_z = CARLA_y

Row layouts:
    dets : (N, 8) float32 -- [h, w, l, kx, ky, kz, yaw, conf]
    info : (N, 3) int64   -- [frame_idx, guid, cid]

``cid`` is the carla_id when known (used by the edge for beacon detections to
carry identity) and ``-1`` for anonymous sensor detections. ``guid`` is a
process-monotonic counter the caller owns; pass any callable that returns the
next id.
"""
from __future__ import annotations

from typing import Callable, Iterable, List, Tuple

import numpy as np


def empty_bundle() -> dict:
    """Return a shape-correct empty AB3DMOT bundle."""
    return {
        'dets': np.zeros((0, 8), dtype=np.float32),
        'info': np.zeros((0, 3), dtype=np.int64),
    }


def vehicles_to_ab3dmot_rows(
    vehicles: Iterable,
    frame_idx: int,
    guid_provider: Callable[[], int],
    yaw: float = 0.0,
    conf: float = 0.5,
    cid: int = -1,
) -> Tuple[List[List[float]], List[List[int]]]:
    """Convert ObstacleVehicle-like objects to AB3DMOT (dets, info) rows.

    Returns lists, not arrays, so the caller can concatenate with other
    detection sources (e.g. the edge's beacon rows) before stacking. For a
    standalone bundle use ``vehicles_to_ab3dmot_dets``.

    Each input object must expose ``.location.{x,y,z}`` and
    ``.bounding_box.extent.{x,y,z}`` (half-extents, as CARLA reports).
    """
    det_rows: List[List[float]] = []
    info_rows: List[List[int]] = []
    for obj in vehicles or []:
        ext = obj.bounding_box.extent
        h, w, l = ext.z * 2, ext.y * 2, ext.x * 2
        loc = obj.location
        det_rows.append([h, w, l, loc.x, loc.z, loc.y, yaw, conf])
        info_rows.append([frame_idx, guid_provider(), cid])
    return det_rows, info_rows


def vehicles_world_edges(vehicles: Iterable) -> List[List[float]]:
    """World-frame edges [xmin, xmax, ymin, ymax] of each object's measured
    box: the corners of the fused point box when the object carries them
    (the raw visible extent, before any centre offset), else the bounding
    box the object was built from. The tracker publishes velocity from the
    slower-moving edge, so a parked vehicle whose visible side grows as a
    sensor passes is not read as moving."""
    rows: List[List[float]] = []
    for obj in vehicles or []:
        c = getattr(obj, "corner", None)
        if c is None:
            c = getattr(obj, "corners", None)
        try:
            c = None if c is None else np.asarray(c, dtype=float)
        except Exception:
            c = None
        if c is not None and c.ndim == 2 and c.shape[0] >= 2 and c.shape[1] >= 2:
            rows.append([float(c[:, 0].min()), float(c[:, 0].max()),
                         float(c[:, 1].min()), float(c[:, 1].max())])
            continue
        bb = obj.bounding_box
        loc, ext = bb.location, bb.extent
        rows.append([loc.x - ext.x, loc.x + ext.x, loc.y - ext.y, loc.y + ext.y])
    return rows


def vehicles_to_ab3dmot_dets(
    vehicles: Iterable,
    frame_idx: int,
    guid_provider: Callable[[], int],
    yaw: float = 0.0,
    conf: float = 0.5,
    cid: int = -1,
) -> dict:
    """Return a complete AB3DMOT detection bundle for sensor-detected vehicles."""
    det_rows, info_rows = vehicles_to_ab3dmot_rows(
        vehicles, frame_idx, guid_provider, yaw=yaw, conf=conf, cid=cid,
    )
    if not det_rows:
        return empty_bundle()
    return {
        'dets': np.asarray(det_rows, dtype=np.float32),
        'info': np.asarray(info_rows, dtype=np.int64),
    }


def stack_rows(
    det_rows: List[List[float]],
    info_rows: List[List[int]],
    edge_rows: List[List[float]] = None,
) -> dict:
    """Stack already-collected rows into the bundle layout; ``edge_rows``
    (one [xmin, xmax, ymin, ymax] per detection) rides along as 'edges'."""
    if not det_rows:
        return empty_bundle()
    out = {
        'dets': np.asarray(det_rows, dtype=np.float32),
        'info': np.asarray(info_rows, dtype=np.int64),
    }
    if edge_rows is not None and len(edge_rows) == len(det_rows):
        out['edges'] = np.asarray(edge_rows, dtype=np.float64)
    return out
