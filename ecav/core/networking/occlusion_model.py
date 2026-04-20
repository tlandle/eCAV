# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
CARLA ray-cast based occlusion model for V2X link characterization.

Three occlusion levels per link:
  1. Clear LOS: no obstructions. LOS path loss.
  2. Vehicle-blocked: ray hits vehicle bodies only. LOS base + 15 dB per
     blocking vehicle (Boban et al., IEEE TAP 2014; He et al., IEEE TVT 2020).
  3. Building-blocked: ray hits building/wall. Full NLOS path loss model.

Shared infrastructure used by V2V, V2I, and edge communication paths.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional

import carla

logger = logging.getLogger("OcclusionModel")

# CARLA semantic tag values for ray-cast hit classification
_TAG_VEHICLES = 10
_TAG_BUILDINGS = 1
_TAG_WALLS = 3

_VEHICLE_BODY_LOSS_DB = 15.0  # dB per blocking vehicle


@dataclass
class OcclusionResult:
    """Per-link occlusion classification from a single CARLA ray cast."""
    los: bool                    # True = clear or vehicle-only blockage
    building_blocked: bool       # True = building/wall in ray path (full NLOS)
    num_vehicles_blocking: int   # count of vehicle bodies intersected
    extra_loss_db: float         # 15 dB per blocking vehicle

    @property
    def is_clear(self) -> bool:
        return self.los and self.num_vehicles_blocking == 0

    @property
    def nlos(self) -> bool:
        return self.building_blocked


def _classify_ray_hits(hits) -> OcclusionResult:
    """Classify a list of CARLA LabelledPoint hits into occlusion result."""
    n_veh = 0
    bldg = False
    for hit in hits:
        tag = hit.label
        if tag == _TAG_VEHICLES:
            n_veh += 1
        elif tag in (_TAG_BUILDINGS, _TAG_WALLS):
            bldg = True

    return OcclusionResult(
        los=not bldg,
        building_blocked=bldg,
        num_vehicles_blocking=n_veh,
        extra_loss_db=n_veh * _VEHICLE_BODY_LOSS_DB,
    )


def compute_occlusion_matrix(
    world: carla.World,
    vehicle_locs: list,
    antenna_h: float = 1.5,
) -> List[List[Optional[OcclusionResult]]]:
    """
    Compute NxN occlusion matrix via CARLA world.cast_ray().

    Parameters
    ----------
    world : carla.World
        Active CARLA world handle.
    vehicle_locs : list
        List of N vehicle locations. Each element must have .x, .y, .z attrs.
    antenna_h : float
        Antenna height above vehicle center (meters).

    Returns
    -------
    matrix : List[List[Optional[OcclusionResult]]]
        NxN symmetric matrix. Diagonal entries are None.
    """
    N = len(vehicle_locs)
    matrix: List[List[Optional[OcclusionResult]]] = [
        [None] * N for _ in range(N)
    ]

    for i in range(N):
        for j in range(i + 1, N):
            a = carla.Location(
                x=float(vehicle_locs[i].x),
                y=float(vehicle_locs[i].y),
                z=float(vehicle_locs[i].z) + antenna_h,
            )
            b = carla.Location(
                x=float(vehicle_locs[j].x),
                y=float(vehicle_locs[j].y),
                z=float(vehicle_locs[j].z) + antenna_h,
            )
            hits = world.cast_ray(a, b)
            result = _classify_ray_hits(hits)
            matrix[i][j] = result
            matrix[j][i] = result

    return matrix


def compute_occlusion_for_pair(
    world: carla.World,
    loc_a,
    loc_b,
    antenna_h: float = 1.5,
) -> OcclusionResult:
    """Single-pair occlusion query. Useful for V2I uplink checks."""
    a = carla.Location(
        x=float(loc_a.x), y=float(loc_a.y), z=float(loc_a.z) + antenna_h
    )
    b = carla.Location(
        x=float(loc_b.x), y=float(loc_b.y), z=float(loc_b.z) + antenna_h
    )
    hits = world.cast_ray(a, b)
    return _classify_ray_hits(hits)
