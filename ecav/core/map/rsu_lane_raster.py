# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""RSU-zone lane raster for the MTR Swin lane encoder.

Renders drivable-lane ribbons white-on-black around a static RSU pose,
matching the convention of the Multi-V2X training rasters
(models/lane_maps/*.png: 256x256 RGB, white lane polygons on black).
Reuses MapManager's HD-map lane geometry so any CARLA town / RSU pose
gets a raster live, without shipping per-zone PNGs.
"""

import logging

import carla
import cv2
import numpy as np

from ecav.core.map.map_manager import MapManager
from ecav.core.map.map_drawing import CV2_SUB_VALUES

logger = logging.getLogger(__name__)


class _StaticAgentShim:
    """Minimal stand-in for the vehicle argument of MapManager: the RSU is
    not an actor, so provide just the two members the constructor touches."""

    def __init__(self, world):
        self._world = world
        self.id = 'rsu_lane_raster'

    def get_world(self):
        return self._world


def generate_rsu_lane_raster(world, carla_map, rsu_location,
                             out_px: int = 256,
                             range_m: float = 90.0) -> np.ndarray:
    """Render a white-on-black lane raster centered on the RSU.

    Args:
        world: carla.World (for MapManager's traffic-light scan).
        carla_map: carla.Map of the running town.
        rsu_location: carla.Location (or object with x/y/z) of the RSU.
        out_px: output image side length in pixels.
        range_m: half-extent in meters covered by the raster
                 (out_px pixels span 2 * range_m meters).

    Returns:
        (out_px, out_px, 3) uint8 image, white lanes on black. World-axis
        aligned (yaw 0), like the offline zone rasters.
    """
    cfg = {
        'activate': False,
        'visualize': False,
        'pixels_per_meter': out_px / (2.0 * range_m),
        'raster_size': [out_px, out_px],
        'lane_sample_resolution': 2,
    }
    mm = MapManager(_StaticAgentShim(world), carla_map, cfg)
    center = carla.Transform(
        carla.Location(x=float(rsu_location.x), y=float(rsu_location.y),
                       z=float(getattr(rsu_location, 'z', 0.0))),
        carla.Rotation())
    mm.update_information(center)

    img = np.zeros((out_px, out_px, 3), dtype=np.uint8)
    lane_indices = mm.indices_in_bounds(
        mm.bound_info['lanes']['bounds'], mm.raster_radius)
    for lane_idx in lane_indices:
        lane_id = mm.bound_info['lanes']['ids'][lane_idx]
        lane_info = mm.lane_info[lane_id]
        lane_area = mm.generate_lane_area(
            lane_info['xyz_left'], lane_info['xyz_right'])
        cv2.fillPoly(img, [lane_area.reshape(-1, 2)], (255, 255, 255),
                     **CV2_SUB_VALUES)

    logger.info("RSU lane raster: %d lanes in %.0fm range at (%.1f, %.1f)",
                len(lane_indices), range_m,
                center.location.x, center.location.y)
    return img
