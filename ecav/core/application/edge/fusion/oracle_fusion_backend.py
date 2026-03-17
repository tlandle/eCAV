"""Oracle fusion backend: uses CARLA ground truth as detections."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
from typing import Any, Dict, List, Set

import numpy as np

from ecav.core.application.edge.fusion.base_fusion import BaseFusionBackend

logger = logging.getLogger(__name__)

_GUID = 0

# Vehicle types the model was trained to detect
VALID_VEHICLE_TYPES = {
    'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
    'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
    'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
    'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
    'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
    'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
    'patrol', 'mkz_2017', 'wrangler', 'carlacola',
}


class OracleFusionBackend(BaseFusionBackend):
    """Oracle detection: queries CARLA world for ground-truth actor
    positions and uses them as perfect detections.

    Used to isolate perception errors from tracking/prediction errors.

    Config keys:
        detection_range: float (default 50.0) meters from managed vehicles
        anchoring: bool (default True)
    """

    def __init__(self, cfg: dict, world=None):
        self.world = world
        self.detection_range = cfg.get('detection_range', 50.0)
        self.anchoring = cfg.get('anchoring', True)

    def collect_and_push(self, frame_idx, vehicle_managers, rsu_managers,
                         jitter_buffer, latency_model, mac_model=None,
                         **kwargs):
        gt_actors = self._query_gt_actors(vehicle_managers)

        beacons = {}
        vehicle_ids = [vm.vehicle.id for vm in vehicle_managers]
        mac_delivery = (mac_model.attempt_tick(frame_idx, vehicle_ids)
                        if mac_model else {v: True for v in vehicle_ids})

        for vm in vehicle_managers:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue
            beacons[vid] = (vm.vehicle.get_location(),
                            vm.vehicle.bounding_box.extent)

        arrival = latency_model.stamp(frame_idx)
        jitter_buffer.push(frame_idx, arrival, (gt_actors, beacons))

    def detect(self, payload, frame_idx, beacon_id_mgr=None,
               vehicle_managers=None):
        global _GUID
        gt_actors, beacons = payload
        det_rows, info_rows = [], []

        managed_ids: Set[int] = set()
        if vehicle_managers:
            managed_ids = {vm.vehicle.id for vm in vehicle_managers}

        # Beacons
        if vehicle_managers:
            for vm in vehicle_managers:
                if vm.vehicle.id not in beacons:
                    continue
                loc, ext = beacons[vm.vehicle.id]
                h, w, l = ext.z * 2, ext.y * 2, ext.x * 2
                det_rows.append([h, w, l, loc.x, loc.z, loc.y, 0.0, 1.0])
                _GUID += 1
                if beacon_id_mgr is not None:
                    identity = beacon_id_mgr.get_temp_id(
                        vm.vehicle.id, loc, frame_idx)
                else:
                    identity = vm.vehicle.id
                info_rows.append([frame_idx, _GUID, identity])

        # GT detections (non-managed actors)
        for actor in gt_actors:
            if actor['carla_id'] in managed_ids:
                continue
            h = actor['hz'] * 2
            w = actor['hy'] * 2
            l = actor['hx'] * 2
            x, y, z = actor['x'], actor['y'], actor['z']
            yaw = np.radians(actor['yaw'])
            # KITTI: x=CARLA_x, y=CARLA_z, z=CARLA_y
            det_rows.append([h, w, l, x, z, y, yaw, 1.0])
            _GUID += 1
            info_rows.append([frame_idx, _GUID, -1])

        dets = np.asarray(det_rows, np.float32) if det_rows else np.empty(
            (0, 8), np.float32)
        info = np.asarray(info_rows, np.int64) if info_rows else np.empty(
            (0, 3), np.int64)

        return {'dets': dets, 'info': info}

    def _query_gt_actors(self, vehicle_managers) -> List[Dict]:
        """Query CARLA for all vehicles within detection range."""
        managed_locs = [vm.vehicle.get_location()
                        for vm in vehicle_managers]
        gt_actors = []

        try:
            actors = self.world.get_actors()
            for actor in actors:
                if 'vehicle' not in actor.type_id.lower():
                    continue
                loc = actor.get_location()
                if loc.z < -10.0:
                    continue

                vehicle_type = actor.type_id.split('.')[-1].lower()
                is_valid = any(vt in vehicle_type
                               for vt in VALID_VEHICLE_TYPES)
                if not is_valid:
                    continue

                in_range = any(
                    np.sqrt((loc.x - ml.x)**2 + (loc.y - ml.y)**2)
                    <= self.detection_range
                    for ml in managed_locs)
                if not in_range:
                    continue

                ext = actor.bounding_box.extent
                gt_actors.append({
                    'carla_id': actor.id,
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'yaw': actor.get_transform().rotation.yaw,
                    'hx': ext.x, 'hy': ext.y, 'hz': ext.z,
                })
        except Exception as e:
            logger.warning("Oracle GT query failed: %s", e)

        return gt_actors
