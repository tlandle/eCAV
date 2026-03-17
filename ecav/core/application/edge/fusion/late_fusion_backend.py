"""Late fusion backend: collects per-object detections from cameras+LiDAR."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

import logging
from typing import Any, Dict, List

import numpy as np

from ecav.core.application.edge.fusion.base_fusion import BaseFusionBackend

logger = logging.getLogger(__name__)

_GUID = 0


class LateFusionBackend(BaseFusionBackend):
    """Late fusion: each agent runs local perception (YOLO + LiDAR),
    transmits detected objects to edge. Edge deduplicates and
    formats for the tracker.

    Config keys:
        anchoring: bool (default True)
    """

    def __init__(self, cfg: dict):
        self.anchoring = cfg.get('anchoring', True)

    def collect_and_push(self, frame_idx, vehicle_managers, rsu_managers,
                         jitter_buffer, latency_model, mac_model=None,
                         **kwargs):
        objects: Dict[str, List] = {}
        beacons = {}

        vehicle_ids = [vm.vehicle.id for vm in vehicle_managers]
        mac_delivery = (mac_model.attempt_tick(frame_idx, vehicle_ids)
                        if mac_model else {v: True for v in vehicle_ids})

        for vm in vehicle_managers:
            vid = vm.vehicle.id
            if not mac_delivery.get(vid, True):
                continue
            for obj in vm.agent.objects.get('vehicles', []):
                obj._det_source = f"ego_{vid}"
            self._dict_extend(objects, vm.agent.objects)
            beacons[vid] = (vm.vehicle.get_location(),
                            vm.vehicle.bounding_box.extent)

        for rsu in rsu_managers:
            rsu_id = getattr(rsu, 'id', 'rsu')
            for obj in rsu.objects.get('vehicles', []):
                obj._det_source = f"rsu_{rsu_id}"
            self._dict_extend(objects, rsu.objects)

        arrival = latency_model.stamp(frame_idx)
        jitter_buffer.push(frame_idx, arrival, (objects, beacons))

    def detect(self, payload, frame_idx, beacon_id_mgr=None,
               vehicle_managers=None):
        global _GUID
        objects, beacons = payload
        det_rows, info_rows = [], []

        # Beacons (one per managed vehicle)
        if vehicle_managers:
            for vm in vehicle_managers:
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

        # Sensor detections
        for obj in objects.get('vehicles', []):
            bbx = obj.bounding_box.extent
            h, w, l = bbx.z * 2, bbx.y * 2, bbx.x * 2
            loc = obj.location
            det_rows.append([h, w, l, loc.x, loc.z, loc.y, 0.0, 0.5])
            _GUID += 1
            info_rows.append([frame_idx, _GUID, -1])

        dets = np.asarray(det_rows, np.float32) if det_rows else np.empty(
            (0, 8), np.float32)
        info = np.asarray(info_rows, np.int64) if info_rows else np.empty(
            (0, 3), np.int64)

        result = {'dets': dets, 'info': info}

        # Cross-source NMS
        result = self._cross_source_nms(result)

        return result

    def _cross_source_nms(self, dets_dict, cdist_thresh=3.0):
        """Deduplicate anonymous detections from multiple sources."""
        dets = dets_dict['dets']
        info = dets_dict['info']
        if len(dets) == 0:
            return dets_dict

        cids = info[:, 2] if info.ndim == 2 and info.shape[1] > 2 else np.full(
            len(info), -1)
        beacon_mask = cids > 0
        anon_mask = ~beacon_mask

        if anon_mask.sum() <= 1:
            return dets_dict

        anon_dets = dets[anon_mask]
        anon_info = info[anon_mask]
        centers = anon_dets[:, [3, 5]]  # CARLA x, CARLA y
        scores = anon_dets[:, 7] if anon_dets.shape[1] > 7 else np.ones(
            len(anon_dets))
        order = np.argsort(-scores)
        keep = []
        suppressed = np.zeros(len(anon_dets), dtype=bool)

        for i in order:
            if suppressed[i]:
                continue
            keep.append(i)
            dists = np.linalg.norm(centers[i] - centers, axis=1)
            suppressed |= dists < cdist_thresh

        anon_keep = np.array(keep)
        kept_dets = np.concatenate(
            [dets[beacon_mask], anon_dets[anon_keep]], axis=0)
        kept_info = np.concatenate(
            [info[beacon_mask], anon_info[anon_keep]], axis=0)

        return {'dets': kept_dets, 'info': kept_info}

    @staticmethod
    def _dict_extend(dest, src):
        for k, v in src.items():
            if k in dest:
                dest[k].extend(v)
            else:
                dest[k] = list(v)
