# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Infrastructure-only edge manager.

Represents the VRF~\\cite{vrf}, VI-Eye~\\cite{vieye}, CIP~\\cite{cip}
architecture class: the RSU is the sole observation source, the edge
tracks RSU observations, and tracks are published to vehicles. No vehicle
uplink, no cross-source temporal alignment (single source), no self-beacon
anchoring (no beacons -> nothing to anchor).

This baseline shares with the late-fusion design the AoI-at-use
methodology and the edge-tracking pipeline, but cannot exhibit
self-ghosting because the ego is never republished as an identified
actor. The RSU sees the ego (or does not) as one anonymous detection
among many. The interesting failure mode here is a recall ceiling under
occlusion: the RSU cannot see what a participating vehicle's onboard
sensors could have shared.

Implemented as a thin subclass of PredictionLateFusionEdge that overrides
only ``update_information`` to drop the vehicle uplink path and force
SBA off.
"""
from __future__ import annotations

import logging
from typing import Dict, List

import numpy as np

from .edge_manager_prediction_late_fusion_ab3dmot_linear_predictor import (
    PredictionLateFusionEdge,
)

logger = logging.getLogger(__name__)


class InfraOnlyEdge(PredictionLateFusionEdge):
    """RSU-only edge manager (VRF/VI-Eye/CIP-class architecture)."""

    def __init__(self, world, cfg, cav_world, carla_client, *args, **kwargs):
        # Disable SBA defensively: with no beacons there is no anchoring.
        cfg = dict(cfg)
        if cfg.get("enable_sba", False):
            logger.warning("InfraOnlyEdge ignoring enable_sba=True; no beacons in this architecture")
        cfg["enable_sba"] = False
        super().__init__(world, cfg, cav_world, carla_client, *args, **kwargs)

    def update_information(self, frame_idx: int = 0):
        """Collect detections from RSUs only. No vehicle uplink, no beacons."""
        if frame_idx == self._last_update_tick:
            return
        self._last_update_tick = frame_idx

        objects: Dict[str, List] = {}

        # Reuse the parent class's CARLA ground-truth snapshot logic by
        # invoking just that portion. The simplest stable way is to inline
        # the snapshot capture here so we are not coupled to parent
        # refactors of the uplink section.
        DETECTION_RANGE = 50.0
        VALID_VEHICLE_TYPES = {
            'sedan', 'coupe', 'hatchback', 'wagon', 'suv', 'crossover',
            'pickup', 'van', 'minivan', 'mkz', 'model3', 'mustang',
            'charger', 'crown', 'impala', 'prius', 'civic', 'a2',
            'etron', 'tt', 'lincoln', 'dodge', 'chevrolet', 'nissan',
            'bmw', 'audi', 'mercedes', 'tesla', 'ford', 'jeep',
            'mini', 'seat', 'citroen', 'volkswagen', 'low_rider',
            'patrol', 'mkz_2017', 'wrangler', 'carlacola',
        }
        carla_snapshot: dict = {}
        excluded_vehicles_snapshot: dict = {}
        managed_locs = [vm.vehicle.get_location() for vm in self.vehicle_manager_list]

        try:
            for actor in self.world.get_actors():
                if 'vehicle' not in actor.type_id.lower():
                    continue
                loc = actor.get_location()
                if loc.z < -10.0:
                    continue
                vehicle_type = actor.type_id.split('.')[-1].lower()
                is_valid_type = any(vt in vehicle_type for vt in VALID_VEHICLE_TYPES)
                in_range = False
                for mloc in managed_locs:
                    if np.hypot(loc.x - mloc.x, loc.y - mloc.y) <= DETECTION_RANGE:
                        in_range = True
                        break
                if not in_range:
                    continue
                vehicle_data = {
                    'type': actor.type_id.split('.')[-1],
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'yaw': actor.get_transform().rotation.yaw,
                    'vx': actor.get_velocity().x, 'vy': actor.get_velocity().y,
                    'speed': float(np.hypot(actor.get_velocity().x, actor.get_velocity().y)),
                }
                if is_valid_type:
                    carla_snapshot[actor.id] = vehicle_data
                else:
                    excluded_vehicles_snapshot[actor.id] = vehicle_data
        except Exception as e:  # noqa: BLE001
            logger.warning("Could not capture CARLA snapshot: %s", e)

        self._gt_snapshots[frame_idx] = carla_snapshot
        self._excluded_snapshots[frame_idx] = excluded_vehicles_snapshot
        for old in [k for k in self._gt_snapshots if frame_idx - k > 100]:
            del self._gt_snapshots[old]
            self._excluded_snapshots.pop(old, None)

        # --- The infra-only change: RSU only, no vehicle uplink, no beacons.
        beacons: dict = {}
        for rsu in self.rsu_manager_list:
            rsu_id = getattr(rsu, 'id', 'rsu')
            for obj in rsu.objects.get('vehicles', []):
                obj._det_source = f"rsu_{rsu_id}"
            self._dict_extend(objects, rsu.objects)

        arrival = self.latency_model.stamp(frame_idx)
        self._jitter_buffer.push(frame_idx, arrival, (objects, beacons))
