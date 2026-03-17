"""Abstract base class for edge fusion backends."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class BaseFusionBackend(ABC):
    """Interface for edge fusion backends.

    A fusion backend is responsible for:
    1. Collecting sensor data / features from vehicles and RSUs
    2. Running any fusion / detection model
    3. Outputting detections in AB3DMOT-compatible format

    The edge manager handles jitter buffering, latency stamping,
    and GT snapshot capture. The fusion backend only processes
    a single frame's worth of data.

    Output format:
        {'dets': np.ndarray (N, 8),  # [h,w,l,x,y,z,yaw,score]
         'info': np.ndarray (N, 3)}  # [frame_idx, guid, identity]

    Implementations: LateFusionBackend, IntermediateFusionBackend,
                     OracleFusionBackend.
    """

    @abstractmethod
    def detect(self, payload: Any, frame_idx: int,
               beacon_id_mgr=None) -> dict:
        """Run detection on a single frame payload.

        Args:
            payload: Frame data from the jitter buffer. Format depends
                on the backend (objects+beacons for late fusion,
                features+poses for intermediate fusion, gt_actors for oracle).
            frame_idx: Source tick for this frame.
            beacon_id_mgr: Optional BeaconIdManager for temp-ID lookup.

        Returns:
            Dict with 'dets' (N, 8) and 'info' (N, 3) in AB3DMOT format.
        """
        ...

    @abstractmethod
    def collect_and_push(self, frame_idx: int, vehicle_managers,
                         rsu_managers, jitter_buffer, latency_model,
                         mac_model=None, **kwargs):
        """Collect data from agents and push to the jitter buffer.

        Each backend collects different data:
        - Late fusion: per-object detections + beacon positions
        - Intermediate fusion: spatial features + agent poses
        - Oracle: GT actor positions from CARLA world

        Args:
            frame_idx: Current simulation tick.
            vehicle_managers: List of VehicleManager instances.
            rsu_managers: List of RSU manager instances.
            jitter_buffer: JitterBuffer to push payload into.
            latency_model: LatencyModel for arrival-time stamping.
            mac_model: Optional MAC model for uplink loss.
        """
        ...
