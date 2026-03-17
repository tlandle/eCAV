"""Abstract base class for multi-object trackers."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Tuple

import numpy as np


class BaseTracker(ABC):
    """Interface for MOT trackers in the edge pipeline.

    Input format (AB3DMOT convention):
        dets_all = {
            'dets': np.ndarray (N, 8)  -- [h,w,l,x,y,z,yaw,score]
            'info': np.ndarray (N, 3)  -- [frame_idx, guid, carla_id]
        }

    Output format:
        tracks: List[np.ndarray] where tracks[0] is (M, 9+) with
            columns [h,w,l,x,y,z,yaw, track_id, carla_id, ...]
            KF velocity at indices 10-12 if available.

    Implementations: AB3DMOTWrapper.
    """

    @abstractmethod
    def track(self, dets_all: dict, frame: int) -> Tuple[List[np.ndarray], Any]:
        """Run one tracking step.

        Args:
            dets_all: Detection dict with 'dets' and 'info' arrays.
            frame: Frame index for temporal ordering.

        Returns:
            (tracks_list, affinity_matrix) where tracks_list[0] is
            the array of active tracks.
        """
        ...

    @abstractmethod
    def reset(self):
        """Reset all tracker state for a new episode."""
        ...
