"""AB3DMOT tracker wrapper conforming to BaseTracker interface."""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from typing import Any, List, Tuple

import numpy as np
from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT

from ecav.core.tracking.base_tracker import BaseTracker


class AB3DMOTWrapper(BaseTracker):
    """Wraps AB3DMOT_libs.model.AB3DMOT with config-dict construction.

    Accepts a flat config dict and translates to the edict format
    that AB3DMOT expects internally.
    """

    # Default AB3DMOT parameters
    _DEFAULTS = {
        'min_hits': 3,
        'max_age': 6,
        'thres': 2.0,
        'use_3d_iou': False,
        'category': 'Car',
        'anchoring': True,
        'dup_x_max': 8.0,
        'dup_y_max': 2.0,
        'dup_size_ratio': 2.5,
        'cull_consec_ticks': 3,
    }

    def __init__(self, cfg: dict):
        merged = {**self._DEFAULTS, **cfg}
        self.category = merged.pop('category')

        self._cfg = edict({
            'vis': False,
            'save_path': None,
            'use_3d_iou': merged['use_3d_iou'],
            'thres': merged['thres'],
            'output_dir': None,
            'min_hits': merged['min_hits'],
            'max_age': merged['max_age'],
            'ego_com': None,
            'affi_pro': False,
            'dataset': 'KITTI',
            'det_name': 'deprecated',
            'anchoring': merged['anchoring'],
            'dup_x_max': merged.get('dup_x_max', 8.0),
            'dup_y_max': merged.get('dup_y_max', 2.0),
            'dup_size_ratio': merged.get('dup_size_ratio', 2.5),
            'cull_consec_ticks': merged.get('cull_consec_ticks', 3),
        })

        self._tracker = AB3DMOT(self._cfg, self.category)

    @property
    def tracker(self) -> AB3DMOT:
        """Access the underlying AB3DMOT instance (for beacon reconciliation)."""
        return self._tracker

    def track(self, dets_all: dict, frame: int) -> Tuple[List[np.ndarray], Any]:
        return self._tracker.track(dets_all, frame)

    def reset(self):
        self._tracker = AB3DMOT(self._cfg, self.category)
