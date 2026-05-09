"""
Mamba3DMOT wrapper conforming to ecav's BaseTracker interface.

Wraps the Mamba3DTracker for use in the edge pipeline.
Same input/output format as AB3DMOTWrapper.
"""
import os
import logging
from typing import Any, List, Tuple

import numpy as np
import torch
import yaml

from ecav.core.tracking.base_tracker import BaseTracker
from ecav.core.tracking.mamba3dmot.tracker import Mamba3DTracker

logger = logging.getLogger("Mamba3DMOTWrapper")


class Mamba3DMOTWrapper(BaseTracker):
    """
    Wraps Mamba3DTracker with AB3DMOT-compatible input/output format.

    Input:  dets_all = {'dets': (N, 8) [h,w,l,x,y,z,yaw,score], 'info': (N, 3)}
    Output: tracks_list = [(M, 14) [h,w,l,x,y,z,yaw,id,frame,det_idx,carla_id,vx,vy,vz]]
    """

    _DEFAULTS = {
        'min_hits': 3,
        'max_age': 6,
        'filter_thresh': 0.2,
        'new_track_thresh': 0.3,
        'match_thresh': 0.7,
        'max_time_lost': 30,
        'enable_time_thresh': 5,
        'max_window': 10,
        'manner': 'diff',
        'scale_factor': 1.0,
        'd_m': 512,
        'd_state': 16,
        'L': 3,
        'box_dim': 4,
        'avg_pool_out_dim': [1, 128],
        'pred_head_dims': [64, 4],
        'q': 10,
    }

    def __init__(self, cfg: dict):
        merged = {**self._DEFAULTS, **cfg}
        self.device = merged.get('device', 'cuda:0' if torch.cuda.is_available() else 'cpu')

        # Load config from yaml if provided
        config_path = merged.get('config_yaml')
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                yaml_cfg = yaml.safe_load(f)
            for section in ['train', 'inference']:
                if section in yaml_cfg:
                    merged.update(yaml_cfg[section])

        self.min_hits = merged.get('min_hits', 3)
        self._cfg = merged
        self._tracker = Mamba3DTracker(merged, self.device)

    def track(self, dets_all: dict, frame: int) -> Tuple[List[np.ndarray], Any]:
        """
        Run one tracking step.

        Args:
            dets_all: {'dets': (N, 8) [h,w,l,x,y,z,yaw,score], 'info': (N, 3)}
            frame: frame index

        Returns:
            (tracks_list, None) matching AB3DMOT output format
        """
        dets = dets_all['dets']
        info = dets_all['info']

        if len(dets) == 0:
            scores = np.array([])
            dets_3d = np.empty((0, 7))
        else:
            scores = dets[:, 7]  # last column is score
            # Convert AB3DMOT format [h,w,l,x,y,z,yaw] to tracker format [x,y,z,l,w,h,yaw]
            dets_3d = np.column_stack([
                dets[:, 3],  # x
                dets[:, 4],  # y (AB3DMOT uses z here but we keep consistent)
                dets[:, 5],  # z
                dets[:, 2],  # l
                dets[:, 1],  # w
                dets[:, 0],  # h
                dets[:, 6],  # yaw
            ])

        active_tracklets = self._tracker.update(dets_3d, scores)

        # Convert output to AB3DMOT format
        results = []
        for trk in active_tracklets:
            if trk.is_activated and trk.time_since_update == 0:
                state = trk.state  # [x,y,z,l,w,h,yaw]
                # Convert back to AB3DMOT output: [h,w,l,x,y,z,yaw,id,...]
                # Velocity from history
                if len(trk.memo_bank) >= 2:
                    vel = trk.memo_bank[-1][:3] - trk.memo_bank[-2][:3]
                else:
                    vel = np.zeros(3)

                out = np.array([
                    state[5],  # h
                    state[4],  # w
                    state[3],  # l
                    state[0],  # x
                    state[1],  # y
                    state[2],  # z
                    state[6],  # yaw
                    trk.track_id,
                    frame,
                    0,   # det_idx
                    -1,  # carla_id
                    vel[0],  # vx
                    vel[1],  # vy
                    vel[2],  # vz
                ], dtype=np.float64)
                results.append(out)

        if results:
            return [np.array(results)], None
        else:
            return [np.empty((0, 14))], None

    def reset(self):
        self._tracker = Mamba3DTracker(self._cfg, self.device)
