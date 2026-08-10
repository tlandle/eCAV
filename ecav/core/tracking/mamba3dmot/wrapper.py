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
    Output: tracks_list = [(M, 14)
            [h,w,l,x,y,z,yaw,id,carla_id,det_idx,vx,vz,vy,0]]
            (AB3DMOT-consumer layout: ab3d_tracks_to_trajectories reads
            carla_id at col 8 and velocities at cols 10/12)
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

    @property
    def tracker(self) -> Mamba3DTracker:
        """Access the underlying Mamba3DTracker (for migration state transfer)."""
        return self._tracker

    # Gate for associating an updated tracklet back to the detection that
    # fed it this tick, to recover the detection's carla_id.
    _CARLA_ID_GATE_M = 2.0

    def _associate_carla_ids(self, dets_3d: np.ndarray, info: np.ndarray) -> None:
        """Stamp carla_id on tracklets updated this tick by nearest detection.

        Mamba3DTracker.update() does not see the info array, so the stable
        CARLA actor id would otherwise be lost. Migration export needs it to
        find a vehicle's tracklet by persistent id.
        """
        if len(dets_3d) == 0 or info is None or len(info) == 0:
            return
        det_xyz = dets_3d[:, :3]
        for trk in self._tracker.tracked_tracklets:
            if trk.time_since_update != 0:
                continue
            d = np.linalg.norm(det_xyz - np.asarray(trk.state[:3]), axis=1)
            j = int(np.argmin(d))
            if d[j] <= self._CARLA_ID_GATE_M:
                trk.carla_id = int(info[j, 2])

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

        # Internal call counter, NOT the caller's tick number: callers pass
        # sim ticks striding 4+ per edge cycle, which stretches every frame-
        # denominated constant and the motion-model extrapolation (observed
        # live: tracks teleporting between vehicles). Offline retrack and
        # training count one frame per update call; live must match.
        self._frame_n = getattr(self, '_frame_n', 0) + 1
        frame = self._frame_n
        active_tracklets = self._tracker.update(dets_3d, scores)
        self._associate_carla_ids(dets_3d, info)

        # Convert output to AB3DMOT format
        results = []
        coast_window = self._cfg.get('coast_window', 8)
        for trk in active_tracklets:
            if trk.is_activated and trk.time_since_update <= coast_window:
                # Coasting output: during a short detection gap use the
                # predicted (advancing) box, not the frozen last observation,
                # so a fast track stays continuous in downstream predictions.
                # Without this the oncoming vanishes from the planner's view
                # ~half the time (real WF recall ~50%) and the overtake gate
                # sees a false "clear" at the commit tick.
                if trk.time_since_update > 0 and trk.predicted_last_bbox is not None:
                    state = np.asarray(trk.predicted_last_bbox)  # [x,y,z,l,w,h,yaw]
                else:
                    state = trk.state  # [x,y,z,l,w,h,yaw]
                # Convert back to AB3DMOT output: [h,w,l,x,y,z,yaw,id,...]
                # Per-frame velocity from the previous EMITTED state: the
                # memo-bank diff spans variable coasting intervals, which
                # consumers (dividing by one tick) turned into absurd
                # speeds (hundreds of m/s).
                prev = getattr(trk, '_prev_out', None)
                cur_xyz = np.asarray(state[:3], dtype=np.float64)
                if prev is not None and frame > prev[0]:
                    raw_vel = (cur_xyz - prev[1]) / float(frame - prev[0])
                else:
                    raw_vel = np.zeros(3)
                # EMA smoothing: single-step diffs of raw detections are
                # jitter-dominated (0.2 m noise -> m/s spikes on parked
                # vehicles); AB3DMOT consumers expect KF-quality velocity.
                prev_ema = getattr(trk, '_vel_ema', None)
                vel = (0.3 * raw_vel + 0.7 * prev_ema)                     if prev_ema is not None else raw_vel
                trk._vel_ema = vel
                trk._prev_out = (frame, cur_xyz)

                # Column layout matches what ab3d_tracks_to_trajectories
                # parses for AB3DMOT rows: carla_id at 8, vx at 10, vy at 12
                # (previously carla_id sat at 10 and frame at 8, so replay
                # stamped carla_id=frame and kf_speed from (carla_id, vy)).
                out = np.array([
                    state[5],  # h
                    state[4],  # w
                    state[3],  # l
                    state[0],  # x
                    state[1],  # y
                    state[2],  # z
                    state[6],  # yaw
                    trk.track_id,
                    getattr(trk, 'carla_id', -1),
                    0,       # det_idx
                    vel[0],  # vx  (m/tick, consumer divides by dt)
                    vel[2],  # vz
                    vel[1],  # vy
                    0.0,
                ], dtype=np.float64)
                results.append(out)

        if results:
            return [np.array(results)], None
        else:
            return [np.empty((0, 14))], None

    def reset(self):
        self._tracker = Mamba3DTracker(self._cfg, self.device)
