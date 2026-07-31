# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Migration-aware metrics: per-frame track state vs ground truth, per-handoff
records, CSV persistence.

The frame log is the raw material for the post-handoff gap curve: for every
tick it records the vehicle's actual CARLA pose (ground truth) and the owning
edge's tracked state for that vehicle (or absence). Post-processing computes
tracking error and linear-extrapolation displacement error by frame index
relative to the handoff tick, for warm vs cold runs.

Coordinate note: tracker state is stored in AB3DMOT/KITTI axis convention
(KITTI y = CARLA z, KITTI z = CARLA y). ``log_frame`` converts back to CARLA
order before writing, so the CSV columns are all CARLA-frame.
"""
from __future__ import annotations

import csv
import logging
import os
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

FRAME_FIELDS = [
    "tick", "mode", "handoff_tick", "edge_id", "carla_id",
    "track_present", "track_id",
    "track_x", "track_y", "track_z",
    "vel_x", "vel_y",
    "gt_x", "gt_y", "gt_z",
]

HANDOFF_FIELDS = [
    "tick", "mode", "vehicle_id", "src_edge", "dst_edge",
    "payload_bytes", "serialize_ms", "network_ms", "total_ms",
    "state_imported",
]


class MigrationMetricsLogger:
    """Collects per-frame and per-handoff rows; dumps CSVs at cleanup."""

    def __init__(self, mode: str, handoff_tick: int):
        self.mode = mode
        self.handoff_tick = handoff_tick
        self._frames = []
        self._handoffs = []

    # ------------------------------------------------------------------
    def log_frame(self, tick: int, edge_id, carla_id: int,
                  gt_xyz, tracklet=None) -> None:
        """Record one tick: ground truth plus the owning edge's track state.

        ``tracklet`` is a MambaTracklet3D (state in KITTI axis order) or None
        when the edge holds no track for the vehicle this tick.
        """
        row = {
            "tick": tick,
            "mode": self.mode,
            "handoff_tick": self.handoff_tick,
            "edge_id": edge_id,
            "carla_id": carla_id,
            "track_present": 0,
            "track_id": -1,
            "track_x": "", "track_y": "", "track_z": "",
            "vel_x": "", "vel_y": "",
            "gt_x": round(float(gt_xyz[0]), 4),
            "gt_y": round(float(gt_xyz[1]), 4),
            "gt_z": round(float(gt_xyz[2]), 4),
        }
        if tracklet is not None:
            state = np.asarray(tracklet.state, dtype=float)
            # KITTI -> CARLA: (x, z_c, y_c) stored as state[0], state[1], state[2]
            row.update({
                "track_present": 1,
                "track_id": int(getattr(tracklet, "track_id", -1)),
                "track_x": round(state[0], 4),
                "track_y": round(state[2], 4),
                "track_z": round(state[1], 4),
            })
            memo = getattr(tracklet, "memo_bank", None)
            if memo is not None and len(memo) >= 2:
                v = np.asarray(memo[-1][:3], float) - np.asarray(memo[-2][:3], float)
                row["vel_x"] = round(float(v[0]), 4)
                row["vel_y"] = round(float(v[2]), 4)
        self._frames.append(row)

    def log_handoff(self, tick: int, vehicle_id: int, src_edge, dst_edge,
                    cost, state_imported: bool) -> None:
        self._handoffs.append({
            "tick": tick,
            "mode": self.mode,
            "vehicle_id": vehicle_id,
            "src_edge": src_edge,
            "dst_edge": dst_edge,
            "payload_bytes": cost.payload_bytes,
            "serialize_ms": round(cost.sim_serialize_ms, 4),
            "network_ms": round(cost.sim_network_ms, 4),
            "total_ms": round(cost.total_ms, 4),
            "state_imported": int(state_imported),
        })

    # ------------------------------------------------------------------
    def dump(self, out_dir: str) -> Optional[str]:
        """Write frames + handoffs CSVs under ``out_dir``; returns the dir."""
        if not self._frames and not self._handoffs:
            logger.warning("MigrationMetricsLogger: nothing to dump")
            return None
        os.makedirs(out_dir, exist_ok=True)
        fpath = os.path.join(out_dir, f"migration_frames_{self.mode}.csv")
        with open(fpath, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FRAME_FIELDS)
            w.writeheader()
            w.writerows(self._frames)
        hpath = os.path.join(out_dir, f"migration_handoffs_{self.mode}.csv")
        with open(hpath, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=HANDOFF_FIELDS)
            w.writeheader()
            w.writerows(self._handoffs)
        logger.info("MigrationMetricsLogger: wrote %d frames, %d handoffs -> %s",
                    len(self._frames), len(self._handoffs), out_dir)
        return out_dir
