# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Late fusion with the ETSI TR 103 562 object self-announcement rule at the
senders: the state-of-the-art baseline against self-beacon anchoring.

ETSI TR 103 562 V2.1.1 section 4.5.6 (object self-announcement redundancy
mitigation): a station that can match a locally perceived object to a
received V2X message (a CAM; here the self-beacon) may omit that object from
the CPM it generates, on the assumption that every receiver also holds that
message. The rule lives at the SENDER and is unspecified beyond "match". The
report lists two drawbacks, and they are what the fault sweep measures:

  * a receiver that missed the message loses the object entirely;
  * a wrong match drops a real object from the message.

This arm: each sender (every CAV, every RSU) sees the beacons broadcast this
tick, minus its own, matches its detections to them with a speed-scaled
displacement gate (the gate BeaconIdManager uses for ID reconciliation,
floored at ``etsi_match_min_gate_m``), and omits the matched detections from
its uplink. The edge then runs the plain late-fusion AB3DMOT merge with
anchoring forced off; beacons still reach the edge as rows, exactly as in
the no-contract late-fusion arm. Self-beacon anchoring code is untouched.

Config keys (edge block, all optional):
  etsi_match_min_gate_m      floor of the match gate, m              (5.0)
  etsi_match_tolerance       gate = tolerance * speed * beacon age   (1.5)
  etsi_beacon_delay_ticks    broadcast delay as seen by senders      (0)
  etsi_beacon_max_age_ticks  oldest beacon a sender still uses       (2)
  etsi_gt_match_radius_m     GT radius for judging an omission       (2.5)
"""
from __future__ import annotations

import logging
import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np

from .edge_manager_prediction_late_fusion_ab3dmot_linear_predictor import (
    PredictionLateFusionEdge,
)

logger = logging.getLogger(__name__)

_MAX_TICK_ROWS = 50_000


class LateFusionEtsiSelfAnnouncementEdge(PredictionLateFusionEdge):
    """Late fusion, no anchoring, ETSI self-announcement omission at senders."""

    def __init__(self, world, cfg, cav_world, carla_client, *args, **kwargs):
        if cfg.get("anchoring", False):
            logger.warning(
                "LateFusionEtsiSelfAnnouncementEdge ignoring anchoring=True; "
                "this arm is the sender-side ETSI rule with a plain merge")
        cfg = dict(cfg)
        cfg["anchoring"] = False
        super().__init__(world, cfg, cav_world, carla_client, *args, **kwargs)

        self.etsi_match_min_gate_m = float(cfg.get("etsi_match_min_gate_m", 5.0))
        self.etsi_match_tolerance = float(cfg.get("etsi_match_tolerance", 1.5))
        self.etsi_beacon_delay_ticks = int(cfg.get("etsi_beacon_delay_ticks", 0))
        self.etsi_beacon_max_age_ticks = int(cfg.get("etsi_beacon_max_age_ticks", 2))
        self.etsi_gt_match_radius_m = float(cfg.get("etsi_gt_match_radius_m", 2.5))

        # Beacon broadcasts as senders see them: tick -> {vid: (x, y, vx, vy)}.
        hist_len = self.etsi_beacon_delay_ticks + self.etsi_beacon_max_age_ticks + 2
        self._etsi_beacon_hist: Deque[Tuple[int, Dict[int, Tuple[float, float, float, float]]]] = \
            deque(maxlen=hist_len)
        self._etsi_hist_tick = -1

        # Counters. Per sender: omitted, omitted_true_self, omitted_wrong,
        # missed_self_no_beacon, missed_self_gate, kept. Plus per-tick rows.
        self._etsi_per_sender: Dict[str, Dict[str, int]] = {}
        self._etsi_tick_rows: List[Dict[str, int]] = []

    # ------------------------------------------------------------------
    @staticmethod
    def _new_counter() -> Dict[str, int]:
        return {"omitted": 0, "omitted_true_self": 0, "omitted_wrong": 0,
                "missed_self_no_beacon": 0, "missed_self_gate": 0, "kept": 0}

    def _record_broadcast(self, beacons, frame_idx: int) -> None:
        """Store this tick's beacons with velocity, once per tick."""
        if frame_idx == self._etsi_hist_tick:
            return
        self._etsi_hist_tick = frame_idx
        vm_by_id = {vm.vehicle.id: vm for vm in self.vehicle_manager_list}
        row: Dict[int, Tuple[float, float, float, float]] = {}
        for vid, (loc, _ext) in beacons.items():
            vm = vm_by_id.get(vid)
            if vm is None:
                continue
            vel = vm.vehicle.get_velocity()
            row[vid] = (float(loc.x), float(loc.y), float(vel.x), float(vel.y))
        self._etsi_beacon_hist.append((frame_idx, row))

    def _visible_beacons(self, sender_vid: Optional[int], frame_idx: int):
        """Beacons a sender can use now: newest per vehicle inside the age
        window [delay, max_age], never its own. Returns
        {vid: (pred_x, pred_y, speed, age_s)} with the position propagated
        to frame_idx under constant velocity."""
        lo = frame_idx - self.etsi_beacon_delay_ticks - self.etsi_beacon_max_age_ticks
        hi = frame_idx - self.etsi_beacon_delay_ticks
        newest: Dict[int, Tuple[int, Tuple[float, float, float, float]]] = {}
        for tick, row in self._etsi_beacon_hist:
            if tick < lo or tick > hi:
                continue
            for vid, state in row.items():
                if vid == sender_vid:
                    continue
                if vid not in newest or tick > newest[vid][0]:
                    newest[vid] = (tick, state)
        out = {}
        for vid, (tick, (x, y, vx, vy)) in newest.items():
            age_s = (frame_idx - tick) * self.dt
            out[vid] = (x + vx * age_s, y + vy * age_s, math.hypot(vx, vy), age_s)
        return out

    def _nearest_gt(self, x: float, y: float, frame_idx: int) -> Optional[int]:
        """Nearest CARLA actor id within the GT match radius, from the
        snapshot update_information took this tick."""
        snap = self._gt_snapshots.get(frame_idx) or {}
        best_id, best_d = None, self.etsi_gt_match_radius_m
        for aid, v in snap.items():
            d = math.hypot(v["x"] - x, v["y"] - y)
            if d <= best_d:
                best_id, best_d = aid, d
        return best_id

    # ------------------------------------------------------------------
    def _sender_payload(self, sender_tag, sender_vid, src_objects, beacons,
                        frame_idx):
        self._record_broadcast(beacons, frame_idx)
        dets = list(src_objects.get("vehicles", []) or [])
        if not dets:
            return src_objects

        visible = self._visible_beacons(sender_vid, frame_idx)
        managed = {vm.vehicle.id for vm in self.vehicle_manager_list}
        counter = self._etsi_per_sender.setdefault(sender_tag, self._new_counter())
        tick_row = self._new_counter()

        # Greedy one-to-one match by displacement, nearest pairs first.
        pairs = []
        for di, det in enumerate(dets):
            dx, dy = float(det.location.x), float(det.location.y)
            for vid, (px, py, speed, age_s) in visible.items():
                gate = max(self.etsi_match_min_gate_m,
                           self.etsi_match_tolerance * speed * age_s)
                disp = math.hypot(dx - px, dy - py)
                if disp <= gate:
                    pairs.append((disp, di, vid))
        pairs.sort()
        matched_det: Dict[int, int] = {}
        used_beacon = set()
        for _disp, di, vid in pairs:
            if di in matched_det or vid in used_beacon:
                continue
            matched_det[di] = vid
            used_beacon.add(vid)

        kept = []
        for di, det in enumerate(dets):
            nearest = self._nearest_gt(float(det.location.x),
                                       float(det.location.y), frame_idx)
            if di in matched_det:
                vid = matched_det[di]
                key = "omitted_true_self" if nearest == vid else "omitted_wrong"
                counter["omitted"] += 1; tick_row["omitted"] += 1
                counter[key] += 1; tick_row[key] += 1
                continue
            kept.append(det)
            counter["kept"] += 1; tick_row["kept"] += 1
            if nearest in managed and nearest != sender_vid:
                # A self-announced object went out anyway: the sender had no
                # usable beacon for it, or the gate rejected the match.
                key = ("missed_self_gate" if nearest in visible
                       else "missed_self_no_beacon")
                counter[key] += 1; tick_row[key] += 1

        if tick_row["omitted"] or tick_row["missed_self_no_beacon"] or tick_row["missed_self_gate"]:
            logger.info("[ETSI] tick=%d sender=%s omitted=%d true_self=%d wrong=%d "
                        "missed_self(no_beacon=%d gate=%d) kept=%d visible_beacons=%d",
                        frame_idx, sender_tag, tick_row["omitted"],
                        tick_row["omitted_true_self"], tick_row["omitted_wrong"],
                        tick_row["missed_self_no_beacon"], tick_row["missed_self_gate"],
                        tick_row["kept"], len(visible))
        if len(self._etsi_tick_rows) < _MAX_TICK_ROWS:
            tick_row["tick"] = int(frame_idx)
            tick_row["sender"] = sender_tag
            self._etsi_tick_rows.append(tick_row)

        if len(kept) == len(dets):
            return src_objects
        out = dict(src_objects)
        out["vehicles"] = kept
        return out

    # ------------------------------------------------------------------
    def get_etsi_metrics(self) -> Dict[str, Any]:
        total = self._new_counter()
        for c in self._etsi_per_sender.values():
            for k in total:
                total[k] += c[k]
        omitted = total["omitted"]
        missed = total["missed_self_no_beacon"] + total["missed_self_gate"]
        self_announced = total["omitted_true_self"] + missed
        return {
            "config": {
                "match_min_gate_m": self.etsi_match_min_gate_m,
                "match_tolerance": self.etsi_match_tolerance,
                "beacon_delay_ticks": self.etsi_beacon_delay_ticks,
                "beacon_max_age_ticks": self.etsi_beacon_max_age_ticks,
                "gt_match_radius_m": self.etsi_gt_match_radius_m,
            },
            "totals": total,
            "omitted_wrong_fraction": (total["omitted_wrong"] / omitted) if omitted else 0.0,
            "self_announced_forwarded_fraction": (missed / self_announced) if self_announced else 0.0,
            "per_sender": self._etsi_per_sender,
            "ticks_logged": len(self._etsi_tick_rows),
        }

    def evaluate(self):
        fig, txt, metrics = super().evaluate()
        if not self.is_proxy:
            metrics["etsi_self_announcement"] = self.get_etsi_metrics()
        return fig, txt, metrics
