# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Harbor-style Hybrid V2I+V2V Cooperative Perception Manager.

Based on Harbor (Zhu et al., SenSys 2024). V2I-primary with opportunistic
V2V relay. Vehicles are classified as helpers (good V2I) or helpees (poor
V2I). Helpees relay point clouds to helpers via V2V. Helpers upload all
point clouds to the edge server via V2I backhaul. The edge merges point
clouds, runs detection (PointPillars), and returns results to vehicles.

Key path: Vehicle -> V2V relay -> Helper -> V2I uplink -> backhaul ->
          edge server -> detection -> backhaul -> V2I downlink -> Vehicle

Vehicles run local detection in parallel. If edge results arrive before
the E2E deadline (500ms), the vehicle merges them with local detection.
Otherwise, the vehicle falls back to local-only.

Channel model: C-V2X PC5 sidelink for V2V relay, Uu interface model for
V2I uplink/downlink (from latency module).
"""
from __future__ import annotations

import logging
import time
import uuid
import weakref
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np
import carla

from ecav.ecav_carla import (
    Location as _Loc, Rotation as _Rot, Transform as _Tf)
from ecav.core.application.edge.latency import create_latency_model
from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.tracking import get_tracker
from ecav.core.networking.occlusion_model import compute_occlusion_matrix
from ecav.core.networking.channel_engine_py import (
    get_channel_engine, OcclusionInfo)
from ecav.core.application.v2v.v2v_metrics import V2VMetrics

logger = logging.getLogger("HarborManager")

# Harbor parameters from the paper
_E2E_DEADLINE_MS = 500.0
_HELPER_BW_THRESHOLD_MBPS = 1.0  # V2I uplink > 1 Mbps => helper
_BW_REQ_PER_VEHICLE_MBPS = 4.8  # at 10fps with encoding
_ASSIGNMENT_INTERVAL_TICKS = 2   # update helper-helpee every 100ms (2 ticks at 20Hz)


def _box_to_transform(box: np.ndarray) -> _Tf:
    loc = _Loc(x=float(box[3]), y=float(box[4]), z=float(box[5]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)


class HarborManager:
    """
    Harbor-style hybrid V2I+V2V cooperative perception.

    The edge server (not the RSU) performs point cloud merging and detection.
    V2V is used as a relay mechanism for vehicles with poor V2I connectivity,
    not for direct cooperative perception.

    This models the full path: vehicle -> (V2V relay) -> V2I uplink ->
    backhaul -> edge server -> backhaul -> V2I downlink -> vehicle.
    """

    def __init__(
        self,
        world: carla.World,
        cfg: Dict[str, Any],
        cav_world,
        carla_client: carla.Client = None,
        *,
        world_dt: float = 0.05,
        **kwargs,
    ):
        self.edgeid = str(uuid.uuid4())[:8]
        self.world = world
        self.carla = carla_client
        self.dt = world_dt

        self.vehicle_manager_list: List[Any] = []
        self.rsu_manager_list: List[Any] = []

        self.downlink_pl = float(cfg.get("downlink_packet_loss_pct", 0))
        self.run_distributed = (
            getattr(cav_world, 'run_distributed', False)
            if cav_world else False)

        if cav_world:
            weakref.ref(cav_world)().update_edge(self)

        # --- V2I latency model (uplink + backhaul + compute + downlink) ---
        self.latency_model = create_latency_model(cfg, world_dt)

        # --- V2V channel engine (for relay links) ---
        ch_cfg = cfg.get("v2v_channel", {})
        mac_cfg = cfg.get("mac", {})
        self.channel_engine = get_channel_engine(
            carrier_ghz=ch_cfg.get("carrier_ghz", 5.9),
            bw_mhz=ch_cfg.get("bandwidth_mhz", 10.0),
            tx_power_dbm=ch_cfg.get("tx_power_dbm", 23.0),
            num_subchannels=mac_cfg.get("M", 20),
            rc_min=mac_cfg.get("RC_min", 5),
            rc_max=mac_cfg.get("RC_max", 15),
            p_keep=mac_cfg.get("p_keep", 0.0),
            sinr_thresh_db=ch_cfg.get("sinr_threshold_db", 0.0),
            antenna_h=ch_cfg.get("antenna_h", 1.5),
        )

        # --- V2I bandwidth per vehicle (simulated, from config) ---
        self._v2i_bw_mean_mbps = cfg.get("v2i_bandwidth_mbps", 12.0)
        self._v2i_bw_std_mbps = cfg.get("v2i_bandwidth_std_mbps", 8.0)

        # --- Harbor parameters ---
        harbor_cfg = cfg.get("harbor", {})
        self._e2e_deadline_ms = harbor_cfg.get(
            "e2e_deadline_ms", _E2E_DEADLINE_MS)
        self._helper_bw_thresh = harbor_cfg.get(
            "helper_bw_threshold_mbps", _HELPER_BW_THRESHOLD_MBPS)
        self._assignment_interval = harbor_cfg.get(
            "assignment_interval_ticks", _ASSIGNMENT_INTERVAL_TICKS)
        self._edge_compute_ms = harbor_cfg.get("edge_compute_ms", 50.0)

        # --- V2V metrics ---
        self.v2v_metrics = V2VMetrics()

        # --- Per-vehicle state ---
        self._v2i_bw: Dict[int, float] = {}  # current V2I bandwidth estimate
        self._is_helper: Dict[int, bool] = {}
        self._helper_assignment: Dict[int, int] = {}  # helpee -> helper

        # --- Shared tracker and predictor ---
        tracker_name = cfg.get("tracker", "ab3dmot")
        tracker_cfg = cfg.get("tracker_cfg", {})
        self._tracker_name = tracker_name
        self._tracker_cfg = tracker_cfg
        self._tracker = None
        self._tracked_trajectories: Dict[int, ObstacleTrajectory] = {}
        self._track_to_carla: Dict[int, int] = {}
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        self._tick = 0
        logger.info("Harbor manager initialized. deadline=%.0fms "
                     "v2i_bw=%.1f+/-%.1f Mbps",
                     self._e2e_deadline_ms,
                     self._v2i_bw_mean_mbps, self._v2i_bw_std_mbps)

    # ------------------------------------------------------------------
    # Lifecycle interface
    # ------------------------------------------------------------------

    def add_member(self, vm: Any) -> None:
        self.vehicle_manager_list.append(vm)

    def add_rsu(self, rsu: Any) -> None:
        self.rsu_manager_list.append(rsu)

    def set_destination(self, destination: carla.Location) -> None:
        self.destination = destination

    def start_edge(self):
        pass

    def update_information(self, frame_idx: int):
        pass

    def run_step(self, tick: int):
        self._tick = tick
        N = len(self.vehicle_manager_list)
        all_agents = list(self.vehicle_manager_list) + list(self.rsu_manager_list)
        N_total = len(all_agents)

        if N == 0:
            return

        # --- 1. Sample V2I bandwidth per vehicle (heterogeneous) ---
        for i in range(N):
            self._v2i_bw[i] = max(0.1, np.random.normal(
                self._v2i_bw_mean_mbps, self._v2i_bw_std_mbps))

        # RSUs have dedicated backhaul (high V2I bandwidth)
        for i in range(N, N_total):
            self._v2i_bw[i] = 100.0  # dedicated fiber/ethernet

        # --- 2. Helper-helpee assignment (every N ticks) ---
        if tick % self._assignment_interval == 0:
            self._update_helper_assignment(N_total)

        # --- 3. V2V channel for relay links ---
        agent_locs = [a.localizer.get_ego_pos().location for a in all_agents]
        occ_matrix = compute_occlusion_matrix(self.world, agent_locs)

        positions = [[float(l.x), float(l.y), float(l.z)]
                     for l in agent_locs]
        occ_info = []
        for i in range(N_total):
            row = []
            for j in range(N_total):
                if i == j:
                    row.append(OcclusionInfo())
                else:
                    r = occ_matrix[i][j]
                    row.append(OcclusionInfo(
                        building_blocked=r.building_blocked,
                        num_vehicles_blocking=r.num_vehicles_blocking,
                        extra_loss_db=r.extra_loss_db))
            occ_info.append(row)

        link_results = self.channel_engine.compute_tick(
            positions, occ_info, tick)
        sc_assign = self.channel_engine.get_subchannel_assignments()
        stats = self.v2v_metrics.record_tick(
            tick, link_results, N_total, sc_assign)
        delivery_map = {(lr.tx_id, lr.rx_id): lr.delivered
                        for lr in link_results}

        # --- 4. Simulate data path: collect point clouds at edge ---
        # Determine which vehicles' data reaches the edge this tick
        edge_received = set()  # agent indices whose data the edge has

        for i in range(N_total):
            if self._is_helper.get(i, True):
                # Helper: direct V2I upload if bandwidth sufficient
                upload_time_ms = (_BW_REQ_PER_VEHICLE_MBPS /
                                  max(self._v2i_bw[i], 0.1)) * 1000.0
                if upload_time_ms < self._e2e_deadline_ms * 0.5:
                    edge_received.add(i)
            else:
                # Helpee: V2V relay to assigned helper, then helper uploads
                helper_id = self._helper_assignment.get(i)
                if helper_id is not None:
                    # Check V2V delivery to helper
                    v2v_ok = delivery_map.get((i, helper_id), False)
                    if v2v_ok:
                        # Helper has capacity to relay?
                        helper_bw = self._v2i_bw.get(helper_id, 0)
                        n_relaying = sum(
                            1 for h_id, hh in self._helper_assignment.items()
                            if hh == helper_id and h_id != i)
                        capacity = int((helper_bw - _HELPER_BW_THRESHOLD_MBPS)
                                       / _BW_REQ_PER_VEHICLE_MBPS)
                        if n_relaying < max(capacity, 1):
                            edge_received.add(i)

        # --- 5. Edge-side: merge point clouds and detect ---
        # Simulate E2E latency: V2I uplink + backhaul + compute + downlink
        v2i_latency_ms = self.latency_model.sample() * 1000.0
        total_edge_ms = v2i_latency_ms + self._edge_compute_ms

        edge_on_time = total_edge_ms < self._e2e_deadline_ms

        # Use GT detection at edge (isolate channel/scheduling effect)
        if edge_on_time and len(edge_received) > 0:
            edge_det_results = self._edge_detect_gt(
                edge_received, all_agents, tick)
        else:
            edge_det_results = {"dets": np.empty((0, 8)),
                                "info": np.empty((0, 3))}

        n_edge_dets = len(edge_det_results.get("dets", []))

        print(f"[Harbor] tick={tick} helpers={sum(self._is_helper.values())} "
              f"edge_received={len(edge_received)}/{N_total} "
              f"edge_latency={total_edge_ms:.0f}ms "
              f"on_time={edge_on_time} edge_dets={n_edge_dets} "
              f"PRR={stats.prr:.3f}")

        # --- 6. Per-vehicle: merge edge results with local, track, predict ---
        # Harbor merges at the edge. All vehicles get the same edge detection.
        # Filter self-detections per vehicle.
        ego_id = (self.vehicle_manager_list[0].vehicle.id
                  if self.vehicle_manager_list else -1)

        # Track at edge level (centralized detection result)
        if self._tracker is None:
            self._tracker = get_tracker(self._tracker_name, self._tracker_cfg)

        tracks, _ = self._tracker.track(edge_det_results, tick)

        history: Deque[np.ndarray] = deque(maxlen=10)
        if tracks and len(tracks[0]) > 0:
            history.append(tracks[0])

        self._tracks_to_trajectories(history)

        predictions = self.lin_pred.generate_predicted_trajectories(
            self._tracked_trajectories)

        n_preds = len(predictions) if predictions else 0
        print(f"[Harbor] tick={tick} tracks={len(tracks[0]) if tracks and len(tracks[0]) > 0 else 0} "
              f"trajs={len(self._tracked_trajectories)} preds={n_preds}")

        # --- 7. Advance vehicles ---
        self._advance_vehicles(tick, predictions)

    # ------------------------------------------------------------------
    # Helper-helpee assignment
    # ------------------------------------------------------------------

    def _update_helper_assignment(self, N_total: int):
        """
        Classify vehicles as helpers or helpees based on V2I bandwidth.
        Assign each helpee to the nearest helper.
        """
        helpers = []
        helpees = []

        for i in range(N_total):
            bw = self._v2i_bw.get(i, 0)
            if bw >= self._helper_bw_thresh:
                self._is_helper[i] = True
                helpers.append(i)
            else:
                self._is_helper[i] = False
                helpees.append(i)

        # RSUs are always helpers
        N_veh = len(self.vehicle_manager_list)
        for i in range(N_veh, N_total):
            self._is_helper[i] = True
            if i not in helpers:
                helpers.append(i)

        # Assign each helpee to nearest helper (greedy)
        self._helper_assignment.clear()
        all_agents = list(self.vehicle_manager_list) + list(self.rsu_manager_list)
        for helpee_idx in helpees:
            best_helper = None
            best_dist = float('inf')
            helpee_loc = all_agents[helpee_idx].localizer.get_ego_pos().location
            for helper_idx in helpers:
                helper_loc = all_agents[helper_idx].localizer.get_ego_pos().location
                d = np.sqrt((helpee_loc.x - helper_loc.x)**2 +
                            (helpee_loc.y - helper_loc.y)**2)
                if d < best_dist:
                    best_dist = d
                    best_helper = helper_idx
            if best_helper is not None:
                self._helper_assignment[helpee_idx] = best_helper

    # ------------------------------------------------------------------
    # Edge detection (GT-based, isolating channel effect)
    # ------------------------------------------------------------------

    def _edge_detect_gt(
        self, received_agents: set, all_agents: list, frame_id: int
    ) -> dict:
        """
        Simulate edge-side detection by merging perception from all
        agents whose data reached the edge. Uses CARLA GT.
        """
        dets_list = []
        seen_ids = set()

        # Combined detection range: union of all agents' 70m ranges
        for agent_idx in received_agents:
            agent_loc = all_agents[agent_idx].localizer.get_ego_pos().location
            try:
                actors = self.world.get_actors().filter('vehicle.*')
                managed_ids = set()
                for vm in self.vehicle_manager_list:
                    managed_ids.add(vm.vehicle.id)

                for actor in actors:
                    if actor.id in seen_ids:
                        continue
                    if actor.id in managed_ids:
                        continue
                    loc = actor.get_location()
                    if loc.z < -10:
                        continue
                    dist = np.sqrt((loc.x - agent_loc.x)**2 +
                                   (loc.y - agent_loc.y)**2)
                    if dist > 70.0:
                        continue

                    seen_ids.add(actor.id)
                    ext = actor.bounding_box.extent
                    rot = actor.get_transform().rotation
                    yaw_rad = np.radians(rot.yaw)

                    dets_list.append([
                        ext.z * 2, ext.y * 2, ext.x * 2,
                        loc.x, loc.y, loc.z,
                        yaw_rad, 1.0
                    ])
            except Exception as e:
                logger.debug("Edge GT detection failed: %s", e)

        if not dets_list:
            return {"dets": np.empty((0, 8)), "info": np.empty((0, 3))}

        dets = np.array(dets_list)
        info = np.array([[frame_id, i, -1] for i in range(len(dets))])
        return {"dets": dets, "info": info}

    # ------------------------------------------------------------------
    # Tracking
    # ------------------------------------------------------------------

    def _tracks_to_trajectories(self, hist, horizon=10):
        updated = set()
        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                tid = int(trk[7])
                cid = int(trk[8]) if len(trk) > 8 else -1
                tf = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self._tracked_trajectories:
                    dummy = ObstacleVehicle(
                        corners=np.zeros((8, 3)), o3d_bbx=None,
                        track_id=tid, tick_id=0)
                    self._tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))

                traj = self._tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid
                self._track_to_carla[tid] = cid

                if len(trk) > 12:
                    traj.obstacle.kf_vx = float(trk[10])
                    traj.obstacle.kf_vy = float(trk[12])
                    traj.obstacle.kf_speed_mps = (
                        (trk[10]**2 + trk[12]**2)**0.5) / self.dt

        if updated:
            for tid in list(self._tracked_trajectories):
                if tid not in updated:
                    del self._tracked_trajectories[tid]

    # ------------------------------------------------------------------
    # Vehicle advancement
    # ------------------------------------------------------------------

    def _advance_vehicles(self, tick, predictions):
        for vm in self.vehicle_manager_list:
            if predictions:
                vm.agent.edge_predictions = list(predictions)
            else:
                vm.agent.edge_predictions = []
            if not self.run_distributed:
                vm.update_info(tick)
                vm.vehicle.apply_control(vm.run_step())

        for rsu in self.rsu_manager_list:
            if not self.run_distributed:
                rsu.update_info()
                rsu.run_step()

    # ------------------------------------------------------------------
    # Evaluation + metrics
    # ------------------------------------------------------------------

    def get_v2v_metrics(self):
        return self.v2v_metrics.summary_dict()

    def evaluate(self):
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))

        if self.v2v_metrics.history:
            ticks = [s.tick for s in self.v2v_metrics.history]
            prrs = [s.prr for s in self.v2v_metrics.history]
            axes[0].plot(ticks, prrs, label="V2V relay PRR")
            axes[0].set_xlabel("Tick")
            axes[0].set_ylabel("PRR")
            axes[0].set_title("Harbor V2V Relay PRR")
            axes[0].legend()

        # V2I bandwidth distribution
        if self._v2i_bw:
            bws = list(self._v2i_bw.values())
            axes[1].hist(bws, bins=20)
            axes[1].axvline(x=self._helper_bw_thresh, color='r',
                           linestyle='--', label=f'Helper thresh ({self._helper_bw_thresh} Mbps)')
            axes[1].set_xlabel("V2I Bandwidth (Mbps)")
            axes[1].set_ylabel("Count")
            axes[1].set_title("V2I Bandwidth Distribution")
            axes[1].legend()

        plt.tight_layout()
        summary = self.v2v_metrics.summary_dict()
        summary["n_helpers"] = sum(1 for v in self._is_helper.values() if v)
        summary["n_helpees"] = sum(1 for v in self._is_helper.values() if not v)
        text = "\n".join(f"{k}: {v}" for k, v in summary.items())
        return fig, text, summary
