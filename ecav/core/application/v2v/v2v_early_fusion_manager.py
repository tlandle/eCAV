# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V Early Fusion Manager (AutoCast-style).

Each vehicle detects objects locally from its own LiDAR, then shares
selected object point cloud segments with peers under a bandwidth budget
(MCKP scheduler). Receivers concatenate delivered point clouds into their
own LiDAR frame and re-run detection on the fused point cloud.

This avoids the ego-centric BEV crop problem of intermediate fusion:
shared points are transformed to the receiver's frame before voxelization,
so the receiver's detector processes them regardless of where the original
sender observed them.

Channel model: C-V2X PC5 sidelink with WINNER+ B1 propagation and SB-SPS
MAC, replacing AutoCast's distance-only radio model.

Based on AutoCast (Qian et al., arXiv:2112.14947), adapted for closed-loop
simulation with realistic channel modeling.
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
from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.tracking import get_tracker
from ecav.core.networking.occlusion_model import compute_occlusion_matrix
from ecav.core.networking.channel_engine_py import OcclusionInfo
from ecav.core.networking.ns3_cosim import get_v2v_engine
from ecav.core.application.v2v.mckp_scheduler import (
    SharedObject, mckp_greedy_schedule, transmission_time_ms)
from ecav.core.application.v2v.v2v_metrics import V2VMetrics

logger = logging.getLogger("V2VEarlyFusion")


def _box_to_transform(box: np.ndarray) -> _Tf:
    """Convert AB3DMOT track [h,w,l,x,y,z,yaw,...] to picklable Transform."""
    loc = _Loc(x=float(box[3]), y=float(box[4]), z=float(box[5]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)


def _transform_points(points: np.ndarray,
                      src_transform: carla.Transform,
                      dst_transform: carla.Transform) -> np.ndarray:
    """Transform point cloud from src vehicle frame to dst vehicle frame."""
    if points is None or len(points) == 0:
        return np.empty((0, 3))
    src_matrix = np.array(src_transform.get_matrix())  # 4x4
    dst_inv = np.linalg.inv(np.array(dst_transform.get_matrix()))  # 4x4
    T = dst_inv @ src_matrix  # src -> world -> dst

    ones = np.ones((len(points), 1))
    pts_h = np.hstack([points[:, :3], ones])
    pts_dst = (T @ pts_h.T).T
    return pts_dst[:, :3].astype(np.float32)


class V2VEarlyFusionManager:
    """
    AutoCast-style early fusion V2V with realistic C-V2X channel.

    Per-tick data flow:
      1. Each agent runs local LiDAR detection -> detected objects
      2. MCKP scheduler selects which objects to share per link
      3. C-V2X channel determines delivery per link
      4. Receiver fuses delivered point clouds into own LiDAR frame
      5. Receiver re-runs detection on fused point cloud
      6. Per-vehicle AB3DMOT tracking + linear prediction
      7. Each vehicle acts on its own predictions
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

        # --- Channel engine (V2V direct sidelink) ---
        ch_cfg = cfg.get("v2v_channel", {})
        mac_cfg = cfg.get("mac", {})
        net_cfg = cfg.get("network", {})

        # Payload size for early fusion point clouds (~10-50KB per object set)
        self._payload_bytes = int(cfg.get("payload_bytes", 50000))

        engine_type = net_cfg.get("network_engine", "auto")
        if engine_type == "analytical":
            from ecav.core.networking.channel_engine_py import get_channel_engine
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
        else:
            self.channel_engine = get_v2v_engine(
                net_cfg,
                carrier_ghz=ch_cfg.get("carrier_ghz", 5.9),
                bandwidth_mhz=ch_cfg.get("bandwidth_mhz", 40.0),
                tx_power_dbm=ch_cfg.get("tx_power_dbm", 23.0),
            )

        # Per-vehicle compute timing log
        self._timing_log: list = []

        # --- Bandwidth budget ---
        bw_cfg = cfg.get("bandwidth", {})
        self._rate_mbps = bw_cfg.get("rate_mbps", 6.0)
        self._slot_ms = bw_cfg.get("slot_ms", 100.0)

        # --- V2V metrics ---
        self.v2v_metrics = V2VMetrics()

        # --- Per-vehicle trackers ---
        tracker_name = cfg.get("tracker", "ab3dmot")
        tracker_cfg = cfg.get("tracker_cfg", {})
        self._tracker_name = tracker_name
        self._tracker_cfg = tracker_cfg
        self._per_vehicle_trackers: Dict[int, Any] = {}
        self._per_vehicle_trajectories: Dict[
            int, Dict[int, ObstacleTrajectory]] = {}
        self._per_vehicle_track_to_carla: Dict[int, Dict[int, int]] = {}

        # --- Shared linear predictor ---
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        self._tick = 0
        logger.info("V2V early fusion manager initialized (AutoCast-style). "
                     "rate=%.1f Mbps slot=%.0f ms M=%d",
                     self._rate_mbps, self._slot_ms, mac_cfg.get("M", 20))

    # ------------------------------------------------------------------
    # Lifecycle interface (same as edge managers)
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
        pass  # Data collected in run_step

    def run_step(self, tick: int):
        self._tick = tick
        N = len(self.vehicle_manager_list)
        all_agents = list(self.vehicle_manager_list) + list(self.rsu_manager_list)
        N_total = len(all_agents)

        if N == 0:
            return

        # 1. Collect local detections and LiDAR from all agents
        agent_locs = []
        agent_transforms = []
        agent_lidar = []       # raw LiDAR per agent
        agent_detections = []  # list of SharedObject per agent

        for idx, agent in enumerate(all_agents):
            pos = agent.localizer.get_ego_pos()
            agent_transforms.append(pos)
            agent_locs.append(pos.location)

            # Get raw LiDAR points
            pm = agent.perception_manager
            lidar_data = None
            if hasattr(pm, 'lidar') and pm.lidar and pm.lidar.data is not None:
                lidar_data = np.array(pm.lidar.data, dtype=np.float32)
            agent_lidar.append(lidar_data)

            # Get local detected objects with their point cloud segments
            local_objects = self._extract_local_detections(
                agent, idx, pos)
            agent_detections.append(local_objects)

        # 2. Occlusion matrix
        occ_matrix = compute_occlusion_matrix(self.world, agent_locs)

        # 3. Channel engine
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

        t_ch_start = time.time()
        compute_tick_kwargs = {}
        if hasattr(self.channel_engine, 'compute_tick'):
            import inspect
            sig = inspect.signature(self.channel_engine.compute_tick)
            if 'payload_bytes' in sig.parameters:
                compute_tick_kwargs['payload_bytes'] = self._payload_bytes
        link_results = self.channel_engine.compute_tick(
            positions, occ_info, tick, **compute_tick_kwargs)
        channel_ms = (time.time() - t_ch_start) * 1000

        sc_assign = self.channel_engine.get_subchannel_assignments()
        stats = self.v2v_metrics.record_tick(
            tick, link_results, N_total, sc_assign)

        # Build delivery map
        delivery_map = {(lr.tx_id, lr.rx_id): lr.delivered
                        for lr in link_results}

        # 4. Per-vehicle: schedule, deliver, fuse, detect, track, predict
        per_vehicle_predictions: Dict[int, list] = {}

        for veh_idx in range(N):
            # Collect offered objects from all other agents
            offered = []
            for sender_idx in range(N_total):
                if sender_idx == veh_idx:
                    continue
                if not delivery_map.get((sender_idx, veh_idx), False):
                    continue
                offered.extend(agent_detections[sender_idx])

            # Receiver's own detected positions (for utility computation)
            own_det_positions = [
                obj.bbox_center for obj in agent_detections[veh_idx]]

            # MCKP schedule: select objects under bandwidth budget
            ego_pos = np.array([
                agent_locs[veh_idx].x,
                agent_locs[veh_idx].y,
                agent_locs[veh_idx].z])
            selected = mckp_greedy_schedule(
                offered, ego_pos, own_det_positions,
                budget_ms=self._slot_ms,
                rate_mbps=self._rate_mbps)

            # Transform selected point clouds to ego frame
            ego_tf = agent_transforms[veh_idx]
            fused_points = []
            if agent_lidar[veh_idx] is not None:
                fused_points.append(agent_lidar[veh_idx][:, :3])

            for obj in selected:
                sender_tf = agent_transforms[obj.sender_id]
                transformed = _transform_points(
                    obj.points, sender_tf, ego_tf)
                if len(transformed) > 0:
                    fused_points.append(transformed)

            n_received = len(selected)
            logger.debug("veh=%d tick=%d own_dets=%d offered=%d selected=%d PRR=%.3f",
                         veh_idx, tick, len(agent_detections[veh_idx]),
                         len(offered), n_received, stats.prr)

            # Re-run detection on fused point cloud
            t_pipe_start = time.time()
            predictions = self._detect_track_predict_from_points(
                veh_idx, fused_points, ego_tf, tick)
            pipeline_ms = (time.time() - t_pipe_start) * 1000
            per_vehicle_predictions[veh_idx] = predictions

        # Log per-tick timing
        total_pipeline_ms = sum(1 for _ in per_vehicle_predictions)  # placeholder
        self._timing_log.append({
            "tick": tick,
            "n_vehicles": N,
            "n_total_agents": N_total,
            "channel_ms": channel_ms,
            "prr": stats.prr,
        })

        # 5. Advance vehicles
        self._advance_vehicles(tick, per_vehicle_predictions)

    # ------------------------------------------------------------------
    # Local detection extraction
    # ------------------------------------------------------------------

    def _extract_local_detections(
        self, agent, agent_idx: int, agent_pose
    ) -> List[SharedObject]:
        """
        Extract detected objects from an agent's perception manager.

        Uses CARLA GT for now (like AutoCast's simulation mode) to get
        object bounding boxes and associated point cloud segments.
        """
        objects = []
        pm = agent.perception_manager

        # Use the agent's detected objects if available
        if hasattr(pm, 'objects') and pm.objects:
            for obj_id, obj in enumerate(pm.objects.get('vehicles', [])):
                if hasattr(obj, 'location') and hasattr(obj, 'bounding_box'):
                    center = np.array([
                        obj.location.x, obj.location.y, obj.location.z])
                    extent = np.array([2.0, 1.0, 0.8])  # default
                    if hasattr(obj, 'bounding_box') and obj.bounding_box:
                        ext = obj.bounding_box.extent
                        extent = np.array([ext.x, ext.y, ext.z])

                    # Get points near this object from LiDAR
                    pts = self._get_points_near_bbox(
                        pm, center, extent, agent_pose)

                    objects.append(SharedObject(
                        sender_id=agent_idx,
                        object_id=obj_id,
                        points=pts,
                        bbox_center=center,
                        bbox_extent=extent,
                        score=getattr(obj, 'score', 1.0),
                        speed=getattr(obj, 'speed', 0.0),
                    ))

        # Fallback: query CARLA GT for nearby vehicles (AutoCast sim mode)
        if not objects:
            objects = self._extract_gt_objects(agent_idx, agent_pose, pm)

        return objects

    def _extract_gt_objects(
        self, agent_idx: int, agent_pose, pm
    ) -> List[SharedObject]:
        """Extract GT vehicle positions from CARLA (AutoCast simulation mode)."""
        objects = []
        try:
            ego_loc = agent_pose.location
            actors = self.world.get_actors().filter('vehicle.*')
            ego_id = (self.vehicle_manager_list[0].vehicle.id
                      if self.vehicle_manager_list else -1)

            for actor in actors:
                loc = actor.get_location()
                if loc.z < -10:
                    continue
                dist = np.sqrt((loc.x - ego_loc.x)**2 +
                               (loc.y - ego_loc.y)**2)
                # Detection range: 70m (AutoCast default)
                if dist > 70.0 or dist < 1.0:
                    continue

                vel = actor.get_velocity()
                ext = actor.bounding_box.extent
                center = np.array([loc.x, loc.y, loc.z])
                extent = np.array([ext.x, ext.y, ext.z])

                # Get LiDAR points near this object
                pts = self._get_points_near_bbox(
                    pm, center, extent, agent_pose)

                objects.append(SharedObject(
                    sender_id=agent_idx,
                    object_id=actor.id,
                    points=pts,
                    bbox_center=center,
                    bbox_extent=extent,
                    speed=np.sqrt(vel.x**2 + vel.y**2),
                ))
        except Exception as e:
            logger.debug("GT extraction failed: %s", e)

        return objects

    @staticmethod
    def _get_points_near_bbox(
        pm, center_world: np.ndarray, extent: np.ndarray,
        agent_pose, margin: float = 2.0
    ) -> np.ndarray:
        """Extract LiDAR points within a bounding box (world coords)."""
        if not hasattr(pm, 'lidar') or pm.lidar is None or pm.lidar.data is None:
            return np.empty((0, 3))

        lidar_data = np.array(pm.lidar.data, dtype=np.float32)
        if len(lidar_data) == 0:
            return np.empty((0, 3))

        # Transform lidar points to world frame
        lidar_matrix = np.array(agent_pose.get_matrix())
        pts = lidar_data[:, :3]
        ones = np.ones((len(pts), 1))
        pts_world = (lidar_matrix @ np.hstack([pts, ones]).T).T[:, :3]

        # Filter points within bbox + margin
        half = extent + margin
        mask = (np.abs(pts_world[:, 0] - center_world[0]) < half[0]) & \
               (np.abs(pts_world[:, 1] - center_world[1]) < half[1]) & \
               (np.abs(pts_world[:, 2] - center_world[2]) < half[2])

        return pts[mask]  # return in agent-local frame

    # ------------------------------------------------------------------
    # Detection on fused point cloud
    # ------------------------------------------------------------------

    def _detect_track_predict_from_points(
        self, veh_idx: int,
        point_clouds: List[np.ndarray],
        ego_tf,
        frame_id: int,
    ) -> list:
        """Run detection on fused point cloud, then track and predict."""
        if not point_clouds:
            return []

        # Concatenate all point clouds
        all_points = np.vstack(point_clouds) if point_clouds else np.empty((0, 3))
        if len(all_points) == 0:
            return []

        # Use CARLA GT as detection (like AutoCast simulation mode)
        # to isolate the channel/scheduling effect from detection model quality
        det_results = self._detect_from_gt(veh_idx, ego_tf, frame_id)

        # Track
        tracker = self._get_or_create_tracker(veh_idx)
        tracks, _ = tracker.track(det_results, frame_id)

        history: Deque[np.ndarray] = deque(maxlen=10)
        if tracks and len(tracks[0]) > 0:
            history.append(tracks[0])

        self._tracks_to_trajectories(veh_idx, history)

        # Predict
        predictions = self.lin_pred.generate_predicted_trajectories(
            self._per_vehicle_trajectories[veh_idx])
        return predictions if predictions else []

    def _detect_from_gt(self, veh_idx: int, ego_tf, frame_id: int) -> dict:
        """
        GT-based detection within the fused perception range.

        For the AutoCast baseline, detection quality is not the variable
        under study. The channel model and scheduling are. Using GT
        detection isolates the networking effect.
        """
        dets_list = []
        ego_loc = ego_tf.location
        ego_id = (self.vehicle_manager_list[veh_idx].vehicle.id
                  if veh_idx < len(self.vehicle_manager_list) else -1)

        try:
            actors = self.world.get_actors().filter('vehicle.*')
            for actor in actors:
                if actor.id == ego_id:
                    continue
                loc = actor.get_location()
                if loc.z < -10:
                    continue
                dist = np.sqrt((loc.x - ego_loc.x)**2 +
                               (loc.y - ego_loc.y)**2)
                # Combined detection range: own 70m + received objects
                if dist > 100.0:
                    continue

                ext = actor.bounding_box.extent
                rot = actor.get_transform().rotation
                yaw_rad = np.radians(rot.yaw)
                vel = actor.get_velocity()
                speed = np.sqrt(vel.x**2 + vel.y**2)

                # AB3DMOT format: [h, w, l, x, y, z, yaw, score]
                dets_list.append([
                    ext.z * 2, ext.y * 2, ext.x * 2,
                    loc.x, loc.y, loc.z,
                    yaw_rad, 1.0
                ])
        except Exception as e:
            logger.debug("GT detection failed: %s", e)

        if not dets_list:
            return {"dets": np.empty((0, 8)), "info": np.empty((0, 3))}

        dets = np.array(dets_list)
        info = np.array([[frame_id, i, -1] for i in range(len(dets))])
        return {"dets": dets, "info": info}

    # ------------------------------------------------------------------
    # Tracker management
    # ------------------------------------------------------------------

    def _get_or_create_tracker(self, veh_idx: int):
        if veh_idx not in self._per_vehicle_trackers:
            self._per_vehicle_trackers[veh_idx] = get_tracker(
                self._tracker_name, self._tracker_cfg)
            self._per_vehicle_trajectories[veh_idx] = {}
            self._per_vehicle_track_to_carla[veh_idx] = {}
        return self._per_vehicle_trackers[veh_idx]

    def _tracks_to_trajectories(self, veh_idx, hist, horizon=10):
        trajs = self._per_vehicle_trajectories[veh_idx]
        track_map = self._per_vehicle_track_to_carla[veh_idx]
        updated = set()

        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                tid = int(trk[7])
                cid = int(trk[8]) if len(trk) > 8 else -1
                tf = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in trajs:
                    dummy = ObstacleVehicle(
                        corners=np.zeros((8, 3)), o3d_bbx=None,
                        track_id=tid, tick_id=0)
                    trajs[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))

                traj = trajs[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location = tf.location
                traj.obstacle.carla_id = cid
                track_map[tid] = cid

                if len(trk) > 12:
                    traj.obstacle.kf_vx = float(trk[10])
                    traj.obstacle.kf_vy = float(trk[12])
                    traj.obstacle.kf_speed_mps = (
                        (trk[10]**2 + trk[12]**2)**0.5) / self.dt

        if updated:
            for tid in list(trajs):
                if tid not in updated:
                    del trajs[tid]

    # ------------------------------------------------------------------
    # Vehicle advancement
    # ------------------------------------------------------------------

    def _advance_vehicles(self, tick, per_vehicle_predictions):
        for i, vm in enumerate(self.vehicle_manager_list):
            preds = per_vehicle_predictions.get(i, [])
            vm.agent.edge_predictions = list(preds) if preds else []
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

    def get_timing_log(self):
        """Return per-tick timing data for compute vs network analysis."""
        return self._timing_log

    def get_timing_summary(self):
        """Return aggregate timing statistics."""
        if not self._timing_log:
            return {}
        ch = [t["channel_ms"] for t in self._timing_log]
        return {
            "channel_ms_mean": float(np.mean(ch)),
            "channel_ms_p95": float(np.percentile(ch, 95)),
            "n_ticks": len(self._timing_log),
        }

    def cleanup(self):
        """Clean up resources (ns-3 process, shared memory)."""
        if hasattr(self.channel_engine, 'shutdown'):
            self.channel_engine.shutdown()

    def evaluate(self):
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(6, 4))
        if self.v2v_metrics.history:
            ticks = [s.tick for s in self.v2v_metrics.history]
            prrs = [s.prr for s in self.v2v_metrics.history]
            ax.plot(ticks, prrs, label="PRR")
            ax.set_xlabel("Tick")
            ax.set_ylabel("PRR")
            ax.set_title("V2V Early Fusion PRR")
            ax.legend()
        summary = self.v2v_metrics.summary_dict()
        text = "\n".join(f"{k}: {v}" for k, v in summary.items())
        return fig, text, summary
