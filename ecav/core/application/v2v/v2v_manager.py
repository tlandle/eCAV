# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V Cooperative Perception Manager.

Peer-to-peer cooperative perception over C-V2X PC5 sidelink. No central
edge server. Each vehicle extracts BEV features locally, broadcasts over
the radio channel, receives peer features subject to channel delivery,
and fuses independently.

The channel engine models WINNER+ B1 propagation (3GPP TR 36.885), SB-SPS
MAC with per-link SINR and capture effect. At high N, subchannel collisions
degrade PRR. This is the network saturation cliff.

Implements the same lifecycle interface as edge managers so the simulation
loop can use it interchangeably (start_edge, update_information, run_step,
add_member, add_rsu, set_destination).
"""
from __future__ import annotations

import logging
import os
import time
import uuid
import weakref
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple

import numpy as np
import carla
import torch

from ecav.ecav_carla import (
    Location as _Loc, Rotation as _Rot, Transform as _Tf)
from ecav.core.application.edge.edge_metrics import EdgeMetrics
from ecav.core.application.edge.latency import create_latency_model
from ecav.core.prediction.linear_predictor_manager import LinearPredictorManager
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.tracking import get_tracker
from ecav.core.networking.occlusion_model import compute_occlusion_matrix
from ecav.core.networking.channel_engine_py import OcclusionInfo
from ecav.core.networking.ns3_cosim import get_v2v_engine
from ecav.core.application.v2v.v2v_fusion import (
    FeatureBudget, V2VFeatureExchange)
from ecav.core.application.v2v.v2v_metrics import V2VMetrics

logger = logging.getLogger("V2VCoopManager")


def _box_to_transform(box: np.ndarray) -> _Tf:
    """Convert AB3DMOT track [h,w,l,x,y,z,yaw,...] to picklable Transform."""
    loc = _Loc(x=float(box[3]), y=float(box[4]), z=float(box[5]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)


class V2VCooperativeManager:
    """
    V2V cooperative perception with realistic C-V2X channel modeling.

    Each vehicle fuses its own features with those received from peers
    over PC5 sidelink. No central server. The channel engine determines
    which links deliver each tick.
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

        # Downlink packet loss (for comparison parity with edge managers)
        self.downlink_pl = float(cfg.get("downlink_packet_loss_pct", 0))

        # Distributed mode flag
        self.run_distributed = (
            getattr(cav_world, 'run_distributed', False) if cav_world else False)

        # Register with CavWorld
        if cav_world:
            weakref.ref(cav_world)().update_edge(self)

        # --- Channel engine (V2V direct sidelink) ---
        ch_cfg = cfg.get("v2v_channel", {})
        mac_cfg = cfg.get("mac", {})
        net_cfg = cfg.get("network", {})

        # Payload size for BM2CP intermediate fusion features (~200KB)
        self._payload_bytes = int(cfg.get("payload_bytes", 200000))

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
            n_veh = len(cfg.get("vehicles", []))
            n_rsu = len(cfg.get("rsus", []))
            net_cfg["max_vehicles"] = net_cfg.get("max_vehicles", max(n_veh + n_rsu + 2, 8))
            self.channel_engine = get_v2v_engine(
                net_cfg,
                carrier_ghz=ch_cfg.get("carrier_ghz", 5.9),
                bandwidth_mhz=ch_cfg.get("bandwidth_mhz", 40.0),
                tx_power_dbm=ch_cfg.get("tx_power_dbm", 23.0),
            )

        # Per-vehicle compute timing log
        self._timing_log: list = []

        # --- Feature exchange ---
        fb_cfg = cfg.get("feature_budget", {})
        self.feature_exchange = V2VFeatureExchange(
            feature_budget=FeatureBudget(
                confidence_threshold=fb_cfg.get("confidence_threshold", 0.1),
                max_bytes_per_tti=fb_cfg.get("max_bytes_per_tti", 175_000),
            ))

        # --- V2V metrics ---
        self.v2v_metrics = V2VMetrics()

        # --- BM2CP fusion model (shared weights, per-vehicle inference) ---
        self._init_bm2cp_model(cfg)

        # --- Per-vehicle trackers ---
        tracker_name = cfg.get("tracker", "ab3dmot")
        tracker_cfg = cfg.get("tracker_cfg", {})
        self._tracker_name = tracker_name
        self._tracker_cfg = tracker_cfg
        self._per_vehicle_trackers: Dict[int, Any] = {}
        self._per_vehicle_trajectories: Dict[
            int, Dict[int, ObstacleTrajectory]] = {}
        self._per_vehicle_track_to_carla: Dict[int, Dict[int, int]] = {}

        # --- Feature history buffer ---
        self.feat_history: Deque[
            Tuple[int, Dict[int, Dict], List, List[Dict]]] = deque(maxlen=200)

        # --- Shared linear predictor ---
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        self._tick = 0
        logger.info("V2V cooperative manager initialized. M=%d",
                     mac_cfg.get("M", 20))

    # ------------------------------------------------------------------
    # BM2CP model loading
    # ------------------------------------------------------------------

    def _init_bm2cp_model(self, cfg: Dict):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.data_utils.post_processor import VoxelPostprocessor
        from opencood.tools import train_utils

        bm_cfg = cfg["bm2cp_model"]
        hypes = load_yaml(bm_cfg["hypes_yaml"])
        self.hypes = hypes

        self.model = train_utils.create_model(hypes).cuda().eval()
        ckpt_dir = os.path.dirname(bm_cfg["checkpoint"])
        epoch_id = int(bm_cfg["checkpoint"].split("epoch")[-1].split(".")[0])
        _, self.model = train_utils.load_model(ckpt_dir, self.model, epoch_id)
        self.post_processor = VoxelPostprocessor(
            self.hypes["postprocess"], dataset=None, train=False)
        logger.info("BM2CP model loaded (epoch %d)", epoch_id)

    # ------------------------------------------------------------------
    # Per-vehicle tracker
    # ------------------------------------------------------------------

    def _get_or_create_tracker(self, veh_idx: int):
        if veh_idx not in self._per_vehicle_trackers:
            self._per_vehicle_trackers[veh_idx] = get_tracker(
                self._tracker_name, self._tracker_cfg)
            self._per_vehicle_trajectories[veh_idx] = {}
            self._per_vehicle_track_to_carla[veh_idx] = {}
        return self._per_vehicle_trackers[veh_idx]

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
        per_vehicle_features: Dict[int, Dict] = {}
        all_locs = []

        for i, vm in enumerate(self.vehicle_manager_list):
            pm = vm.perception_manager
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                fd = pm.feature_dict
                per_vehicle_features[i] = {
                    "spatial_features": fd["spatial_features"],
                    "pose": vm.localizer.get_ego_pos(),
                }
                # BM2CP also produces psm, rm, thres_map
                for key in ("psm", "rm", "thres_map"):
                    if key in fd:
                        per_vehicle_features[i][key] = fd[key]
            all_locs.append(vm.localizer.get_ego_pos().location)

        rsu_features = []
        for rsu in self.rsu_manager_list:
            pm = rsu.perception_manager
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                fd = pm.feature_dict
                rsu_feat = {
                    "spatial_features": fd["spatial_features"],
                    "pose": rsu.localizer.get_ego_pos(),
                }
                for key in ("psm", "rm", "thres_map"):
                    if key in fd:
                        rsu_feat[key] = fd[key]
                rsu_features.append(rsu_feat)

        print(f"[V2V update_info] frame={frame_idx} veh_feats={len(per_vehicle_features)} "
              f"rsu_feats={len(rsu_features)} keys={list(per_vehicle_features.get(0, {}).keys())}")
        if per_vehicle_features:
            self.feat_history.appendleft(
                (frame_idx, per_vehicle_features, all_locs, rsu_features))

    def run_step(self, tick: int):
        self._tick = tick
        self.update_information(tick)

        if not self.feat_history:
            self._advance_vehicles(tick, {})
            return

        frame_id, per_veh_features, all_locs, rsu_features = (
            self.feat_history[0])
        N = len(self.vehicle_manager_list)
        if N == 0:
            return

        # 1. Occlusion matrix
        occ_matrix = compute_occlusion_matrix(self.world, all_locs)

        # 2. Channel engine inputs
        positions = [[float(l.x), float(l.y), float(l.z)] for l in all_locs]
        occ_info = self._build_occlusion_info(N, occ_matrix)

        # 3. Channel engine (V2V direct sidelink)
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

        # 4. Metrics
        sc_assign = self.channel_engine.get_subchannel_assignments()
        stats = self.v2v_metrics.record_tick(tick, link_results, N, sc_assign)
        logger.debug("tick=%d N=%d PRR=%.3f CBR=%.3f SINR=%.1fdB ch=%.1fms",
                     tick, N, stats.prr, stats.cbr,
                     stats.mean_sinr_db, channel_ms)

        # 5. Delivery map
        delivery_map = self.feature_exchange.build_delivery_map(link_results)

        # 6. Per-vehicle fusion pipeline (with per-stage timing)
        per_vehicle_predictions: Dict[int, list] = {}
        per_vehicle_timing: Dict[int, Dict[str, float]] = {}
        for veh_idx in range(N):
            if veh_idx not in per_veh_features:
                per_vehicle_predictions[veh_idx] = []
                continue

            t_fuse_start = time.time()
            received = self.feature_exchange.collect_received_features(
                veh_idx, per_veh_features, delivery_map)
            for rsu_feat in rsu_features:
                received.append(rsu_feat)
            t_collect_ms = (time.time() - t_fuse_start) * 1000

            t_pipeline_start = time.time()
            per_vehicle_predictions[veh_idx] = (
                self._fuse_detect_track_predict(veh_idx, received, frame_id))
            pipeline_ms = (time.time() - t_pipeline_start) * 1000

            per_vehicle_timing[veh_idx] = {
                "collect_ms": t_collect_ms,
                "pipeline_ms": pipeline_ms,
            }

        # Log per-tick timing summary
        total_pipeline_ms = sum(
            t.get("pipeline_ms", 0) for t in per_vehicle_timing.values())
        self._timing_log.append({
            "tick": tick,
            "n_vehicles": N,
            "channel_ms": channel_ms,
            "total_pipeline_ms": total_pipeline_ms,
            "prr": stats.prr,
            "per_vehicle": per_vehicle_timing,
        })

        # 7. Advance
        self._advance_vehicles(tick, per_vehicle_predictions)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _build_occlusion_info(N, occ_matrix):
        occ_info = []
        for i in range(N):
            row = []
            for j in range(N):
                if i == j:
                    row.append(OcclusionInfo())
                else:
                    r = occ_matrix[i][j]
                    row.append(OcclusionInfo(
                        building_blocked=r.building_blocked,
                        num_vehicles_blocking=r.num_vehicles_blocking,
                        extra_loss_db=r.extra_loss_db))
            occ_info.append(row)
        return occ_info

    def _fuse_detect_track_predict(self, veh_idx, received_features, frame_id):
        if not received_features:
            return []

        feat_tensors, poses = [], []
        psm_tensors, rm_tensors, thres_tensors = [], [], []
        for feat in received_features:
            if "spatial_features" in feat:
                feat_tensors.append(feat["spatial_features"])
                poses.append(feat["pose"])
                if "psm" in feat:
                    psm_tensors.append(feat["psm"])
                if "rm" in feat:
                    rm_tensors.append(feat["rm"])
                if "thres_map" in feat:
                    thres_tensors.append(feat["thres_map"])
        if not feat_tensors:
            print(f"[V2V fuse] veh={veh_idx} NO feat_tensors from {len(received_features)} received")
            return []

        print(f"[V2V fuse] veh={veh_idx} n_agents={len(feat_tensors)} "
              f"feat_shape={feat_tensors[0].shape} psm={len(psm_tensors)} rm={len(rm_tensors)} thres={len(thres_tensors)}")

        spatial_features = torch.cat(feat_tensors, dim=0).cuda()
        pairwise_t = self._compute_pairwise_transforms(poses)
        record_len = torch.tensor([len(feat_tensors)], dtype=torch.int64).cuda()

        psm_tensor = torch.cat(psm_tensors, dim=0).cuda() if psm_tensors else None
        rm_tensor = torch.cat(rm_tensors, dim=0).cuda() if rm_tensors else None
        thres_tensor = torch.cat(thres_tensors, dim=0).cuda() if thres_tensors else None

        with torch.no_grad():
            pred_dict = self._run_fusion(
                spatial_features, pairwise_t, record_len,
                psm_tensor, rm_tensor, thres_tensor)

        ego_pose = poses[0]
        det_results = self._to_ab3dmot_format(pred_dict, frame_id, ego_pose)
        n_before = len(det_results.get("dets", []))
        det_results = self._filter_self_detection(det_results, ego_pose)
        n_after = len(det_results.get("dets", []))
        print(f"[V2V fuse] veh={veh_idx} dets_before_selffilter={n_before} dets_after={n_after}")

        # GT association diagnostic (temporary)
        self._diagnose_detections(det_results, ego_pose, frame_id)

        tracker = self._get_or_create_tracker(veh_idx)
        tracks, _ = tracker.track(det_results, frame_id)

        history: Deque[np.ndarray] = deque(maxlen=10)
        if tracks and len(tracks[0]) > 0:
            history.append(tracks[0])

        self._tracks_to_trajectories(veh_idx, history)

        n_tracks = sum(1 for t in tracks[0]) if tracks and len(tracks[0]) > 0 else 0
        n_trajs = len(self._per_vehicle_trajectories[veh_idx])
        predictions = self.lin_pred.generate_predicted_trajectories(
            self._per_vehicle_trajectories[veh_idx])
        print(f"[V2V fuse] veh={veh_idx} tracks={n_tracks} trajs={n_trajs} predictions={len(predictions) if predictions else 0}")
        return predictions if predictions else []

    def _diagnose_detections(self, det_results, ego_pose, frame_id):
        """Temporary GT diagnostic: match detections to CARLA vehicles."""
        dets = det_results.get("dets", [])
        if len(dets) == 0:
            return

        # Query all CARLA vehicles
        gt_vehicles = []
        try:
            actors = self.world.get_actors()
            ego_id = self.vehicle_manager_list[0].vehicle.id if self.vehicle_manager_list else -1
            for actor in actors:
                if 'vehicle' not in actor.type_id.lower():
                    continue
                loc = actor.get_location()
                if loc.z < -10.0:
                    continue
                vel = actor.get_velocity()
                gt_vehicles.append({
                    'id': actor.id,
                    'type': actor.type_id.split('.')[-1],
                    'x': loc.x, 'y': loc.y, 'z': loc.z,
                    'speed': np.sqrt(vel.x**2 + vel.y**2),
                    'is_ego': actor.id == ego_id,
                })
        except Exception as e:
            print(f"[V2V DIAG] CARLA query failed: {e}")
            return

        # Print GT vehicles
        for v in gt_vehicles:
            tag = "EGO" if v['is_ego'] else v['type']
            print(f"[V2V GT] {tag} id={v['id']} pos=({v['x']:.1f},{v['y']:.1f}) speed={v['speed']:.1f}m/s")

        # Match each detection to nearest GT
        # dets format: [h,w,l,x,y,z,yaw,score] (8 columns)
        tp_scores = []
        fp_scores = []
        for i in range(len(dets)):
            det_x, det_y = dets[i, 3], dets[i, 4]
            score = dets[i, 7] if dets.shape[1] > 7 else -1

            best_dist = float('inf')
            best_gt = None
            for v in gt_vehicles:
                if v['is_ego']:
                    continue
                d = np.sqrt((det_x - v['x'])**2 + (det_y - v['y'])**2)
                if d < best_dist:
                    best_dist = d
                    best_gt = v

            if best_gt and best_dist < 5.0:
                tp_scores.append(score)
                if i < 5:
                    print(f"[V2V DIAG] det[{i}] TP pos=({det_x:.1f},{det_y:.1f}) "
                          f"score={score:.3f} -> {best_gt['type']} dist={best_dist:.1f}m")
            else:
                fp_scores.append(score)

        tp_scores = np.array(tp_scores) if tp_scores else np.array([])
        fp_scores = np.array(fp_scores) if fp_scores else np.array([])
        print(f"[V2V DIAG] frame={frame_id} total={len(dets)} TP={len(tp_scores)} FP={len(fp_scores)}")
        if len(tp_scores) > 0:
            print(f"[V2V DIAG] TP scores: min={tp_scores.min():.3f} max={tp_scores.max():.3f} mean={tp_scores.mean():.3f}")
        if len(fp_scores) > 0:
            print(f"[V2V DIAG] FP scores: min={fp_scores.min():.3f} max={fp_scores.max():.3f} mean={fp_scores.mean():.3f}")
        if len(tp_scores) > 0 and len(fp_scores) > 0:
            # Suggest threshold that separates TP from FP
            tp_min = tp_scores.min()
            fp_below_tp = fp_scores[fp_scores < tp_min]
            fp_above_tp = fp_scores[fp_scores >= tp_min]
            print(f"[V2V DIAG] FP below TP_min({tp_min:.3f}): {len(fp_below_tp)}  FP above: {len(fp_above_tp)}")

    def _run_fusion(self, spatial_features, pairwise_t_matrix, record_len,
                     psm_tensor=None, rm_tensor=None, thres_tensor=None):
        """Run BM2CP fusion_net on features from self + delivered peers."""
        fused_feature, comm_rates, result_dict = self.model.fusion_net(
            spatial_features,
            psm_tensor,
            thres_tensor,
            record_len,
            pairwise_t_matrix,
            backbone=self.model.backbone,
            heads=[self.model.shrink_conv, self.model.cls_head,
                   self.model.reg_head],
        )

        if self.model.shrink_flag:
            fused_feature = self.model.shrink_conv(fused_feature)

        pred_dict = {
            "psm": self.model.cls_head(fused_feature),
            "rm": self.model.reg_head(fused_feature),
        }
        return pred_dict

    def _compute_pairwise_transforms(self, poses):
        """Compute ego-centric pairwise transforms for BM2CP fusion."""
        from opencood.utils import transformation_utils
        L = len(poses)
        max_cav = self.hypes["train_params"]["max_cav"]

        pose_list = [
            [p.location.x, p.location.y, p.location.z,
             p.rotation.roll, p.rotation.yaw, p.rotation.pitch]
            for p in poses]

        pairwise = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))
        for i in range(L):
            for j in range(L):
                pairwise[0, i, j] = transformation_utils.x1_to_x2(
                    pose_list[i], pose_list[j])

        return torch.from_numpy(pairwise).float().cuda()

    def _to_ab3dmot_format(self, pred_dict, frame_id, anchor_pose):
        """Convert predictions from ego-centric BM2CP to world-frame AB3DMOT."""
        from opencood.utils import box_utils
        anchor_box_np = self.post_processor.generate_anchor_box()
        data_dict = {"ego": {
            "anchor_box": torch.from_numpy(anchor_box_np).cuda(),
            "transformation_matrix": torch.eye(4).cuda(),
        }}
        output_dict = {"ego": pred_dict}

        pred_corners, pred_scores = self.post_processor.post_process(
            data_dict, output_dict)

        if pred_corners is None or len(pred_corners) == 0:
            return {"dets": np.empty((0, 8)), "info": np.empty((0, 3))}

        corners_np = pred_corners.cpu().detach().numpy()
        scores_np = pred_scores.cpu().detach().numpy()
        boxes_7dof = box_utils.corner_to_center(corners_np, order="hwl")

        # BM2CP outputs in ego frame. Transform to world frame.
        boxes_7dof[:, 1] *= -1  # Y flip (opencood -> CARLA convention)
        anchor_world_matrix = np.array(anchor_pose.get_matrix())
        centers_homo = np.hstack(
            (boxes_7dof[:, :3], np.ones((len(boxes_7dof), 1))))
        centers_world = (anchor_world_matrix @ centers_homo.T).T

        world_boxes = boxes_7dof.copy()
        world_boxes[:, :3] = centers_world[:, :3]
        world_boxes[:, 6] += np.radians(anchor_pose.rotation.yaw)

        # AB3DMOT format: [h,w,l,x,y,z,ry,score] - 8 columns
        ab3d_boxes = np.column_stack([
            world_boxes[:, [3, 4, 5, 0, 1, 2, 6]],
            scores_np,
        ])
        info = np.array([[frame_id, i, -1] for i in range(len(scores_np))])
        return {"dets": ab3d_boxes, "info": info}

    @staticmethod
    def _filter_self_detection(det_results, ego_pose, threshold_m=2.0):
        if len(det_results.get("dets", [])) == 0:
            return det_results
        dets = det_results["dets"]
        ego_x, ego_y = ego_pose.location.x, ego_pose.location.y
        keep = [i for i in range(len(dets))
                if np.sqrt((dets[i, 3]-ego_x)**2 + (dets[i, 4]-ego_y)**2)
                >= threshold_m]
        if not keep:
            return {"dets": np.empty((0, 8)), "info": np.empty((0, 3))}
        idx = np.array(keep)
        out = {"dets": dets[idx], "info": det_results["info"][idx]}
        if "scores" in det_results:
            out["scores"] = det_results["scores"][idx]
        return out

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
        pipe = [t["total_pipeline_ms"] for t in self._timing_log]
        return {
            "channel_ms_mean": float(np.mean(ch)),
            "channel_ms_p95": float(np.percentile(ch, 95)),
            "pipeline_ms_mean": float(np.mean(pipe)),
            "pipeline_ms_p95": float(np.percentile(pipe, 95)),
            "n_ticks": len(self._timing_log),
        }

    def cleanup(self):
        """Clean up resources (ns-3 process, shared memory)."""
        if hasattr(self.channel_engine, 'shutdown'):
            self.channel_engine.shutdown()

    def evaluate(self):
        """Return evaluation results for EvaluationManager integration."""
        # Minimal evaluation: return V2V metrics summary
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 1, figsize=(6, 4))
        if self.v2v_metrics.history:
            ticks = [s.tick for s in self.v2v_metrics.history]
            prrs = [s.prr for s in self.v2v_metrics.history]
            ax.plot(ticks, prrs, label="PRR")
            ax.set_xlabel("Tick")
            ax.set_ylabel("PRR")
            ax.set_title("V2V Packet Reception Ratio")
            ax.legend()
        summary = self.v2v_metrics.summary_dict()
        text = "\n".join(f"{k}: {v}" for k, v in summary.items())
        return fig, text, summary
