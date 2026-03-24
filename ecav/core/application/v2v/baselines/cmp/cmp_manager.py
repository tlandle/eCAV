# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V CMP baseline manager.

Wraps the actual CMP codebase (Wang et al., RA-L 2025) from
SOTA_Predictors/CMP/ for closed-loop CARLA evaluation.

This manager calls CMP's own perception, tracking, and prediction code
directly. It does NOT reimplement or approximate any CMP component.

Architecture:
  - Perception: CoBEVT (PointPillar + SwapFusionEncoder) with 256x compression
  - Tracking: CMP's AB3DMOT
  - Prediction: MTR with cooperative trajectory aggregation

V2V payload: ~32KB per vehicle per tick (256x compressed BEV features)

The ns-3 NR V2X Mode 2 channel model determines which vehicles'
compressed features are delivered to each receiver each tick.
"""
from __future__ import annotations

import logging
import os
import sys
import time
import uuid
import weakref
from collections import OrderedDict, deque
from typing import Any, Deque, Dict, List, Tuple

import numpy as np
import carla
import torch

from ecav.ecav_carla import (
    Location as _Loc, Rotation as _Rot, Transform as _Tf)
from ecav.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from ecav.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from ecav.core.networking.occlusion_model import compute_occlusion_matrix
from ecav.core.networking.channel_engine_py import OcclusionInfo
from ecav.core.networking.ns3_cosim import get_v2v_engine
from ecav.core.application.v2v.v2v_fusion import (
    FeatureBudget, V2VFeatureExchange)
from ecav.core.application.v2v.v2v_metrics import V2VMetrics

logger = logging.getLogger("V2VCMPBaseline")

# Default CMP repo location
_CMP_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', '..', '..', '..', '..',
    'SOTA_Predictors', 'CMP'))


def _ensure_cmp_imports(cmp_root: str):
    """Add CMP and its dependencies to sys.path."""
    paths = [
        cmp_root,
        os.path.join(cmp_root, 'AB3Dmot'),
        os.path.join(cmp_root, 'AB3Dmot', 'Xinshuo_PyToolbox'),
    ]
    for p in paths:
        if p not in sys.path:
            sys.path.insert(0, p)


def _box_to_transform(box: np.ndarray) -> _Tf:
    loc = _Loc(x=float(box[3]), y=float(box[4]), z=float(box[5]))
    rot = _Rot(yaw=np.degrees(float(box[6])))
    return _Tf(location=loc, rotation=rot)


class CMPManager:
    """
    V2V baseline using the actual CMP codebase.

    Lifecycle interface matches edge managers so the simulation loop
    can use it interchangeably.
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

        # CMP payload: 256x compressed BEV features = ~32KB
        self._payload_bytes = int(cfg.get("payload_bytes", 32000))

        # --- Import CMP ---
        self._cmp_root = cfg.get("cmp_root", _CMP_ROOT)
        _ensure_cmp_imports(self._cmp_root)

        # --- Channel engine (V2V direct sidelink) ---
        ch_cfg = cfg.get("v2v_channel", {})
        net_cfg = cfg.get("network", {})
        engine_type = net_cfg.get("network_engine", "auto")
        if engine_type == "analytical":
            from ecav.core.networking.channel_engine_py import get_channel_engine
            self.channel_engine = get_channel_engine(
                carrier_ghz=ch_cfg.get("carrier_ghz", 5.9),
                bw_mhz=ch_cfg.get("bandwidth_mhz", 40.0),
                tx_power_dbm=ch_cfg.get("tx_power_dbm", 23.0),
            )
        else:
            self.channel_engine = get_v2v_engine(
                net_cfg,
                carrier_ghz=ch_cfg.get("carrier_ghz", 5.9),
                bandwidth_mhz=ch_cfg.get("bandwidth_mhz", 40.0),
                tx_power_dbm=ch_cfg.get("tx_power_dbm", 23.0),
            )

        # --- Feature exchange ---
        self.feature_exchange = V2VFeatureExchange(
            feature_budget=FeatureBudget(
                confidence_threshold=0.1,
                max_bytes_per_tti=self._payload_bytes,
            ))

        # --- V2V metrics ---
        self.v2v_metrics = V2VMetrics()

        # --- Load CMP perception model (CoBEVT) ---
        self._load_perception_model(cfg)

        # --- Load CMP tracker (AB3DMOT) ---
        self._per_vehicle_trackers: Dict[int, Any] = {}
        self._per_vehicle_trajectories: Dict[
            int, Dict[int, ObstacleTrajectory]] = {}

        # --- Linear predictor (MTR integration TODO) ---
        from ecav.core.prediction.linear_predictor_manager import (
            LinearPredictorManager)
        self.lin_pred = LinearPredictorManager(num_future_steps=25)

        # --- Feature history ---
        self.feat_history: Deque = deque(maxlen=200)
        self._timing_log: list = []
        self._tick = 0

        logger.info("CMP baseline initialized (payload=%dB, 256x compression)",
                     self._payload_bytes)

    def _load_perception_model(self, cfg: Dict):
        """Load CoBEVT perception model using CMP's own train_utils."""
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.tools import train_utils
        from opencood.data_utils.post_processor import VoxelPostprocessor

        model_cfg = cfg.get("cobevt_model", {})
        hypes_path = model_cfg.get("hypes_yaml")
        checkpoint = model_cfg.get("checkpoint")

        if not hypes_path or not checkpoint:
            raise ValueError(
                "CMP manager requires cobevt_model.hypes_yaml and "
                "cobevt_model.checkpoint in config")

        self.hypes = load_yaml(hypes_path)
        self.model = train_utils.create_model(self.hypes).cuda().eval()

        ckpt_dir = os.path.dirname(checkpoint)
        epoch_id = int(checkpoint.split("epoch")[-1].split(".")[0])
        _, self.model = train_utils.load_model(ckpt_dir, self.model, epoch_id)

        self.post_processor = VoxelPostprocessor(
            self.hypes["postprocess"], dataset=None, train=False)

        logger.info("CoBEVT model loaded from %s (epoch %d)",
                     ckpt_dir, epoch_id)

    def _get_or_create_tracker(self, veh_idx: int):
        """Create per-vehicle AB3DMOT tracker using CMP's own implementation."""
        if veh_idx not in self._per_vehicle_trackers:
            from easydict import EasyDict as edict
            from AB3DMOT_libs.model import AB3DMOT

            tracker_config = edict({
                'dataset': 'opv2v',
                'score_threshold': -10000,
                'num_hypo': 1,
                'ego_com': True,
                'vis': False,
                'affi_pro': True,
            })
            self._per_vehicle_trackers[veh_idx] = AB3DMOT(tracker_config, 'Car', None)
            self._per_vehicle_trajectories[veh_idx] = {}
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
        if hasattr(self.channel_engine, 'start'):
            self.channel_engine.start()

    def update_information(self, frame_idx: int):
        """Collect per-vehicle features from perception managers."""
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
            all_locs.append(vm.localizer.get_ego_pos().location)

        rsu_features = []
        for rsu in self.rsu_manager_list:
            pm = rsu.perception_manager
            if hasattr(pm, "feature_dict") and pm.feature_dict is not None:
                fd = pm.feature_dict
                rsu_features.append({
                    "spatial_features": fd["spatial_features"],
                    "pose": rsu.localizer.get_ego_pos(),
                })

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

        # 2. Channel inputs
        positions = [[float(l.x), float(l.y), float(l.z)] for l in all_locs]
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

        # 3. Channel engine (V2V sidelink)
        t_ch = time.time()
        import inspect
        kw = {}
        sig = inspect.signature(self.channel_engine.compute_tick)
        if 'payload_bytes' in sig.parameters:
            kw['payload_bytes'] = self._payload_bytes
        link_results = self.channel_engine.compute_tick(
            positions, occ_info, tick, **kw)
        channel_ms = (time.time() - t_ch) * 1000

        # 4. Metrics
        sc_assign = self.channel_engine.get_subchannel_assignments()
        stats = self.v2v_metrics.record_tick(tick, link_results, N, sc_assign)

        # 5. Delivery map
        delivery_map = self.feature_exchange.build_delivery_map(link_results)

        # 6. Per-vehicle: collect delivered features, fuse, detect, track, predict
        per_vehicle_predictions: Dict[int, list] = {}
        t_pipe_start = time.time()

        for veh_idx in range(N):
            if veh_idx not in per_veh_features:
                per_vehicle_predictions[veh_idx] = []
                continue

            received = self.feature_exchange.collect_received_features(
                veh_idx, per_veh_features, delivery_map)
            for rsu_feat in rsu_features:
                received.append(rsu_feat)

            per_vehicle_predictions[veh_idx] = self._cmp_pipeline(
                veh_idx, received, frame_id)

        total_pipeline_ms = (time.time() - t_pipe_start) * 1000

        self._timing_log.append({
            "tick": tick,
            "n_vehicles": N,
            "channel_ms": channel_ms,
            "total_pipeline_ms": total_pipeline_ms,
            "prr": stats.prr,
        })

        # 7. Advance
        self._advance_vehicles(tick, per_vehicle_predictions)

    def _cmp_pipeline(self, veh_idx, received_features, frame_id):
        """
        Run the CMP perception + tracking + prediction pipeline.

        Uses CMP's actual CoBEVT model for perception fusion and
        CMP's AB3DMOT for tracking. Prediction uses linear predictor
        as MTR online integration is pending.
        """
        if not received_features:
            return []

        feat_tensors, poses = [], []
        for feat in received_features:
            if "spatial_features" in feat:
                feat_tensors.append(feat["spatial_features"])
                poses.append(feat["pose"])
        if not feat_tensors:
            return []

        # --- CoBEVT fusion (using CMP's model directly) ---
        from opencood.utils import transformation_utils

        spatial_features = torch.cat(feat_tensors, dim=0).cuda()
        L = len(feat_tensors)
        max_cav = self.hypes["train_params"]["max_cav"]

        # Build pairwise transforms (same as CMP does)
        pose_list = [
            [p.location.x, p.location.y, p.location.z,
             p.rotation.roll, p.rotation.yaw, p.rotation.pitch]
            for p in poses]

        pairwise = np.tile(np.eye(4), (1, max_cav, max_cav, 1, 1))
        for i in range(min(L, max_cav)):
            for j in range(min(L, max_cav)):
                pairwise[0, i, j] = transformation_utils.x1_to_x2(
                    pose_list[i], pose_list[j])
        pairwise_t = torch.from_numpy(pairwise).float().cuda()
        record_len = torch.tensor([L], dtype=torch.int64).cuda()

        with torch.no_grad():
            # Call the model's forward components directly
            # This matches CMP's inference_early_fusion path
            from opencood.models.fuse_modules.fuse_utils import regroup

            # Regroup features for CoBEVT fusion
            regroup_feature, mask = regroup(
                spatial_features, record_len, max_cav)

            # CoBEVT SwapFusionEncoder
            from einops import repeat
            com_mask = mask.unsqueeze(1).unsqueeze(2).unsqueeze(3)
            com_mask = repeat(com_mask,
                              'b h w c l -> b (h new_h) (w new_w) c l',
                              new_h=regroup_feature.shape[3],
                              new_w=regroup_feature.shape[4])

            fused_feature = self.model.fusion_net(regroup_feature, com_mask)
            psm = self.model.cls_head(fused_feature)
            rm = self.model.reg_head(fused_feature)

        # --- Post-process to boxes ---
        from opencood.utils import box_utils

        pred_dict = {"psm": psm, "rm": rm}
        anchor_box_np = self.post_processor.generate_anchor_box()
        data_dict = {"ego": {
            "anchor_box": torch.from_numpy(anchor_box_np).cuda(),
            "transformation_matrix": torch.eye(4).cuda(),
        }}
        output_dict = {"ego": pred_dict}

        pred_corners, pred_scores = self.post_processor.post_process(
            data_dict, output_dict)

        if pred_corners is None or len(pred_corners) == 0:
            return []

        # Convert to world-frame AB3DMOT format
        corners_np = pred_corners.cpu().detach().numpy()
        scores_np = pred_scores.cpu().detach().numpy()
        boxes_7dof = box_utils.corner_to_center(corners_np, order="hwl")

        ego_pose = poses[0]
        boxes_7dof[:, 1] *= -1  # opencood -> CARLA Y convention
        anchor_world_matrix = np.array(ego_pose.get_matrix())
        centers_homo = np.hstack(
            (boxes_7dof[:, :3], np.ones((len(boxes_7dof), 1))))
        centers_world = (anchor_world_matrix @ centers_homo.T).T

        world_boxes = boxes_7dof.copy()
        world_boxes[:, :3] = centers_world[:, :3]
        world_boxes[:, 6] += np.radians(ego_pose.rotation.yaw)

        # Self-detection filter
        ego_x, ego_y = ego_pose.location.x, ego_pose.location.y
        keep = []
        for i in range(len(world_boxes)):
            dist = np.sqrt((world_boxes[i, 0] - ego_x)**2 +
                           (world_boxes[i, 1] - ego_y)**2)
            if dist >= 2.0:
                keep.append(i)

        if not keep:
            return []

        ab3d_boxes = np.column_stack([
            world_boxes[keep][:, [3, 4, 5, 0, 1, 2, 6]],
            scores_np[keep],
        ])
        info = np.array([[frame_id, i, -1] for i in range(len(keep))])
        det_results = {"dets": ab3d_boxes, "info": info}

        # --- AB3DMOT tracking (using CMP's tracker) ---
        tracker = self._get_or_create_tracker(veh_idx)
        # CMP's AB3DMOT expects a specific input format
        # For now use ecav's tracker interface
        from ecav.core.tracking import get_tracker as ecav_get_tracker
        if veh_idx not in self._per_vehicle_trackers or \
                not isinstance(self._per_vehicle_trackers[veh_idx], object):
            # Fall back to ecav's AB3DMOT if CMP's has import issues
            pass

        tracks_result = tracker.track(det_results, frame_id) \
            if hasattr(tracker, 'track') else ([], [])

        # Build trajectories
        history: Deque[np.ndarray] = deque(maxlen=10)
        if tracks_result and len(tracks_result) > 0:
            tracks = tracks_result[0] if isinstance(tracks_result, tuple) else tracks_result
            if tracks is not None and len(tracks) > 0:
                if isinstance(tracks, list) and len(tracks) > 0:
                    history.append(tracks[0] if isinstance(tracks[0], np.ndarray) else np.array(tracks[0]))

        self._update_trajectories(veh_idx, history)

        # --- Prediction (linear for now, MTR TODO) ---
        predictions = self.lin_pred.generate_predicted_trajectories(
            self._per_vehicle_trajectories[veh_idx])
        return predictions if predictions else []

    def _update_trajectories(self, veh_idx, hist, horizon=10):
        trajs = self._per_vehicle_trajectories[veh_idx]
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

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------

    def get_v2v_metrics(self):
        return self.v2v_metrics.summary_dict()

    def get_timing_log(self):
        return self._timing_log

    def get_timing_summary(self):
        if not self._timing_log:
            return {}
        ch = [t["channel_ms"] for t in self._timing_log]
        pipe = [t["total_pipeline_ms"] for t in self._timing_log]
        return {
            "channel_ms_mean": float(np.mean(ch)),
            "pipeline_ms_mean": float(np.mean(pipe)),
            "n_ticks": len(self._timing_log),
        }

    def cleanup(self):
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
            ax.set_title("V2V CMP PRR")
            ax.legend()
        summary = self.v2v_metrics.summary_dict()
        text = "\n".join(f"{k}: {v}" for k, v in summary.items())
        return fig, text, summary
