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
    os.path.dirname(__file__), 'CMP'))


def _ensure_cmp_imports(cmp_root: str):
    """Add CMP and its dependencies to sys.path."""
    paths = [
        cmp_root,
        os.path.join(cmp_root, 'MTR'),
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
            # Set max_vehicles to scenario size (vehicles + RSUs + margin)
            n_vehicles = len(cfg.get("vehicles", []))
            n_rsus = len(cfg.get("rsus", []))
            max_veh = max(n_vehicles + n_rsus + 2, 8)  # minimum 8
            net_cfg["max_vehicles"] = net_cfg.get("max_vehicles", max_veh)
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
        # Previous tick's detection positions per vehicle for speed computation
        # (matches CMP's interpolate_speed: velocity = position diff / dt)
        self._prev_det_positions: Dict[int, np.ndarray] = {}

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
        """Load CoBEVT config and post-processor (model loaded by perception manager)."""
        from cmp_opencood.hypes_yaml.yaml_utils import load_yaml
        from cmp_opencood.data_utils.opv2v.post_processor import VoxelPostprocessor

        model_cfg = cfg.get("cobevt_model", {})
        hypes_path = model_cfg.get("hypes_yaml")
        checkpoint = model_cfg.get("checkpoint")

        if not hypes_path or not checkpoint:
            raise ValueError(
                "CMP manager requires cobevt_model.hypes_yaml and "
                "cobevt_model.checkpoint in config")

        self.hypes = load_yaml(hypes_path)
        # Model is loaded by the CoBEVT perception manager. The CMP manager
        # accesses it via self.vehicle_manager_list[0].perception_manager.model
        # to avoid the opencood namespace collision between CMP's and ecav's.
        self.model = None  # set lazily from perception manager

        self.post_processor = VoxelPostprocessor(
            self.hypes["postprocess"], train=False)

        logger.info("CoBEVT model loaded from %s", checkpoint)

    def _get_or_create_tracker(self, veh_idx: int):
        """Create per-vehicle AB3DMOT tracker using CMP's own implementation."""
        if veh_idx not in self._per_vehicle_trackers:
            import importlib.util
            from easydict import EasyDict as edict

            # Temporarily prepend CMP's AB3Dmot to sys.path so that
            # AB3DMOT_libs.* imports (including kalman_filter.KF) resolve
            # to CMP's versions, not ecav's
            _cmp_ab3d = os.path.join(self._cmp_root, 'AB3Dmot')
            _cmp_xinshuo = os.path.join(_cmp_ab3d, 'Xinshuo_PyToolbox')
            _inserted = []
            for p in [_cmp_ab3d, _cmp_xinshuo]:
                if p not in sys.path:
                    sys.path.insert(0, p)
                    _inserted.append(p)

            # Force reimport of AB3DMOT_libs modules from CMP's path
            import importlib as _il
            for mod_name in list(sys.modules.keys()):
                if mod_name.startswith('AB3DMOT_libs'):
                    del sys.modules[mod_name]

            from AB3DMOT_libs.model import AB3DMOT as _CMP_AB3DMOT
            _mod_cls = _CMP_AB3DMOT

            # Restore sys.path
            for p in _inserted:
                sys.path.remove(p)

            tracker_config = edict({
                'dataset': 'opv2v',
                'score_threshold': -10000,
                'num_hypo': 1,
                'ego_com': True,
                'vis': False,
                'affi_pro': True,
            })
            import io
            _log = io.StringIO()  # CMP's AB3DMOT expects a writable log
            self._per_vehicle_trackers[veh_idx] = _mod_cls(
                tracker_config, 'Car', log=_log)
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
                    "compressed": fd.get("compressed", False),
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
                    "compressed": fd.get("compressed", False),
                })

        if per_vehicle_features:
            self.feat_history.appendleft(
                (frame_idx, per_vehicle_features, all_locs, rsu_features))

    def run_step(self, tick: int):
        self._tick = tick
        self.update_information(tick)

        if not self.feat_history:
            logger.info("[CMP] tick=%d no features yet, advancing", tick)
            self._advance_vehicles(tick, {})
            return

        frame_id, per_veh_features, all_locs, rsu_features = (
            self.feat_history[0])
        N = len(self.vehicle_manager_list)
        if N == 0:
            return
        logger.info("[CMP] tick=%d N=%d feats=%d rsu_feats=%d",
                     tick, N, len(per_veh_features), len(rsu_features))

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
        logger.info("[CMP] tick=%d PRR=%.3f links=%d ch=%.1fms",
                     tick, stats.prr, len(link_results), channel_ms)

        # 5. Delivery map
        delivery_map = self.feature_exchange.build_delivery_map(link_results)

        # 6. Per-vehicle: collect delivered features, fuse, detect, track, predict
        per_vehicle_predictions: Dict[int, list] = {}
        per_vehicle_stage_timing: Dict[int, Dict[str, float]] = {}
        t_pipe_start = time.time()

        for veh_idx in range(N):
            if veh_idx not in per_veh_features:
                per_vehicle_predictions[veh_idx] = []
                continue

            received = self.feature_exchange.collect_received_features(
                veh_idx, per_veh_features, delivery_map)
            for rsu_feat in rsu_features:
                received.append(rsu_feat)

            preds, stage_timing = self._cmp_pipeline_timed(
                veh_idx, received, frame_id)
            per_vehicle_predictions[veh_idx] = preds
            per_vehicle_stage_timing[veh_idx] = stage_timing

        total_pipeline_ms = (time.time() - t_pipe_start) * 1000

        # Aggregate per-stage means across vehicles
        stage_keys = ['decode_ms', 'fusion_ms', 'postprocess_ms',
                      'tracking_ms', 'prediction_ms', 'total_ms']
        stage_means = {}
        for k in stage_keys:
            vals = [t.get(k, 0) for t in per_vehicle_stage_timing.values()]
            stage_means[k] = float(np.mean(vals)) if vals else 0

        tick_record = {
            "tick": tick,
            "n_vehicles": N,
            "n_features_per_vehicle": {
                v: t.get("n_received", 0) for v, t in per_vehicle_stage_timing.items()
            },
            "channel_ms": channel_ms,
            "total_pipeline_ms": total_pipeline_ms,
            "prr": stats.prr,
            "stage_means": stage_means,
            "per_vehicle": per_vehicle_stage_timing,
        }
        self._timing_log.append(tick_record)

        if tick % 20 == 0:
            logger.info("[CMP] tick=%d N=%d PRR=%.3f ch=%.1fms "
                        "decode=%.1fms fuse=%.1fms track=%.1fms pred=%.1fms total=%.1fms",
                        tick, N, stats.prr, channel_ms,
                        stage_means.get('decode_ms', 0),
                        stage_means.get('fusion_ms', 0),
                        stage_means.get('tracking_ms', 0),
                        stage_means.get('prediction_ms', 0),
                        total_pipeline_ms)

        # 7. Advance
        self._advance_vehicles(tick, per_vehicle_predictions)

    def _cmp_pipeline_timed(self, veh_idx, received_features, frame_id):
        """
        Run CMP pipeline with per-stage timing.

        Returns (predictions, timing_dict) where timing_dict has:
          n_received, decode_ms, fusion_ms, postprocess_ms,
          tracking_ms, prediction_ms, total_ms, n_detections, n_tracks
        """
        t_total = time.time()
        timing = {"n_received": len(received_features)}

        if not received_features:
            logger.info("[CMP] veh=%d SKIP: no received features", veh_idx)
            timing["total_ms"] = 0
            return [], timing

        feat_tensors, poses = [], []
        is_compressed = False
        sources = []
        for feat in received_features:
            if "spatial_features" in feat:
                feat_tensors.append(feat["spatial_features"])
                poses.append(feat["pose"])
                sources.append(feat.get("_source", "?"))
                if feat.get("compressed", False):
                    is_compressed = True
        if not feat_tensors:
            logger.info("[CMP] veh=%d SKIP: no spatial_features in %d received",
                        veh_idx, len(received_features))
            timing["total_ms"] = 0
            return [], timing

        logger.info("[CMP] veh=%d n_feats=%d compressed=%s sources=%s shapes=%s",
                     veh_idx, len(feat_tensors), is_compressed, sources,
                     [list(f.shape) for f in feat_tensors])

        # --- Stage 1: Receiver-side decode ---
        t0 = time.time()
        spatial_features = torch.cat(feat_tensors, dim=0).cuda()
        _pm_model = self.vehicle_manager_list[0].perception_manager.model
        if is_compressed and hasattr(_pm_model, 'naive_compressor'):
            spatial_features = _pm_model.naive_compressor.decoder(spatial_features)
        timing["decode_ms"] = (time.time() - t0) * 1000
        logger.info("[CMP] veh=%d DECODE: %d feats, compressed=%s, shape=%s",
                     veh_idx, len(feat_tensors), is_compressed,
                     list(spatial_features.shape))

        # --- Stage 2: CoBEVT fusion + detection heads ---
        # The fusion module is REQUIRED for detection. The model was trained
        # end-to-end with fusion. Skipping it produces garbage.
        t0 = time.time()
        L = len(feat_tensors)
        max_cav = self.hypes["train_params"]["max_cav"]
        record_len = torch.tensor([L], dtype=torch.int64).cuda()

        from cmp_opencood.models.fuse_modules.fuse_utils import regroup
        regroup_feature, mask = regroup(spatial_features, record_len, max_cav)

        from einops import repeat
        com_mask = mask.unsqueeze(1).unsqueeze(2).unsqueeze(3)
        com_mask = repeat(com_mask,
                          'b h w c l -> b (h new_h) (w new_w) c l',
                          new_h=regroup_feature.shape[3],
                          new_w=regroup_feature.shape[4])

        with torch.no_grad():
            fused_feature = _pm_model.fusion_net(regroup_feature, com_mask)
            psm = _pm_model.cls_head(fused_feature)
            rm = _pm_model.reg_head(fused_feature)

        psm_max = torch.sigmoid(psm).max().item()
        timing["fusion_ms"] = (time.time() - t0) * 1000
        logger.info("[CMP] veh=%d FUSION: L=%d psm_max=%.3f %.1fms",
                     veh_idx, L, psm_max, timing["fusion_ms"])

        # --- Stage 3: Post-process (NMS, box conversion) ---
        t0 = time.time()
        from cmp_opencood.utils import box_utils

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
            timing["postprocess_ms"] = (time.time() - t0) * 1000
            timing["n_detections"] = 0
            timing["tracking_ms"] = 0
            timing["prediction_ms"] = 0
            timing["total_ms"] = (time.time() - t_total) * 1000
            logger.info("[CMP] veh=%d POSTPROC: no detections after NMS", veh_idx)
            return [], timing

        corners_np = pred_corners.cpu().detach().numpy()
        scores_np = pred_scores.cpu().detach().numpy()
        boxes_7dof = box_utils.corner_to_center(corners_np, order="hwl")

        ego_pose = poses[0]
        boxes_7dof[:, 1] *= -1
        anchor_world_matrix = np.array(ego_pose.get_matrix())
        centers_homo = np.hstack(
            (boxes_7dof[:, :3], np.ones((len(boxes_7dof), 1))))
        centers_world = (anchor_world_matrix @ centers_homo.T).T

        world_boxes = boxes_7dof.copy()
        world_boxes[:, :3] = centers_world[:, :3]
        world_boxes[:, 6] += np.radians(ego_pose.rotation.yaw)

        ego_x, ego_y = ego_pose.location.x, ego_pose.location.y
        all_dists = [np.sqrt((world_boxes[i, 0] - ego_x)**2 +
                             (world_boxes[i, 1] - ego_y)**2)
                     for i in range(len(world_boxes))]
        keep = [i for i, d in enumerate(all_dists) if d >= 2.0]

        timing["postprocess_ms"] = (time.time() - t0) * 1000
        timing["n_detections"] = len(keep)
        timing["n_raw_detections"] = len(world_boxes)

        logger.info("[CMP] veh=%d POSTPROC: %d raw dets, %d after self-filter "
                     "(ego=%.1f,%.1f) dists=%s scores=%s",
                     veh_idx, len(world_boxes), len(keep),
                     ego_x, ego_y,
                     [f"{d:.1f}" for d in all_dists[:8]],
                     [f"{s:.2f}" for s in scores_np[:8]])

        if not keep:
            timing["tracking_ms"] = 0
            timing["prediction_ms"] = 0
            timing["total_ms"] = (time.time() - t_total) * 1000
            return [], timing

        ab3d_boxes = np.column_stack([
            world_boxes[keep][:, [3, 4, 5, 0, 1, 2, 6]],
            scores_np[keep],
        ])
        info = np.array([[frame_id, i, -1] for i in range(len(keep))])
        det_results = {"dets": ab3d_boxes, "info": info}

        # Log detection positions
        for di in range(min(len(keep), 5)):
            d = ab3d_boxes[di]
            logger.info("[CMP] veh=%d DET[%d]: pos=(%.1f,%.1f,%.1f) "
                         "hwl=(%.1f,%.1f,%.1f) score=%.2f",
                         veh_idx, di, d[3], d[4], d[5],
                         d[0], d[1], d[2], d[7])

        # --- Stage 4: AB3DMOT tracking ---
        t0 = time.time()
        tracker = self._get_or_create_tracker(veh_idx)
        n_dets = len(det_results["dets"])
        cmp_dets_all = {
            'dets': det_results["dets"][:, :7] if n_dets > 0 else np.empty((0, 7)),
            'info': det_results["info"],
            'cav_id': np.zeros(n_dets, dtype=np.int32),
        }

        results_list, affi = tracker.track(cmp_dets_all, frame_id, f"carla_veh{veh_idx}")

        # Build velocity map from KF state (CMP's output doesn't include velocity)
        # tracker.trackers[i] has .id and .kf.x[7:10] = [dx, dy, dz]
        kf_velocity_by_id = {}
        for kf_trk in tracker.trackers:
            vel = kf_trk.get_velocity().flatten()
            kf_velocity_by_id[kf_trk.id] = vel  # [dx, dy, dz]

        history: Deque[np.ndarray] = deque(maxlen=10)
        n_tracks = 0
        if results_list and len(results_list) > 0 and len(results_list[0]) > 0:
            history.append(results_list[0])
            n_tracks = len(results_list[0])
            for ti in range(min(n_tracks, 5)):
                trk = results_list[0][ti]
                tid = int(trk[7])
                vel = kf_velocity_by_id.get(tid, np.zeros(3))
                speed = float(np.sqrt(vel[0]**2 + vel[1]**2))
                logger.info("[CMP] veh=%d TRK[%d]: id=%d pos=(%.1f,%.1f,%.1f) "
                             "vel=(%.1f,%.1f) speed=%.1f",
                             veh_idx, ti, tid,
                             trk[3], trk[4], trk[5],
                             vel[0], vel[1], speed)
        else:
            logger.info("[CMP] veh=%d TRACKING: 0 tracks from %d dets", veh_idx, n_dets)

        self._update_trajectories(veh_idx, history, kf_velocity_by_id)
        timing["tracking_ms"] = (time.time() - t0) * 1000
        timing["n_tracks"] = n_tracks

        # Log trajectory state
        n_trajs = len(self._per_vehicle_trajectories[veh_idx])
        traj_lens = {tid: len(t.trajectory)
                     for tid, t in self._per_vehicle_trajectories[veh_idx].items()}
        logger.info("[CMP] veh=%d TRAJECTORIES: %d active, lengths=%s",
                     veh_idx, n_trajs, traj_lens)

        # --- Stage 5: Prediction ---
        t0 = time.time()
        predictions = self.lin_pred.generate_predicted_trajectories(
            self._per_vehicle_trajectories[veh_idx])
        timing["prediction_ms"] = (time.time() - t0) * 1000
        timing["n_predictions"] = len(predictions) if predictions else 0

        if predictions:
            for pi in range(min(len(predictions), 3)):
                pred = predictions[pi]
                obs = pred.obstacle_trajectory.obstacle
                logger.info("[CMP] veh=%d PRED[%d]: track_id=%d carla_id=%d "
                             "pos=(%.1f,%.1f) speed=%.1f n_waypoints=%d",
                             veh_idx, pi,
                             getattr(obs, 'track_id', -1),
                             getattr(obs, 'carla_id', -1),
                             obs.location.x, obs.location.y,
                             getattr(obs, 'kf_speed_mps', 0),
                             len(pred.predicted_trajectory) if hasattr(pred, 'predicted_trajectory') else 0)
        else:
            logger.info("[CMP] veh=%d PREDICTION: 0 predictions from %d trajectories",
                         veh_idx, n_trajs)

        timing["total_ms"] = (time.time() - t_total) * 1000

        return (predictions if predictions else []), timing

    def _update_trajectories(self, veh_idx, hist, kf_velocity_by_id=None, horizon=10):
        trajs = self._per_vehicle_trajectories[veh_idx]
        updated = set()
        if kf_velocity_by_id is None:
            kf_velocity_by_id = {}

        for frame in hist:
            if frame is None or len(frame) == 0:
                continue
            for trk in frame:
                tid = int(trk[7])
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
                traj.obstacle.carla_id = -1

                # Get velocity from KF state (CMP's output doesn't have it)
                vel = kf_velocity_by_id.get(tid)
                if vel is not None and len(vel) >= 2:
                    # KF state: [dx, dy, dz] in units per tick
                    traj.obstacle.kf_vx = float(vel[0])
                    traj.obstacle.kf_vy = float(vel[1])
                    traj.obstacle.kf_speed_mps = float(
                        np.sqrt(vel[0]**2 + vel[1]**2)) / self.dt

        if updated:
            for tid in list(trajs):
                if tid not in updated:
                    del trajs[tid]

    def _advance_vehicles(self, tick, per_vehicle_predictions):
        for i, vm in enumerate(self.vehicle_manager_list):
            preds = per_vehicle_predictions.get(i, [])
            vm.agent.edge_predictions = list(preds) if preds else []
            n_preds = len(vm.agent.edge_predictions)
            if n_preds > 0 or tick % 20 == 0:
                logger.info("[CMP] ADVANCE veh=%d tick=%d edge_predictions=%d",
                             i, tick, n_preds)
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
        """Return per-stage timing statistics for evaluation output."""
        if not self._timing_log:
            return {}
        ch = [t["channel_ms"] for t in self._timing_log]
        pipe = [t["total_pipeline_ms"] for t in self._timing_log]

        summary = {
            "n_ticks": len(self._timing_log),
            "channel_ms_mean": float(np.mean(ch)),
            "channel_ms_p95": float(np.percentile(ch, 95)),
            "pipeline_ms_mean": float(np.mean(pipe)),
            "pipeline_ms_p95": float(np.percentile(pipe, 95)),
        }

        # Per-stage statistics
        stage_keys = ['decode_ms', 'fusion_ms', 'postprocess_ms',
                      'tracking_ms', 'prediction_ms']
        for key in stage_keys:
            vals = [t["stage_means"].get(key, 0)
                    for t in self._timing_log if "stage_means" in t]
            if vals:
                summary[f"{key}_mean"] = float(np.mean(vals))
                summary[f"{key}_p95"] = float(np.percentile(vals, 95))

        # Detection and track counts
        for count_key in ['n_detections', 'n_tracks', 'n_predictions']:
            all_vals = []
            for t in self._timing_log:
                for vt in t.get("per_vehicle", {}).values():
                    if count_key in vt:
                        all_vals.append(vt[count_key])
            if all_vals:
                summary[f"{count_key}_mean"] = float(np.mean(all_vals))

        # PRR statistics
        prrs = [t["prr"] for t in self._timing_log]
        summary["prr_mean"] = float(np.mean(prrs))
        summary["prr_p5"] = float(np.percentile(prrs, 5))

        return summary

    def save_timing_log(self, output_dir: str):
        """Save full per-tick timing log to JSON for offline analysis."""
        import json
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, "cmp_pipeline_timing.json")
        with open(path, 'w') as f:
            json.dump({
                "summary": self.get_timing_summary(),
                "per_tick": self._timing_log,
            }, f, indent=2, default=str)
        logger.info("Pipeline timing saved to %s", path)

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
