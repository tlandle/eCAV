# -*- coding: utf-8 -*-
"""
Sequential-mode edge manager
===========================

Full perception ➜ AB3DMOT tracking ➜ linear prediction chain for BM2CP.

YAML (edge_list entry)
----------------------
edge_list:
  - manager_type: bm2cp
    mode: PREDICTION
    bm2cp_model:
      hypes_yaml:  opencda/BM2CP/opencood/logs/opv2v_bm2cp_det_2025_07_03_23_44_53/config.yaml
      checkpoint:  opencda/BM2CP/opencood/logs/opv2v_bm2cp_det_2025_07_03_23_44_53/net_epoch50.pth
      # score_thresh: 0.25        # (optional) confidence cut-off
"""

import os, time, torch, numpy as np
from collections import deque
import carla

import opencood.hypes_yaml.yaml_utils as yaml_utils
from opencood.tools   import train_utils
from opencood.models.point_pillar_bm2cp import PointPillarBM2CP

from easydict import EasyDict as edict
from AB3DMOT_libs.model import AB3DMOT
from opencda.core.

from opencda.core.prediction.linear_predictor_manager import \
    LinearPredictorManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.core.sensing.tracking.obstacle_trajectory import ObstacleTrajectory
from opencda.core.application.edge.edge_debug_helper import EdgeDebugHelper


# ───────────────────────── helpers ──────────────────────────
_GUID = 0
def _stack_feats(vms):
    feats, proj, poses = [], [], []
    for vm in vms:
        pm = vm.perception_manager
        feats.append(pm.bev_feat.cpu())       # (C,H,W)
        proj .append(pm.proj_mat.cpu())       # (4,4) lidar→ego
        poses.append(vm.localizer.get_ego_pos())
    return feats, proj, poses


def _detections_from_bm2cp(preds, frame_id):
    """BM2CP decoded boxes ➜ dict that AB3DMOT wants."""
    global _GUID
    det, info = [], []
    for det_dict in preds:
        boxes   = det_dict['pred_box_tensor'].cpu().numpy()   # (N,7)
        scores  = det_dict['pred_score'].cpu().numpy()
        for b, sc in zip(boxes, scores):
            x,y,z,l,w,h,yaw = b
            det.append([h,w,l,x,y,z,yaw])     # KITTI order
            _GUID += 1
            info.append([frame_id, _GUID, -1])
    det = np.asarray(det , dtype=np.float32)
    info= np.asarray(info, dtype=np.int64 )
    return {'dets': det, 'info': info}


def _box_to_transform(box):
    """[h,w,l,x,y,z,yaw] → CARLA Transform (centre)."""
    x,y,z,l,w,h,yaw = box
    loc = carla.Location(x=float(x), y=float(y), z=float(z))
    rot = carla.Rotation(yaw=np.degrees(float(yaw)))
    return carla.Transform(loc, rot)


# ─────────────────── EdgeManagerSequential ───────────────────
class EdgeManagerSequential:
    """
    Drop-in replacement when *manager_type: bm2cp* and *mode: PREDICTION*.
    """

    # ---------------------------------------------------------
    def __init__(self, world, cfg, cav_world, carla_client,
                 world_dt=0.05, **kw):

        self.world = world
        self.dt    = world_dt
        self.vm_list, self.rsu_list = [], []
        self.debug = EdgeDebugHelper(0)

        # ─ 1.  load BM2CP detector ───────────────────────────
        bm_cfg   = cfg['bm2cp_model']
        hypes    = yaml_utils.load_yaml(bm_cfg['hypes_yaml'])
        if 'score_thresh' in bm_cfg:
            hypes['postprocess']['target_args']['score_threshold'] = \
                float(bm_cfg['score_thresh'])

        self.hypes = hypes
        self.model = train_utils.create_model(hypes).cuda().eval()

        ckpt_dir = os.path.dirname(bm_cfg['checkpoint'])
        epoch_id = int(bm_cfg['checkpoint'].split('epoch')[-1].split('.')[0])
        _, self.model = train_utils.load_model(
            ckpt_dir, self.model, epoch_id, start_from_best=False)

        # ─ 2.  AB3DMOT tracker ───────────────────────────────
        trk_cfg = dict(vis=False, save_path=None, use_3d_iou=False,
                       thres=2.0, output_dir=None, min_hits=3,
                       max_age=2, ego_com=None, affi_pro=False,
                       dataset="KITTI", det_name="bm2cp")
        self.tracker  = AB3DMOT(trk_cfg, 'Car')
        self.trk_hist = deque(maxlen=10)

        # ─ 3.  linear predictor ──────────────────────────────
        self.lin_pred = LinearPredictorManager(num_future_steps=25)
        self.tracked_trajectories = {}

    # ---------------------------------------------------------
    def add_member(self, vm):  self.vm_list.append(vm)
    def add_rsu   (self, rsu): self.rsu_list.append(rsu)

    # ---------------------------------------------------------
    @torch.no_grad()
    def run_step(self, tick: int):

        # 1 ─ BM2CP fusion/decoding
        t0 = time.perf_counter()
        feats, proj, poses = _stack_feats(self.vm_list)
        pred_list   = self.model.collaborative_inference(feats, proj, poses)
        dets_dict   = _detections_from_bm2cp(pred_list, tick)

        # 2 ─ tracking
        tracks_np, _ = self.tracker.track(dets_dict, tick)
        self.trk_hist.appendleft(tracks_np)
        track_ms = (time.perf_counter() - t0)*1e3

        # 3 ─ build short trajectories & predict
        t1 = time.perf_counter()
        self._tracks_to_trajs(self.trk_hist, horizon=10)
        preds = self.lin_pred.generate_predicted_trajectories(
                    self.tracked_trajectories)
        pred_ms = (time.perf_counter() - t1)*1e3

        # 4 ─ broadcast predictions
        for vm in self.vm_list:
            vm.agent.edge_predictions = preds.copy()

        # 5 ─ debug stats
        self.debug.update_edge(0,
                               tracking_time=track_ms,
                               prediction_time=pred_ms)

        # 6 ─ let normal control loop run
        for vm in self.vm_list:
            vm.update_info(tick)
            vm.vehicle.apply_control(vm.run_step())

        for rsu in self.rsu_list:
            rsu.update_info()
            rsu.run_step()

    # ---------------------------------------------------------
    def _tracks_to_trajs(self, hist: deque, horizon: int):
        """
        Convert AB3DMOT history ➜ {track_id : ObstacleTrajectory}
        """
        updated = set()
        for frame_tracks in hist:
            if frame_tracks is None or len(frame_tracks)==0:
                continue
            for trk in frame_tracks:
                tid = int(trk[7])
                tf  = _box_to_transform(trk[:7])
                updated.add(tid)

                if tid not in self.tracked_trajectories:
                    dummy = ObstacleVehicle(corners=np.zeros((8,3)),
                                            o3d_bbx=None,
                                            track_id=tid,
                                            tick_id=0)
                    self.tracked_trajectories[tid] = ObstacleTrajectory(
                        dummy, deque(maxlen=horizon))

                traj = self.tracked_trajectories[tid]
                traj.trajectory.appendleft(tf)
                traj.obstacle.transform = tf
                traj.obstacle.location  = tf.location

        # drop stale
        for tid in list(self.tracked_trajectories):
            if tid not in updated:
                del self.tracked_trajectories[tid]
