#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The edge service pipeline for the ISPASS live-service timing validation.

One invocation runs what the WorldFusion edge manager runs on a fused frame:
intermediate-fusion forward (PointPillarWorldFusion, the checkpoint the
intermediate-fusion scenarios load), detection post-processing, AB3DMOT, the
track-to-trajectory conversion, and the MTR predictor (MTR-WF stage 1). The
tracker and trajectory glue are the edge manager's own methods, bound to a
bare instance so no CARLA world is needed.

Inputs are recorded Multi-V2X frames of one roadside-unit zone
(Town10HD, rsu_40, the densest zone in the dataset: 31 to 32 connected CAVs
over the selected window). A frame holds the voxelised lidar of the roadside
unit and its connected CAVs. For N contributors the first N agents of the frame
are used (the roadside unit and N-1 CAVs).
"""
from __future__ import annotations

import os
import re
import sys
import time
from collections import deque
from pathlib import Path
from typing import Dict, List

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[2]
for p in (REPO / "ecav", REPO / "ecav" / "worldfusion", REPO):
    sys.path.insert(0, str(p))

WF_DIR = REPO / "ecav/ml_manager/models/worldfusion_multiv2x_translaug_finetune"
WF_EPOCH = 45
MTR = dict(
    cmp_root=str(REPO / "ecav/core/application/v2v/baselines/cmp/CMP"),
    mtr_checkpoint=str(REPO / "ecav/ml_manager/models/mtr_wf_stage1/best_model.pth"),
    cfg_yaml=str(REPO / "ecav/core/application/v2v/baselines/cmp/CMP/MTR/tools/cfgs/multiv2x/multiv2x_multiego_worldfusion.yaml"),
    intention_points_file=str(REPO / "models/multiv2x_mtr_wf_translaug/multiv2x_cluster_64_center_dict.pkl"),
    dataset="multiv2x", aggregator="None", past_frames=10, future_frames=25,
    time_interval=0.2, history_subsample=1, device="cuda",
)
DATA_ROOT = "/data1/Datasets/Multi-V2X"
SCENARIO = "Town10HD__2023_11_15_00_20_38"
RSU = "rsu_40"
MAX_CAV = 33


def _conn_count(yaml_path: str) -> int:
    t = open(yaml_path).read()
    i = t.find("conn_agents:")
    if i < 0:
        return 0
    seg = t[i + 12:]
    j = re.search(r"\n[A-Za-z_]", seg)
    return len(re.findall(r"-?\d+", seg[: j.start()] if j else seg))


def select_window(n_frames: int, min_conn: int = 31) -> List[str]:
    """Longest run of consecutive timestamps of the zone with >= min_conn connected CAVs."""
    d = os.path.join(DATA_ROOT, SCENARIO, RSU)
    ts = sorted(f[:-5] for f in os.listdir(d) if re.fullmatch(r"\d+\.yaml", f))
    ok = [(_conn_count(os.path.join(d, t + ".yaml")) >= min_conn) for t in ts]
    best, cur = (0, 0), None
    for i, o in enumerate(ok + [False]):
        if o and cur is None:
            cur = i
        if not o and cur is not None:
            if i - cur > best[1] - best[0]:
                best = (cur, i)
            cur = None
    win = ts[best[0]:best[1]]
    if len(win) < 2:
        raise RuntimeError("no window with %d connected CAVs" % min_conn)
    return win[:n_frames]


class Pipeline:
    def __init__(self, n_frames: int = 20, device: str = "cuda"):
        from opencood.hypes_yaml.yaml_utils import load_yaml
        from opencood.data_utils.datasets import build_dataset
        from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
        from opencood.tools import train_utils
        from opencood.data_utils.post_processor.world_voxel_postprocessor import WorldVoxelPostprocessor
        from easydict import EasyDict as edict
        from AB3DMOT_libs.model import AB3DMOT
        from ecav.core.application.edge.edge_manager.edge_manager_worldfusion_ab3dmot_linear_predictor import WorldFusionEdge
        from ecav.core.prediction.mtr_edge_predictor import MTREdgePredictor

        # the glue modules print per track on every call: silence them, the service does not log per track
        import ecav.core.application.edge.edge_manager.edge_manager_worldfusion_ab3dmot_linear_predictor as _wfmod
        import ecav.core.prediction.mtr_edge_predictor as _mtrmod
        _wfmod.print = lambda *a, **k: None
        _mtrmod.print = lambda *a, **k: None
        self.device = device
        hypes = load_yaml(str(WF_DIR / "config.yaml"))
        hypes["root_dir"] = hypes["validate_dir"] = DATA_ROOT
        hypes["train_params"]["max_cav"] = MAX_CAV
        if "max_cav" in hypes["model"]["args"]:
            hypes["model"]["args"]["max_cav"] = MAX_CAV
        self.hypes = hypes
        ds = build_dataset(hypes, visualize=False, train=False)
        ds.max_cav = MAX_CAV
        sid = next(k for k, v in ds.scenario_database.items() if SCENARIO in v["path"])
        self.window = select_window(n_frames)
        ds.samples = [dict(scenario_id=sid, rsu_name=RSU, timestamp=t) for t in self.window]
        self._ds = ds
        self._staged = []

        model = PointPillarWorldFusion(hypes["model"]["args"]).to(device).eval()
        _, model = train_utils.load_model(str(WF_DIR), model, WF_EPOCH)
        self.model = model.eval()
        # stage the recorded frames: each agent's pillar features scattered to its
        # canvas (the vehicle-side step of the platform), kept sparse on the GPU
        self.frames = []
        sensor = model.sensor
        with torch.no_grad():
            for i in range(len(ds.samples)):
                b = ds.collate_batch_test([ds[i]])["ego"]
                pl = b["processed_lidar"]
                coords = pl["voxel_coords"].to(device)
                vf = pl["voxel_features"].to(device)
                vn = pl["voxel_num_points"].to(device)
                n_agents = int(b["record_len"][0])
                agents = []
                for k in range(n_agents):
                    mk = coords[:, 0] == k
                    c = coords[mk].clone()
                    c[:, 0] = 0
                    bd = dict(voxel_features=vf[mk], voxel_coords=c, voxel_num_points=vn[mk],
                              record_len=torch.tensor([1], device=device))
                    bd = sensor.scatter(sensor.pillar_vfe(bd))
                    s3 = bd["spatial_features_3d"]
                    bb, cc, zz, yy, xx = s3.shape
                    dense = s3.view(cc * zz, yy, xx)
                    nz = (dense != 0).any(dim=0).nonzero(as_tuple=False)
                    agents.append((nz, dense[:, nz[:, 0], nz[:, 1]].clone()))
                    self.canvas = (cc * zz, yy, xx)
                self.frames.append(dict(agents=agents, n_agents=n_agents,
                                        pair=b["pairwise_t_matrix"].to(device).float()))
        self.max_n = min(f["n_agents"] for f in self.frames)
        self.agent_counts = [f["n_agents"] for f in self.frames]

        # the edge manager's detection, tracking and trajectory glue on a bare instance
        m = object.__new__(WorldFusionEdge)
        m.post_processor = WorldVoxelPostprocessor(hypes["postprocess"], dataset=None, train=False)
        m.world_anchor = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        m.rsu_manager_list = []
        m.vehicle_manager_list = []
        m.anchoring = False
        m.cfg = {"edge_dt": 0.05}
        self._ab_cfg = edict(dict(vis=False, save_path=None, use_3d_iou=False, thres=2.0, output_dir=None,
                                  min_hits=3, max_age=3, ego_com=None, affi_pro=False, dataset="KITTI",
                                  det_name="deprecated", anchoring=False, dup_x_max=8.0, dup_y_max=2.0,
                                  dup_size_ratio=2.5, cull_consec_ticks=3))
        self._AB3DMOT = AB3DMOT
        self.mgr = m
        self.predictor = MTREdgePredictor(
            num_output_steps=30, output_dt=0.05, budget_ms=1000.0,
            enable_amortization=False, enable_risk_budget=False, ego_vehicles=[], **MTR)
        self.reset()

    def reset(self):
        """Fresh tracker and trajectory state (start of a run)."""
        m = self.mgr
        m.tracker = self._AB3DMOT(self._ab_cfg, "Car")
        m.mot_tracker = m.tracker
        m.tracked_trajectories = {}
        m.track_to_carla = {}
        m._tracker_output_history = deque(maxlen=30)
        if hasattr(self.predictor, "reset_cache"):
            self.predictor.reset_cache()
        self.n_calls = 0

    def frame_index(self, seq: int) -> int:
        """Recorded frame for capture sequence number seq: forward then backward
        through the window, so consecutive captures are consecutive recordings."""
        f = len(self.frames)
        if f == 1:
            return 0
        k = seq % (2 * f - 2)
        return k if k < f else 2 * f - 2 - k

    @torch.no_grad()
    def invoke(self, seq: int, n: int, predict: bool = True) -> Dict[str, float]:
        """One service invocation on capture seq with n contributors.
        Returns stage durations in ms (GPU synchronised at stage ends)."""
        fr = self.frames[self.frame_index(seq)]
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        C, H, W = self.canvas
        sf = torch.zeros((n, C, H, W), device=self.device)
        for k in range(n):
            nz, val = fr["agents"][k]
            sf[k][:, nz[:, 0], nz[:, 1]] = val
        record_len = torch.tensor([n], dtype=torch.int64, device=self.device)
        model, sensor = self.model, self.model.sensor
        # the edge manager's _run_fusion without its per-agent debug probes
        psm_single = []
        for k in range(n):
            bd = sensor.backbone({"spatial_features": sf[k:k + 1]})
            bev = bd["spatial_features_2d"]
            if sensor.shrink_flag:
                bev = sensor.shrink_conv(bev)
            psm_single.append(model.cls_head(bev))
        psm_single = torch.cat(psm_single, dim=0)
        fused, _, _ = model.fusion(
            sf, psm_single, record_len, fr["pair"], backbone=sensor.backbone,
            heads=[sensor.shrink_conv if sensor.shrink_flag else None, model.cls_head, model.reg_head])
        if sensor.shrink_flag:
            fused = sensor.shrink_conv(fused)
        out = {"psm": model.cls_head(fused), "rm": model.reg_head(fused)}
        det = self.mgr._to_ab3dmot_format({"psm": out["psm"], "rm": out["rm"]}, self.n_calls)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        n_trk = n_pred = 0
        if predict:
            m = self.mgr
            tracks, _ = m.tracker.track(det, self.n_calls)
            if tracks and len(tracks[0]) > 0:
                m._tracker_output_history.append(tracks[0])
            m._ab3d_history_to_trajs(m._tracker_output_history, horizon=30)
            n_trk = len(m.tracked_trajectories)
            t2 = time.perf_counter()
            if hasattr(self.predictor, "set_fused_feature"):
                self.predictor.set_fused_feature(fused.detach())
            preds = self.predictor.generate_predicted_trajectories(
                m.tracked_trajectories, source_tick=seq, publish_tick=seq)
            n_pred = len(preds) if preds else 0
            torch.cuda.synchronize()
            t3 = time.perf_counter()
        else:
            t2 = t3 = t1
        self.n_calls += 1
        return dict(total_ms=(t3 - t0) * 1e3, fusion_det_ms=(t1 - t0) * 1e3,
                    track_ms=(t2 - t1) * 1e3, predict_ms=(t3 - t2) * 1e3,
                    n_det=int(len(det["scores"])), n_trk=n_trk, n_pred=n_pred)


if __name__ == "__main__":
    import json
    p = Pipeline(n_frames=int(os.environ.get("FRAMES", 20)))
    print("window", p.window[0], p.window[-1], "frames", len(p.frames), "max_n", p.max_n, "agents", p.agent_counts)
    for n in (1, 2, 4, 8, 16, 24, 32):
        if n > p.max_n:
            print("N", n, "exceeds agents in window", p.max_n)
            continue
        p.reset()
        rows = [p.invoke(s, n) for s in range(45)][5:]
        med = {k: float(np.median([r[k] for r in rows])) for k in rows[0]}
        print("N", n, json.dumps({k: round(v, 1) for k, v in med.items()}))
