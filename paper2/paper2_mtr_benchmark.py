#!/usr/bin/env python3
"""Benchmark MTR prediction at varying track counts.

Constructs synthetic trajectory data using CMP's OPV2VMultiEgoDataset
utilities and times the full MTR forward pass (prediction + aggregation).
"""

import sys
import os
import time
import numpy as np
import torch

CMP_ROOT = 'ecav/core/application/v2v/baselines/cmp/CMP'
MTR_ROOT = os.path.join(CMP_ROOT, 'MTR')
sys.path.insert(0, MTR_ROOT)
sys.path.insert(0, CMP_ROOT)
sys.path.insert(0, '.')

DEVICE = 'cuda:0'
N_WARMUP = 3
N_REPEATS = 10
TRACK_COUNTS = [2, 5, 11, 22, 44, 67, 89]  # matching N=1,2,4,8,16,24,32 egos


def load_mtr_model():
    from mtr.config import cfg, cfg_from_yaml_file
    from mtr.models_opv2v.multi_ego_mtr_model import MotionTransformerWithMultiEgoAggregation

    cfg_path = os.path.join(MTR_ROOT, 'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
    cfg_from_yaml_file(cfg_path, cfg)

    if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
        cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
            CMP_ROOT, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)

    model = MotionTransformerWithMultiEgoAggregation(cfg.MODEL)
    model.to(DEVICE).eval()

    ckpt = 'ecav/ml_manager/models/mtr/best_model.pth'
    state = torch.load(ckpt, map_location=DEVICE)
    if 'model_state' in state:
        state = state['model_state']
    model.load_state_dict(state, strict=False)
    print(f"MTR loaded ({sum(p.numel() for p in model.parameters())/1e6:.1f}M params)")
    return model


def build_batch(n_tracks):
    """Build synthetic MTR batch_dict for n_tracks objects."""
    from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

    # Synthetic trajectories: (N, 11, 8) = [x,y,z,l,w,h,heading,valid]
    obj_trajs = np.zeros((n_tracks, 11, 8), dtype=np.float32)
    for i in range(n_tracks):
        angle = 2 * np.pi * i / n_tracks
        speed = 8.0 + 2.0 * (i % 4)
        radius = 20 + 5 * (i % 3)
        for t in range(11):
            a = angle + speed * t * 0.1 / radius
            obj_trajs[i, t] = [
                radius * np.cos(a),      # x
                radius * np.sin(a),      # y
                0.5,                      # z
                4.5,                      # l
                1.8,                      # w
                1.5,                      # h
                a + np.pi / 2,           # heading
                1.0,                      # valid
            ]

    obj_ids = np.arange(n_tracks)
    obj_types = np.array(['TYPE_VEHICLE'] * n_tracks)
    center_objects = obj_trajs[:, -1, :].copy()
    obj_trajs_future = np.zeros((n_tracks, 50, 8), dtype=np.float32)
    timestamps = np.array([0.1 * i for i in range(11)], dtype=np.float32)

    ret = OPV2VMultiEgoDataset.create_agent_data_for_center_objects(
        center_objects=center_objects,
        obj_trajs_past=obj_trajs,
        obj_trajs_future=obj_trajs_future,
        track_index_to_predict=np.arange(n_tracks),
        sdc_track_index=0,
        timestamps=timestamps,
        obj_types=obj_types,
        obj_ids=obj_ids,
    )
    otd, otm, otp, otlp, otfs, otfm, cgt, cgtm, cgfvi, tipn, stin, oto, oio = ret

    input_dict = {
        'obj_trajs': torch.tensor(otd, dtype=torch.float32, device=DEVICE),
        'obj_trajs_mask': torch.tensor(otm, dtype=torch.bool, device=DEVICE),
        'track_index_to_predict': torch.tensor(tipn, dtype=torch.long, device=DEVICE),
        'obj_trajs_pos': torch.tensor(otp, dtype=torch.float32, device=DEVICE),
        'obj_trajs_last_pos': torch.tensor(otlp, dtype=torch.float32, device=DEVICE),
        'obj_types': oto,
        'obj_ids': oio,
        'center_objects_world': center_objects,
        'center_objects_id': oio[tipn],
        'center_objects_type': oto[tipn],
        'obj_trajs_future_state': torch.tensor(otfs, dtype=torch.float32, device=DEVICE),
        'obj_trajs_future_mask': torch.tensor(otfm, dtype=torch.bool, device=DEVICE),
        'center_gt_trajs': torch.tensor(cgt, dtype=torch.float32, device=DEVICE),
        'center_gt_trajs_mask': torch.tensor(cgtm, dtype=torch.bool, device=DEVICE),
        'center_gt_final_valid_idx': torch.tensor(cgfvi, dtype=torch.long, device=DEVICE),
        'map_polylines': torch.zeros(n_tracks, 256, 256, 3, dtype=torch.float32, device=DEVICE),
        'fused_feature': torch.randn(1, 256, 48, 176, device=DEVICE).unsqueeze(0),  # [1, 1, 256, 48, 176] for indexing
    }
    batch = {
        'input_dict': input_dict,
        'num_cavs': 1,
        'batch_sample_count': [n_tracks],
    }
    return batch


def main():
    print("=" * 60)
    print("MTR Prediction Benchmark")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print("=" * 60)

    model = load_mtr_model()

    print(f"\n{'Tracks':>8s}  {'Mean (ms)':>10s}  {'Std':>8s}  {'P95':>8s}")
    print("-" * 40)

    results = []
    for n_tracks in TRACK_COUNTS:
        batch = build_batch(n_tracks)

        # Warmup
        for _ in range(N_WARMUP):
            with torch.no_grad():
                model(batch)

        # Timed
        times = []
        for _ in range(N_REPEATS):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            with torch.no_grad():
                model(batch)
            torch.cuda.synchronize()
            t1 = time.perf_counter()
            times.append((t1 - t0) * 1000)

        mean = np.mean(times)
        std = np.std(times)
        p95 = np.percentile(times, 95)
        results.append((n_tracks, mean, std, p95))
        print(f"{n_tracks:8d}  {mean:10.1f}  {std:8.1f}  {p95:8.1f}")

    # Save CSV
    import csv
    with open('paper2_figures/mtr_benchmark.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['n_tracks', 'mean_ms', 'std_ms', 'p95_ms'])
        for r in results:
            w.writerow([r[0], f"{r[1]:.2f}", f"{r[2]:.2f}", f"{r[3]:.2f}"])
    print(f"\nSaved paper2_figures/mtr_benchmark.csv")


if __name__ == '__main__':
    main()
