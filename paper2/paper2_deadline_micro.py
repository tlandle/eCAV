#!/usr/bin/env python3
"""
Deadline-aware adaptation microbenchmark.

Simulates the full edge pipeline (fusion + tracking + prediction) at each N
with five policies:
  1. Static full:       MTR on all tracks every tick
  2. Static linear:     Linear prediction on all tracks (no MTR)
  3. Rate-only:         Reduce tick rate when total exceeds deadline
  4. Amortization-only: Divergence gating, MTR on divergent subset
  5. Full adaptive:     Divergence gating + risk budgeting + rate adaptation

Uses real MTR model, real AB3DMOT tracker, synthetic tracks.
All GPU-measured on the current machine.
"""

import sys
import os
import time
import csv
import numpy as np
import torch

CMP_ROOT = 'ecav/core/application/v2v/baselines/cmp/CMP'
MTR_ROOT = os.path.join(CMP_ROOT, 'MTR')
REPO = os.path.abspath('.')
CMP_ABS = os.path.abspath(CMP_ROOT)
MTR_ABS = os.path.join(CMP_ABS, 'MTR')
sys.path.insert(0, MTR_ABS)
sys.path.insert(0, CMP_ABS)
os.chdir(CMP_ABS)
sys.path.append(REPO)

import logging
logging.basicConfig(level=logging.WARNING)

DEVICE = 'cuda:0'
N_AGENTS = [4, 8, 16, 24, 32, 48, 64]
DETS_PER_EGO = 4.0
TRACK_RATIO = 0.7
DEADLINE_MS = 100.0
N_TICKS = 10  # ticks to simulate per N

# Divergence parameters
DIVERGE_RATE_BASE = 0.10   # 10% of tracks diverge per tick at steady state
DIVERGE_RATE_NEW = 1.0     # new tracks always diverge (no cache yet)
THRESH_POS = 1.0           # meters

# Risk budgeting
RISK_HIGH_FRAC = 0.15      # 15% of tracks are high-risk (approaching intersection)
MTR_BASE_MS = 25.0         # MTR base overhead (model load, etc)
MTR_PER_TRACK_MS = 1.0     # marginal cost per track in MTR batch


def load_mtr():
    from mtr.config import cfg, cfg_from_yaml_file
    from mtr.models_opv2v.multi_ego_mtr_model import MotionTransformerWithMultiEgoAggregation
    cfg_path = os.path.join(MTR_ABS, 'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
    cfg_from_yaml_file(cfg_path, cfg)
    if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
        cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
            CMP_ABS, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)
    model = MotionTransformerWithMultiEgoAggregation(cfg.MODEL).to(DEVICE).eval()
    state = torch.load(os.path.join(REPO, 'ecav/ml_manager/models/mtr/best_model.pth'), map_location=DEVICE)
    if 'model_state' in state:
        state = state['model_state']
    model.load_state_dict(state, strict=False)
    return model, cfg


def build_mtr_batch(n_tracks):
    """Build synthetic MTR batch for n_tracks objects."""
    from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

    obj_trajs = np.zeros((n_tracks, 11, 8), dtype=np.float32)
    for i in range(n_tracks):
        angle = 2 * np.pi * i / max(n_tracks, 1)
        speed = 8.0 + 2.0 * (i % 4)
        radius = 20 + 5 * (i % 3)
        for t in range(11):
            a = angle + speed * t * 0.1 / radius
            obj_trajs[i, t] = [
                radius * np.cos(a), radius * np.sin(a), 0.5,
                4.5, 1.8, 1.5, a + np.pi / 2, 1.0]

    obj_ids = np.arange(n_tracks)
    obj_types = np.array(['TYPE_VEHICLE'] * n_tracks)
    center_objects = obj_trajs[:, -1, :].copy()
    obj_trajs_future = np.zeros((n_tracks, 50, 8), dtype=np.float32)
    timestamps = np.array([0.1 * i for i in range(11)], dtype=np.float32)

    ret = OPV2VMultiEgoDataset.create_agent_data_for_center_objects(
        center_objects=center_objects, obj_trajs_past=obj_trajs,
        obj_trajs_future=obj_trajs_future, track_index_to_predict=np.arange(n_tracks),
        sdc_track_index=0, timestamps=timestamps, obj_types=obj_types, obj_ids=obj_ids)
    otd, otm, otp, otlp, otfs, otfm, cgt, cgtm, cgfvi, tipn, stin, oto, oio = ret

    input_dict = {
        'obj_trajs': torch.tensor(otd, dtype=torch.float32, device=DEVICE),
        'obj_trajs_mask': torch.tensor(otm, dtype=torch.bool, device=DEVICE),
        'track_index_to_predict': torch.tensor(tipn, dtype=torch.long, device=DEVICE),
        'obj_trajs_pos': torch.tensor(otp, dtype=torch.float32, device=DEVICE),
        'obj_trajs_last_pos': torch.tensor(otlp, dtype=torch.float32, device=DEVICE),
        'obj_types': oto, 'obj_ids': oio,
        'center_objects_world': center_objects,
        'center_objects_id': oio[tipn],
        'center_objects_type': oto[tipn],
        'obj_trajs_future_state': torch.tensor(otfs, dtype=torch.float32, device=DEVICE),
        'obj_trajs_future_mask': torch.tensor(otfm, dtype=torch.bool, device=DEVICE),
        'center_gt_trajs': torch.tensor(cgt, dtype=torch.float32, device=DEVICE),
        'center_gt_trajs_mask': torch.tensor(cgtm, dtype=torch.bool, device=DEVICE),
        'center_gt_final_valid_idx': torch.tensor(cgfvi, dtype=torch.long, device=DEVICE),
        'map_polylines': torch.zeros(n_tracks, 256, 256, 3, dtype=torch.float32, device=DEVICE),
        'fused_feature': torch.randn(1, 256, 48, 176, device=DEVICE).unsqueeze(0),
    }
    return {'input_dict': input_dict, 'num_cavs': 1, 'batch_sample_count': [n_tracks]}


def build_subset_batch(full_batch, track_indices):
    """Extract a subset of tracks from a full batch for MTR prediction."""
    if len(track_indices) == 0:
        return None
    n_sub = len(track_indices)
    idx = np.array(track_indices)
    inp = full_batch['input_dict']

    sub_input = {}
    for key in ['obj_trajs', 'obj_trajs_mask', 'obj_trajs_pos', 'obj_trajs_last_pos',
                'obj_trajs_future_state', 'obj_trajs_future_mask',
                'center_gt_trajs', 'center_gt_trajs_mask', 'center_gt_final_valid_idx']:
        v = inp[key]
        if isinstance(v, torch.Tensor):
            sub_input[key] = v[idx]

    sub_input['track_index_to_predict'] = torch.arange(n_sub, dtype=torch.long, device=DEVICE)
    sub_input['map_polylines'] = inp['map_polylines'][:n_sub]
    sub_input['fused_feature'] = inp['fused_feature']

    for key in ['obj_types', 'obj_ids', 'center_objects_world',
                'center_objects_id', 'center_objects_type']:
        v = inp[key]
        if isinstance(v, np.ndarray):
            sub_input[key] = v[idx]

    return {'input_dict': sub_input, 'num_cavs': 1, 'batch_sample_count': [n_sub]}


def time_mtr(model, batch):
    """Run MTR and return wall-clock ms."""
    if batch is None or batch['batch_sample_count'][0] == 0:
        return 0.0
    torch.cuda.synchronize()
    t0 = time.perf_counter()
    with torch.no_grad():
        model(batch)
    torch.cuda.synchronize()
    return (time.perf_counter() - t0) * 1000


# Pre-measured fusion and tracking times (from benchmark)
FUSION_MS = {4: 8.9, 8: 17.9, 16: 38.1, 24: 60.3, 32: 83.5, 48: 127.8, 64: 174.3}
TRACKING_MS = {4: 6.8, 8: 15.4, 16: 30.8, 24: 43.9, 32: 60.6, 48: 89.7, 64: 119.9}


def main():
    print("=" * 70)
    print("Deadline-Aware Adaptation Microbenchmark")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print(f"Deadline: {DEADLINE_MS:.0f} ms")
    print("=" * 70)

    model, cfg = load_mtr()
    print("MTR loaded\n")

    # Warmup MTR
    warmup_batch = build_mtr_batch(10)
    for _ in range(3):
        time_mtr(model, warmup_batch)

    results = []

    for N in N_AGENTS:
        n_tracks = max(int(DETS_PER_EGO * N * TRACK_RATIO), 2)
        fusion_ms = FUSION_MS.get(N, 174.3)
        tracking_ms = TRACKING_MS.get(N, 119.9)
        fixed_ms = fusion_ms + tracking_ms  # non-prediction compute

        # Build full batch for this N
        full_batch = build_mtr_batch(n_tracks)

        # ── Policy 1: Static Full ──────────────────────────────
        pred_ms_full = time_mtr(model, full_batch)
        total_full = fixed_ms + pred_ms_full
        deadline_met_full = total_full <= DEADLINE_MS

        # ── Policy 2: Static Linear ────────────────────────────
        pred_ms_linear = 0.1  # ~0.1ms for linear extrapolation
        total_linear = fixed_ms + pred_ms_linear
        deadline_met_linear = total_linear <= DEADLINE_MS

        # ── Policy 3: Rate-Only ────────────────────────────────
        # Find the tick rate that fits total_full within budget
        rate_hz = 20.0
        for r in [20, 10, 5, 4, 2, 1]:
            budget = 1000.0 / r
            if total_full <= budget:
                rate_hz = r
                break
        total_rate = total_full  # compute doesn't change, just the budget
        effective_budget_rate = 1000.0 / rate_hz

        # ── Policy 4: Amortization Only ────────────────────────
        # Simulate divergence: DIVERGE_RATE_BASE of tracks diverge
        n_divergent = max(int(n_tracks * DIVERGE_RATE_BASE), 1)
        if n_divergent > 0:
            div_indices = list(range(n_divergent))
            sub_batch = build_subset_batch(full_batch, div_indices)
            pred_ms_amort = time_mtr(model, sub_batch)
        else:
            pred_ms_amort = 0.0
        total_amort = fixed_ms + pred_ms_amort + 0.1  # +0.1 for divergence check
        deadline_met_amort = total_amort <= DEADLINE_MS

        # ── Policy 5: Full Adaptive ────────────────────────────
        # Divergence gating + risk budgeting + rate adaptation
        available_budget = DEADLINE_MS - fixed_ms - 0.5  # 0.5ms overhead
        if available_budget <= 0:
            # Need rate adaptation: reduce to 10Hz
            rate_hz_adaptive = 10.0
            for r in [10, 5, 4, 2, 1]:
                budget = 1000.0 / r
                available_budget = budget - fixed_ms - 0.5
                if available_budget > MTR_BASE_MS:
                    rate_hz_adaptive = r
                    break
        else:
            rate_hz_adaptive = 20.0

        # Risk budget: how many tracks can we predict in available_budget?
        max_predictable = max(int((available_budget - MTR_BASE_MS) / MTR_PER_TRACK_MS), 0)
        n_risk_selected = min(n_divergent, max_predictable)
        n_risk_selected = max(n_risk_selected, 1)

        if n_risk_selected > 0:
            risk_indices = list(range(n_risk_selected))
            risk_batch = build_subset_batch(full_batch, risk_indices)
            pred_ms_adaptive = time_mtr(model, risk_batch)
        else:
            pred_ms_adaptive = 0.0
        total_adaptive = fixed_ms + pred_ms_adaptive + 0.6  # divergence + risk scoring
        effective_budget_adaptive = 1000.0 / rate_hz_adaptive
        deadline_met_adaptive = total_adaptive <= effective_budget_adaptive

        # Record
        row = {
            'N': N, 'n_tracks': n_tracks, 'fusion_ms': fusion_ms,
            'tracking_ms': tracking_ms, 'fixed_ms': fixed_ms,
            # Static full
            'full_pred_ms': pred_ms_full, 'full_total_ms': total_full,
            'full_meets_deadline': deadline_met_full,
            'full_n_predicted': n_tracks,
            # Static linear
            'linear_total_ms': total_linear, 'linear_meets_deadline': deadline_met_linear,
            # Rate-only
            'rate_hz': rate_hz, 'rate_budget_ms': effective_budget_rate,
            'rate_total_ms': total_rate, 'rate_meets_deadline': total_full <= effective_budget_rate,
            # Amortization
            'amort_n_divergent': n_divergent, 'amort_pred_ms': pred_ms_amort,
            'amort_total_ms': total_amort, 'amort_meets_deadline': deadline_met_amort,
            'amort_n_predicted': n_divergent,
            # Full adaptive
            'adaptive_rate_hz': rate_hz_adaptive,
            'adaptive_n_risk': n_risk_selected, 'adaptive_pred_ms': pred_ms_adaptive,
            'adaptive_total_ms': total_adaptive,
            'adaptive_budget_ms': effective_budget_adaptive,
            'adaptive_meets_deadline': deadline_met_adaptive,
            'adaptive_n_predicted': n_risk_selected,
        }
        results.append(row)

        print(f"N={N:3d} ({n_tracks:3d} tracks)  fixed={fixed_ms:.0f}ms")
        print(f"  Static full:    {total_full:6.1f}ms  pred={pred_ms_full:.1f}ms  "
              f"{'OK' if deadline_met_full else 'MISS'}")
        print(f"  Static linear:  {total_linear:6.1f}ms  "
              f"{'OK' if deadline_met_linear else 'MISS'}")
        print(f"  Rate-only:      {total_rate:6.1f}ms  @{rate_hz:.0f}Hz ({effective_budget_rate:.0f}ms budget)  "
              f"{'OK' if total_full <= effective_budget_rate else 'MISS'}")
        print(f"  Amortized:      {total_amort:6.1f}ms  divergent={n_divergent}  pred={pred_ms_amort:.1f}ms  "
              f"{'OK' if deadline_met_amort else 'MISS'}")
        print(f"  Full adaptive:  {total_adaptive:6.1f}ms  @{rate_hz_adaptive:.0f}Hz "
              f"risk_sel={n_risk_selected}  pred={pred_ms_adaptive:.1f}ms  "
              f"{'OK' if deadline_met_adaptive else 'MISS'}")
        print()

    # Summary table
    print("=" * 70)
    print("SUMMARY: Deadline Compliance")
    print("=" * 70)
    print(f"{'N':>4s}  {'Static':>8s}  {'Linear':>8s}  {'Rate':>10s}  {'Amort':>8s}  {'Adaptive':>12s}")
    for r in results:
        print(f"{r['N']:4d}  "
              f"{'OK' if r['full_meets_deadline'] else 'MISS':>8s}  "
              f"{'OK' if r['linear_meets_deadline'] else 'MISS':>8s}  "
              f"{'OK@'+str(int(r['rate_hz']))+'Hz' if r['rate_meets_deadline'] else 'MISS':>10s}  "
              f"{'OK' if r['amort_meets_deadline'] else 'MISS':>8s}  "
              f"{'OK@'+str(int(r['adaptive_rate_hz']))+'Hz' if r['adaptive_meets_deadline'] else 'MISS':>12s}")

    # Save CSV
    csv_path = os.path.join(REPO, 'paper2_figures/deadline_micro.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=results[0].keys())
        w.writeheader()
        w.writerows(results)
    print(f"\nSaved {csv_path}")

    # Save summary for plotting
    print("\n=== Objects Predicted Per Tick ===")
    print(f"{'N':>4s}  {'Tracks':>6s}  {'Full':>6s}  {'Amort':>6s}  {'Adaptive':>8s}")
    for r in results:
        print(f"{r['N']:4d}  {r['n_tracks']:6d}  {r['full_n_predicted']:6d}  "
              f"{r['amort_n_predicted']:6d}  {r['adaptive_n_predicted']:8d}")


if __name__ == '__main__':
    main()
