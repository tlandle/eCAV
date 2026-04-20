#!/usr/bin/env python3
"""
Compare MTR predictions: num_cavs=N (CMP V2V) vs num_cavs=1 (Edge).

Loads OPV2V test scenes with multiple CAVs, runs MTR both ways,
and compares ADE/FDE against ground truth trajectories.

Also measures latency for both configurations.
"""

import sys
import os
import time
import pickle
import numpy as np
import torch
from collections import defaultdict

CMP_ROOT = 'ecav/core/application/v2v/baselines/cmp/CMP'
MTR_ROOT = os.path.join(CMP_ROOT, 'MTR')
sys.path.insert(0, MTR_ROOT)
sys.path.insert(0, CMP_ROOT)
sys.path.insert(0, '.')

DEVICE = 'cuda:0'
PREPROC_ROOT = '/home/atlas/TrafficSimulator_eCloud/SOTA_Predictors/CMP/preprocessed_data/opv2v'


def load_model():
    from mtr.config import cfg, cfg_from_yaml_file
    from mtr.models_opv2v.multi_ego_mtr_model import MotionTransformerWithMultiEgoAggregation

    cfg_path = os.path.join(MTR_ROOT, 'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
    cfg_from_yaml_file(cfg_path, cfg)
    if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
        cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
            CMP_ROOT, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)

    model = MotionTransformerWithMultiEgoAggregation(cfg.MODEL)
    model.to(DEVICE).eval()

    ckpt = 'models/mtr/best_model.pth'
    state = torch.load(ckpt, map_location=DEVICE)
    if 'model_state' in state:
        state = state['model_state']
    model.load_state_dict(state, strict=False)
    print(f"MTR loaded ({sum(p.numel() for p in model.parameters())/1e6:.1f}M params)")
    return model


def load_dataset():
    """Load the OPV2V test dataset using CMP's dataset class."""
    from mtr.config import cfg
    from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

    # The dataset needs its config
    dataset_cfg = cfg.DATA_CONFIG
    import logging
    logger = logging.getLogger('mtr_benchmark')
    logging.basicConfig(level=logging.INFO)
    dataset = OPV2VMultiEgoDataset(
        dataset_cfg=dataset_cfg,
        training=False,
        logger=logger,
    )
    return dataset


def run_comparison(model, dataset, max_scenes=20):
    """Run MTR in both modes and compare."""
    from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

    results_multi = []  # num_cavs=N (CMP way)
    results_single = []  # num_cavs=1 (edge way)
    timing_multi = []
    timing_single = []

    num_scenes = min(len(dataset), max_scenes)
    print(f"\nProcessing {num_scenes} scenes...")

    for idx in range(num_scenes):
        try:
            scene_data = dataset[idx]
        except Exception as e:
            print(f"  Scene {idx}: load failed ({e})")
            continue

        if scene_data is None:
            continue

        # Build batch the CMP way (multi-CAV)
        batch_multi = dataset.collate_batch([scene_data])
        num_cavs = batch_multi['num_cavs']

        if num_cavs < 2:
            continue  # Need multi-CAV scenes for comparison

        # Run multi-CAV (CMP way)
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            output_multi = model(batch_multi)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        timing_multi.append((t1 - t0) * 1000)

        # Build single-CAV batch (edge way): merge all tracks into one CAV
        input_dict = batch_multi['input_dict']
        total_tracks = sum(batch_multi['batch_sample_count'])

        batch_single = {
            'input_dict': input_dict,  # same data, different grouping
            'num_cavs': 1,
            'batch_sample_count': [total_tracks],
        }
        # fused_feature: use only the first CAV's feature for single mode
        if 'fused_feature' in input_dict:
            if isinstance(input_dict['fused_feature'], list):
                batch_single['input_dict'] = dict(input_dict)
                batch_single['input_dict']['fused_feature'] = [input_dict['fused_feature'][0]]

        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            output_single = model(batch_single)
        torch.cuda.synchronize()
        t1 = time.perf_counter()
        timing_single.append((t1 - t0) * 1000)

        # Compare predictions against GT
        gt_trajs = input_dict['center_gt_trajs']
        gt_mask = input_dict['center_gt_trajs_mask']
        if isinstance(gt_trajs, torch.Tensor):
            gt_trajs = gt_trajs.cpu().numpy()
        if isinstance(gt_mask, torch.Tensor):
            gt_mask = gt_mask.cpu().numpy()

        # Get best-mode predictions (highest confidence)
        for label, output in [('multi', output_multi), ('single', output_single)]:
            pred_trajs = output['pred_trajs'].cpu().numpy()  # (N, modes, T, 5)
            pred_scores = output['pred_scores'].cpu().numpy()  # (N, modes)

            # Select best mode per object
            best_mode = pred_scores.argmax(axis=1)
            N = pred_trajs.shape[0]
            best_trajs = pred_trajs[np.arange(N), best_mode]  # (N, T, 5)

            # Compute ADE/FDE for first CAV's tracks only (fair comparison)
            n_first = batch_multi['batch_sample_count'][0]
            for i in range(min(n_first, N)):
                gt = gt_trajs[i]  # (T, dim)
                pred = best_trajs[i]  # (T, 5)
                mask = gt_mask[i] if gt_mask.ndim > 1 else gt_mask

                # Use x,y only
                if gt.shape[-1] >= 2 and pred.shape[-1] >= 2:
                    gt_xy = gt[:, :2]
                    pred_xy = pred[:, :2]
                    valid = mask > 0 if mask.ndim == 1 else np.ones(len(gt_xy), dtype=bool)
                    if valid.sum() > 0:
                        errors = np.linalg.norm(gt_xy[valid] - pred_xy[valid[:len(pred_xy)]], axis=1)
                        ade = errors.mean()
                        fde = errors[-1] if len(errors) > 0 else 0
                        if label == 'multi':
                            results_multi.append({'ade': ade, 'fde': fde})
                        else:
                            results_single.append({'ade': ade, 'fde': fde})

        n_tracks = sum(batch_multi['batch_sample_count'])
        print(f"  Scene {idx}: {num_cavs} CAVs, {n_tracks} tracks, "
              f"multi={timing_multi[-1]:.1f}ms, single={timing_single[-1]:.1f}ms")

    return results_multi, results_single, timing_multi, timing_single


def main():
    print("=" * 60)
    print("MTR num_cavs=N vs num_cavs=1 Comparison")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print("=" * 60)

    model = load_model()

    try:
        dataset = load_dataset()
        print(f"Dataset loaded: {len(dataset)} scenes")
    except Exception as e:
        print(f"Dataset load failed: {e}")
        import traceback
        traceback.print_exc()
        return

    results_multi, results_single, timing_multi, timing_single = run_comparison(model, dataset)

    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)

    if results_multi and results_single:
        ade_multi = np.mean([r['ade'] for r in results_multi])
        fde_multi = np.mean([r['fde'] for r in results_multi])
        ade_single = np.mean([r['ade'] for r in results_single])
        fde_single = np.mean([r['fde'] for r in results_single])

        print(f"\nPrediction Quality (lower is better):")
        print(f"  {'Mode':<20s} {'ADE (m)':>10s} {'FDE (m)':>10s} {'N samples':>10s}")
        print(f"  {'Multi-CAV (CMP)':<20s} {ade_multi:>10.3f} {fde_multi:>10.3f} {len(results_multi):>10d}")
        print(f"  {'Single-CAV (Edge)':<20s} {ade_single:>10.3f} {fde_single:>10.3f} {len(results_single):>10d}")
        print(f"  {'Degradation':<20s} {(ade_single-ade_multi)/ade_multi*100:>+9.1f}% {(fde_single-fde_multi)/fde_multi*100:>+9.1f}%")

    if timing_multi and timing_single:
        print(f"\nLatency:")
        print(f"  Multi-CAV (CMP):  {np.mean(timing_multi):.1f} ms (std={np.std(timing_multi):.1f})")
        print(f"  Single-CAV (Edge): {np.mean(timing_single):.1f} ms (std={np.std(timing_single):.1f})")
        print(f"  Speedup: {np.mean(timing_multi)/np.mean(timing_single):.2f}x")


if __name__ == '__main__':
    main()
