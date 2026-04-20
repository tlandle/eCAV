#!/usr/bin/env python3
"""Compare SMART vs MTR prediction quality on OPV2V test scenes."""

import sys, os, time, math, numpy as np, torch, logging

REPO = os.path.abspath('.')
CMP_ABS = os.path.abspath('ecav/core/application/v2v/baselines/cmp/CMP')
MTR_ABS = os.path.join(CMP_ABS, 'MTR')
sys.path.insert(0, MTR_ABS)
sys.path.insert(0, CMP_ABS)
os.chdir(CMP_ABS)
sys.path.append(REPO)

logging.basicConfig(level=logging.WARNING)
DEVICE = 'cuda:0'
MAX_SCENES = 100


def main():
    print("=" * 60)
    print("SMART vs MTR on OPV2V Test Scenes")
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    print("=" * 60)

    # Load MTR
    from mtr.config import cfg, cfg_from_yaml_file
    from mtr.models_opv2v.multi_ego_mtr_model import MotionTransformerWithMultiEgoAggregation
    from mtr.datasets.opv2v_multiego_dataset import OPV2VMultiEgoDataset

    cfg_path = os.path.join(MTR_ABS, 'tools/cfgs/opv2v/opv2v_multiego_cobevt_c256.yaml')
    cfg_from_yaml_file(cfg_path, cfg)
    if not os.path.isabs(cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER):
        cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER = os.path.join(
            CMP_ABS, cfg.MODEL.CONTEXT_ENCODER.LANE_ENCODER)

    mtr = MotionTransformerWithMultiEgoAggregation(cfg.MODEL).to(DEVICE).eval()
    state = torch.load(os.path.join(REPO, 'models/mtr/best_model.pth'), map_location=DEVICE)
    if 'model_state' in state: state = state['model_state']
    mtr.load_state_dict(state, strict=False)
    print("MTR loaded")

    # Load SMART
    from ecav.core.prediction.smart_predictor_manager import SMARTPredictorManager3D
    smart = SMARTPredictorManager3D(
        checkpoint_path=os.path.join(REPO, 'ecav/core/prediction/smart/checkpoints/epoch=105.ckpt'),
        map_cache_path=None, device=DEVICE, num_output_steps=50)
    print("SMART loaded")

    # Load dataset
    logger = logging.getLogger('ds')
    logger.setLevel(logging.INFO)
    dataset = OPV2VMultiEgoDataset(
        dataset_cfg=cfg.DATA_CONFIG, training=False, logger=logger)
    print(f"Dataset: {len(dataset)} scenes\n")

    mtr_ades, mtr_fdes, smart_ades, smart_fdes = [], [], [], []
    mtr_times, smart_times = [], []

    for idx in range(min(len(dataset), MAX_SCENES)):
        try:
            scene = dataset[idx]
        except Exception:
            continue
        if scene is None:
            continue

        batch = dataset.collate_batch([scene])
        total_tracks = sum(batch['batch_sample_count'])

        # Single-CAV edge mode
        batch_edge = {
            'input_dict': batch['input_dict'],
            'num_cavs': 1,
            'batch_sample_count': [total_tracks],
        }
        if 'fused_feature' in batch['input_dict']:
            if isinstance(batch['input_dict']['fused_feature'], list):
                batch_edge['input_dict'] = dict(batch['input_dict'])
                batch_edge['input_dict']['fused_feature'] = [
                    batch['input_dict']['fused_feature'][0]]

        # MTR
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        with torch.no_grad():
            mtr_out = mtr(batch_edge)
        torch.cuda.synchronize()
        mtr_ms = (time.perf_counter() - t0) * 1000
        mtr_times.append(mtr_ms)

        mtr_pred = mtr_out['pred_trajs'].cpu().numpy()
        mtr_scores = mtr_out['pred_scores'].cpu().numpy()
        best = mtr_scores.argmax(axis=1)
        N = mtr_pred.shape[0]
        mtr_best = mtr_pred[np.arange(N), best, :, :2]

        # SMART
        obj_trajs = batch['input_dict']['obj_trajs']
        if isinstance(obj_trajs, torch.Tensor):
            obj_trajs = obj_trajs.cpu().numpy()

        positions = obj_trajs[:, 0, :, :2]
        headings = obj_trajs[:, 0, :, 6] if obj_trajs.shape[-1] > 6 else np.zeros((N, 11))
        waymo_pos = positions.copy()
        waymo_pos[:, :, 1] = -positions[:, :, 1]
        waymo_head = -headings

        try:
            tok_idx, tok_pos, tok_head = smart._tokenize_agents(waymo_pos, waymo_head)
            data = smart._build_hetero_data(
                tok_idx, tok_pos, tok_head, waymo_pos, waymo_head, N)
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            with torch.no_grad():
                sout = smart.model.inference(data)
            torch.cuda.synchronize()
            smart_ms = (time.perf_counter() - t0) * 1000
            smart_times.append(smart_ms)

            smart_pred = sout['pred_traj'].cpu().numpy()
            smart_pred[:, :, 1] = -smart_pred[:, :, 1]  # Waymo->CARLA
        except Exception as e:
            print(f"  Scene {idx}: SMART failed ({e})")
            continue

        # GT comparison
        gt = batch['input_dict']['center_gt_trajs']
        gt_mask = batch['input_dict']['center_gt_trajs_mask']
        if isinstance(gt, torch.Tensor): gt = gt.cpu().numpy()
        if isinstance(gt_mask, torch.Tensor): gt_mask = gt_mask.cpu().numpy()

        n_compare = min(N, smart_pred.shape[0])
        for i in range(n_compare):
            gt_xy = gt[i, :, :2]
            T = min(50, gt_xy.shape[0], mtr_best.shape[1], smart_pred.shape[1])
            if T < 5:
                continue
            mask = gt_mask[i, :T] if gt_mask.ndim > 1 else np.ones(T)
            valid = mask > 0
            if valid.sum() < 5:
                continue

            mtr_err = np.linalg.norm(gt_xy[:T][valid] - mtr_best[i, :T][valid], axis=1)
            smart_err = np.linalg.norm(gt_xy[:T][valid] - smart_pred[i, :T][valid], axis=1)
            mtr_ades.append(mtr_err.mean())
            mtr_fdes.append(mtr_err[-1])
            smart_ades.append(smart_err.mean())
            smart_fdes.append(smart_err[-1])

        if (idx + 1) % 10 == 0:
            print(f"  {idx+1} scenes, {len(mtr_ades)} tracks compared")

    print("\n" + "=" * 60)
    print(f"RESULTS ({len(mtr_ades)} tracks, {len(mtr_times)} scenes)")
    print("=" * 60)

    if mtr_ades:
        print(f"\n  {'Model':<12s} {'ADE (m)':>10s} {'FDE (m)':>10s} {'Latency (ms)':>14s}")
        print(f"  {'MTR':<12s} {np.mean(mtr_ades):>10.3f} {np.mean(mtr_fdes):>10.3f} {np.mean(mtr_times):>14.1f}")
        print(f"  {'SMART':<12s} {np.mean(smart_ades):>10.3f} {np.mean(smart_fdes):>10.3f} {np.mean(smart_times):>14.1f}")

    import csv
    with open(os.path.join(REPO, 'paper2_figures/smart_vs_mtr_opv2v.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['model', 'ade', 'fde', 'latency_ms', 'n_tracks', 'n_scenes'])
        if mtr_ades:
            w.writerow(['MTR', f"{np.mean(mtr_ades):.4f}", f"{np.mean(mtr_fdes):.4f}",
                         f"{np.mean(mtr_times):.2f}", len(mtr_ades), len(mtr_times)])
            w.writerow(['SMART', f"{np.mean(smart_ades):.4f}", f"{np.mean(smart_fdes):.4f}",
                         f"{np.mean(smart_times):.2f}", len(smart_ades), len(smart_times)])
    print(f"\nSaved paper2_figures/smart_vs_mtr_opv2v.csv")


if __name__ == '__main__':
    main()
