"""Run missing mechanism ablation configs (amort_only, risk_only) on Zone A.

Produces paper2_figures/ablation_mechanisms.csv with rows that can be
combined with existing ablation_rsu250_joint.csv for the 5-row table.
"""
import os
import sys
from pathlib import Path

import pandas as pd
import torch

REPO = str(Path(__file__).resolve().parents[1])
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, 'paper2'))

from paper2_offline_profiler_v3 import (
    OfflinePipelineV3,
    MultiV2XScene,
    profile_config,
)


def main():
    data_root = '/data1/Datasets/Multi-V2X'
    town = 'Town05__2023_11_13_23_03_07'
    rsu = 'rsu_250'
    max_frames = 50  # matches ablation_rsu250_joint.csv for apples-to-apples comparison

    pipeline = OfflinePipelineV3()
    scene = MultiV2XScene(data_root, town, rsu_name=rsu, max_frames=max_frames)

    lane_map_path = os.path.join(
        REPO, 'models/lane_maps', f'{town}_{scene.ego_cav}.png'
    )
    lane_img = None
    if os.path.exists(lane_map_path):
        from PIL import Image
        import numpy as np
        lane_img = np.array(Image.open(lane_map_path))
        print(f'Lane map loaded: {lane_map_path} ({lane_img.shape})')

    # Warmup
    agents = scene.connected_agents(0)
    if scene.ego_cav not in agents:
        agents = [scene.ego_cav] + agents
    agents = agents[:4]
    with torch.no_grad():
        sf, rl, pw = pipeline.voxelize_agents(scene, 0, agents)
        for _ in range(3):
            pipeline.run_edge_fusion(sf, pw, rl)
    print('Warmup done.')

    # Two missing configs: amort_only and risk_only with full contributor set (no filter)
    configs_to_run = [
        ('amort_only', 'amort_only'),
        ('risk_only', 'risk_only'),
    ]

    all_rows = []
    for label, policy in configs_to_run:
        print(f'\n--- Running {label} (policy={policy}, no filter) ---')
        rows = profile_config(
            pipeline, scene, policy,
            apply_filter=False, k_cav=None,
            lane_img=lane_img, filter_mode='none',
        )
        for r in rows:
            r['config'] = label
            r['town'] = town
            r['rsu'] = scene.ego_cav
        all_rows.extend(rows)

    out = 'paper2_figures/ablation_mechanisms.csv'
    df = pd.DataFrame(all_rows)
    df.to_csv(out, index=False)
    print(f'\nWrote {len(df)} rows to {out}')
    # Quick summary
    for cfg in df['config'].unique():
        d = df[(df['config'] == cfg) & (df['tick'] >= 5)]
        mean_ms = d['edge_ms'].mean()
        p95 = d['edge_ms'].quantile(0.95)
        dl = 100 * d['deadline_met'].mean()
        tp = d['det_tp'].sum()
        fn = d['det_fn'].sum()
        recall = tp / (tp + fn) if (tp + fn) else 0
        print(f'{cfg:15s} mean={mean_ms:6.1f}ms p95={p95:6.1f}ms dl={dl:5.1f}% recall={recall:.3f}')


if __name__ == '__main__':
    main()
