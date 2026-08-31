#!/usr/bin/env python3
# Author: Tyler Landle <tlandle3@gatech.edu>
"""Fusion latency vs canvas size at fixed contributor count.

The locale-size question: the BEV grid grows with the canvas, and the
fusion stage (backbone + attention + heads) grows with grid area. This
sweeps the grid side at fixed N and reports wall latency on the edge
GPU; an out-of-memory result is recorded as infeasible, which is the
"cannot fit" bound. Canvas meters = cells x m_per_cell, taken from the
production model (140.8 m over 200 cells = 0.704 m/cell).

Usage: python scripts/canvas_latency_bench.py [-o out.csv]
"""
import sys, os, time, argparse, csv
sys.path.insert(0, 'ecav/worldfusion'); sys.path.insert(0, '.')
import torch
import torch.nn.functional as F
from opencood.hypes_yaml.yaml_utils import load_yaml
from opencood.models.point_pillar_worldfusion import PointPillarWorldFusion
import opencood.tools.train_utils as tu

M_PER_CELL = 140.8 / 200.0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-o', '--out', default='evaluation_outputs/canvas_latency.csv')
    ap.add_argument('--model-dir', default='ecav/worldfusion/opencood/logs/worldfusion_v2xsim_det_2026_01_19_21_00_10')
    args = ap.parse_args()
    h = load_yaml(os.path.join(args.model_dir, 'config.yaml'))
    m = PointPillarWorldFusion(h['model']['args']).to('cuda:0').eval()
    _, m = tu.load_model(args.model_dir, m, epoch=50)

    rows = []
    for N in (4, 8):
        for side in (96, 144, 200, 288, 400, 568, 800):
            canvas_m = side * M_PER_CELL
            try:
                sf = torch.randn(N, 64, side, side, device='cuda:0')
                rl = torch.tensor([N], dtype=torch.int64, device='cuda:0')
                pw = torch.eye(4, device='cuda:0').reshape(1, 1, 1, 4, 4).expand(1, N, N, 4, 4).contiguous()
                shrk = m.sensor.shrink_conv if m.sensor.shrink_flag else None
                hds = [shrk, m.cls_head, m.reg_head]

                def once():
                    bd = m.sensor.backbone({'spatial_features': sf})
                    bev = bd['spatial_features_2d']
                    if m.sensor.shrink_flag:
                        bev = m.sensor.shrink_conv(bev)
                    psm = m.cls_head(bev)
                    H1, W1 = side // 2, side // 2
                    if psm.shape[-2:] != (H1, W1):
                        psm = F.interpolate(psm, size=(H1, W1), mode='nearest')
                    m.fusion(sf, psm, rl, pw, backbone=m.sensor.backbone, heads=hds)

                with torch.no_grad():
                    for _ in range(3):
                        once()
                    ts = []
                    for _ in range(5):
                        torch.cuda.synchronize(); t0 = time.perf_counter()
                        once()
                        torch.cuda.synchronize(); ts.append((time.perf_counter() - t0) * 1e3)
                lat = sum(ts) / len(ts)
                mem = torch.cuda.max_memory_allocated() / 2**30
                rows.append((N, side, f"{canvas_m:.0f}", f"{lat:.1f}", f"{mem:.2f}", "ok"))
                print(f"N={N} side={side} canvas={canvas_m:.0f}m lat={lat:.1f}ms mem={mem:.2f}GB")
            except torch.cuda.OutOfMemoryError:
                rows.append((N, side, f"{canvas_m:.0f}", "", "", "OOM"))
                print(f"N={N} side={side} canvas={canvas_m:.0f}m OOM")
            finally:
                torch.cuda.empty_cache()
                torch.cuda.reset_peak_memory_stats()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['n_contributors', 'grid_side', 'canvas_m', 'latency_ms', 'peak_mem_gb', 'status'])
        w.writerows(rows)
    print(f"-> {args.out}")

if __name__ == '__main__':
    main()
