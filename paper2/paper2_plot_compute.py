#!/usr/bin/env python3
"""Generate compute-only edge pipeline plots from measured benchmark data."""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from pathlib import Path

OUT = Path("paper2_figures")
CSV = OUT / "benchmark_timing.csv"

plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'figure.figsize': (7, 4.5),
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Read CSV
rows = []
with open(CSV) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

Ns = np.array([int(r['N']) for r in rows])
fusion = np.array([float(r['fusion_mean_ms']) for r in rows])
tracking = np.array([float(r['tracking_mean_ms']) for r in rows])
prediction = np.array([float(r['prediction_mean_ms']) for r in rows])
compute = fusion + tracking + prediction

# ── Fig 1: Compute-only stacked area ──────────────────────────────
fig, ax = plt.subplots()
ax.stackplot(Ns, fusion, tracking, prediction,
             labels=['Fusion (Where2Comm)', 'Tracking (AB3DMOT)',
                     'Prediction (SMART)'],
             colors=['#e15759', '#f28e2b', '#b07aa1'],
             alpha=0.85)
ax.axhline(y=100, color='k', linestyle='--', linewidth=1.5, label='100 ms deadline')
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Compute latency (ms)')
ax.set_title('Edge Compute: Per-Stage Latency (GPU-measured, RTX 4080 SUPER)')
ax.legend(loc='upper left', fontsize=9)
ax.set_xlim(Ns[0], Ns[-1])
ax.set_ylim(0, max(compute) * 1.1)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(OUT / 'fig_compute_only.pdf', dpi=150)
fig.savefig(OUT / 'fig_compute_only.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_compute_only.png'}")

# ── Fig 2: Per-component line plot with error context ─────────────
fig, ax = plt.subplots()
ax.plot(Ns, fusion, 'o-', color='#e15759', label='Fusion (Where2Comm)', linewidth=2, markersize=5)
ax.plot(Ns, tracking, 's-', color='#f28e2b', label='Tracking (AB3DMOT)', linewidth=2, markersize=5)
ax.plot(Ns, prediction, '^-', color='#b07aa1', label='Prediction (SMART)', linewidth=2, markersize=5)
ax.plot(Ns, compute, 'D-', color='#4e79a7', label='Total compute', linewidth=2.5, markersize=5)
ax.axhline(y=100, color='k', linestyle='--', linewidth=1.5, label='100 ms deadline')
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Latency (ms)')
ax.set_title('Edge Compute: Component Scaling (GPU-measured)')
ax.legend(fontsize=9)
ax.set_xlim(Ns[0], Ns[-1])
ax.set_ylim(0, max(compute) * 1.1)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(OUT / 'fig_compute_components.pdf', dpi=150)
fig.savefig(OUT / 'fig_compute_components.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_compute_components.png'}")

# ── Fig 3: Scene complexity ───────────────────────────────────────
n_dets = np.array([int(r['n_dets']) for r in rows])
n_tracks = np.array([int(r['n_tracks']) for r in rows])
fig, ax = plt.subplots()
ax.plot(Ns, n_dets, 'o-', color='#f28e2b', label='Detections / frame', linewidth=2)
ax.plot(Ns, n_tracks, 's-', color='#b07aa1', label='Confirmed tracks / frame', linewidth=2)
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Object count per frame')
ax.set_title('Scene Complexity vs Agent Count')
ax.legend(fontsize=9)
ax.set_xlim(Ns[0], Ns[-1])
ax.set_ylim(0)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(OUT / 'fig_scene_complexity.pdf', dpi=150)
fig.savefig(OUT / 'fig_scene_complexity.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_scene_complexity.png'}")

# Print summary table
print("\n=== Compute-Only Summary (all GPU-measured) ===")
print(f"{'N':>4s}  {'Fusion':>8s}  {'Track':>8s}  {'Predict':>8s}  {'Total':>8s}")
for i, N in enumerate(Ns):
    print(f"{N:4d}  {fusion[i]:8.1f}  {tracking[i]:8.1f}  {prediction[i]:8.1f}  {compute[i]:8.1f}")
