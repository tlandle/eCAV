#!/usr/bin/env python3
"""Generate plots comparing static vs adaptive policy from offline profile data."""

import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from pathlib import Path
from collections import defaultdict

OUT = Path("paper2_figures")
CSV = OUT / "offline_profile.csv"

plt.rcParams.update({
    'font.size': 11,
    'font.family': 'serif',
    'figure.figsize': (7, 4.5),
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# Load CSV
rows = []
with open(CSV) as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append({
            'tick': int(r['tick']),
            'N': int(r['N']),
            'fusion': float(r['fusion_ms']),
            'detection': float(r['detection_ms']),
            'tracking': float(r['tracking_ms']),
            'prediction': float(r['prediction_ms']),
            'total': float(r['total_ms']),
            'n_dets': int(r['n_detections']),
            'n_tracks': int(r['n_tracks']),
            'n_preds': int(r['n_predictions']),
            'deadline_met': r['deadline_met'] == 'True',
            'policy': r['predictor'],
        })

# Filter to steady state (ticks >= 25 for mature tracks)
steady = [r for r in rows if r['tick'] >= 25]

# Group by policy and N
def agg(policy, metric):
    result = {}
    for N in sorted(set(r['N'] for r in steady if r['policy'] == policy)):
        vals = [r[metric] for r in steady if r['policy'] == policy and r['N'] == N]
        if vals:
            result[N] = np.mean(vals)
    return result

Ns = sorted(set(r['N'] for r in steady))
Ns_arr = np.array(Ns)

static_total = [agg('mtr_static', 'total').get(N, 0) for N in Ns]
adaptive_total = [agg('mtr_adaptive', 'total').get(N, 0) for N in Ns]
static_pred = [agg('mtr_static', 'prediction').get(N, 0) for N in Ns]
adaptive_pred = [agg('mtr_adaptive', 'prediction').get(N, 0) for N in Ns]
static_fuse = [agg('mtr_static', 'fusion').get(N, 0) for N in Ns]
static_track = [agg('mtr_static', 'tracking').get(N, 0) for N in Ns]

def deadline_rate(policy, N):
    rows_f = [r for r in steady if r['policy'] == policy and r['N'] == N]
    if not rows_f:
        return 0
    return sum(1 for r in rows_f if r['deadline_met']) / len(rows_f) * 100

static_dl = [deadline_rate('mtr_static', N) for N in Ns]
adaptive_dl = [deadline_rate('mtr_adaptive', N) for N in Ns]

DEADLINE_MS = 100.0

# ── Fig 1: Total compute static vs adaptive ──────────────────────
fig, ax = plt.subplots()
ax.plot(Ns_arr, static_total, 'o-', color='#e15759', label='Static MTR', linewidth=2, markersize=6)
ax.plot(Ns_arr, adaptive_total, 's-', color='#4e79a7', label='Adaptive MTR (ours)', linewidth=2, markersize=6)
ax.axhline(y=DEADLINE_MS, color='k', linestyle='--', linewidth=1.5, label='100 ms deadline')
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Total compute latency (ms)')
ax.set_title('End-to-End Edge Compute: Static vs Adaptive')
ax.legend(fontsize=9, loc='upper left')
ax.set_xlim(Ns_arr[0], Ns_arr[-1])
ax.set_ylim(0, max(max(static_total), max(adaptive_total)) * 1.15)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(OUT / 'fig_adaptive_total.pdf', dpi=150)
fig.savefig(OUT / 'fig_adaptive_total.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_adaptive_total.png'}")

# ── Fig 2: Per-stage breakdown static (stacked) ──────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)

for ax, policy, label in [(ax1, 'mtr_static', 'Static'), (ax2, 'mtr_adaptive', 'Adaptive')]:
    fuse = np.array([agg(policy, 'fusion').get(N, 0) for N in Ns])
    det = np.array([agg(policy, 'detection').get(N, 0) for N in Ns])
    trk = np.array([agg(policy, 'tracking').get(N, 0) for N in Ns])
    pred = np.array([agg(policy, 'prediction').get(N, 0) for N in Ns])
    ax.stackplot(Ns_arr, fuse, det, trk, pred,
                 labels=['Fusion', 'Detection', 'Tracking', 'Prediction'],
                 colors=['#e15759', '#f28e2b', '#f5c76e', '#4e79a7'],
                 alpha=0.85)
    ax.axhline(y=DEADLINE_MS, color='k', linestyle='--', linewidth=1.5)
    ax.set_xlabel('Number of agents (N)')
    ax.set_title(f'{label} MTR')
    ax.set_xlim(Ns_arr[0], Ns_arr[-1])
    ax.set_ylim(0, 160)
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if ax == ax1:
        ax.set_ylabel('Compute latency (ms)')
        ax.legend(loc='upper left', fontsize=9)

fig.suptitle('Per-Stage Latency: Static vs Adaptive MTR')
fig.tight_layout()
fig.savefig(OUT / 'fig_adaptive_breakdown.pdf', dpi=150)
fig.savefig(OUT / 'fig_adaptive_breakdown.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_adaptive_breakdown.png'}")

# ── Fig 3: Deadline compliance rate ──────────────────────────────
fig, ax = plt.subplots()
x = np.arange(len(Ns))
width = 0.35
ax.bar(x - width/2, static_dl, width, label='Static MTR',
       color='#e15759', alpha=0.85)
ax.bar(x + width/2, adaptive_dl, width, label='Adaptive MTR',
       color='#4e79a7', alpha=0.85)
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Deadline compliance (%)')
ax.set_title('100 ms Deadline Compliance Rate')
ax.set_xticks(x)
ax.set_xticklabels([str(n) for n in Ns])
ax.set_ylim(0, 105)
ax.legend(fontsize=9)
ax.axhline(y=100, color='gray', linestyle=':', linewidth=1)
fig.tight_layout()
fig.savefig(OUT / 'fig_deadline_compliance.pdf', dpi=150)
fig.savefig(OUT / 'fig_deadline_compliance.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_deadline_compliance.png'}")

# ── Fig 4: Prediction cost zoom ──────────────────────────────────
fig, ax = plt.subplots()
ax.plot(Ns_arr, static_pred, 'o-', color='#e15759',
        label='Static: MTR on all tracks', linewidth=2, markersize=6)
ax.plot(Ns_arr, adaptive_pred, 's-', color='#4e79a7',
        label='Adaptive: MTR on risk-selected subset', linewidth=2, markersize=6)
ax.set_xlabel('Number of cooperative agents (N)')
ax.set_ylabel('Prediction latency (ms)')
ax.set_title('Prediction Cost: Effect of Adaptive Subset Selection')
ax.legend(fontsize=9)
ax.set_xlim(Ns_arr[0], Ns_arr[-1])
ax.set_ylim(0)
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
fig.tight_layout()
fig.savefig(OUT / 'fig_adaptive_prediction.pdf', dpi=150)
fig.savefig(OUT / 'fig_adaptive_prediction.png', dpi=150)
plt.close(fig)
print(f"Saved {OUT / 'fig_adaptive_prediction.png'}")

# Summary table
print("\n=== Summary: Static vs Adaptive ===")
print(f"{'N':>4s} {'Static Total':>14s} {'Adapt Total':>14s} {'Static DL%':>12s} {'Adapt DL%':>12s} {'Pred Savings':>14s}")
for i, N in enumerate(Ns):
    saving = static_pred[i] - adaptive_pred[i]
    print(f"{N:4d} {static_total[i]:14.1f} {adaptive_total[i]:14.1f} "
          f"{static_dl[i]:11.0f}% {adaptive_dl[i]:11.0f}% {saving:13.1f}ms")
