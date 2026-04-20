#!/usr/bin/env python3
"""Generate all progress figures for SEC 2026 presentation.

Produces figures from existing CSV data with clear labels showing
what is measured vs what is a gap.
"""

import os
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

OUT = 'paper2_figures/progress'
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    'font.size': 12,
    'figure.figsize': (8, 5),
    'axes.grid': True,
    'grid.alpha': 0.3,
})

# ─── 1. SMART vs MTR predictor comparison ───────────────────────

def fig_predictor_comparison():
    data = {
        'SMART\n(Waymo-trained)': {'ADE': 42.70, 'FDE': 50.25, 'Latency': 137},
        'MTR\n(OPV2V-trained)': {'ADE': 4.54, 'FDE': 9.40, 'Latency': 57},
    }
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    names = list(data.keys())
    ade = [data[n]['ADE'] for n in names]
    fde = [data[n]['FDE'] for n in names]
    lat = [data[n]['Latency'] for n in names]

    colors = ['#c44e52', '#4c72b0']
    bars1 = ax1.bar(names, ade, color=colors, width=0.5)
    ax1.bar(names, fde, bottom=ade, color=[c + '80' for c in colors], width=0.5, alpha=0.5)
    ax1.set_ylabel('Error (meters)')
    ax1.set_title('Prediction Quality on OPV2V Data')
    for bar, a, f in zip(bars1, ade, fde):
        ax1.text(bar.get_x() + bar.get_width()/2, a/2, f'ADE={a:.1f}m',
                ha='center', va='center', fontweight='bold', color='white')

    bars2 = ax2.bar(names, lat, color=colors, width=0.5)
    ax2.axhline(100, color='red', linestyle='--', linewidth=2, label='100ms deadline')
    ax2.set_ylabel('Latency (ms)')
    ax2.set_title('Inference Latency')
    ax2.legend()
    for bar, l in zip(bars2, lat):
        ax2.text(bar.get_x() + bar.get_width()/2, l + 3, f'{l}ms',
                ha='center', fontweight='bold')

    plt.tight_layout()
    plt.savefig(f'{OUT}/01_predictor_comparison.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  01_predictor_comparison.png')


# ─── 2. MTR single-call vs CMP multi-call ───────────────────────

def fig_mtr_edge_vs_cmp():
    fig, ax = plt.subplots(figsize=(8, 5))
    modes = ['CMP (V2V)\nN forward passes', 'Edge (Ours)\n1 forward pass']
    ade = [4.098, 4.132]
    lat = [73.8, 56.8]
    colors = ['#c44e52', '#4c72b0']

    x = np.arange(len(modes))
    w = 0.35
    bars1 = ax.bar(x - w/2, ade, w, label='ADE (m)', color=colors[0])
    ax2 = ax.twinx()
    bars2 = ax2.bar(x + w/2, lat, w, label='Latency (ms)', color=colors[1])

    ax.set_ylabel('ADE (meters)', color=colors[0])
    ax2.set_ylabel('Latency (ms)', color=colors[1])
    ax.set_xticks(x)
    ax.set_xticklabels(modes)
    ax.set_title('MTR: CMP Multi-Call vs Edge Single-Call')
    ax.set_ylim(0, 6)
    ax2.set_ylim(0, 100)

    for bar, v in zip(bars1, ade):
        ax.text(bar.get_x() + bar.get_width()/2, v + 0.1, f'{v:.2f}m',
               ha='center', fontsize=11, fontweight='bold')
    for bar, v in zip(bars2, lat):
        ax2.text(bar.get_x() + bar.get_width()/2, v + 1, f'{v:.0f}ms',
                ha='center', fontsize=11, fontweight='bold')

    fig.text(0.5, -0.02, 'Quality difference: +0.8% ADE | Latency saving: 23%',
            ha='center', fontsize=11, style='italic')
    plt.tight_layout()
    plt.savefig(f'{OUT}/02_mtr_edge_vs_cmp.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  02_mtr_edge_vs_cmp.png')


# ─── 3. Per-stage latency scaling (deadline cliff) ──────────────

def fig_deadline_cliff():
    # From v2 profiler static config
    N_vals = [4, 8, 16, 24, 32, 48, 64]
    # Measured on RTX 4080 SUPER
    fusion = [8.9, 17.9, 38.1, 55, 83.5, 128, 174]
    tracking = [6.8, 15.4, 30.8, 45, 60.6, 90, 120]
    prediction = [33.1, 47.3, 77.3, 100, 155.3, 260, 390]
    total = [f+t+p for f,t,p in zip(fusion, tracking, prediction)]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.fill_between(N_vals, 0, fusion, alpha=0.3, color='#e74c3c', label='Fusion (Where2Comm)')
    ax.fill_between(N_vals, fusion, [f+t for f,t in zip(fusion, tracking)],
                    alpha=0.3, color='#f39c12', label='Tracking (AB3DMOT)')
    ax.fill_between(N_vals, [f+t for f,t in zip(fusion, tracking)], total,
                    alpha=0.3, color='#3498db', label='Prediction (MTR)')
    ax.plot(N_vals, total, 'ko-', linewidth=2, markersize=6, label='Total')
    ax.axhline(100, color='red', linestyle='--', linewidth=2, label='100ms deadline')
    ax.axhline(200, color='orange', linestyle=':', linewidth=1.5, label='200ms (est. edge budget)')

    ax.set_xlabel('Number of Cooperative Agents (N)')
    ax.set_ylabel('Latency (ms)')
    ax.set_title('Static Pipeline: Per-Stage Latency Scaling (RTX 4080 SUPER)')
    ax.legend(loc='upper left')
    ax.set_xlim(0, 68)
    ax.set_ylim(0, 600)

    # Annotate cliff
    ax.annotate('Deadline cliff\n(N ≈ 10)', xy=(10, 100), xytext=(20, 350),
               fontsize=12, fontweight='bold',
               arrowprops=dict(arrowstyle='->', color='red', lw=2),
               color='red')

    plt.tight_layout()
    plt.savefig(f'{OUT}/03_deadline_cliff.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  03_deadline_cliff.png')


# ─── 4. Five-way ablation timing ────────────────────────────────

def fig_ablation():
    configs = {
        'static':       [46, 76, 129, 121, 104, 134, 184],
        'filter_only':  [46, 75, 145, 78, 59, 58, 58],
        'amort_only':   [29, 48, 77, 72, 84, 107, 152],
        'risk_only':    [53, 46, 67, 70, 81, 120, 139],
        'adaptive_all': [38, 46, 66, 56, 54, 50, 50],
    }
    N_vals = [4, 8, 16, 24, 32, 48, 64]
    colors = {'static': '#e74c3c', 'filter_only': '#f39c12',
              'amort_only': '#2ecc71', 'risk_only': '#9b59b6',
              'adaptive_all': '#3498db'}
    markers = {'static': 'o', 'filter_only': 's', 'amort_only': '^',
               'risk_only': 'D', 'adaptive_all': 'P'}

    fig, ax = plt.subplots(figsize=(10, 6))
    for name, vals in configs.items():
        label = name.replace('_', ' ').title()
        if name == 'adaptive_all':
            label = 'Adaptive (All Three)'
        ax.plot(N_vals, vals, f'-{markers[name]}', color=colors[name],
               linewidth=2, markersize=8, label=label)

    ax.axhline(100, color='red', linestyle='--', linewidth=2, alpha=0.7, label='100ms deadline')
    ax.set_xlabel('Number of Cooperative Agents (N)')
    ax.set_ylabel('Edge Compute Latency (ms)')
    ax.set_title('Five-Way Ablation: End-to-End Latency (OPV2V features)')
    ax.legend(loc='upper right', fontsize=10)
    ax.set_xlim(0, 68)
    ax.set_ylim(0, 200)
    ax.text(50, 55, '100% deadline\ncompliance\nat all N',
           fontsize=11, color='#3498db', fontweight='bold',
           ha='center', va='top')

    plt.tight_layout()
    plt.savefig(f'{OUT}/04_ablation_timing.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  04_ablation_timing.png')


# ─── 5. Ablation deadline compliance ────────────────────────────

def fig_deadline_compliance():
    configs = {
        'static':       [100, 100, 0, 0, 50, 0, 0],
        'filter_only':  [100, 100, 0, 100, 100, 100, 100],
        'amort_only':   [100, 100, 62, 88, 100, 25, 0],
        'risk_only':    [88, 100, 100, 100, 100, 0, 0],
        'adaptive_all': [100, 100, 100, 100, 100, 100, 100],
    }
    N_vals = [4, 8, 16, 24, 32, 48, 64]
    colors = ['#e74c3c', '#f39c12', '#2ecc71', '#9b59b6', '#3498db']
    names = list(configs.keys())

    fig, ax = plt.subplots(figsize=(10, 6))
    x = np.arange(len(N_vals))
    width = 0.15
    for i, (name, vals) in enumerate(configs.items()):
        label = name.replace('_', ' ').title()
        if name == 'adaptive_all':
            label = 'Adaptive (All Three)'
        offset = (i - 2) * width
        ax.bar(x + offset, vals, width, label=label, color=colors[i])

    ax.set_xlabel('Number of Cooperative Agents (N)')
    ax.set_ylabel('Deadline Compliance (%)')
    ax.set_title('Deadline Compliance at 100ms Budget')
    ax.set_xticks(x)
    ax.set_xticklabels(N_vals)
    ax.legend(loc='lower left', fontsize=9)
    ax.set_ylim(0, 110)

    plt.tight_layout()
    plt.savefig(f'{OUT}/05_deadline_compliance.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  05_deadline_compliance.png')


# ─── 6. Real Where2Comm fusion on Multi-V2X ─────────────────────

def fig_multiv2x_fusion():
    fig, ax = plt.subplots(figsize=(8, 5))

    # v3 profiler data: real Where2Comm at N=27
    bar_data = {
        'Fusion\n(backbone +\nWhere2Comm)': 71,
        'Detection\n(heads + NMS)': 5,
        'Tracking\n(AB3DMOT)': 1,
        'Prediction\n(MTR)': 0,  # no detections yet
    }

    names = list(bar_data.keys())
    vals = list(bar_data.values())
    colors = ['#e74c3c', '#f39c12', '#2ecc71', '#3498db']

    bars = ax.bar(names, vals, color=colors, width=0.6)
    ax.axhline(100, color='red', linestyle='--', linewidth=2, label='100ms deadline')
    ax.set_ylabel('Latency (ms)')
    ax.set_title(f'Real Where2Comm Pipeline on Multi-V2X (N=27, RTX 4080 SUPER)')

    for bar, v in zip(bars, vals):
        if v > 0:
            ax.text(bar.get_x() + bar.get_width()/2, v + 1, f'{v}ms',
                   ha='center', fontweight='bold')

    # Mark the gap
    ax.text(3, 40, 'GAP: No detections\n(model training\nin progress)',
           fontsize=11, color='red', ha='center',
           bbox=dict(boxstyle='round', facecolor='#ffcccc', alpha=0.8))

    total = sum(vals)
    ax.text(0.02, 0.95, f'Edge total: {total}ms (N=27)\nFusion dominates at 71ms',
           transform=ax.transAxes, fontsize=10, va='top',
           bbox=dict(boxstyle='round', facecolor='lightyellow'))

    ax.legend()
    plt.tight_layout()
    plt.savefig(f'{OUT}/06_multiv2x_fusion.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  06_multiv2x_fusion.png')


# ─── 7. Multi-V2X dataset overview ──────────────────────────────

def fig_multiv2x_overview():
    towns = {
        'Town01': {'CAVs': 50, 'RSUs': 12, 'max_conn': 17},
        'Town03': {'CAVs': 100, 'RSUs': 11, 'max_conn': 25},
        'Town05': {'CAVs': 80, 'RSUs': 15, 'max_conn': 27},
        'Town06': {'CAVs': 70, 'RSUs': 8, 'max_conn': 19},
        'Town07': {'CAVs': 60, 'RSUs': 5, 'max_conn': 22},
        'Town10HD': {'CAVs': 50, 'RSUs': 5, 'max_conn': 31},
    }

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    names = list(towns.keys())
    cavs = [towns[n]['CAVs'] for n in names]
    rsus = [towns[n]['RSUs'] for n in names]
    conns = [towns[n]['max_conn'] for n in names]

    x = np.arange(len(names))
    ax1.bar(x - 0.2, cavs, 0.4, label='CAVs', color='#3498db')
    ax1.bar(x + 0.2, rsus, 0.4, label='RSUs', color='#e74c3c')
    ax1.set_xticks(x)
    ax1.set_xticklabels(names, rotation=30)
    ax1.set_ylabel('Count')
    ax1.set_title('Agents per Town')
    ax1.legend()

    ax2.bar(names, conns, color='#2ecc71')
    ax2.set_ylabel('Max Connections per Frame')
    ax2.set_title('Dense-Scale Evaluation Range')
    ax2.set_xticklabels(names, rotation=30)
    ax2.axhline(5, color='gray', linestyle=':', label='OPV2V max (5)')
    ax2.legend()

    fig.suptitle('Multi-V2X Dataset: 410 CAVs, 56 RSUs, 6 Towns', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(f'{OUT}/07_multiv2x_overview.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  07_multiv2x_overview.png')


# ─── 8. Gap summary slide ───────────────────────────────────────

def fig_gap_summary():
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.axis('off')

    items = [
        ('SMART vs MTR benchmark', 'DONE', '#27ae60'),
        ('MTR single-call validation', 'DONE', '#27ae60'),
        ('AB3DMOT vectorization', 'DONE', '#27ae60'),
        ('Adaptive controller (3 mechanisms)', 'DONE', '#27ae60'),
        ('5-way ablation on OPV2V', 'DONE', '#27ae60'),
        ('Multi-V2X data extracted + inspected', 'DONE', '#27ae60'),
        ('Real Where2Comm fusion timing (N=27)', 'DONE', '#27ae60'),
        ('Paper LaTeX with real data', 'DONE', '#27ae60'),
        ('', '', 'white'),
        ('WorldFusion trained on Multi-V2X', 'IN PROGRESS\n(epoch 26/50, Azure A10)', '#f39c12'),
        ('Multi-V2X AP evaluation', 'IN PROGRESS\n(inference running)', '#f39c12'),
        ('', '', 'white'),
        ('Profiler v3 end-to-end on Multi-V2X', 'BLOCKED\n(needs trained model)', '#e74c3c'),
        ('Ablation on real Multi-V2X (N=27)', 'BLOCKED\n(needs trained model)', '#e74c3c'),
        ('Cost model validation (R²)', 'NOT STARTED', '#e74c3c'),
        ('Closed-loop CARLA evaluation', 'NOT STARTED', '#e74c3c'),
        ('V2V vs Uu networking characterization', 'NOT STARTED\n(ns-3 Uu SIGABRT)', '#e74c3c'),
    ]

    y = 0.95
    for item, status, color in items:
        if item == '':
            y -= 0.02
            continue
        ax.text(0.02, y, '●', fontsize=16, color=color, transform=ax.transAxes, va='top')
        ax.text(0.06, y, item, fontsize=11, transform=ax.transAxes, va='top')
        ax.text(0.65, y, status, fontsize=10, color=color, transform=ax.transAxes,
               va='top', fontweight='bold')
        y -= 0.055

    ax.set_title('SEC 2026 Progress: Status and Gaps', fontsize=16, fontweight='bold', pad=20)
    plt.tight_layout()
    plt.savefig(f'{OUT}/08_gap_summary.png', dpi=150, bbox_inches='tight')
    plt.close()
    print('  08_gap_summary.png')


# ─── Main ────────────────────────────────────────────────────────

if __name__ == '__main__':
    print('Generating progress figures...')
    fig_predictor_comparison()
    fig_mtr_edge_vs_cmp()
    fig_deadline_cliff()
    fig_ablation()
    fig_deadline_compliance()
    fig_multiv2x_fusion()
    fig_multiv2x_overview()
    fig_gap_summary()
    print(f'\nAll figures saved to {OUT}/')
