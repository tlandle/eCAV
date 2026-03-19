#!/usr/bin/env python3
"""
Paper 1: Real-data plots from sweep 20260311_162505.

Generates figures matching the synthetic plot format using actual simulation
metrics. Format matches paper1_synthetic_plots.py exactly.

Usage:
    cd /home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
    python paper1_real_data_plots.py
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import os

# ── Style (match synthetic) ──────────────────────────────────────────
plt.rcParams.update({
    'font.size': 9, 'axes.labelsize': 9, 'axes.titlesize': 10,
    'legend.fontsize': 7, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.08,
    'lines.linewidth': 1.5, 'lines.markersize': 4,
})
FIG_1COL = (3.35, 2.2)
FIG_2COL = (7.0, 2.6)
FIG_2COL_TALL = (7.0, 5.0)

C = {
    'oracle_on':  '#2ca02c', 'oracle_off': '#98df8a',
    'lf_on':      '#1f77b4', 'lf_off':     '#aec7e8',
    'vips_on':    '#ff7f0e', 'vips_off':   '#d62728',
}
L = {
    'oracle_on': 'Oracle+SBA', 'oracle_off': 'Oracle',
    'lf_on': 'LF+SBA', 'lf_off': 'LF',
    'vips_on': 'VIPS+SBA', 'vips_off': 'VIPS',
}
MARKERS = {
    'oracle_on': 's', 'oracle_off': 'D',
    'lf_on': 'o', 'lf_off': 'v',
    'vips_on': 'P', 'vips_off': '^',
}
STYLES = {
    'oracle_on': '-', 'oracle_off': '--',
    'lf_on': '-', 'lf_off': '--',
    'vips_on': '-', 'vips_off': '--',
}

MANAGER_NAMES = ['oracle', 'late_fusion', 'vips']
MANAGER_LABELS = {'oracle': 'Oracle', 'late_fusion': 'Late Fusion', 'vips': 'VIPS'}

FAIL_COLORS = {
    'success':   '#2ca02c',
    'ghost':     '#d62728',
    'fp':        '#ff7f0e',
    'collision': '#1f77b4',
    'stuck':     '#9467bd',
}
FAIL_ORDER = ['success', 'ghost', 'fp', 'collision', 'stuck']
FAIL_LABELS = {
    'success': 'Success', 'ghost': 'Ghosting',
    'fp': 'Other FP', 'collision': 'Collision', 'stuck': 'Stuck',
}

OUT_REAL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'paper1_figures_real')
OUT_COMPARE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'paper1_figures_comparison')
os.makedirs(OUT_REAL, exist_ok=True)
os.makedirs(OUT_COMPARE, exist_ok=True)


def _key(mgr, anch):
    """Map (manager, anchoring) to style key."""
    m = 'oracle' if mgr == 'oracle' else ('lf' if mgr == 'late_fusion' else 'vips')
    return f'{m}_{anch}'


def _save(fig, name):
    """Save figure as PNG + PDF, then close."""
    fig.savefig(os.path.join(OUT_REAL, f'{name}.png'), dpi=150)
    fig.savefig(os.path.join(OUT_REAL, f'{name}.pdf'))
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════
# DATA LOADING
# ══════════════════════════════════════════════════════════════════════

from collections import defaultdict
import re

from recompute_metrics import compute_run_metrics


def load_sweep(sweep_dirs=None, focal_ego_only=True):
    """Load sweep data from one or more directories.

    Returns groups: dict of (mgr, anch, lat, ego_count) -> [list of run dicts].
    Each run dict contains recomputed focal-ego metrics plus derived fields
    expected by the plotting code.
    """
    if sweep_dirs is None:
        sweep_dirs = [os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'opencda/scenario_testing/evaluation_outputs/20260311_230618')]
    if isinstance(sweep_dirs, str):
        sweep_dirs = [sweep_dirs]

    groups = defaultdict(list)

    for base in sweep_dirs:
        if not os.path.isdir(base):
            print(f"  WARNING: sweep dir not found: {base}")
            continue
        for d in sorted(os.listdir(base)):
            path = os.path.join(base, d, 'simulation_metrics.json')
            if not os.path.exists(path):
                continue
            with open(path) as f:
                data = json.load(f)

            # Parse config from directory name
            anch = 'on' if 'anchoring_on' in d else 'off'
            lat_m = re.search(r'lat_(\d+)', d)
            lat = int(lat_m.group(1)) if lat_m else 0
            if 'mgr_oracle' in d:
                mgr = 'oracle'
            elif 'mgr_vips' in d:
                mgr = 'vips'
            else:
                mgr = 'late_fusion'
            ego_m = re.search(r'ego_(\d+)', d)
            ego_count = int(ego_m.group(1)) if ego_m else 1

            # Use recompute_metrics for focal-ego episode-based metrics
            rm = compute_run_metrics(data)
            if rm is None:
                continue

            # Focal vehicle data
            vehicles = data.get('vehicles', {})
            focal_vid = min(vehicles.keys(), key=lambda k: int(k)) if vehicles else None
            fv = vehicles.get(focal_vid, {}) if focal_vid else {}

            # Edge data
            ekeys = list(data.get('edges', {}).keys())
            e = ekeys and data['edges'][ekeys[0]] or {}

            # First failure class (priority: ghost > fp > collision > stuck)
            if rm['s_op'] == 1:
                first_fail = 'success'
            elif rm['s_ghost'] == 0:
                first_fail = 'ghost'
            elif rm['s_fp'] == 0:
                first_fail = 'fp'
            elif rm['s_coll'] == 0:
                first_fail = 'collision'
            else:
                first_fail = 'stuck'

            # Brake provenance fractions (using reclassified tick counts)
            ghost_gt = rm['focal_ghost_ticks']
            other_fp_gt = rm['focal_fp_ticks']
            tp_gt = rm['focal_tp_ticks']
            total_brakes = ghost_gt + other_fp_gt + tp_gt
            if total_brakes > 0:
                prov_ghost_frac = ghost_gt / total_brakes
                prov_tp_frac = tp_gt / total_brakes
                prov_fp_frac = other_fp_gt / total_brakes
                prov_unknown_frac = 0.0
            else:
                prov_ghost_frac = prov_tp_frac = prov_fp_frac = prov_unknown_frac = 0.0

            dist_m = fv.get('distance_traveled_m', 0)
            dist_km = max(dist_m, 1.0) / 1000.0

            run = {
                # Recomputed S_op components (episode-based, focal-only)
                'S_op': rm['s_op'], 'S_coll': rm['s_coll'],
                'S_ghost': rm['s_ghost'], 'S_fp': rm['s_fp'],
                'S_clear': rm['s_clear'],
                # Legacy names for compatibility
                's_op': rm['s_op'], 's_coll': rm['s_coll'],
                's_ghost': rm['s_ghost'], 's_fp': rm['s_fp'],
                's_prog': rm['s_clear'],
                'first_failure': first_fail,
                # Episode counts (focal ego)
                'ghost_episodes': rm['focal_ghost_episodes'],
                'fp_episodes': rm['focal_fp_episodes'],
                'tp_episodes': rm['focal_tp_episodes'],
                'total_episodes': rm['focal_total_episodes'],
                # Tick counts
                'ghost_gt': ghost_gt,
                'other_fp_gt': other_fp_gt,
                'tp_gt': tp_gt,
                'total_brakes': total_brakes,
                # Continuous
                'distance_m': dist_m,
                'dist_km': dist_km,
                'ghost_rate': rm['focal_ghost_episodes'] / dist_km if dist_km > 0 else 0,
                'fp_rate': rm['focal_fp_episodes'] / dist_km if dist_km > 0 else 0,
                'tp_rate': rm['focal_tp_episodes'] / dist_km if dist_km > 0 else 0,
                'avg_speed': rm.get('focal_avg_speed_mps', 0),
                'focal_collisions': rm['focal_collisions'],
                'collision_count': data.get('collision_count', 0),
                # AoI
                'aoi_p99_ms': rm.get('aoi_p99_ms'),
                'aoi_mean_ms': rm.get('aoi_mean_ms'),
                'aoi_p99': e.get('aoi_p99_ticks', 0),
                'aoi_mean': e.get('aoi_mean_ticks', 0),
                'aoi_hist_counts': e.get('aoi_hist_counts', []),
                'aoi_hist_bins': e.get('aoi_hist_bin_edges_ticks', []),
                'timing_avg_ms': (e.get('tracking_avg_ms', 0)
                                  + e.get('prediction_avg_ms', 0)
                                  + e.get('detection_avg_ms', 0)),
                # Provenance
                'prov_ghost_frac': prov_ghost_frac,
                'prov_tp_frac': prov_tp_frac,
                'prov_fp_frac': prov_fp_frac,
                'prov_unknown_frac': prov_unknown_frac,
                # Edge metrics
                'ego_uniqueness_total_violations': rm.get('ego_uniqueness_violations', 0),
                'ego_uniqueness_viol_frac': e.get('ego_uniqueness', {}).get(
                    'violation_tick_fraction', 0) if isinstance(
                    e.get('ego_uniqueness'), dict) else 0,
            }
            groups[(mgr, anch, lat, ego_count)].append(run)

    return groups


def get_latencies(groups):
    """Return sorted list of unique latencies in the data."""
    return sorted(set(k[2] for k in groups))


def get_ego_counts(groups):
    """Return sorted list of unique ego counts in the data."""
    return sorted(set(k[3] for k in groups))


LATENCIES = [0, 100, 200, 300, 400, 500, 600]


def get_series(runs, mgr, anch, field):
    vals = []
    for lat in LATENCIES:
        r = runs.get((mgr, anch, lat))
        vals.append(r[field] if r else np.nan)
    return np.array(vals, dtype=float)


def _has_manager(runs, mgr):
    """Check if we have any data for this manager."""
    return any(k[0] == mgr for k in runs)


# ══════════════════════════════════════════════════════════════════════
# FIGURE 1: S_op vs Configured Latency
# Synthetic format: 1x3 panels (Oracle, LF, VIPS), SBA on vs off per panel
# ══════════════════════════════════════════════════════════════════════

def fig1_sop_vs_latency(runs):
    available_mgrs = [m for m in MANAGER_NAMES if _has_manager(runs, m)]
    n_panels = len(available_mgrs)
    fig, axes = plt.subplots(1, n_panels, figsize=FIG_2COL, sharey=True)
    if n_panels == 1:
        axes = [axes]

    for i, mgr in enumerate(available_mgrs):
        ax = axes[i]
        for anch in ['on', 'off']:
            k = _key(mgr, anch)
            y = get_series(runs, mgr, anch, 's_op')
            if np.all(np.isnan(y)):
                continue
            ax.plot(LATENCIES, y, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], markersize=3)
        ax.set_title(MANAGER_LABELS[mgr], fontsize=9)
        ax.set_xlabel('Configured Latency (ms)')
        ax.set_xlim(0, 600)
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=6, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[S_{op}=1]$')
    fig.tight_layout()
    _save(fig, 'fig1_sop_vs_latency')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 2: Failure Decomposition
# Synthetic format: 1x4 panels (S_ghost, S_coll, S_fp, S_prog),
#   SBA on = solid, SBA off = dashed, same color per component.
#   Late Fusion as representative manager.
# ══════════════════════════════════════════════════════════════════════

def fig2_failure_decomposition(runs):
    metrics = ['s_ghost', 's_coll', 's_fp', 's_prog']
    metric_colors = {'s_coll': '#1f77b4', 's_ghost': '#d62728',
                     's_fp': '#ff7f0e', 's_prog': '#9467bd'}
    metric_titles = {'s_coll': r'$S_{coll}$ (Collision-Free)',
                     's_ghost': r'$S_{ghost}$ (Ghost-Free)',
                     's_fp': r'$S_{fp}$ (FP-Free)',
                     's_prog': r'$S_{prog}$ (Progress)'}

    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.4), sharey=True)
    mgr = 'late_fusion'

    for i, met in enumerate(metrics):
        ax = axes[i]
        col = metric_colors[met]
        for anch, ls, alpha, label in [
            ('on',  '-',  1.0, 'SBA ON'),
            ('off', '--', 0.7, 'SBA OFF'),
        ]:
            y = get_series(runs, mgr, anch, met)
            ax.plot(LATENCIES, y, ls=ls, color=col, alpha=alpha,
                    label=label, linewidth=1.5)
        ax.set_title(metric_titles[met], fontsize=8)
        ax.set_xlim(0, 600)
        ax.set_ylim(-0.05, 1.05)
        ax.set_xlabel('Latency (ms)', fontsize=7)
        ax.grid(alpha=0.2)
        if i == 0:
            ax.legend(fontsize=6, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[S=1]$')
    fig.tight_layout()
    _save(fig, 'fig2_failure_decomposition')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 3: First-Failure Stacked Bars
# Synthetic format: 3x2 grid (managers x SBA off/on), stacked bars per latency
# ══════════════════════════════════════════════════════════════════════

def fig3_first_failure_stacked(runs):
    available_mgrs = [m for m in MANAGER_NAMES if _has_manager(runs, m)]
    n_rows = len(available_mgrs)
    fig, axes = plt.subplots(n_rows, 2, figsize=(7.0, 1.8 * n_rows),
                             sharex=True, sharey=True)
    if n_rows == 1:
        axes = axes.reshape(1, -1)

    n_lats = len(LATENCIES)
    x = np.arange(n_lats)

    for row, mgr in enumerate(available_mgrs):
        for col, anch in enumerate(['off', 'on']):
            ax = axes[row, col]
            # For N=1 (single rep), each latency is one run
            fracs = {cat: np.zeros(n_lats) for cat in FAIL_ORDER}
            for li, lat in enumerate(LATENCIES):
                r = runs.get((mgr, anch, lat))
                if r is not None:
                    fracs[r['first_failure']][li] = 1.0

            bottoms = np.zeros(n_lats)
            for cat in FAIL_ORDER:
                ax.bar(x, fracs[cat], 0.7, bottom=bottoms,
                       color=FAIL_COLORS[cat], label=FAIL_LABELS[cat])
                bottoms += fracs[cat]

            ax.set_ylim(0, 1.05)
            ax.set_xlim(-0.5, n_lats - 0.5)
            ax.set_xticks(x[::2])
            ax.set_xticklabels([str(int(l)) for l in LATENCIES[::2]], fontsize=6)
            ax.grid(alpha=0.2, axis='y')

            if row == 0:
                ax.set_title('SBA OFF' if anch == 'off' else 'SBA ON', fontsize=9)
            if col == 0:
                ax.set_ylabel(f'{MANAGER_LABELS[mgr]}\nFraction', fontsize=8)
            if row == n_rows - 1:
                ax.set_xlabel('Configured Latency (ms)')
            if row == 0 and col == 0:
                ax.legend(fontsize=5, loc='upper right', ncol=2)

    fig.tight_layout()
    _save(fig, 'fig3_first_failure_stacked')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 4: Oracle Isolation
# Synthetic format: 1x2 panels (S_ghost, S_coll). Oracle on/off + LF/VIPS
#   off as dotted comparison lines.
# ══════════════════════════════════════════════════════════════════════

def fig4_oracle_isolation(runs):
    fig, axes = plt.subplots(1, 2, figsize=FIG_2COL, sharey=True)

    mgr = 'oracle'

    # Panel (a): S_ghost-free rate
    ax = axes[0]
    for anch in ['on', 'off']:
        k = _key(mgr, anch)
        y = get_series(runs, mgr, anch, 's_ghost')
        if not np.all(np.isnan(y)):
            ax.plot(LATENCIES, y, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], markersize=3)
    # Add multi-source managers for comparison (off only)
    for mgr_cmp in ['late_fusion', 'vips']:
        if _has_manager(runs, mgr_cmp):
            k = _key(mgr_cmp, 'off')
            y = get_series(runs, mgr_cmp, 'off', 's_ghost')
            if not np.all(np.isnan(y)):
                ax.plot(LATENCIES, y, ls=':', color=C[k], label=L[k],
                        alpha=0.6, linewidth=1.0)
    ax.set_title('(a) Self-Ghost-Free Rate', fontsize=9)
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel(r'$\Pr[S_{ghost}=1]$')
    ax.set_xlim(0, 600)
    ax.set_ylim(-0.05, 1.05)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=5, loc='lower left')

    # Panel (b): Collision-free rate
    ax = axes[1]
    for anch in ['on', 'off']:
        k = _key(mgr, anch)
        y = get_series(runs, mgr, anch, 's_coll')
        if not np.all(np.isnan(y)):
            ax.plot(LATENCIES, y, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], markersize=3)
    ax.set_title('(b) Collision-Free Rate', fontsize=9)
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_xlim(0, 600)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=6, loc='lower left')

    fig.tight_layout()
    _save(fig, 'fig4_oracle_isolation')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 5: AoI CDF (unchanged; will be replaced with multi-rep data)
# ══════════════════════════════════════════════════════════════════════

def fig5_aoi_cdf(runs):
    """AoI CDF from histogram data, one curve per latency."""
    fig, ax = plt.subplots(figsize=FIG_1COL)
    cmap = plt.cm.viridis
    lats_to_plot = [0, 200, 400, 600]
    for i, lat in enumerate(lats_to_plot):
        r = runs.get(('late_fusion', 'on', lat))
        if r is None or not r['aoi_hist_counts']:
            continue
        counts = np.array(r['aoi_hist_counts'], dtype=float)
        bins = np.array(r['aoi_hist_bins'], dtype=float)
        if counts.sum() == 0:
            continue
        bin_centers = (bins[:-1] + bins[1:]) / 2.0
        aoi_ms = bin_centers * 50.0
        cdf = np.cumsum(counts) / counts.sum()
        color = cmap(i / max(len(lats_to_plot) - 1, 1))
        ax.step(aoi_ms, cdf, where='mid', color=color, label=f'{lat} ms')

    ax.set_xlabel('AoI at Use (ms)')
    ax.set_ylabel('CDF')
    ax.set_ylim(0, 1.05)
    ax.legend(title='Configured\nLatency')
    ax.set_title('AoI Distribution')
    _save(fig, 'fig5_aoi_cdf')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 8: Brake Provenance
# Synthetic format: 1x2 panels (SBA OFF / SBA ON), stacked bars showing
#   brake source fractions (self-ghost, NPC hazard, parked, unknown).
#   Real data categories: self_ghost, true_positive (= NPC hazard),
#   other_fp (= parked/other), unknown (remainder).
# ══════════════════════════════════════════════════════════════════════

def fig8_brake_provenance(runs):
    fig, axes = plt.subplots(1, 2, figsize=FIG_2COL, sharey=True)

    prov_keys = ['ghost', 'tp', 'fp', 'unknown']
    prov_colors = {'ghost': '#d62728', 'tp': '#2ca02c',
                   'fp': '#ff7f0e', 'unknown': '#7f7f7f'}
    prov_labels = {'ghost': 'Self-Ghost', 'tp': 'NPC Hazard',
                   'fp': 'Other FP', 'unknown': 'Unknown'}

    mgr = 'late_fusion'
    n_lats = len(LATENCIES)
    x = np.arange(n_lats)

    for col, anch in enumerate(['off', 'on']):
        ax = axes[col]
        fracs = {pk: np.zeros(n_lats) for pk in prov_keys}

        for li, lat in enumerate(LATENCIES):
            r = runs.get((mgr, anch, lat))
            if r is not None:
                fracs['ghost'][li] = r['prov_ghost_frac']
                fracs['tp'][li] = r['prov_tp_frac']
                fracs['fp'][li] = r['prov_fp_frac']
                fracs['unknown'][li] = r['prov_unknown_frac']

        bottoms = np.zeros(n_lats)
        for pk in prov_keys:
            ax.bar(x, fracs[pk], 0.7, bottom=bottoms,
                   color=prov_colors[pk], label=prov_labels[pk])
            bottoms += fracs[pk]

        ax.set_title('SBA OFF' if anch == 'off' else 'SBA ON', fontsize=9)
        ax.set_xlabel('Configured Latency (ms)')
        ax.set_ylim(0, 1.05)
        ax.set_xlim(-0.5, n_lats - 0.5)
        ax.set_xticks(x[::2])
        ax.set_xticklabels([str(int(l)) for l in LATENCIES[::2]], fontsize=6)
        ax.grid(alpha=0.2, axis='y')

    axes[0].set_ylabel('Brake Source Fraction')
    axes[0].legend(fontsize=6, loc='upper left')

    fig.tight_layout()
    _save(fig, 'fig8_brake_provenance')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 9: Brake Episodes per km
# Synthetic format: single panel, 6 lines (ghost/fp/tp x SBA off/on).
#   SBA OFF = solid, SBA ON = dashed. Color by category.
# ══════════════════════════════════════════════════════════════════════

def fig9_brake_episodes_per_km(runs):
    fig, ax = plt.subplots(figsize=FIG_1COL)

    rate_keys = ['ghost_rate', 'fp_rate', 'tp_rate']
    rate_colors = {'ghost_rate': '#d62728', 'fp_rate': '#ff7f0e',
                   'tp_rate': '#2ca02c'}
    rate_labels = {'ghost_rate': 'Ghost', 'fp_rate': 'Other FP',
                   'tp_rate': 'True Pos.'}

    mgr = 'late_fusion'

    for rk in rate_keys:
        for anch, ls, alpha in [('off', '-', 1.0), ('on', '--', 0.7)]:
            y = get_series(runs, mgr, anch, rk)
            sba_tag = '' if anch == 'off' else ' +SBA'
            ax.plot(LATENCIES, y, color=rate_colors[rk], ls=ls,
                    alpha=alpha, label=f'{rate_labels[rk]}{sba_tag}',
                    linewidth=1.3)

    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('Brake Episodes / km')
    ax.set_xlim(0, 600)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=5, loc='upper left', ncol=2)
    fig.tight_layout()
    _save(fig, 'fig9_brake_episodes_per_km')


# ══════════════════════════════════════════════════════════════════════
# FIGURE 12: System Overhead (Edge Frame Time)
# Synthetic format: 2-panel (a) vs latency N=1 all managers, (b) vs N.
#   We only have N=1 data, so panel (b) is placeholder or omitted.
# ══════════════════════════════════════════════════════════════════════

def fig12_system_overhead(runs):
    available_mgrs = [m for m in MANAGER_NAMES if _has_manager(runs, m)]

    fig, ax = plt.subplots(figsize=FIG_2COL)

    for mgr in available_mgrs:
        for anch in ['on', 'off']:
            k = _key(mgr, anch)
            y = get_series(runs, mgr, anch, 'timing_avg_ms')
            if np.all(np.isnan(y)):
                continue
            ax.plot(LATENCIES, y, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], markersize=2)

    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('Edge Frame Time (ms)')
    ax.set_title('(a) vs Latency (N=1)', fontsize=9)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=5, loc='upper left', ncol=2)

    fig.tight_layout()
    _save(fig, 'fig12_system_overhead')


# ══════════════════════════════════════════════════════════════════════
# COMPARISON HELPER
# ══════════════════════════════════════════════════════════════════════

def make_comparison(fig_name, real_name=None):
    from PIL import Image, ImageDraw

    if real_name is None:
        real_name = fig_name

    synth_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              'paper1_figures_synthetic', f'{fig_name}.png')
    real_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'paper1_figures_real_aligned')
    real_path = os.path.join(real_dir, f'{real_name}.png')

    if not os.path.exists(synth_path):
        print(f"    Missing synthetic: {synth_path}")
        return
    if not os.path.exists(real_path):
        print(f"    Missing real: {real_path}")
        return

    synth = Image.open(synth_path)
    real = Image.open(real_path)

    target_h = max(synth.height, real.height)
    if synth.height != target_h:
        ratio = target_h / synth.height
        synth = synth.resize((int(synth.width * ratio), target_h), Image.LANCZOS)
    if real.height != target_h:
        ratio = target_h / real.height
        real = real.resize((int(real.width * ratio), target_h), Image.LANCZOS)

    gap = 20
    combined = Image.new('RGB', (synth.width + real.width + gap, target_h + 30), (255, 255, 255))
    combined.paste(synth, (0, 30))
    combined.paste(real, (synth.width + gap, 30))

    draw = ImageDraw.Draw(combined)
    draw.text((synth.width // 2 - 30, 5), "SYNTHETIC", fill='black')
    draw.text((synth.width + gap + real.width // 2 - 30, 5), "REAL DATA", fill='black')

    combined.save(os.path.join(OUT_COMPARE, f'{fig_name}_compare.png'))
    print(f"    {fig_name}_compare.png")


# ══════════════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════════════

def main():
    import sys as _sys
    compare_only = '--compare-only' in _sys.argv

    if not compare_only:
        print("Loading sweep data...")
        runs = load_sweep()
        print(f"  {len(runs)} runs loaded")

        mgrs_found = sorted(set(k[0] for k in runs))
        print(f"  Managers: {mgrs_found}")

        print("\nGenerating real-data figures...")
        fig1_sop_vs_latency(runs)
        print("  fig1_sop_vs_latency")
        fig2_failure_decomposition(runs)
        print("  fig2_failure_decomposition")
        fig3_first_failure_stacked(runs)
        print("  fig3_first_failure_stacked")
        fig4_oracle_isolation(runs)
        print("  fig4_oracle_isolation")
        fig5_aoi_cdf(runs)
        print("  fig5_aoi_cdf")
        fig8_brake_provenance(runs)
        print("  fig8_brake_provenance")
        fig9_brake_episodes_per_km(runs)
        print("  fig9_brake_episodes_per_km")
        fig12_system_overhead(runs)
        print("  fig12_system_overhead")

        print(f"\nReal figures saved to: {OUT_REAL}/")

    print("\nGenerating side-by-side comparisons...")
    # Mapping: (synthetic_name, real_aligned_name)
    # Real aligned single-ego figures have _ego1 suffix
    comparison_pairs = [
        ('fig1_sop_vs_latency',        'fig1_sop_vs_latency_ego1'),
        ('fig2_failure_decomposition', 'fig2_failure_decomposition_ego1'),
        ('fig3_first_failure_stacked', 'fig3_first_failure_stacked_ego1'),
        ('fig4_oracle_isolation',      'fig4_oracle_isolation_ego1'),
        ('fig5_aoi_cdf',              'fig5_aoi_cdf'),
        ('fig8_brake_provenance',     'fig8_brake_provenance_ego1'),
        ('fig9_brake_episodes_per_km', 'fig9_brake_episodes_per_km_ego1'),
        ('fig12_system_overhead',     'fig12_system_overhead_ego1'),
        ('fig6_multi_ego_scaling',    'fig6_multi_ego_scaling'),
        ('fig14_ego_uniqueness',      'fig14_ego_uniqueness'),
        ('fig16_envelope_boundary',   'fig16_envelope_boundary'),
    ]
    try:
        for synth_name, real_name in comparison_pairs:
            make_comparison(synth_name, real_name)
        print(f"Comparisons saved to: {OUT_COMPARE}/")
    except ImportError:
        print("  PIL not available, skipping comparisons")

    print("\nDone.")


if __name__ == '__main__':
    main()
