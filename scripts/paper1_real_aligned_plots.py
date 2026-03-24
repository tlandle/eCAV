#!/usr/bin/env python3
"""
Paper 1: Real-data plots aligned to the synthetic figure layout.

Generates the SAME figures as paper1_synthetic_plots.py but from real
experiment data. Uses load_sweep() from paper1_real_data_plots.py for
data loading (focal-ego-only, episode-based counting, post-hoc
reclassification).

Usage:
    python3 paper1_real_aligned_plots.py [sweep_dir1] [sweep_dir2] ...
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from collections import defaultdict

# Import data loader from real data plots script
sys.path.insert(0, os.path.dirname(__file__))
from paper1_real_data_plots import load_sweep, get_latencies, get_ego_counts

# ── Style constants (match synthetic script exactly) ─────────────────
plt.rcParams.update({
    'font.size': 9, 'axes.labelsize': 9, 'axes.titlesize': 10,
    'legend.fontsize': 7, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'figure.dpi': 150, 'savefig.bbox': 'tight', 'savefig.pad_inches': 0.08,
    'lines.linewidth': 1.5, 'lines.markersize': 4,
})

FIG_1COL = (3.35, 2.2)
FIG_2COL = (7.0, 2.6)

# ── Config key system (matches synthetic) ────────────────────────────
# Oracle is a single config (no on/off distinction).
MANAGER_NAMES = ['oracle', 'late_fusion', 'vips']
MANAGER_LABELS = {'oracle': 'Oracle', 'late_fusion': 'Late Fusion', 'vips': 'VIPS'}

C = {
    'oracle':     '#2ca02c',
    'lf_on':      '#1f77b4', 'lf_off':     '#aec7e8',
    'vips_on':    '#ff7f0e', 'vips_off':    '#d62728',
}
L = {
    'oracle': 'Oracle',
    'lf_on': 'LF+SBA',         'lf_off': 'LF',
    'vips_on': 'VIPS+SBA',     'vips_off': 'VIPS',
}
MARKERS = {
    'oracle': 's',
    'lf_on': 'o', 'lf_off': 'v',
    'vips_on': 'P', 'vips_off': '^',
}
STYLES = {
    'oracle': '-',
    'lf_on': '-', 'lf_off': '--',
    'vips_on': '-', 'vips_off': '--',
}
# Map key -> (manager, anchoring) for data lookup
MGR_MAP = {
    'oracle':     ('oracle', 'on'),
    'lf_on':      ('late_fusion', 'on'), 'lf_off': ('late_fusion', 'off'),
    'vips_on':    ('vips', 'on'),    'vips_off':   ('vips', 'off'),
}
# Order for iterating configs in plots
CONFIG_ORDER = ['oracle', 'lf_on', 'lf_off', 'vips_on', 'vips_off']

OUT = os.path.join(os.path.dirname(__file__), 'paper1_figures_real_aligned')
os.makedirs(OUT, exist_ok=True)


def _key(mgr, anch):
    if mgr == 'oracle':
        return 'oracle'
    prefix = {'late_fusion': 'lf', 'vips': 'vips'}
    return f"{prefix[mgr]}_{anch}"


def _get_mean_std(groups, mgr, anch, metric, latencies, ego_count=None):
    """Get mean and std for a metric across latencies from real data."""
    means, stds = [], []
    for lat in latencies:
        key = (mgr, anch, lat, ego_count)
        runs = groups.get(key, [])
        vals = [r.get(metric, 0) for r in runs]
        if vals:
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        else:
            means.append(np.nan)
            stds.append(0)
    return np.array(means), np.array(stds)


def _get_mean_std_by_ego(groups, mgr, anch, metric, ego_counts, lat):
    """Get mean and std for a metric across ego counts at a fixed latency."""
    means, stds = [], []
    for ec in ego_counts:
        key = (mgr, anch, lat, ec)
        runs = groups.get(key, [])
        vals = [r.get(metric, 0) for r in runs]
        if vals:
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        else:
            means.append(np.nan)
            stds.append(0)
    return np.array(means), np.array(stds)


def _save(fig, name):
    fig.savefig(f'{OUT}/{name}.pdf')
    fig.savefig(f'{OUT}/{name}.png', dpi=150)
    plt.close(fig)
    print(f"  Saved {name}")


# ═════════════════════════════════════════════════════════════════════
# FIGURE 1: S_op vs Latency (1×3 panels: Oracle, LF, VIPS)
# ═════════════════════════════════════════════════════════════════════
def fig1_sop_vs_latency(groups, ego_count=None):
    """1×3 panels (Oracle, LF, VIPS): SBA on vs off with CI bands."""
    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig1 (no data for ego_count={ego_count})")
        return

    fig, axes = plt.subplots(1, 3, figsize=FIG_2COL, sharey=True)

    for i, mgr in enumerate(MANAGER_NAMES):
        ax = axes[i]
        anchs = ['on'] if mgr == 'oracle' else ['on', 'off']
        for anch in anchs:
            k = _key(mgr, anch)
            means, stds = _get_mean_std(groups, mgr, anch, 'S_op', lats, ego_count)
            if np.all(np.isnan(means)):
                continue
            ax.plot(lats, means, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], markersize=3)
            ax.fill_between(lats, np.clip(means - stds, 0, 1),
                            np.clip(means + stds, 0, 1),
                            color=C[k], alpha=0.15)
        ax.set_title(MANAGER_LABELS[mgr], fontsize=9)
        ax.set_xlabel('Configured Latency (ms)')
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=6, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[S_{op}=1]$')
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig1_sop_vs_latency{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 2: Failure Decomposition — 1×4 panels, SBA overlaid, LF rep
# ═════════════════════════════════════════════════════════════════════
def fig2_failure_decomposition(groups, ego_count=None):
    """1×4 panels (S_ghost, S_coll, S_fp, S_clear): SBA on vs off overlaid."""
    metrics = ['S_ghost', 'S_coll', 'S_fp', 'S_clear']
    metric_colors = {'S_coll': '#1f77b4', 'S_ghost': '#d62728',
                     'S_fp': '#ff7f0e', 'S_clear': '#9467bd'}
    metric_titles = {
        'S_coll':  r'$S_{coll}$ (Collision-Free)',
        'S_ghost': r'$S_{ghost}$ (Ghost-Free)',
        'S_fp':    r'$S_{fp}$ (FP-Free)',
        'S_clear': r'$S_{clear}$ (Progress)',
    }

    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig2 (no data for ego_count={ego_count})")
        return

    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.4), sharey=True)
    mgr = 'late_fusion'

    for idx, metric in enumerate(metrics):
        ax = axes[idx]
        col = metric_colors[metric]
        for anch, ls, alpha, lbl_suffix in [('on', '-', 1.0, '+SBA'),
                                              ('off', '--', 0.7, '')]:
            means, stds = _get_mean_std(groups, mgr, anch, metric, lats, ego_count)
            if np.all(np.isnan(means)):
                continue
            ax.plot(lats, means, ls=ls, color=col, alpha=alpha,
                    label=f'LF{lbl_suffix}', marker='o', markersize=3)
            ax.fill_between(lats, np.clip(means - stds, 0, 1),
                            np.clip(means + stds, 0, 1),
                            color=col, alpha=0.1)
        ax.set_title(metric_titles[metric], fontsize=8)
        ax.set_xlabel('Latency (ms)', fontsize=7)
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2)
        if idx == 0:
            ax.legend(fontsize=6, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[S=1]$')
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig2_failure_decomposition{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 3: First-Failure Stacked Bars — 3×2 grid (mgr × SBA)
# ═════════════════════════════════════════════════════════════════════
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

def fig3_first_failure_stacked(groups, ego_count=None):
    """3×2 grid: stacked bars per latency showing first-failure class.
    Oracle row shows single panel (no on/off distinction)."""
    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig3 (no data for ego_count={ego_count})")
        return

    fig, axes = plt.subplots(3, 2, figsize=(7.0, 5.0), sharex=True, sharey=True)
    n_lats = len(lats)
    x = np.arange(n_lats)

    for row, mgr in enumerate(MANAGER_NAMES):
        for col, anch in enumerate(['off', 'on']):
            ax = axes[row, col]

            if mgr == 'oracle' and anch == 'off':
                ax.set_visible(False)
                continue

            actual_anch = 'on' if mgr == 'oracle' else anch

            # Compute first-failure fractions across runs at each latency
            fracs = {cat: np.zeros(n_lats) for cat in FAIL_ORDER}
            for li, lat in enumerate(lats):
                key = (mgr, actual_anch, lat, ego_count)
                runs = groups.get(key, [])
                if not runs:
                    continue
                for r in runs:
                    ff = r.get('first_failure', 'stuck')
                    if ff in fracs:
                        fracs[ff][li] += 1.0
                total = sum(fracs[cat][li] for cat in FAIL_ORDER)
                if total > 0:
                    for cat in FAIL_ORDER:
                        fracs[cat][li] /= total

            bottoms = np.zeros(n_lats)
            for cat in FAIL_ORDER:
                ax.bar(x, fracs[cat], 0.7, bottom=bottoms,
                       color=FAIL_COLORS[cat], label=FAIL_LABELS[cat])
                bottoms += fracs[cat]

            ax.set_ylim(0, 1.05)
            ax.set_xlim(-0.5, n_lats - 0.5)
            ax.set_xticks(x)
            ax.set_xticklabels([str(int(l)) for l in lats],
                               fontsize=5, rotation=45, ha='right')
            ax.grid(alpha=0.2, axis='y')

            if row == 0:
                title = 'Oracle' if mgr == 'oracle' else (
                    'SBA OFF' if anch == 'off' else 'SBA ON')
                ax.set_title(title, fontsize=9)
            if col == 0:
                ax.set_ylabel(f'{MANAGER_LABELS[mgr]}\nFraction', fontsize=8)
            if row == 2:
                ax.set_xlabel('Configured Latency (ms)')
            if row == 0 and col == 1:
                ax.legend(fontsize=5, loc='upper right', ncol=2)

    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig3_first_failure_stacked{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 4: Oracle Isolation — 1×2 panels (S_ghost, S_coll)
# ═════════════════════════════════════════════════════════════════════
def fig4_oracle_isolation(groups, ego_count=None):
    """Oracle vs LF: S_ghost and S_coll side by side."""
    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig4 (no data for ego_count={ego_count})")
        return

    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.6), sharey=True)
    metrics = ['S_ghost', 'S_coll']
    titles = [r'$S_{ghost}$ (Ghost-Free)', r'$S_{coll}$ (Collision-Free)']

    for ax, metric, title in zip(axes, metrics, titles):
        for k_name in ['oracle', 'lf_off', 'lf_on']:
            mgr, anch = MGR_MAP[k_name]
            means, stds = _get_mean_std(groups, mgr, anch, metric, lats, ego_count)
            if np.all(np.isnan(means)):
                continue
            ax.plot(lats, means, marker=MARKERS[k_name], ls=STYLES[k_name],
                    color=C[k_name], label=L[k_name], markersize=3)
            ax.fill_between(lats, np.clip(means - stds, 0, 1),
                            np.clip(means + stds, 0, 1),
                            color=C[k_name], alpha=0.1)
        ax.set_title(title, fontsize=9)
        ax.set_xlabel('Latency (ms)')
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=6, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[\cdot = 1]$')
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig4_oracle_isolation{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 5: AoI CDF — color by latency, SBA on/off as line style
# ═════════════════════════════════════════════════════════════════════
def fig5_aoi_cdf(groups, ego_count=None):
    """AoI CDF colored by latency, SBA on vs off as solid/dashed."""
    mgr = 'late_fusion'
    lats = sorted(set(k[2] for k in groups
                      if k[0] == mgr and k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig5 (no AoI data for ego_count={ego_count})")
        return

    # Pick a subset of latencies for readability
    lat_subset = [l for l in [0, 100, 200, 300, 400, 500] if l in lats]
    if not lat_subset:
        lat_subset = lats[:6]

    fig, ax = plt.subplots(figsize=(5, 3.5))
    cmap = plt.cm.viridis
    norm = plt.Normalize(vmin=min(lat_subset), vmax=max(lat_subset))

    for lat in lat_subset:
        c = cmap(norm(lat))
        for anch, ls, lw in [('on', '-', 1.2), ('off', '--', 1.0)]:
            key = (mgr, anch, lat, ego_count)
            runs = groups.get(key, [])
            # Collect all AoI values from aoi_hist
            aoi_vals = []
            for r in runs:
                bins = r.get('aoi_hist_bins', [])
                counts = r.get('aoi_hist_counts', [])
                if bins and counts and len(bins) >= 2:
                    # Reconstruct samples from histogram
                    for i, cnt in enumerate(counts):
                        if cnt > 0 and i < len(bins) - 1:
                            mid = (bins[i] + bins[i+1]) / 2.0 * 50  # ticks to ms
                            aoi_vals.extend([mid] * int(cnt))
                # Also try per-run p99
                p99 = r.get('aoi_p99_ms')
                if p99 is not None and p99 > 0 and not aoi_vals:
                    aoi_vals.append(p99)

            if not aoi_vals:
                continue
            sorted_aoi = np.sort(aoi_vals)
            cdf = np.arange(1, len(sorted_aoi) + 1) / len(sorted_aoi)
            label = f'{int(lat)} ms' if anch == 'on' else None
            ax.plot(sorted_aoi, cdf, color=c, ls=ls, linewidth=lw,
                    label=label)

    # Legend for line styles
    from matplotlib.lines import Line2D
    custom = [Line2D([0], [0], color='k', ls='-', lw=1.2),
              Line2D([0], [0], color='k', ls='--', lw=1.0)]
    leg1 = ax.legend(fontsize=6, loc='lower right', ncol=2,
                     title='Configured Latency', title_fontsize=6)
    ax.add_artist(leg1)
    ax.legend(custom, ['SBA ON', 'SBA OFF'], fontsize=6, loc='center right')

    ax.set_xlabel('Age of Information at Use (ms)')
    ax.set_ylabel('CDF')
    ax.set_ylim(0, 1.02)
    ax.grid(alpha=0.2)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax, label='Configured Latency (ms)', pad=0.02)
    cbar.ax.tick_params(labelsize=7)

    fig.tight_layout()
    _save(fig, 'fig5_aoi_cdf')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 6: Multi-Ego Scaling — 1×3 panels at 0, 200, 400 ms
# ═════════════════════════════════════════════════════════════════════
def fig6_multi_ego_scaling(groups):
    """1×3 panels by latency (0, 200, 400). X=N_ego, SBA on/off overlaid."""
    ego_counts = sorted(set(k[3] for k in groups if k[3] is not None))
    if len(ego_counts) < 2:
        print("  Skipping fig6 (need multi-ego data)")
        return

    # Use the multi-ego latencies that exist in data
    multi_lats_available = sorted(set(
        k[2] for k in groups
        if k[3] is not None and k[3] > 1
    ))
    # Prefer 0, 200, 400 to match synthetic
    rep_lats = [l for l in [0, 200, 400] if l in multi_lats_available]
    if not rep_lats:
        rep_lats = multi_lats_available[:3]

    fig, axes = plt.subplots(1, len(rep_lats), figsize=FIG_2COL, sharey=True)
    if len(rep_lats) == 1:
        axes = [axes]

    mgr = 'late_fusion'

    # Match synthetic: S_op=blue, S_ghost=red; solid=SBA ON, dashed=SBA OFF
    met_cfg = [
        ('S_op',    'o', r'$S_{op}$',    '#1f77b4'),
        ('S_ghost', 's', r'$S_{ghost}$', '#d62728'),
    ]

    for ax, lat in zip(axes, rep_lats):
        for metric, marker, label_base, met_color in met_cfg:
            for anch, ls, alpha, suffix in [('on', '-', 1.0, '+SBA'),
                                             ('off', '--', 0.65, '')]:
                means, stds = _get_mean_std_by_ego(groups, mgr, anch, metric,
                                                     ego_counts, lat)
                if np.all(np.isnan(means)):
                    continue
                lbl = f'{label_base}{suffix}'
                ax.plot(ego_counts, means, marker=marker, ls=ls,
                        color=met_color, alpha=alpha, label=lbl, markersize=4,
                        linewidth=1.5)
                ax.fill_between(ego_counts,
                                np.clip(means - stds, 0, 1),
                                np.clip(means + stds, 0, 1),
                                color=met_color, alpha=0.08 * alpha)

        ax.set_title(f'{int(lat)} ms', fontsize=9)
        ax.set_xlabel('Ego Count (N)')
        ax.set_xticks(ego_counts)
        ax.set_ylim(-0.05, 1.05)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=5, loc='lower left')

    axes[0].set_ylabel(r'$\Pr[\cdot = 1]$')
    fig.tight_layout()
    _save(fig, 'fig6_multi_ego_scaling')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 8: Brake Provenance — stacked bars using fraction fields
# ═════════════════════════════════════════════════════════════════════
def fig8_brake_provenance(groups, ego_count=None):
    """1×2 panels (off/on): stacked bars showing brake source breakdown.
    3 categories: Self Ghost, True Positive (NPC Hazard), Other FP (FP+Unknown)."""
    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig8 (no data for ego_count={ego_count})")
        return

    mgr = 'late_fusion'
    prov_keys = ['self_ghost', 'true_positive', 'other_fp']
    prov_colors = {'self_ghost': '#d62728', 'true_positive': '#2ca02c',
                   'other_fp': '#ff7f0e'}
    prov_labels = {'self_ghost': 'Self-Ghost', 'true_positive': 'True Positive',
                   'other_fp': 'Other FP'}

    fig, axes = plt.subplots(1, 2, figsize=FIG_2COL, sharey=True)
    n_lats = len(lats)
    x = np.arange(n_lats)

    for col, (anch, title) in enumerate([('off', 'SBA Off'), ('on', 'SBA On')]):
        ax = axes[col]
        fracs = {pk: np.zeros(n_lats) for pk in prov_keys}

        for li, lat in enumerate(lats):
            key = (mgr, anch, lat, ego_count)
            runs = groups.get(key, [])
            if runs:
                fracs['self_ghost'][li] = np.mean(
                    [r.get('prov_ghost_frac', 0) for r in runs])
                fracs['true_positive'][li] = np.mean(
                    [r.get('prov_tp_frac', 0) for r in runs])
                fracs['other_fp'][li] = np.mean(
                    [r.get('prov_fp_frac', 0) + r.get('prov_unknown_frac', 0)
                     for r in runs])

        bottoms = np.zeros(n_lats)
        for pk in prov_keys:
            vals = fracs[pk]
            if np.any(vals > 0):
                ax.bar(x, vals, 0.7, bottom=bottoms,
                       color=prov_colors[pk], label=prov_labels[pk])
                bottoms += vals

        ax.set_title(title, fontsize=9)
        ax.set_xlabel('Configured Latency (ms)')
        ax.set_ylim(0, 1.05)
        ax.set_xlim(-0.5, n_lats - 0.5)
        ax.set_xticks(x)
        ax.set_xticklabels([str(int(l)) for l in lats],
                           fontsize=5, rotation=45, ha='right')
        ax.grid(alpha=0.2, axis='y')

    axes[0].set_ylabel('Brake Source Fraction')
    axes[0].legend(fontsize=6, loc='upper left')
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig8_brake_provenance{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 9: Brake Episodes Per km — using ghost_rate, fp_rate, tp_rate
# ═════════════════════════════════════════════════════════════════════
def fig9_brake_episodes_per_km(groups, ego_count=None):
    """Single panel: brake rate by category, SBA on/off overlaid."""
    lats = sorted(set(k[2] for k in groups if k[3] == ego_count))
    if not lats:
        print(f"  Skipping fig9 (no data for ego_count={ego_count})")
        return

    fig, ax = plt.subplots(figsize=FIG_1COL)
    mgr = 'late_fusion'

    rate_keys = ['ghost_rate', 'fp_rate', 'tp_rate']
    rate_colors = {'ghost_rate': '#d62728', 'fp_rate': '#ff7f0e',
                   'tp_rate': '#2ca02c'}
    rate_labels = {'ghost_rate': 'Ghost', 'fp_rate': 'Other FP',
                   'tp_rate': 'True Pos.'}

    for rk in rate_keys:
        for anch, ls, alpha in [('off', '-', 1.0), ('on', '--', 0.7)]:
            means, stds = _get_mean_std(groups, mgr, anch, rk, lats, ego_count)
            if np.all(np.isnan(means)):
                continue
            sba_tag = '' if anch == 'off' else ' +SBA'
            ax.plot(lats, means, color=rate_colors[rk], ls=ls,
                    alpha=alpha, label=f'{rate_labels[rk]}{sba_tag}',
                    linewidth=1.3)

    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('Brake Episodes / km')
    ax.grid(alpha=0.2)
    ax.legend(fontsize=5, loc='upper left', ncol=2)
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig9_brake_episodes_per_km{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 12: System Overhead — edge frame time vs N_ego
# ═════════════════════════════════════════════════════════════════════
def fig12_system_overhead(groups, ego_count=None):
    """Edge frame time vs N_ego, all managers. Averages across latencies."""
    ego_counts = sorted(set(k[3] for k in groups if k[3] is not None))
    if not ego_counts:
        print(f"  Skipping fig12 (no ego count data)")
        return

    fig, ax = plt.subplots(figsize=FIG_1COL)

    for k_name in ['oracle', 'lf_on', 'vips_on']:
        mgr, anch = MGR_MAP[k_name]
        means = []
        for ec in ego_counts:
            vals = []
            for lat in sorted(set(k[2] for k in groups)):
                key = (mgr, anch, lat, ec)
                for r in groups.get(key, []):
                    v = r.get('timing_avg_ms', 0)
                    if v > 0:
                        vals.append(v)
            means.append(np.mean(vals) if vals else np.nan)
        if all(np.isnan(m) for m in means):
            continue
        ax.plot(ego_counts, means, marker=MARKERS[k_name],
                ls=STYLES[k_name], color=C[k_name],
                label=L[k_name], markersize=5)

    if len(ego_counts) > 3:
        ax.set_xscale('log', base=2)
    ax.set_xticks(ego_counts)
    ax.set_xticklabels([str(n) for n in ego_counts])
    ax.set_xlabel(r'Number of Ego Vehicles ($N$)')
    ax.set_ylabel('Edge Frame Time (ms)')
    ax.grid(alpha=0.2)
    ax.legend(fontsize=6)
    fig.tight_layout()
    suffix = f'_ego{ego_count}' if ego_count is not None else ''
    _save(fig, f'fig12_system_overhead{suffix}')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 14: Ego Uniqueness Violations vs N
# ═════════════════════════════════════════════════════════════════════
def fig14_ego_uniqueness(groups):
    """Ego-uniqueness violation rate vs ego count. SBA on/off overlaid.
    Matches synthetic plot structure: LF, LF+SBA, Oracle."""
    ego_counts = sorted(set(k[3] for k in groups if k[3] is not None))
    if len(ego_counts) < 2:
        print("  Skipping fig14 (need multi-ego data)")
        return

    fig, ax = plt.subplots(figsize=FIG_1COL)

    for mgr in ['late_fusion', 'oracle']:
        anchs = ['on'] if mgr == 'oracle' else ['on', 'off']
        for anch in anchs:
            k = _key(mgr, anch)
            ls = STYLES[k]
            alpha = 0.65 if anch == 'off' else 1.0
            means = []
            for ec in ego_counts:
                vals = []
                for lat in sorted(set(k2[2] for k2 in groups)):
                    key = (mgr, anch, lat, ec)
                    for r in groups.get(key, []):
                        v = r.get('ego_uniqueness_viol_frac', 0)
                        vals.append(v)
                means.append(np.mean(vals) if vals else np.nan)
            sba_tag = '+SBA' if anch == 'on' and mgr != 'oracle' else ''
            ax.plot(ego_counts, means, marker='o', ls=ls,
                    color=C[k], alpha=alpha, label=f'{L[k]}{sba_tag}',
                    markersize=4, linewidth=1.5)

    ax.set_xlabel(r'Number of Ego Vehicles ($N$)')
    ax.set_ylabel('Ego-Uniq. Violation Rate')
    ax.set_xticks(ego_counts)
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=6)
    fig.tight_layout()
    _save(fig, 'fig14_ego_uniqueness')


# ═════════════════════════════════════════════════════════════════════
# FIGURE 16: Safety Envelope Boundary — max latency for P(S_op)≥0.8
# ═════════════════════════════════════════════════════════════════════
def fig16_envelope_boundary(groups):
    """Max latency where P(S_op=1) >= 0.8, vs ego count. All managers."""
    ego_counts = sorted(set(k[3] for k in groups if k[3] is not None))
    if not ego_counts:
        print("  Skipping fig16 (no data)")
        return

    fig, ax = plt.subplots(figsize=FIG_1COL)
    threshold = 0.8

    for k_name in CONFIG_ORDER:
        mgr, anch = MGR_MAP[k_name]
        max_lats = []
        for ec in ego_counts:
            lats = sorted(set(k[2] for k in groups
                              if k[0] == mgr and k[1] == anch and k[3] == ec))
            best_lat = 0
            for lat in lats:
                key = (mgr, anch, lat, ec)
                runs = groups.get(key, [])
                if runs:
                    p_sop = np.mean([r.get('S_op', 0) for r in runs])
                    if p_sop >= threshold:
                        best_lat = lat
            max_lats.append(best_lat)

        if not max_lats or all(v == 0 for v in max_lats):
            continue
        ax.plot(ego_counts, max_lats, marker=MARKERS[k_name],
                ls=STYLES[k_name], color=C[k_name], label=L[k_name],
                markersize=5, linewidth=1.8)

    ax.set_xlabel(r'Number of Ego Vehicles ($N$)')
    ax.set_ylabel(r'Max Latency for $P(S_{op}) \geq 0.8$ (ms)')
    if len(ego_counts) > 3:
        ax.set_xscale('log', base=2)
    ax.set_xticks(ego_counts)
    ax.set_xticklabels([str(n) for n in ego_counts])
    ax.set_ylim(bottom=0)
    ax.grid(alpha=0.2)
    ax.legend(fontsize=6, loc='lower left')
    fig.tight_layout()
    _save(fig, 'fig16_envelope_boundary')


# ═════════════════════════════════════════════════════════════════════
# SUMMARY TABLE (printed to console)
# ═════════════════════════════════════════════════════════════════════
def print_summary(groups):
    """Print data coverage summary."""
    managers = sorted(set(k[0] for k in groups))
    anchs = sorted(set(k[1] for k in groups))
    lats = sorted(set(k[2] for k in groups))
    egos = sorted(set(k[3] for k in groups))
    n_runs = sum(len(v) for v in groups.values())

    print(f"\n{'='*70}")
    print(f"Real Data Summary")
    print(f"{'='*70}")
    print(f"  Total runs:      {n_runs}")
    print(f"  Managers:        {managers}")
    print(f"  Anchoring:       {anchs}")
    print(f"  Latencies:       {lats}")
    print(f"  Ego counts:      {egos}")
    print(f"  Configs:         {len(groups)}")
    print(f"{'='*70}\n")

    # Per-config summary
    print(f"{'Manager':<14} {'Anch':>4} {'Ego':>4} {'Lat':>5} {'N':>3} | "
          f"{'S_op':>5} {'S_coll':>6} {'S_ghst':>6} {'S_fp':>5} {'S_clr':>5} | "
          f"{'Ghost':>5} {'FP':>4}")
    print('-' * 90)
    for key in sorted(groups.keys(), key=lambda k: (k[0], k[1], k[3] or 0, k[2])):
        mgr, anch, lat, ec = key
        runs = groups[key]
        n = len(runs)
        s_op = np.mean([r.get('S_op', 0) for r in runs]) * 100
        s_coll = np.mean([r.get('S_coll', 0) for r in runs]) * 100
        s_ghost = np.mean([r.get('S_ghost', 0) for r in runs]) * 100
        s_fp = np.mean([r.get('S_fp', 0) for r in runs]) * 100
        s_clear = np.mean([r.get('S_clear', 0) for r in runs]) * 100
        ghost = np.mean([r.get('ghost_episodes', 0) for r in runs])
        fp = np.mean([r.get('fp_episodes', 0) for r in runs])
        print(f'{mgr:<14} {anch:>4} {ec or 1:>4} {lat:>5.0f} {n:>3} | '
              f'{s_op:>4.0f}% {s_coll:>5.0f}% {s_ghost:>5.0f}% {s_fp:>4.0f}% {s_clear:>4.0f}% | '
              f'{ghost:>5.1f} {fp:>4.1f}')


# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    EVAL_BASE = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/ecav/scenario_testing/evaluation_outputs"

    # Default sweep directories (March 2026 sweep)
    SWEEP_DIRS = [
        os.path.join(EVAL_BASE, "20260311_230618"),  # LF single-ego (both anch)
        os.path.join(EVAL_BASE, "20260312_011249"),  # Oracle single-ego
        os.path.join(EVAL_BASE, "20260312_022727"),  # VIPS single-ego (both anch)
        os.path.join(EVAL_BASE, "20260312_044701"),  # LF multi-ego (both anch)
        os.path.join(EVAL_BASE, "20260312_075240"),  # Oracle multi-ego
        os.path.join(EVAL_BASE, "20260312_093013"),  # VIPS multi-ego (both anch)
    ]

    if len(sys.argv) > 1:
        sweep_dirs = sys.argv[1:]
    else:
        sweep_dirs = [d for d in SWEEP_DIRS if os.path.isdir(d)]

    print(f"Loading from {len(sweep_dirs)} sweep dir(s)...")
    for sd in sweep_dirs:
        print(f"  {sd}")

    groups = load_sweep(sweep_dirs, focal_ego_only=True)
    print_summary(groups)

    # Detect ego counts
    ego_counts = sorted(set(k[3] for k in groups))
    single_ego = ego_counts[0] if ego_counts else None  # None or 1

    # ── Single-ego figures ──
    print("\n── Single-ego figures ──")
    fig1_sop_vs_latency(groups, ego_count=single_ego)
    fig2_failure_decomposition(groups, ego_count=single_ego)
    fig3_first_failure_stacked(groups, ego_count=single_ego)
    fig4_oracle_isolation(groups, ego_count=single_ego)
    fig5_aoi_cdf(groups, ego_count=single_ego)
    fig8_brake_provenance(groups, ego_count=single_ego)
    fig9_brake_episodes_per_km(groups, ego_count=single_ego)
    fig12_system_overhead(groups, ego_count=single_ego)

    # ── Multi-ego figures ──
    if len(ego_counts) > 1:
        print("\n── Multi-ego figures ──")
        fig6_multi_ego_scaling(groups)
        fig14_ego_uniqueness(groups)
        fig16_envelope_boundary(groups)

    print(f"\nAll figures saved to {OUT}/")
