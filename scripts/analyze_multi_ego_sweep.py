#!/usr/bin/env python3
"""
Analyze multi-ego sweep results from 20260228_190907.
FOCAL EGO ONLY (lowest actor ID = hero).

Generates diagnostic plots:
  1. Ghost brakes vs ego count (anchoring on/off, per manager)
  2. Focal ego speed vs ego count (degradation indicator)
  3. Brake composition: stacked bars (ghost / FP / TP) — focal ego
  4. Provenance breakdown vs ego count — focal ego
  5. Anchoring effectiveness — ghost brake reduction
  6. Summary table (printed)
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import os
import glob
from collections import defaultdict

SWEEP_DIR = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/experiment_results/openscenario_3_edge_late_fusion/20260228_190907"
OUT = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/multi_ego_analysis"
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 8.5,
    'figure.dpi': 150,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.15,
})

CONFIGS = {
    'lf_off':   {'color': '#aec7e8', 'marker': 'v', 'ls': '--', 'label': 'Late Fusion (no anchoring)'},
    'lf_on':    {'color': '#1f77b4', 'marker': 'o', 'ls': '-',  'label': 'Late Fusion + Anchoring'},
    'vips_off': {'color': '#d62728', 'marker': '^', 'ls': '--', 'label': 'VIPS Temporal (no anchoring)'},
    'vips_on':  {'color': '#ff7f0e', 'marker': 'P', 'ls': '-',  'label': 'VIPS Temporal + Anchoring'},
}

MGR_MAP = {
    'lf_off':   ('late_fusion', 'off'),
    'lf_on':    ('late_fusion', 'on'),
    'vips_off': ('vips', 'off'),
    'vips_on':  ('vips', 'on'),
}


def load_focal_ego_runs():
    """Load all runs, extract focal ego (lowest actor ID) metrics only."""
    pattern = os.path.join(SWEEP_DIR, "**", "simulation_metrics.json")
    all_files = glob.glob(pattern, recursive=True)
    files = [f for f in all_files if "evaluation_output" not in f]

    runs = []
    for filepath in files:
        with open(filepath) as f:
            d = json.load(f)

        mgr = d.get("config_manager_type", "unknown")
        anch = "on" if d.get("config_anchoring", False) else "off"
        lat = int(d.get("config_latency_ms", 0))
        ego_count = d.get("config_ego_count", 1)

        vehicles = d.get('vehicles', {})
        if not vehicles:
            continue

        # Focal ego = lowest actor ID
        focal_vid = min(vehicles.keys(), key=lambda k: int(k))
        veh = vehicles[focal_vid]

        ghost_gt = veh.get('ghost_brake_gt', 0) or 0
        fp_gt = veh.get('other_fp_gt', 0) or 0
        tp_gt = veh.get('true_positive_gt', 0) or 0

        # Provenance
        prov = defaultdict(int)
        for a in veh.get('brake_attributions', []):
            prov[a.get('gt_provenance', 'unknown')] += 1

        run = {
            'manager': mgr,
            'anchoring': anch,
            'latency': lat,
            'ego_count': ego_count,
            'focal_vid': focal_vid,
            'ghost_gt': ghost_gt,
            'fp_gt': fp_gt,
            'tp_gt': tp_gt,
            'total_brakes': veh.get('total_brake_events', 0),
            'collision_count': veh.get('safety', {}).get('collision_count', 0),
            'stuck': veh.get('safety', {}).get('stuck', False),
            'avg_speed_mps': veh.get('avg_speed_mps', 0),
            'prov_self': prov.get('self', 0),
            'prov_cross_ego': prov.get('cross_ego', 0),
            'prov_parked': prov.get('parked', 0),
            'prov_npc_hazard': prov.get('npc_hazard', 0),
            'prov_npc_non_hazard': prov.get('npc_non_hazard', 0),
            'prov_unknown': prov.get('unknown', 0),
            'filepath': filepath,
        }

        # S metrics (focal ego)
        run['S_coll'] = 1 if run['collision_count'] == 0 else 0
        run['S_ghost'] = 1 if ghost_gt == 0 else 0
        run['S_fp'] = 1 if fp_gt == 0 else 0
        run['S_clear'] = 1 if (not run['stuck'] and run['collision_count'] == 0) else 0
        run['S_op'] = run['S_coll'] * run['S_ghost'] * run['S_fp'] * run['S_clear']

        runs.append(run)

    print(f"Loaded {len(runs)} focal-ego runs")
    return runs


def get_config_key(mgr, anch):
    if mgr == 'late_fusion':
        return f'lf_{anch}'
    return f'{mgr}_{anch}'


def _get_val(runs, mgr, anch, lat, ego_count, metric):
    """Get metric value for a specific config."""
    for r in runs:
        if (r['manager'] == mgr and r['anchoring'] == anch
                and r['latency'] == lat and r['ego_count'] == ego_count):
            return r.get(metric, 0)
    return None


def fig1_ghost_brakes_vs_ego(runs):
    """Ghost brake ticks (focal ego) vs ego count, split by latency."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))

    fig, axes = plt.subplots(1, len(latencies), figsize=(6*len(latencies), 5),
                             squeeze=False, sharey=True)

    for li, lat in enumerate(latencies):
        ax = axes[0][li]
        for ckey, style in CONFIGS.items():
            mgr, anch = MGR_MAP[ckey]
            vals, xs = [], []
            for ec in ego_counts:
                v = _get_val(runs, mgr, anch, lat, ec, 'ghost_gt')
                if v is not None:
                    vals.append(v)
                    xs.append(ec)
            if xs:
                ax.plot(xs, vals, marker=style['marker'], ls=style['ls'],
                        color=style['color'], label=style['label'],
                        linewidth=2, markersize=7)

        ax.set_xlabel('Number of Ego Vehicles')
        ax.set_title(f'Latency = {lat}ms')
        ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(x) for x in ego_counts])
        ax.grid(True, alpha=0.3)

    axes[0][0].set_ylabel('Ghost Brake Ticks (Focal Ego, GT-labeled)')
    axes[0][-1].legend(loc='upper left', fontsize=8)
    fig.suptitle('Focal Ego Ghost Brakes vs Ego Count', fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig1_focal_ghost_vs_ego.png'))
    plt.close(fig)
    print("  fig1: focal ego ghost brakes vs ego count")


def fig2_speed_vs_ego(runs):
    """Focal ego average speed vs ego count."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))

    fig, axes = plt.subplots(1, len(latencies), figsize=(6*len(latencies), 5),
                             squeeze=False, sharey=True)

    for li, lat in enumerate(latencies):
        ax = axes[0][li]
        for ckey, style in CONFIGS.items():
            mgr, anch = MGR_MAP[ckey]
            vals, xs = [], []
            for ec in ego_counts:
                v = _get_val(runs, mgr, anch, lat, ec, 'avg_speed_mps')
                if v is not None:
                    vals.append(v)
                    xs.append(ec)
            if xs:
                ax.plot(xs, vals, marker=style['marker'], ls=style['ls'],
                        color=style['color'], label=style['label'],
                        linewidth=2, markersize=7)

        ax.set_xlabel('Number of Ego Vehicles')
        ax.set_title(f'Latency = {lat}ms')
        ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(x) for x in ego_counts])
        ax.grid(True, alpha=0.3)
        ax.axhline(y=50/3.6, color='gray', ls=':', alpha=0.5,
                    label='Target 50 km/h' if li == 0 else None)

    axes[0][0].set_ylabel('Focal Ego Avg Speed (m/s)')
    axes[0][-1].legend(loc='upper right', fontsize=8)
    fig.suptitle('Focal Ego Speed vs Ego Count', fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig2_focal_speed_vs_ego.png'))
    plt.close(fig)
    print("  fig2: focal ego speed vs ego count")


def fig3_brake_composition(runs):
    """Stacked bar: focal ego ghost / FP / TP brake ticks per config."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))
    config_keys = list(CONFIGS.keys())

    fig, axes = plt.subplots(len(latencies), 1,
                             figsize=(max(14, len(ego_counts)*len(config_keys)*0.6),
                                      5*len(latencies)),
                             squeeze=False)

    for li, lat in enumerate(latencies):
        ax = axes[li][0]
        labels, ghost_vals, fp_vals, tp_vals = [], [], [], []

        for ec in ego_counts:
            for ckey in config_keys:
                mgr, anch = MGR_MAP[ckey]
                g = _get_val(runs, mgr, anch, lat, ec, 'ghost_gt')
                f = _get_val(runs, mgr, anch, lat, ec, 'fp_gt')
                t = _get_val(runs, mgr, anch, lat, ec, 'tp_gt')
                ghost_vals.append(g if g is not None else 0)
                fp_vals.append(f if f is not None else 0)
                tp_vals.append(t if t is not None else 0)
                short = ckey.replace('_', '\n')
                labels.append(f"N={ec}\n{short}")

        x = np.arange(len(labels))
        w = 0.6
        ax.bar(x, ghost_vals, w, label='Ghost (GT)', color='#e74c3c', alpha=0.85)
        ax.bar(x, fp_vals, w, bottom=ghost_vals, label='Other FP (GT)',
               color='#f39c12', alpha=0.85)
        ax.bar(x, tp_vals, w,
               bottom=np.array(ghost_vals)+np.array(fp_vals),
               label='True Positive (GT)', color='#2ecc71', alpha=0.85)

        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=6.5, rotation=0)
        ax.set_ylabel('Focal Ego Brake Ticks')
        ax.set_title(f'Focal Ego Brake Composition (Latency = {lat}ms)')
        ax.legend(fontsize=8)
        ax.grid(axis='y', alpha=0.3)

        for i in range(1, len(ego_counts)):
            ax.axvline(x=i*len(config_keys) - 0.5, color='gray',
                        ls='--', alpha=0.4)

    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig3_focal_brake_composition.png'))
    plt.close(fig)
    print("  fig3: focal ego brake composition")


def fig4_provenance(runs):
    """Provenance breakdown for focal ego brakes, late_fusion only."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))

    prov_keys = ['prov_self', 'prov_cross_ego', 'prov_npc_hazard',
                 'prov_parked', 'prov_npc_non_hazard', 'prov_unknown']
    prov_labels = ['Self-ghost', 'Cross-ego', 'NPC hazard',
                   'Parked', 'NPC non-hazard', 'Unknown']
    prov_colors = ['#e74c3c', '#9b59b6', '#2ecc71',
                   '#f39c12', '#3498db', '#95a5a6']

    fig, axes = plt.subplots(len(latencies), 2, figsize=(14, 5*len(latencies)),
                             squeeze=False)

    for li, lat in enumerate(latencies):
        for ai, anch in enumerate(['off', 'on']):
            ax = axes[li][ai]
            for eci, ec in enumerate(ego_counts):
                matching = [r for r in runs
                            if r['latency'] == lat and r['anchoring'] == anch
                            and r['ego_count'] == ec and r['manager'] == 'late_fusion']
                if not matching:
                    continue
                r = matching[0]
                bottom = 0
                for pi, pk in enumerate(prov_keys):
                    val = r[pk]
                    ax.bar(eci, val, bottom=bottom, color=prov_colors[pi],
                           label=prov_labels[pi] if ec == ego_counts[0] else None,
                           alpha=0.85, width=0.6)
                    bottom += val

            ax.set_xticks(range(len(ego_counts)))
            ax.set_xticklabels([str(ec) for ec in ego_counts])
            ax.set_xlabel('Number of Ego Vehicles')
            ax.set_ylabel('Focal Ego Brake Ticks by Provenance')
            ax.set_title(f'Late Fusion, Anchoring {"ON" if anch == "on" else "OFF"}, Lat={lat}ms')
            if li == 0 and ai == 0:
                ax.legend(fontsize=7, loc='upper left')
            ax.grid(axis='y', alpha=0.3)

    fig.suptitle('Focal Ego Brake Provenance (Late Fusion)', fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig4_focal_provenance.png'))
    plt.close(fig)
    print("  fig4: focal ego provenance")


def fig5_anchoring_delta(runs):
    """Anchoring effectiveness: ghost brake reduction (OFF - ON) for focal ego."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))
    managers = sorted(set(r['manager'] for r in runs if r['manager'] != 'oracle'))

    mgr_colors = {'late_fusion': '#1f77b4', 'vips': '#ff7f0e'}
    mgr_labels = {'late_fusion': 'Late Fusion', 'vips': 'VIPS Temporal'}

    fig, axes = plt.subplots(1, len(latencies), figsize=(6*len(latencies), 5),
                             squeeze=False, sharey=True)

    for li, lat in enumerate(latencies):
        ax = axes[0][li]
        for mgr in managers:
            reductions, xs = [], []
            for ec in ego_counts:
                off = _get_val(runs, mgr, 'off', lat, ec, 'ghost_gt')
                on = _get_val(runs, mgr, 'on', lat, ec, 'ghost_gt')
                if off is not None and on is not None:
                    reductions.append(off - on)
                    xs.append(ec)
            if xs:
                ax.plot(xs, reductions, marker='o', ls='-',
                        color=mgr_colors.get(mgr, 'gray'),
                        label=mgr_labels.get(mgr, mgr),
                        linewidth=2, markersize=7)

        ax.set_xlabel('Number of Ego Vehicles')
        ax.set_title(f'Latency = {lat}ms')
        ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(x) for x in ego_counts])
        ax.axhline(y=0, color='gray', ls=':', alpha=0.5)
        ax.grid(True, alpha=0.3)

    axes[0][0].set_ylabel('Focal Ego Ghost Reduction (OFF - ON)')
    axes[0][-1].legend(fontsize=8)
    fig.suptitle('Anchoring Effectiveness: Focal Ego Ghost Brake Reduction',
                 fontsize=14, y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig5_focal_anchoring_delta.png'))
    plt.close(fig)
    print("  fig5: anchoring effectiveness (focal ego)")


def fig6_s_ghost_heatmap(runs):
    """S_ghost heatmap: ego count × config."""
    latencies = sorted(set(r['latency'] for r in runs))
    ego_counts = sorted(set(r['ego_count'] for r in runs))
    config_keys = list(CONFIGS.keys())

    for lat in latencies:
        fig, ax = plt.subplots(figsize=(8, 4))
        data = np.full((len(config_keys), len(ego_counts)), np.nan)

        for ci, ckey in enumerate(config_keys):
            mgr, anch = MGR_MAP[ckey]
            for ei, ec in enumerate(ego_counts):
                v = _get_val(runs, mgr, anch, lat, ec, 'S_ghost')
                if v is not None:
                    data[ci, ei] = v

        im = ax.imshow(data, cmap='RdYlGn', vmin=0, vmax=1, aspect='auto')
        ax.set_xticks(range(len(ego_counts)))
        ax.set_xticklabels([str(ec) for ec in ego_counts])
        ax.set_yticks(range(len(config_keys)))
        ax.set_yticklabels([CONFIGS[k]['label'] for k in config_keys], fontsize=9)
        ax.set_xlabel('Number of Ego Vehicles')
        ax.set_title(f'S_ghost (Focal Ego) — Latency = {lat}ms')

        # Annotate cells
        for ci in range(len(config_keys)):
            for ei in range(len(ego_counts)):
                val = data[ci, ei]
                if not np.isnan(val):
                    # Also get ghost count
                    mgr, anch = MGR_MAP[config_keys[ci]]
                    gc = _get_val(runs, mgr, anch, lat, ego_counts[ei], 'ghost_gt')
                    color = 'white' if val < 0.5 else 'black'
                    txt = f"{'PASS' if val == 1 else 'FAIL'}\n({gc}g)"
                    ax.text(ei, ci, txt, ha='center', va='center',
                            fontsize=8, color=color, fontweight='bold')

        fig.colorbar(im, ax=ax, label='S_ghost (1=pass, 0=fail)')
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f'fig6_s_ghost_heatmap_lat{lat}.png'))
        plt.close(fig)

    print("  fig6: S_ghost heatmaps")


def print_summary(runs):
    """Print focal-ego summary table."""
    print("\n" + "="*145)
    print(f"{'Mgr':<14} {'Anch':<5} {'Lat':>4} {'Ego':>4} | "
          f"{'FocalID':>7} {'Ghost':>6} {'FP':>6} {'TP':>5} {'Brakes':>6} | "
          f"{'Coll':>4} {'Speed':>6} | "
          f"{'S_op':>4} {'S_coll':>6} {'S_ghost':>7} {'S_fp':>4} | "
          f"{'Self':>5} {'XEgo':>5} {'Park':>5} {'NPChz':>5}")
    print("-"*145)

    runs_sorted = sorted(runs, key=lambda r: (
        r['manager'], r['anchoring'], r['latency'], r['ego_count']))
    for r in runs_sorted:
        print(f"{r['manager']:<14} {r['anchoring']:<5} {r['latency']:>4} "
              f"{r['ego_count']:>4} | "
              f"{r['focal_vid']:>7} {r['ghost_gt']:>6} {r['fp_gt']:>6} "
              f"{r['tp_gt']:>5} {r['total_brakes']:>6} | "
              f"{r['collision_count']:>4} {r['avg_speed_mps']:>6.1f} | "
              f"{r['S_op']:>4} {r['S_coll']:>6} {r['S_ghost']:>7} "
              f"{r['S_fp']:>4} | "
              f"{r['prov_self']:>5} {r['prov_cross_ego']:>5} "
              f"{r['prov_parked']:>5} {r['prov_npc_hazard']:>5}")
    print("="*145)


if __name__ == '__main__':
    runs = load_focal_ego_runs()
    print_summary(runs)

    print("\nGenerating plots...")
    fig1_ghost_brakes_vs_ego(runs)
    fig2_speed_vs_ego(runs)
    fig3_brake_composition(runs)
    fig4_provenance(runs)
    fig5_anchoring_delta(runs)
    fig6_s_ghost_heatmap(runs)

    print(f"\nAll plots saved to: {OUT}/")
