#!/usr/bin/env python3
"""
Paper 1: Real-data plots from experiment sweeps.

Loads simulation_metrics.json from each run, groups by
(manager_type, anchoring, latency), computes means/stds,
and generates publication-quality figures.

OPERATIONAL SUCCESS (4-tier, focal ego only):
  S_coll  = 1[no collision]                          (safety)
  S_ghost = 1[no self-ghost brake EPISODES]           (targeted failure class)
  S_fp    = 1[no other false-brake EPISODES]          (stack hygiene)
  S_clear = 1[not stuck AND no collision]             (task completion)
  S_op    = S_coll AND S_ghost AND S_fp AND S_clear

Brake episodes: consecutive ticks (gap <= 2) with same track_id and GT class
count as a single episode. This prevents RSS latch hold (5 ticks) from being
counted as 5 separate events.

Focal ego: lowest actor ID per run (ScenarioRunner hero, first spawned).
Multi-ego metrics are computed for focal ego only by default.

GT brake classification (from edge_manager_base):
  self_ghost:     GT matched actor = ego
  other_fp:       GT matched actor != ego AND not a GT hazard
  true_positive:  GT matched actor IS a GT hazard
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import json
import os
import glob
from collections import defaultdict

# ── Style constants ──────────────────────────────────────────────────
plt.rcParams.update({
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 8.5,
    'figure.dpi': 150,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.15,
})

# 6 configs: (manager, anchoring) → style key
C = {
    'oracle_on':  '#2ca02c',
    'oracle_off': '#98df8a',
    'lf_on':      '#1f77b4',
    'lf_off':     '#aec7e8',
    'vips_on':    '#ff7f0e',
    'vips_off':   '#d62728',
}
L = {
    'oracle_on':  'Oracle + Anchoring',
    'oracle_off': 'Oracle (no anchoring)',
    'lf_on':      'Late Fusion + Anchoring',
    'lf_off':     'Late Fusion (no anchoring)',
    'vips_on':    'VIPS Temporal + Anchoring',
    'vips_off':   'VIPS Temporal (no anchoring)',
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

SWEEP_DIR = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/experiment_results/openscenario_3_edge_late_fusion/20260222_231459"
OUT = '/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/paper1_figures'
os.makedirs(OUT, exist_ok=True)

MGR_MAP = {
    'oracle_on': ('oracle', 'on'), 'oracle_off': ('oracle', 'off'),
    'lf_on': ('late_fusion', 'on'), 'lf_off': ('late_fusion', 'off'),
    'vips_on': ('vips', 'on'), 'vips_off': ('vips', 'off'),
}

ALL_KEYS = ['oracle_on', 'oracle_off', 'lf_on', 'lf_off', 'vips_on', 'vips_off']


# ── Episode counting ──────────────────────────────────────────────────

def _count_episodes(brake_attributions, gap=2):
    """Count brake episodes by GT class.

    An episode = contiguous ticks (gap <= `gap`) with the same track_id
    and same gt_brake_class. Returns (ghost_episodes, fp_episodes, tp_episodes).
    """
    by_class = defaultdict(list)
    for a in brake_attributions:
        cls = a.get('gt_brake_class', 'unknown')
        tick = a.get('trigger_tick')
        track = a.get('trigger_track_id')
        if tick is not None:
            by_class[cls].append((tick, track))

    def count_eps(events):
        if not events:
            return 0
        events.sort()
        eps = 1
        prev_tick, prev_track = events[0]
        for tick, track in events[1:]:
            if tick > prev_tick + gap or track != prev_track:
                eps += 1
            prev_tick, prev_track = tick, track
        return eps

    ghost = count_eps(by_class.get('self_ghost', []))
    fp = count_eps(by_class.get('other_fp', []))
    tp = count_eps(by_class.get('true_positive', []))
    return ghost, fp, tp


# ── Data loading ─────────────────────────────────────────────────────

def load_sweep(sweep_dir=None, focal_ego_only=True):
    """Load all simulation_metrics.json, compute derived metrics per run.

    Args:
        sweep_dir: Path to sweep results directory.
        focal_ego_only: If True (default), only load the focal ego (lowest
            actor ID = hero) per run. If False, load all vehicles.

    Uses episode-based counting for S_ghost and S_fp (not per-tick).
    Uses S_clear (not stuck AND no collision) instead of S_prog.
    """
    if sweep_dir is None:
        sweep_dir = SWEEP_DIR

    pattern = os.path.join(sweep_dir, "**", "simulation_metrics.json")
    all_files = glob.glob(pattern, recursive=True)
    files = [f for f in all_files if "evaluation_output" not in f]

    groups = defaultdict(list)
    gt_labeled_count = 0

    for filepath in files:
        with open(filepath) as f:
            d = json.load(f)

        mgr_raw = d.get("config_manager_type", "unknown")
        # Normalize manager names to canonical short forms
        _MGR_ALIASES = {
            'vips_temporal': 'vips',
            'vips_temporal_alignment': 'vips',
            'vips_ab3dmot_linear_predictor': 'vips',
            'worldfusion': 'worldfusion',
            'worldfusion_ab3dmot_linear_predictor': 'worldfusion',
        }
        mgr = _MGR_ALIASES.get(mgr_raw, mgr_raw)
        anch = d.get("config_anchoring", False)
        anch_str = "on" if anch else "off"
        lat = int(d.get("config_latency_ms", 0))
        jitter = d.get("config_jitter_std_ms", None)

        run = {
            'collision_count': d.get('collision_count', 0),
            'filepath': filepath,
            'jitter_std_ms': jitter,
        }

        # Additional config dimensions
        ego_count = d.get('config_ego_count', 1)
        nms_thresh = d.get('config_nms_threshold', None)
        ts_noise = d.get('config_timestamp_noise_ms', None)
        sidr = d.get('config_self_id_radius', None)
        gps_noise = d.get('config_gps_noise_stddev', None)
        gps_dropout = d.get('config_gps_dropout_pct', None)

        run['config_ego_count'] = ego_count
        run['config_nms_threshold'] = nms_thresh
        run['config_timestamp_noise_ms'] = ts_noise
        run['config_self_id_radius'] = sidr
        run['config_gps_noise_stddev'] = gps_noise
        run['config_gps_dropout_pct'] = gps_dropout

        # SEE-V2X trace config
        run['config_see_v2x_filter'] = d.get('config_see_v2x_filter', None)
        run['config_see_v2x_trace'] = d.get('config_see_v2x_trace', None)
        run['config_disable_backhaul'] = d.get('config_disable_backhaul', False)
        run['config_latency_distribution'] = d.get('config_latency_distribution', None)

        # Edge metrics (first edge)
        if d.get('edges'):
            edge = list(d['edges'].values())[0]
            eu = edge.get('ego_uniqueness', {})
            run['ego_uniqueness_total_violations'] = eu.get('total_violations', 0)
            run['ego_uniqueness_violation_tick_fraction'] = eu.get('violation_tick_fraction', 0)
            run['ego_uniqueness_duplicate_track_rate'] = eu.get('duplicate_track_rate', 0)

            eg = edge.get('ego_gate', {})
            run['ego_gate_violations_total'] = eg.get('ego_gate_violations_total', 0)
            run['ego_gate_violation_tick_fraction'] = eg.get('ego_gate_violation_tick_fraction', 0)

            cr = edge.get('competing_risk_events', {})
            run['competing_risk_events'] = cr

            # GeomDup metrics (from profiler summary)
            run['geom_dup_total_clusters'] = edge.get('geom_dup_total_clusters', 0)
            run['geom_dup_total_tracks'] = edge.get('geom_dup_total_tracks', 0)
            run['geom_dup_tick_fraction'] = edge.get('geom_dup_tick_fraction', 0.0)

            # Contract metrics (A1/A2/A3)
            run['contract_a1_violations'] = edge.get('contract_a1_freshness_violations', 0)
            run['contract_a1_fraction'] = edge.get('contract_a1_freshness_violation_fraction', 0.0)
            run['contract_a2_violations'] = edge.get('contract_a2_ego_gate_violations', 0)
            run['contract_a2_fraction'] = edge.get('contract_a2_ego_gate_violation_fraction', 0.0)
            run['contract_a3_violations'] = edge.get('contract_a3_uniqueness_violations', 0)
            run['contract_a3_fraction'] = edge.get('contract_a3_uniqueness_violation_fraction', 0.0)

            # Edge timing metrics (from profiler)
            run['edge_timing_avg_ms'] = edge.get('timing_avg_ms', None)
            run['edge_timing_p95_ms'] = edge.get('timing_p95_ms', None)
            run['edge_timing_p99_ms'] = edge.get('timing_p99_ms', None)
            run['edge_gpu_util_pct'] = edge.get('gpu_utilization_avg_pct', None)
            run['edge_cpu_pct'] = edge.get('cpu_avg_pct', None)

            # AoI at publish boundary (from profiler, in ticks)
            run['edge_aoi_mean_ticks'] = edge.get('aoi_mean_ticks', None)
            run['edge_aoi_p95_ticks'] = edge.get('aoi_p95_ticks', None)
            run['edge_aoi_p99_ticks'] = edge.get('aoi_p99_ticks', None)

            # MAC model metrics
            mac = edge.get('mac', {})
            run['mac_prr'] = mac.get('prr', None)
            run['mac_burst_histogram'] = mac.get('burst_histogram', {})
            run['mac_collision_drops'] = mac.get('collision_drops', 0)
            run['mac_fading_drops'] = mac.get('fading_drops', 0)
            run['mac_burst_mean'] = mac.get('burst_length_mean', None)
            run['mac_burst_p95'] = mac.get('burst_length_p95', None)
            run['mac_burstiness_factor'] = mac.get('mac_burstiness_factor', None)

        # Top-level MAC config dimensions
        run['config_mac_type'] = d.get('config_mac_type', None)
        run['config_mac_M'] = d.get('config_mac_M', None)
        run['config_mac_p_keep'] = d.get('config_mac_p_keep', None)

        # Vehicle selection: focal ego only or all
        vehicles = d.get('vehicles', {})
        if not vehicles:
            continue

        if focal_ego_only:
            focal_vid = min(vehicles.keys(), key=lambda k: int(k))
            vehicle_iter = [(0, focal_vid, vehicles[focal_vid])]
        else:
            vehicle_iter = [(idx, vid, veh) for idx, (vid, veh)
                            in enumerate(vehicles.items())]

        for ego_idx, vid, veh in vehicle_iter:
            veh_run = dict(run)
            veh_run['ego_index'] = ego_idx
            veh_run['ego_vehicle_id'] = vid

            veh_run['total_brake_events'] = veh.get('total_brake_events', 0)
            veh_run['ghost_brake_events'] = veh.get('ghost_brake_events', 0)
            veh_run['ghost_brake_by_id'] = veh.get('ghost_brake_by_id', 0)
            veh_run['ghost_brake_by_proximity'] = veh.get('ghost_brake_by_proximity', 0)
            veh_run['false_brake_rate'] = veh.get('false_brake_rate', 0.0)
            veh_run['avg_speed_mps'] = veh.get('avg_speed_mps', 0)

            # Exposure metrics (for normalization)
            veh_run['distance_traveled_m'] = veh.get('distance_traveled_m', None)
            veh_run['time_alive_s'] = veh.get('time_alive_s', None)

            safety = veh.get('safety', {})
            veh_run['collision_count'] = safety.get('collision_count', 0)
            veh_run['stuck'] = safety.get('stuck', False)

            brake_attrs = veh.get('brake_attributions', [])
            veh_run['brake_attributions'] = brake_attrs

            # GT-labeled tick counts (for reference)
            veh_run['ghost_brake_gt'] = veh.get('ghost_brake_gt', None)
            veh_run['other_fp_gt'] = veh.get('other_fp_gt', None)
            veh_run['true_positive_gt'] = veh.get('true_positive_gt', None)
            veh_run['gt_labeled_count'] = veh.get('gt_labeled_count', 0)

            # ── Episode-based counting ──
            ghost_eps, fp_eps, tp_eps = _count_episodes(brake_attrs)
            veh_run['ghost_episodes'] = ghost_eps
            veh_run['fp_episodes'] = fp_eps
            veh_run['tp_episodes'] = tp_eps
            veh_run['total_episodes'] = ghost_eps + fp_eps + tp_eps

            # Legacy tick-based counts (for reference / comparison)
            veh_run['ghost_count'] = veh_run.get('ghost_brake_gt', 0) or 0
            veh_run['other_fp_count'] = veh_run.get('other_fp_gt', 0) or 0
            veh_run['tp_count'] = veh_run.get('true_positive_gt', 0) or 0

            # ── AoI extraction (Δ_meas and Δ_gen) ──
            dt_ms = 50.0  # 0.05s * 1000
            delta_meas_list = []
            delta_gen_list = []
            for a in brake_attrs:
                st = a.get('source_tick')
                pt = a.get('publish_tick')
                tt = a.get('trigger_tick')
                if st is not None and tt is not None and tt > st:
                    delta_meas_list.append((tt - st) * dt_ms)
                if pt is not None and tt is not None and tt > pt:
                    delta_gen_list.append((tt - pt) * dt_ms)
            veh_run['aoi_mean_ms'] = float(np.mean(delta_meas_list)) if delta_meas_list else None
            veh_run['aoi_p95_ms'] = float(np.percentile(delta_meas_list, 95)) if len(delta_meas_list) >= 2 else None
            veh_run['aoi_p99_ms'] = float(np.percentile(delta_meas_list, 99)) if len(delta_meas_list) >= 2 else None
            veh_run['aoi_samples'] = delta_meas_list
            veh_run['delta_meas_samples'] = delta_meas_list
            veh_run['delta_gen_samples'] = delta_gen_list
            veh_run['delta_gen_mean_ms'] = float(np.mean(delta_gen_list)) if delta_gen_list else None
            veh_run['delta_gen_p99_ms'] = float(np.percentile(delta_gen_list, 99)) if len(delta_gen_list) >= 2 else None

            # ── Provenance extraction ──
            prov_counts = defaultdict(int)
            for a in brake_attrs:
                prov_counts[a.get('gt_provenance', 'unknown')] += 1
            veh_run['provenance_self'] = prov_counts.get('self', 0)
            veh_run['provenance_cross_ego'] = prov_counts.get('cross_ego', 0)
            veh_run['provenance_npc_hazard'] = prov_counts.get('npc_hazard', 0)
            veh_run['provenance_parked'] = prov_counts.get('parked', 0)
            veh_run['provenance_npc_non_hazard'] = prov_counts.get('npc_non_hazard', 0)
            veh_run['provenance_unknown'] = prov_counts.get('unknown', 0)

            # ── Compute 4-tier success metrics (EPISODE-based) ──
            has_gt = veh_run.get('ghost_brake_gt') is not None

            # S_coll: focal ego had no collisions
            veh_run['S_coll'] = 1.0 if veh_run['collision_count'] == 0 else 0.0

            if has_gt:
                gt_labeled_count += 1
                # Episode-based S_ghost and S_fp
                veh_run['S_ghost'] = 1.0 if ghost_eps == 0 else 0.0
                veh_run['S_fp'] = 1.0 if fp_eps == 0 else 0.0
            else:
                veh_run['S_ghost'] = 1.0 if veh_run['ghost_brake_events'] == 0 else 0.0
                veh_run['S_fp'] = 1.0

            # S_clear: task completion (not stuck AND no collision)
            veh_run['S_clear'] = 1.0 if (not veh_run['stuck'] and veh_run['collision_count'] == 0) else 0.0

            # S_op: operational success (all four AND'd)
            veh_run['S_op'] = veh_run['S_coll'] * veh_run['S_ghost'] * veh_run['S_fp'] * veh_run['S_clear']

            groups[(mgr, anch_str, lat, ego_count)].append(veh_run)

    n_runs = sum(len(v) for v in groups.values())
    pct_gt = (gt_labeled_count / n_runs * 100) if n_runs else 0
    mode = "focal-ego-only" if focal_ego_only else "all-ego"
    print(f"  {n_runs} runs ({mode}), {gt_labeled_count} GT-labeled ({pct_gt:.0f}%)")

    return groups


def get_latencies(groups):
    """Auto-detect sorted latency values from loaded data."""
    lats = set()
    for key in groups.keys():
        lats.add(key[2])  # lat is index 2 in (mgr, anch, lat, ego_count)
    return sorted(lats)


def get_ego_counts(groups):
    """Auto-detect sorted ego count values from loaded data."""
    return sorted({key[3] for key in groups.keys()})


def filter_groups(groups, ego_count=1):
    """Filter groups to a specific ego count, returning 3-tuple keyed dict."""
    return {(mgr, anch, lat): runs
            for (mgr, anch, lat, ec), runs in groups.items()
            if ec == ego_count}


def get_series(groups, metric, mgr, anch_str, latencies=None, ego_count=1):
    """Get mean and std arrays for a metric across latencies."""
    if latencies is None:
        latencies = get_latencies(groups)
    means, stds = [], []
    for lat in latencies:
        key = (mgr, anch_str, lat, ego_count)
        vals = [r.get(metric, 0) for r in groups.get(key, [])]
        if vals:
            means.append(np.mean(vals))
            stds.append(np.std(vals))
        else:
            means.append(np.nan)
            stds.append(0)
    return np.array(means), np.array(stds)


def _plot_lines(ax, groups, metric, keys=None, ego_count=1):
    """Plot configs for a given metric with error bars."""
    if keys is None:
        keys = ALL_KEYS
    latencies = get_latencies(groups)
    lats = np.array(latencies)
    for k in keys:
        mgr, anch = MGR_MAP[k]
        means, stds = get_series(groups, metric, mgr, anch, latencies, ego_count=ego_count)
        if np.all(np.isnan(means)):
            continue
        ax.errorbar(lats, means, yerr=stds,
                    marker=MARKERS[k], ls=STYLES[k], color=C[k],
                    label=L[k], linewidth=2, markersize=5, capsize=3)


# ── Figure 0: SEE-V2X Trace Characterization ──────────────────────

def fig0_trace_characterization(see_v2x_trace_path=None):
    """Fig 0: SEE-V2X C-V2X trace characterization.

    (a) CDF of radio latency per intersection and aggregate.
    (b) PDF (histogram) of radio latency per intersection.
    (c) CDF of total latency: radio + backhaul + base, per intersection.
    (d) Table inset: per-intersection summary stats.

    This figure is generated directly from the trace data (not from
    simulation runs), so it takes the trace path instead of groups.
    """
    try:
        from opencda.core.application.edge.latency.latency_model import (
            get_trace_metadata, _DEFAULT_BACKHAUL_MU, _DEFAULT_BACKHAUL_SIGMA
        )
    except ImportError:
        print("  Skipping fig0_trace_characterization (cannot import latency_model)")
        return

    meta = get_trace_metadata(see_v2x_trace_path)
    if not meta or 'per_intersection' not in meta:
        print("  Skipping fig0_trace_characterization (no trace data)")
        return

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    int_colors = {'3': '#1f77b4', '4': '#ff7f0e', '5': '#2ca02c'}
    int_labels = {'3': 'Int 3 (L)', '4': 'Int 4 (M)', '5': 'Int 5 (H)'}

    # (a) CDF of radio latency per intersection
    ax = axes[0][0]
    for iid in sorted(meta['per_intersection'].keys()):
        s = np.sort(meta['per_intersection'][iid]['samples'])
        cdf = np.arange(1, len(s) + 1) / len(s)
        ax.plot(s, cdf, color=int_colors.get(iid, 'gray'),
                label=int_labels.get(iid, f'Int {iid}'), linewidth=1.5)
    # Aggregate
    agg = np.sort(meta['aggregate']['samples'])
    ax.plot(agg, np.arange(1, len(agg)+1)/len(agg),
            'k--', label='Aggregate', linewidth=1.5, alpha=0.7)
    ax.set_xlabel('Radio Latency (ms)')
    ax.set_ylabel('CDF')
    ax.set_title('(a) C-V2X Radio Latency CDF')
    ax.legend(fontsize=8, loc='lower right')
    ax.grid(True, alpha=0.2)
    ax.set_xlim(0, 70)

    # (b) PDF (histogram) per intersection
    ax = axes[0][1]
    bins = np.linspace(0, 65, 60)
    for iid in sorted(meta['per_intersection'].keys()):
        samples = meta['per_intersection'][iid]['samples']
        ax.hist(samples, bins=bins, alpha=0.5,
                color=int_colors.get(iid, 'gray'),
                label=int_labels.get(iid, f'Int {iid}'),
                density=True)
    ax.set_xlabel('Radio Latency (ms)')
    ax.set_ylabel('Density')
    ax.set_title('(b) C-V2X Radio Latency Distribution')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2)

    # (c) CDF of total latency: radio + backhaul + base (0ms base)
    ax = axes[1][0]
    rng = np.random.default_rng(42)
    for iid in sorted(meta['per_intersection'].keys()):
        radio = meta['per_intersection'][iid]['samples']
        # Simulate backhaul
        backhaul = rng.lognormal(_DEFAULT_BACKHAUL_MU, _DEFAULT_BACKHAUL_SIGMA,
                                  size=len(radio)).astype(np.float32)
        total = radio + backhaul
        s = np.sort(total)
        cdf = np.arange(1, len(s)+1) / len(s)
        ax.plot(s, cdf, color=int_colors.get(iid, 'gray'),
                label=f'{int_labels.get(iid, f"Int {iid}")} (radio+backhaul)',
                linewidth=1.5)
    # Also plot radio-only aggregate for reference
    agg = np.sort(meta['aggregate']['samples'])
    ax.plot(agg, np.arange(1, len(agg)+1)/len(agg),
            'k:', label='Radio only (agg)', linewidth=1, alpha=0.5)
    ax.set_xlabel('Total Latency (ms)')
    ax.set_ylabel('CDF')
    ax.set_title('(c) Total Latency CDF (radio + backhaul)')
    ax.legend(fontsize=7, loc='lower right')
    ax.grid(True, alpha=0.2)
    ax.set_xlim(0, 200)

    # (d) Summary stats table
    ax = axes[1][1]
    ax.axis('off')
    # Build table data
    col_labels = ['Intersection', 'N', 'Mean', 'P50', 'P95', 'P99', 'Max']
    table_data = []
    for iid in sorted(meta['per_intersection'].keys()):
        st = meta['per_intersection'][iid]
        regime = {'3': 'L', '4': 'M', '5': 'H'}.get(iid, '?')
        table_data.append([
            f'Int {iid} ({regime})',
            f"{st['n']:,}",
            f"{st['mean']:.1f}",
            f"{st['median']:.1f}",
            f"{st['p95']:.1f}",
            f"{st['p99']:.1f}",
            f"{st['max']:.1f}",
        ])
    st = meta['aggregate']
    table_data.append([
        'Aggregate',
        f"{meta['total_samples']:,}",
        f"{st['mean']:.1f}",
        f"{st['median']:.1f}",
        f"{st['p95']:.1f}",
        f"{st['p99']:.1f}",
        f"{st['max']:.1f}",
    ])

    tbl = ax.table(cellText=table_data, colLabels=col_labels,
                    loc='center', cellLoc='center')
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1, 1.4)
    ax.set_title('(d) SEE-V2X Trace Summary (ms)', fontsize=10, pad=20)

    fig.suptitle('Fig 0: SEE-V2X C-V2X Trace Characterization',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig0_trace_characterization.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig0_trace_characterization.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig0_trace_characterization")


# ── Figure 1: Main Envelope (Pr[S_op=1] vs latency) ────────────────

def fig1_main_envelope(groups):
    """Fig 1: Main envelope plot — Pr[S_op=1] vs latency for all configs."""
    fig, ax = plt.subplots(figsize=(8, 5))

    _plot_lines(ax, groups, 'S_op')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{op}=1]$')
    ax.set_title('Operational Success vs Edge Latency')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=8)
    ax.grid(True, alpha=0.2)

    fig.tight_layout()
    fig.savefig(f'{OUT}/fig1_main_envelope.png')
    fig.savefig(f'{OUT}/fig1_main_envelope.eps')
    plt.close(fig)
    print("  Saved fig1_main_envelope")


# ── Figure 1b-e: Individual S_op components ──────────────────────────

def fig1b_s_coll(groups):
    """Fig 1b: Pr[S_coll=1] vs latency — collision-free rate."""
    fig, ax = plt.subplots(figsize=(8, 5))
    _plot_lines(ax, groups, 'S_coll')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{coll}=1]$')
    ax.set_title('Collision-Free Rate vs Edge Latency')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=8)
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig1b_s_coll.png')
    fig.savefig(f'{OUT}/fig1b_s_coll.eps')
    plt.close(fig)
    print("  Saved fig1b_s_coll")


def fig1c_s_ghost(groups):
    """Fig 1c: Pr[S_ghost=1] vs latency — self-ghost-free rate."""
    fig, ax = plt.subplots(figsize=(8, 5))
    _plot_lines(ax, groups, 'S_ghost')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{ghost}=1]$')
    ax.set_title('Self-Ghost-Free Rate vs Edge Latency')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=8)
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig1c_s_ghost.png')
    fig.savefig(f'{OUT}/fig1c_s_ghost.eps')
    plt.close(fig)
    print("  Saved fig1c_s_ghost")


def fig1d_s_fp(groups):
    """Fig 1d: Pr[S_fp=1] vs latency — other-FP-free rate."""
    fig, ax = plt.subplots(figsize=(8, 5))
    _plot_lines(ax, groups, 'S_fp')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{fp}=1]$')
    ax.set_title('Other-FP-Free Rate vs Edge Latency')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=8)
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig1d_s_fp.png')
    fig.savefig(f'{OUT}/fig1d_s_fp.eps')
    plt.close(fig)
    print("  Saved fig1d_s_fp")


def fig1e_s_clear(groups):
    """Fig 1e: Pr[S_clear=1] vs latency — task completion rate."""
    fig, ax = plt.subplots(figsize=(8, 5))
    _plot_lines(ax, groups, 'S_clear')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{clear}=1]$')
    ax.set_title('Task Completion Rate vs Edge Latency')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=8)
    ax.grid(True, alpha=0.2)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig1e_s_clear.png')
    fig.savefig(f'{OUT}/fig1e_s_clear.eps')
    plt.close(fig)
    print("  Saved fig1e_s_clear")


# ── Figure 2: Decomposition (failure probabilities) ─────────────────

def fig2_decomposition(groups):
    """Fig 2: Decomposition of failure probabilities.

    Four panels showing (1-Pr[S_x]) for each component:
    (a) physics failures (1-S_coll)
    (b) self-ghost failures (1-S_ghost)  — our target
    (c) other-FP failures (1-S_fp)       — confound check
    (d) progress failures (1-S_prog)     — liveness
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    metrics = [
        ('S_coll', '(a) $1-\\Pr[S_{coll}]$: Physics Failures', 'Collision Rate'),
        ('S_ghost', '(b) $1-\\Pr[S_{ghost}]$: Self-Ghost Failures', 'Self-Ghost Rate'),
        ('S_fp', '(c) $1-\\Pr[S_{fp}]$: Other False-Brake Failures', 'Other FP Rate'),
        ('S_clear', '(d) $1-\\Pr[S_{clear}]$: Task Completion Failures', 'Task Failure Rate'),
    ]

    for ax, (metric, title, ylabel) in zip(axes.flat, metrics):
        # Plot 1 - Pr[S_x] = failure probability
        latencies = get_latencies(groups)
        lats = np.array(latencies)
        for k in ALL_KEYS:
            mgr, anch = MGR_MAP[k]
            means, stds = get_series(groups, metric, mgr, anch, latencies)
            fail_means = 1.0 - means
            ax.errorbar(lats, fail_means, yerr=stds,
                        marker=MARKERS[k], ls=STYLES[k], color=C[k],
                        label=L[k], linewidth=2, markersize=5, capsize=3)

        ax.set_xlabel('Configured Latency (ms)')
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.set_ylim(-0.05, 1.1)
        ax.legend(loc='upper left', fontsize=6.5)
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 2: Failure Decomposition', fontsize=13, y=1.01)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig2_decomposition.png')
    fig.savefig(f'{OUT}/fig2_decomposition.eps')
    plt.close(fig)
    print("  Saved fig2_decomposition")


# ── Figure 3: Mechanism (ego duplicates → ghost brakes) ─────────────

def fig3_mechanism(groups):
    """Fig 3: Mechanism — ego-uniqueness violations and ghost brakes.

    (a) Ego-duplicate incidence vs latency
    (b) Ghost brake count vs latency
    (c) Scatter: ego-uniqueness violations → ghost brakes
    """
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # (a) Ego-uniqueness violations vs latency
    ax = axes[0]
    _plot_lines(ax, groups, 'ego_uniqueness_total_violations')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('Ego-Uniqueness Violations')
    ax.set_title('(a) Duplicate Ego Tracks vs Latency')
    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True, alpha=0.2)

    # (b) Ghost brake count vs latency
    ax = axes[1]
    _plot_lines(ax, groups, 'ghost_count')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('Self-Ghost Brake Events')
    ax.set_title('(b) Self-Ghost Brakes vs Latency')
    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True, alpha=0.2)

    # (c) Scatter: violations → ghost brakes
    ax = axes[2]
    for k in ALL_KEYS:
        mgr, anch = MGR_MAP[k]
        ghosts, viols = [], []
        for lat in get_latencies(groups):
            for ec in get_ego_counts(groups):
                key = (mgr, anch, lat, ec)
                for r in groups.get(key, []):
                    ghosts.append(r.get('ghost_episodes', 0))
                    viols.append(r.get('ego_uniqueness_total_violations', 0))
        ax.scatter(viols, ghosts, color=C[k], marker=MARKERS[k],
                   label=L[k], alpha=0.6, s=30, edgecolors='black', linewidths=0.3)
    ax.set_xlabel('Ego-Uniqueness Violations')
    ax.set_ylabel('Self-Ghost Brake Events')
    ax.set_title('(c) Violations $\\rightarrow$ Ghost Brakes')
    ax.legend(loc='upper left', fontsize=6.5)
    ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 3: Mechanism — Self-Ghosting Causes Emergency Braking',
                 fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig3_mechanism.png')
    fig.savefig(f'{OUT}/fig3_mechanism.eps')
    plt.close(fig)
    print("  Saved fig3_mechanism")


# ── Figure 4: Oracle Instrumentation ────────────────────────────────

def fig4_oracle(groups):
    """Fig 4: Oracle as upper bound.

    Oracle = single-source GT → no multi-camera duplicates → no self-ghosting.
    Shows that self-ghosting is a multi-source consistency problem,
    not a perception quality problem.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # (a) S_ghost: oracle always 1, multi-source degrades
    ax = ax1
    _plot_lines(ax, groups, 'S_ghost')
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{ghost}=1]$')
    ax.set_title('(a) Self-Ghost-Free Rate')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=7)
    ax.grid(True, alpha=0.2)

    # (b) S_coll: physics limit
    ax = ax2
    oracle_keys = ['oracle_on', 'oracle_off', 'lf_on', 'lf_off']
    _plot_lines(ax, groups, 'S_coll', keys=oracle_keys)
    ax.set_xlabel('Configured Latency (ms)')
    ax.set_ylabel('$\\Pr[S_{coll}=1]$')
    ax.set_title('(b) Collision-Free Rate (Physics Limit)')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(loc='lower left', fontsize=7.5)
    ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 4: Oracle — Single-Source Has No Self-Ghosting',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig4_oracle.png')
    fig.savefig(f'{OUT}/fig4_oracle.eps')
    plt.close(fig)
    print("  Saved fig4_oracle")


# ── Figure 5: First-Failure Categorization (stacked bars) ─────────────

FAIL_COLORS = {
    'SUCCESS':       '#4daf4a',
    'FAIL_GHOST':    '#e41a1c',
    'FAIL_OTHER_FP': '#ff7f00',
    'FAIL_PHYS':     '#377eb8',
    'FAIL_STUCK':    '#984ea3',
}
FAIL_ORDER = ['SUCCESS', 'FAIL_GHOST', 'FAIL_OTHER_FP', 'FAIL_PHYS', 'FAIL_STUCK']


def classify_run(run):
    """Assign exactly one first-failure label per run (episode-based).

    Priority hierarchy (targeted failure class first):
      1. FAIL_GHOST   — has any self-ghost brake EPISODE
      2. FAIL_OTHER_FP — has other-FP brake EPISODES (no self-ghost)
      3. FAIL_PHYS    — collision occurred (no ghost/FP episodes)
      4. FAIL_STUCK   — vehicle permanently stuck
      5. SUCCESS
    """
    ghost_eps = run.get('ghost_episodes', 0)
    fp_eps = run.get('fp_episodes', 0)
    coll = run.get('collision_count', 0)
    stuck = run.get('stuck', False)

    if ghost_eps > 0:
        return 'FAIL_GHOST'
    if fp_eps > 0:
        return 'FAIL_OTHER_FP'
    if coll > 0:
        return 'FAIL_PHYS'
    if stuck:
        return 'FAIL_STUCK'
    return 'SUCCESS'


def fig5_first_failure(groups, ego_count=1):
    """Fig 5: Stacked bar plot of first-failure categories vs latency.

    One subplot per config pair (manager × anchoring).
    Each bar shows the fraction of runs in each category.
    """
    latencies = get_latencies(groups)
    n_lats = len(latencies)

    mgr_labels = [('oracle', 'Oracle'), ('late_fusion', 'Late Fusion'), ('vips', 'VIPS')]
    anch_labels = [('on', '+ Anchoring'), ('off', 'no anchoring')]

    fig, axes = plt.subplots(len(mgr_labels), len(anch_labels),
                              figsize=(12, 10), sharey=True, sharex=True)

    for i, (mgr, mgr_name) in enumerate(mgr_labels):
        for j, (anch, anch_name) in enumerate(anch_labels):
            ax = axes[i][j]
            fracs = {cat: [] for cat in FAIL_ORDER}

            for lat in latencies:
                key = (mgr, anch, lat, ego_count)
                runs = groups.get(key, [])
                n = max(len(runs), 1)
                counts = {cat: 0 for cat in FAIL_ORDER}
                for r in runs:
                    counts[classify_run(r)] += 1
                for cat in FAIL_ORDER:
                    fracs[cat].append(counts[cat] / n)

            x = np.arange(n_lats)
            bottoms = np.zeros(n_lats)
            bar_width = 0.7
            for cat in FAIL_ORDER:
                vals = np.array(fracs[cat])
                ax.bar(x, vals, bar_width, bottom=bottoms,
                       color=FAIL_COLORS[cat], label=cat if i == 0 and j == 0 else None)
                bottoms += vals

            ax.set_title(f'{mgr_name} ({anch_name})', fontsize=10)
            ax.set_xticks(x)
            ax.set_xticklabels([str(l) for l in latencies], fontsize=8)
            ax.set_ylim(0, 1.05)
            if j == 0:
                ax.set_ylabel('Fraction of Runs')
            if i == len(mgr_labels) - 1:
                ax.set_xlabel('Latency (ms)')

    handles = [plt.Rectangle((0, 0), 1, 1, color=FAIL_COLORS[c]) for c in FAIL_ORDER]
    labels = [c.replace('FAIL_', '').replace('_', ' ').title() for c in FAIL_ORDER]
    fig.legend(handles, labels, loc='upper center', ncol=len(FAIL_ORDER),
               fontsize=9, bbox_to_anchor=(0.5, 1.02))

    fig.suptitle(f'Fig 5: First-Failure Categorization ({ego_count}-ego, focal)',
                 fontsize=13, y=1.06)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig5_first_failure_{ego_count}ego.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig5_first_failure_{ego_count}ego.eps', bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved fig5_first_failure_{ego_count}ego")


# ── Figure 6: Regime Map (latency × jitter heatmap) ─────────────────

def fig6_regime_map(groups):
    """Fig 6: 2D heatmap of P(S_op=1) over mean latency × jitter std.

    One panel per (manager, anchoring) pair showing the AoI safety envelope.
    """
    # Collect unique latencies and jitters
    latencies = get_latencies(groups)

    # Group runs by (mgr, anch, lat, jitter) — collapse ego_count for regime map
    regime_data = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if ec != 1:
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            regime_data[(mgr, anch, lat, j)].append(r)

    jitters = sorted({r.get('jitter_std_ms') or 0 for runs in groups.values() for r in runs})
    if len(jitters) < 2:
        print("  Skipping fig6_regime_map (need >=2 jitter values)")
        return

    mgr_labels = [('oracle', 'Oracle'), ('late_fusion', 'Late Fusion'), ('vips', 'VIPS')]
    anch_labels = [('on', '+ Anchoring'), ('off', 'no anchoring')]

    fig, axes = plt.subplots(len(mgr_labels), len(anch_labels),
                              figsize=(12, 10))
    if len(mgr_labels) == 1:
        axes = axes[np.newaxis, :]

    for i, (mgr, mgr_name) in enumerate(mgr_labels):
        for j, (anch, anch_name) in enumerate(anch_labels):
            ax = axes[i][j]
            grid = np.full((len(jitters), len(latencies)), np.nan)

            for li, lat in enumerate(latencies):
                for ji, jit in enumerate(jitters):
                    runs = regime_data.get((mgr, anch, lat, jit), [])
                    if runs:
                        grid[ji, li] = np.mean([r['S_op'] for r in runs])

            im = ax.imshow(grid, origin='lower', aspect='auto',
                          vmin=0, vmax=1, cmap='RdYlGn',
                          extent=[latencies[0], latencies[-1],
                                  jitters[0], jitters[-1]])
            ax.set_title(f'{mgr_name} ({anch_name})', fontsize=10)
            ax.set_xlabel('Mean Latency (ms)')
            ax.set_ylabel('Jitter Std (ms)')

            # Overlay dominant first-failure text
            for li, lat in enumerate(latencies):
                for ji, jit in enumerate(jitters):
                    runs = regime_data.get((mgr, anch, lat, jit), [])
                    if runs:
                        cats = [classify_run(r) for r in runs]
                        from collections import Counter
                        most = Counter(cats).most_common(1)[0][0]
                        short = {'SUCCESS': 'OK', 'FAIL_GHOST': 'G',
                                 'FAIL_OTHER_FP': 'FP', 'FAIL_PHYS': 'P',
                                 'FAIL_STUCK': 'S'}.get(most, '?')
                        ax.text(lat, jit, short, ha='center', va='center',
                                fontsize=7, fontweight='bold',
                                color='white' if grid[ji, li] < 0.5 else 'black')

    fig.colorbar(im, ax=axes, label='$\\Pr[S_{op}=1]$', shrink=0.6)
    fig.suptitle('Fig 6: AoI Safety Regime Map', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig6_regime_map.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig6_regime_map.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig6_regime_map")


# ── Figure 7: Provenance Breakdown ──────────────────────────────────

PROV_COLORS = {
    'self': '#e41a1c',
    'cross_ego': '#984ea3',
    'npc_hazard': '#4daf4a',
    'parked': '#ff7f00',
    'npc_non_hazard': '#a6cee3',
    'unknown': '#999999',
}
PROV_ORDER = ['self', 'cross_ego', 'npc_hazard', 'parked', 'npc_non_hazard', 'unknown']


def fig7_provenance(groups, ego_count=1):
    """Fig 7: Stacked bar of brake provenance per config."""
    latencies = get_latencies(groups)
    mgr_labels = [('oracle', 'Oracle'), ('late_fusion', 'Late Fusion'), ('vips', 'VIPS')]
    anch_labels = [('on', '+ Anchoring'), ('off', 'no anchoring')]

    fig, axes = plt.subplots(len(mgr_labels), len(anch_labels),
                              figsize=(12, 10), sharey=True, sharex=True)

    for i, (mgr, mgr_name) in enumerate(mgr_labels):
        for j, (anch, anch_name) in enumerate(anch_labels):
            ax = axes[i][j]
            fracs = {p: [] for p in PROV_ORDER}

            for lat in latencies:
                key = (mgr, anch, lat, ego_count)
                runs = groups.get(key, [])
                total = sum(r.get(f'provenance_{p}', 0) for r in runs for p in PROV_ORDER)
                total = max(total, 1)
                for p in PROV_ORDER:
                    cnt = sum(r.get(f'provenance_{p}', 0) for r in runs)
                    fracs[p].append(cnt / total)

            x = np.arange(len(latencies))
            bottoms = np.zeros(len(latencies))
            for p in PROV_ORDER:
                vals = np.array(fracs[p])
                ax.bar(x, vals, 0.7, bottom=bottoms,
                       color=PROV_COLORS[p],
                       label=p if i == 0 and j == 0 else None)
                bottoms += vals

            ax.set_title(f'{mgr_name} ({anch_name})', fontsize=10)
            ax.set_xticks(x)
            ax.set_xticklabels([str(l) for l in latencies], fontsize=8)
            ax.set_ylim(0, 1.05)
            if j == 0:
                ax.set_ylabel('Fraction of Brakes')
            if i == len(mgr_labels) - 1:
                ax.set_xlabel('Latency (ms)')

    handles = [plt.Rectangle((0, 0), 1, 1, color=PROV_COLORS[p]) for p in PROV_ORDER]
    fig.legend(handles, PROV_ORDER, loc='upper center', ncol=len(PROV_ORDER),
               fontsize=8, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle('Fig 7: Brake Provenance Breakdown', fontsize=13, y=1.06)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig7_provenance.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig7_provenance.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig7_provenance")


# ── Figure 8: Multi-Ego Scaling ─────────────────────────────────────

def fig8_multi_ego_scaling(groups):
    """Fig 8: S_op vs N_ego for fixed latency points.

    Shows the cliff shift where ghost/cross-ego brakes dominate.
    """
    # Detect ego counts in data
    ego_counts = get_ego_counts(groups)
    if len(ego_counts) < 2:
        print("  Skipping fig8_multi_ego_scaling (need >=2 ego counts)")
        return

    latencies = get_latencies(groups)
    ego_groups = defaultdict(lambda: defaultdict(list))
    for (mgr, anch, lat, ec), runs in groups.items():
        for r in runs:
            ego_groups[(mgr, anch, lat, ec)]['S_op'].append(r['S_op'])
            ego_groups[(mgr, anch, lat, ec)]['S_ghost'].append(r['S_ghost'])
            ego_groups[(mgr, anch, lat, ec)]['S_coll'].append(r['S_coll'])
            ego_groups[(mgr, anch, lat, ec)]['ghost_episodes'].append(r.get('ghost_episodes', 0))

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # Plot S_op, S_ghost, S_coll vs N_ego
    for ax, metric, title in zip(axes,
            ['S_op', 'S_ghost', 'S_coll'],
            ['$\\Pr[S_{op}=1]$', '$\\Pr[S_{ghost}=1]$', '$\\Pr[S_{coll}=1]$']):
        for lat in latencies[:4]:  # top 4 latencies
            for anch, ls in [('on', '-'), ('off', '--')]:
                means = []
                for n in ego_counts:
                    key = ('late_fusion', anch, lat, n)
                    vals = ego_groups[key].get(metric, [])
                    means.append(np.mean(vals) if vals else np.nan)
                label = f'lat={lat}ms anch={anch}'
                ax.plot(ego_counts, means, marker='o', ls=ls, label=label)
        ax.set_xlabel('Number of Egos (N)')
        ax.set_ylabel(title)
        ax.set_ylim(-0.05, 1.1)
        ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])
        ax.legend(fontsize=6.5, loc='lower left')
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 8: Multi-Ego Scaling (Late Fusion)', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig8_multi_ego_scaling.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig8_multi_ego_scaling.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig8_multi_ego_scaling")


# ── Figure 9: Sensitivity Panels ────────────────────────────────────

def fig9_sensitivity(groups):
    """Fig 9: 2×2 sensitivity panel.

    (a) S_op vs NMS threshold
    (b) S_op vs GPS noise
    (c) S_op vs timestamp noise
    (d) S_op vs self-ID radius
    """
    panels = [
        ('config_nms_threshold', 'NMS Threshold (m)'),
        ('config_gps_noise_stddev', 'GNSS Noise Std'),
        ('config_timestamp_noise_ms', 'Timestamp Noise (ms)'),
        ('config_self_id_radius', 'Self-ID Radius (m)'),
    ]

    # Check which dimensions vary
    active_panels = []
    for field, label in panels:
        vals = {r.get(field) for runs in groups.values() for r in runs}
        vals.discard(None)
        if len(vals) >= 2:
            active_panels.append((field, label, sorted(vals)))

    if not active_panels:
        print("  Skipping fig9_sensitivity (no ablation dimensions vary)")
        return

    n_panels = len(active_panels)
    cols = min(n_panels, 2)
    rows = (n_panels + 1) // 2

    fig, axes = plt.subplots(rows, cols, figsize=(6 * cols, 5 * rows), squeeze=False)

    for idx, (field, label, vals) in enumerate(active_panels):
        ax = axes[idx // cols][idx % cols]

        for k in ['lf_on', 'lf_off']:
            mgr, anch = MGR_MAP[k]
            means = []
            for v in vals:
                s_ops = [r['S_op'] for (m, a, l, ec), runs in groups.items()
                         for r in runs
                         if m == mgr and a == anch and r.get(field) == v]
                means.append(np.mean(s_ops) if s_ops else np.nan)
            ax.plot(vals, means, marker=MARKERS[k], ls=STYLES[k],
                    color=C[k], label=L[k], linewidth=2, markersize=6)

        ax.set_xlabel(label)
        ax.set_ylabel('$\\Pr[S_{op}=1]$')
        ax.set_ylim(-0.05, 1.1)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)

    # Hide unused subplots
    for idx in range(len(active_panels), rows * cols):
        axes[idx // cols][idx % cols].set_visible(False)

    fig.suptitle('Fig 9: Parameter Sensitivity', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig9_sensitivity.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig9_sensitivity.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig9_sensitivity")


# ── Figure 10: AoI Distribution ─────────────────────────────────────

def fig10_aoi_distribution(groups):
    """Fig 10: CDF of per-brake AoI (ms) by config.

    Left: CDF per (latency, jitter) config.
    Right: AoI p99 vs configured latency.
    """
    # Collect all AoI samples
    config_aoi = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if mgr != 'late_fusion' or ec != 1:
            continue
        for r in runs:
            samples = r.get('aoi_samples', [])
            j = r.get('jitter_std_ms') or 0
            config_aoi[(anch, lat, j)].extend(samples)

    if not any(config_aoi.values()):
        print("  Skipping fig10_aoi_distribution (no AoI data)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # (a) CDF per config
    colors = plt.cm.viridis(np.linspace(0, 1, len(config_aoi)))
    for idx, ((anch, lat, jit), samples) in enumerate(sorted(config_aoi.items())):
        if not samples:
            continue
        sorted_s = np.sort(samples)
        cdf = np.arange(1, len(sorted_s) + 1) / len(sorted_s)
        ax1.plot(sorted_s, cdf, color=colors[idx],
                 label=f'lat={lat} jit={jit} anch={anch}', linewidth=1.5)
    ax1.set_xlabel('AoI at Use (ms)')
    ax1.set_ylabel('CDF')
    ax1.set_title('(a) AoI Distribution per Config')
    ax1.legend(fontsize=6, loc='lower right')
    ax1.grid(True, alpha=0.2)

    # (b) AoI p99 vs latency
    latencies = get_latencies(groups)
    for anch, ls in [('on', '-'), ('off', '--')]:
        p99s = []
        for lat in latencies:
            all_samples = []
            for j_key, samples in config_aoi.items():
                if j_key[0] == anch and j_key[1] == lat:
                    all_samples.extend(samples)
            if all_samples:
                p99s.append(np.percentile(all_samples, 99))
            else:
                p99s.append(np.nan)
        ax2.plot(latencies, p99s, marker='o', ls=ls,
                 label=f'Anchoring {anch}', linewidth=2)
    ax2.set_xlabel('Configured Latency (ms)')
    ax2.set_ylabel('AoI p99 (ms)')
    ax2.set_title('(b) Tail AoI vs Configured Latency')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.2)

    fig.suptitle('Fig 10: Age of Information at Brake Decision',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig10_aoi_distribution.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig10_aoi_distribution.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig10_aoi_distribution")


# ── Table 1: LaTeX Summary ──────────────────────────────────────────

def table1_latex(groups, ego_count=1):
    """Generate LaTeX-formatted summary table."""
    latencies = get_latencies(groups)
    rows = []

    for k in ALL_KEYS:
        mgr, anch = MGR_MAP[k]
        for lat in latencies:
            key = (mgr, anch, lat, ego_count)
            runs = groups.get(key, [])
            if not runs:
                continue
            n = len(runs)
            s_op = np.mean([r['S_op'] for r in runs])
            s_coll = np.mean([r['S_coll'] for r in runs])
            s_ghost = np.mean([r['S_ghost'] for r in runs])
            s_fp = np.mean([r['S_fp'] for r in runs])

            # Dominant failure
            cats = [classify_run(r) for r in runs]
            from collections import Counter
            dom = Counter(cats).most_common(1)[0][0]

            # AoI stats
            aoi_means = [r['aoi_mean_ms'] for r in runs if r.get('aoi_mean_ms') is not None]
            aoi_p99s = [r['aoi_p99_ms'] for r in runs if r.get('aoi_p99_ms') is not None]
            aoi_mean = f"{np.mean(aoi_means):.0f}" if aoi_means else "--"
            aoi_p99 = f"{np.mean(aoi_p99s):.0f}" if aoi_p99s else "--"

            j = runs[0].get('jitter_std_ms') or 0
            rows.append(f"  {L[k]:<30} & {lat:>4} & {j:>4} & {n:>3} & "
                        f"{s_op:.2f} & {s_coll:.2f} & {s_ghost:.2f} & {s_fp:.2f} & "
                        f"{dom:<15} & {aoi_mean:>5} & {aoi_p99:>5} \\\\")

    table = "\\begin{tabular}{lrrr|rrrr|lrr}\n"
    table += "  Config & Lat & Jit & N & $S_{op}$ & $S_{coll}$ & $S_{ghost}$ & $S_{fp}$ & Dom.~Failure & $\\bar{\\Delta}$ & $\\Delta_{99}$ \\\\\n"
    table += "  \\hline\n"
    table += "\n".join(rows) + "\n"
    table += "\\end{tabular}\n"

    out_path = f'{OUT}/table1_summary.tex'
    with open(out_path, 'w') as f:
        f.write(table)
    print(f"  Saved table1_summary.tex")


# ── Summary table ────────────────────────────────────────────────────

def print_summary_table(groups):
    """Print summary table, auto-detecting jitter dimension."""
    # Check if jitter varies
    all_jitters = set()
    for runs in groups.values():
        for r in runs:
            all_jitters.add(r.get('jitter_std_ms'))
    has_jitter = len(all_jitters - {None}) > 0

    # Sub-group by jitter if it varies
    if has_jitter:
        sub_groups = defaultdict(list)
        for (mgr, anch, lat, ec), runs in groups.items():
            for r in runs:
                j = r.get('jitter_std_ms', 0) or 0
                sub_groups[(mgr, anch, lat, ec, j)].append(r)
        jitter_label = "Jitt"
    else:
        sub_groups = {(mgr, anch, lat, ec, 0): runs
                      for (mgr, anch, lat, ec), runs in groups.items()}
        jitter_label = None

    print(f"\n{'='*180}")
    print(f"  SWEEP SUMMARY (avg over reps)")
    print(f"{'='*180}")
    jit_col = f" {jitter_label:>5}" if jitter_label else ""
    header = (f"{'Manager':<17} {'Anch':<5} {'Lat':>4}{jit_col} {'ego':>3} | "
              f"{'N':>3} {'P(op)':>6} {'P(col)':>6} {'P(gho)':>6} {'P(fp)':>6} {'P(clr)':>6} | "
              f"{'gho_e':>5} {'fp_e':>5} {'tp_e':>5} {'tot_e':>5} | "
              f"{'gho_t':>5} {'fp_t':>5} {'tp_t':>5} | "
              f"{'uniq_v':>6} {'spd':>5}")
    print(header)
    print("-" * len(header))

    sorted_keys = sorted(sub_groups.keys(), key=lambda k: (k[3], k[0], k[1], k[4], k[2]))
    prev_section = None
    for key in sorted_keys:
        mgr, anch, lat, ec, jit = key
        runs = sub_groups[key]

        section = (ec, mgr, anch, jit)
        if prev_section is not None and section != prev_section:
            if prev_section[0] != ec:
                print(f"\n{'='*20} {ec}-EGO {'='*20}")
            else:
                print()
        elif prev_section is None:
            print(f"\n{'='*20} {ec}-EGO {'='*20}")
        prev_section = section

        def avg(metric):
            vals = [r.get(metric, 0) for r in runs if r.get(metric) is not None]
            return np.mean(vals) if vals else 0

        jit_str = f" {int(jit):>5}" if jitter_label else ""
        print(f"{mgr:<17} {anch:<5} {lat:>4}{jit_str} {ec:>3} | "
              f"{len(runs):>3} "
              f"{avg('S_op'):>6.2f} "
              f"{avg('S_coll'):>6.2f} "
              f"{avg('S_ghost'):>6.2f} "
              f"{avg('S_fp'):>6.2f} "
              f"{avg('S_clear'):>6.2f} | "
              f"{avg('ghost_episodes'):>5.1f} "
              f"{avg('fp_episodes'):>5.1f} "
              f"{avg('tp_episodes'):>5.1f} "
              f"{avg('total_episodes'):>5.1f} | "
              f"{avg('ghost_count'):>5.1f} "
              f"{avg('other_fp_count'):>5.1f} "
              f"{avg('tp_count'):>5.1f} | "
              f"{avg('ego_uniqueness_total_violations'):>6.0f} "
              f"{avg('avg_speed_mps'):>5.1f}")


# ── Figure 11: Δ_meas vs Δ_gen CDFs ──────────────────────────────────

def fig11_delta_meas_vs_gen(groups):
    """Fig 11: CDF of Δ_meas and Δ_gen per config.

    Shows that measurement freshness (Δ_meas) and model freshness (Δ_gen)
    diverge under jitter, revealing the AoI ambiguity under prediction.
    """
    config_meas = defaultdict(list)
    config_gen = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if mgr != 'late_fusion' or ec != 1:
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            config_meas[(anch, lat, j)].extend(r.get('delta_meas_samples', []))
            config_gen[(anch, lat, j)].extend(r.get('delta_gen_samples', []))

    has_meas = any(config_meas.values())
    has_gen = any(config_gen.values())
    if not has_meas and not has_gen:
        print("  Skipping fig11_delta_meas_vs_gen (no AoI data)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    colors = plt.cm.viridis(np.linspace(0, 1, max(len(config_meas), 1)))

    for idx, key in enumerate(sorted(config_meas.keys())):
        anch, lat, jit = key
        lbl = f'lat={lat} jit={jit} anch={anch}'
        # Δ_meas CDF
        samples = config_meas.get(key, [])
        if samples:
            s = np.sort(samples)
            ax1.plot(s, np.arange(1, len(s)+1)/len(s), color=colors[idx],
                     label=lbl, linewidth=1.5)
        # Δ_gen CDF
        samples = config_gen.get(key, [])
        if samples:
            s = np.sort(samples)
            ax2.plot(s, np.arange(1, len(s)+1)/len(s), color=colors[idx],
                     label=lbl, linewidth=1.5)

    ax1.set_xlabel('$\\Delta_{meas}$ (ms)')
    ax1.set_ylabel('CDF')
    ax1.set_title('(a) Measurement Freshness')
    ax1.legend(fontsize=6, loc='lower right')
    ax1.grid(True, alpha=0.2)

    ax2.set_xlabel('$\\Delta_{gen}$ (ms)')
    ax2.set_ylabel('CDF')
    ax2.set_title('(b) Model/Update Freshness')
    ax2.legend(fontsize=6, loc='lower right')
    ax2.grid(True, alpha=0.2)

    fig.suptitle('Fig 11: $\\Delta_{meas}$ vs $\\Delta_{gen}$ Distributions',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig11_delta_meas_vs_gen.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig11_delta_meas_vs_gen.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig11_delta_meas_vs_gen")


# ── Figure 12: Success vs AoI p95/p99 (directly, not configured jitter) ──

def fig12_success_vs_aoi_tail(groups):
    """Fig 12: P(S_op=1) plotted directly against measured AoI tail.

    This is the key MobiCom result: tail freshness predicts failure,
    not configured mean latency.
    """
    # Collect (aoi_p95, aoi_p99, S_op) per run
    runs_with_aoi = []
    for (mgr, anch, lat, ec), runs in groups.items():
        for r in runs:
            p95 = r.get('aoi_p95_ms')
            p99 = r.get('aoi_p99_ms')
            if p95 is not None:
                runs_with_aoi.append({
                    'mgr': mgr, 'anch': anch, 'lat': lat,
                    'aoi_p95': p95, 'aoi_p99': p99 or p95,
                    'S_op': r['S_op'], 'S_ghost': r['S_ghost'],
                    'S_coll': r['S_coll'],
                })

    if not runs_with_aoi:
        print("  Skipping fig12_success_vs_aoi_tail (no AoI data)")
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    for ax, metric, title in zip(axes,
            ['S_op', 'S_ghost', 'S_coll'],
            ['$\\Pr[S_{op}=1]$', '$\\Pr[S_{ghost}=1]$', '$\\Pr[S_{coll}=1]$']):
        # Bin by AoI p99
        all_p99 = [r['aoi_p99'] for r in runs_with_aoi]
        bins = np.linspace(min(all_p99), max(all_p99), 10)
        bin_centers = (bins[:-1] + bins[1:]) / 2
        bin_means = []
        for lo, hi in zip(bins[:-1], bins[1:]):
            vals = [r[metric] for r in runs_with_aoi if lo <= r['aoi_p99'] < hi]
            bin_means.append(np.mean(vals) if vals else np.nan)

        ax.plot(bin_centers, bin_means, 'o-', linewidth=2, markersize=6)
        ax.set_xlabel('Measured AoI $p_{99}$ (ms)')
        ax.set_ylabel(title)
        ax.set_ylim(-0.05, 1.1)
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 12: Success vs Measured AoI Tail (p99)',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig12_success_vs_aoi_tail.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig12_success_vs_aoi_tail.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig12_success_vs_aoi_tail")


# ── Figure 13: Same mean, different tail ──────────────────────────────

def fig13_same_mean_different_tail(groups):
    """Fig 13: Two configs with same mean latency but different tails.

    Shows that mean delay is non-predictive when tails diverge.
    Compares fixed vs normal/lognormal distributions at matched means.
    """
    # Group by (mgr, anch, mean_lat) and look for different jitter values
    matched = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if ec != 1:
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            matched[(mgr, anch, lat)].append((j, r))

    # Find cases with same mean lat, different jitter
    found_pairs = []
    for (mgr, anch, lat), pairs in matched.items():
        jitters = sorted(set(j for j, _ in pairs))
        if len(jitters) >= 2:
            found_pairs.append((mgr, anch, lat, jitters))

    if not found_pairs:
        print("  Skipping fig13_same_mean_different_tail (need same-mean configs)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # (a) S_op vs jitter at fixed mean latency
    ax = axes[0]
    for mgr, anch, lat, jitters in found_pairs:
        key_label = f'{mgr} anch={anch} lat={lat}'
        s_ops = []
        for j in jitters:
            vals = [r['S_op'] for jj, r in matched[(mgr, anch, lat)] if jj == j]
            s_ops.append(np.mean(vals) if vals else np.nan)
        ax.plot(jitters, s_ops, 'o-', label=key_label, linewidth=2, markersize=6)
    ax.set_xlabel('Jitter Std (ms) [same mean latency]')
    ax.set_ylabel('$\\Pr[S_{op}=1]$')
    ax.set_title('(a) Same Mean, Different Tail')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(fontsize=6.5, loc='lower left')
    ax.grid(True, alpha=0.2)

    # (b) AoI p99 vs jitter at fixed mean latency
    ax = axes[1]
    for mgr, anch, lat, jitters in found_pairs:
        key_label = f'{mgr} anch={anch} lat={lat}'
        p99s = []
        for j in jitters:
            all_samples = []
            for jj, r in matched[(mgr, anch, lat)]:
                if jj == j:
                    all_samples.extend(r.get('aoi_samples', []))
            p99s.append(np.percentile(all_samples, 99) if all_samples else np.nan)
        ax.plot(jitters, p99s, 'o-', label=key_label, linewidth=2, markersize=6)
    ax.set_xlabel('Jitter Std (ms) [same mean latency]')
    ax.set_ylabel('AoI $p_{99}$ (ms)')
    ax.set_title('(b) AoI Tail Amplification')
    ax.legend(fontsize=6.5, loc='upper left')
    ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 13: Same Mean Delay, Different Tail $\\rightarrow$ Different Outcome',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig13_same_mean_different_tail.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig13_same_mean_different_tail.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig13_same_mean_different_tail")


# ── Figure 14: Ego-Gate Violation Overlay on Regime Map ───────────────

def fig14_ego_gate_regime(groups):
    """Fig 14: Regime map with ego-gate violation probability overlay.

    Shows where the ego-consistency invariant is violated.
    """
    regime_data = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if ec != 1:
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            regime_data[(mgr, anch, lat, j)].append(r)

    jitters = sorted({r.get('jitter_std_ms') or 0
                      for runs in groups.values() for r in runs})
    latencies = get_latencies(groups)

    has_gate = any(r.get('ego_gate_violations_total', 0) > 0
                   for runs in groups.values() for r in runs)
    if not has_gate or len(jitters) < 2:
        print("  Skipping fig14_ego_gate_regime (no ego-gate data or <2 jitters)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    for j, (anch, anch_name) in enumerate([('on', '+ Anchoring'), ('off', 'no anchoring')]):
        ax = axes[j]
        grid = np.full((len(jitters), len(latencies)), np.nan)

        for li, lat in enumerate(latencies):
            for ji, jit in enumerate(jitters):
                runs = regime_data.get(('late_fusion', anch, lat, jit), [])
                if runs:
                    # Fraction of runs with any ego-gate violations
                    frac = np.mean([1 if r.get('ego_gate_violations_total', 0) > 0
                                   else 0 for r in runs])
                    grid[ji, li] = frac

        im = ax.imshow(grid, origin='lower', aspect='auto',
                       vmin=0, vmax=1, cmap='Reds',
                       extent=[latencies[0], latencies[-1],
                               jitters[0], jitters[-1]])
        ax.set_title(f'Late Fusion ({anch_name})', fontsize=10)
        ax.set_xlabel('Mean Latency (ms)')
        ax.set_ylabel('Jitter Std (ms)')

    fig.colorbar(im, ax=axes, label='$\\Pr[$ego-gate violated$]$', shrink=0.6)
    fig.suptitle('Fig 14: Ego-Consistency Gate Violations', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig14_ego_gate_regime.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig14_ego_gate_regime.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig14_ego_gate_regime")


# ── Figure 15: Competing Risk Stability ───────────────────────────────

def fig15_competing_risk_stability(groups):
    """Fig 15: First-failure stacked bars with stability inset.

    Shows fraction of runs where first two failure events are within
    epsilon ticks of each other (tie rate).
    """
    latencies = get_latencies(groups)

    # Compute tie rates from per-brake timing
    tie_data = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if mgr != 'late_fusion' or ec != 1:
            continue
        for r in runs:
            attrs = r.get('brake_attributions', [])
            if len(attrs) < 2:
                continue
            # Sort by trigger_tick
            sorted_attrs = sorted(
                [a for a in attrs if a.get('trigger_tick') is not None],
                key=lambda a: a['trigger_tick'])
            if len(sorted_attrs) < 2:
                continue
            # Check if first two have different classes
            c0 = sorted_attrs[0].get('gt_brake_class', 'unknown')
            c1 = sorted_attrs[1].get('gt_brake_class', 'unknown')
            t0 = sorted_attrs[0]['trigger_tick']
            t1 = sorted_attrs[1]['trigger_tick']
            epsilon_ticks = 5  # 250ms
            is_tie = abs(t1 - t0) <= epsilon_ticks and c0 != c1
            tie_data[(mgr, anch, lat)].append(1 if is_tie else 0)

    if not any(tie_data.values()):
        print("  Skipping fig15_competing_risk_stability (no multi-brake data)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # (a) First-failure stacked bars for late_fusion only
    for anch, ax in [('on', ax1), ('off', ax2)]:
        fracs = {cat: [] for cat in FAIL_ORDER}
        tie_rates = []

        for lat in latencies:
            key = ('late_fusion', anch, lat, 1)
            runs = groups.get(key, [])
            n = max(len(runs), 1)
            counts = {cat: 0 for cat in FAIL_ORDER}
            for r in runs:
                counts[classify_run(r)] += 1
            for cat in FAIL_ORDER:
                fracs[cat].append(counts[cat] / n)

            ties = tie_data.get(('late_fusion', anch, lat), [])
            tie_rates.append(np.mean(ties) if ties else 0)

        x = np.arange(len(latencies))
        bottoms = np.zeros(len(latencies))
        for cat in FAIL_ORDER:
            vals = np.array(fracs[cat])
            ax.bar(x, vals, 0.6, bottom=bottoms,
                   color=FAIL_COLORS[cat], label=cat)
            bottoms += vals

        # Overlay tie rate as line on secondary axis
        ax2_twin = ax.twinx()
        ax2_twin.plot(x, tie_rates, 'k--o', linewidth=1.5, markersize=4,
                     label='Tie rate ($\\epsilon$=5 ticks)')
        ax2_twin.set_ylabel('Tie Rate', fontsize=9)
        ax2_twin.set_ylim(0, 1)

        ax.set_title(f'Late Fusion (anch={anch})', fontsize=10)
        ax.set_xticks(x)
        ax.set_xticklabels([str(l) for l in latencies], fontsize=8)
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Fraction of Runs')
        ax.set_ylim(0, 1.05)

    handles = [plt.Rectangle((0, 0), 1, 1, color=FAIL_COLORS[c]) for c in FAIL_ORDER]
    labels = [c.replace('FAIL_', '').replace('_', ' ').title() for c in FAIL_ORDER]
    fig.legend(handles, labels, loc='upper center', ncol=len(FAIL_ORDER),
               fontsize=8, bbox_to_anchor=(0.5, 1.02))
    fig.suptitle('Fig 15: First-Failure with Competing-Risk Tie Rate',
                 fontsize=13, y=1.06)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig15_competing_risk.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig15_competing_risk.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig15_competing_risk")


# ── Figure 16: Scaling Attribution ─────────────────────────────────────

def fig16_scaling_attribution(groups):
    """Fig 16: Δ_meas and Δ_gen vs N_ego to attribute scaling.

    Shows whether degradation comes from network (Δ_meas grows) or
    compute (Δ_gen grows) as participant count increases.
    """
    ego_counts = get_ego_counts(groups)
    if len(ego_counts) < 2:
        print("  Skipping fig16_scaling_attribution (need >=2 ego counts)")
        return

    scaling_data = defaultdict(lambda: {'meas': [], 'gen': []})
    for (mgr, anch, lat, ec), runs in groups.items():
        if mgr != 'late_fusion':
            continue
        n = ec
        for r in runs:
            meas = r.get('aoi_mean_ms')
            gen = r.get('delta_gen_mean_ms')
            if meas is not None:
                scaling_data[(anch, lat, n)]['meas'].append(meas)
            if gen is not None:
                scaling_data[(anch, lat, n)]['gen'].append(gen)

    if not any(v['meas'] or v['gen'] for v in scaling_data.values()):
        print("  Skipping fig16_scaling_attribution (no AoI data)")
        return

    latencies = get_latencies(groups)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    for ax, metric_key, title in zip(axes,
            ['meas', 'gen'],
            ['$\\bar{\\Delta}_{meas}$ (ms)', '$\\bar{\\Delta}_{gen}$ (ms)']):
        for lat in latencies[:4]:
            for anch, ls in [('on', '-'), ('off', '--')]:
                means = []
                for n in ego_counts:
                    vals = scaling_data.get((anch, lat, n), {}).get(metric_key, [])
                    means.append(np.mean(vals) if vals else np.nan)
                ax.plot(ego_counts, means, marker='o', ls=ls,
                        label=f'lat={lat} anch={anch}', linewidth=1.5)
        ax.set_xlabel('Number of Egos (N)')
        ax.set_ylabel(title)
        ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])
        ax.legend(fontsize=6.5, loc='upper left')
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 16: Scaling Attribution ($\\Delta_{meas}$ vs $\\Delta_{gen}$)',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/fig16_scaling_attribution.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/fig16_scaling_attribution.eps', bbox_inches='tight')
    plt.close(fig)
    print("  Saved fig16_scaling_attribution")


# ── Updated Table 2: Extended LaTeX Summary with AoI ──────────────────

def table2_extended_latex(groups, ego_count=1):
    """Extended LaTeX table with Δ_meas, Δ_gen, and ego-gate metrics."""
    latencies = get_latencies(groups)
    rows = []

    for k in ALL_KEYS:
        mgr, anch = MGR_MAP[k]
        for lat in latencies:
            key = (mgr, anch, lat, ego_count)
            runs = groups.get(key, [])
            if not runs:
                continue
            n = len(runs)
            s_op = np.mean([r['S_op'] for r in runs])
            s_coll = np.mean([r['S_coll'] for r in runs])
            s_ghost = np.mean([r['S_ghost'] for r in runs])
            s_fp = np.mean([r['S_fp'] for r in runs])

            cats = [classify_run(r) for r in runs]
            from collections import Counter
            dom = Counter(cats).most_common(1)[0][0]

            # Δ_meas stats
            dm = [r['aoi_mean_ms'] for r in runs if r.get('aoi_mean_ms') is not None]
            dm99 = [r['aoi_p99_ms'] for r in runs if r.get('aoi_p99_ms') is not None]
            dm_str = f"{np.mean(dm):.0f}" if dm else "--"
            dm99_str = f"{np.mean(dm99):.0f}" if dm99 else "--"

            # Δ_gen stats
            dg = [r['delta_gen_mean_ms'] for r in runs if r.get('delta_gen_mean_ms') is not None]
            dg99 = [r['delta_gen_p99_ms'] for r in runs if r.get('delta_gen_p99_ms') is not None]
            dg_str = f"{np.mean(dg):.0f}" if dg else "--"
            dg99_str = f"{np.mean(dg99):.0f}" if dg99 else "--"

            # Ego-gate
            gate = np.mean([r.get('ego_gate_violations_total', 0) for r in runs])
            gate_str = f"{gate:.1f}"

            j = runs[0].get('jitter_std_ms') or 0
            rows.append(
                f"  {L[k]:<30} & {lat:>4} & {j:>4} & {n:>3} & "
                f"{s_op:.2f} & {s_coll:.2f} & {s_ghost:.2f} & {s_fp:.2f} & "
                f"{dom:<15} & {dm_str:>5} & {dm99_str:>5} & "
                f"{dg_str:>5} & {dg99_str:>5} & {gate_str:>5} \\\\")

    table = "\\begin{tabular}{lrrr|rrrr|lrr|rr|r}\n"
    table += ("  Config & Lat & Jit & N & $S_{op}$ & $S_{coll}$ & $S_{ghost}$ & $S_{fp}$ "
              "& Dom.~Failure & $\\bar{\\Delta}_m$ & $\\Delta_{m,99}$ "
              "& $\\bar{\\Delta}_g$ & $\\Delta_{g,99}$ & Gate \\\\\n")
    table += "  \\hline\n"
    table += "\n".join(rows) + "\n"
    table += "\\end{tabular}\n"

    out_path = f'{OUT}/table2_extended.tex'
    with open(out_path, 'w') as f:
        f.write(table)
    print(f"  Saved table2_extended.tex")


# ══════════════════════════════════════════════════════════════════════
#  EVALUATION FIGURE SET (12 figures — aligned to MobiCom paper)
#
#  Naming: eval_fig{N}_{short_name}
#  All-egos rule: system/edge/network properties use all egos
#  Focal rule:    safety outcomes use focal ego only
# ══════════════════════════════════════════════════════════════════════

def eval_fig1_delay_cdfs(groups):
    """Fig 1: Delay CDFs [All-egos].

    (a) CDF of configured + jitter delay per (latency, jitter) config.
    (b) CDF of measured AoI at publish boundary (Δ_meas) per config.
    Shows what the network actually delivers vs what was configured.
    """
    # Collect AoI samples from ALL egos (not focal-only)
    groups_all = load_sweep(focal_ego_only=False)

    config_aoi = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups_all.items():
        if mgr != 'late_fusion':
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            config_aoi[(anch, lat, j)].extend(r.get('delta_meas_samples', []))

    if not any(config_aoi.values()):
        print("  Skipping eval_fig1_delay_cdfs (no AoI data)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    cmap = plt.cm.viridis
    keys_sorted = sorted(config_aoi.keys())
    colors = cmap(np.linspace(0.1, 0.9, max(len(keys_sorted), 1)))

    # (a) Configured delay CDF (theoretical)
    for idx, (anch, lat, jit) in enumerate(keys_sorted):
        # Generate synthetic samples from configured distribution
        rng = np.random.default_rng(42 + idx)
        if jit > 0:
            synthetic = rng.normal(lat, jit, size=2000).clip(0)
        else:
            synthetic = np.full(2000, float(lat))
        s = np.sort(synthetic)
        ax1.plot(s, np.arange(1, len(s)+1)/len(s), color=colors[idx],
                 label=f'μ={lat} σ={jit} anch={anch}', linewidth=1.2)
    ax1.set_xlabel('Configured Delay (ms)')
    ax1.set_ylabel('CDF')
    ax1.set_title('(a) Configured Delay Distribution')
    ax1.legend(fontsize=5.5, loc='lower right')
    ax1.grid(True, alpha=0.2)

    # (b) Measured AoI CDF
    for idx, key in enumerate(keys_sorted):
        samples = config_aoi[key]
        if not samples:
            continue
        s = np.sort(samples)
        anch, lat, jit = key
        ax2.plot(s, np.arange(1, len(s)+1)/len(s), color=colors[idx],
                 label=f'μ={lat} σ={jit} anch={anch}', linewidth=1.2)
    ax2.set_xlabel('Measured $\\Delta_{meas}$ at Use (ms)')
    ax2.set_ylabel('CDF')
    ax2.set_title('(b) Measured AoI at Brake Decision')
    ax2.legend(fontsize=5.5, loc='lower right')
    ax2.grid(True, alpha=0.2)

    fig.suptitle('Fig 1: Delay CDFs [All-egos]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig1_delay_cdfs.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig1_delay_cdfs.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig1_delay_cdfs")


def eval_fig2_aoi_at_use_cdf(groups):
    """Fig 2: AoI-at-use CDF [All-egos + focal overlay].

    CDF of per-brake Δ_use for all egos, with focal ego overlaid.
    Panels per (manager, anchoring).
    """
    groups_all = load_sweep(focal_ego_only=False)

    mgr_labels = [('late_fusion', 'Late Fusion'), ('oracle', 'Oracle'), ('vips', 'VIPS')]
    anch_labels = [('on', '+ Anchoring'), ('off', 'no anchoring')]

    fig, axes = plt.subplots(len(mgr_labels), len(anch_labels),
                              figsize=(12, 10))

    for i, (mgr, mgr_name) in enumerate(mgr_labels):
        for j, (anch, anch_name) in enumerate(anch_labels):
            ax = axes[i][j]

            # All-egos samples
            all_samples = []
            focal_samples = []
            for (m, a, lat, ec), runs in groups_all.items():
                if m != mgr or a != anch:
                    continue
                for r in runs:
                    all_samples.extend(r.get('delta_meas_samples', []))
                    if r.get('ego_index', 0) == 0:
                        focal_samples.extend(r.get('delta_meas_samples', []))

            if all_samples:
                s = np.sort(all_samples)
                ax.plot(s, np.arange(1, len(s)+1)/len(s),
                        'b-', label='All egos', linewidth=1.5, alpha=0.7)
            if focal_samples:
                s = np.sort(focal_samples)
                ax.plot(s, np.arange(1, len(s)+1)/len(s),
                        'r--', label='Focal ego', linewidth=2)

            ax.set_title(f'{mgr_name} ({anch_name})', fontsize=10)
            ax.set_xlabel('$\\Delta_{use}$ (ms)')
            ax.set_ylabel('CDF')
            ax.legend(fontsize=8)
            ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 2: AoI-at-Use CDF [All-egos + focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig2_aoi_at_use_cdf.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig2_aoi_at_use_cdf.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig2_aoi_at_use_cdf")


def eval_fig3_tail_aoi_vs_n(groups):
    """Fig 3: Tail AoI inflation with N [All-egos].

    p99(Δ_use) vs N_ego at various fixed latency points.
    Shows that adding egos inflates tail AoI.
    """
    groups_all = load_sweep(focal_ego_only=False)
    ego_counts = get_ego_counts(groups_all)
    if len(ego_counts) < 2:
        print("  Skipping eval_fig3_tail_aoi_vs_n (need >=2 ego counts)")
        return

    latencies = get_latencies(groups_all)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    for anch, ax, title in [('on', ax1, '+ Anchoring'), ('off', ax2, 'no anchoring')]:
        for lat in latencies[:5]:
            p99s = []
            for n in ego_counts:
                all_samples = []
                key = ('late_fusion', anch, lat, n)
                for r in groups_all.get(key, []):
                    all_samples.extend(r.get('delta_meas_samples', []))
                if len(all_samples) >= 2:
                    p99s.append(np.percentile(all_samples, 99))
                else:
                    p99s.append(np.nan)
            ax.plot(ego_counts, p99s, 'o-', label=f'lat={lat}ms', linewidth=1.5)

        ax.set_xlabel('Number of Egos (N)')
        ax.set_ylabel('$p_{99}(\\Delta_{use})$ (ms)')
        ax.set_title(f'Late Fusion ({title})')
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])
        ax.legend(fontsize=7, loc='upper left')
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 3: Tail AoI Inflation with N [All-egos]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig3_tail_aoi_vs_n.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig3_tail_aoi_vs_n.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig3_tail_aoi_vs_n")


def eval_fig4_exposure_vs_n(groups):
    """Fig 4: Exposure/operating point vs N [Focal].

    Shows distance traveled, time alive, and run count vs N.
    Ensures multi-ego runs have comparable exposure before comparing safety.
    """
    ego_counts = get_ego_counts(groups)
    if len(ego_counts) < 2:
        print("  Skipping eval_fig4_exposure_vs_n (need >=2 ego counts)")
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    for anch, ls, mk in [('on', '-', 'o'), ('off', '--', 's')]:
        dist_means, time_means, run_counts = [], [], []
        for n in ego_counts:
            key = ('late_fusion', anch, 0, n)  # baseline latency=0
            runs = groups.get(key, [])
            # Try other latencies if 0ms not available
            if not runs:
                for lat in sorted(get_latencies(groups)):
                    key = ('late_fusion', anch, lat, n)
                    runs = groups.get(key, [])
                    if runs:
                        break

            dists = [r['distance_traveled_m'] for r in runs
                     if r.get('distance_traveled_m') is not None]
            times = [r['time_alive_s'] for r in runs
                     if r.get('time_alive_s') is not None]
            dist_means.append(np.mean(dists) if dists else np.nan)
            time_means.append(np.mean(times) if times else np.nan)
            run_counts.append(len(runs))

        label = f'Anchoring {anch}'
        axes[0].plot(ego_counts, dist_means, marker=mk, ls=ls, label=label, linewidth=2)
        axes[1].plot(ego_counts, time_means, marker=mk, ls=ls, label=label, linewidth=2)
        axes[2].bar([n + (0.2 if anch == 'on' else -0.2) for n in ego_counts],
                    run_counts, width=0.35,
                    label=label, alpha=0.7,
                    color='#1f77b4' if anch == 'on' else '#ff7f0e')

    axes[0].set_xlabel('N egos'); axes[0].set_ylabel('Distance (m)')
    axes[0].set_title('(a) Distance Traveled'); axes[0].grid(True, alpha=0.2)
    axes[0].legend(fontsize=8)

    axes[1].set_xlabel('N egos'); axes[1].set_ylabel('Time (s)')
    axes[1].set_title('(b) Simulation Time'); axes[1].grid(True, alpha=0.2)
    axes[1].legend(fontsize=8)

    axes[2].set_xlabel('N egos'); axes[2].set_ylabel('Runs')
    axes[2].set_title('(c) Run Count'); axes[2].grid(True, alpha=0.2)
    axes[2].legend(fontsize=8)

    for ax in axes[:2]:
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])

    fig.suptitle('Fig 4: Exposure & Operating Point vs N [Focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig4_exposure_vs_n.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig4_exposure_vs_n.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig4_exposure_vs_n")


def eval_fig5_safety_envelope(groups):
    """Fig 5: Safety envelope S_op vs p99(Δ_use) [Focal] — CORE RESULT.

    The main result: P(S_op=1) plotted directly against measured AoI tail.
    Tail freshness predicts failure, not configured mean latency.
    Shows the AoI safety boundary.
    """
    runs_with_aoi = []
    for (mgr, anch, lat, ec), runs in groups.items():
        for r in runs:
            p99 = r.get('aoi_p99_ms')
            if p99 is not None:
                runs_with_aoi.append({
                    'mgr': mgr, 'anch': anch, 'lat': lat, 'ec': ec,
                    'aoi_p99': p99,
                    'S_op': r['S_op'], 'S_ghost': r['S_ghost'],
                    'S_coll': r['S_coll'], 'S_clear': r['S_clear'],
                })

    if not runs_with_aoi:
        print("  Skipping eval_fig5_safety_envelope (no AoI data)")
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # (a) All configs scatter with binned means
    ax = axes[0]
    for k in ALL_KEYS:
        mgr, anch = MGR_MAP[k]
        pts = [r for r in runs_with_aoi if r['mgr'] == mgr and r['anch'] == anch]
        if not pts:
            continue
        x = [p['aoi_p99'] for p in pts]
        y = [p['S_op'] for p in pts]
        ax.scatter(x, y, color=C[k], marker=MARKERS[k], label=L[k],
                   alpha=0.5, s=25, edgecolors='black', linewidths=0.3)

    # Binned mean overlay (all data)
    all_p99 = [r['aoi_p99'] for r in runs_with_aoi]
    if len(set(all_p99)) > 3:
        bins = np.linspace(min(all_p99), max(all_p99), 12)
        centers = (bins[:-1] + bins[1:]) / 2
        means = []
        for lo, hi in zip(bins[:-1], bins[1:]):
            vals = [r['S_op'] for r in runs_with_aoi if lo <= r['aoi_p99'] < hi]
            means.append(np.mean(vals) if vals else np.nan)
        ax.plot(centers, means, 'k-', linewidth=2.5, label='Binned mean', zorder=10)

    ax.set_xlabel('Measured AoI $p_{99}$ (ms)')
    ax.set_ylabel('$\\Pr[S_{op}=1]$')
    ax.set_title('(a) Safety Envelope')
    ax.set_ylim(-0.05, 1.1)
    ax.legend(fontsize=6, loc='lower left')
    ax.grid(True, alpha=0.2)

    # (b) SLO threshold view: what p99 target gives P(S_op) >= SLO?
    ax = axes[1]
    slo_levels = [0.5, 0.7, 0.8, 0.9, 0.95]
    for k in ['lf_on', 'lf_off']:
        mgr, anch = MGR_MAP[k]
        pts = [r for r in runs_with_aoi if r['mgr'] == mgr and r['anch'] == anch]
        if len(pts) < 5:
            continue
        # For each SLO, find max p99 that achieves it
        pts_sorted = sorted(pts, key=lambda p: p['aoi_p99'])
        slo_thresholds = []
        for slo in slo_levels:
            # Scan from high p99 to low; find last bin where P(S_op) >= SLO
            best_p99 = 0
            for cutoff in np.linspace(min(all_p99), max(all_p99), 50):
                below = [p['S_op'] for p in pts if p['aoi_p99'] <= cutoff]
                if below and np.mean(below) >= slo:
                    best_p99 = cutoff
            slo_thresholds.append(best_p99)
        ax.plot(slo_levels, slo_thresholds, marker=MARKERS[k], ls=STYLES[k],
                color=C[k], label=L[k], linewidth=2, markersize=6)

    ax.set_xlabel('SLO: min $\\Pr[S_{op}=1]$')
    ax.set_ylabel('Max Tolerable AoI $p_{99}$ (ms)')
    ax.set_title('(b) AoI Budget per SLO')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 5: Safety Envelope — $S_{op}$ vs $p_{99}(\\Delta_{use})$ [Focal]',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig5_safety_envelope.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig5_safety_envelope.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig5_safety_envelope")


def eval_fig6_component_success_vs_aoi(groups):
    """Fig 6: Component success vs tail AoI [Focal] — APPENDIX.

    Individual S_coll, S_ghost, S_fp, S_clear each vs p99(AoI).
    Shows which component degrades first as AoI grows.
    """
    runs_with_aoi = []
    for (mgr, anch, lat, ec), runs in groups.items():
        if mgr != 'late_fusion':
            continue
        for r in runs:
            p99 = r.get('aoi_p99_ms')
            if p99 is not None:
                runs_with_aoi.append(r | {'aoi_p99': p99, 'anch': anch})

    if not runs_with_aoi:
        print("  Skipping eval_fig6_component_success_vs_aoi (no data)")
        return

    all_p99 = [r['aoi_p99'] for r in runs_with_aoi]
    bins = np.linspace(min(all_p99), max(all_p99), 10)
    centers = (bins[:-1] + bins[1:]) / 2

    components = [
        ('S_coll', '$S_{coll}$', '#377eb8'),
        ('S_ghost', '$S_{ghost}$', '#e41a1c'),
        ('S_fp', '$S_{fp}$', '#ff7f00'),
        ('S_clear', '$S_{clear}$', '#984ea3'),
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    for ax, anch, title in [(ax1, 'on', '+ Anchoring'), (ax2, 'off', 'no anchoring')]:
        pts = [r for r in runs_with_aoi if r['anch'] == anch]
        for metric, label, color in components:
            means = []
            for lo, hi in zip(bins[:-1], bins[1:]):
                vals = [r[metric] for r in pts if lo <= r['aoi_p99'] < hi]
                means.append(np.mean(vals) if vals else np.nan)
            ax.plot(centers, means, 'o-', label=label, color=color,
                    linewidth=2, markersize=5)
        ax.set_xlabel('Measured AoI $p_{99}$ (ms)')
        ax.set_ylabel('$\\Pr[S_x=1]$')
        ax.set_title(f'Late Fusion ({title})')
        ax.set_ylim(-0.05, 1.1)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 6: Component Success vs Tail AoI [Focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig6_component_success_vs_aoi.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig6_component_success_vs_aoi.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig6_component_success_vs_aoi")


def eval_fig7_regime_map(groups):
    """Fig 7: First-failure regime map [Focal] — CORE RESULT.

    2D heatmap of P(S_op=1) over (mean latency × jitter std).
    One panel per (manager, anchoring). Dominant failure class overlay.
    """
    # Re-use fig6_regime_map logic but with updated styling
    latencies = get_latencies(groups)

    regime_data = defaultdict(list)
    for (mgr, anch, lat, ec), runs in groups.items():
        if ec != 1:
            continue
        for r in runs:
            j = r.get('jitter_std_ms') or 0
            regime_data[(mgr, anch, lat, j)].append(r)

    jitters = sorted({r.get('jitter_std_ms') or 0 for runs in groups.values() for r in runs})
    if len(jitters) < 2:
        print("  Skipping eval_fig7_regime_map (need >=2 jitter values)")
        return

    mgr_labels = [('late_fusion', 'Late Fusion'), ('oracle', 'Oracle'), ('vips', 'VIPS')]
    anch_labels = [('on', '+ Anchoring'), ('off', 'no anchoring')]

    n_mgrs = sum(1 for m, _ in mgr_labels
                 if any(k[0] == m for k in groups.keys()))
    if n_mgrs == 0:
        print("  Skipping eval_fig7_regime_map (no data)")
        return

    fig, axes = plt.subplots(n_mgrs, len(anch_labels), figsize=(12, 4*n_mgrs))
    if n_mgrs == 1:
        axes = axes[np.newaxis, :]

    row = 0
    for mgr, mgr_name in mgr_labels:
        if not any(k[0] == mgr for k in groups.keys()):
            continue
        for j, (anch, anch_name) in enumerate(anch_labels):
            ax = axes[row][j]
            grid = np.full((len(jitters), len(latencies)), np.nan)

            for li, lat in enumerate(latencies):
                for ji, jit in enumerate(jitters):
                    runs = regime_data.get((mgr, anch, lat, jit), [])
                    if runs:
                        grid[ji, li] = np.mean([r['S_op'] for r in runs])

            im = ax.imshow(grid, origin='lower', aspect='auto',
                          vmin=0, vmax=1, cmap='RdYlGn',
                          extent=[latencies[0], latencies[-1],
                                  jitters[0], jitters[-1]])
            ax.set_title(f'{mgr_name} ({anch_name})', fontsize=10)
            ax.set_xlabel('Mean Latency (ms)')
            ax.set_ylabel('Jitter Std (ms)')

            # Dominant first-failure overlay
            for li, lat in enumerate(latencies):
                for ji, jit in enumerate(jitters):
                    runs = regime_data.get((mgr, anch, lat, jit), [])
                    if runs:
                        from collections import Counter
                        cats = [classify_run(r) for r in runs]
                        most = Counter(cats).most_common(1)[0][0]
                        short = {'SUCCESS': 'OK', 'FAIL_GHOST': 'G',
                                 'FAIL_OTHER_FP': 'FP', 'FAIL_PHYS': 'P',
                                 'FAIL_STUCK': 'S'}.get(most, '?')
                        ax.text(lat, jit, short, ha='center', va='center',
                                fontsize=7, fontweight='bold',
                                color='white' if grid[ji, li] < 0.5 else 'black')
        row += 1

    fig.colorbar(im, ax=axes, label='$\\Pr[S_{op}=1]$', shrink=0.6)
    fig.suptitle('Fig 7: First-Failure Regime Map [Focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig7_regime_map.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig7_regime_map.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig7_regime_map")


def eval_fig8_boundary_shift(groups):
    """Fig 8: Boundary shift with N [Focal] — CORE RESULT.

    SLO curves: for each N_ego, plot P(S_op=1) vs latency.
    Shows the safety boundary shifting left as N increases.
    """
    ego_counts = get_ego_counts(groups)
    if len(ego_counts) < 2:
        print("  Skipping eval_fig8_boundary_shift (need >=2 ego counts)")
        return

    latencies = get_latencies(groups)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    cmap = plt.cm.plasma
    n_colors = max(len(ego_counts), 1)

    for ax, anch, title in [(ax1, 'on', '+ Anchoring'), (ax2, 'off', 'no anchoring')]:
        for idx, n in enumerate(ego_counts):
            means = []
            for lat in latencies:
                key = ('late_fusion', anch, lat, n)
                runs = groups.get(key, [])
                vals = [r['S_op'] for r in runs]
                means.append(np.mean(vals) if vals else np.nan)
            color = cmap(idx / n_colors)
            ax.plot(latencies, means, 'o-', color=color,
                    label=f'N={n}', linewidth=2, markersize=5)

        # SLO line at P(S_op) = 0.8
        ax.axhline(0.8, color='gray', ls=':', linewidth=1, alpha=0.5)
        ax.text(latencies[-1], 0.82, 'SLO=0.8', fontsize=7, color='gray',
                ha='right')

        ax.set_xlabel('Configured Latency (ms)')
        ax.set_ylabel('$\\Pr[S_{op}=1]$')
        ax.set_title(f'Late Fusion ({title})')
        ax.set_ylim(-0.05, 1.1)
        ax.legend(fontsize=7, loc='lower left')
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 8: Safety Boundary Shift with N [Focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig8_boundary_shift.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig8_boundary_shift.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig8_boundary_shift")


def eval_fig9_brake_episodes_per_km(groups):
    """Fig 9: Brake episodes/km [Focal].

    Episode-normalized rate per km, split by GT class.
    """
    latencies = get_latencies(groups)
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    episode_types = [
        ('ghost_episodes', 'Self-Ghost', '#e41a1c'),
        ('fp_episodes', 'Other FP', '#ff7f00'),
        ('tp_episodes', 'True Positive', '#4daf4a'),
    ]

    for ax, anch, title in [(axes[0], 'on', '+ Anchoring'),
                             (axes[1], 'off', 'no anchoring')]:
        x = np.arange(len(latencies))
        bar_w = 0.25

        for bi, (metric, label, color) in enumerate(episode_types):
            rates = []
            for lat in latencies:
                key = ('late_fusion', anch, lat, 1)
                runs = groups.get(key, [])
                per_km = []
                for r in runs:
                    dist_km = (r.get('distance_traveled_m') or 0) / 1000.0
                    if dist_km > 0.01:
                        per_km.append(r.get(metric, 0) / dist_km)
                rates.append(np.mean(per_km) if per_km else 0)

            ax.bar(x + bi * bar_w, rates, bar_w, color=color, label=label)

        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Episodes / km')
        ax.set_title(f'Late Fusion ({title})')
        ax.set_xticks(x + bar_w)
        ax.set_xticklabels([str(l) for l in latencies])
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2, axis='y')

    fig.suptitle('Fig 9: Brake Episodes per km [Focal]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig9_brake_episodes_per_km.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig9_brake_episodes_per_km.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig9_brake_episodes_per_km")


def eval_fig10_geomdep_vs_n(groups):
    """Fig 10: GeomDup vs N [All-egos].

    Geometric duplicate clusters and tracks vs ego count.
    Shows anchoring-agnostic duplicate burden scaling.
    """
    groups_all = load_sweep(focal_ego_only=False)
    ego_counts = get_ego_counts(groups_all)
    if len(ego_counts) < 2:
        print("  Skipping eval_fig10_geomdep_vs_n (need >=2 ego counts)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    for k in ['lf_on', 'lf_off', 'oracle_on', 'oracle_off']:
        mgr, anch = MGR_MAP[k]
        cluster_means, track_means = [], []
        for n in ego_counts:
            key = (mgr, anch, 0, n)
            runs = groups_all.get(key, [])
            # Try other latencies if 0ms not available
            if not runs:
                for lat in sorted(get_latencies(groups_all)):
                    key = (mgr, anch, lat, n)
                    runs = groups_all.get(key, [])
                    if runs:
                        break

            clusters = [r.get('geom_dup_total_clusters', 0) for r in runs]
            tracks = [r.get('geom_dup_total_tracks', 0) for r in runs]
            cluster_means.append(np.mean(clusters) if clusters else np.nan)
            track_means.append(np.mean(tracks) if tracks else np.nan)

        ax1.plot(ego_counts, cluster_means, marker=MARKERS[k], ls=STYLES[k],
                 color=C[k], label=L[k], linewidth=2, markersize=5)
        ax2.plot(ego_counts, track_means, marker=MARKERS[k], ls=STYLES[k],
                 color=C[k], label=L[k], linewidth=2, markersize=5)

    ax1.set_xlabel('N egos'); ax1.set_ylabel('GeomDup Clusters')
    ax1.set_title('(a) Geometric Duplicate Clusters')
    ax1.legend(fontsize=7); ax1.grid(True, alpha=0.2)

    ax2.set_xlabel('N egos'); ax2.set_ylabel('GeomDup Tracks')
    ax2.set_title('(b) Tracks in Geometric Duplicates')
    ax2.legend(fontsize=7); ax2.grid(True, alpha=0.2)

    for ax in [ax1, ax2]:
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])

    fig.suptitle('Fig 10: Geometric Duplicates vs N [All-egos]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig10_geomdep_vs_n.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig10_geomdep_vs_n.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig10_geomdep_vs_n")


def eval_fig11_contract_violations(groups):
    """Fig 11: Contract violations A1/A2/A3 [All-egos] — CORE RESULT.

    Contract violation fractions vs N_ego or latency.
    A1=freshness, A2=ego-gate, A3=uniqueness.
    """
    groups_all = load_sweep(focal_ego_only=False)
    ego_counts = get_ego_counts(groups_all)
    latencies = get_latencies(groups_all)

    contracts = [
        ('contract_a1_fraction', 'A1: Freshness', '#377eb8'),
        ('contract_a2_fraction', 'A2: Ego-Gate', '#e41a1c'),
        ('contract_a3_fraction', 'A3: Uniqueness', '#4daf4a'),
    ]

    if len(ego_counts) >= 2:
        # (a) vs N, (b) vs latency
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

        # (a) Contract violations vs N (at fixed latency)
        ref_lat = latencies[len(latencies)//2] if latencies else 0
        for metric, label, color in contracts:
            means = []
            for n in ego_counts:
                key = ('late_fusion', 'on', ref_lat, n)
                runs = groups_all.get(key, [])
                vals = [r.get(metric, 0) for r in runs]
                means.append(np.mean(vals) if vals else np.nan)
            ax1.plot(ego_counts, means, 'o-', color=color, label=label,
                     linewidth=2, markersize=6)

        ax1.set_xlabel('N egos')
        ax1.set_ylabel('Violation Fraction')
        ax1.set_title(f'(a) vs N (lat={ref_lat}ms, LF+anch)')
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.2)
        if len(ego_counts) > 2:
            ax1.set_xscale('log', base=2)
        ax1.set_xticks(ego_counts)
        ax1.set_xticklabels([str(n) for n in ego_counts])

        # (b) Contract violations vs latency (at N=1)
        for metric, label, color in contracts:
            means = []
            for lat in latencies:
                key = ('late_fusion', 'on', lat, 1)
                runs = groups_all.get(key, [])
                vals = [r.get(metric, 0) for r in runs]
                means.append(np.mean(vals) if vals else np.nan)
            ax2.plot(latencies, means, 'o-', color=color, label=label,
                     linewidth=2, markersize=6)

        ax2.set_xlabel('Latency (ms)')
        ax2.set_ylabel('Violation Fraction')
        ax2.set_title('(b) vs Latency (N=1, LF+anch)')
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.2)

    else:
        # Only latency dimension
        fig, ax = plt.subplots(figsize=(8, 5))
        for metric, label, color in contracts:
            for anch, ls in [('on', '-'), ('off', '--')]:
                means = []
                for lat in latencies:
                    key = ('late_fusion', anch, lat, 1)
                    runs = groups_all.get(key, [])
                    vals = [r.get(metric, 0) for r in runs]
                    means.append(np.mean(vals) if vals else np.nan)
                ax.plot(latencies, means, marker='o', ls=ls, color=color,
                        label=f'{label} (anch={anch})', linewidth=2, markersize=5)
        ax.set_xlabel('Latency (ms)')
        ax.set_ylabel('Violation Fraction')
        ax.set_title('Contract Violations vs Latency')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.2)

    fig.suptitle('Fig 11: Contract Violations A1/A2/A3 [All-egos]',
                 fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig11_contract_violations.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig11_contract_violations.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig11_contract_violations")


def eval_fig12_system_overhead(groups):
    """Fig 12: System cost/overhead [All-egos] — APPENDIX.

    Edge timing, GPU utilization, CPU vs N or latency.
    """
    groups_all = load_sweep(focal_ego_only=False)
    ego_counts = get_ego_counts(groups_all)
    latencies = get_latencies(groups_all)

    timing_metrics = [
        ('edge_timing_avg_ms', 'Avg Frame (ms)', '#1f77b4'),
        ('edge_timing_p95_ms', 'P95 Frame (ms)', '#ff7f0e'),
        ('edge_timing_p99_ms', 'P99 Frame (ms)', '#2ca02c'),
    ]

    if len(ego_counts) >= 2:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

        # (a) Frame time vs N
        ax = axes[0]
        ref_lat = latencies[len(latencies)//2] if latencies else 0
        for metric, label, color in timing_metrics:
            means = []
            for n in ego_counts:
                key = ('late_fusion', 'on', ref_lat, n)
                runs = groups_all.get(key, [])
                vals = [r.get(metric) for r in runs if r.get(metric) is not None]
                means.append(np.mean(vals) if vals else np.nan)
            ax.plot(ego_counts, means, 'o-', color=color, label=label,
                    linewidth=2, markersize=5)
        ax.set_xlabel('N egos')
        ax.set_ylabel('Time (ms)')
        ax.set_title(f'(a) Edge Frame Time (lat={ref_lat}ms)')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])

        # (b) GPU util vs N
        ax = axes[1]
        for anch, ls, mk in [('on', '-', 'o'), ('off', '--', 's')]:
            means = []
            for n in ego_counts:
                key = ('late_fusion', anch, ref_lat, n)
                runs = groups_all.get(key, [])
                vals = [r.get('edge_gpu_util_pct') for r in runs
                        if r.get('edge_gpu_util_pct') is not None]
                means.append(np.mean(vals) if vals else np.nan)
            ax.plot(ego_counts, means, marker=mk, ls=ls,
                    label=f'Anch={anch}', linewidth=2)
        ax.set_xlabel('N egos')
        ax.set_ylabel('GPU Utilization (%)')
        ax.set_title('(b) GPU Utilization')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])

        # (c) CPU vs N
        ax = axes[2]
        for anch, ls, mk in [('on', '-', 'o'), ('off', '--', 's')]:
            means = []
            for n in ego_counts:
                key = ('late_fusion', anch, ref_lat, n)
                runs = groups_all.get(key, [])
                vals = [r.get('edge_cpu_pct') for r in runs
                        if r.get('edge_cpu_pct') is not None]
                means.append(np.mean(vals) if vals else np.nan)
            ax.plot(ego_counts, means, marker=mk, ls=ls,
                    label=f'Anch={anch}', linewidth=2)
        ax.set_xlabel('N egos')
        ax.set_ylabel('CPU (%)')
        ax.set_title('(c) CPU Utilization')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)
        if len(ego_counts) > 2:
            ax.set_xscale('log', base=2)
        ax.set_xticks(ego_counts)
        ax.set_xticklabels([str(n) for n in ego_counts])

    else:
        # Only latency dimension
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        for metric, label, color in timing_metrics:
            means = []
            for lat in latencies:
                key = ('late_fusion', 'on', lat, 1)
                runs = groups_all.get(key, [])
                vals = [r.get(metric) for r in runs if r.get(metric) is not None]
                means.append(np.mean(vals) if vals else np.nan)
            ax1.plot(latencies, means, 'o-', color=color, label=label,
                     linewidth=2, markersize=5)
        ax1.set_xlabel('Latency (ms)')
        ax1.set_ylabel('Time (ms)')
        ax1.set_title('(a) Edge Frame Time')
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.2)

        for anch, ls in [('on', '-'), ('off', '--')]:
            means = []
            for lat in latencies:
                key = ('late_fusion', anch, lat, 1)
                runs = groups_all.get(key, [])
                vals = [r.get('edge_gpu_util_pct') for r in runs
                        if r.get('edge_gpu_util_pct') is not None]
                means.append(np.mean(vals) if vals else np.nan)
            ax2.plot(latencies, means, 'o' + ls, label=f'Anch={anch}', linewidth=2)
        ax2.set_xlabel('Latency (ms)')
        ax2.set_ylabel('GPU Utilization (%)')
        ax2.set_title('(b) GPU Utilization')
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.2)

    fig.suptitle('Fig 12: System Overhead [All-egos]', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig12_system_overhead.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig12_system_overhead.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig12_system_overhead")


def eval_fig_mac_characterization(groups):
    """MAC characterization: PRR vs N, burst-length CDF, burstiness factor.

    Three-panel figure:
      (a) PRR vs N_ego -- lines per M value
      (b) Burst-length CDF per N_ego + geometric baseline
      (c) Burstiness factor (p95/mean) vs N -- proves bursty, not memoryless
    """
    # Collect runs with MAC data
    mac_runs = []
    for (mgr, anch, lat, ec), runs in groups.items():
        for r in runs:
            if r.get('mac_prr') is not None:
                mac_runs.append({
                    'mgr': mgr, 'anch': anch, 'lat': lat, 'ec': ec,
                    'mac_prr': r['mac_prr'],
                    'mac_burst_histogram': r.get('mac_burst_histogram', {}),
                    'mac_burst_mean': r.get('mac_burst_mean'),
                    'mac_burst_p95': r.get('mac_burst_p95'),
                    'mac_burstiness_factor': r.get('mac_burstiness_factor'),
                    'config_mac_M': r.get('config_mac_M'),
                    'config_mac_p_keep': r.get('config_mac_p_keep'),
                })

    if not mac_runs:
        print("  Skipping eval_fig_mac_characterization (no MAC data)")
        return

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # (a) PRR vs N_ego, lines per M value
    ax = axes[0]
    m_values = sorted(set(r['config_mac_M'] for r in mac_runs
                          if r['config_mac_M'] is not None))
    if not m_values:
        m_values = [None]
    for m_val in m_values:
        subset = [r for r in mac_runs
                  if r.get('config_mac_M') == m_val]
        ego_counts = sorted(set(r['ec'] for r in subset))
        prr_means = []
        for ec in ego_counts:
            vals = [r['mac_prr'] for r in subset if r['ec'] == ec]
            prr_means.append(np.mean(vals) if vals else np.nan)
        label = f'M={m_val}' if m_val is not None else 'default'
        ax.plot(ego_counts, prr_means, 'o-', label=label, linewidth=2)
    ax.set_xlabel('Number of Ego Vehicles (N)')
    ax.set_ylabel('Packet Reception Ratio')
    ax.set_title('(a) PRR vs N')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2)
    ax.set_ylim(0, 1.05)

    # (b) Burst-length CDF per N_ego + geometric baseline
    ax = axes[1]
    ego_counts = sorted(set(r['ec'] for r in mac_runs))
    colors = plt.cm.viridis(np.linspace(0.2, 0.8, len(ego_counts)))
    for i, ec in enumerate(ego_counts):
        # Merge histograms across runs for this ego count
        merged = defaultdict(int)
        for r in mac_runs:
            if r['ec'] == ec:
                for k, v in r.get('mac_burst_histogram', {}).items():
                    merged[int(k)] += v
        if not merged:
            continue
        sorted_lens = sorted(merged.keys())
        total = sum(merged.values())
        cdf = []
        cumul = 0
        for bl in sorted_lens:
            cumul += merged[bl]
            cdf.append(cumul / total)
        ax.step(sorted_lens, cdf, where='post', color=colors[i],
                label=f'N={ec}', linewidth=2)

        # Geometric baseline with same mean
        burst_mean = sum(bl * c for bl, c in merged.items()) / total
        if burst_mean > 0:
            p_geom = 1.0 / burst_mean
            geom_cdf = [1.0 - (1.0 - p_geom)**bl for bl in sorted_lens]
            ax.plot(sorted_lens, geom_cdf, '--', color=colors[i],
                    alpha=0.5, linewidth=1)
    ax.set_xlabel('Burst Length (consecutive drops)')
    ax.set_ylabel('CDF')
    ax.set_title('(b) Burst-Length CDF (dashed = geometric)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2)

    # (c) Burstiness factor vs N
    ax = axes[2]
    for m_val in m_values:
        subset = [r for r in mac_runs
                  if r.get('config_mac_M') == m_val]
        ego_counts_m = sorted(set(r['ec'] for r in subset))
        bf_means = []
        for ec in ego_counts_m:
            vals = [r['mac_burstiness_factor'] for r in subset
                    if r['ec'] == ec and r.get('mac_burstiness_factor') is not None]
            bf_means.append(np.mean(vals) if vals else np.nan)
        label = f'M={m_val}' if m_val is not None else 'default'
        ax.plot(ego_counts_m, bf_means, 's-', label=label, linewidth=2)
    ax.axhline(y=1.0, color='gray', linestyle=':', alpha=0.5,
               label='Memoryless (geometric)')
    ax.set_xlabel('Number of Ego Vehicles (N)')
    ax.set_ylabel('Burstiness Factor ($p_{95}$ / mean)')
    ax.set_title('(c) Burstiness Factor vs N')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.2)

    fig.suptitle('MAC Characterization (SB-SPS)', fontsize=13, y=1.02)
    fig.tight_layout()
    fig.savefig(f'{OUT}/eval_fig_mac_characterization.png', bbox_inches='tight')
    fig.savefig(f'{OUT}/eval_fig_mac_characterization.pdf', bbox_inches='tight')
    plt.close(fig)
    print("  Saved eval_fig_mac_characterization")


# ── Main ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import sys

    sweep_dir = sys.argv[1] if len(sys.argv) > 1 else SWEEP_DIR
    print(f"Loading sweep from {sweep_dir} ...")
    groups = load_sweep(sweep_dir)
    n_runs = sum(len(v) for v in groups.values())
    print(f"Loaded {n_runs} runs across {len(groups)} config groups")

    print_summary_table(groups)

    ego_counts = get_ego_counts(groups)
    print(f"Ego counts in data: {ego_counts}")

    print("\nGenerating figures (1-ego focus)...")
    fig0_trace_characterization()
    fig1_main_envelope(groups)
    fig1b_s_coll(groups)
    fig1c_s_ghost(groups)
    fig1d_s_fp(groups)
    fig1e_s_clear(groups)
    fig2_decomposition(groups)
    fig3_mechanism(groups)
    fig4_oracle(groups)
    for ec in ego_counts:
        fig5_first_failure(groups, ego_count=ec)
    fig6_regime_map(groups)
    fig7_provenance(groups)
    fig8_multi_ego_scaling(groups)
    fig9_sensitivity(groups)
    fig10_aoi_distribution(groups)
    fig11_delta_meas_vs_gen(groups)
    fig12_success_vs_aoi_tail(groups)
    fig13_same_mean_different_tail(groups)
    fig14_ego_gate_regime(groups)
    fig15_competing_risk_stability(groups)
    fig16_scaling_attribution(groups)
    table1_latex(groups)
    table2_extended_latex(groups)

    # ── Evaluation figure set (12 figures aligned to MobiCom paper) ──
    print("\nGenerating evaluation figure set (MobiCom 12-figure spec)...")
    print("  Main 10: 1,2,3,4,5,7,8,9,10,11  |  Appendix: 6,12")
    eval_fig1_delay_cdfs(groups)
    eval_fig2_aoi_at_use_cdf(groups)
    eval_fig3_tail_aoi_vs_n(groups)
    eval_fig4_exposure_vs_n(groups)
    eval_fig5_safety_envelope(groups)
    eval_fig6_component_success_vs_aoi(groups)   # appendix
    eval_fig7_regime_map(groups)
    eval_fig8_boundary_shift(groups)
    eval_fig9_brake_episodes_per_km(groups)
    eval_fig10_geomdep_vs_n(groups)
    eval_fig11_contract_violations(groups)
    eval_fig12_system_overhead(groups)            # appendix
    eval_fig_mac_characterization(groups)

    print(f"\nAll figures saved to {OUT}/")
