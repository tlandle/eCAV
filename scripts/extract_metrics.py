#!/usr/bin/env python3
"""Extract metrics from two experiment directories and produce a merged summary table."""

import json
import os
import glob
from collections import defaultdict
from pathlib import Path

OLD_DIR = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/experiment_results/openscenario_3_edge_late_fusion/20260220_235128"
NEW_DIR = "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/experiment_results/openscenario_3_edge_late_fusion/20260221_135032"


def find_metrics_files(base_dir):
    """Find all simulation_metrics.json files (not in evaluation_output/)."""
    pattern = os.path.join(base_dir, "**", "simulation_metrics.json")
    all_files = glob.glob(pattern, recursive=True)
    return [f for f in all_files if "evaluation_output" not in f]


def extract_run_metrics(filepath):
    """Extract all relevant metrics from a single simulation_metrics.json."""
    with open(filepath) as f:
        d = json.load(f)

    result = {}

    # Top-level config
    result["success_rate"] = d.get("success_rate", None)
    result["config_latency_ms"] = d.get("config_latency_ms", None)
    result["config_manager_type"] = d.get("config_manager_type", None)
    result["config_anchoring"] = d.get("config_anchoring", None)

    # Vehicle metrics (first vehicle)
    if d.get("vehicles"):
        veh = list(d["vehicles"].values())[0]
        result["total_brake_events"] = veh.get("total_brake_events", None)
        result["ghost_brake_events"] = veh.get("ghost_brake_events", None)
        result["ghost_brake_by_id"] = veh.get("ghost_brake_by_id", None)
        result["ghost_brake_by_proximity"] = veh.get("ghost_brake_by_proximity", None)
        result["false_brake_rate"] = veh.get("false_brake_rate", None)
        result["avg_speed_mps"] = veh.get("avg_speed_mps", None)
        result["collisions"] = veh.get("collisions", None)
        result["camera_disagreement_total_clusters"] = veh.get("camera_disagreement_total_clusters", None)
        result["camera_disagreement_frac_gt_4_5m"] = veh.get("camera_disagreement_frac_gt_4_5m", None)
        result["camera_disagreement_nms_leaks"] = veh.get("camera_disagreement_nms_leaks", None)

    # Safety summary
    if d.get("safety_summary"):
        result["total_collisions"] = d["safety_summary"].get("total_collisions", None)

    # Edge metrics (first edge)
    if d.get("edges"):
        edge = list(d["edges"].values())[0]
        result["ego_uniqueness_total_violations"] = edge.get("ego_uniqueness_total_violations", None)
        result["ego_uniqueness_violation_tick_fraction"] = edge.get("ego_uniqueness_violation_tick_fraction", None)
        result["spatial_self_id_failures"] = edge.get("spatial_self_id_failures", None)
        result["spatial_self_id_failure_rate"] = edge.get("spatial_self_id_failure_rate", None)

    # Source file for debugging
    result["_filepath"] = filepath

    return result


def group_key(run):
    """Create a grouping key: (manager_type, anchoring, latency_ms)."""
    mgr = run.get("config_manager_type", "unknown")
    anch = run.get("config_anchoring", None)
    anch_str = "on" if anch else "off"
    lat = int(run.get("config_latency_ms", 0))
    return (mgr, anch_str, lat)


def avg(values):
    """Average of non-None values."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    return sum(vals) / len(vals)


def fmt(val, decimals=2):
    """Format a value for display."""
    if val is None:
        return "N/A"
    if isinstance(val, float):
        return f"{val:.{decimals}f}"
    return str(val)


def fmt_int(val):
    """Format as integer or N/A."""
    if val is None:
        return "N/A"
    return f"{val:.1f}"


def print_table(title, groups, metrics_keys, col_headers, formatters=None):
    """Print a formatted table."""
    if formatters is None:
        formatters = {k: (lambda v: fmt(v, 2)) for k in metrics_keys}

    print(f"\n{'='*140}")
    print(f"  {title}")
    print(f"{'='*140}")

    # Build header
    key_header = f"{'Manager':<15} {'Anch':<5} {'Lat':>4}"
    col_str = "  ".join(f"{h:>12}" for h in col_headers)
    header = f"{key_header}  {col_str}  {'N':>3}"
    print(header)
    print("-" * len(header))

    sorted_keys = sorted(groups.keys(), key=lambda k: (k[0], k[1], k[2]))
    prev_mgr = None
    for key in sorted_keys:
        mgr, anch, lat = key
        runs = groups[key]
        n = len(runs)

        if prev_mgr is not None and mgr != prev_mgr:
            print()
        prev_mgr = mgr

        vals = []
        for mk in metrics_keys:
            raw_vals = [r.get(mk) for r in runs]
            mean = avg(raw_vals)
            vals.append(formatters.get(mk, lambda v: fmt(v, 2))(mean))

        val_str = "  ".join(f"{v:>12}" for v in vals)
        print(f"{mgr:<15} {anch:<5} {lat:>4}  {val_str}  {n:>3}")


def main():
    # --- Load all runs ---
    old_runs = [extract_run_metrics(f) for f in find_metrics_files(OLD_DIR)]
    new_runs = [extract_run_metrics(f) for f in find_metrics_files(NEW_DIR)]

    print(f"Loaded {len(old_runs)} runs from OLD directory")
    print(f"Loaded {len(new_runs)} runs from NEW directory")

    # --- Group by (manager, anchoring, latency) ---
    old_groups = defaultdict(list)
    for r in old_runs:
        old_groups[group_key(r)].append(r)

    new_groups = defaultdict(list)
    for r in new_runs:
        new_groups[group_key(r)].append(r)

    # Print group summaries
    print("\nOLD directory groups:")
    for k in sorted(old_groups.keys()):
        print(f"  {k}: {len(old_groups[k])} runs")

    print("\nNEW directory groups:")
    for k in sorted(new_groups.keys()):
        print(f"  {k}: {len(new_groups[k])} runs")

    # --- Merge: use NEW data for late_fusion + anchoring_off, OLD for everything else ---
    merged = {}
    for k, runs in old_groups.items():
        mgr, anch, lat = k
        if mgr == "late_fusion" and anch == "off":
            # Will be replaced by new data
            continue
        merged[k] = runs

    for k, runs in new_groups.items():
        merged[k] = runs

    # --- Print merged summary table ---
    metrics_keys = [
        "success_rate",
        "ghost_brake_events",
        "ghost_brake_by_proximity",
        "false_brake_rate",
        "avg_speed_mps",
        "total_collisions",
        "ego_uniqueness_total_violations",
        "spatial_self_id_failures",
    ]
    col_headers = [
        "success",
        "ghost_brk",
        "ghost_prox",
        "false_rate",
        "avg_spd",
        "collisions",
        "ego_uniq_v",
        "self_id_f",
    ]
    formatters = {
        "success_rate": lambda v: fmt(v, 2),
        "ghost_brake_events": lambda v: fmt_int(v),
        "ghost_brake_by_proximity": lambda v: fmt_int(v),
        "false_brake_rate": lambda v: fmt(v, 3),
        "avg_speed_mps": lambda v: fmt(v, 2),
        "total_collisions": lambda v: fmt_int(v),
        "ego_uniqueness_total_violations": lambda v: fmt_int(v),
        "spatial_self_id_failures": lambda v: fmt_int(v),
    }

    print_table(
        "MERGED SUMMARY TABLE (avg over 3 reps)",
        merged, metrics_keys, col_headers, formatters
    )

    # --- Camera disagreement metrics (new data only) ---
    cam_keys = [
        "camera_disagreement_total_clusters",
        "camera_disagreement_frac_gt_4_5m",
        "camera_disagreement_nms_leaks",
    ]
    cam_headers = [
        "cam_clusters",
        "cam_>4.5m",
        "cam_nms_leak",
    ]
    cam_formatters = {
        "camera_disagreement_total_clusters": lambda v: fmt_int(v),
        "camera_disagreement_frac_gt_4_5m": lambda v: fmt(v, 4),
        "camera_disagreement_nms_leaks": lambda v: fmt_int(v),
    }

    # Show camera disagreement only for groups that have these metrics
    cam_groups = {}
    for k, runs in merged.items():
        has_cam = any(r.get("camera_disagreement_total_clusters") is not None for r in runs)
        if has_cam:
            cam_groups[k] = runs

    if cam_groups:
        print_table(
            "CAMERA DISAGREEMENT METRICS (new runs only)",
            cam_groups, cam_keys, cam_headers, cam_formatters
        )

    # ===================================================================
    # SIDE-BY-SIDE COMPARISON: OLD vs NEW for late_fusion + anchoring_off
    # ===================================================================
    print(f"\n{'='*140}")
    print("  GHOST BRAKE ATTRIBUTION FIX: OLD vs NEW for late_fusion + anchoring_off")
    print(f"{'='*140}")

    compare_metrics = [
        "total_brake_events",
        "ghost_brake_events",
        "ghost_brake_by_id",
        "ghost_brake_by_proximity",
        "false_brake_rate",
        "avg_speed_mps",
        "total_collisions",
        "ego_uniqueness_total_violations",
        "spatial_self_id_failures",
        "camera_disagreement_total_clusters",
        "camera_disagreement_frac_gt_4_5m",
        "camera_disagreement_nms_leaks",
    ]
    compare_headers = [
        "tot_brk",
        "ghost_brk",
        "ghost_id",
        "ghost_prox",
        "false_rate",
        "avg_spd",
        "collis",
        "ego_uniq",
        "self_id_f",
        "cam_clust",
        "cam_>4.5m",
        "cam_nms",
    ]

    # Latencies to compare
    latencies = [0, 200, 400, 600]

    # Column widths
    cw = 10

    # Sub-header with metric names
    metric_header = "  ".join(f"{h:>{cw}}" for h in compare_headers)

    for lat in latencies:
        key = ("late_fusion", "off", lat)
        old_data = old_groups.get(key, [])
        new_data = new_groups.get(key, [])

        print(f"\n--- Latency: {lat}ms  (OLD: {len(old_data)} runs, NEW: {len(new_data)} runs) ---")
        print(f"{'':>8}  {metric_header}")

        # Old averages
        old_vals = []
        for mk in compare_metrics:
            raw = [r.get(mk) for r in old_data]
            mean = avg(raw)
            if mk == "false_brake_rate" or mk == "camera_disagreement_frac_gt_4_5m":
                old_vals.append(fmt(mean, 3))
            elif mk == "avg_speed_mps":
                old_vals.append(fmt(mean, 2))
            else:
                old_vals.append(fmt_int(mean))
        old_line = "  ".join(f"{v:>{cw}}" for v in old_vals)
        print(f"{'OLD':>8}  {old_line}")

        # New averages
        new_vals = []
        for mk in compare_metrics:
            raw = [r.get(mk) for r in new_data]
            mean = avg(raw)
            if mk == "false_brake_rate" or mk == "camera_disagreement_frac_gt_4_5m":
                new_vals.append(fmt(mean, 3))
            elif mk == "avg_speed_mps":
                new_vals.append(fmt(mean, 2))
            else:
                new_vals.append(fmt_int(mean))
        new_line = "  ".join(f"{v:>{cw}}" for v in new_vals)
        print(f"{'NEW':>8}  {new_line}")

        # Delta row
        delta_vals = []
        for i, mk in enumerate(compare_metrics):
            old_raw = [r.get(mk) for r in old_data]
            new_raw = [r.get(mk) for r in new_data]
            old_mean = avg(old_raw)
            new_mean = avg(new_raw)
            if old_mean is not None and new_mean is not None:
                diff = new_mean - old_mean
                if abs(diff) < 0.001:
                    delta_vals.append(f"{'=':>{cw}}")
                elif diff > 0:
                    delta_vals.append(f"{'+'}{fmt(diff, 2):>{cw-1}}")
                else:
                    delta_vals.append(f"{fmt(diff, 2):>{cw}}")
            else:
                delta_vals.append(f"{'--':>{cw}}")
        delta_line = "  ".join(delta_vals)
        print(f"{'DELTA':>8}  {delta_line}")

    # ===================================================================
    # Per-run detail for the comparison (to see variance)
    # ===================================================================
    print(f"\n{'='*140}")
    print("  PER-RUN DETAIL: late_fusion + anchoring_off (NEW directory)")
    print(f"{'='*140}")

    detail_metrics = [
        "total_brake_events",
        "ghost_brake_events",
        "ghost_brake_by_id",
        "ghost_brake_by_proximity",
        "false_brake_rate",
        "collisions",
        "camera_disagreement_total_clusters",
        "camera_disagreement_nms_leaks",
    ]
    detail_headers = [
        "tot_brk",
        "ghost",
        "g_id",
        "g_prox",
        "f_rate",
        "collis",
        "cam_clust",
        "cam_nms",
    ]

    dw = 10
    dhdr = "  ".join(f"{h:>{dw}}" for h in detail_headers)
    print(f"{'Lat':>5} {'Run':>4}  {dhdr}")
    print("-" * (5 + 4 + 2 + len(detail_headers) * (dw + 2)))

    for lat in latencies:
        key = ("late_fusion", "off", lat)
        runs = new_groups.get(key, [])
        for i, r in enumerate(sorted(runs, key=lambda x: x["_filepath"])):
            vals = []
            for mk in detail_metrics:
                v = r.get(mk)
                if mk == "false_brake_rate":
                    vals.append(fmt(v, 3))
                elif v is not None:
                    if isinstance(v, float):
                        vals.append(fmt(v, 1))
                    else:
                        vals.append(str(v))
                else:
                    vals.append("N/A")
            val_line = "  ".join(f"{v:>{dw}}" for v in vals)
            print(f"{lat:>5} {i+1:>4}  {val_line}")
        if lat != 600:
            print()


if __name__ == "__main__":
    main()
