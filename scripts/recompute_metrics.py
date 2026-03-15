#!/usr/bin/env python3
"""
Recompute sweep metrics from existing simulation_metrics.json files.

Changes from the in-evaluator computation:
  1. Focal-ego only: metrics computed for vehicle index 0 (hero), not aggregated.
  2. S_prog replaced with S_clear: pass unless stuck or collision (not speed-based).
  3. S_op = S_coll & S_ghost & S_fp & S_clear
  4. Per-ego breakdown available for multi-ego analysis.
"""

import json
import os
import sys
import numpy as np
from collections import defaultdict


def load_sweep(base_dir, exclude_ego_counts=None):
    """Load all simulation_metrics.json under base_dir into a list of dicts."""
    exclude_ego_counts = exclude_ego_counts or set()
    runs = []
    for root, dirs, files in os.walk(base_dir):
        if "simulation_metrics.json" not in files:
            continue
        if "evaluation_output" in root:
            continue
        path = os.path.join(root, "simulation_metrics.json")
        try:
            with open(path) as f:
                d = json.load(f)
        except Exception:
            continue
        ego_count = d.get("config_ego_count", 1)
        if ego_count in exclude_ego_counts:
            continue
        d["_path"] = path
        runs.append(d)
    return runs


def focal_ego(d):
    """Return the vehicle dict for the focal ego (lowest actor ID = hero)."""
    vehicles = d.get("vehicles", {})
    if not vehicles:
        return None
    # Hero is the lowest-numbered actor ID (first spawned)
    focal_vid = min(vehicles.keys(), key=lambda k: int(k))
    return vehicles[focal_vid], focal_vid


def _count_episodes(brake_attributions, gap=2):
    """
    Count brake episodes by GT class.

    An episode = contiguous ticks (gap <= `gap`) with the same track_id
    and same gt_brake_class.

    FP ticks that are concurrent with TP ticks (within `gap`) are filtered
    out: if the ego is already braking for a real hazard, a co-occurring FP
    does not cause additional unwanted behavior.

    Returns (ghost_episodes, fp_episodes, tp_episodes).
    """
    by_class = defaultdict(list)
    for a in brake_attributions:
        cls = a.get("gt_brake_class", "unknown")
        tick = a.get("trigger_tick")
        track = a.get("trigger_track_id")
        if tick is not None:
            by_class[cls].append((tick, track))

    # Build set of TP-active ticks (expanded by gap for episode overlap)
    tp_ticks_raw = set(t for t, _ in by_class.get("true_positive", []))
    tp_ticks_expanded = set()
    for t in tp_ticks_raw:
        for dt in range(-gap, gap + 1):
            tp_ticks_expanded.add(t + dt)

    # Filter FP events: remove ticks concurrent with TP
    fp_filtered = [(t, tr) for t, tr in by_class.get("other_fp", [])
                   if t not in tp_ticks_expanded]

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

    ghost = count_eps(by_class.get("self_ghost", []))
    fp = count_eps(fp_filtered)
    tp = count_eps(by_class.get("true_positive", []))
    return ghost, fp, tp


def compute_run_metrics(d):
    """Compute corrected per-run metrics using focal ego only."""
    result = {}

    # Config
    result["mgr"] = d.get("config_manager_type", "?")
    result["anch"] = "on" if d.get("config_anchoring") else "off"
    result["lat"] = d.get("config_latency_ms", 0)
    result["ego_count"] = d.get("config_ego_count", 1)

    foc = focal_ego(d)
    if foc is None:
        return None
    fv, focal_vid = foc
    result["focal_vid"] = focal_vid

    # --- Safety (focal ego only) ---
    focal_safety = fv.get("safety", {})
    focal_coll = focal_safety.get("collision_count", 0)
    result["focal_collisions"] = focal_coll
    result["focal_stuck"] = focal_safety.get("stuck", False)
    result["focal_offroad"] = focal_safety.get("offroad", False)

    # S_coll: focal ego had no collisions
    result["s_coll"] = 1 if focal_coll == 0 else 0

    # --- Brake classifications (focal ego only, EPISODE-based) ---
    # Post-hoc reclassification: if GT matched the ego actor, it is a
    # self-ghost regardless of distance.  The runtime 3m gate leaked
    # high-latency ego-ghosts into other_fp.
    ba = fv.get("brake_attributions", [])
    ego_actor_id = int(focal_vid)
    reclassified = 0
    for attr in ba:
        if (attr.get("gt_matched_actor_id") == ego_actor_id
                and attr.get("gt_brake_class") != "self_ghost"):
            attr["gt_brake_class"] = "self_ghost"
            attr["gt_provenance"] = "self"
            reclassified += 1

    # Recount tick totals after reclassification
    ghost_ticks = sum(1 for a in ba if a.get("gt_brake_class") == "self_ghost")
    fp_ticks = sum(1 for a in ba if a.get("gt_brake_class") == "other_fp")
    tp_ticks = sum(1 for a in ba if a.get("gt_brake_class") == "true_positive")
    result["focal_ghost_ticks"] = ghost_ticks
    result["focal_fp_ticks"] = fp_ticks
    result["focal_tp_ticks"] = tp_ticks
    result["focal_total_brake_ticks"] = fv.get("total_brake_events", 0)

    # Episode counting: consecutive ticks with same track_id = 1 episode
    # Gap of >2 ticks or different track_id starts a new episode
    ghost_eps, fp_eps, tp_eps = _count_episodes(ba)
    result["focal_ghost_episodes"] = ghost_eps
    result["focal_fp_episodes"] = fp_eps
    result["focal_tp_episodes"] = tp_eps
    result["focal_total_episodes"] = ghost_eps + fp_eps + tp_eps

    # S_ghost: focal ego had no self-ghost EPISODES
    result["s_ghost"] = 1 if ghost_eps == 0 else 0

    # S_fp: focal ego had no other-FP EPISODES
    result["s_fp"] = 1 if fp_eps == 0 else 0

    # --- S_clear: task completion (replaces S_prog) ---
    # Pass unless focal ego is stuck or had a collision
    result["s_clear"] = 1 if (not result["focal_stuck"] and focal_coll == 0) else 0

    # --- S_op: operational success ---
    result["s_op"] = result["s_coll"] & result["s_ghost"] & result["s_fp"] & result["s_clear"]

    # --- Continuous metrics ---
    result["focal_avg_speed_mps"] = fv.get("avg_speed_mps", 0)

    # Edge metrics (scenario-level, not per-ego)
    for eid, edata in d.get("edges", {}).items():
        result["edge_fps"] = edata.get("frames_per_second", 0)
        result["edge_timing_avg_ms"] = edata.get("timing_avg_ms", 0)
        result["ego_uniqueness_violations"] = edata.get("ego_uniqueness_total_violations", 0)
        result["ego_ghost_tracks"] = edata.get("ego_uniqueness_total_ego_ghost_tracks", 0)
        result["detection_precision"] = edata.get("detection_precision", 0)
        result["detection_recall"] = edata.get("detection_recall", 0)
        result["tracking_mota"] = edata.get("tracking_avg_mota", 0)
        result["prediction_ade_1s"] = edata.get("prediction_ade_1s_m", 0)
        result["prediction_fde"] = edata.get("prediction_fde_m", 0)
        break  # Only one edge

    # AoI from brake attributions (focal ego)
    aoi_list = []
    for a in fv.get("brake_attributions", []):
        src = a.get("source_tick")
        trg = a.get("trigger_tick")
        if src is not None and trg is not None:
            aoi_ticks = trg - src
            aoi_list.append(aoi_ticks * 50)  # Convert to ms (dt=0.05s)
    if aoi_list:
        result["aoi_mean_ms"] = float(np.mean(aoi_list))
        result["aoi_p95_ms"] = float(np.percentile(aoi_list, 95))
        result["aoi_p99_ms"] = float(np.percentile(aoi_list, 99))
    else:
        result["aoi_mean_ms"] = None
        result["aoi_p95_ms"] = None
        result["aoi_p99_ms"] = None

    # Provenance breakdown (focal ego)
    prov_counts = defaultdict(int)
    for a in fv.get("brake_attributions", []):
        prov = a.get("gt_provenance", "unknown")
        prov_counts[prov] += 1
    result["prov_self"] = prov_counts.get("self", 0)
    result["prov_cross_ego"] = prov_counts.get("cross_ego", 0)
    result["prov_npc_hazard"] = prov_counts.get("npc_hazard", 0)
    result["prov_parked"] = prov_counts.get("parked", 0)
    result["prov_npc_non_hazard"] = prov_counts.get("npc_non_hazard", 0)
    result["prov_unknown"] = prov_counts.get("unknown", 0)

    return result


def print_table(groups, title=""):
    """Print a formatted summary table."""
    if title:
        print(f"\n=== {title} ===")
    header = (f"{'mgr':<12} {'anch':<5} {'lat':>4} {'ego':>3} | "
              f"{'N':>3} {'P(op)':>6} {'P(col)':>6} {'P(gho)':>6} {'P(fp)':>6} {'P(clr)':>6} | "
              f"{'gho_e':>5} {'fp_e':>5} {'tp_e':>5} {'tot_e':>5} | "
              f"{'gho_t':>5} {'fp_t':>5} {'tp_t':>5} | "
              f"{'uniq_v':>6} {'spd':>5}")
    print(header)
    print("-" * len(header))

    for key in sorted(groups.keys()):
        runs = groups[key]
        n = len(runs)
        mgr, anch, lat, ego = key

        def avg(field):
            vals = [r[field] for r in runs if r.get(field) is not None]
            return np.mean(vals) if vals else 0

        print(f"{mgr:<12} {anch:<5} {lat:>4.0f} {ego:>3} | "
              f"{n:>3} "
              f"{avg('s_op'):>6.2f} "
              f"{avg('s_coll'):>6.2f} "
              f"{avg('s_ghost'):>6.2f} "
              f"{avg('s_fp'):>6.2f} "
              f"{avg('s_clear'):>6.2f} | "
              f"{avg('focal_ghost_episodes'):>5.1f} "
              f"{avg('focal_fp_episodes'):>5.1f} "
              f"{avg('focal_tp_episodes'):>5.1f} "
              f"{avg('focal_total_episodes'):>5.1f} | "
              f"{avg('focal_ghost_ticks'):>5.1f} "
              f"{avg('focal_fp_ticks'):>5.1f} "
              f"{avg('focal_tp_ticks'):>5.1f} | "
              f"{avg('ego_uniqueness_violations'):>6.0f} "
              f"{avg('focal_avg_speed_mps'):>5.1f}")


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <sweep_dir> [--exclude-ego N,N,...]")
        sys.exit(1)

    base_dir = sys.argv[1]
    exclude = set()
    for i, arg in enumerate(sys.argv):
        if arg == "--exclude-ego" and i + 1 < len(sys.argv):
            exclude = {int(x) for x in sys.argv[i + 1].split(",")}

    print(f"Loading from: {base_dir}")
    print(f"Excluding ego counts: {exclude or 'none'}")

    raw_runs = load_sweep(base_dir, exclude_ego_counts=exclude)
    print(f"Loaded {len(raw_runs)} raw run files")

    # Compute corrected metrics
    runs = []
    for d in raw_runs:
        r = compute_run_metrics(d)
        if r is not None:
            runs.append(r)
    print(f"Computed metrics for {len(runs)} runs")

    # Group by config
    groups = defaultdict(list)
    for r in runs:
        key = (r["mgr"], r["anch"], r["lat"], r["ego_count"])
        groups[key].append(r)

    # Print by ego count
    for ego_n in sorted({k[3] for k in groups}):
        ego_groups = {k: v for k, v in groups.items() if k[3] == ego_n}
        print_table(ego_groups, title=f"{ego_n}-EGO (focal ego only)")

    # First-failure decomposition for 1-ego
    print("\n=== FIRST-FAILURE DECOMPOSITION (1-ego, focal) ===")
    print(f"{'mgr':<12} {'anch':<5} {'lat':>4} | {'N':>3} "
          f"{'OK':>5} {'COLL':>5} {'GHOST':>5} {'FP':>5} {'STUCK':>5}")
    print("-" * 70)
    for key in sorted(groups.keys()):
        mgr, anch, lat, ego = key
        if ego != 1:
            continue
        runs_k = groups[key]
        n = len(runs_k)
        ok = sum(1 for r in runs_k if r["s_op"] == 1)
        coll = sum(1 for r in runs_k if r["s_coll"] == 0)
        # First failure priority: collision > ghost > fp
        ghost_first = sum(1 for r in runs_k
                         if r["s_coll"] == 1 and r["s_ghost"] == 0)
        fp_first = sum(1 for r in runs_k
                      if r["s_coll"] == 1 and r["s_ghost"] == 1 and r["s_fp"] == 0)
        stuck = sum(1 for r in runs_k
                   if r["s_coll"] == 1 and r["s_ghost"] == 1 and r["s_fp"] == 1
                   and r["s_clear"] == 0)
        print(f"{mgr:<12} {anch:<5} {lat:>4.0f} | {n:>3} "
              f"{ok/n:>5.0%} {coll/n:>5.0%} {ghost_first/n:>5.0%} "
              f"{fp_first/n:>5.0%} {stuck/n:>5.0%}")


if __name__ == "__main__":
    main()
