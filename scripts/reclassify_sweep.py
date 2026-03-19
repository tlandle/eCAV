#!/usr/bin/env python3
"""
Reclassify S_op from existing sweep data without re-running simulations.

Two fixes applied:
1. GT 3m distance gate: brake attributions where gt_matched_actor_id == ego
   AND gt_match_dist_m > 3.0 are reclassified from self_ghost -> other_fp.
2. Focal ego S_op: for multi-ego runs (N>1), S_op is recomputed using only
   the focal ego (lowest actor_id) instead of aggregating all vehicles.

Original files are backed up to simulation_metrics.json.bak before overwriting.
"""

import json
import os
import glob
import shutil
import sys
from collections import defaultdict

SWEEP_DIRS = [
    # Single-ego sweep
    "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/opencda/scenario_testing/evaluation_outputs/20260311_162505",
    # Multi-ego sweep
    "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox/opencda/scenario_testing/evaluation_outputs/20260311_171848",
]

SELF_GHOST_MAX_DIST_M = 3.0
TARGET_SPEED_KMH = 43
S_PROG_THRESHOLD = 0.6  # avg_speed >= 60% of target


def find_metrics_files(base_dir):
    pattern = os.path.join(base_dir, "**", "simulation_metrics.json")
    all_files = sorted(glob.glob(pattern, recursive=True))
    # Filter out copies nested inside evaluation_output/ subdirectories,
    # but keep files under evaluation_outputs/ (the top-level sweep dir).
    return [f for f in all_files if "/evaluation_output/" not in f]


def reclassify_gt_brakes(d):
    """Apply 3m distance gate to GT brake classifications.

    Returns (changed_count, reclassified_details) where changed_count is
    the number of brake attributions reclassified from self_ghost to other_fp.
    """
    vehicles = d.get("vehicles", {})
    changed = 0
    details = []

    for vid, vdata in vehicles.items():
        attribs = vdata.get("brake_attributions", [])
        ghost_reclassified = 0

        for attr in attribs:
            if (attr.get("gt_brake_class") == "self_ghost"
                    and attr.get("gt_match_dist_m", 0) > SELF_GHOST_MAX_DIST_M):
                details.append({
                    "vehicle": vid,
                    "old_class": "self_ghost",
                    "new_class": "other_fp",
                    "dist_m": attr["gt_match_dist_m"],
                    "matched_actor": attr.get("gt_matched_actor_id"),
                })
                attr["gt_brake_class"] = "other_fp"
                attr["gt_provenance"] = "reclassified_distance_gate"
                ghost_reclassified += 1
                changed += 1

        if ghost_reclassified > 0:
            # Recount per-vehicle totals from attributions
            ghost_count = sum(1 for a in attribs if a.get("gt_brake_class") == "self_ghost")
            fp_count = sum(1 for a in attribs if a.get("gt_brake_class") == "other_fp")
            tp_count = sum(1 for a in attribs if a.get("gt_brake_class") == "true_positive")
            vdata["ghost_brake_gt"] = ghost_count
            vdata["other_fp_gt"] = fp_count
            vdata["true_positive_gt"] = tp_count
            vdata["gt_labeled_count"] = ghost_count + fp_count + tp_count

    # Update top-level totals
    if changed > 0:
        total_ghost = sum(v.get("ghost_brake_gt", 0) for v in vehicles.values())
        total_fp = sum(v.get("other_fp_gt", 0) for v in vehicles.values())
        total_tp = sum(v.get("true_positive_gt", 0) for v in vehicles.values())
        d["total_ghost_brake_gt"] = total_ghost
        d["total_other_fp_gt"] = total_fp
        d["total_tp_gt"] = total_tp

    return changed, details


def recompute_sop(d, focal_only=False):
    """Recompute S_op components from per-vehicle data.

    If focal_only=True, use only the focal ego (lowest actor_id).
    Otherwise, aggregate across all vehicles (original behavior).

    Returns dict with old and new S_op values.
    """
    vehicles = d.get("vehicles", {})
    if not vehicles:
        return {"changed": False}

    target_mps = TARGET_SPEED_KMH / 3.6

    old_sop = {k: d.get(k) for k in ["s_coll", "s_ghost", "s_fp", "s_prog", "s_op"]}

    if focal_only and len(vehicles) > 1:
        # Find focal ego: lowest actor_id
        focal_vid = min(vehicles.keys(), key=lambda v: vehicles[v].get("actor_id", int(v)))
        focal = vehicles[focal_vid]
        focal_id = focal.get("actor_id", int(focal_vid))

        s_coll = 1 if focal.get("collisions", 0) == 0 else 0
        s_ghost = 1 if focal.get("ghost_brake_gt", 0) == 0 else 0
        s_fp = 1 if focal.get("other_fp_gt", 0) == 0 else 0
        s_prog = 1 if focal.get("avg_speed_mps", 0) >= S_PROG_THRESHOLD * target_mps else 0

        d["focal_vehicle_id"] = focal_id
        # Recompute top-level totals from focal only
        d["focal_collisions"] = focal.get("collisions", 0)
        d["focal_ghost_brake_gt"] = focal.get("ghost_brake_gt", 0)
        d["focal_other_fp_gt"] = focal.get("other_fp_gt", 0)
        d["focal_true_positive_gt"] = focal.get("true_positive_gt", 0)
        d["focal_total_brake_events"] = focal.get("total_brake_events", 0)
        d["focal_avg_speed_mps"] = focal.get("avg_speed_mps", 0)
    else:
        # Single ego or aggregate: use totals
        s_coll = 1 if d.get("collision_count", 0) == 0 else 0
        s_ghost = 1 if d.get("total_ghost_brake_gt", 0) == 0 else 0
        s_fp = 1 if d.get("total_other_fp_gt", 0) == 0 else 0
        # For single ego, use the vehicle's avg_speed
        if len(vehicles) == 1:
            vid = list(vehicles.keys())[0]
            avg_speed = vehicles[vid].get("avg_speed_mps", 0)
        else:
            avg_speed = d.get("overall_avg_speed_mps", 0)
        s_prog = 1 if avg_speed >= S_PROG_THRESHOLD * target_mps else 0

    s_op = int(s_coll and s_ghost and s_fp and s_prog)

    d["s_coll"] = s_coll
    d["s_ghost"] = s_ghost
    d["s_fp"] = s_fp
    d["s_prog"] = s_prog
    d["s_op"] = s_op

    new_sop = {"s_coll": s_coll, "s_ghost": s_ghost, "s_fp": s_fp, "s_prog": s_prog, "s_op": s_op}
    changed = any(old_sop.get(k) != new_sop[k] for k in new_sop)

    return {"changed": changed, "old": old_sop, "new": new_sop}


def process_file(fpath, dry_run=False):
    """Process a single simulation_metrics.json file."""
    with open(fpath) as f:
        d = json.load(f)

    # Determine if multi-ego
    n_vehicles = len(d.get("vehicles", {}))
    is_multi = n_vehicles > 1

    # Step 1: reclassify GT brakes (3m distance gate)
    gt_changed, gt_details = reclassify_gt_brakes(d)

    # Step 2: recompute S_op (focal ego for multi-ego)
    sop_result = recompute_sop(d, focal_only=is_multi)

    any_change = gt_changed > 0 or sop_result.get("changed", False)

    if any_change and not dry_run:
        # Backup original
        bak = fpath + ".bak"
        if not os.path.exists(bak):
            shutil.copy2(fpath, bak)
        # Write updated
        with open(fpath, "w") as f:
            json.dump(d, f, indent=4)

    return {
        "path": fpath,
        "n_vehicles": n_vehicles,
        "gt_reclassified": gt_changed,
        "gt_details": gt_details,
        "sop_changed": sop_result.get("changed", False),
        "sop_old": sop_result.get("old"),
        "sop_new": sop_result.get("new"),
        "any_change": any_change,
    }


def main():
    dry_run = "--dry-run" in sys.argv

    if dry_run:
        print("=== DRY RUN MODE (no files modified) ===\n")

    total_files = 0
    total_changed = 0
    total_gt_reclassified = 0
    total_sop_changed = 0

    for base_dir in SWEEP_DIRS:
        if not os.path.isdir(base_dir):
            print(f"SKIP: {base_dir} not found")
            continue

        sweep_name = os.path.basename(base_dir)
        files = find_metrics_files(base_dir)
        print(f"\n{'='*80}")
        print(f"Sweep: {sweep_name} ({len(files)} files)")
        print(f"{'='*80}")

        for fpath in files:
            total_files += 1
            result = process_file(fpath, dry_run=dry_run)

            run_name = os.path.basename(os.path.dirname(fpath))

            if result["any_change"]:
                total_changed += 1
                print(f"\n  CHANGED: {run_name}")
                print(f"    vehicles: {result['n_vehicles']}")

                if result["gt_reclassified"] > 0:
                    total_gt_reclassified += result["gt_reclassified"]
                    print(f"    GT reclassified: {result['gt_reclassified']} brakes (self_ghost -> other_fp)")
                    for det in result["gt_details"]:
                        print(f"      veh {det['vehicle']}: dist={det['dist_m']:.1f}m, matched_actor={det['matched_actor']}")

                if result["sop_changed"]:
                    total_sop_changed += 1
                    old = result["sop_old"]
                    new = result["sop_new"]
                    changes = [k for k in new if old.get(k) != new[k]]
                    print(f"    S_op changed: {', '.join(f'{k}: {old.get(k)}->{new[k]}' for k in changes)}")
            else:
                # Compact output for unchanged files
                pass

    print(f"\n{'='*80}")
    print(f"SUMMARY")
    print(f"{'='*80}")
    print(f"  Files processed:     {total_files}")
    print(f"  Files changed:       {total_changed}")
    print(f"  GT brakes reclassified: {total_gt_reclassified}")
    print(f"  S_op values changed: {total_sop_changed}")
    if dry_run:
        print(f"\n  (dry run: no files were modified)")


if __name__ == "__main__":
    main()
