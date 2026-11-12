#!/usr/bin/env python
"""First difference between two fixed-seed runs, by quantity.

    python scripts/ispass/repeat_first_diff.py <stampA> <stampB> [--root experiment_results/<geometry>]

Reads per_tick_age.csv of run_1 under each stamp and reports the first tick at which each
group of columns differs: the scripted conflict actor's ground truth (world state the
scenario runner drives), the delivery events (publish tick and sequence, generation tick,
age at use), the planner input (observed position and speed of consumed tracks), the ego
trajectory, and the decision. Also the ego position RMS and maximum difference, and the
operational outcome of each run.
"""
import argparse, csv, glob, json, math, os, sys
from collections import defaultdict

GROUPS = {
    "scripted actor ground truth": ["conflict_gt_x", "conflict_gt_y"],
    "delivery events": ["gen_tick", "publish_tick", "publish_seq", "consume_tick", "age_at_use_ms"],
    "planner input": ["track_id", "source", "obs_x", "obs_y", "obs_speed_mps"],
    "ego trajectory": ["ego_x", "ego_y", "ego_speed_mps"],
    "control": ["ctrl_throttle", "ctrl_brake"],
    "decision": ["decision", "brake_event"],
}


def load(stamp, root):
    d = glob.glob(os.path.join(root, stamp, "**", "run_1"), recursive=True)
    if not d:
        sys.exit(f"no run under {root}/{stamp}")
    d = d[0]
    rows = list(csv.DictReader(open(os.path.join(d, "evaluation_output", "per_tick_age.csv"))))
    m = json.load(open(os.path.join(d, "simulation_metrics.json")))
    t = defaultdict(list)
    for r in rows:
        t[int(r["tick"])].append(r)
    return t, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--root", default="experiment_results/highway_merge_edge_late_fusion")
    a = ap.parse_args()
    ta, ma = load(a.a, a.root); tb, mb = load(a.b, a.root)
    common = sorted(set(ta) & set(tb))
    first = {}
    for g, cols in GROUPS.items():
        for t in common:
            ka = sorted(tuple(r.get(c, "") for c in cols) for r in ta[t])
            kb = sorted(tuple(r.get(c, "") for c in cols) for r in tb[t])
            if ka != kb:
                first[g] = t
                break
    d = []
    for t in common:
        ra, rb = ta[t][0], tb[t][0]
        if ra["ego_x"] and rb["ego_x"]:
            d.append(math.hypot(float(ra["ego_x"]) - float(rb["ego_x"]), float(ra["ego_y"]) - float(rb["ego_y"])))
    rms = math.sqrt(sum(x * x for x in d) / len(d)) if d else float("nan")
    print(f"{a.a} vs {a.b}: ticks {len(ta)} / {len(tb)}, common {len(common)}")
    for g in GROUPS:
        print(f"  first difference, {g:<28}: {'none' if g not in first else 'tick ' + str(first[g])}")
    print(f"  ego position RMS {rms:.3f} m, max {max(d) if d else float('nan'):.3f} m; "
          f"s_op {ma.get('s_op')} / {mb.get('s_op')}, collisions {ma.get('focal_collisions')} / {mb.get('focal_collisions')}")


if __name__ == "__main__":
    main()
