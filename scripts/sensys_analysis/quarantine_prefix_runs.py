#!/usr/bin/env python3
"""Move the exact-state reference (oracle) runs that landed before the exact-velocity fix out of the
campaign tree so a resume reruns them. A run is moved when its simulation_metrics.json is older than
--before (local time). Runs go to <root>/_prefix_oracle/<same relative path>; the cell directory
stays so the rerun lands in place. Prints one line per cell.
    python scripts/sensys_analysis/quarantine_prefix_runs.py --root experiment_results --before "2026-10-29 03:52:15" [--apply]
"""
import argparse, collections, glob, os, re, shutil, time

ap = argparse.ArgumentParser()
ap.add_argument("--root", default="experiment_results")
ap.add_argument("--before", required=True, help="local time 'YYYY-MM-DD HH:MM:SS'")
ap.add_argument("--stamps", default="sensys_nov27_j5*")
ap.add_argument("--apply", action="store_true")
a = ap.parse_args()
cut = time.mktime(time.strptime(a.before, "%Y-%m-%d %H:%M:%S"))
cells = collections.defaultdict(lambda: [0, 0])
moved = 0
for f in sorted(glob.glob(os.path.join(a.root, "*", a.stamps, "speed_spd_*", "baseline_default", "mgr_oracle", "anchoring_*", "*", "run_*", "simulation_metrics.json"))):
    rel = os.path.relpath(f, a.root)
    if rel.startswith("_"): continue
    run_dir = os.path.dirname(f); cell = os.path.dirname(run_dir)
    k = os.path.relpath(cell, a.root)
    cells[k][1] += 1
    if os.path.getmtime(f) >= cut: continue
    cells[k][0] += 1; moved += 1
    if a.apply:
        dst = os.path.join(a.root, "_prefix_oracle", os.path.relpath(run_dir, a.root))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.move(run_dir, dst)
for k in sorted(cells):
    m, n = cells[k]
    p = k.split("/"); lat = re.sub(r"_loss.*", "", p[6]); ego = re.search(r"_ego_(\d+)_", p[6]).group(1)
    short = f"{p[0][:14]} {p[1]} {p[2]} {lat} {ego}cav"
    print(f"{'moved' if a.apply else 'would move'} {m:3d}/{n:3d}  {short}")
print(f"{'moved' if a.apply else 'would move'} {moved} runs older than {a.before}")
