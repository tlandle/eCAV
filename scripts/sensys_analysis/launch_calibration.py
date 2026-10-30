#!/usr/bin/env python3
"""Launch calibration from the free-flow runs (encounter check, Oct 29).

    python scripts/sensys_analysis/launch_calibration.py [--stamp sensys_nov27_j5v_ff] [--ego-entry 4.30]

For every scenario and target speed, the free-flow runs (planner never
responds to a hazard, ego_x/ego_y in the per-tick log) give the ego's
position against time on its approach. The crossing car's footprint is
inside the conflict box from launch + 4.15 s to + 4.70 s in every cell, so a
genuine conflict needs the ego's footprint to enter the box about 4.3 s
after the launch. The script reports, per cell, the ego's distance to the
box entry at that moment (the distance at which the car must launch), the
ego's speed there against its target, and the straight-line distance to the
conflict point the trigger would use.
"""
import argparse
import csv
import glob
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import GEOM_RE, ROOT  # noqa: E402
from encounter_check import load_log, launch_tick, footprint_metrics, CONFLICT  # noqa: E402


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stamp", default="sensys_nov27_j5v_ff")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--ego-entry", type=float, default=4.30, help="wanted ego box entry after launch, s")
    a = ap.parse_args(argv)
    cells = {}
    for f in glob.glob(os.path.join(a.root, "*", a.stamp, "**", "per_tick_age.csv"), recursive=True):
        m = GEOM_RE.search(f)
        if not m:
            continue
        geom, spd, mgr, anch, cfg, rep = m.groups()
        rows, gt, ego_v, ego_xy = load_log(f)
        launch = launch_tick(gt)
        if launch is None or not ego_xy:
            continue
        fp = footprint_metrics(ego_xy, gt, launch)
        if fp["ego_in"] != fp["ego_in"]:
            continue
        entry_tick = launch + int(round(fp["ego_in"] / 0.05))
        want_tick = entry_tick - int(round(a.ego_entry / 0.05))   # the launch tick that gives the wanted entry
        near = lambda t: ego_xy[min(ego_xy, key=lambda k: abs(k - t))]
        x, y = near(want_tick)
        d_conflict = math.hypot(x - CONFLICT[0], y - CONFLICT[1])
        v = ego_v.get(want_tick, float("nan"))
        vmax = max(ego_v.values())
        v_launch_now = ego_v.get(launch, float("nan"))
        cells.setdefault((geom, int(spd)), []).append(dict(ego_in=fp["ego_in"], d=d_conflict, v=v, vmax=vmax,
                                                          v_launch_now=v_launch_now, shift=(want_tick - launch) * 0.05,
                                                          hit=fp["fp_hit"], fpmin=fp["fp_min_m"]))
    print("| scenario | target km/h | seeds | free-flow ego box entry (+s) | footprints hit | min footprint gap | launch shift needed (s) | trigger distance to conflict (m) | ego speed there (m/s) | ego peak speed | speed at today's launch |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    med = lambda L: statistics.median(L)
    for k in sorted(cells):
        rs = cells[k]
        print("| %s | %d | %d | %.2f | %d/%d | %.1f | %+.2f | %.1f | %.1f (%d km/h) | %.1f (%d km/h) | %.1f |" % (
            "left turn" if k[0].startswith("openscenario_3") else "straight crossing", k[1], len(rs),
            med([r["ego_in"] for r in rs]), sum(r["hit"] for r in rs), len(rs), med([r["fpmin"] for r in rs]),
            med([r["shift"] for r in rs]), med([r["d"] for r in rs]), med([r["v"] for r in rs]), med([r["v"] for r in rs]) * 3.6,
            med([r["vmax"] for r in rs]), med([r["vmax"] for r in rs]) * 3.6, med([r["v_launch_now"] for r in rs])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
