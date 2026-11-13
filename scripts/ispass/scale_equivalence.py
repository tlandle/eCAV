#!/usr/bin/env python3
"""Compare the vehicle trajectories of two launch_scale.sh runs of the same scenario.

    python scripts/ispass/scale_equivalence.py experiment_results/ispass_scale/<tagA> <tagB dir> [--json out.json] [--lp]

Every vehicle client prints its ground-truth position once per tick ([LOC] ... gt=(x,y), 0.1 m) and closes
each tick with [CLIENT VEHICLE TICK n]. The runs are aligned per vehicle index on the tick number n. The planner's
own ego position ([LP] ego=, 0.01 m) carries the localization noise, so it is used only with --lp, for runs generated
with --exact-localization, where it is the simulator's transform and the [LOC] line is not printed.
Reports, over all vehicles and
common ticks, the largest and the root-mean-square position difference and the first tick at which any
vehicle differs by more than the 0.1 m print resolution.
"""
import glob
import json
import math
import os
import re
import sys

LOC = re.compile(r"\[LOC\].*?gt=\(([-\d.]+),([-\d.]+)\)")
LP = re.compile(r"\[LP\] ego=\(([-\d.]+),([-\d.]+)\)")
TICK = re.compile(r"\[CLIENT VEHICLE TICK (\d+)\]")


def load(d, pat=LOC):
    out = {}
    for f in glob.glob(os.path.join(d, "*", "veh_*.log")):
        k = int(re.search(r"veh_(\d+)\.log$", f).group(1))
        tr, loc = {}, None
        with open(f, errors="replace") as fh:
            for line in fh:
                m = pat.search(line)
                if m:
                    loc = (float(m.group(1)), float(m.group(2))); continue
                m = TICK.search(line)
                if m and loc:
                    tr[int(m.group(1))] = loc
                    loc = None
        out[k] = tr
    return out


def main():
    lp = "--lp" in sys.argv
    res_m = 0.01 if lp else 0.1
    a, b = load(sys.argv[1], LP if lp else LOC), load(sys.argv[2], LP if lp else LOC)
    veh = sorted(set(a) & set(b))
    worst, sq, n, first = 0.0, 0.0, 0, None
    per_tick = {}
    for k in veh:
        for ts in sorted(set(a[k]) & set(b[k])):
            d = math.hypot(a[k][ts][0] - b[k][ts][0], a[k][ts][1] - b[k][ts][1])
            worst = max(worst, d); sq += d * d; n += 1
            per_tick[ts] = max(per_tick.get(ts, 0.0), d)
    for ts in sorted(per_tick):
        if per_tick[ts] > res_m + 1e-4:
            first = ts; break
    ticks = sorted(per_tick)
    res = {"a": sys.argv[1], "b": sys.argv[2], "vehicles": len(veh), "common_ticks": len(ticks),
           "max_diff_m": round(worst, 4), "max_diff_by_tick": {str(t): round(per_tick[t], 2) for t in ticks if t in (1, 50, 100, 150, 200, 250, 300)}, "rms_diff_m": round(math.sqrt(sq / n), 4) if n else None,
           "first_tick_over_resolution": first, "tick_span": [ticks[0], ticks[-1]] if ticks else None,
           "position_resolution_m": res_m, "position_source": "planner ego position" if lp else "ground truth"}
    print(json.dumps(res))
    if "--json" in sys.argv:
        with open(sys.argv[sys.argv.index("--json") + 1], "w") as fh:
            json.dump(res, fh, indent=1)


if __name__ == "__main__":
    main()
