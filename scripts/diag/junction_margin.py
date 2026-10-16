#!/usr/bin/env python3
"""Conflict margin at the scenario_3 junction, per run, from per_tick_age.csv.

    python scripts/diag/junction_margin.py <stamp> [--geom openscenario_3_edge_late_fusion] [--root experiment_results]

The crossing car's ground-truth x at each observation tick gives the ticks at
which it occupies the hero's lane (|x + 84.8| < 3.4 m, half lengths summed).
The hero's distance along its approach is the integral of its logged speed
(the approach from the spawn to the crossing car's near edge at y=126.9 is
straight; --approach is that length to the hero's front bumper: 94.5 m from
the y=30 spawn in use since e97fc929, 44.5 m from the April y=80 spawn). The margin is the
tick at which the hero's front reaches the crossing car's lane minus the tick
at which the crossing car leaves it: positive means the hero arrived after the
car had cleared, negative means the hero entered the lane while the car was in
it (a collision unless the hero stopped short). Also printed: the hero's
minimum speed over the window and the run's collision count (success is
read from campaign_check, which recomputes the metrics).
"""
import argparse, csv, glob, os, re, sys

HERO_X = -84.8
HALF_SUM = 3.4
APPROACH_M = 94.5


def analyse(path, approach=APPROACH_M):
    rows = list(csv.DictReader(open(path)))
    gt = {}
    speed = {}
    for r in rows:
        t = int(r["tick"])
        if r.get("ego_speed_mps") not in ("", None, "nan"):
            speed[t] = float(r["ego_speed_mps"])
        if r.get("is_conflict_actor") in ("1", "True", "true") and r.get("conflict_gt_x") not in ("", "nan", None):
            ot = int(float(r["obs_tick"])) if r.get("obs_tick") not in ("", "nan", None) else int(float(r["gen_tick"]))
            gt[ot] = (float(r["conflict_gt_x"]), float(r["conflict_gt_y"]))
    if not speed:
        return None
    in_lane = sorted(t for t, (x, y) in gt.items() if abs(x - HERO_X) < HALF_SUM)
    # integrate speed, linear between logged ticks, v=0 at tick 0
    ticks = sorted(speed)
    s = 0.0
    s_at = {}
    prev_t, prev_v = 0, 0.0
    reach = None
    for t in ticks:
        v = speed[t]
        for k in range(prev_t + 1, t + 1):
            vk = prev_v + (v - prev_v) * (k - prev_t) / (t - prev_t)
            s += vk * 0.05
            s_at[k] = s
            if reach is None and s >= approach:
                reach = k
        prev_t, prev_v = t, v
    vmin = min(speed[t] for t in ticks if (in_lane and in_lane[0] - 40 <= t <= (in_lane[-1] + 20))) if in_lane else float("nan")
    # Unbraked arrival: extrapolate from the last tick before the first
    # sustained brake (ctrl_brake > 0.5 for 3 ticks after tick 60) at the
    # speed held then. delta = car's lane entry minus that arrival: negative
    # means the car enters before the hero would have arrived unbraked.
    brk = {}
    for r in rows:
        if r.get("ctrl_brake") not in ("", None, "nan"):
            brk[int(r["tick"])] = float(r["ctrl_brake"])
    tb = None
    for t in sorted(brk):
        if t > 60 and all(brk.get(t + k, 0) > 0.5 for k in range(3)):
            tb = t
            break
    unbraked = None
    if tb is not None and tb in s_at and speed.get(tb, 0) > 1.0:
        unbraked = tb + (approach - s_at[tb]) / speed[tb] / 0.05
    elif reach is not None:
        unbraked = reach
    delta = (in_lane[0] - unbraked) if (in_lane and unbraked is not None) else None
    return dict(in_lane=(in_lane[0], in_lane[-1]) if in_lane else None, reach=reach, vmin=vmin, n_gt=len(gt),
                s_end=s, brake_tick=tb, unbraked=round(unbraked) if unbraked is not None else None,
                delta=round(delta) if delta is not None else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamp")
    ap.add_argument("--geom", default="openscenario_3_edge_late_fusion")
    ap.add_argument("--root", default="experiment_results")
    ap.add_argument("--mgr", default=None)
    ap.add_argument("--approach", type=float, default=APPROACH_M, help="metres from the spawn to the crossing car's near edge (front bumper)")
    a = ap.parse_args()
    base = os.path.join(a.root, a.geom, a.stamp)
    paths = sorted(glob.glob(os.path.join(base, "**", "per_tick_age.csv"), recursive=True))
    print(f"{len(paths)} runs under {base}")
    print(f"{'mgr':>12} {'spd':>4} {'lat':>5} {'run':>4} {'car in lane':>13} {'hero reach':>10} {'margin':>7} {'vmin':>5} {'coll':>4} {'brake':>6} {'unbraked':>8} {'delta':>6}")
    import json
    out = []
    for p in paths:
        rd = os.path.dirname(p)
        m = re.search(r"speed_spd_(\d+)/.*?mgr_([a-z_]+)/anchoring_(\w+)/lat_(\d+)_.*?/run_(\d+)", p)
        if not m:
            continue
        spd, mgr, anch, lat, run = int(m.group(1)), m.group(2), m.group(3), int(m.group(4)), int(m.group(5))
        if a.mgr and mgr != a.mgr:
            continue
        r = analyse(p, a.approach)
        if r is None:
            continue
        mj = os.path.join(rd, "simulation_metrics.json")
        coll = "?"
        if os.path.exists(mj):
            try:
                coll = json.load(open(mj)).get("focal_collisions", "?")
            except Exception:
                pass
        il = r["in_lane"]
        margin = (r["reach"] - il[1]) if (il and r["reach"] is not None) else None
        print(f"{mgr:>12} {spd:>4} {lat:>5} {run:>4} {str(il):>13} {str(r['reach']):>10} {str(margin):>7} {r['vmin']:5.1f} {str(coll):>4} {str(r['brake_tick']):>6} {str(r['unbraked']):>8} {str(r['delta']):>6}")
        out.append((mgr, spd, lat, run, il, r["reach"], margin, r["vmin"], coll, r["delta"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
