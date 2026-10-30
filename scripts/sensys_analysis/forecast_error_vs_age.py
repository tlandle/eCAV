"""Conductor-style deadline from the sweep's own logs: the error of the planner's re-indexed forecast of the conflict
actor at the moment of use, per configured age. For each tick after the launch and before the ego reaches the box,
the planner holds an observation (obs_x, obs_y, obs_speed, obs_tick); the re-indexed estimate is the observation
advanced at its speed for the age along the car's heading (-x); the error is its distance to the ground truth at that
tick. Reported per age: p50/p99 over all ticks of all runs, split into the launch ramp (first 1.5 s) and the steady
phase, plus the arrival error: the forecast's time to reach the conflict x versus the true time. Reference arm only
(exact state at the source tick, so the error is the age's alone)."""
import csv, glob, math, os, sys, collections
import numpy as np
roots = sys.argv[1:]
CONFLICT_X = -84.7
errs = collections.defaultdict(lambda: {"ramp": [], "steady": [], "arr": []})
nruns = collections.Counter()
for root in roots:
    for mf in glob.glob(f"{root}/*/sensys_nov27_j6s/speed_spd_70/baseline_default/mgr_" + os.environ.get("MGR","oracle") + "/anchoring_" + os.environ.get("ANCH","on") + "/lat_*/run_*/simulation_metrics.json"):
        if "/evaluation_output/" in mf: continue
        d = os.path.dirname(mf); lat = int(d.split("/")[-2].split("_")[1]); geom = d.split("/")[-8][:3]
        f = d + "/per_tick_age.csv"
        if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
        if not os.path.exists(f): continue
        rows = list(csv.DictReader(open(f)))
        per = {}
        for r in rows: per.setdefault(int(r["tick"]), r)
        ticks = sorted(per); g0 = None; launch = None; gt = {}
        for t in ticks:
            r = per[t]
            try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
            except: continue
            gt[t] = (gx, gy)
            if g0 is None: g0 = (gx, gy)
            if launch is None and math.hypot(gx-g0[0], gy-g0[1]) > 0.3: launch = t
        if launch is None: continue
        nruns[(geom, lat)] += 1
        # true arrival tick at the conflict x
        arr_true = next((t for t in ticks if t in gt and gt[t][0] <= CONFLICT_X), None)
        for r in rows:
            t = int(r["tick"])
            if t < launch or r.get("is_conflict_actor") != "1": continue
            if float(r["ego_y"] or 0) > 126: break
            try:
                ox, oy, ov, ot = float(r["obs_x"]), float(r["obs_y"]), float(r["obs_speed_mps"]), int(float(r["obs_tick"]))
            except: continue
            if t not in gt: continue
            age_s = (t - ot) * 0.05
            fx = ox - ov * age_s  # re-indexed estimate (car heads -x)
            e = math.hypot(fx - gt[t][0], oy - gt[t][1])
            phase = "ramp" if (t - launch) * 0.05 <= 1.5 else "steady"
            errs[(geom, lat)][phase].append(e)
            if arr_true is not None and ov > 0.5 and fx > CONFLICT_X:
                t_fore = t + (fx - CONFLICT_X) / ov / 0.05   # tick at which the forecast reaches the conflict
                errs[(geom, lat)]["arr"].append((t_fore - arr_true) * 0.05)
print(os.environ.get("MGR","oracle")+"/"+os.environ.get("ANCH","on")+", 70 km/h: re-indexed position error at use (m) and forecast arrival error (s, + = forecast late)")
print(f"{'geom':4s} {'age':>5s} {'runs':>4s} | ramp p50 / p99 | steady p50 / p99 | arrival p50 / p99")
for k in sorted(errs):
    e = errs[k]
    def q(v, p): return np.percentile(v, p) if v else float('nan')
    print(f"{k[0]:4s} {k[1]:5d} {nruns[k]:4d} | {q(e['ramp'],50):5.2f} / {q(e['ramp'],99):5.2f} | {q(e['steady'],50):5.2f} / {q(e['steady'],99):5.2f} | {q(e['arr'],50):5.2f} / {q(e['arr'],99):5.2f}")
