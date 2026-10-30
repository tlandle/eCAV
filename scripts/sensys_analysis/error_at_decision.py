"""Forecast error at the conflict at the brake decision tick (first brake_event on the conflict actor after the
launch), per run; p50 / p99 over runs per cell, 70 km/h, with the cell's closed-loop outcome. Also the time the
decision came after the window opened. Usage: error_at_decision.py <roots...>"""
import csv, glob, math, os, sys, collections
import numpy as np
CONFLICT_X = -84.7; HORIZON_S = 3.0
ARMS = {("oracle","on"):"reference", ("late_fusion","on"):"contract", ("late_fusion","off"):"naive"}
cells = collections.defaultdict(list)
for root in sys.argv[1:]:
    for mf in glob.glob(f"{root}/*/sensys_nov27_j6s/speed_spd_70/baseline_default/mgr_*/anchoring_*/lat_*/run_*/simulation_metrics.json"):
        if "/evaluation_output/" in mf: continue
        d = os.path.dirname(mf); p = d.split("/"); geom = p[-8][:3]; mgr = p[-4][4:]; anch = p[-3].split("_")[-1]; lat = int(p[-2].split("_")[1])
        arm = ARMS.get((mgr, anch));
        if arm is None: continue
        f = d + "/per_tick_age.csv"
        if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
        if not os.path.exists(f): continue
        rows = list(csv.DictReader(open(f))); per = {}
        for r in rows: per.setdefault(int(r["tick"]), r)
        ticks = sorted(per); g0 = None; launch = None; gt = {}
        for t in ticks:
            r = per[t]
            try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
            except Exception: continue
            gt[t] = (gx, gy)
            if g0 is None: g0 = (gx, gy)
            if launch is None and math.hypot(gx-g0[0], gy-g0[1]) > 0.3: launch = t
        if launch is None: continue
        arr_true = next((t for t in ticks if t in gt and gt[t][0] <= CONFLICT_X), None)
        if arr_true is None: continue
        m = {}
        try:
            import json; mm = json.load(open(mf)); 
        except Exception: mm = {}
        opened = None; dec = None
        for r in rows:
            t = int(r["tick"])
            if t < launch or r.get("is_conflict_actor") != "1" or t not in gt: continue
            if float(r["ego_y"] or 0) > 126: break
            try: ox, ov, ot = float(r["obs_x"]), float(r["obs_speed_mps"]), int(float(r["obs_tick"]))
            except Exception: continue
            fx = ox - ov * (t - ot) * 0.05
            if opened is None and ov > 0.5 and (fx - CONFLICT_X) / ov <= HORIZON_S: opened = t
            if r.get("brake_event") == "1":
                h = (arr_true - t) * 0.05
                err = abs((fx - ov * max(h, 0)) - CONFLICT_X)
                arr_err = (fx - CONFLICT_X) / ov - h if ov > 0.5 else float("nan")
                dec = (t, err, arr_err); break
        cells[(geom, arm, lat)].append((dec, opened, launch))
runs = {}
for pth in ("safety_envelope_paper/analysis/envelope_j6s/runs.csv","safety_envelope_paper/analysis/envelope_j6s_cetus/runs.csv","safety_envelope_paper/analysis/envelope_j6s_azure/runs.csv"):
    for r in csv.DictReader(open(os.path.join(os.path.dirname(sys.argv[1].rstrip("/")) if False else "/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox", pth))):
        if int(float(r["speed"])) != 70: continue
        arm = ARMS.get((r["mgr"], r["anch"]))
        if arm: runs.setdefault((r["geom"][:3], arm, int(float(r["lat"]))), []).append(r)
print("geom arm        age   n  decided | error at the conflict at the decision (m) p50 / p99 | arrival error (s) p50 / p99 | decision after window opened (s) p50 | closed loop ok/n coll")
for k in sorted(cells):
    rs = cells[k]; dec = [x for x in rs if x[0] is not None]
    e = [x[0][1] for x in dec]; a = [x[0][2] for x in dec if not math.isnan(x[0][2])]; dt = [(x[0][0]-x[1])*0.05 for x in dec if x[1] is not None]
    o = runs.get(k, []); ok = sum(int(float(r["s_op"])) for r in o); co = sum(int(float(r["collided"])) for r in o)
    q = lambda v, p: (np.percentile(v, p) if v else float("nan"))
    print(f"{k[0]:4s} {k[1]:10s} {k[2]:5d} {len(rs):3d} {len(dec):3d}     | {q(e,50):5.2f} / {q(e,99):5.2f}                                 | {q(a,50):5.2f} / {q(a,99):5.2f}            | {q(dt,50):5.2f}                            | {ok}/{len(o)} {co}")
