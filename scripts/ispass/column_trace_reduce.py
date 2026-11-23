#!/usr/bin/env python
"""Reduce the column study with the crossing car by position in the column (column_sweep.sh with COLUMN_TRACE=1).

    python scripts/ispass/column_trace_reduce.py <stem> --speeds 60 70 --ns 2 4 8 16 [--csv cells.csv] [--runs-csv runs.csv] [--pos-csv positions.csv]

A cell is <stem><speed>_<arm>_n<N>. Position 1 is the leading vehicle, which meets the crossing car. Per run, from
column_trace.csv (one row per vehicle and tick), the [SYNC LAUNCH] line and simulation_metrics.json:
  gap at launch      centre distance to the vehicle ahead when the crossing car launches, and the time gap
                     (centre distance minus the vehicle length of 4.6 m, over the follower's speed)
  brake start        seconds from the launch to the first of three consecutive ticks with brake >= 0.5
  peak deceleration  largest speed loss over 0.5 s after the launch, as m/s^2
  collided           the vehicle has a collision in the run (binary), and whether its first contact is the crossing
                     car, another vehicle of the column, or something else
A run is valid when its log has no exception, it has N vehicles, the recorded vehicle state is the arm's and the
crossing car launched.
"""
import argparse, ast, csv, glob, json, math, os, re
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
ARMS = ["gt_all", "gt_vis", "onboard"]; STATE = {"gt_all": "gt_all", "gt_vis": "gt_visible", "onboard": None}; LENGTH = 4.6; DT = 0.05


def as_obj(x):
    if isinstance(x, str):
        try: return ast.literal_eval(x)
        except Exception: return {}
    return x or {}


def load_run(rd, arm, n, per_lane=10 ** 6):
    out = dict(valid=0)
    try: d = json.load(open(os.path.join(rd, "simulation_metrics.json")))
    except (OSError, ValueError): return out, []
    log = open(os.path.join(rd, "runner_stdout.txt"), errors="replace").read()
    m = re.search(r"\[SYNC LAUNCH\] step=(\d+)", log); tr = os.path.join(rd, "evaluation_output", "column_trace.csv")
    if not os.path.exists(tr): tr = os.path.join(rd, "column_trace.csv")
    veh = d.get("vehicles") or {}
    if "error" in d or "Traceback" in log or len(veh) != n or d.get("config_vehicle_state") != STATE[arm] or not m or not os.path.exists(tr):
        return out, []
    launch = int(m.group(1)); T = {}
    for r in csv.DictReader(open(tr)):
        T.setdefault(int(r["index"]), {})[int(r["step"])] = (float(r["x"]), float(r["y"]), float(r["speed_mps"]), float(r["brake"]), int(r["actor_id"]))
    if len(T) != n or any(launch not in T[k] for k in T): return out, []
    ids = {str(v.get("actor_id")): vid for vid, v in veh.items()}; col_ids = set(ids) | set(veh)
    pos = []
    for k in range(n):
        tk = T[k]; steps = sorted(s for s in tk if s >= launch); x, y, v, _, aid = tk[launch]
        gap = tgap = float("nan")
        if k % per_lane > 0:
            px, py = T[k - 1][launch][0], T[k - 1][launch][1]; gap = math.hypot(px - x, py - y); tgap = (gap - LENGTH) / v if v > 0.5 else float("nan")
        # brake start: the first of three consecutive ticks with brake >= 0.5 (speed control uses lighter braking)
        b = next((s for s in steps if all(s + i in tk and tk[s + i][3] >= 0.5 for i in range(3))), None)
        # within 10 s of the launch and before the last second of the run (the scenario's end stops every vehicle)
        last = max(tk) - 20
        dec = max([(tk[s][2] - tk[s + 10][2]) / (10 * DT) for s in steps if s + 10 in tk and s <= launch + 200 and s + 10 <= last] or [float("nan")])
        vv = veh.get(ids.get(str(aid), ""), {}); s_ = as_obj(vv.get("safety")); hit = isinstance(s_, dict) and int(s_.get("collision_count", 0) or 0) > 0
        partner = ""
        if hit:
            hist = as_obj(s_.get("collision_history")); first = hist[0] if hist else {}
            other = str(first.get("other_actor_id")); typ = str(first.get("other_type_id", ""))
            partner = "column" if other in col_ids else ("crossing" if typ.startswith("vehicle.") else "other")
        pos.append(dict(position=k % per_lane + 1, lane=k // per_lane + 1, gap_m=gap, time_gap_s=tgap, speed_at_launch=v, brake_start_s=(b - launch) * DT if b is not None else float("nan"),
                        peak_decel=dec, collided=int(hit), partner=partner))
    out = dict(valid=1, launch=launch, collided=sum(p["collided"] for p in pos), any=int(any(p["collided"] for p in pos)),
               lead=sum(p["collided"] for p in pos if p["position"] == 1), followers=sum(p["collided"] for p in pos if p["position"] > 1), with_column=sum(1 for p in pos if p["partner"] == "column"),
               with_crossing=sum(1 for p in pos if p["partner"] == "crossing"), first_follower=min([p["position"] for p in pos if p["position"] > 1 and p["collided"]] or [0]),
               min_time_gap=float(np.nanmin([p["time_gap_s"] for p in pos if p["position"] > 1])) if any(p["position"] > 1 for p in pos) else float("nan"),
               med_gap=float(np.nanmedian([p["gap_m"] for p in pos if p["position"] > 1])) if any(p["position"] > 1 for p in pos) else float("nan"),
               med_time_gap=float(np.nanmedian([p["time_gap_s"] for p in pos if p["position"] > 1])) if any(p["position"] > 1 for p in pos) else float("nan"), lead_speed=pos[0]["speed_at_launch"])
    return out, pos


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stem"); ap.add_argument("--speeds", type=int, nargs="+", default=[60, 70])
    ap.add_argument("--ns", type=int, nargs="+", default=[2, 4, 8, 16]); ap.add_argument("--csv"); ap.add_argument("--runs-csv"); ap.add_argument("--pos-csv")
    ap.add_argument("--root", default=ROOT, help="checkout that holds experiment_results (default: the one of the script)")
    ap.add_argument("--per-lane", type=int, default=10 ** 6, help="vehicles per lane: larger N fill a second lane, positions count within a lane")
    a = ap.parse_args(); runs = []; poss = []; cells = []
    for spd in a.speeds:
        for n in a.ns:
            for arm in ARMS:
                for f in sorted(glob.glob(os.path.join(a.root, "experiment_results", "*", f"{a.stem}{spd}_{arm}_n{n}", "**", "column_trace.csv"), recursive=True)
                                + [p for p in glob.glob(os.path.join(a.root, "experiment_results", "*", f"{a.stem}{spd}_{arm}_n{n}", "**", "runner_stdout.txt"), recursive=True)
                                   if not os.path.exists(os.path.join(os.path.dirname(p), "evaluation_output", "column_trace.csv"))]):
                    rd = os.path.dirname(f)
                    if os.path.basename(rd) == "evaluation_output": rd = os.path.dirname(rd)      # the trace lies beside the per-tick log
                    rep = int(os.path.basename(rd).split("_")[-1]); r, pos = load_run(rd, arm, n, a.per_lane)
                    runs.append(dict(speed=spd, n=n, arm=arm, rep=rep, **r)); poss += [dict(speed=spd, n=n, arm=arm, rep=rep, **p) for p in pos]
    print(f"{'km/h':>4} {'N':>3} {'arm':8s} runs valid  with collision  collided veh  lead  followers (rear-end)  first follower pos  gap m  time gap s (median, min)  lead speed")
    for spd in a.speeds:
        for n in a.ns:
            for arm in ARMS:
                cell = [r for r in runs if (r["speed"], r["n"], r["arm"]) == (spd, n, arm)]; ok = [r for r in cell if r.get("valid")]
                if not cell: continue
                med = lambda key: float(np.nanmedian([r[key] for r in ok])) if ok else float("nan")
                ff = sorted(r["first_follower"] for r in ok if r["first_follower"])
                c = dict(speed=spd, n=n, arm=arm, runs=len(cell), valid=len(ok), runs_with_collision=sum(r["any"] for r in ok), collided_vehicles=sum(r["collided"] for r in ok),
                         lead=sum(r["lead"] for r in ok), followers=sum(r["followers"] for r in ok), with_column=sum(r["with_column"] for r in ok),
                         med_gap=med("med_gap"), med_time_gap=med("med_time_gap"), min_time_gap=float(np.nanmin([r["min_time_gap"] for r in ok])) if ok and n > 1 else float("nan"), lead_speed=med("lead_speed"))
                cells.append(c)
                print(f"{spd:4d} {n:3d} {arm:8s} {len(cell):4d} {len(ok):5d}  {c['runs_with_collision']:3d} of {len(ok):<3d}     {c['collided_vehicles']:6d}      {c['lead']:4d}  {c['followers']:4d} ({c['with_column']})            {str(ff):18s} "
                      f"{c['med_gap']:5.1f}  {c['med_time_gap']:5.2f} {c['min_time_gap']:5.2f}             {c['lead_speed']:5.1f}")
    print("\nby position (median over valid runs): brake start s after launch | peak deceleration m/s^2 | collided count")
    for spd in a.speeds:
        for n in a.ns:
            for arm in ARMS:
                pp = [p for p in poss if (p["speed"], p["n"], p["arm"]) == (spd, n, arm)]
                if not pp: continue
                line = []
                for k in range(1, min(n, a.per_lane) + 1):
                    q = [p for p in pp if p["position"] == k]
                    line.append("%d: %.2f|%.1f|%d" % (k, np.nanmedian([p["brake_start_s"] for p in q]) if any(p["brake_start_s"] == p["brake_start_s"] for p in q) else float("nan"),
                                                     np.nanmedian([p["peak_decel"] for p in q]), sum(p["collided"] for p in q)))
                print(f"{spd} km/h N={n} {arm}: " + "  ".join(line))
    for path, data in ((a.csv, cells), (a.runs_csv, [r for r in runs if r.get("valid")]), (a.pos_csv, poss)):
        if path and data:
            with open(path, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(data[0].keys())); w.writeheader(); w.writerows(data)
            print("wrote", path)


if __name__ == "__main__":
    main()
