#!/usr/bin/env python
"""Reduce the column study (scripts/ispass/column_sweep.sh).

    python scripts/ispass/column_reduce.py <stamp> [--ns 1 2 4 8 16] [--csv cells.csv] [--runs-csv runs.csv]

Every vehicle of the column is a vehicle of the run's simulation_metrics.json. A run is valid when its log has no
exception, it has N vehicles, the vehicle state it recorded is the arm's, and the crossing car launched (not
required with --no-crossing, the queue-tail variant). Per cell:
  runs with a collision          at least one vehicle of the column collided (Wilson 95 % interval)
  collided vehicles per vehicle  vehicles that collided, over N times the valid runs: the exposure-normalized rate
  leader / followers             the same count for vehicle 1 (meets the crossing car) and for vehicles 2..N
  partner                        collided vehicles whose first collision is with a scripted vehicle (the crossing car or the stopped
                                 vehicle), with another vehicle of the column, or with anything else
  brake events per vehicle, mean speed, wall time per tick
"""
import argparse, ast, csv, glob, json, math, os, sys
import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
ARMS = ["gt_vis", "onboard"]; NAMES = {"gt_vis": "ground truth in view", "onboard": "camera and LiDAR"}
STATE = {"gt_vis": "gt_visible", "onboard": None}


def wilson(k, n, z=1.96):
    if n == 0: return float("nan"), float("nan")
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def as_obj(x):
    if isinstance(x, str):
        try: return ast.literal_eval(x)
        except Exception: return {}
    return x or {}


def launched(pt):
    """The crossing car moved, and the mean wall time per tick of the run."""
    if not os.path.exists(pt): return False, float("nan")
    gt = {}; wall = {}
    for r in csv.DictReader(open(pt)):
        t = int(r["tick"])
        if r.get("conflict_gt_x"): gt[t] = (float(r["conflict_gt_x"]), float(r["conflict_gt_y"]))
        if r.get("wall_time_s"): wall.setdefault(t, float(r["wall_time_s"]))
    ts = sorted(gt); moved = any(t - 1 in gt and math.hypot(gt[t][0] - gt[t - 1][0], gt[t][1] - gt[t - 1][1]) > 0.02 for t in ts)
    w = sorted(wall.items()); dt = [(b[1] - a[1]) / (b[0] - a[0]) * 1000 for a, b in zip(w, w[1:]) if b[0] > a[0]]
    return moved, float(np.mean(dt)) if dt else float("nan")


def load(stamp, ns, no_crossing=False):
    rows = []
    for arm in ARMS:
        for n in ns:
            for f in sorted(glob.glob(os.path.join(ROOT, "experiment_results", "*", f"{stamp}_{arm}_n{n}", "**", "simulation_metrics.json"), recursive=True)):
                if "/evaluation_output/" in f: continue
                rd = os.path.dirname(f); rep = int(os.path.basename(rd).split("_")[-1])
                try: d = json.load(open(f))
                except (OSError, ValueError): rows.append(dict(arm=arm, n=n, rep=rep, valid=0)); continue
                lp = os.path.join(rd, "runner_stdout.txt"); log = open(lp, errors="replace").read() if os.path.exists(lp) else ""
                veh = d.get("vehicles") or {}; moved, tick_ms = launched(os.path.join(rd, "evaluation_output", "per_tick_age.csv"))
                valid = ("error" not in d and "Traceback" not in log and len(veh) == n and d.get("config_vehicle_state") == STATE[arm] and (moved or no_crossing))
                focal = str(d.get("focal_vehicle_id")); ids = set(veh)
                hit = {}; brakes = []; speeds = []
                for vid, v in veh.items():
                    s = as_obj(v.get("safety")); hist = as_obj(s.get("collision_history")) if isinstance(s, dict) else []
                    brakes.append(float(v.get("total_brake_events", 0) or 0)); speeds.append(float(v.get("avg_speed_mps", 0) or 0))
                    if isinstance(s, dict) and int(s.get("collision_count", 0) or 0) > 0:
                        first = hist[0] if hist else {}
                        other = str(first.get("other_actor_id")); typ = str(first.get("other_type_id", ""))
                        hit[vid] = "column" if other in ids else ("crossing" if typ.startswith("vehicle.") else "other")
                rows.append(dict(arm=arm, n=n, rep=rep, valid=int(valid), vehicles=len(veh), collided=len(hit), any=int(bool(hit)),
                                 leader=int(focal in hit), followers=len([v for v in hit if v != focal]),
                                 with_crossing=sum(1 for p in hit.values() if p == "crossing"), with_column=sum(1 for p in hit.values() if p == "column"),
                                 with_other=sum(1 for p in hit.values() if p == "other"), brakes=float(np.mean(brakes)) if brakes else float("nan"),
                                 speed=float(np.mean(speeds)) if speeds else float("nan"), tick_ms=tick_ms))
    return rows


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stamp"); ap.add_argument("--ns", type=int, nargs="*", default=[1, 2, 4, 8, 16]); ap.add_argument("--csv"); ap.add_argument("--runs-csv")
    ap.add_argument("--no-crossing", action="store_true", help="queue-tail variant: the crossing car does not launch")
    a = ap.parse_args(); rows = load(a.stamp, a.ns, a.no_crossing); summ = []
    print(f"{'N':>3} {'arm':22s} runs valid  runs with a collision [95% CI]   collided/vehicle  leader  followers  crossing  column  other  brakes/veh  speed  tick ms")
    for n in a.ns:
        for arm in ARMS:
            cell = [r for r in rows if (r["arm"], r["n"]) == (arm, n)]; ok = [r for r in cell if r["valid"]]
            if not cell: continue
            m = len(ok); k = sum(r["any"] for r in ok); lo, hi = wilson(k, m); tot = sum(r["collided"] for r in ok)
            mean = lambda key: float(np.nanmean([r[key] for r in ok])) if ok else float("nan")
            s = dict(n=n, arm=arm, runs=len(cell), valid=m, runs_with_collision=k, lo=lo, hi=hi, collided_vehicles=tot,
                     collided_per_vehicle=tot / (n * m) if m else float("nan"), leader=sum(r["leader"] for r in ok), followers=sum(r["followers"] for r in ok),
                     with_crossing=sum(r["with_crossing"] for r in ok), with_column=sum(r["with_column"] for r in ok), with_other=sum(r["with_other"] for r in ok),
                     brakes=mean("brakes"), speed=mean("speed"), tick_ms=mean("tick_ms"))
            summ.append(s)
            print(f"{n:3d} {NAMES[arm]:22s} {len(cell):4d} {m:5d}  {k:3d} of {m:<3d} [{lo:.2f},{hi:.2f}]            {s['collided_per_vehicle']:8.3f}       {s['leader']:4d}  {s['followers']:9d}  "
                  f"{s['with_crossing']:8d}  {s['with_column']:6d}  {s['with_other']:5d}  {s['brakes']:10.1f}  {s['speed']:5.1f}  {s['tick_ms']:7.0f}")
    for path, data in ((a.csv, summ), (a.runs_csv, [r for r in rows if "any" in r])):
        if path and data:
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(data[0].keys())); w.writeheader(); w.writerows(data)
            print("wrote", path)


if __name__ == "__main__":
    main()
