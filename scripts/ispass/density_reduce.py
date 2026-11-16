#!/usr/bin/env python
"""Reduce the traffic density study (density_sweep.sh): ground truth in sensor view against camera and LiDAR,
no edge, two encounters.

    python scripts/ispass/density_reduce.py <stamp> [--ms 4 8 16 32 64 128] [--csv out.csv]

One row per (background road users M, arm). A run is valid only if its scenario runner did not
fail (no traceback in runner_stdout.txt), the scenario reported M background road users, and the
crossing vehicle is in the per-tick log. Per cell: valid runs, the fraction completed without
collision, the fraction with a false brake (a brake on a track that is not the crossing vehicle),
operational success (no collision and no false brake) with a Wilson 95 % interval, and from the
window in which the crossing vehicle moves, up to the connected vehicle's first brake: coverage
(share of planner ticks with a state for the crossing vehicle), the longest gap, the number of
distinct track identities of the crossing vehicle, the tracks delivered per tick, and the p99 age.
"""
import argparse, csv, json, math, os, re, sys, collections
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
import fidelity_reduce as fr   # noqa: E402
DT = 0.05
ARMS = ["gt_all", "gt_vis", "onboard"]; ENCS = ["open", "occ"]
NAMES = {"gt_all": "all simulator state", "gt_vis": "ground truth in view", "onboard": "camera and LiDAR"}
STATE = {"gt_all": "gt_all", "gt_vis": "gt_visible", "onboard": None}


def window_stats(pt):
    rows = list(csv.DictReader(open(pt)))
    gt = {int(x["tick"]): (float(x["conflict_gt_x"]), float(x["conflict_gt_y"])) for x in rows if x["conflict_gt_x"] not in ("", "nan")}
    spd = {t: math.hypot(gt[t][0] - gt[t - 1][0], gt[t][1] - gt[t - 1][1]) / DT for t in gt if t - 1 in gt}
    moving = sorted(t for t in spd if spd[t] > 2.0)
    if not moving: return None
    t0 = moving[0]; brakes = [int(x["tick"]) for x in rows if x["brake_event"] == "1" and int(x["tick"]) >= t0]
    t1 = min(brakes + [moving[-1]]); win = list(range(t0, t1 + 1))
    cov, ids, per_tick, ages = set(), [], collections.Counter(), []
    for x in rows:
        t = int(x["tick"])
        if t < t0 or t > t1 or x["source"] == "none": continue
        per_tick[t] += 1
        if x["is_conflict_actor"] == "1":
            cov.add(t); ids.append(x["track_id"])
            if x["age_at_use_ms"] not in ("", "nan"): ages.append(float(x["age_at_use_ms"]))
    gap = run = 0
    for t in win:
        run = 0 if t in cov else run + 1; gap = max(gap, run)
    by_tick = {}
    for x in rows: by_tick.setdefault(int(x["tick"]), x)
    ticks = sorted(by_tick); wall = [float(by_tick[t]["wall_time_s"]) for t in ticks]
    speeds = [float(by_tick[t]["ego_speed_mps"]) for t in ticks if t >= t0 and by_tick[t]["ego_speed_mps"] not in ("", "nan")]
    return dict(coverage=len(cov) / len(win), gap_ms=gap * DT * 1000, ids=len(set(ids)), tracks=float(np.mean([per_tick[t] for t in win])),
                first_s=((min(cov) - t0) * DT) if cov else float("nan"),
                brakes=float(sum(1 for t in ticks if by_tick[t]["brake_event"] == "1")), min_speed=min(speeds) if speeds else float("nan"),
                tick_ms=1000.0 * float(np.median(np.diff(wall[20:]))) if len(wall) > 25 else float("nan"))


def load(stamp, ms):
    """Every cell has its own stamp: <stamp>_<encounter>_<arm>_m<M>."""
    out = []; root = os.path.join(ROOT, "experiment_results")
    for enc in ENCS:
        for arm in ARMS:
            for m in ms:
                for r in fr.load_runs(root, f"{stamp}_{enc}_{arm}_m{m}"):
                    run_dir = os.path.dirname(r["path"]); pt = os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")
                    lp = os.path.join(run_dir, "runner_stdout.txt"); log = open(lp, errors="replace").read() if os.path.exists(lp) else ""
                    bg = re.search(r"background road users: (\d+)", log); meta = json.load(open(r["path"]))
                    state_ok = meta.get("config_vehicle_state") == STATE[arm]
                    valid = (not r.get("bad")) and "Traceback" not in log and bg is not None and int(bg.group(1)) == m and os.path.exists(pt) and state_ok
                    ws = window_stats(pt) if valid else None
                    att = fr.attribution(pt) if valid else {}
                    valid = valid and ws is not None
                    fb = int(r.get("focal_fp_episodes", 0) > 0 or att.get("false_brake", 0)) if valid else 0
                    col = int(r.get("focal_collisions", 0) > 0) if valid else 0
                    out.append(dict(enc=enc, m=m, arm=arm, rep=int(r["rep"]), valid=int(valid), collided=col, fbrake=fb,
                                    s_op=int(valid and not col and not fb), onset_s=att.get("tp_onset", float("nan")) / 1000.0,
                                    min_sep=att.get("min_sep", float("nan")), **(ws or {})))
    return out


def summarize(rows, ms):
    summ = []
    for enc in ENCS:
        for m in ms:
            for a in ARMS:
                cell = [r for r in rows if (r["enc"], r["m"], r["arm"]) == (enc, m, a)]; ok = [r for r in cell if r["valid"]]
                if not cell: continue
                n = len(ok); k = n - sum(r["collided"] for r in ok); lo, hi = fr.wilson(k, n) if n else (float("nan"),) * 2
                mean = lambda key: float(np.nanmean([r[key] for r in ok])) if ok else float("nan")
                summ.append(dict(enc=enc, m=m, arm=a, runs=len(cell), valid=n, no_collision=k / n if n else float("nan"), lo=lo, hi=hi,
                                 false_brake=(sum(r["fbrake"] for r in ok) / n) if n else float("nan"),
                                 success=(sum(r["s_op"] for r in ok) / n) if n else float("nan"),
                                 coverage=mean("coverage"), first_s=mean("first_s"), gap_ms=mean("gap_ms"), ids=mean("ids"), tracks=mean("tracks"),
                                 brakes=mean("brakes"), min_speed=mean("min_speed"), tick_ms=mean("tick_ms"),
                                 collisions=sum(r["collided"] for r in ok), onset_s=mean("onset_s"), min_sep=mean("min_sep")))
    return summ


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("stamp"); ap.add_argument("--ms", type=int, nargs="*", default=[4, 8, 16, 32, 64, 128]); ap.add_argument("--csv"); ap.add_argument("--runs-csv", help="one row per run")
    a = ap.parse_args(); rows = load(a.stamp, a.ms); summ = summarize(rows, a.ms)
    print(f"{'enc':5s} {'M':>4} {'arm':22s} runs valid  no coll [95% CI]    f.brake  success  seen  first s  gap ms  ids  tracks  brakes  min v  tick ms  onset s  min sep m")
    for s in summ:
        print(f"{s['enc']:5s} {s['m']:4d} {NAMES[s['arm']]:22s} {s['runs']:4d} {s['valid']:5d}  {s['no_collision']:5.2f} [{s['lo']:.2f},{s['hi']:.2f}]  {s['false_brake']:7.2f}  "
              f"{s['success']:7.2f}  {s['coverage']:4.2f}  {s['first_s']:7.2f}  {s['gap_ms']:6.0f}  {s['ids']:3.1f}  {s['tracks']:6.1f}  {s['brakes']:6.1f}  {s['min_speed']:5.1f}  {s['tick_ms']:7.0f}  {s['onset_s']:7.2f}  {s['min_sep']:9.1f}")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
        print("csv:", a.csv)
    if a.runs_csv:
        keys = ["enc", "m", "arm", "rep", "valid", "collided", "fbrake", "onset_s", "min_sep", "first_s", "tick_ms"]
        with open(a.runs_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore"); w.writeheader(); w.writerows(rows)
        print("runs csv:", a.runs_csv)
