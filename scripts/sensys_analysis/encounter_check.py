#!/usr/bin/env python3
"""Encounter geometry check per scenario, target speed and seed (review of Oct 29, section 6).

    python scripts/sensys_analysis/encounter_check.py [--stamp sensys_nov27_j5] [--lat 0] [--mgr oracle]
        [--roots experiment_results experiment_results/_azure_mirror/experiment_results ...]
        [--freeflow-stamp <stamp of the no-evasion runs>] [--nocoop-stamp <stamp of the local-only runs>]
        [--out safety_envelope_paper/analysis/encounter_check_<stamp>]

From each run's per-tick age log and brake record: the ego's speed when the
crossing car launches (first ground-truth motion), its speed at the first
brake for a shared track, the car's arrival at and clearance of the conflict
point, the first useful shared observation (first consumed moving state of
the conflict actor: tick after launch and age at use), and the outcome. When
the no-evasion and local-only stamps are given, their runs supply the ego's
free-flow arrival, the occupancy overlap, the minimum separation (needs the
ego position columns in the per-tick log) and the no-cooperation outcome,
matched by scenario, speed and seed.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import GEOM_RE, ROOT  # noqa: E402

CONFLICT = (-84.8, 127.9)
CLEAR_M = 5.0      # the car has cleared when it is this far past the conflict point
EGO_LEN = 4.9      # m, vehicle footprints for the occupancy window
CAR_LEN = 4.7


def fl(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def load_log(path):
    rows = list(csv.DictReader(open(path)))
    gt, ego_v, ego_xy = {}, {}, {}
    for r in rows:
        t = int(r["tick"])
        if r.get("conflict_gt_x") not in (None, "", "nan"):
            gt.setdefault(t, (fl(r["conflict_gt_x"]), fl(r["conflict_gt_y"])))
        v = fl(r.get("ego_speed_mps"))
        if v == v:
            ego_v[t] = v
        if r.get("ego_x") not in (None, "", "nan"):
            ego_xy[t] = (fl(r["ego_x"]), fl(r["ego_y"]))
    return rows, gt, ego_v, ego_xy


def launch_tick(gt):
    ticks = sorted(gt)
    for a, b in zip(ticks, ticks[1:]):
        if math.hypot(gt[b][0] - gt[a][0], gt[b][1] - gt[a][1]) > 0.05:
            return b
    return None


def car_times(gt, launch):
    ticks = sorted(gt)
    reach = min((t for t in ticks if gt[t][0] <= CONFLICT[0]), default=None)
    clear = min((t for t in ticks if gt[t][0] <= CONFLICT[0] - CLEAR_M), default=None)
    return ((reach - launch) * 0.05 if reach else float("nan"), (clear - launch) * 0.05 if clear else float("nan"))


def ego_conflict_times(ego_xy, launch, radius=None):
    """First and last tick the ego centre is within half a car length of the
    conflict point, relative to launch (needs ego_x/ego_y)."""
    if not ego_xy:
        return float("nan"), float("nan")
    r = radius or (EGO_LEN / 2 + 1.0)
    inside = [t for t, (x, y) in ego_xy.items() if math.hypot(x - CONFLICT[0], y - CONFLICT[1]) <= r]
    if not inside:
        return float("nan"), float("nan")
    return (min(inside) - launch) * 0.05, (max(inside) - launch) * 0.05


def min_separation(ego_xy, gt):
    best = float("nan")
    for t, (x, y) in ego_xy.items():
        if t in gt:
            d = math.hypot(x - gt[t][0], y - gt[t][1])
            if best != best or d < best:
                best = d
    return best


def run_rows(roots, stamp, mgr, lat):
    out = {}
    for root in roots:
        for f in glob.glob(os.path.join(root, "*", stamp, "**", "simulation_metrics.json"), recursive=True):
            if "/evaluation_output/" in f:
                continue
            m = GEOM_RE.search(f)
            if not m:
                continue
            geom, spd, mg, anch, cfg, rep = m.groups()
            if mg != mgr or not cfg.startswith(f"lat_{lat}_"):
                continue
            out[(geom, int(spd), int(rep))] = f
    return out


def analyse(f):
    p = f.replace("simulation_metrics.json", "evaluation_output/per_tick_age.csv")
    if not os.path.exists(p):
        return None
    rows, gt, ego_v, ego_xy = load_log(p)
    launch = launch_tick(gt)
    if launch is None or not ego_v:
        return None
    d = json.load(open(f))
    fv = d["vehicles"][min(d["vehicles"], key=int)]
    ba = [a for a in fv.get("brake_attributions", []) if (a.get("prediction_horizon_s") or 3) >= 1.0
          and a.get("trigger_tick") is not None and a["trigger_tick"] >= launch]
    near = lambda t: ego_v.get(t) if t in ego_v else ego_v[min(ego_v, key=lambda k: abs(k - t))]
    r = dict(launch_tick=launch, v_launch=near(launch))
    r["car_reach_s"], r["car_clear_s"] = car_times(gt, launch)
    if ba:
        a = ba[0]
        r["brake_s"] = (a["trigger_tick"] - launch) * 0.05
        r["v_brake"] = a["ego_speed_mps"]
        r["d_brake"] = math.hypot(a["ego_x"] - CONFLICT[0], a["ego_y"] - CONFLICT[1])
    else:
        r["brake_s"] = r["v_brake"] = r["d_brake"] = float("nan")
    conf = [x for x in rows if x.get("is_conflict_actor") == "1" and fl(x.get("obs_speed_mps")) > 1.0]
    if conf:
        first = min(conf, key=lambda x: int(x["tick"]))
        r["first_useful_s"] = (int(first["tick"]) - launch) * 0.05
        r["first_useful_age_ms"] = fl(first.get("age_at_use_ms"))
    else:
        r["first_useful_s"] = r["first_useful_age_ms"] = float("nan")
    r["collided"] = int((fv.get("safety") or {}).get("collision_count", 0) > 0)
    r["ego_enter_s"], r["ego_leave_s"] = ego_conflict_times(ego_xy, launch)
    r["min_sep_m"] = min_separation(ego_xy, gt)
    r["max_brake_ctrl"] = max((fl(x.get("ctrl_brake")) for x in rows if fl(x.get("ctrl_brake")) == fl(x.get("ctrl_brake"))), default=float("nan"))
    return r


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stamp", default="sensys_nov27_j5")
    ap.add_argument("--mgr", default="oracle")
    ap.add_argument("--lat", type=int, default=0)
    ap.add_argument("--roots", nargs="*", default=[os.path.join(ROOT, "experiment_results"),
                                                   os.path.join(ROOT, "experiment_results", "_azure_mirror", "experiment_results"),
                                                   os.path.join(ROOT, "experiment_results", "_cetus_mirror", "experiment_results")])
    ap.add_argument("--freeflow-stamp", default=None)
    ap.add_argument("--nocoop-stamp", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "safety_envelope_paper", "analysis", "encounter_check_" + a.stamp.replace("sensys_nov27_", ""))
    os.makedirs(out, exist_ok=True)
    ref = run_rows(a.roots, a.stamp, a.mgr, a.lat)
    ff = run_rows(a.roots, a.freeflow_stamp, "oracle", a.lat) if a.freeflow_stamp else {}
    nc = run_rows(a.roots, a.nocoop_stamp, "late_fusion", a.lat) if a.nocoop_stamp else {}
    rows = []
    for key in sorted(ref):
        geom, spd, rep = key
        r = analyse(ref[key])
        if r is None:
            continue
        row = dict(scenario="left turn" if geom.startswith("openscenario_3") else "straight crossing", target_kmh=spd, seed=rep)
        row.update({k: r[k] for k in ("v_launch", "v_brake", "d_brake", "brake_s", "car_reach_s", "car_clear_s",
                                      "first_useful_s", "first_useful_age_ms", "collided")})
        row["zero_age_outcome"] = "collision" if r["collided"] else ("brake, no collision" if r["brake_s"] == r["brake_s"] else "no brake, no collision")
        f = analyse(ff[key]) if key in ff else None
        if f:
            row.update(ff_enter_s=f["ego_enter_s"], ff_leave_s=f["ego_leave_s"], ff_min_sep_m=f["min_sep_m"], ff_collided=f["collided"])
            overlap = (f["ego_enter_s"] == f["ego_enter_s"] and not (f["ego_leave_s"] < r["car_reach_s"] or f["ego_enter_s"] > r["car_clear_s"]))
            row["overlap"] = int(overlap)
        else:
            row.update(ff_enter_s=float("nan"), ff_leave_s=float("nan"), ff_min_sep_m=float("nan"), ff_collided="", overlap="")
        n = analyse(nc[key]) if key in nc else None
        if n:
            row["nocoop_outcome"] = "collision" if n["collided"] else ("evasive action" if n["brake_s"] == n["brake_s"] or (n["max_brake_ctrl"] == n["max_brake_ctrl"] and n["max_brake_ctrl"] > 0.3) else "uneventful passage")
            row["nocoop_min_sep_m"] = n["min_sep_m"]
        else:
            row["nocoop_outcome"] = ""; row["nocoop_min_sep_m"] = float("nan")
        rows.append(row)
    if not rows:
        print("no rows"); return 2
    with open(os.path.join(out, "encounter_check.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    # per-cell summary (medians)
    cells = collections.defaultdict(list)
    for row in rows:
        cells[(row["scenario"], row["target_kmh"])].append(row)
    lines = ["# Encounter check (reference at %d ms, stamp %s)" % (a.lat, a.stamp), "",
             "Per cell: seeds, ego speed at the car's launch and at the first brake (m/s), first brake after launch (s), "
             "car reaches and clears the conflict point after launch (s), first useful shared observation (s after launch, age ms), "
             "zero-age outcomes, free-flow ego window at the conflict (s, from the no-evasion runs), occupancy overlap, "
             "free-flow minimum separation (m), no-cooperation outcomes.", "",
             "| scenario | target | n | v launch | v brake | brake +s | car reach +s | car clear +s | first useful +s / age | zero-age outcomes | free-flow ego window | overlap | min sep | no-coop outcomes |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    med = lambda L: statistics.median([x for x in L if x == x]) if any(x == x for x in L) else float("nan")
    fm = lambda v, p=2: ("%.*f" % (p, v)) if v == v else "-"
    for k in sorted(cells):
        rs = cells[k]
        oc = collections.Counter(r["zero_age_outcome"] for r in rs)
        nco = collections.Counter(r["nocoop_outcome"] for r in rs if r["nocoop_outcome"])
        ov = [r["overlap"] for r in rs if r["overlap"] != ""]
        lines.append("| %s | %d | %d | %s | %s | %s | %s | %s | %s / %s | %s | %s to %s | %s | %s | %s |" % (
            k[0], k[1], len(rs), fm(med([r["v_launch"] for r in rs]), 1), fm(med([r["v_brake"] for r in rs]), 1),
            fm(med([r["brake_s"] for r in rs])), fm(med([r["car_reach_s"] for r in rs])), fm(med([r["car_clear_s"] for r in rs])),
            fm(med([r["first_useful_s"] for r in rs])), fm(med([r["first_useful_age_ms"] for r in rs]), 0),
            ", ".join(f"{v} {c}" for c, v in oc.items()),
            fm(med([r["ff_enter_s"] for r in rs])), fm(med([r["ff_leave_s"] for r in rs])),
            (f"{sum(ov)}/{len(ov)}" if ov else "pending"), fm(med([r["ff_min_sep_m"] for r in rs]), 1),
            (", ".join(f"{v} {c}" for c, v in nco.items()) if nco else "pending")))
    open(os.path.join(out, "encounter_check.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
