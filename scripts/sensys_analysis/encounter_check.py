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
EGO_LEN, EGO_WID = 4.9, 2.1   # m, vehicle footprints (Lincoln MKZ ego, Tesla Model 3 crossing car)
CAR_LEN, CAR_WID = 4.7, 1.9
LANE_W = 3.5
# The conflict region is the crossing of the two lanes: the car's lane (y about 127.7, width 3.5 m)
# across the ego's lane (x about -84.8, width 3.5 m).
CONFLICT_BOX = (CONFLICT[0] - LANE_W / 2, 127.7 - LANE_W / 2, CONFLICT[0] + LANE_W / 2, 127.7 + LANE_W / 2)

try:
    from shapely.geometry import box as _box
    from shapely import affinity as _aff
    _HAVE_SHAPELY = True
except Exception:   # pragma: no cover
    _HAVE_SHAPELY = False


def _footprint(x, y, heading_rad, length, width):
    b = _box(-length / 2, -width / 2, length / 2, width / 2)
    return _aff.translate(_aff.rotate(b, math.degrees(heading_rad), origin=(0, 0)), x, y)


def _headings(xy, window=2):
    """Heading per tick from the position change over +/- window ticks; a
    vehicle at rest keeps its last heading (or the first moving one)."""
    ticks = sorted(xy)
    out, last = {}, None
    pend = []
    for i, t in enumerate(ticks):
        a = xy[ticks[max(0, i - window)]]; b = xy[ticks[min(len(ticks) - 1, i + window)]]
        dx, dy = b[0] - a[0], b[1] - a[1]
        if math.hypot(dx, dy) > 0.2:
            last = math.atan2(dy, dx)
            for q in pend:
                out[q] = last
            pend = []
            out[t] = last
        elif last is not None:
            out[t] = last
        else:
            pend.append(t)
    for q in pend:
        out[q] = 0.0
    return out


def footprint_metrics(ego_xy, gt, launch):
    """Footprint-based geometry: minimum distance between the two vehicle
    rectangles over the logged ticks (0 when they intersect), whether they
    ever intersect, and each vehicle's first and last tick inside the
    conflict box, relative to launch."""
    nan = float("nan")
    if not _HAVE_SHAPELY or not ego_xy or not gt:
        return dict(fp_min_m=nan, fp_hit=None, ego_in=nan, ego_out=nan, car_in=nan, car_out=nan)
    eh, ch = _headings(ego_xy), _headings(gt)
    cbox = _box(*CONFLICT_BOX)
    fp_min, fp_hit = nan, 0
    ego_t, car_t = [], []
    for t in sorted(set(ego_xy) | set(gt)):
        E = _footprint(ego_xy[t][0], ego_xy[t][1], eh[t], EGO_LEN, EGO_WID) if t in ego_xy else None
        C = _footprint(gt[t][0], gt[t][1], ch[t], CAR_LEN, CAR_WID) if t in gt else None
        if E is not None and C is not None:
            d = E.distance(C)
            if fp_min != fp_min or d < fp_min:
                fp_min = d
            if d <= 0.0:
                fp_hit = 1
        if E is not None and E.intersects(cbox):
            ego_t.append(t)
        if C is not None and C.intersects(cbox):
            car_t.append(t)
    rel = lambda L, f: ((f(L) - launch) * 0.05 if L else nan)
    return dict(fp_min_m=fp_min, fp_hit=fp_hit, ego_in=rel(ego_t, min), ego_out=rel(ego_t, max),
                car_in=rel(car_t, min), car_out=rel(car_t, max))


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
    r.update(footprint_metrics(ego_xy, gt, launch))
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
            # Footprint geometry from the free-flow run: the ego's and the car's windows inside the
            # conflict box, whether those windows overlap, the minimum footprint distance and whether
            # the footprints ever intersect (the collision test), plus the simulator's collision count.
            row.update(ff_enter_s=f["ego_in"], ff_leave_s=f["ego_out"], ff_car_in_s=f["car_in"], ff_car_out_s=f["car_out"],
                       ff_min_sep_m=f["fp_min_m"], ff_footprints_hit=f["fp_hit"], ff_collided=f["collided"],
                       ff_centre_sep_m=f["min_sep_m"])
            overlap = (f["ego_in"] == f["ego_in"] and f["car_in"] == f["car_in"]
                       and not (f["ego_out"] < f["car_in"] or f["ego_in"] > f["car_out"]))
            row["overlap"] = int(overlap)
        else:
            row.update(ff_enter_s=float("nan"), ff_leave_s=float("nan"), ff_car_in_s=float("nan"), ff_car_out_s=float("nan"),
                       ff_min_sep_m=float("nan"), ff_footprints_hit="", ff_collided="", ff_centre_sep_m=float("nan"), overlap="")
        n = analyse(nc[key]) if key in nc else None
        if n:
            # Evasive action = a brake attribution for a track (the speed controller's own throttle and
            # brake pulses at a low set point do not count).
            row["nocoop_outcome"] = "collision" if n["collided"] else ("evasive action" if n["brake_s"] == n["brake_s"] else "uneventful passage")
            row["nocoop_min_sep_m"] = n["fp_min_m"]
            row["nocoop_brake_s"] = n["brake_s"]
        else:
            row["nocoop_outcome"] = ""; row["nocoop_min_sep_m"] = float("nan"); row["nocoop_brake_s"] = float("nan")
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
             "| scenario | target | n | v launch | v brake | brake +s | car reach +s | car clear +s | first useful +s / age | zero-age outcomes | free-flow ego window in box | car window in box | overlap | footprint min sep | footprints hit | no-coop outcomes |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    med = lambda L: statistics.median([x for x in L if x == x]) if any(x == x for x in L) else float("nan")
    fm = lambda v, p=2: ("%.*f" % (p, v)) if v == v else "-"
    for k in sorted(cells):
        rs = cells[k]
        oc = collections.Counter(r["zero_age_outcome"] for r in rs)
        nco = collections.Counter(r["nocoop_outcome"] for r in rs if r["nocoop_outcome"])
        ov = [r["overlap"] for r in rs if r["overlap"] != ""]
        hits = [r["ff_footprints_hit"] for r in rs if r["ff_footprints_hit"] != ""]
        lines.append("| %s | %d | %d | %s | %s | %s | %s | %s | %s / %s | %s | %s to %s | %s to %s | %s | %s | %s | %s |" % (
            k[0], k[1], len(rs), fm(med([r["v_launch"] for r in rs]), 1), fm(med([r["v_brake"] for r in rs]), 1),
            fm(med([r["brake_s"] for r in rs])), fm(med([r["car_reach_s"] for r in rs])), fm(med([r["car_clear_s"] for r in rs])),
            fm(med([r["first_useful_s"] for r in rs])), fm(med([r["first_useful_age_ms"] for r in rs]), 0),
            ", ".join(f"{v} {c}" for c, v in oc.items()),
            fm(med([r["ff_enter_s"] for r in rs])), fm(med([r["ff_leave_s"] for r in rs])),
            fm(med([r["ff_car_in_s"] for r in rs])), fm(med([r["ff_car_out_s"] for r in rs])),
            (f"{sum(ov)}/{len(ov)}" if ov else "pending"), fm(med([r["ff_min_sep_m"] for r in rs]), 1),
            (f"{sum(hits)}/{len(hits)}" if hits else "pending"),
            (", ".join(f"{v} {c}" for c, v in nco.items()) if nco else "pending")))
    open(os.path.join(out, "encounter_check.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
