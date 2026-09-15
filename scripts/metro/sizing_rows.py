#!/usr/bin/env python3
"""Lander for the locale-sizing replay sweep (Figure 11 rebuilt measured).

Reads one replay JSON per tile side (metro_replay.py, warm arm, deployed row)
and the canvas latency sweep, writes frozen1r_sizing_rows.csv (schema in
docs/agent_plans/locale_sizing_replay_design.md) and prints the invalidation
checks. Usage:
  python sizing_rows.py --canvas evaluation_outputs/canvas_latency.csv \
      --out evaluation_outputs/metro/sizing/frozen1r_sizing_rows.csv \
      evaluation_outputs/metro/sizing/replay_sizing_t*.json
"""
import argparse
import csv
import glob
import json
import os
import re
import sys

import numpy as np

COLS = ["tile_side_m", "locales", "tracks_per_locale", "fusion_util", "dest_load_p95", "within_limit",
        "crossings_per_track_per_min", "prepares_per_locale_cycle", "never_crossed_frac", "late_prepare_frac",
        "unprepared_crossing_frac", "boundaries", "degree_used", "dest_load_mean", "within_limit_mechanism",
        "fusion_measured", "eval_tag", "notes"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("jsons", nargs="+")
    ap.add_argument("--canvas", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--contributors", type=int, default=8)
    ap.add_argument("--allocation_ms", type=float, default=130.0)
    ap.add_argument("--tag", default="khonsu-eval-frozen-1r")
    ap.add_argument("--reference", default="", help="landed deployed JSON to reproduce at its tile side (condition 1)")
    a = ap.parse_args()
    cv = [r for r in csv.DictReader(open(a.canvas)) if int(r["n_contributors"]) == a.contributors and r["status"] == "ok"]
    cm = np.array(sorted(float(r["canvas_m"]) for r in cv)); lm = np.array([float(r["latency_ms"]) for r in sorted(cv, key=lambda r: float(r["canvas_m"]))])
    rows = []
    for f in sorted(set(sum([glob.glob(g) for g in a.jsons], []))):
        j = json.load(open(f)); w = j["arms"]["warm"]; rg = (w.get("instrumentation") or {}).get("registry") or {}
        ts = j.get("trace_stats") or {}
        side = int(j.get("tile_m") or re.search(r"_t(\d+)", os.path.basename(f)).group(1))
        measured = side <= cm[-1]
        rows.append({
            "tile_side_m": side, "locales": j["L"],
            "tracks_per_locale": round(j["tracks_per_locale"], 2) if j.get("tracks_per_locale") is not None else "",
            "fusion_util": round(float(np.interp(side, cm, lm)) / a.allocation_ms, 4) if measured else "",
            "dest_load_p95": round(w["p95_load_frac"], 4), "within_limit": round(w["within_limit_postcross_coupled"], 4),
            "crossings_per_track_per_min": round(ts["crossing_rate_per_track_s_5hz"] * 60.0, 4) if ts.get("crossing_rate_per_track_s_5hz") is not None else "",
            "prepares_per_locale_cycle": round(rg["prepares_per_dest_cycle"], 5) if rg.get("prepares_per_dest_cycle") is not None else "",
            "never_crossed_frac": round(rg["never_crossed_frac"], 5) if rg.get("never_crossed_frac") is not None else "",
            "late_prepare_frac": round(rg["late_prepare_frac"], 5) if rg.get("late_prepare_frac") is not None else "",
            "unprepared_crossing_frac": round(rg["unprepared_crossing_frac"], 5) if rg.get("unprepared_crossing_frac") is not None else "",
            "boundaries": j.get("boundaries"), "degree_used": round(j.get("degree_used", 0), 3),
            "dest_load_mean": round(w["mean_load_frac"], 5), "within_limit_mechanism": round(w["within_limit_postcross_frac"], 4),
            "fusion_measured": 1 if measured else 0, "eval_tag": a.tag,
            "notes": ("fusion beyond the measured sweep (563 m); infeasible by its last point" if not measured else ""),
        })
    rows.sort(key=lambda r: r["tile_side_m"])
    # duplicate partitions (same locale count and boundaries as the previous side)
    for i in range(1, len(rows)):
        if rows[i]["locales"] == rows[i - 1]["locales"] and rows[i]["boundaries"] == rows[i - 1]["boundaries"] and rows[i]["dest_load_p95"] == rows[i - 1]["dest_load_p95"]:
            rows[i]["notes"] = (rows[i]["notes"] + "; " if rows[i]["notes"] else "") + f"same partition as {rows[i-1]['tile_side_m']} m"
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{len(rows)} rows -> {a.out}")
    # ---- checks
    ok = True
    sides = [r["tile_side_m"] for r in rows]; locs = [r["locales"] for r in rows]
    tpl = [float(r["tracks_per_locale"]) for r in rows if r["tracks_per_locale"] != ""]
    cr = [float(r["crossings_per_track_per_min"]) for r in rows if r["crossings_per_track_per_min"] != ""]
    c2 = all(locs[i] >= locs[i + 1] for i in range(len(locs) - 1)); c2b = all(tpl[i] <= tpl[i + 1] + 1e-9 for i in range(len(tpl) - 1))
    c3 = all(cr[i] >= cr[i + 1] - 1e-9 for i in range(len(cr) - 1))
    print(f"check 2 locales non-increasing with side: {c2}; tracks/locale non-decreasing: {c2b}")
    print(f"check 3 crossings/track/min non-increasing with side: {c3}" + ("" if c3 else "  VIOLATION: " + str([(s, round(c, 3)) for s, c in zip(sides, cr)])))
    ok = ok and c2 and c2b and c3
    if a.reference:
        ref = json.load(open(a.reference)); rw = ref["arms"]["warm"]; rr = (rw.get("instrumentation") or {}).get("registry") or {}
        mine = [r for r in rows if r["tile_side_m"] == int(ref.get("tile_m", 300))]
        if mine:
            m = mine[0]
            same = (abs(m["dest_load_p95"] - round(rw["p95_load_frac"], 4)) < 1e-9 and abs(m["within_limit"] - round(rw["within_limit_postcross_coupled"], 4)) < 1e-9
                    and abs(float(m["never_crossed_frac"]) - round(rr["never_crossed_frac"], 5)) < 1e-9 and abs(float(m["prepares_per_locale_cycle"]) - round(rr["prepares_per_dest_cycle"], 5)) < 1e-9)
            print(f"check 1 reproduces the landed deployed row at {ref.get('tile_m')} m: {same}  (load {m['dest_load_p95']} vs {round(rw['p95_load_frac'], 4)}; within {m['within_limit']} vs {round(rw['within_limit_postcross_coupled'], 4)}; never {m['never_crossed_frac']} vs {round(rr['never_crossed_frac'], 5)})")
            ok = ok and same
        else:
            print("check 1: no row at the reference side"); ok = False
    feas = [r["tile_side_m"] for r in rows if r["fusion_util"] != "" and r["fusion_util"] < 1.0 and r["dest_load_p95"] < 1.0]
    print(f"feasible sides (fusion < 1 and dest load < 1): {feas[0] if feas else None} to {feas[-1] if feas else None}")
    for r in rows:
        print(f"  {r['tile_side_m']:4d} m  L={r['locales']:4d}  tpl={r['tracks_per_locale']:>7}  fusion={r['fusion_util'] if r['fusion_util'] != '' else 'n/a':>6}  load={r['dest_load_p95']:.3f}  within={r['within_limit']:.3f}  cross/min={r['crossings_per_track_per_min']}  prep/lc={r['prepares_per_locale_cycle']}  never={r['never_crossed_frac']}  late={r['late_prepare_frac']}  unprep={r['unprepared_crossing_frac']}  {r['notes']}")
    print("ALL CHECKS PASS" if ok else "CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
