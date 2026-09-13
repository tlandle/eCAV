#!/usr/bin/env python3
"""pNEUMA measured-point rows in the compare_measured metro schema, one CSV per
admission row (demand / admit92 / admit50 / deployed). within_limit is the
LOAD-COUPLED fraction (the figure's right axis); within_limit_mechanism is the
mechanism-only fraction (excludes load-induced rate fallback).
Usage: pneuma_schema_rows.py <metro_dir>   (reads pneuma_0900_replay_<row>.json)"""
import csv, json, os, sys
D = sys.argv[1]
for row in ("demand", "admit92", "admit50", "deployed"):
    f = os.path.join(D, f"pneuma_0900_replay_{row}.json")
    if not os.path.exists(f): print("missing", f); continue
    j = json.load(open(f)); o = j["n_owned_per_locale_cycle"]; ts = j.get("trace_stats") or {}; rows = []
    for arm, r in j["arms"].items():
        rows.append({"locales": j["L"], "tracks": round(ts.get("active_tracks_p50") or 0), "policy": arm,
                     "dest_load_p95": round(r["p95_load_frac"], 4), "dest_load": round(r["mean_load_frac"], 5),
                     "within_limit": round(r["within_limit_postcross_coupled"], 4), "within_limit_mechanism": round(r["within_limit_postcross_frac"], 4),
                     "within_limit_pooled": round(r["within_limit_frac_coupled"], 4), "within_limit_pooled_mechanism": round(r["within_limit_frac"], 4),
                     "rate_k_gt1_frac": round(r["rate_k_frac_gt1_at_consumption"], 4), "rate_k_mean": round(r["rate_k_mean_at_consumption"], 3),
                     "rate_k_dist": "|".join(f"{k}:{(v or 0):.3f}" for k, v in r["rate_k_frac_owned_locale_cycles"].items()),
                     "pass_p95_ms": round(r["pass_ms_p95_executed"], 1), "pass_p95_owned_only_ms": round(r["pass_ms_p95_owned_only"] or 0, 1),
                     "load_standalone": round(r["p95_load_frac_standalone"], 4), "displacement_mean": r.get("displaced_owned_frac_mean"), "displacement_p95": r.get("displaced_owned_frac_p95"),
                     "K_admit": o.get("K_admit"), "admit_rule": o.get("admit_rule"), "bytes_per_s_per_locale": round(r["bytes_per_s_per_locale"]),
                     "crossing_rate_per_track_s": ts.get("crossing_rate_per_track_s_5hz"), "near_boundary_frac_20m": (ts.get("near_boundary_frac") or {}).get("20"),
                     "vehicles_per_lane_km": ts.get("vehicles_per_lane_km"), "degree_used": j.get("degree_used"), "charging_curve": j.get("charging_curve"), "trace": j.get("trace"), "trace_sha": j.get("trace_sha")})
    out = os.path.join(D, f"pneuma_0900_metro_rows_{row}.csv")
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{row}: {len(rows)} rows -> {os.path.basename(out)}")
    for r in rows: print(f"   {r['policy']:12s} load {r['dest_load_p95']:6.3f}  within mech {r['within_limit_mechanism']:.3f} COUPLED {r['within_limit']:.3f}  rate_k>1 {r['rate_k_gt1_frac']:.3f}  pass p95 {r['pass_p95_ms']:.0f} ms (owned-only {r['pass_p95_owned_only_ms']:.0f})")
