#!/usr/bin/env python3
"""pNEUMA measured-point rows in the compare_measured metro schema, one CSV per
admission row (demand / admit92 / admit50 / deployed). within_limit is the
LOAD-COUPLED fraction (the figure's right axis); within_limit_mechanism is the
mechanism-only fraction (excludes load-induced rate fallback).
Usage: pneuma_schema_rows.py <metro_dir>   (reads pneuma_0900_replay_<row>.json)"""
import csv, json, os, sys
def _rg(r, key, nd):
    v = ((r.get("instrumentation") or {}).get("registry") or {}).get(key)
    return round(v, nd) if v is not None else ""
def _im(r, key, stat):
    v = (r.get("instrumentation") or {}).get(key) or {}
    return round(v[stat], 3) if v.get(stat) is not None else ""
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
                     "vehicles_per_lane_km": ts.get("vehicles_per_lane_km"), "degree_used": j.get("degree_used"), "charging_curve": j.get("charging_curve"), "trace": j.get("trace"), "trace_sha": j.get("trace_sha"),
                     "tracks_per_locale": (round(j["tracks_per_locale"], 2) if j.get("tracks_per_locale") is not None else ""),
                         "prepares_per_dest_cycle": _rg(r, "prepares_per_dest_cycle", 5), "prepare_cycle_frac": _rg(r, "prepare_cycle_frac", 5), "prepares_p95_given_any": _rg(r, "prepares_p95_given_any", 2),
                         "predicted_crossers_per_locale": _rg(r, "predicted_crossers_per_locale", 2), "refreshes_per_dest_cycle": _rg(r, "refreshes_per_dest_cycle", 5), "refreshes_p95_given_any": _rg(r, "refreshes_p95_given_any", 2),
                         "crossing_burst_p95": _rg(r, "crossing_burst_p95", 2), "never_crossed_frac": _rg(r, "never_crossed_frac", 4),
                     "notes": ("coupled freshness = deployed rate fallback replayed per locale, induced work at skipped cycles not deferred (the lighter case for replication)" + ("; low-density floor: one padded call plus one remainder call on a locale with no owned tracks" if (arm in ("replall", "delta_ready") and (ts.get("active_tracks_p50") or 0) <= 450) else "")),})
    out = os.path.join(D, f"pneuma_0900_metro_rows_{row}.csv")
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{row}: {len(rows)} rows -> {os.path.basename(out)}")
    for r in rows: print(f"   {r['policy']:12s} load {r['dest_load_p95']:6.3f}  within mech {r['within_limit_mechanism']:.3f} COUPLED {r['within_limit']:.3f}  rate_k>1 {r['rate_k_gt1_frac']:.3f}  pass p95 {r['pass_p95_ms']:.0f} ms (owned-only {r['pass_p95_owned_only_ms']:.0f})")
