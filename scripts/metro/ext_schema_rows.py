#!/usr/bin/env python3
"""Extension rows in the compare_measured metro schema, one CSV per admission
variant (demand / admit92 / deployed), plus a by-point table against the
generated L=100 rows at the same tracks. Usage: ext_schema_rows.py <ext_dir> <generated.csv>"""
import csv, glob, json, re, sys
E, GEN = sys.argv[1], sys.argv[2]
gen = [r for r in csv.DictReader(open(GEN)) if r["locales"] == "100"]
by = {"none": "demand", "single": "admit92", "deployed": "deployed"}
out = {v: [] for v in by.values()}; pts = {}
for f in sorted(glob.glob(f"{E}/replay_ext_L*_t*_s1_*.json")):
    j = json.load(open(f)); m = re.search(r"_L(\d+)_t(\d+)_s1_(\w+)\.json", f); L, t, rule = int(m.group(1)), int(m.group(2)), m.group(3)
    ts = j.get("trace_stats") or {}; o = j["n_owned_per_locale_cycle"]; tag = by[rule]
    for arm, r in j["arms"].items():
        out[tag].append({"locales": L, "tracks": t, "tracks_active_onnet_p50": round(ts.get("active_tracks_p50") or 0), "policy": arm,
                         "dest_load_p95": round(r["p95_load_frac"], 4), "dest_load": round(r["mean_load_frac"], 5),
                         # within_limit = LOAD-COUPLED (the figure's right axis: deployed rate fallback replayed per locale);
                         # within_limit_mechanism = mechanism-only (excludes load-induced rate fallback)
                         "within_limit": round(r["within_limit_postcross_coupled"], 4), "within_limit_mechanism": round(r["within_limit_postcross_frac"], 4),
                         "within_limit_pooled": round(r["within_limit_frac_coupled"], 4), "within_limit_pooled_mechanism": round(r["within_limit_frac"], 4),
                         "rate_k_gt1_frac": round(r["rate_k_frac_gt1_at_consumption"], 4), "rate_k_mean": round(r["rate_k_mean_at_consumption"], 3),
                         "rate_k_dist": "|".join(f"{k}:{(v or 0):.3f}" for k, v in r["rate_k_frac_owned_locale_cycles"].items()),
                         "pass_p95_ms": round(r["pass_ms_p95_executed"], 1), "pass_p95_owned_only_ms": round(r["pass_ms_p95_owned_only"] or 0, 1), "load_standalone": round(r["p95_load_frac_standalone"], 4),
                         "displacement_mean": r.get("displaced_owned_frac_mean"), "displacement_p95": r.get("displaced_owned_frac_p95"), "K_admit": o.get("K_admit"), "admit_rule": rule,
                         "crossing_rate_per_track_s": ts.get("crossing_rate_per_track_s_5hz"), "near_boundary_frac_20m": (ts.get("near_boundary_frac") or {}).get("20"),
                         "vehicles_per_lane_km": ts.get("vehicles_per_lane_km"), "degree_used": j.get("degree_used"), "label": "calibrated extrapolation (SUMO on the real Athens network)",
                         "notes": ("coupled freshness = deployed rate fallback replayed per locale, induced work at skipped cycles not deferred (the lighter case for replication)" + ("; low-density floor: one padded call plus one remainder call on a locale with no owned tracks" if (arm in ("replall", "delta_ready") and (ts.get("active_tracks_p50") or 0) <= 450) else "")),})
    pts[(t, rule)] = j
for tag, rows in out.items():
    if not rows: continue
    with open(f"{E}/ext_metro_rows_{tag}.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in sorted(rows, key=lambda r: (r["tracks"], r["policy"]))]
    print(f"{tag}: {len(rows)} rows -> ext_metro_rows_{tag}.csv")
print("\nby-point vs generated L=100 (dest_load_p95 as fraction of slack; measured demand / K16 / deployed):")
print(f"{'tracks':>6s} {'policy':12s} {'gen p95':>8s} {'gen within':>10s} | {'meas demand':>11s} {'K16':>7s} {'deployed':>8s} | within mech (demand/K16/deployed)  within COUPLED (demand/K16/deployed)  rate_k>1 (deployed)")
for t in sorted({t for t, r in pts}):
    for pol in ("warm", "replall", "delta_state", "delta_ready", "reactive"):
        g = [r for r in gen if r["policy"] == pol and int(r["tracks"]) == t]
        gp = f"{float(g[0]['dest_load_p95']):8.3f}" if g else "     n/a"; gw = f"{float(g[0]['within_limit']):10.3f}" if g else "       n/a"
        vals = [pts.get((t, rule)) for rule in ("none", "single", "deployed")]
        ms = " ".join(f"{(v['arms'][pol]['p95_load_frac'] if v else float('nan')):7.3f}" for v in vals)
        wm = " ".join(f"{(v['arms'][pol]['within_limit_postcross_frac'] if v else float('nan')):6.3f}" for v in vals)
        wc = " ".join(f"{(v['arms'][pol]['within_limit_postcross_coupled'] if v else float('nan')):6.3f}" for v in vals)
        rk = vals[2]["arms"][pol]["rate_k_frac_gt1_at_consumption"] if vals[2] else float("nan")
        print(f"{t:6d} {pol:12s} {gp} {gw} | {ms} | {wm}   {wc}   {rk:6.3f}")
