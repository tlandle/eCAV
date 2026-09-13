#!/usr/bin/env python3
"""Assemble the measured rows of fig:metro from replay JSONs into one CSV for the
paper side's compare_measured.py. One row per (replay file, arm). Columns carry
the point's coordinates (L, tracks per locale, vehicles per lane-km), the
admission rule and K, the figure load (marginal on top of the admitted pass),
the standalone column, both freshness populations, the displacement face,
batch sizes and provenance (cost file, trace hash). Usage: metro_rows.py <out.csv> <replay.json>..."""
import csv, json, sys
COLS = ["file", "trace", "trace_sha", "net", "tile_m", "L", "tracks", "cycles", "tracks_per_locale_p50", "n_owned_p50", "n_owned_p95",
        "admit_rule", "admit_budget_ms", "K_admit", "K_deployed_cycle", "admission_ratio_p50", "admission_ratio_p5", "charge", "pad_isolated", "cap",
        "nb_m", "nbr_mode", "degree_used", "degree_topological", "cost_source", "cost_file", "arm",
        "p95_load_frac", "p95_load_frac_standalone", "max_locale_cycle_frac", "mean_load_frac",
        "within_limit_postcross_frac", "within_limit_postcross_w5", "within_limit_postcross_w15", "within_limit_frac_pooled",
        "displaced_owned_frac_mean", "displaced_owned_frac_p95", "batch_owned_plus_added_p50", "batch_owned_plus_added_p95",
        "imports", "deltas", "predict_replicas", "bytes_per_s_per_locale", "postcross_consumptions", "consumptions"]
def main():
    out, files = sys.argv[1], sys.argv[2:]
    rows = []
    for f in files:
        j = json.load(open(f)); o = j.get("n_owned_per_locale_cycle", {})
        base = {"file": f.split("/")[-1], "trace": j.get("trace"), "trace_sha": j.get("trace_sha"), "net": j.get("net"), "tile_m": j.get("tile_m"), "L": j.get("L"),
                "tracks": j.get("tracks"), "cycles": j.get("cycles"), "tracks_per_locale_p50": round(o.get("p50", 0) , 1), "n_owned_p50": o.get("p50"), "n_owned_p95": o.get("p95"),
                "admit_rule": o.get("admit_rule", "none"), "admit_budget_ms": o.get("admit_budget_ms"), "K_admit": o.get("K_admit"), "K_deployed_cycle": o.get("K_deployed_cycle"),
                "admission_ratio_p50": o.get("admission_ratio_p50"), "admission_ratio_p5": o.get("admission_ratio_p95_low"), "charge": j["arms"][next(iter(j["arms"]))].get("charge"),
                "pad_isolated": j.get("pad_isolated"), "cap": j.get("cap"), "nb_m": j.get("nb_m"), "nbr_mode": j.get("nbr_mode"), "degree_used": j.get("degree_used"), "degree_topological": j.get("degree_topological"),
                "cost_source": j.get("cost_source"), "cost_file": (j.get("cost_meta") or {}).get("file")}
        for arm, r in j["arms"].items():
            row = dict(base); row["arm"] = arm
            for c in COLS:
                if c in r and c not in row: row[c] = r[c]
            row["within_limit_frac_pooled"] = r.get("within_limit_frac")
            rows.append(row)
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS, extrasaction="ignore"); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{len(rows)} rows -> {out}")
    for r in rows: print(f"  {r['file']:44s} {r['admit_rule']:8s} K={r['K_admit']} {r['arm']:12s} load {100*(r['p95_load_frac'] or 0):6.1f}%  window {100*(r['within_limit_postcross_frac'] or 0):5.1f}%  displaced {r.get('displaced_owned_frac_mean')}")
if __name__ == "__main__": main()
