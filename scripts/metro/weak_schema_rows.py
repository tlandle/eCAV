#!/usr/bin/env python3
"""Weak-scaling rows in the metro schema (+ L, window, series, interior p95,
tracks_per_locale, degree): the REFERENCE series from the chooser CSVs
(ref_t<T>_<row>_windows.csv, density_matched windows of the full-network run)
and the CROP series from the crop replay JSONs (crop_t<T>_k<k>_<row>.json).
Usage: weak_schema_rows.py <weak_dir> <out.csv>"""
import csv, glob, json, os, re, sys
W, OUT = sys.argv[1], sys.argv[2]
by = {"none": "demand", "single": "admit92", "deployed": "deployed"}
rows = []
for f in sorted(glob.glob(f"{W}/ref_t*_*_windows.csv")):
    m = re.search(r"ref_t(\d+)_(\w+)_windows\.csv", f); t, rule = int(m.group(1)), m.group(2)
    for r in csv.DictReader(open(f)):
        if r["window_kind"] != "density_matched": continue
        rows.append({"series": "reference", "tracks": t, "admit_rule": rule, "row": by[rule], "k": int(r["k"]), "locales": int(r["L"]), "locales_interior": int(r["L_interior"]), "window": r["window"],
                     "policy": r["policy"], "dest_load_p95": float(r["dest_load_p95"]), "p95_load_frac_interior": (float(r["dest_load_p95_interior"]) if r["dest_load_p95_interior"] else ""),
                     "tracks_per_locale": float(r["tracks_per_locale"]), "network_tracks_per_locale": float(r["network_tracks_per_locale"]), "density_dev": float(r["density_dev"]), "within_tol": int(r["within_tol"]),
                     "degree_observed": float(r["degree_used"]), "network_degree": float(r["network_degree"]), "within_limit": "", "within_limit_mechanism": "", "reference_interior_same_window": "",
                     "notes": "reference: pooled p95 over a density-matched window of the FULL-network replay (only sampling changes with L)"})
for f in sorted(glob.glob(f"{W}/crop_t*_k*_*.json")):
    m = re.search(r"crop_t(\d+)_k(\d+)_(\w+)\.json", f); t, k, rule = int(m.group(1)), int(m.group(2)), m.group(3); j = json.load(open(f)); c = j.get("crop") or {}
    for arm, r in j["arms"].items():
        rows.append({"series": "crop", "tracks": t, "admit_rule": rule, "row": by[rule], "k": k, "locales": j["L"], "locales_interior": c.get("interior_locales", ""), "window": ",".join(str(v) for v in c.get("tile_window", [])),
                     "policy": arm, "dest_load_p95": round(r["p95_load_frac"], 4), "p95_load_frac_interior": (round(r["p95_load_frac_interior"], 4) if r.get("p95_load_frac_interior") is not None else ""),
                     "tracks_per_locale": round(j["tracks_per_locale"], 2), "network_tracks_per_locale": "", "density_dev": "", "within_tol": "", "degree_observed": round(j["degree_used"], 2), "network_degree": "",
                     "within_limit": round(r["within_limit_postcross_coupled"], 4), "within_limit_mechanism": round(r["within_limit_postcross_frac"], 4), "reference_interior_same_window": "",
                     "notes": "crop: the deployment ends at the window; boundaries to the outside are dropped (rim effect real for a small deployment; degree stated)"})
# crop-interior vs reference-interior at the same window (the check that the rim is the only difference)
ref = {(r["tracks"], r["row"], r["window"], r["policy"]): r["p95_load_frac_interior"] for r in rows if r["series"] == "reference"}
for r in rows:
    if r["series"] == "crop": r["reference_interior_same_window"] = ref.get((r["tracks"], r["row"], r["window"], r["policy"]), "")
rows.sort(key=lambda r: (r["series"], r["tracks"], r["row"], r["k"], r["policy"]))
with open(OUT, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); [w.writerow(r) for r in rows]
print(f"{len(rows)} rows -> {OUT}")
for r in rows:
    if r["policy"] in ("warm", "replall") and r["row"] == "deployed": print(f"  {r['series']:9s} t{r['tracks']:<5d} k{r['k']:<2d} L{r['locales']:<3d} d {r['tracks_per_locale']:6.2f} deg {r['degree_observed']:.2f} {r['policy']:8s} p95 {r['dest_load_p95']:.3f} interior {r['p95_load_frac_interior']} ref-interior {r['reference_interior_same_window']}")
