#!/usr/bin/env python3
"""Diff the deterministic-partition pNEUMA surveys (*_survey_det.json) against
the surveys reported on 2026-09-12 (*_survey.json): the statistics quoted to
the paper side (crossings per track-second at 5 Hz, near-boundary fraction at
20 m, observed-boundary degree, active tracks, vehicles per lane-km, L,
boundaries). Usage: survey_diff.py <metro_dir>"""
import glob, json, os, sys
D = sys.argv[1]
def pick(j):
    t = j["levels"]["tile300"]; tr = t["tracked"]
    return {"L": t["L"], "boundaries": t["boundaries"], "degree_mean": t.get("neighbour_degree_mean"), "degree_observed": t.get("neighbour_degree_observed_mean", t.get("observed_degree_mean")),
            "crossing_rate_5hz": tr["crossing_rate_per_track_s_5hz"], "near20": tr["near_boundary_frac"]["20"], "boundaries_observed_frac": tr.get("boundaries_observed_frac"),
            "active_p50": (j.get("active_tracks_per_cycle") or {}).get("p50") if isinstance(j.get("active_tracks_per_cycle"), dict) else j.get("active_tracks_per_cycle"),
            "veh_per_lane_km": j.get("vehicles_per_lane_km_median")}
for f in sorted(glob.glob(f"{D}/pneuma_20181024_*_survey_det.json")):
    g = f.replace("_survey_det.json", "_survey.json")
    if not os.path.exists(g): print("no reference for", f); continue
    a, b = pick(json.load(open(g))), pick(json.load(open(f))); slot = os.path.basename(f).split("_survey")[0]
    diffs = []
    for k in a:
        va, vb = a[k], b[k]
        if va is None or vb is None: continue
        rel = abs(va - vb) / max(abs(va), 1e-12)
        diffs.append(f"{k} {va:.5g}->{vb:.5g}" + (f" ({100*rel:.2f}%)" if rel > 1e-9 else " (=)"))
    print(slot + ": " + "; ".join(diffs))
