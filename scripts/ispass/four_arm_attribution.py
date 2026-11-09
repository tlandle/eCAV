#!/usr/bin/env python3
"""Experiment D (ISPASS): timing channel against information channel from the
SenSys evaluation export, no new runs. Four arms per junction at 70 km/h, N=1:
    exact state, 0 ms          (Arm A: ground-truth state, zero added delay)
    exact state, delayed       (C-lat: ground-truth content at each configured delay)
    no cooperation             (Arm B: onboard sensing only; encounter_validation.csv)
    cooperative sensor stack   (Arm C: V2X2V prediction with SBA at each configured delay)

    python scripts/ispass/four_arm_attribution.py --data ~/Downloads/eval_run_v12/eval_run_v12 [--out paper_ispass/four_arm_attribution.csv]

Columns: geom, arm, lat_ms, runs, success, collided, self_ghost, mean_p99_age_ms,
median_decision_age_ms. The no-cooperation arm has success = 1 - no_coop_collision over
the validation runs and no age (nothing is consumed from the edge).
"""
import argparse
import csv
import json
import os
import statistics as st


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan")
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * ((p * (1 - p) / n + z * z / (4 * n * n)) ** 0.5) / d
    return c - h, c + h


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--data", default=os.path.expanduser("~/Downloads/eval_run_v12/eval_run_v12"))
    ap.add_argument("--out", default="paper_ispass/four_arm_attribution.csv")
    ap.add_argument("--speed", default="70")
    a = ap.parse_args(argv)
    rows = []
    with open(os.path.join(a.data, "runs.csv")) as fh:
        for r in csv.DictReader(fh):
            if r["speed"] != a.speed or r["N"] != "1" or r["geom"] not in ("openscenario_3_edge_late_fusion", "scp_edge_late_fusion"):
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if kv.get("workload") or kv.get("fault_type") or kv.get("controller") or kv.get("lead_s") or kv.get("motion") != "constant_velocity":
                continue  # main envelope cells only
            if (r["mgr"], r["anch"]) in (("oracle", "on"), ("late_fusion", "on")):
                rows.append(r)
    out = []
    geoms = sorted({r["geom"] for r in rows})
    for g in geoms:
        for mgr, anch, name in (("oracle", "on", "exact state"), ("late_fusion", "on", "cooperative sensor stack")):
            lats = sorted({int(r["lat"]) for r in rows if r["geom"] == g and r["mgr"] == mgr and r["anch"] == anch})
            for lat in lats:
                rs = [r for r in rows if r["geom"] == g and r["mgr"] == mgr and r["anch"] == anch and int(r["lat"]) == lat]
                n = len(rs)
                k = sum(int(r["s_op"]) for r in rs)
                lo, hi = wilson(k, n)
                ages = [float(r["axis_ms"]) for r in rs if r["axis_ms"]]
                dec = [float(r["decision_age_ms"]) for r in rs if r.get("decision_age_ms") not in ("", None)]
                arm = name if not (mgr == "oracle" and lat > 0) else "exact state, delayed"
                if mgr == "oracle" and lat == 0:
                    arm = "exact state, 0 ms"
                out.append(dict(geom=g, arm=arm, lat_ms=lat, runs=n, success=round(k / n, 3), ci_lo=round(lo, 3), ci_hi=round(hi, 3),
                                collided=sum(int(r["collided"]) for r in rs), self_ghost=sum(int(r["self_ghost"]) for r in rs),
                                mean_p99_age_ms=round(st.mean(ages), 1) if ages else "",
                                median_decision_age_ms=round(st.median(dec), 1) if dec else ""))
        # no cooperation from the validation table
        ev = [r for r in csv.DictReader(open(os.path.join(a.data, "encounter_validation.csv"))) if r["geom"] == g and r["speed"] == a.speed]
        if ev:
            n = len(ev); coll = sum(int(float(r["no_coop_collision"])) for r in ev)
            lo, hi = wilson(n - coll, n)
            out.append(dict(geom=g, arm="no cooperation", lat_ms="", runs=n, success=round((n - coll) / n, 3), ci_lo=round(lo, 3), ci_hi=round(hi, 3),
                            collided=coll, self_ghost=0, mean_p99_age_ms="", median_decision_age_ms=""))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)
    for r in out:
        if r["arm"] in ("exact state, 0 ms", "no cooperation") or r["lat_ms"] in (0, 200, 400):
            print(f"{r['geom'][:14]:14s} {r['arm']:26s} lat {str(r['lat_ms']):>4s} n {r['runs']:3d} success {r['success']:.2f} "
                  f"[{r['ci_lo']:.2f},{r['ci_hi']:.2f}] coll {r['collided']:3d} ghost {r['self_ghost']:3d} p99 {r['mean_p99_age_ms']} dec {r['median_decision_age_ms']}")
    print("wrote", a.out, len(out), "rows")


if __name__ == "__main__":
    main()
