#!/usr/bin/env python3
"""Kinematic design of the highway merge (cut-in) encounter, the table the
evaluation export ships as highway_merge_design.csv, from the same formulas the
CARLA scenario (scenarios/scenario_highway_merge.py) applies at run time.

    python scripts/sensys_analysis/highway_merge_design.py [--check <export>/highway_merge_design.csv] [--out table.csv]

    initial_gap        = clearance + ttc * closing              bumper gap at the cut-in
    rel_stop_distance  = closing^2 / (2 * ego_braking)
    kinematic_budget   = ttc - non_age_response - closing / (2 * ego_braking)
"""
import argparse
import csv
import sys

EGO_SPEED = 27.0
TTC = 1.9
CLEARANCE = 3.0
RESPONSE = 0.28
BRAKING = 6.0
CLOSING = [4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0]
COLS = ["closing_speed_mps", "ego_speed_mps", "intruder_speed_mps", "initial_gap_m",
        "initial_ttc_to_clearance_s", "clearance_m", "non_age_response_s", "ego_braking_mps2",
        "relative_stop_distance_m", "kinematic_budget_ms", "positive_kinematic_budget"]


def row(cs, ego=EGO_SPEED, ttc=TTC, clearance=CLEARANCE, response=RESPONSE, braking=BRAKING):
    gap = clearance + ttc * cs
    rel_stop = cs * cs / (2.0 * braking)
    budget_ms = 1000.0 * (ttc - response - cs / (2.0 * braking))
    return dict(closing_speed_mps=cs, ego_speed_mps=ego, intruder_speed_mps=ego - cs,
                initial_gap_m=round(gap, 4), initial_ttc_to_clearance_s=ttc, clearance_m=clearance,
                non_age_response_s=response, ego_braking_mps2=braking,
                relative_stop_distance_m=round(rel_stop, 4), kinematic_budget_ms=round(budget_ms, 3),
                positive_kinematic_budget=int(budget_ms > 0))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", help="export CSV to compare against")
    ap.add_argument("--out", help="write the table here")
    a = ap.parse_args(argv)
    rows = [row(cs) for cs in CLOSING]
    for r in rows:
        print("  ".join(f"{k}={r[k]}" for k in ("closing_speed_mps", "intruder_speed_mps", "initial_gap_m",
                                                  "relative_stop_distance_m", "kinematic_budget_ms")))
    if a.out:
        with open(a.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=COLS); w.writeheader(); w.writerows(rows)
    if a.check:
        ref = list(csv.DictReader(open(a.check)))
        bad = 0
        for r, e in zip(rows, ref):
            for k in COLS:
                if abs(float(r[k]) - float(e[k])) > 1e-3:
                    bad += 1; print(f"MISMATCH cs={r['closing_speed_mps']} {k}: ours {r[k]} export {e[k]}")
        print(f"check against {a.check}: {len(ref)} rows, {bad} mismatching values")
        return 1 if bad or len(ref) != len(rows) else 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
