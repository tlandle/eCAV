#!/usr/bin/env python3
"""ISPASS: exact-state against the cooperative sensor stack at matched measured age.

The two arms share the configured delay cells, but the cooperative arm's measured p99 age
at use is about 27 ms higher at each cell. This script reads four_arm_attribution.csv,
interpolates the exact-state success curve (linear between cells) at each cooperative
cell's measured age, and reports the difference at matched age.

    python scripts/ispass/matched_age_attribution.py [--csv paper_ispass/four_arm_attribution.csv]
"""
import argparse
import csv

import numpy as np


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="paper_ispass/four_arm_attribution.csv")
    a = ap.parse_args(argv)
    rows = list(csv.DictReader(open(a.csv)))
    for g in sorted({r["geom"] for r in rows}):
        ex = sorted((float(r["mean_p99_age_ms"]), float(r["success"])) for r in rows
                    if r["geom"] == g and r["arm"].startswith("exact state"))
        co = sorted((float(r["mean_p99_age_ms"]), float(r["success"]), int(r["lat_ms"]), int(r["self_ghost"]), int(r["runs"]))
                    for r in rows if r["geom"] == g and r["arm"] == "cooperative sensor stack")
        xa, ya = np.array([e[0] for e in ex]), np.array([e[1] for e in ex])
        print(g)
        print("  lat  coop_age  coop  exact@cell  exact@matched  same-cell diff  matched diff  ghost")
        same, matched = [], []
        for age, s, lat, gh, n in co:
            cell = [r for r in rows if r["geom"] == g and r["arm"].startswith("exact state") and int(r["lat_ms"]) == lat]
            e_cell = float(cell[0]["success"]) if cell else float("nan")
            e_m = float(np.interp(age, xa, ya))
            if age <= 600:
                print(f"  {lat:4d}  {age:7.1f}  {s:.2f}  {e_cell:.2f}        {e_m:.3f}          {s - e_cell:+.2f}           {s - e_m:+.3f}      {gh}")
            if max(e_m, s) > 0.02:
                same.append(s - e_cell); matched.append(s - e_m)
        print(f"  cells with either arm above 0.02: {len(matched)}; mean same-cell diff {np.mean(same):+.3f}, mean matched-age diff {np.mean(matched):+.3f}, "
              f"matched range [{min(matched):+.3f}, {max(matched):+.3f}]")


if __name__ == "__main__":
    main()
