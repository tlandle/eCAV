#!/usr/bin/env python3
"""Rows of the supplementary budget table (tab:budget) at the 0.8, 0.9 and 0.95 thresholds from three
fig_situation.py tau CSVs (one per --threshold run): the exact-state reference on the two junction
geometries at the six approach speeds, with the bootstrap interval under the 0.8 row, and the blind
overtake line. Writes a LaTeX fragment to paste between \\midrule lines of floats/tbl-budget.tex.
    python scripts/sensys_analysis/tbl_budget_thresholds.py --csv 0.8=<tau.csv> 0.9=<tau.csv> 0.95=<tau.csv> --out <fragment.tex>
"""
import argparse, csv

GEOMS = [("openscenario_3_edge_late_fusion", "Occluded left turn"), ("scp_edge_late_fusion", "Straight crossing")]
SPEEDS = [38, 50, 53, 63, 70, 88]


def load(path):
    rows = {}
    for r in csv.DictReader(open(path)):
        if r["arm"] not in ("reference", "oracle"):
            continue
        rows[(r["geom"], int(r["speed_kmh"]))] = r
    return rows


def fmt(v):
    v = float(v)
    return f"{int(round(v)):,}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs=3, required=True, help="threshold=path, three entries")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    tabs = {}
    for item in a.csv:
        th, path = item.split("=", 1)
        tabs[th] = load(path)
    ths = sorted(tabs, key=float)
    out = []
    for geom, name in GEOMS:
        for th in ths:
            vals = [tabs[th][(geom, s)] for s in SPEEDS]
            cells = " & ".join(fmt(v["tau_ms"]) + ("$^{+}$" if v["censored"] == "1" else "") for v in vals)
            label = name if th == ths[0] else ""
            out.append(f"{label} & {th} & {cells} \\\\")
            if th == ths[0]:
                ivs = " & ".join("{\\footnotesize[%s, %s]}" % (fmt(v["boot_lo_ms"]), fmt(v["boot_hi_ms"])) for v in vals)
                out.append(f" & & {ivs} \\\\")
        out.append("\\midrule")
    bo = []
    for th in ths:
        v = tabs[th].get(("blind_overtake_edge_late_fusion", 43))
        if v:
            bo.append(f"{fmt(v['tau_ms'])} at {th}" + (" {\\footnotesize[%s, %s]}" % (fmt(v["boot_lo_ms"]), fmt(v["boot_hi_ms"])) if th == ths[0] else ""))
    out.append("\\multicolumn{8}{l}{Blind overtake at 43\\,km/h, oncoming 12\\,m/s: " + ", ".join(bo) + "} \\\\")
    open(a.out, "w").write("\n".join(out) + "\n")
    print("\n".join(out))


if __name__ == "__main__":
    main()
