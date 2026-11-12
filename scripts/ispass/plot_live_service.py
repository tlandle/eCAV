#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Summary table and three-panel figure for the live-service timing validation.

  python scripts/ispass/plot_live_service.py <stamp> [--workload full]

Reads experiment_results/ispass_timing_live/<stamp>/results.json, writes
summary.json, summary.csv and fig_live_service.pdf next to it (column width,
7 pt text): (a) N against p95 age at use, live service and barrier model at
each step; (b) N against delivered update rate; (c) step size against the
mean absolute error of the p95 age and against wall-clock cost per simulated
second.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
KEYS = ["age_mean", "age_p50", "age_p95", "age_p99", "update_hz", "superseded",
        "svc_mean", "svc_p95", "svc_p99"]


def agg(rows, key):
    v = [r[key] for r in rows if r.get(key) == r.get(key)]
    return (float(np.mean(v)), float(np.min(v)), float(np.max(v))) if v else (float("nan"),) * 3


def saturation(ns, rates, frame_hz=20.0):
    """Smallest N whose delivered update rate is below 95 % of the frame rate."""
    for n, r in zip(ns, rates):
        if r < 0.95 * frame_hz:
            return n
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stamp")
    ap.add_argument("--workload", default="full")
    a = ap.parse_args()
    root = REPO / "experiment_results" / "ispass_timing_live" / a.stamp
    res = json.load(open(root / "results.json"))
    cells = defaultdict(lambda: defaultdict(list))      # (workload, n) -> arm -> rows
    for r in res:
        k = (r["workload"], r["n"])
        cells[k]["live"].append(r["live"])
        for dt, b in r["barrier"].items():
            cells[k]["barrier" + dt].append(b)
        if "develop_no_occupancy" in r:
            cells[k]["develop"].append(r["develop_no_occupancy"])
    summary = []
    for (wl, n), arms in sorted(cells.items()):
        row = dict(workload=wl, n=n)
        for arm, rows in arms.items():
            for key in KEYS + (["wall_per_sim_s"] if arm != "live" else
                               ["ul_transit_p50", "dl_transit_p50", "frames_complete_frac"]):
                m, lo, hi = agg(rows, key)
                row[f"{arm}.{key}"] = m
                if key in ("age_p95", "update_hz"):
                    row[f"{arm}.{key}.min"], row[f"{arm}.{key}.max"] = lo, hi
            row[f"{arm}.reps"] = len(rows)
        for arm in arms:
            if arm == "live":
                continue
            for key in ("age_mean", "age_p50", "age_p95", "age_p99", "update_hz"):
                ref = row[f"live.{key}"]
                row[f"{arm}.err.{key}"] = row[f"{arm}.{key}"] - ref
                row[f"{arm}.relerr.{key}"] = (row[f"{arm}.{key}"] - ref) / ref if ref else float("nan")
        summary.append(row)
    sat = {}
    for wl in sorted({r["workload"] for r in summary}):
        rs = [r for r in summary if r["workload"] == wl]
        ns = [r["n"] for r in rs]
        sat[wl] = {arm: saturation(ns, [r[f"{arm}.update_hz"] for r in rs])
                   for arm in ("live", "barrier50", "barrier25", "develop") if f"{arm}.update_hz" in rs[0]}
    json.dump(dict(rows=summary, saturation_n=sat), open(root / "summary.json", "w"), indent=1)
    cols = sorted({k for r in summary for k in r})
    with open(root / "summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(summary)

    def fmt(x, d=0):
        return "nan" if x != x else f"{x:.{d}f}"
    for wl in sorted(sat):
        print(f"\n{wl}: saturation N (first N with rate < 19 Hz): {sat[wl]}")
        print("  N | live svc  p50  p95  p99  rate sup | b50 p50  p95  p99 rate | b25 p50  p95  p99 rate | dev p95 rate")
        for r in [r for r in summary if r["workload"] == wl]:
            g = lambda k, d=0: fmt(r.get(k, float("nan")), d)
            print(f" {r['n']:2d} | {g('live.svc_mean'):>4} {g('live.age_p50'):>5} {g('live.age_p95'):>4} {g('live.age_p99'):>4} "
                  f"{g('live.update_hz',1):>5} {g('live.superseded',2)} | {g('barrier50.age_p50'):>5} {g('barrier50.age_p95'):>4} "
                  f"{g('barrier50.age_p99'):>4} {g('barrier50.update_hz',1):>4} | {g('barrier25.age_p50'):>5} "
                  f"{g('barrier25.age_p95'):>4} {g('barrier25.age_p99'):>4} {g('barrier25.update_hz',1):>4} | "
                  f"{g('develop.age_p95'):>5} {g('develop.update_hz',1):>4}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "legend.fontsize": 7, "pdf.fonttype": 42, "axes.linewidth": 0.6,
                         "lines.linewidth": 1.0, "lines.markersize": 3})
    rs = [r for r in summary if r["workload"] == a.workload]
    ns = [r["n"] for r in rs]
    fig, ax = plt.subplots(1, 3, figsize=(7.0, 1.75), constrained_layout=True)
    styles = [("live", "live service", "k", "o", "-"), ("barrier50", "barrier, 50 ms", "C0", "s", "--"),
              ("barrier25", "barrier, 25 ms", "C1", "^", ":")]
    for arm, lab, c, mk, ls in styles:
        if f"{arm}.age_p95" not in rs[0]:
            continue
        ax[0].plot(ns, [r[f"{arm}.age_p95"] for r in rs], color=c, marker=mk, ls=ls, label=lab)
        ax[1].plot(ns, [r[f"{arm}.update_hz"] for r in rs], color=c, marker=mk, ls=ls, label=lab)
    ax[0].set_xlabel("contributors N"); ax[0].set_ylabel("p95 age at use (ms)")
    ax[1].set_xlabel("contributors N"); ax[1].set_ylabel("update rate (Hz)")
    ax[1].axhline(20, color="0.6", lw=0.5)
    ax[0].legend(frameon=False, handlelength=2.2)
    steps = [s for s in (50, 25) if f"barrier{s}.age_p95" in rs[0]]
    err = [float(np.mean([abs(r[f"barrier{s}.err.age_p95"]) for r in rs])) for s in steps]
    cost = [float(np.mean([r[f"barrier{s}.wall_per_sim_s"] for r in rs])) for s in steps]
    ax[2].bar([str(s) for s in steps], err, width=0.45, color="C0")
    ax[2].set_xlabel("barrier step (ms)"); ax[2].set_ylabel("mean |p95 age error| (ms)")
    ax2 = ax[2].twinx()
    ax2.plot([str(s) for s in steps], cost, color="C3", marker="o")
    ax2.set_ylabel("wall s per simulated s", color="C3")
    for x, t in zip(ax, "abc"):
        x.set_title(f"({t})", loc="left", fontsize=7, pad=2)
    out = root / "fig_live_service.pdf"
    fig.savefig(out)
    print("\nfigure", out)


if __name__ == "__main__":
    main()
