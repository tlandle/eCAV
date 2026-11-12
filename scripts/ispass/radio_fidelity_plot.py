#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Reduce and plot ISPASS experiment 4 (radio_fidelity_bench.py rows).

  python scripts/ispass/radio_fidelity_plot.py <stamp> [--json paper_ispass/radio_fidelity_summary.json]
                                                       [--fig paper_ispass/fig_radio_fidelity.pdf]

Summary: one row per (model, N, payload), means over seeds, plus table-minus-live
differences. Figure, column width (3.4 in), 7 pt text, four small axes:
  (a) downlink p95 delay and delivery ratio against N, live and table, per payload
  (b) wall-clock cost of the live model and the table's error in p95 planner age
"""
import argparse
import json
import os
from collections import defaultdict

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
KEYS = ["ul_p50", "ul_p95", "ul_p99", "ul_delivery", "ul_loss_lag1", "ul_loss_run_mean", "ul_loss_run_max",
        "ul_delay_lag1", "dl_p50", "dl_p95", "dl_p99", "dl_delivery", "dl_loss_lag1", "dl_loss_run_mean",
        "dl_loss_run_max", "dl_delay_lag1", "age_mean", "age_p50", "age_p95", "age_p99", "update_gap_p95",
        "update_gap_max", "coverage", "wall_ms_per_tick", "wall_s_per_sim_s", "startup_s"]
COL = {300: "#2a78d6", 1000: "#eb6834", 16896: "#1baf7a"}      # validated categorical slots 1 to 3
LAB = {300: "300 B", 1000: "1 KB", 16896: "16.9 KB"}


def reduce(rows):
    g = defaultdict(list)
    for r in rows:
        g[(r["model"], r["n"], r["payload"])].append(r)
    out = []
    for (m, n, p), rs in sorted(g.items()):
        o = {"model": m, "n": n, "payload": p, "seeds": len(rs)}
        for k in KEYS:
            v = [r[k] for r in rs if k in r and r[k] is not None and not (isinstance(r[k], float) and np.isnan(r[k]))]
            o[k] = float(np.mean(v)) if v else None
        out.append(o)
    idx = {(o["model"], o["n"], o["payload"]): o for o in out}
    diffs = []
    for (m, n, p), o in idx.items():
        if m != "table" or ("live", n, p) not in idx:
            continue
        lv = idx[("live", n, p)]
        d = {"n": n, "payload": p}
        for k in ("ul_p50", "ul_p95", "dl_p50", "dl_p95", "dl_p99", "ul_delivery", "dl_delivery",
                  "age_mean", "age_p50", "age_p95", "age_p99", "coverage", "dl_loss_run_mean", "update_gap_p95"):
            if o.get(k) is not None and lv.get(k) is not None:
                d["table_minus_live." + k] = o[k] - lv[k]
        diffs.append(d)
    return out, sorted(diffs, key=lambda d: (d["payload"], d["n"]))


def table_md(summary):
    L = ["| N | payload | model | UL p50/p95 (ms) | UL del. | DL p50/p95/p99 (ms) | DL del. | DL loss run | age p50/p95 (ms) | coverage | wall s / sim s |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    f = lambda v, d=1: "n/a" if v is None else f"{v:.{d}f}"
    for o in sorted(summary, key=lambda o: (o["payload"], o["n"], o["model"])):
        L.append(f"| {o['n']} | {o['payload']} | {o['model']} | {f(o['ul_p50'])}/{f(o['ul_p95'])} | {f(o['ul_delivery'],2)} | "
                 f"{f(o['dl_p50'])}/{f(o['dl_p95'])}/{f(o['dl_p99'])} | {f(o['dl_delivery'],2)} | {f(o['dl_loss_run_mean'])} | "
                 f"{f(o['age_p50'],0)}/{f(o['age_p95'],0)} | {f(o['coverage'],2)} | {f(o['wall_s_per_sim_s'],2)} |")
    return "\n".join(L)


def figure(summary, diffs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "legend.fontsize": 7, "axes.linewidth": 0.5, "pdf.fonttype": 42,
                         "axes.spines.top": False, "axes.spines.right": False})
    idx = {(o["model"], o["n"], o["payload"]): o for o in summary}
    ns = sorted({o["n"] for o in summary}); ps = sorted({o["payload"] for o in summary})
    fig, ax = plt.subplots(2, 2, figsize=(3.4, 2.7), sharex=True)
    for p in ps:
        for m, ls, mf in (("live", "-", COL[p]), ("table", "--", "white")):
            xs = [n for n in ns if (m, n, p) in idx]
            kw = dict(color=COL[p], ls=ls, lw=1.2, marker="o", ms=3.5, mfc=mf, mec=COL[p], mew=0.9)
            ax[0, 0].plot(xs, [idx[(m, n, p)]["dl_p95"] for n in xs], **kw)
            ax[1, 0].plot(xs, [idx[(m, n, p)]["dl_delivery"] for n in xs], **kw)
        xs = [n for n in ns if ("live", n, p) in idx]
        ax[0, 1].plot(xs, [idx[("live", n, p)]["wall_s_per_sim_s"] for n in xs], color=COL[p], lw=1.2,
                      marker="o", ms=3.5)
        dd = {d["n"]: d for d in diffs if d["payload"] == p}
        xs = [n for n in ns if n in dd and "table_minus_live.age_p95" in dd[n]]
        ax[1, 1].plot(xs, [dd[n]["table_minus_live.age_p95"] for n in xs], color=COL[p], lw=1.2, marker="o", ms=3.5)
    ax[0, 0].set_yscale("log"); ax[0, 0].set_ylabel("DL p95 delay (ms)")
    ax[1, 0].set_ylabel("DL delivery ratio"); ax[1, 0].set_ylim(0, 1.05)
    ax[0, 1].set_yscale("log"); ax[0, 1].set_ylabel("live: wall s per sim. s")
    ax[0, 1].axhline(1.0, color="#8a8a86", lw=0.5, ls=":")
    ax[1, 1].set_ylabel("table - live,\np95 age (ms)"); ax[1, 1].axhline(0.0, color="#8a8a86", lw=0.5, ls=":")
    for a in ax[1]:
        a.set_xlabel("senders N"); a.set_xticks(ns)
    for a in ax.flat:
        a.grid(True, lw=0.3, color="#d9d8d2"); a.tick_params(width=0.5, length=2)
    ax[0, 0].set_title("(a) radio: live and table", fontsize=7, loc="left")
    ax[0, 1].set_title("(b) cost and age error", fontsize=7, loc="left")
    from matplotlib.lines import Line2D
    h = [Line2D([], [], color=COL[p], lw=1.2, marker="o", ms=3.5, label=LAB.get(p, str(p))) for p in ps]
    h += [Line2D([], [], color="#52514e", lw=1.2, ls="-", marker="o", ms=3.5, label="live ns-3"),
          Line2D([], [], color="#52514e", lw=1.2, ls="--", marker="o", ms=3.5, mfc="white", label="table")]
    fig.legend(handles=h, loc="lower center", ncol=5, frameon=False, handlelength=1.8, columnspacing=0.9,
               bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(pad=0.4, rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    print("figure:", path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stamp")
    ap.add_argument("--also", nargs="*", default=[], help="more stamps whose rows are merged (parallel runs)")
    ap.add_argument("--json", default=None)
    ap.add_argument("--fig", default=None)
    a = ap.parse_args()
    d = os.path.join(ROOT, "experiment_results/ispass_radio_fidelity", a.stamp)
    rows = json.load(open(os.path.join(d, "rows.json")))
    for st in a.also:
        rows += json.load(open(os.path.join(ROOT, "experiment_results/ispass_radio_fidelity", st, "rows.json")))
    summary, diffs = reduce(rows)
    js = {"stamp": a.stamp, "config": json.load(open(os.path.join(d, "config.json"))),
          "rows": summary, "table_minus_live": diffs}
    for path in [os.path.join(d, "summary.json")] + ([a.json] if a.json else []):
        json.dump(js, open(path, "w"), indent=1)
    md = table_md(summary)
    open(os.path.join(d, "summary.md"), "w").write(md + "\n")
    print(md)
    figure(summary, diffs, a.fig or os.path.join(d, "fig_radio_fidelity.pdf"))


if __name__ == "__main__":
    main()
