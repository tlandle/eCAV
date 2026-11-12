#!/usr/bin/env python3
"""Experiment A figure for the ISPASS draft: the barrier-stepped two-time-base model
against the event-driven reference model, from paper_ispass/timing_fidelity_table.csv
(scripts/bench_timing_fidelity.py). Drawn at IEEE column width with 7 pt text.

    python scripts/ispass/plot_timing_fidelity.py [--table paper_ispass/timing_fidelity_table.csv] [--out paper_ispass/fig_timing_fidelity.pdf]
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

COL_IN = 3.5
plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7, "legend.fontsize": 6.5,
                     "xtick.labelsize": 6.5, "ytick.labelsize": 6.5, "pdf.fonttype": 42, "ps.fonttype": 42})


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--table", default="paper_ispass/timing_fidelity_table.csv")
    ap.add_argument("--out", default="paper_ispass/fig_timing_fidelity.pdf")
    a = ap.parse_args(argv)
    rows = list(csv.DictReader(open(a.table)))
    N = [int(r["N"]) for r in rows]
    f = lambda k: [float(r[k]) for r in rows]
    fig, ax = plt.subplots(1, 3, figsize=(COL_IN * 2, 1.9))
    ax[0].plot(N, f("cont_age_p50_ms"), "o-", c="k", ms=3, label="reference p50")
    ax[0].plot(N, f("ecav_age_p50_ms"), "s--", c="C0", ms=3, label="barrier model p50")
    ax[0].plot(N, f("cont_age_p95_ms"), "o-", c="0.5", ms=3, label="reference p95")
    ax[0].plot(N, f("ecav_age_p95_ms"), "s--", c="C1", ms=3, label="barrier model p95")
    ax[0].set_ylabel("planner-side age (ms)")
    ax[1].plot(N, f("cont_rate_hz"), "o-", c="k", ms=3, label="reference")
    ax[1].plot(N, f("ecav_rate_hz"), "s--", c="C0", ms=3, label="barrier model")
    ax[1].set_ylabel("service update rate (Hz)")
    ax[2].plot(N, f("cont_superseded_pct"), "o-", c="k", ms=3, label="reference")
    ax[2].plot(N, f("ecav_superseded_pct"), "s--", c="C0", ms=3, label="barrier model")
    ax[2].set_ylabel("superseded frames (%)")
    for x in ax:
        x.set_xlabel("contributors N")
        x.set_xscale("log", base=2)
        x.set_xticks(N)
        x.set_xticklabels([str(n) for n in N])
        x.grid(alpha=0.3, lw=0.4)
        x.legend(frameon=False, handlelength=1.6)
    fig.tight_layout(pad=0.3)
    fig.savefig(a.out)
    print("wrote", a.out, "size in:", fig.get_size_inches())


if __name__ == "__main__":
    main()
