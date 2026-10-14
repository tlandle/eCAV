#!/usr/bin/env python3
"""The envelope on the axis the paper defines: success against measured age at use.

    python scripts/sensys_analysis/envelope_measured.py <stamp> [--root experiment_results]
        [--bin 50] [--threshold 0.8] [--out safety_envelope_paper/analysis/<stamp>]

Every run's p99 of age at use is read from its per_tick_age.csv (edge-source
rows consumed by the focal vehicle), so the x axis is what the planner saw, not
the configured delay. Runs are binned by that p99 (50 ms bins by default) per
(geometry, speed, manager, anchoring, N, knobs); each bin reports P(S_op) with a
Wilson 95% interval. tau_max is the largest bin centre at which the running
minimum of P(S_op) over bins is still at or above the threshold (the running
minimum keeps a single high bin past the cliff from counting).

Writes <out>/envelope_measured.csv (one row per bin), <out>/tau_max.md, and one
PNG per geometry with a curve per arm.
"""
import argparse
import collections
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import load_runs, wilson, ROOT  # noqa: E402

ARM_LABEL = {
    ("oracle", "on"): "Oracle",
    ("late_fusion", "off"): "V2X2V naive",
    ("late_fusion", "on"): "V2X2V with contract (SBA)",
    ("late_fusion_etsi_self_announcement", "off"): "V2X2V ETSI self-announcement",
    ("vips", "off"): "V2X2V VIPS",
    ("vips", "on"): "V2X2V VIPS with contract",
    ("infra_only", "off"): "I2V object sharing",
    ("infra_only", "on"): "I2V with receiver contract",
    ("cip", "on"): "CIP planning",
}


def label(mgr, anch, kv):
    base = ARM_LABEL.get((mgr, anch), f"{mgr}/{anch}")
    if kv:
        base += " + " + kv.replace('"', "").strip("{}")
    return base


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamp")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--bin", type=float, default=50.0)
    ap.add_argument("--threshold", type=float, default=0.8)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "safety_envelope_paper", "analysis", a.stamp)
    os.makedirs(out, exist_ok=True)

    runs = [r for r in load_runs(a.root, a.stamp) if not r.get("bad")]
    with_age = [r for r in runs if r["age_p99"] == r["age_p99"]]
    print(f"{len(runs)} runs, {len(with_age)} with a per-tick age log")
    if not with_age:
        return 2

    bins = collections.defaultdict(list)
    for r in with_age:
        b = int(r["age_p99"] // a.bin) * a.bin + a.bin / 2
        bins[(r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"], b)].append(r["s_op"])

    rows = []
    for k in sorted(bins):
        v = bins[k]
        p, lo, hi = wilson(sum(v), len(v))
        rows.append(dict(geom=k[0], speed=k[1], mgr=k[2], anch=k[3], N=k[4], kv=k[5], age_p99_bin_ms=k[6],
                         n=len(v), p_sop=round(p, 3), lo=round(lo, 3), hi=round(hi, 3)))
    with open(os.path.join(out, "envelope_measured.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # tau_max per arm and situation on the measured axis
    taus = []
    arms = sorted(set((r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) for r in rows))
    for arm in arms:
        pts = sorted((r["age_p99_bin_ms"], r["p_sop"], r["n"]) for r in rows if (r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) == arm)
        running = 1.0
        tau = None
        for x, p, n in pts:
            running = min(running, p)
            if running >= a.threshold:
                tau = x
            else:
                break
        taus.append((arm, tau, len(pts), sum(n for _, _, n in pts)))
    with open(os.path.join(out, "tau_max.md"), "w") as fh:
        fh.write(f"# tau_max at P(S_op) >= {a.threshold} on measured p99 age at use ({a.stamp})\n\n")
        fh.write("| geometry | km/h | arm | N | bins | runs | tau_max (bin centre, ms) |\n|---|---|---|---|---|---|---|\n")
        for (geom, spd, mgr, anch, N, kv), tau, nb, nr in taus:
            fh.write(f"| {geom} | {spd} | {label(mgr, anch, kv)} | {N} | {nb} | {nr} | {tau if tau is not None else 'below threshold in the first bin'} |\n")
    print(open(os.path.join(out, "tau_max.md")).read())

    # figures
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib missing, no figures")
        return 0
    for geom in sorted(set(r["geom"] for r in rows)):
        fig, ax = plt.subplots(figsize=(6, 3.6))
        for arm in arms:
            if arm[0] != geom:
                continue
            pts = sorted((r["age_p99_bin_ms"], r["p_sop"], r["lo"], r["hi"]) for r in rows if (r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) == arm)
            xs = [p[0] for p in pts]
            ax.errorbar(xs, [p[1] for p in pts], yerr=[[p[1] - p[2] for p in pts], [p[3] - p[1] for p in pts]],
                        marker="o", capsize=2, label=f"{label(arm[2], arm[3], arm[5])} {arm[1]} km/h N={arm[4]}")
        ax.axhline(a.threshold, color="gray", lw=0.8, ls="--")
        ax.set_xlabel("measured p99 age at use (ms)")
        ax.set_ylabel("P(S_op)")
        ax.set_ylim(-0.02, 1.02)
        ax.set_title(geom)
        ax.legend(fontsize=6)
        fig.tight_layout()
        fig.savefig(os.path.join(out, f"envelope_{geom}.png"), dpi=160)
        plt.close(fig)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
