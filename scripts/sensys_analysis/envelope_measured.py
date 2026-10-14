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
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from reduce_per_tick_age import reduce_run  # noqa: E402


def run_axis(r, axis):
    """Per-run age statistic on the chosen axis: critical (p99 over the
    conflict actor's consumed objects, the paper's planner-side tail), global
    (p99 over every consumed object; dominated by the planner's 24-tick hold of
    unrefreshed tracks), or brake (age of the conflict actor at the first
    braking decision)."""
    rd = os.path.dirname(r["path"])
    for p in (os.path.join(rd, "per_tick_age.csv"), os.path.join(rd, "evaluation_output", "per_tick_age.csv")):
        if os.path.exists(p):
            m = reduce_run(p)
            key = {"critical": "critical_object_p99_ms", "global": "global_p99_age_ms", "brake": "conflict_brake_age_ms"}[axis]
            v = m.get(key, float("nan"))
            if v != v and axis == "critical":
                v = m.get("global_p99_age_ms", float("nan"))
            return v
    return float("nan")

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
    ap.add_argument("--axis", choices=("critical", "global", "brake"), default="critical")
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "safety_envelope_paper", "analysis", a.stamp)
    os.makedirs(out, exist_ok=True)

    runs = [r for r in load_runs(a.root, a.stamp) if not r.get("bad")]
    for r in runs:
        r["axis_ms"] = run_axis(r, a.axis)
    with_age = [r for r in runs if r["axis_ms"] == r["axis_ms"]]
    print(f"{len(runs)} runs, {len(with_age)} with a per-tick age log; axis = {a.axis} p99 age at use")
    if not with_age:
        return 2

    bins = collections.defaultdict(list)
    for r in with_age:
        b = int(r["axis_ms"] // a.bin) * a.bin + a.bin / 2
        bins[(r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"], b)].append(r)

    # S_op and its components per bin, so a curve shift (e.g. between control
    # policies) is attributable to collisions, self-ghosts, false brakes, or
    # stalls rather than read off the aggregate alone.
    rows = []
    for k in sorted(bins):
        v = bins[k]
        p, lo, hi = wilson(sum(r["s_op"] for r in v), len(v))
        rows.append(dict(geom=k[0], speed=k[1], mgr=k[2], anch=k[3], N=k[4], kv=k[5], age_p99_bin_ms=k[6],
                         n=len(v), p_sop=round(p, 3), lo=round(lo, 3), hi=round(hi, 3),
                         collided=sum(1 for r in v if r.get("focal_collisions", 0) > 0),
                         self_ghost=sum(1 for r in v if r.get("focal_ghost_episodes", 0) > 0),
                         false_brake=sum(1 for r in v if r.get("focal_fp_episodes", 0) > 0),
                         stalled=sum(1 for r in v if r.get("s_prog", 1) == 0)))
    with open(os.path.join(out, "envelope_measured.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # tau_max per arm and situation on the measured axis, at three thresholds
    # (reviewer C: sensitivity of the useful-age budget to the success level),
    # plus a bootstrap interval of the per-cell measured p99 age (run-level
    # uncertainty of the axis itself).
    import random
    rng = random.Random(0)

    def tau_at(pts, thr):
        running = 1.0
        tau = None
        for x, p, n in pts:
            running = min(running, p)
            if running >= thr:
                tau = x
            else:
                break
        return tau

    thresholds = (0.8, 0.9, 0.95)
    taus = []
    arms = sorted(set((r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) for r in rows))
    for arm in arms:
        pts = sorted((r["age_p99_bin_ms"], r["p_sop"], r["n"]) for r in rows if (r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) == arm)
        axis_vals = [r["axis_ms"] for r in with_age if (r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"]) == arm]
        boots = []
        for _ in range(1000):
            smp = [rng.choice(axis_vals) for _ in axis_vals]
            smp.sort()
            boots.append(smp[len(smp) // 2])
        boots.sort()
        med_lo, med_hi = boots[int(0.025 * len(boots))], boots[int(0.975 * len(boots)) - 1]
        taus.append((arm, [tau_at(pts, t) for t in thresholds], len(pts), sum(n for _, _, n in pts), med_lo, med_hi))
    with open(os.path.join(out, "tau_max.md"), "w") as fh:
        fh.write(f"# tau_max on measured {a.axis} p99 age at use ({a.stamp})\n\n")
        fh.write("tau_max is the largest bin centre at which the running minimum of P(S_op) stays at or above the threshold. "
                 "The last column is the 95% bootstrap interval of the median per-run p99 age across the arm's runs.\n\n")
        fh.write("| geometry | km/h | arm | N | bins | runs | tau 0.8 | tau 0.9 | tau 0.95 | median p99 age CI (ms) |\n|---|---|---|---|---|---|---|---|---|---|\n")
        for (geom, spd, mgr, anch, N, kv), tl, nb, nr, lo, hi in taus:
            cells = " | ".join(str(t) if t is not None else "< first bin" for t in tl)
            fh.write(f"| {geom} | {spd} | {label(mgr, anch, kv)} | {N} | {nb} | {nr} | {cells} | [{lo:.0f}, {hi:.0f}] |\n")
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
        ax.set_xlabel(f"measured {a.axis} p99 age at use (ms)")
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
