#!/usr/bin/env python3
"""Envelope figure (arch_safety_scale): P(S_op) against the conflict actor's measured p99 age at use,
one panel per junction geometry at one approach speed, one curve per arm, Wilson 95% bars, pooled
over the hosts' envelope_measured.csv files (scripts/sensys_analysis/envelope_measured.py).
    python scripts/sensys_analysis/fig_envelope.py --csv safety_envelope_paper/analysis/envelope_j3_atlas/envelope_measured.csv \
        safety_envelope_paper/analysis/envelope_j3_cetus/envelope_measured.csv --speed 70 --out figures/arch_safety_scale.pdf
Arms: reference (oracle), V2X2V prediction with the contract (late_fusion on), without (late_fusion off),
I2V object sharing (infra_only off); knob-keyed arms (gate kv) are excluded here (see fig_ablation.py).
"""
import argparse, collections, csv, json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from campaign_check import wilson  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GEOMS = [("openscenario_3_edge_late_fusion", "Occluded left turn"), ("scp_edge_late_fusion", "Straight crossing")]
ARMS = [(("oracle", "on"), "Exact-state reference", "k", "o"), (("late_fusion", "on"), "V2X2V prediction, +contract", "#1f77b4", "s"),
        (("late_fusion", "off"), "V2X2V prediction, no contract", "#d62728", "^"), (("infra_only", "off"), "I2V object sharing", "#2ca02c", "D")]


def pool(paths, speed, N=1):
    acc = collections.defaultdict(lambda: [0, 0])  # (geom, mgr, anch, bin) -> [k, n]
    for p in paths:
        for r in csv.DictReader(open(p)):
            if int(r["speed"]) != speed or int(r["N"]) != N:
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("assoc_", "beacon_fault", "beacon_validity")) or k == "behavior.hazard_response" for k in kv):
                continue
            key = (r["geom"], r["mgr"], r["anch"], float(r["age_p99_bin_ms"]))
            n = int(r["n"]); acc[key][0] += int(round(float(r["p_sop"]) * n)); acc[key][1] += n
    return acc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True); ap.add_argument("--speed", type=int, default=70)
    ap.add_argument("--max-age", type=float, default=1100); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    acc = pool(a.csv, a.speed)
    fig, axes = plt.subplots(1, 2, figsize=(7.2, 2.6), sharey=True)
    rows = []
    for ax, (geom, title) in zip(axes, GEOMS):
        for (mgr, anch), label, color, marker in ARMS:
            pts = sorted((b, kn) for (g, m, an, b), kn in acc.items() if g == geom and m == mgr and an == anch and b <= a.max_age)
            if not pts:
                continue
            xs = [b for b, _ in pts]; ps = [k / n for _, (k, n) in pts]
            lo = [max(0.0, p - wilson(k, n)[1]) for p, (_, (k, n)) in zip(ps, pts)]; hi = [max(0.0, wilson(k, n)[2] - p) for p, (_, (k, n)) in zip(ps, pts)]
            ax.errorbar(xs, ps, yerr=[lo, hi], fmt=marker + "-", color=color, ms=3.5, lw=1, capsize=1.5, label=label)
            rows += [dict(geom=geom, arm=label, bin_ms=b, n=n, k=k) for b, (k, n) in pts]
        ax.axhline(0.8, color="gray", lw=0.7, ls="--"); ax.set_title(f"{title}, {a.speed} km/h", fontsize=9)
        ax.set_xlabel("conflict actor's measured p99 age at use (ms)", fontsize=8); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
    axes[0].set_ylabel("$\\Pr[S_{op}]$"); axes[1].legend(fontsize=6.5, loc="lower left")
    fig.tight_layout(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out); fig.savefig(os.path.splitext(a.out)[0] + ".png", dpi=150)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote", a.out, len(rows), "bins")


if __name__ == "__main__":
    main()
