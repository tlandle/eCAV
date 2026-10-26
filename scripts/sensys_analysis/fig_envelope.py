#!/usr/bin/env python3
"""Envelope figure (arch_safety_scale): P(S_op) against the conflict actor's measured p99 age at use,
one panel per junction geometry at one approach speed, one curve per arm, Wilson 95% bars, pooled
over the hosts' runs.csv files (scripts/sensys_analysis/envelope_measured.py). One point per configured
delay cell, placed at the cell's mean measured age, Wilson y bars (the cell's 10th to 90th percentile age is in the CSV).
Drawn at text width (figstyle.TXT) for a figure* float.
    python scripts/sensys_analysis/fig_envelope.py --csv safety_envelope_paper/analysis/envelope_j3_atlas/runs.csv \
        safety_envelope_paper/analysis/envelope_j3_cetus/runs.csv --speed 70 --out figures/arch_safety_scale.pdf
Arms: reference (oracle), V2X2V prediction with the contract (late_fusion on), without (late_fusion off),
I2V object sharing (infra_only off); knob-keyed arms (gate kv) are excluded here (see fig_ablation.py).
"""
import argparse, collections, csv, json, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from campaign_check import wilson  # noqa: E402
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402

GEOMS = [("openscenario_3_edge_late_fusion", "Occluded left turn"), ("scp_edge_late_fusion", "Straight crossing")]
ARMS = [(("oracle", "on"), "reference"), (("late_fusion", "on"), "contract"), (("late_fusion", "off"), "naive"),
        (("infra_only", "off"), "i2v")]


def pool(paths, speed, N=1):
    """(geom, mgr, anch, lat) -> list of (axis_ms, s_op) over the runs of all files."""
    acc = collections.defaultdict(list)
    for p in paths:
        for r in csv.DictReader(open(p)):
            if int(r["speed"]) != speed or int(r["N"]) != N:
                continue
            if "v2x_regime" in r.get("cell", ""):
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("assoc_", "beacon_fault", "beacon_validity")) or k == "behavior.hazard_response" for k in kv):
                continue
            acc[(r["geom"], r["mgr"], r["anch"], int(r["lat"]))].append((float(r["axis_ms"]), int(r["s_op"])))
    return acc


def pct(v, q):
    v = sorted(v); i = (len(v) - 1) * q; lo = int(math.floor(i)); hi = min(lo + 1, len(v) - 1)
    return v[lo] + (v[hi] - v[lo]) * (i - lo)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True); ap.add_argument("--speed", type=int, default=70)
    ap.add_argument("--max-age", type=float, default=1100); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    acc = pool(a.csv, a.speed)
    figstyle.rc()
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.TXT, 1.75), sharey=True)
    rows = []
    for i, (ax, (geom, title)) in enumerate(zip(axes, GEOMS)):
        for (mgr, anch), key in ARMS:
            label, color, marker = figstyle.ARMS[key]
            cells = sorted((lat, v) for (g, m, an, lat), v in acc.items() if g == geom and m == mgr and an == anch)
            pts = []
            for lat, v in cells:
                ages = [x for x, _ in v]; k = sum(o for _, o in v); n = len(v)
                x = sum(ages) / n
                if x > a.max_age:
                    continue
                pts.append((lat, x, pct(ages, 0.1), pct(ages, 0.9), k, n))
            if not pts:
                continue
            xs = [x for _, x, _, _, _, _ in pts]; ps = [k / n for _, _, _, _, k, n in pts]
            lo = [max(0.0, p - wilson(k, n)[1]) for p, (_, _, _, _, k, n) in zip(ps, pts)]; hi = [max(0.0, wilson(k, n)[2] - p) for p, (_, _, _, _, k, n) in zip(ps, pts)]
            xl = [max(0.0, x - l) for _, x, l, _, _, _ in pts]; xh = [max(0.0, h - x) for _, x, _, h, _, _ in pts]
            ax.errorbar(xs, ps, yerr=[lo, hi], fmt=marker + "-", color=color, label=label, **figstyle.EB)
            rows += [dict(geom=geom, arm=label, lat=lat, age_mean_ms=round(x, 1), age_p10=round(l, 1), age_p90=round(h, 1), n=n, k=k) for lat, x, l, h, k, n in pts]
        ax.axhline(0.8, color="gray", lw=0.8, ls="--"); ax.set_title(f"({'ab'[i]}) {title}, {a.speed} km/h")
        ax.set_xlabel("conflict actor's measured p99 age at use (ms)"); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
    axes[0].set_ylabel("$\\Pr[S_{op}]$")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="outside upper center", ncol=4)
    figstyle.save(fig, a.out)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote", a.out, len(rows), "bins")


if __name__ == "__main__":
    main()
