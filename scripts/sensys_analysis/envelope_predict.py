#!/usr/bin/env python3
"""Does the measured envelope predict closed-loop success?

    python scripts/sensys_analysis/envelope_predict.py <stamp> [<stamp> ...]
        [--axis critical] [--bin 50] [--out safety_envelope_paper/analysis/predict_<stamp>]

The reference envelope is fitted on the oracle runs only (controlled delay,
success against measured age at use, one curve per geometry and speed, made
monotone non-increasing over the age bins). Every other cell is held out:
its runs' measured ages go through the curve and the mean is the predicted
P(S_op); the observed P(S_op) is the cell's success rate. Three predictors
are reported so the limit of the scalar envelope is visible:

  p99        each run's conflict-actor p99 age through the curve
  coverage   the same, but a run whose conflict-actor coverage is under
             --min-coverage is predicted to fail (absent information)
  gap        the same as coverage, and a run whose longest conflict-actor
             gap exceeds --max-gap ticks is predicted to fail (burst loss)

Cells with self-ghost episodes carry a logic failure the envelope does not
model; they are listed separately with their own error, never pooled.

Writes <out>/predicted_vs_observed.csv, <out>/prediction_error.md and
<out>/predicted_vs_observed.png.
"""
import argparse
import collections
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import load_runs, wilson, ROOT  # noqa: E402
from envelope_measured import run_axis, label  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from reduce_per_tick_age import reduce_run  # noqa: E402


def per_tick_fields(r):
    rd = os.path.dirname(r["path"])
    for p in (os.path.join(rd, "per_tick_age.csv"), os.path.join(rd, "evaluation_output", "per_tick_age.csv")):
        if os.path.exists(p):
            m = reduce_run(p)
            return (m.get("conflict_coverage", float("nan")), m.get("conflict_max_gap_ticks", m.get("max_gap", float("nan"))))
    return float("nan"), float("nan")


def fit_curve(runs, binw):
    """Monotone non-increasing P(S_op) over age bins (pool-adjacent-violators)."""
    bins = collections.defaultdict(list)
    for r in runs:
        bins[int(r["axis_ms"] // binw)].append(r["s_op"])
    xs = sorted(bins)
    blocks = [[x, sum(bins[x]), len(bins[x])] for x in xs]   # [bin, successes, n]
    merged = []
    for b in blocks:
        merged.append(b)
        while len(merged) >= 2 and merged[-2][1] / merged[-2][2] < merged[-1][1] / merged[-1][2]:
            a, c = merged.pop(), merged.pop()
            merged.append([c[0], a[1] + c[1], a[2] + c[2]])
    # expand back to per-bin values (a merged block carries one rate)
    rate = {}
    for i, blk in enumerate(merged):
        end = merged[i + 1][0] if i + 1 < len(merged) else xs[-1] + 1
        for x in xs:
            if blk[0] <= x < end:
                rate[x] = blk[1] / blk[2]
    return rate


def predict(curve, age, binw):
    if age != age:
        return float("nan")
    b = int(age // binw)
    keys = sorted(curve)
    if b <= keys[0]:
        return curve[keys[0]]
    if b >= keys[-1]:
        return curve[keys[-1]]
    lo = max(k for k in keys if k <= b)
    hi = min(k for k in keys if k >= b)
    if lo == hi:
        return curve[lo]
    t = (b - lo) / (hi - lo)
    return curve[lo] + t * (curve[hi] - curve[lo])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamps", nargs="+")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--axis", choices=("critical", "global", "brake"), default="critical")
    ap.add_argument("--bin", type=float, default=50.0)
    ap.add_argument("--min-coverage", type=float, default=0.5)
    ap.add_argument("--max-gap", type=float, default=10.0, help="ticks")
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "safety_envelope_paper", "analysis", "predict_" + "_".join(a.stamps))
    os.makedirs(out, exist_ok=True)

    runs = []
    for st in a.stamps:
        runs += [r for r in load_runs(a.root, st) if not r.get("bad")]
    for r in runs:
        r["axis_ms"] = run_axis(r, a.axis)
        r["coverage"], r["max_gap"] = per_tick_fields(r)
    runs = [r for r in runs if r["axis_ms"] == r["axis_ms"]]
    print(f"{len(runs)} runs with a measured age")

    curves = {}
    for key in sorted(set((r["geom"], r["speed"]) for r in runs)):
        orc = [r for r in runs if (r["geom"], r["speed"]) == key and r["mgr"] == "oracle"]
        if len(orc) >= 10:
            curves[key] = fit_curve(orc, a.bin)
            print(f"reference {key}: {len(orc)} oracle runs, bins {sorted(curves[key])}")
    if not curves:
        print("no oracle cells to fit a reference on")
        return 2

    rows = []
    cells = collections.defaultdict(list)
    for r in runs:
        if r["mgr"] == "oracle":
            continue
        cells[(r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"], r["lat"])].append(r)
    for k in sorted(cells, key=str):
        v = cells[k]
        curve = curves.get((k[0], k[1]))
        if curve is None:
            continue
        p_obs, lo, hi = wilson(sum(r["s_op"] for r in v), len(v))
        preds = {}
        for name in ("p99", "coverage", "gap"):
            vals = []
            for r in v:
                p = predict(curve, r["axis_ms"], a.bin)
                if name in ("coverage", "gap") and r["coverage"] == r["coverage"] and r["coverage"] < a.min_coverage:
                    p = 0.0
                if name == "gap" and r["max_gap"] == r["max_gap"] and r["max_gap"] > a.max_gap:
                    p = 0.0
                vals.append(p)
            preds[name] = sum(vals) / len(vals)
        ghosts = sum(1 for r in v if r.get("focal_ghost_episodes", 0) > 0)
        rows.append(dict(geom=k[0], speed=k[1], arm=label(k[2], k[3], k[5]), N=k[4], latency_ms=k[6], n=len(v),
                         observed=round(p_obs, 3), lo=round(lo, 3), hi=round(hi, 3),
                         pred_p99=round(preds["p99"], 3), pred_coverage=round(preds["coverage"], 3), pred_gap=round(preds["gap"], 3),
                         ghost_runs=ghosts, kind="logic" if ghosts > 0 else "physics"))
    with open(os.path.join(out, "predicted_vs_observed.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    def mae(sel, col):
        s = [abs(r[col] - r["observed"]) for r in rows if r["kind"] == sel]
        return (sum(s) / len(s), len(s)) if s else (float("nan"), 0)

    with open(os.path.join(out, "prediction_error.md"), "w") as fh:
        fh.write(f"# Predicted against observed P(S_op), reference fitted on oracle runs ({', '.join(a.stamps)})\n\n")
        fh.write("| cells | predictor | mean abs error | n cells |\n|---|---|---|---|\n")
        for sel in ("physics", "logic"):
            for col, nm in (("pred_p99", "p99"), ("pred_coverage", "p99 + coverage"), ("pred_gap", "p99 + coverage + gap")):
                e, n = mae(sel, col)
                fh.write(f"| {sel} | {nm} | {e:.3f} | {n} |\n")
        fh.write("\nphysics: cells without a self-ghost episode; logic: cells with one, whose failures the envelope does not model.\n")
    print(open(os.path.join(out, "prediction_error.md")).read())

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return 0
    fig, ax = plt.subplots(figsize=(4.2, 4.0))
    for sel, mk in (("physics", "o"), ("logic", "x")):
        sub = [r for r in rows if r["kind"] == sel]
        if sub:
            ax.errorbar([r["pred_p99"] for r in sub], [r["observed"] for r in sub],
                        yerr=[[r["observed"] - r["lo"] for r in sub], [r["hi"] - r["observed"] for r in sub]],
                        fmt=mk, capsize=2, label=sel, alpha=0.8)
    ax.plot([0, 1], [0, 1], color="gray", lw=0.8, ls="--")
    ax.set_xlabel("predicted P(S_op) from measured age")
    ax.set_ylabel("observed P(S_op)")
    ax.set_xlim(-0.02, 1.02)
    ax.set_ylim(-0.02, 1.02)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "predicted_vs_observed.png"), dpi=160)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
