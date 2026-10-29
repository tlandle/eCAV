#!/usr/bin/env python3
"""Trace replay figure and prediction test (Section 5.2.5).
    python scripts/sensys_analysis/fig_trace_replay.py --runs safety_envelope_paper/analysis/envelope_j3_atlas/runs.csv \
        safety_envelope_paper/analysis/envelope_j3_cetus/runs.csv --stamp sensys_nov27_j3 --root experiment_results \
        --geom openscenario_3_edge_late_fusion --speed 70 --out figures/trace_replay.pdf
Curves: the envelope on the configured-delay cells (reference, V2X2V +contract, no contract), one point per
delay at the cell's mean measured conflict-actor p99 age, Wilson bars (from runs.csv). Markers: the replay
cells (cell dirs matching v2x_regime*, one per SEE-V2X intersection and arm) at the age they produced, with
their observed P(S_op). Prediction: the monotone non-increasing fit of each arm's envelope, read at every
replay run's measured age and averaged, against the observed success; written to a CSV and a Markdown table
with the conflict actor's availability per cell: the share of ticks between its first and last consumed
observation at which the planner consumed one (coverage) and the longest run of ticks without one (gap).
"""
import argparse, collections, csv, glob, json, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(ROOT, "scripts")); sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from campaign_check import wilson  # noqa: E402
from recompute_metrics import compute_run_metrics  # noqa: E402
from reduce_per_tick_age import reduce_run  # noqa: E402
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402

ARMS = [(("oracle", "on"),) + figstyle.ARMS["reference"], (("late_fusion", "on"),) + figstyle.ARMS["contract"],
        (("late_fusion", "off"),) + figstyle.ARMS["naive"]]
SEG = {"L": "intersection 3 (lowest delay)", "M": "intersection 4", "H": "intersection 5 (highest delay)"}


def pav(points):
    """Monotone non-increasing fit of success over age: pool adjacent violators on (x, k, n)."""
    blocks = [[x, k, n] for x, k, n in sorted(points)]
    out = []
    for b in blocks:
        out.append(b)
        while len(out) > 1 and out[-1][1] / out[-1][2] > out[-2][1] / out[-2][2]:
            x1, k1, n1 = out.pop(); x0, k0, n0 = out.pop()
            out.append([x0, k0 + k1, n0 + n1])  # keep the lower age as the block's start
    return [(x, k / n) for x, k, n in out]


def conflict_coverage(pt):
    """(coverage, longest gap in ticks) of the conflict actor in one run's per-tick log: over the
    logged ticks from its first to its last consumed observation, the share at which the planner
    consumed one, and the longest run of logged ticks without one. A tick with no rows at all is a
    tick the logger did not record (the late-fusion edge skips some), not evidence of a missing
    observation, so the window counts logged ticks only."""
    seen = set(); logged = set()
    for r in csv.DictReader(open(pt)):
        t = int(float(r["tick"])); logged.add(t)
        if r.get("is_conflict_actor") == "1":
            seen.add(t)
    if not seen:
        return float("nan"), 0
    lo, hi = min(seen), max(seen)
    window = [t for t in sorted(logged) if lo <= t <= hi]
    gap = run = 0
    for t in window:
        run = 0 if t in seen else run + 1
        gap = max(gap, run)
    return len(seen) / len(window), gap


def predict(fit, age):
    p = fit[0][1]
    for x, v in fit:
        if age >= x:
            p = v
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True); ap.add_argument("--stamp", required=True)
    ap.add_argument("--root", default="experiment_results"); ap.add_argument("--geom", default="openscenario_3_edge_late_fusion")
    ap.add_argument("--speed", type=int, default=70); ap.add_argument("--max-age", type=float, default=1100)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    # envelope cells from runs.csv
    env = collections.defaultdict(list)  # (mgr, anch, lat) -> [(age, s_op)]
    for p in a.runs:
        for r in csv.DictReader(open(p)):
            if r["geom"] != a.geom or int(r["speed"]) != a.speed or int(r["N"]) != 1:
                continue
            if "v2x_regime" in r.get("cell", ""):
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("assoc_", "beacon_fault", "beacon_validity")) or k == "behavior.hazard_response" for k in kv):
                continue
            env[(r["mgr"], r["anch"], int(r["lat"]))].append((float(r["axis_ms"]), int(r["s_op"])))
    # replay cells from the tree
    rep = collections.defaultdict(list)  # (mgr, anch, seg) -> [(age, s_op)]
    pat = os.path.join(a.root, a.geom, a.stamp, "speed_spd_%d/baseline_default/mgr_*/anchoring_*/*v2x_regime*/run_*/simulation_metrics.json" % a.speed)
    for f in sorted(glob.glob(pat)):
        if "/evaluation_output/" in f:
            continue
        d = json.load(open(f)); d["_run_dir"] = os.path.dirname(f)
        m = compute_run_metrics(d)
        if m is None:
            continue
        seg = re.search(r"v2x_regime([LMH])", f).group(1)
        age = float("nan"); cov, gap = float("nan"), 0
        for pt in (os.path.join(d["_run_dir"], "per_tick_age.csv"), os.path.join(d["_run_dir"], "evaluation_output", "per_tick_age.csv")):
            if os.path.exists(pt):
                rr = reduce_run(pt); age = rr.get("critical_object_p99_ms", float("nan"))
                if age != age:
                    age = rr.get("global_p99_age_ms", float("nan"))
                cov, gap = conflict_coverage(pt)
                break
        if age == age:
            rep[(d["config_manager_type"], "on" if d["config_anchoring"] else "off", seg)].append((age, int(bool(m["s_op"])), cov, gap))
    figstyle.rc()
    fig, ax = plt.subplots(figsize=(figstyle.COL, 2.2))  # column width, printed at scale 1
    rows = []
    for (mgr, anch), label, color, marker in ARMS:
        cells = sorted((lat, v) for (m, an, lat), v in env.items() if m == mgr and an == anch)
        pts = [(sum(x for x, _ in v) / len(v), sum(o for _, o in v), len(v)) for _, v in cells]
        pts = [t for t in pts if t[0] <= a.max_age]
        if pts:
            xs = [x for x, _, _ in pts]; ps = [k / n for _, k, n in pts]
            lo = [max(0.0, p - wilson(k, n)[1]) for p, (_, k, n) in zip(ps, pts)]; hi = [max(0.0, wilson(k, n)[2] - p) for p, (_, k, n) in zip(ps, pts)]
            ax.errorbar(xs, ps, yerr=[lo, hi], fmt=marker + "-", color=color, label=label, ecolor=color, elinewidth=0.5, capsize=1.5, capthick=0.5, alpha=0.85, zorder=2)
        fit = pav([(x, k, n) for x, k, n in pts]) if pts else []
        for seg in ("L", "M", "H"):
            v = rep.get((mgr, anch, seg), [])
            if not v:
                continue
            k = sum(t[1] for t in v); n = len(v); x = sum(t[0] for t in v) / n; p = k / n
            pred = sum(predict(fit, t[0]) for t in v) / n if fit else float("nan")
            cov = sum(t[2] for t in v) / n; gap = max(t[3] for t in v)
            w = wilson(k, n)
            ax.errorbar([x], [p], yerr=[[max(0.0, p - w[1])], [max(0.0, w[2] - p)]], fmt=marker, color=color, mfc="white", ms=8, mew=1.5, zorder=4, **figstyle.EB)
            ax.annotate(seg, (x, p), textcoords="offset points", xytext={"L": (-11, 6), "M": (3, -11), "H": (4, 7)}[seg], fontsize=7, fontweight="bold", color=color, zorder=5)
            rows.append(dict(arm=label, segment=SEG[seg], n=n, age_mean_ms=round(x, 1), observed=round(p, 3),
                             wilson_lo=round(w[1], 3), wilson_hi=round(w[2], 3), predicted=round(pred, 3), inside_interval=int(w[1] <= pred <= w[2]),
                             coverage_mean=round(cov, 3), longest_gap_ms=int(gap * 50)))
    ax.axhline(0.8, color="gray", lw=0.7, ls="--"); ax.set_ylim(-0.05, 1.05); ax.set_xlim(left=0); ax.grid(alpha=0.3)
    ax.set_xlabel("conflict actor's measured p99 age at use (ms)"); ax.set_ylabel("$\\Pr[S_{op}]$")
    h, l = ax.get_legend_handles_labels(); fig.legend(h, l, loc="outside upper center", ncol=2, handlelength=1.2, columnspacing=0.8)
    figstyle.save(fig, a.out)
    base = os.path.splitext(a.out)[0]
    if rows:
        with open(base + ".csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
        with open(base + ".md", "w") as fh:
            fh.write("| arm | segment | n | measured p99 age (ms) | coverage | longest gap (ms) | observed P | Wilson | predicted P | inside |\n|---|---|---|---|---|---|---|---|---|---|\n")
            for r in rows:
                fh.write(f"| {r['arm']} | {r['segment']} | {r['n']} | {r['age_mean_ms']} | {r['coverage_mean']} | {r['longest_gap_ms']} | {r['observed']} | [{r['wilson_lo']}, {r['wilson_hi']}] | {r['predicted']} | {r['inside_interval']} |\n")
        print(open(base + ".md").read())
    print("wrote", a.out, "replay cells:", len(rows))


if __name__ == "__main__":
    main()
