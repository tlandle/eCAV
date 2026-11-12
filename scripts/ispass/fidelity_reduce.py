#!/usr/bin/env python
"""Reduce the fidelity ablation (ISPASS experiment 2).

    python scripts/ispass/fidelity_reduce.py <stamp> [--root experiment_results] [--csv out.csv] [--fig out.pdf]

One row per (geometry, architecture, condition, table sender count): runs, operational
success with a Wilson 95 % interval, the paired difference to condition A over shared
seeds with a bootstrap 95 % interval, mean run-level p99 age at use of the conflict
actor, mean age at the braking decision, the fraction of conflict-actor planner
updates within the geometry's useful-age budget, conflict-actor coverage, and the
composed delay terms the edge recorded (compute, uplink, downlink, ms).
"""
import argparse, csv, glob, json, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from reduce_per_tick_age import reduce_run  # noqa: E402

GEOM = {"openscenario_3_edge_late_fusion": "left turn", "scp_edge_late_fusion": "straight crossing"}
BUDGET_MS = {"left turn": 348.0, "straight crossing": 300.0}   # exact-state useful-age budgets at 70 km/h
ARCH_ORDER = ["I2V object sharing", "V2X2V prediction", "intermediate fusion"]
COND_ORDER = ["A", "B", "C", "D"]
COND_NAME = {"A": "idealized", "B": "compute", "C": "network", "D": "full"}


def arch_of(path):
    if "/baseline_wf/" in path: return "intermediate fusion"
    if "/mgr_infra_only/" in path: return "I2V object sharing"
    if "/mgr_late_fusion/" in path: return "V2X2V prediction"
    return None


def cond_of(kv):
    c = str(kv.get("compute_delay", "")).lower() == "measured"
    r = bool(kv.get("use_ns3_lut", False)) or bool(kv.get("lut_dl", False))
    return {(False, False): "A", (True, False): "B", (False, True): "C", (True, True): "D"}[(c, r)]


def wilson(k, n, z=1.96):
    if n == 0: return (float("nan"),) * 2
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return max(0.0, c - h), min(1.0, c + h)


def within_budget(csv_path, budget):
    num = den = 0
    with open(csv_path) as fh:
        for r in csv.DictReader(fh):
            if r.get("is_conflict_actor") == "1" and r.get("age_at_use_ms") not in ("", None):
                den += 1; num += float(r["age_at_use_ms"]) <= budget
    return num / den if den else float("nan")


def load(stamp, root):
    rows = []
    for mp in glob.glob(os.path.join(root, "*", stamp, "**", "simulation_metrics.json"), recursive=True):
        rel = os.path.relpath(mp, root); scen = rel.split(os.sep)[0]
        geom = GEOM.get(re.sub(r"_wf$", "", scen)); arch = arch_of("/" + rel.replace(os.sep, "/"))
        if geom is None or arch is None: continue
        try: m = json.load(open(mp))
        except ValueError: continue
        if "s_op" not in m or "config_edge_kv" not in m: continue   # the copy under evaluation_output carries no config
        if not os.path.basename(os.path.dirname(mp)).startswith("run_"): continue
        kv = m.get("config_edge_kv", {}) or {}
        run_dir = os.path.dirname(mp); pt = os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")
        age = reduce_run(pt) if os.path.exists(pt) else {}
        edges = list((m.get("edges") or {}).values()); comp = (edges[0].get("composed_delay") if edges else None) or {}
        rows.append(dict(
            geom=geom, arch=arch, cond=cond_of(kv), lut_n=int(kv.get("lut_n", 0) or 0),
            rep=int(re.findall(r"run_(\d+)", run_dir)[-1]), s_op=int(m["s_op"]),
            collided=1 - int(m.get("s_coll", 1)), ghost=1 - int(m.get("s_ghost", 1)), fbrake=1 - int(m.get("s_fp", 1)),
            p99=age.get("critical_object_p99_ms", float("nan")), brake_age=age.get("conflict_brake_age_ms", float("nan")),
            coverage=age.get("conflict_coverage", float("nan")),
            in_budget=within_budget(pt, BUDGET_MS[geom]) if os.path.exists(pt) else float("nan"),
            compute=comp.get("compute_ms_mean", 0.0), ul=comp.get("ul_ms_mean", 0.0), dl=comp.get("dl_ms_mean", 0.0)))
    return rows


def nm(v):
    v = [x for x in v if x == x]
    return float(np.mean(v)) if v else float("nan")


def summarize(rows):
    out = []
    keys = sorted({(r["geom"], r["arch"], r["cond"], r["lut_n"]) for r in rows},
                  key=lambda k: (k[0], ARCH_ORDER.index(k[1]), k[3] if k[2] in "CD" else -1, COND_ORDER.index(k[2])))
    rng = np.random.default_rng(0)
    for g, a, c, n in keys:
        cell = [r for r in rows if (r["geom"], r["arch"], r["cond"], r["lut_n"]) == (g, a, c, n)]
        base = {r["rep"]: r["s_op"] for r in rows if (r["geom"], r["arch"], r["cond"]) == (g, a, "A")}
        k = sum(r["s_op"] for r in cell); lo, hi = wilson(k, len(cell))
        d = np.array([r["s_op"] - base[r["rep"]] for r in cell if r["rep"] in base], dtype=float)
        if len(d) and c != "A":
            bs = [d[rng.integers(0, len(d), len(d))].mean() for _ in range(2000)]
            dm, dlo, dhi = d.mean(), *np.percentile(bs, [2.5, 97.5])
        else:
            dm = dlo = dhi = float("nan")
        out.append(dict(geom=g, arch=a, cond=c, lut_n=n, n=len(cell), s_op=k / len(cell), lo=lo, hi=hi,
                        d_vs_A=dm, d_lo=dlo, d_hi=dhi, n_pairs=len(d),
                        collided=nm([r["collided"] for r in cell]), ghost=nm([r["ghost"] for r in cell]), fbrake=nm([r["fbrake"] for r in cell]),
                        p99_age=nm([r["p99"] for r in cell]), brake_age=nm([r["brake_age"] for r in cell]),
                        in_budget=nm([r["in_budget"] for r in cell]), coverage=nm([r["coverage"] for r in cell]),
                        compute_ms=nm([r["compute"] for r in cell]), ul_ms=nm([r["ul"] for r in cell]), dl_ms=nm([r["dl"] for r in cell])))
    return out


def print_table(summ):
    print(f"{'geometry':<18}{'architecture':<22}{'cond':<12}{'N':>3}{'n':>4}  {'S_op [95% CI]':<20}{'vs A [95% CI]':<24}"
          f"{'coll':>5}{'ghost':>6}{'fbrk':>5}{'p99 age':>8}{'brk age':>8}{'in bud':>7}{'cov':>6}{'comp':>6}{'ul':>6}{'dl':>6}")
    for s in summ:
        dv = "" if s["d_vs_A"] != s["d_vs_A"] else f"{s['d_vs_A']:+.2f} [{s['d_lo']:+.2f},{s['d_hi']:+.2f}]"
        print(f"{s['geom']:<18}{s['arch']:<22}{s['cond'] + ' ' + COND_NAME[s['cond']]:<12}{s['lut_n'] or '':>3}{s['n']:>4}  "
              f"{s['s_op']:.2f} [{s['lo']:.2f},{s['hi']:.2f}]   {dv:<24}{s['collided']:>5.2f}{s['ghost']:>6.2f}{s['fbrake']:>5.2f}{s['p99_age']:>8.0f}{s['brake_age']:>8.0f}"
              f"{s['in_budget']:>7.2f}{s['coverage']:>6.2f}{s['compute_ms']:>6.0f}{s['ul_ms']:>6.0f}{s['dl_ms']:>6.0f}")


def figure(summ, path, lut_n=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.linewidth": 0.5, "pdf.fonttype": 42})
    geoms = [g for g in ("left turn", "straight crossing") if any(s["geom"] == g for s in summ)]
    fig, axes = plt.subplots(2, len(geoms), figsize=(3.4, 2.6), squeeze=False, sharex=True)
    colors = {"A": "#bdbdbd", "B": "#6baed6", "C": "#fd8d3c", "D": "#525252"}
    for j, g in enumerate(geoms):
        archs = [a for a in ARCH_ORDER if any(s["geom"] == g and s["arch"] == a for s in summ)]
        for i, c in enumerate(COND_ORDER):
            xs, ys, lo, hi, ages = [], [], [], [], []
            for k, a in enumerate(archs):
                cell = [s for s in summ if (s["geom"], s["arch"], s["cond"]) == (g, a, c) and (c in "AB" or lut_n is None or s["lut_n"] == lut_n)]
                if not cell: continue
                s = cell[0]; xs.append(k + (i - 1.5) * 0.2); ys.append(s["s_op"]); lo.append(s["s_op"] - s["lo"]); hi.append(s["hi"] - s["s_op"]); ages.append(s["p99_age"])
            axes[0][j].bar(xs, ys, 0.2, yerr=[lo, hi], color=colors[c], error_kw=dict(lw=0.5, capsize=1), label=f"{c}: {COND_NAME[c]}")
            axes[1][j].bar(xs, ages, 0.2, color=colors[c])
        axes[0][j].set_title(g, fontsize=7, pad=2); axes[0][j].set_ylim(0, 1.05)
        axes[1][j].axhline(BUDGET_MS[g], color="k", lw=0.5, ls=":")
        axes[1][j].set_xticks(range(len(archs))); axes[1][j].set_xticklabels([a.replace(" ", "\n", 1) for a in archs], fontsize=7)
    axes[0][0].set_ylabel("operational success"); axes[1][0].set_ylabel("p99 age at use (ms)")
    axes[0][0].legend(fontsize=7, frameon=False, ncol=2, loc="lower left", handlelength=1, columnspacing=0.8)
    fig.tight_layout(pad=0.3); fig.savefig(path); print("wrote", path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("stamp"); ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--csv"); ap.add_argument("--fig"); ap.add_argument("--fig-n", type=int, default=None)
    a = ap.parse_args()
    rows = load(a.stamp, a.root); print(f"{len(rows)} runs under stamp {a.stamp}")
    summ = summarize(rows); print_table(summ)
    if a.csv and summ:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
    if a.fig and summ: figure(summ, a.fig, a.fig_n)
