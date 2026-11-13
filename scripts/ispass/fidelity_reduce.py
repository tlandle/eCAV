#!/usr/bin/env python
"""Reduce the fidelity ablation (ISPASS experiment 2).

    python scripts/ispass/fidelity_reduce.py <stamp> [--root experiment_results] [--csv out.csv] [--fig out.pdf]

One row per (geometry, architecture, condition, table sender count): runs, operational
success with a Wilson 95 % interval, the paired difference to condition A over shared
seeds with a bootstrap 95 % interval, mean run-level p99 age at use of the conflict
actor, mean age at the braking decision, the fraction of conflict-actor planner
updates within the geometry's useful-age budget, conflict-actor coverage, and the
composed delay terms the edge recorded (compute, uplink, downlink, ms).

Attribution columns (attribution()): onset of the true-positive forecast brake after the
conflict actor's launch (median and range, ms), the fraction of runs without one, the minimum
separation (median and range, m), the class and time of the first hazard brake of any kind, and
the fraction of runs in which a brake the conflict track's forecast did not cause (another
hazard class, or sustained full braking without a hazard event) came first or replaced it.
Operational success (no collision, no false brake) stays the headline.
"""
import argparse, csv, glob, json, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from reduce_per_tick_age import reduce_run  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "scripts", "sensys_analysis"))
from campaign_check import load_runs  # noqa: E402  (operational success as the safety-envelope study defines it)

GEOM = {"openscenario_3_edge_late_fusion": "left turn", "scp_edge_late_fusion": "straight crossing"}
BUDGET_MS = {"left turn": 348.0, "straight crossing": 300.0}   # exact-state useful-age budgets at 70 km/h
ARCH_ORDER = ["no cooperation", "I2V object sharing", "V2X2V prediction", "intermediate fusion"]
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


def attribution(pt_csv):
    """What braked the connected vehicle, from the per-tick age log of one run.

    launch is the first tick the conflict actor moves. Times are ms after launch.
    tp_onset: start of the first braking episode on the conflict actor's track (a hazard event
        on that track followed by another within three ticks).
    first_class, first_ms: class and time of the first hazard brake of any class.
    other_first: a brake that the conflict track's forecast did not cause came before the
        true-positive brake, or there was no true-positive brake at all: a hazard of another
        class (self_ghost, other_fp, ...), or five consecutive ticks of full braking without a
        hazard event (a speed limit for a curve, a stop hold).
    min_sep: smallest centre distance between the connected vehicle and the conflict actor."""
    gt, per, ev = {}, {}, []
    with open(pt_csv) as fh:
        for r in csv.DictReader(fh):
            t = int(r["tick"])
            if r.get("conflict_gt_x"):
                gt[t] = (float(r["conflict_gt_x"]), float(r["conflict_gt_y"]))
            per.setdefault(t, r)
            if r.get("brake_event") == "1":
                cls = r.get("gt_brake_class") or "unlabelled"
                # the labeler's matched actor decides whether the brake was on the conflict
                # actor's track; a class of self_ghost on that track is the labeler's
                # provenance history, not the vehicle's own echo (logs without the column
                # fall back to the class)
                actor, cid = r.get("brake_gt_actor_id"), r.get("conflict_actor_id")
                if actor not in (None, "") and cid not in (None, ""):
                    try:
                        cls = "true_positive" if int(float(actor)) == int(float(cid)) else (
                            cls if cls != "true_positive" else "other_actor")
                    except ValueError:
                        pass
                ev.append((t, cls))
    nan = float("nan")
    out = dict(tp_onset=nan, first_ms=nan, first_class="", other_first=nan, min_sep=nan, sustained_ms=nan, false_brake=0)
    ts = sorted(gt)
    launch = next((t for t in ts if t - 1 in gt and math.hypot(gt[t][0] - gt[t - 1][0], gt[t][1] - gt[t - 1][1]) > 0.02), None)
    if launch is None:
        return out
    ev = sorted({e for e in ev if e[0] >= launch})
    # onset of the braking episode: the first true-positive event that is followed by another
    # within three ticks (a single event that the next check withdraws does not brake the car)
    tps = [t for t, c in ev if c == "true_positive"]
    tp = next((t for i, t in enumerate(tps) if i + 1 < len(tps) and tps[i + 1] - t <= 3), None)
    if tp is None and len(tps) == 1:
        tp = tps[0]
    other = next((t for t, c in ev if c != "true_positive"), None)
    ticks = [t for t in sorted(per) if t >= launch]
    hazard_ticks = {t for t, _ in ev}
    sustained = None
    for i in range(len(ticks) - 4):
        w = ticks[i:i + 5]
        if all(float(per[u].get("ctrl_brake") or 0) >= 0.9 for u in w) and not any(u in hazard_ticks for u in w):
            sustained = w[0]; break
    # False brake: a hazard on a track that is not the conflict actor's (another actor, or no
    # actor) that starts sustained full braking (five consecutive ticks beginning within three
    # ticks of the event) before the forecast brake, or with no forecast brake at all.
    def _sustained_from(t0):
        for s0 in (u for u in ticks if t0 <= u <= t0 + 3):
            w = [u for u in ticks if s0 <= u < s0 + 5]
            if len(w) == 5 and all(float(per[u].get("ctrl_brake") or 0) >= 0.9 for u in w):
                return True
        return False
    false_brake = any(_sustained_from(t) for t, c in ev if c != "true_positive" and (tp is None or t < tp))
    cands = [t for t in (other, sustained) if t is not None]
    first_other = min(cands) if cands else None
    seps = [math.hypot(float(per[t]["ego_x"]) - gt[t][0], float(per[t]["ego_y"]) - gt[t][1])
            for t in ticks if t in gt and per[t].get("ego_x")]
    out.update(tp_onset=(tp - launch) * 50.0 if tp is not None else nan,
               first_ms=(ev[0][0] - launch) * 50.0 if ev else nan, first_class=ev[0][1] if ev else "none",
               sustained_ms=(sustained - launch) * 50.0 if sustained is not None else nan,
               other_first=float(first_other is not None and (tp is None or first_other < tp)),
               false_brake=int(false_brake),
               min_sep=min(seps) if seps else nan)
    return out


def med_rng(v):
    v = sorted(x for x in v if x == x)
    return (float(np.median(v)), v[0], v[-1]) if v else (float("nan"),) * 3


def load(stamp, root):
    rows = []
    for r in load_runs(root, stamp):
        if r.get("bad"): continue
        rel = os.path.relpath(r["path"], root); scen = rel.split(os.sep)[0]
        geom = GEOM.get(re.sub(r"_wf$", "", scen)); arch = arch_of("/" + rel.replace(os.sep, "/"))
        if geom is None or arch is None: continue
        if "_loss_100_" in rel: arch = "no cooperation"      # every shared message dropped (stage nocoop)
        m = json.load(open(r["path"])); kv = m.get("config_edge_kv", {}) or {}
        run_dir = os.path.dirname(r["path"]); pt = os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")
        age = reduce_run(pt) if os.path.exists(pt) else {}
        att = attribution(pt) if os.path.exists(pt) else {}
        edges = list((m.get("edges") or {}).values()); comp = (edges[0].get("composed_delay") if edges else None) or {}
        # a brake that a non-conflict track caused is a false brake and fails the run, whatever
        # class the run's own labeler gave it
        fb = int(r.get("focal_fp_episodes", 0) > 0 or att.get("false_brake", 0))
        rows.append(dict(
            geom=geom, arch=arch, cond=cond_of(kv), lut_n=int(kv.get("lut_n", 0) or 0), lat=int(r.get("lat", 0) or 0), rep=int(r["rep"]),
            s_op=int(r["s_op"]) if not att.get("false_brake", 0) else 0,
            collided=int(r.get("focal_collisions", 0) > 0), ghost=int(r.get("focal_ghost_episodes", 0) > 0),
            fbrake=fb,
            p99=age.get("critical_object_p99_ms", float("nan")), brake_age=age.get("conflict_brake_age_ms", float("nan")),
            coverage=age.get("conflict_coverage", float("nan")),
            in_budget=within_budget(pt, BUDGET_MS[geom]) if os.path.exists(pt) else float("nan"),
            compute=comp.get("compute_ms_mean", 0.0), ul=comp.get("ul_ms_mean", 0.0), dl=comp.get("dl_ms_mean", 0.0),
            tp_onset=att.get("tp_onset", float("nan")), first_ms=att.get("first_ms", float("nan")), first_class=att.get("first_class", ""),
            other_first=att.get("other_first", float("nan")), min_sep=att.get("min_sep", float("nan"))))
    return rows


def nm(v):
    v = [x for x in v if x == x]
    return float(np.mean(v)) if v else float("nan")


def summarize(rows):
    out = []
    keys = sorted({(r["geom"], r["arch"], r["cond"], r["lut_n"], r["lat"]) for r in rows},
                  key=lambda k: (k[0], k[4], ARCH_ORDER.index(k[1]), k[3] if k[2] in "CD" else -1, COND_ORDER.index(k[2])))
    rng = np.random.default_rng(0)
    for g, a, c, n, lat in keys:
        cell = [r for r in rows if (r["geom"], r["arch"], r["cond"], r["lut_n"], r["lat"]) == (g, a, c, n, lat)]
        # paired reference: the idealized condition of the same architecture at the same added delay
        base = {r["rep"]: r["s_op"] for r in rows if (r["geom"], r["arch"], r["cond"], r["lat"]) == (g, a, "A", lat)}
        k = sum(r["s_op"] for r in cell); lo, hi = wilson(k, len(cell))
        d = np.array([r["s_op"] - base[r["rep"]] for r in cell if r["rep"] in base], dtype=float)
        if len(d) and c != "A":
            bs = [d[rng.integers(0, len(d), len(d))].mean() for _ in range(2000)]
            dm, dlo, dhi = d.mean(), *np.percentile(bs, [2.5, 97.5])
        else:
            dm = dlo = dhi = float("nan")
        tp_m, tp_lo, tp_hi = med_rng([r["tp_onset"] for r in cell])
        sep_m, sep_lo, sep_hi = med_rng([r["min_sep"] for r in cell])
        classes = [r["first_class"] for r in cell if r["first_class"]]
        first_class = max(set(classes), key=classes.count) if classes else ""
        out.append(dict(geom=g, arch=a, cond=c, lut_n=n, lat=lat, n=len(cell), s_op=k / len(cell), lo=lo, hi=hi,
                        d_vs_A=dm, d_lo=dlo, d_hi=dhi, n_pairs=len(d),
                        collided=nm([r["collided"] for r in cell]), ghost=nm([r["ghost"] for r in cell]), fbrake=nm([r["fbrake"] for r in cell]),
                        p99_age=nm([r["p99"] for r in cell]), brake_age=nm([r["brake_age"] for r in cell]),
                        in_budget=nm([r["in_budget"] for r in cell]), coverage=nm([r["coverage"] for r in cell]),
                        compute_ms=nm([r["compute"] for r in cell]), ul_ms=nm([r["ul"] for r in cell]), dl_ms=nm([r["dl"] for r in cell]),
                        tp_brake_ms=tp_m, tp_brake_min=tp_lo, tp_brake_max=tp_hi,
                        no_tp_brake=nm([float(r["tp_onset"] != r["tp_onset"]) for r in cell]),
                        min_sep_m=sep_m, min_sep_min=sep_lo, min_sep_max=sep_hi,
                        first_hazard_class=first_class, first_hazard_ms=med_rng([r["first_ms"] for r in cell])[0],
                        other_brake_first=nm([r["other_first"] for r in cell])))
    return out


def print_table(summ):
    print(f"{'geometry':<18}{'architecture':<22}{'cond':<12}{'lat':>4}{'N':>3}{'n':>4}  {'S_op [95% CI]':<20}{'vs A [95% CI]':<24}"
          f"{'coll':>5}{'ghost':>6}{'fbrk':>5}{'p99 age':>8}{'brk age':>8}{'in bud':>7}{'cov':>6}{'comp':>6}{'ul':>6}{'dl':>6}"
          f"  {'TP brake ms [range]':<22}{'noTP':>5}  {'min sep m [range]':<19}{'first hazard':<20}{'other 1st':>9}")
    for s in summ:
        dv = "" if s["d_vs_A"] != s["d_vs_A"] else f"{s['d_vs_A']:+.2f} [{s['d_lo']:+.2f},{s['d_hi']:+.2f}]"
        print(f"{s['geom']:<18}{s['arch']:<22}{s['cond'] + ' ' + COND_NAME[s['cond']]:<12}{s['lat']:>4}{s['lut_n'] or '':>3}{s['n']:>4}  "
              f"{s['s_op']:.2f} [{s['lo']:.2f},{s['hi']:.2f}]   {dv:<24}{s['collided']:>5.2f}{s['ghost']:>6.2f}{s['fbrake']:>5.2f}{s['p99_age']:>8.0f}{s['brake_age']:>8.0f}"
              f"{s['in_budget']:>7.2f}{s['coverage']:>6.2f}{s['compute_ms']:>6.0f}{s['ul_ms']:>6.0f}{s['dl_ms']:>6.0f}"
              f"  {s['tp_brake_ms']:>5.0f} [{s['tp_brake_min']:.0f},{s['tp_brake_max']:.0f}]".ljust(0)
              + f"{'':>1}".ljust(max(0, 24 - len(f"  {s['tp_brake_ms']:>5.0f} [{s['tp_brake_min']:.0f},{s['tp_brake_max']:.0f}]")))
              + f"{s['no_tp_brake']:>5.2f}  " + f"{s['min_sep_m']:.1f} [{s['min_sep_min']:.1f},{s['min_sep_max']:.1f}]".ljust(19)
              + f"{s['first_hazard_class'] + ' +' + format(s['first_hazard_ms'], '.0f'):<20}{s['other_brake_first']:>9.2f}")


def figure(summ, path, lut_n=None, lat=None):
    if lat is not None:
        summ = [s for s in summ if s["lat"] == lat]
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
    ap.add_argument("--fig-lat", type=int, default=None, help="added delay (ms) of the cells drawn")
    a = ap.parse_args()
    rows = load(a.stamp, a.root); print(f"{len(rows)} runs under stamp {a.stamp}")
    summ = summarize(rows); print_table(summ)
    if a.csv and summ:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summ[0].keys())); w.writeheader(); w.writerows(summ)
    if a.fig and summ: figure(summ, a.fig, a.fig_n, a.fig_lat)
