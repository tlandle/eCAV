#!/usr/bin/env python
"""Reduce the closed-loop part of ISPASS experiment 4 (radio_fidelity_closed_loop.sh).

    python scripts/ispass/radio_fidelity_cl_reduce.py <stamp> [--json out.json]

One row per (radio model, sender count): runs, operational success with a Wilson 95 %
interval, the paired difference to the table model over shared repetitions (bootstrap
95 %), run-level p99 age at use of the conflict actor, age at the braking decision,
coverage, the fraction of updates within the useful-age budget, the composed delay
terms, the messages each direction dropped, and the wall-clock time per run.
"""
import argparse, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from fidelity_reduce import load_runs, reduce_run, within_budget, wilson, nm, BUDGET_MS  # noqa: E402

ORDER = ["table", "table_loss", "live"]


def model_of(kv):
    if str(kv.get("radio", "table")).lower() == "live": return "live"
    return "table_loss" if str(kv.get("lut_loss", "false")).lower() in ("true", "1") else "table"


def load(stamp, root):
    rows = []
    for r in load_runs(root, stamp):
        if r.get("bad"): continue
        m = json.load(open(r["path"])); kv = m.get("config_edge_kv", {}) or {}
        run_dir = os.path.dirname(r["path"]); pt = os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")
        age = reduce_run(pt) if os.path.exists(pt) else {}
        e = (list((m.get("edges") or {}).values()) or [{}])[0]
        comp = e.get("composed_delay") or {}; rad = e.get("radio_model") or {}; live = rad.get("live") or {}
        rows.append(dict(
            model=model_of(kv), n=int(kv.get("lut_n", 0) or 0), rep=int(r["rep"]), s_op=int(r["s_op"]),
            collided=int(r.get("focal_collisions", 0) > 0), fbrake=int(r.get("focal_fp_episodes", 0) > 0),
            p99=age.get("critical_object_p99_ms", float("nan")), brake_age=age.get("conflict_brake_age_ms", float("nan")),
            coverage=age.get("conflict_coverage", float("nan")),
            in_budget=within_budget(pt, BUDGET_MS["left turn"]) if os.path.exists(pt) else float("nan"),
            compute=comp.get("compute_ms_mean", float("nan")), ul=comp.get("ul_ms_mean", float("nan")),
            dl=comp.get("dl_ms_mean", float("nan")), dl_p95=comp.get("dl_ms_p95", float("nan")),
            ul_drop=(rad.get("ul", 0) / rad["ul_sent"]) if rad.get("ul_sent") else 0.0,
            dl_drop=(rad.get("dl", 0) / rad["dl_sent"]) if rad.get("dl_sent") else 0.0,
            ns3_ms_per_step=live.get("wall_ms_per_step", float("nan")), ns3_startup_s=live.get("startup_s", float("nan")),
            wall_s=float(m.get("wall_time_s", m.get("wall_clock_s", float("nan"))) or float("nan"))))
    return rows


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stamp")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results")); ap.add_argument("--json", default=None)
    a = ap.parse_args()
    rows = load(a.stamp, a.root); rng = np.random.default_rng(0); out = []
    for n in sorted({r["n"] for r in rows}):
        base = {r["rep"]: r for r in rows if r["n"] == n and r["model"] == "table"}
        for mdl in ORDER:
            cell = [r for r in rows if r["n"] == n and r["model"] == mdl]
            if not cell: continue
            k = sum(r["s_op"] for r in cell); lo, hi = wilson(k, len(cell))
            d = np.array([r["s_op"] - base[r["rep"]]["s_op"] for r in cell if r["rep"] in base], float)
            da = np.array([r["p99"] - base[r["rep"]]["p99"] for r in cell if r["rep"] in base], float)
            ci = (lambda x: [float(v) for v in np.percentile([x[rng.integers(0, len(x), len(x))].mean() for _ in range(2000)], [2.5, 97.5])]) \
                if len(d) and mdl != "table" else (lambda x: [float("nan")] * 2)
            o = dict(model=mdl, n=n, runs=len(cell), s_op=k / len(cell), lo=lo, hi=hi,
                     d_s_op=float(d.mean()) if len(d) and mdl != "table" else float("nan"), d_s_op_ci=ci(d),
                     d_p99=float(np.nanmean(da)) if len(da) and mdl != "table" else float("nan"), d_p99_ci=ci(da[~np.isnan(da)]) if len(da) else [float("nan")] * 2)
            for key in ("collided", "fbrake", "p99", "brake_age", "coverage", "in_budget", "compute", "ul", "dl", "dl_p95",
                        "ul_drop", "dl_drop", "ns3_ms_per_step", "ns3_startup_s", "wall_s"):
                o[key] = nm([r[key] for r in cell])
            out.append(o)
    print(f"{'N':>3} {'model':<11}{'runs':>5}  {'S_op [95% CI]':<19}{'vs table':<21}{'coll':>5}{'fbrk':>5}{'p99 age':>8}{'d p99':>7}{'brk age':>8}"
          f"{'cov':>6}{'in bud':>7}{'comp':>6}{'ul':>5}{'dl':>6}{'dl p95':>7}{'ul drop':>8}{'dl drop':>8}{'ns3 ms/step':>12}")
    for o in out:
        dv = "" if o["d_s_op"] != o["d_s_op"] else f"{o['d_s_op']:+.2f} [{o['d_s_op_ci'][0]:+.2f},{o['d_s_op_ci'][1]:+.2f}]"
        print(f"{o['n']:>3} {o['model']:<11}{o['runs']:>5}  {o['s_op']:.2f} [{o['lo']:.2f},{o['hi']:.2f}]  {dv:<21}{o['collided']:>5.2f}{o['fbrake']:>5.2f}"
              f"{o['p99']:>8.0f}{o['d_p99']:>7.0f}{o['brake_age']:>8.0f}{o['coverage']:>6.2f}{o['in_budget']:>7.2f}{o['compute']:>6.0f}{o['ul']:>5.0f}"
              f"{o['dl']:>6.0f}{o['dl_p95']:>7.0f}{o['ul_drop']:>8.2f}{o['dl_drop']:>8.2f}{o['ns3_ms_per_step']:>12.0f}")
    if a.json:
        json.dump({"stamp": a.stamp, "rows": out}, open(a.json, "w"), indent=1); print("json:", a.json)


if __name__ == "__main__":
    main()
