#!/usr/bin/env python
"""Closed-loop service occupancy, on against off (ISPASS execution model).

    python scripts/ispass/occupancy_compare.py <stamp> [--root experiment_results]

One row per run: invocations and delivered update rate (invocations over the simulated
time the edge had frames), superseded fraction, measured compute, age at use of every
edge track row and of the conflict actor (p50/p95, ms), operational outcome.
"""
import argparse, csv, glob, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "scripts", "sensys_analysis"))
from campaign_check import load_runs  # noqa: E402


def pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stamp")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    a = ap.parse_args()
    rows = []
    for r in load_runs(a.root, a.stamp):
        if r.get("bad"):
            print("void:", r["path"], r["bad"]); continue
        d = os.path.dirname(r["path"])
        m = json.load(open(r["path"]))
        kv = m.get("config_edge_kv", {})
        occ = str(kv.get("service_occupancy", "off"))
        edge = next(iter(m.get("edges", {}).values()), {})
        comp = edge.get("composed_delay", {})
        so = edge.get("service_occupancy", {})
        ages, cages, ticks, gens = [], [], set(), set()
        with open(os.path.join(d, "evaluation_output", "per_tick_age.csv")) as fh:
            for x in csv.DictReader(fh):
                ticks.add(int(x["tick"]))
                if x.get("source") == "edge" and x.get("age_at_use_ms") not in ("", None):
                    ages.append(float(x["age_at_use_ms"]))
                    if x.get("gen_tick") not in ("", None):
                        gens.add(int(float(x["gen_tick"])))
                    if x.get("is_conflict_actor") == "1":
                        cages.append(float(x["age_at_use_ms"]))
        sim_s = (max(ticks) + 1) * 0.05 if ticks else float("nan")
        n_inv = int(comp.get("n", 0))
        span_s = (max(gens) - min(gens) + 1) * 0.05 if gens else float("nan")
        rows.append(dict(
            occ=occ, rep=r["rep"], sim_s=sim_s, n_inv=n_inv, inv_hz=n_inv / sim_s,
            fresh_hz=len(gens) / span_s if gens else float("nan"),
            sup=so.get("superseded_fraction", float("nan")),
            c_mean=comp.get("compute_ms_mean", float("nan")), c_p95=comp.get("compute_ms_p95", float("nan")),
            a50=pct(ages, 50), a95=pct(ages, 95), ca50=pct(cages, 50), ca95=pct(cages, 95),
            s_op=r.get("s_op"), coll=r.get("focal_collisions"), ghost=r.get("focal_ghost_episodes"), fp=r.get("focal_fp_episodes"), stuck=r.get("focal_stuck")))
    rows.sort(key=lambda x: (x["occ"], x["rep"]))
    print("occ  rep  sim_s  invocations  inv_Hz  fresh_Hz  superseded  compute mean/p95  age p50/p95  conflict age p50/p95  s_op coll ghost fp stuck")
    for x in rows:
        print(f"{x['occ']:<4} {x['rep']:>3} {x['sim_s']:6.1f} {x['n_inv']:>11} {x['inv_hz']:7.1f} {x['fresh_hz']:9.1f} "
              f"{x['sup']:10.2f}   {x['c_mean']:6.0f}/{x['c_p95']:<6.0f}   {x['a50']:4.0f}/{x['a95']:<4.0f}      "
              f"{x['ca50']:4.0f}/{x['ca95']:<4.0f}          {x['s_op']}    {x['coll']}    {x['ghost']}    {x['fp']}  {x['stuck']}")


if __name__ == "__main__":
    main()
