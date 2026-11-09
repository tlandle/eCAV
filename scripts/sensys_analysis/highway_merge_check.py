#!/usr/bin/env python3
"""Pre-launch gate and outcome table for the highway merge (cut-in) runs.

    python scripts/sensys_analysis/highway_merge_check.py <stamp> [--root experiment_results]

Per (closing speed, arm, configured delay, uplink loss): runs, operational
success, collisions, self-ghost episodes, false brakes, stalls, mean frames,
the measured conflict-actor p99 age, and the cut-in tick / gap / ego speed the
scenario printed. The gate (Tyler, Oct 30: every arm per cell at zero age with
margins, one stale probe) passes for a closing speed when at zero configured
delay the exact-state baseline and V2X2V +SBA complete every run, no
cooperation (uplink loss 100 %) collides in every run, and the exact-state
baseline fails at least one run at 1,000 ms (the cut-in reaches the decision).
"""
import argparse
import collections
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import load_runs, ROOT  # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
try:
    from reduce_per_tick_age import reduce_run  # noqa: E402
except Exception:  # pragma: no cover
    reduce_run = None

CUTIN_RE = re.compile(r"\[HIGHWAY_MERGE\] cut-in at tick (-?\d+) x=([\d.]+): bumper gap ([\d.]+) m "
                      r"\(design ([\d.]+)\), ego ([\d.]+) m/s, intruder ([\d.]+) m/s")


def cutin_info(run_dir):
    p = os.path.join(run_dir, "runner_stdout.txt")
    if not os.path.exists(p):
        return None
    try:
        for line in open(p, errors="replace"):
            m = CUTIN_RE.search(line)
            if m:
                return dict(tick=int(m.group(1)), x=float(m.group(2)), gap=float(m.group(3)),
                            design=float(m.group(4)), ego=float(m.group(5)), npc=float(m.group(6)))
    except OSError:
        return None
    return None


def p99_age(run_dir):
    if reduce_run is None:
        return float("nan")
    for p in (os.path.join(run_dir, "per_tick_age.csv"), os.path.join(run_dir, "evaluation_output", "per_tick_age.csv")):
        if os.path.exists(p):
            m = reduce_run(p)
            v = m.get("critical_object_p99_ms", float("nan"))
            return v if v == v else m.get("global_p99_age_ms", float("nan"))
    return float("nan")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamp")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    a = ap.parse_args(argv)
    runs = load_runs(a.root, a.stamp)
    bad = [r for r in runs if r.get("bad")]
    runs = [r for r in runs if not r.get("bad")]
    print(f"{len(runs)} runs ({len(bad)} void: {collections.Counter(r['bad'] for r in bad)})")
    cells = collections.defaultdict(list)
    for r in runs:
        kv = json.loads(r["kv"]) if r.get("kv") else {}
        cs = float(kv.get("closing_speed_mps", float("nan")))
        d = json.load(open(r["path"]))
        loss = float(d.get("config_uplink_packet_loss_pct", d.get("config_packet_loss_pct", 0)) or 0)
        arm = {("oracle", "on"): "exact-state", ("late_fusion", "on"): "V2X2V +SBA",
               ("late_fusion", "off"): "V2X2V no SBA"}.get((r["mgr"], r["anch"]), f"{r['mgr']}/{r['anch']}")
        if loss >= 100:
            arm = "no cooperation"
        rd = os.path.dirname(r["path"])
        r["_cutin"] = cutin_info(rd)
        r["_p99"] = p99_age(rd)
        cells[(cs, arm, r["lat"])].append(r)
    print(f"{'cs':>4} {'arm':<14} {'lat':>5} {'n':>3} {'s_op':>5} {'coll':>4} {'ghost':>5} {'fp':>3} {'stall':>5} "
          f"{'frames':>6} {'p99age':>7} {'cutin tick':>10} {'gap':>5} {'ego':>5} {'npc':>5}")
    gate = collections.defaultdict(dict)
    for (cs, arm, lat), rs in sorted(cells.items(), key=lambda kv: (kv[0][0], kv[0][1], kv[0][2])):
        n = len(rs)
        s = sum(int(bool(r["s_op"])) for r in rs)
        coll = sum(1 for r in rs if r.get("focal_collisions", 0) > 0)
        ghost = sum(1 for r in rs if r.get("focal_ghost_episodes", 0) > 0)
        fp = sum(1 for r in rs if r.get("focal_fp_episodes", 0) > 0)
        stall = sum(1 for r in rs if r.get("s_prog", 1) == 0)
        frames = sum(r.get("frames", 0) for r in rs) / n
        ages = [r["_p99"] for r in rs if r["_p99"] == r["_p99"]]
        age = sum(ages) / len(ages) if ages else float("nan")
        ci = [r["_cutin"] for r in rs if r["_cutin"]]
        tick = sum(c["tick"] for c in ci) / len(ci) if ci else float("nan")
        gap = sum(c["gap"] for c in ci) / len(ci) if ci else float("nan")
        ego = sum(c["ego"] for c in ci) / len(ci) if ci else float("nan")
        npc = sum(c["npc"] for c in ci) / len(ci) if ci else float("nan")
        print(f"{cs:4.0f} {arm:<14} {lat:5d} {n:3d} {s/n:5.2f} {coll:4d} {ghost:5d} {fp:3d} {stall:5d} "
              f"{frames:6.0f} {age:7.0f} {tick:10.0f} {gap:5.1f} {ego:5.1f} {npc:5.1f}"
              + ("" if len(ci) == n else f"   (cut-in logged in {len(ci)} of {n})"))
        gate[cs][(arm, lat)] = (s, coll, n)
    print("\ngate per closing speed:")
    ok_all = True
    for cs in sorted(gate):
        g = gate[cs]
        def full(key):  # every run succeeded
            v = g.get(key); return v is not None and v[0] == v[2]
        def all_collide(key):
            v = g.get(key); return v is not None and v[1] == v[2]
        def some_fail(key):
            v = g.get(key); return v is not None and v[0] < v[2]
        checks = [("exact-state completes at 0", full(("exact-state", 0))),
                  ("V2X2V +SBA completes at 0", full(("V2X2V +SBA", 0))),
                  ("no cooperation collides at 0", all_collide(("no cooperation", 0))),
                  ("exact-state fails at 1000", some_fail(("exact-state", 1000)))]
        ok = all(c[1] for c in checks)
        ok_all &= ok
        print(f"  cs {cs:4.0f}: {'PASS' if ok else 'FAIL'}  " + ", ".join(f"{name}: {'yes' if v else 'NO'}" for name, v in checks))
    return 0 if ok_all else 3


if __name__ == "__main__":
    sys.exit(main())
