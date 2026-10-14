#!/usr/bin/env python3
"""Gate a SenSys campaign stamp against what the data is expected to show.

    python scripts/sensys_analysis/campaign_check.py <stamp> [--root experiment_results]
        [--april safety_envelope_paper/experiment_results/20260419_120000]
        [--stage2 N] [--threshold 0.8]

Reads every run_*/simulation_metrics.json under experiment_results/*/<stamp>/,
reduces each run with scripts/recompute_metrics.compute_run_metrics, and prints:

  1. integrity gates per run set (error-path files, short runs, missing focal,
     per-tick CSV present when the run asked for it, edge frame time);
  2. per-cell P(S_op) with Wilson 95% intervals, ghost and false-brake episode
     counts, collisions, S_prog failures, mean focal speed;
  3. shape gates that every arm must satisfy before the campaign continues
     (listed in GATES below); FAIL stops the launch, WARN is reported;
  4. the April 20260419_120000 cells for the same (manager, anchoring, N,
     latency) next to the new cells, for scenario_3, so a drift in the
     physics or logic cliff is visible;
  5. cells whose interval straddles --threshold after the seeds run so far,
     with the test_runner invocation that adds --stage2 more seeds to each.

Exit status 1 when a FAIL gate fires, so a driver can stop on it.
"""
import argparse
import collections
import csv
import glob
import json
import math
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
from recompute_metrics import compute_run_metrics  # noqa: E402

GEOM_RE = re.compile(r"experiment_results/([^/]+)/[^/]+/speed_(?:spd_)?(\d+|orig)/[^/]+/mgr_([^/]+)/anchoring_([^/]+)/([^/]+)/run_(\d+)")


def wilson(k, n, z=1.96):
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return p, c - h, c + h


def load_runs(root, stamp):
    runs = []
    for f in glob.glob(os.path.join(root, "*", stamp, "**", "simulation_metrics.json"), recursive=True):
        if "/evaluation_output/" in f:
            continue
        m = GEOM_RE.search(f)
        if not m:
            continue
        geom, spd, mgr, anch, cfg, rep = m.groups()
        try:
            d = json.load(open(f))
        except (OSError, ValueError):
            runs.append(dict(path=f, geom=geom, bad="unreadable"))
            continue
        row = dict(path=f, geom=geom, speed=spd, mgr=mgr, anch=anch, cfg=cfg, rep=int(rep))
        if "error" in d or not d.get("vehicles"):
            row["bad"] = "error-path metrics"
            runs.append(row)
            continue
        r = compute_run_metrics(d)
        if r is None:
            row["bad"] = "no focal vehicle"
            runs.append(row)
            continue
        e = list(d.get("edges", {}).values())
        e = e[0] if e else {}
        row.update(r)
        row["frames"] = e.get("total_frames", 0)
        row["edge_ms"] = e.get("timing_avg_ms", float("nan"))
        row["N"] = d.get("config_ego_count", 1)
        row["lat"] = int(d.get("config_latency_ms", 0) or 0)
        row["kv"] = json.dumps(d.get("config_edge_kv", {}), sort_keys=True) if d.get("config_edge_kv") else ""
        row["dist"] = r.get("focal_dist_m", float("nan"))
        row["per_tick_wanted"] = bool(d.get("config_per_tick_age_log"))
        rd = os.path.dirname(f)
        row["per_tick_present"] = os.path.exists(os.path.join(rd, "per_tick_age.csv")) or \
            os.path.exists(os.path.join(rd, "evaluation_output", "per_tick_age.csv"))
        row["age_p50"], row["age_p99"] = per_tick_ages(rd)
        runs.append(row)
    return runs


def per_tick_ages(rd):
    for p in (os.path.join(rd, "per_tick_age.csv"), os.path.join(rd, "evaluation_output", "per_tick_age.csv")):
        if os.path.exists(p):
            ages = []
            with open(p) as fh:
                for r in csv.DictReader(fh):
                    if r.get("source") == "edge" and r.get("age_at_use_ms") not in ("", None):
                        try:
                            ages.append(float(r["age_at_use_ms"]))
                        except ValueError:
                            pass
            if ages:
                ages.sort()
                return ages[len(ages) // 2], ages[min(len(ages) - 1, int(0.99 * len(ages)))]
    return float("nan"), float("nan")


def cells_of(runs):
    cells = collections.defaultdict(list)
    for r in runs:
        if r.get("bad"):
            continue
        cells[(r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["lat"], r["kv"])].append(r)
    return cells


def april_cells(path):
    out = collections.defaultdict(list)
    if not path or not os.path.isdir(path):
        return out
    for f in glob.glob(os.path.join(path, "*", "simulation_metrics.json")):
        d = json.load(open(f))
        r = compute_run_metrics(d)
        if r is None:
            continue
        out[(d.get("config_manager_type"), "on" if d.get("config_anchoring") else "off",
             d.get("config_ego_count", 1), int(d.get("config_latency_ms", 0) or 0),
             d.get("config_speed_kmh"))].append(r["s_op"])
    return out


# (name, severity, predicate over cells dict, message)
def shape_gates(cells, threshold):
    out = []

    def p_of(geom, spd, mgr, anch, N, lat, kv=""):
        v = cells.get((geom, spd, mgr, anch, N, lat, kv))
        if not v:
            return None
        return wilson(sum(r["s_op"] for r in v), len(v))

    keys = sorted(set((k[0], k[1], k[4], k[6]) for k in cells))
    for geom, spd, N, kv in keys:
        arms = sorted(set((k[2], k[3]) for k in cells if k[0] == geom and k[1] == spd and k[4] == N and k[6] == kv))
        lats = sorted(set(k[5] for k in cells if k[0] == geom and k[1] == spd and k[4] == N and k[6] == kv))
        tag = f"{geom} {spd} km/h N={N}{' ' + kv if kv else ''}"
        for mgr, anch in arms:
            # G1: at zero latency every arm succeeds (no conflict reaches the planner late)
            p0 = p_of(geom, spd, mgr, anch, N, 0, kv)
            if p0 is not None and p0[2] < 0.8:
                out.append(("FAIL", f"G1 {tag} {mgr}/{anch}: P(S_op) at 0 ms is {p0[0]:.2f} [{p0[1]:.2f},{p0[2]:.2f}], expected near 1"))
            # G2: the contract removes self-ghost episodes
            if anch == "on" or mgr == "oracle":
                v = cells.get((geom, spd, mgr, anch, N, 0, kv), [])
                g = sum(r["focal_ghost_episodes"] for r in v)
                if g > 0:
                    out.append(("FAIL", f"G2 {tag} {mgr}/{anch}: {g} self-ghost episodes at 0 ms with the contract on"))
            # G3: P(S_op) does not rise with latency by more than noise
            ps = [(lat, p_of(geom, spd, mgr, anch, N, lat, kv)) for lat in lats]
            ps = [(l, p) for l, p in ps if p is not None]
            for (l1, p1), (l2, p2) in zip(ps, ps[1:]):
                if p2[1] > p1[2] + 0.05:
                    out.append(("WARN", f"G3 {tag} {mgr}/{anch}: P rises from {p1[0]:.2f} at {l1} ms to {p2[0]:.2f} at {l2} ms"))
        # G4: naive V2X2V fails before the contract arm at the same latency somewhere in the sweep
        naive = [(lat, p_of(geom, spd, "late_fusion", "off", N, lat, kv)) for lat in lats]
        sba = [(lat, p_of(geom, spd, "late_fusion", "on", N, lat, kv)) for lat in lats]
        pairs = [(l, a, b) for (l, a), (_, b) in zip(naive, sba) if a is not None and b is not None]
        if pairs and not any(b[1] > a[2] for _, a, b in pairs):
            out.append(("WARN", f"G4 {tag}: no latency where SBA beats naive outside the intervals (logic cliff not visible yet)"))
        # G5: oracle holds to at least 200 ms
        po = p_of(geom, spd, "oracle", "on", N, 200, kv)
        if po is not None and po[2] < threshold:
            out.append(("FAIL", f"G5 {tag}: oracle at 200 ms is {po[0]:.2f} [{po[1]:.2f},{po[2]:.2f}], the physics boundary should be past 300 ms"))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamp")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--april", default=os.path.join(ROOT, "safety_envelope_paper", "experiment_results", "20260419_120000"))
    ap.add_argument("--threshold", type=float, default=0.8)
    ap.add_argument("--stage2", type=int, default=10, help="seeds to add to cells whose interval straddles the threshold")
    ap.add_argument("--max-edge-ms", type=float, default=80.0, help="edge frame time above which a run set is flagged")
    a = ap.parse_args(argv)

    runs = load_runs(a.root, a.stamp)
    if not runs:
        print(f"no runs under {a.root}/*/{a.stamp}")
        return 2
    fails = 0
    bad = [r for r in runs if r.get("bad")]
    good = [r for r in runs if not r.get("bad")]
    print(f"== {a.stamp}: {len(good)} runs reduced, {len(bad)} unusable")
    for r in bad[:10]:
        print(f"   unusable ({r['bad']}): {r['path']}")

    # 1. integrity
    short = [r for r in good if r["frames"] < 100]
    if short:
        print(f"FAIL integrity: {len(short)} runs under 100 edge frames (e.g. {short[0]['path']})")
        fails += 1
    missing_pt = [r for r in good if r["per_tick_wanted"] and not r["per_tick_present"]]
    if missing_pt:
        print(f"FAIL integrity: {len(missing_pt)} runs asked for the per-tick log and have none (e.g. {missing_pt[0]['path']})")
        fails += 1
    by_set = collections.defaultdict(list)
    for r in good:
        by_set[(r["geom"], r["mgr"], r["N"])].append(r["edge_ms"])
    for k, v in sorted(by_set.items()):
        v = [x for x in v if x == x]
        if v:
            med = sorted(v)[len(v) // 2]
            flag = "WARN" if med > a.max_edge_ms else "ok  "
            print(f"{flag} edge frame time {k[0]} {k[1]} N={k[2]}: median {med:.0f} ms over {len(v)} runs (April: 25 to 37 ms)")

    # 2. cells
    cells = cells_of(good)
    apr = april_cells(a.april)
    print("\n== cells: P(S_op) [Wilson 95%] | ghost eps | fp eps | collisions | S_prog fails | mean speed | age p50/p99 ms | April P (same mgr/anch/N/lat/speed)")
    stage2 = []
    for k in sorted(cells):
        v = cells[k]
        n = len(v)
        p, lo, hi = wilson(sum(r["s_op"] for r in v), n)
        g = sum(r["focal_ghost_episodes"] for r in v)
        fp = sum(r["focal_fp_episodes"] for r in v)
        c = sum(1 for r in v if r["focal_collisions"] > 0)
        sp = sum(1 for r in v if r.get("s_prog", 1) == 0)
        spd = sum(r.get("focal_avg_speed_mps", 0) for r in v) / n
        ages = [r["age_p50"] for r in v if r["age_p50"] == r["age_p50"]]
        a50 = sorted(ages)[len(ages) // 2] if ages else float("nan")
        a99s = [r["age_p99"] for r in v if r["age_p99"] == r["age_p99"]]
        a99 = sorted(a99s)[len(a99s) // 2] if a99s else float("nan")
        ak = (k[2], k[3], k[4], k[5], int(k[1]) if k[1].isdigit() else None)
        apr_s = ""
        if ak in apr and k[0].startswith("openscenario_3"):
            ap_, al, ah = wilson(sum(apr[ak]), len(apr[ak]))
            apr_s = f"{ap_:.2f} [{al:.2f},{ah:.2f}] n={len(apr[ak])}"
        print(f"{k[0]:<34} {k[1]:>3} {k[2]:<36} {k[3]:<4} N={k[4]:<2} {k[5]:>4} ms {k[6]:<28} n={n:<3} P={p:.2f} [{lo:.2f},{hi:.2f}] | {g:>3} | {fp:>3} | {c:>2} | {sp:>2} | {spd:4.1f} | {a50:5.0f}/{a99:5.0f} | {apr_s}")
        if lo < a.threshold < hi:
            stage2.append((k, n))

    # 3. shape gates
    print("\n== shape gates")
    gates = shape_gates(cells, a.threshold)
    if not gates:
        print("all shape gates pass")
    for sev, msg in gates:
        print(f"{sev} {msg}")
        if sev == "FAIL":
            fails += 1

    # 5. stage-2 cells
    if stage2:
        print(f"\n== {len(stage2)} cells straddle P={a.threshold}; add {a.stage2} seeds each (resume with the same stamp):")
        for (geom, spd, mgr, anch, N, lat, kv), n in stage2:
            kvs = f" --edge-kv {' '.join(f'{x}={y}' for x, y in json.loads(kv).items())}" if kv else ""
            ego = f" --ego-counts {N}" if N and N > 1 else ""
            print(f"python test_runner.py -t {geom} --manager-types {mgr} --anchoring {anch} --latencies {lat} --speeds {spd}{ego}"
                  f" --repetitions {n + a.stage2} --stamp {a.stamp} --latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log{kvs}")
    print(f"\n== result: {'FAIL' if fails else 'PASS'} ({fails} failing gates)")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
