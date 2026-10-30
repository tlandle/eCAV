#!/usr/bin/env python3
"""Sensitivity of the measured outcome to the post-hoc brake labelling rules.

    python scripts/sensys_analysis/label_sensitivity.py <stamp> [<stamp> ...] [--speed 70]
        [--out safety_envelope_paper/analysis/label_sensitivity_<stamp>]

Reviewer check A (Oct 29): the same runs are reduced under every labelling
rule the campaign has used (the paper's match at the observation's source
tick within 3 m; the earlier trigger-tick matches at 3 m fixed, 6 m fixed and
6 m growing at 12.5 m/s with the delay beyond 200 ms; with and without the
live-state re-judgement; shared tracks only or every brake), and the table
separates what the labeller cannot touch (collisions, whether the planner
braked, when it first braked, completion) from what it classifies (false-brake
and self-ghost runs) and the resulting S_op. The per-run CSV helpers of the
reducer are cached per run directory so each run's logs are read once.
"""
import argparse
import collections
import copy
import csv
import glob
import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import recompute_metrics as rm  # noqa: E402
from campaign_check import GEOM_RE, _log_has_sim_timeout, ROOT  # noqa: E402

VARIANTS = [
    # name, match at source tick, source radius m, trigger-tick radius m, growth m/s, count local, live re-judge
    ("paper: 3 m at the source tick, shared tracks, live re-judge", True, 3.0, 6.0, 12.5, False, True),
    ("2 m at the source tick", True, 2.0, 6.0, 12.5, False, True),
    ("trigger tick, 6 m + 12.5 m/s beyond 200 ms (rule before Oct 29)", False, 3.0, 6.0, 12.5, False, True),
    ("trigger tick, 6 m fixed (rule before Oct 26)", False, 3.0, 6.0, 0.0, False, True),
    ("trigger tick, 3 m fixed (rule before Oct 17)", False, 3.0, 3.0, 0.0, False, True),
    ("paper, no live re-judge", True, 3.0, 6.0, 12.5, False, False),
    ("paper, local-track brakes counted", True, 3.0, 6.0, 12.5, True, True),
    ("trigger tick 3 m, local counted, no re-judge", False, 3.0, 3.0, 0.0, True, False),
]


def _cached(fn):
    cache = {}

    def wrap(d):
        k = d.get("_run_dir") if isinstance(d, dict) else d
        if k not in cache:
            cache[k] = fn(d)
        return cache[k]
    return wrap


rm._conflict_live_states = _cached(rm._conflict_live_states)
rm._ego_headings = _cached(rm._ego_headings)
rm._conflict_actor_id = _cached(rm._conflict_actor_id)


def load(root, stamp, speed):
    runs = []
    for f in glob.glob(os.path.join(root, "*", stamp, "**", "simulation_metrics.json"), recursive=True):
        if "/evaluation_output/" in f:
            continue
        m = GEOM_RE.search(f)
        if not m:
            continue
        geom, spd, mgr, anch, cfg, rep = m.groups()
        if int(spd) != speed:
            continue
        try:
            d = json.load(open(f))
        except (OSError, ValueError):
            continue
        if "error" in d or not d.get("vehicles"):
            continue
        if _log_has_sim_timeout(os.path.join(os.path.dirname(f), "runner_stdout.txt")):
            continue
        if int(d.get("config_ego_count", 1) or 1) != 1:
            continue
        d["_run_dir"] = os.path.dirname(f)
        runs.append((stamp, geom, mgr, anch, int(d.get("config_latency_ms", 0) or 0), int(rep), d))
    return runs


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stamps", nargs="+")
    ap.add_argument("--root", default=os.path.join(ROOT, "experiment_results"))
    ap.add_argument("--speed", type=int, default=70)
    ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    out = a.out or os.path.join(ROOT, "safety_envelope_paper", "analysis", "label_sensitivity_" + a.stamps[0])
    os.makedirs(out, exist_ok=True)
    runs = []
    for s in a.stamps:
        runs += load(a.root, s, a.speed)
    print(f"{len(runs)} runs")
    cells = collections.defaultdict(list)
    rows = []
    for stamp, geom, mgr, anch, lat, rep, d in runs:
        fv, _ = rm.focal_ego(d)
        ba = fv.get("brake_attributions", [])
        shared = [x for x in ba if not rm._is_local_track(x)]
        base = dict(stamp=stamp, geom=geom, mgr=mgr, anch=anch, lat=lat, rep=rep,
                    braked_any=int(bool(ba)), braked_shared=int(bool(shared)),
                    first_shared_brake_tick=min((x.get("trigger_tick") for x in shared if x.get("trigger_tick") is not None), default=None))
        for vi, (name, at_src, rs_m, r_m, g_mps, local, rejudge) in enumerate(VARIANTS):
            rm.CONFLICT_MATCH_AT_SOURCE_TICK = at_src
            rm.CONFLICT_SOURCE_MATCH_RADIUS_M = rs_m
            rm.CONFLICT_MATCH_RADIUS_M = r_m
            rm.CONFLICT_MATCH_GROWTH_MPS = g_mps
            rm.COUNT_LOCAL_TRACK_BRAKES = local
            rm.LIVE_HAZARD_REJUDGE = rejudge
            r = rm.compute_run_metrics(copy.deepcopy(d))
            if vi == 0:
                base.update(collided=int(r.get("focal_collisions", 0) > 0), s_clear=int(r.get("s_clear", 1)),
                            s_prog=int(r.get("s_prog", 1)))
            base[f"v{vi}_fp"] = int(r.get("focal_fp_episodes", 0) > 0)
            base[f"v{vi}_ghost"] = int(r.get("focal_ghost_episodes", 0) > 0)
            base[f"v{vi}_sop"] = int(bool(r.get("s_op", 0)))
        rows.append(base)
        cells[(stamp, geom, mgr, anch, lat)].append(base)
    rm.CONFLICT_MATCH_AT_SOURCE_TICK, rm.CONFLICT_SOURCE_MATCH_RADIUS_M = True, 3.0
    rm.CONFLICT_MATCH_RADIUS_M, rm.CONFLICT_MATCH_GROWTH_MPS = 6.0, 12.5
    rm.COUNT_LOCAL_TRACK_BRAKES, rm.LIVE_HAZARD_REJUDGE = False, True
    with open(os.path.join(out, "runs.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    lines = ["# Labelling-rule sensitivity", "",
             "Same runs under every rule. Columns the labeller cannot change: n, collided, braked (any shared brake), "
             "first brake tick (median), incomplete (s_clear or s_prog 0). Per rule: runs with a false-brake episode / "
             "with a self-ghost episode / S_op.", ""]
    for vi, v in enumerate(VARIANTS):
        lines.append(f"- v{vi}: {v[0]}")
    lines.append("")
    hdr = "| stamp | geom | arm | lat | n | coll | braked | brake tick | incomplete | " + " | ".join(f"v{i} fp/ghost/S_op" for i in range(len(VARIANTS))) + " |"
    lines += [hdr, "|" + "---|" * (9 + len(VARIANTS))]
    for key in sorted(cells):
        rs = cells[key]
        stamp, geom, mgr, anch, lat = key
        ticks = [r["first_shared_brake_tick"] for r in rs if r["first_shared_brake_tick"] is not None]
        med = int(statistics.median(ticks)) if ticks else "-"
        inc = sum(1 for r in rs if not (r["s_clear"] and r["s_prog"]))
        cols = [f"{sum(r[f'v{i}_fp'] for r in rs)}/{sum(r[f'v{i}_ghost'] for r in rs)}/{sum(r[f'v{i}_sop'] for r in rs)}" for i in range(len(VARIANTS))]
        lines.append(f"| {stamp} | {geom} | {mgr}/{anch} | {lat} | {len(rs)} | {sum(r['collided'] for r in rs)} | "
                     f"{sum(r['braked_shared'] for r in rs)} | {med} | {inc} | " + " | ".join(cols) + " |")
    open(os.path.join(out, "label_sensitivity.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
