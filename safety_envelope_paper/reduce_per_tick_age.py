#!/usr/bin/env python3
"""Reduce per_tick_age.csv logs into the age metrics the SenSys reviews asked for.

Per run (one per_tick_age.csv):
  global_p99_age_ms       p99 of age_at_use_ms over edge-sourced rows
  critical_object_p99_ms  p99 over rows flagged is_conflict_actor
  conflict_time_age_ms    age of the conflict actor's state at the first
                          braking decision (first tick with decision == brake
                          or a brake_event); NaN if the actor was not in the
                          consumed set at that tick
  conflict_time_tick      the tick of that decision
  burst_loss_max          longest run of consecutive edge publish sequence
                          numbers the focal vehicle never consumed
  missed_publish_frac     fraction of edge publishes (between the first and
                          last consumed) that the focal vehicle never consumed
  n_ticks, n_rows, p50_age_ms, p95_age_ms for context

Per configuration (runs grouped by path with the run index removed):
  mean and median of every per-run metric with a bootstrap percentile
  confidence interval over runs (seeds).

Usage
-----
  python safety_envelope_paper/reduce_per_tick_age.py RUN_OR_CAMPAIGN_DIR [...]
         [--out summary.csv] [--json summary.json] [--boot 2000] [--ci 0.95]
         [--focal lowest|all]

A run directory is any directory holding per_tick_age.csv directly or under
evaluation_output/ (the test_runner copies the eCAV output dir there). A
campaign directory is walked recursively. Grouping keys strip "run_<n>" path
segments and "__run_<n>" name parts, so both the test_runner layout
(.../mgr_x/anchoring_on/lat_100_.../run_3/) and the SenSys archive layout
(mgr_x__anchoring_on__lat_100__ego_4__run_3__scenario/) group correctly.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from collections import defaultdict
from typing import Dict, List, Optional

import numpy as np

CSV_NAME = "per_tick_age.csv"
_RUN_SEG = re.compile(r"(^|/)run_\d+(?=/|$)")
_RUN_PART = re.compile(r"__run_\d+")


# ----------------------------------------------------------------- discovery
def find_runs(paths: List[str]) -> List[str]:
    found = []
    for p in paths:
        p = os.path.abspath(p)
        if os.path.isfile(p) and os.path.basename(p) == CSV_NAME:
            found.append(p)
            continue
        for root, _dirs, files in os.walk(p):
            if CSV_NAME in files:
                found.append(os.path.join(root, CSV_NAME))
    return sorted(set(found))


def group_key(csv_path: str) -> str:
    d = os.path.dirname(csv_path)
    if os.path.basename(d) == "evaluation_output":
        d = os.path.dirname(d)
    k = _RUN_SEG.sub(r"\1run", d)
    k = _RUN_PART.sub("__run", k)
    return k


# ----------------------------------------------------------------- per run
def _f(v: str) -> float:
    try:
        return float(v) if v not in ("", None) else float("nan")
    except ValueError:
        return float("nan")


def _p(vals: List[float], q: float) -> float:
    a = np.asarray([v for v in vals if not math.isnan(v)], dtype=float)
    return float(np.percentile(a, q)) if a.size else float("nan")


def reduce_run(csv_path: str, focal: str = "lowest") -> Dict[str, float]:
    rows: List[dict] = []
    with open(csv_path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows.append(r)
    if not rows:
        return {"n_rows": 0}

    vids = sorted({int(r["focal_vehicle_id"]) for r in rows if r["focal_vehicle_id"]})
    if focal == "lowest" and vids:
        rows = [r for r in rows if int(r["focal_vehicle_id"]) == vids[0]]

    edge = [r for r in rows if r["source"] == "edge"]
    ages = [_f(r["age_at_use_ms"]) for r in edge]
    crit = [_f(r["age_at_use_ms"]) for r in edge if r["is_conflict_actor"] == "1"]

    # first braking decision
    by_tick: Dict[int, List[dict]] = defaultdict(list)
    for r in rows:
        by_tick[int(r["tick"])].append(r)
    ticks = sorted(by_tick)
    brake_tick: Optional[int] = None
    for t in ticks:
        rs = by_tick[t]
        if any(r["decision"] == "brake" or r["brake_event"] == "1" for r in rs):
            brake_tick = t
            break
    conflict_age = float("nan")
    if brake_tick is not None:
        cand = [_f(r["age_at_use_ms"]) for r in by_tick[brake_tick]
                if r["source"] == "edge" and r["is_conflict_actor"] == "1"]
        cand = [c for c in cand if not math.isnan(c)]
        if cand:
            conflict_age = max(cand)

    # burst loss over edge publish sequence numbers
    seqs = sorted({int(_f(r["publish_seq"])) for r in edge
                   if r["publish_seq"] not in ("", None) and not math.isnan(_f(r["publish_seq"]))})
    burst_max = 0
    missed = 0
    if len(seqs) >= 2:
        for a, b in zip(seqs, seqs[1:]):
            gap = b - a - 1
            if gap > 0:
                missed += gap
                burst_max = max(burst_max, gap)
        span = seqs[-1] - seqs[0] + 1
        missed_frac = missed / span if span > 0 else float("nan")
    else:
        missed_frac = float("nan")

    return {
        "n_rows": len(rows),
        "n_ticks": len(ticks),
        "p50_age_ms": _p(ages, 50),
        "p95_age_ms": _p(ages, 95),
        "global_p99_age_ms": _p(ages, 99),
        "critical_object_p99_ms": _p(crit, 99),
        "conflict_time_age_ms": conflict_age,
        "conflict_time_tick": float(brake_tick) if brake_tick is not None else float("nan"),
        "burst_loss_max": float(burst_max),
        "missed_publish_frac": missed_frac,
    }


# ----------------------------------------------------------------- groups
METRICS = ["global_p99_age_ms", "critical_object_p99_ms", "conflict_time_age_ms",
           "burst_loss_max", "missed_publish_frac", "p95_age_ms", "p50_age_ms"]


def bootstrap_ci(vals: List[float], n_boot: int, ci: float, seed: int = 0):
    a = np.asarray([v for v in vals if not math.isnan(v)], dtype=float)
    if a.size == 0:
        return float("nan"), float("nan"), float("nan"), 0
    if a.size == 1:
        return float(a[0]), float(a[0]), float(a[0]), 1
    rng = np.random.default_rng(seed)
    means = rng.choice(a, size=(n_boot, a.size), replace=True).mean(axis=1)
    lo = float(np.percentile(means, (1 - ci) / 2 * 100))
    hi = float(np.percentile(means, (1 + ci) / 2 * 100))
    return float(a.mean()), lo, hi, int(a.size)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="run dirs, campaign dirs, or per_tick_age.csv files")
    ap.add_argument("--out", default=None, help="summary CSV (default: print only)")
    ap.add_argument("--json", default=None, help="summary JSON with per-run rows")
    ap.add_argument("--boot", type=int, default=2000, help="bootstrap resamples")
    ap.add_argument("--ci", type=float, default=0.95, help="confidence level")
    ap.add_argument("--focal", choices=["lowest", "all"], default="lowest",
                    help="reduce the lowest-id managed vehicle (the hero) or all")
    args = ap.parse_args(argv)

    runs = find_runs(args.paths)
    if not runs:
        print(f"no {CSV_NAME} found under {args.paths}", file=sys.stderr)
        return 1

    per_run = []
    groups: Dict[str, List[dict]] = defaultdict(list)
    for csv_path in runs:
        m = reduce_run(csv_path, focal=args.focal)
        m["run"] = csv_path
        m["group"] = group_key(csv_path)
        per_run.append(m)
        groups[m["group"]].append(m)

    summary = []
    for g in sorted(groups):
        ms = groups[g]
        row = {"group": g, "n_runs": len(ms)}
        for k in METRICS:
            vals = [m.get(k, float("nan")) for m in ms]
            mean, lo, hi, n = bootstrap_ci(vals, args.boot, args.ci)
            row[f"{k}_mean"] = mean
            row[f"{k}_lo"] = lo
            row[f"{k}_hi"] = hi
            row[f"{k}_median"] = float(np.nanmedian(np.asarray(vals, dtype=float))) if n else float("nan")
        summary.append(row)

    # print
    hdr = ["group", "n_runs", "global_p99_age_ms", "critical_object_p99_ms",
           "conflict_time_age_ms", "burst_loss_max", "missed_publish_frac"]
    print("\t".join(hdr))
    for row in summary:
        cells = [row["group"][-70:], str(row["n_runs"])]
        for k in hdr[2:]:
            cells.append(f"{row[k + '_mean']:.1f} [{row[k + '_lo']:.1f}, {row[k + '_hi']:.1f}]")
        print("\t".join(cells))

    if args.out:
        keys = ["group", "n_runs"] + [f"{k}_{s}" for k in METRICS for s in ("mean", "lo", "hi", "median")]
        with open(args.out, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for row in summary:
                w.writerow({k: row.get(k) for k in keys})
        print(f"wrote {args.out}")
    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"summary": summary, "per_run": per_run,
                       "boot": args.boot, "ci": args.ci, "focal": args.focal}, fh, indent=2,
                      default=lambda v: None if isinstance(v, float) and math.isnan(v) else v)
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
