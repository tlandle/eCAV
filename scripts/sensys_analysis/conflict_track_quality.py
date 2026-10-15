#!/usr/bin/env python3
"""Per-run tracking quality of the conflict actor from per_tick_age.csv:
coverage, longest gap, distinct track ids, mean published speed over fresh
rows, decision-window p99 age. One line per run, one summary per cell.

    python scripts/sensys_analysis/conflict_track_quality.py <stamp> [<stamp> ...]
"""
import csv
import glob
import os
import statistics
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "safety_envelope_paper"))
from reduce_per_tick_age import reduce_run  # noqa: E402


def run_quality(p):
    m = reduce_run(p)
    ids, speeds = set(), []
    with open(p, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("is_conflict_actor") != "1" or r.get("source") != "edge":
                continue
            ids.add(r.get("track_id"))
            try:
                if r.get("obs_tick") and int(float(r["obs_tick"])) == int(float(r["gen_tick"])):
                    speeds.append(float(r["obs_speed_mps"]))
            except ValueError:
                pass
    return dict(coverage=m.get("conflict_coverage", float("nan")), gap=m.get("conflict_max_gap_ticks", float("nan")),
                ids=len(ids), speed=statistics.mean(speeds) if speeds else float("nan"),
                p99=m.get("critical_object_p99_ms", float("nan")))


def main(stamps):
    for st in stamps:
        cells = {}
        for p in sorted(glob.glob(os.path.join(ROOT, "experiment_results", "*", st, "speed_*", "*", "mgr_*", "anchoring_*", "*", "run_*", "evaluation_output", "per_tick_age.csv"))):
            parts = p.split(os.sep)
            cell = "/".join(x for x in parts if x.startswith(("mgr_", "anchoring_", "lat_")))
            cells.setdefault(cell, []).append(run_quality(p))
        print(f"== {st}")
        for cell, qs in sorted(cells.items()):
            f = lambda k: [q[k] for q in qs if q[k] == q[k]]
            print(f"  {cell.split('/lat_')[0]} lat_{cell.split('/lat_')[1].split('_')[0]}: n={len(qs)} coverage {statistics.mean(f('coverage')):.2f} "
                  f"max_gap {statistics.mean(f('gap')):.1f} ids {statistics.mean(f('ids')):.1f} speed {statistics.mean(f('speed')):.1f} m/s p99 {statistics.median(f('p99')):.0f} ms")


if __name__ == "__main__":
    main(sys.argv[1:])
