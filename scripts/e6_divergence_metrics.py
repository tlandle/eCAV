#!/usr/bin/env python3
"""E6 divergence metrics: parse e6_sweep run logs into a per-cell CSV.

Reads paper_ispass/e6_sweep/<arm>_<weather>/rep<i>/run.log and emits one row
per repetition with:
  - collision count (collision warnings in the run log)
  - first_detect_tick: first tick the edge holds a track for the critical
    actor (identity-first presence lines from the frame logger)
  - presence_ratio: fraction of in-range ticks with the critical actor present
  - det_recall: mean per-tick detection recall tp/(tp+fn) where logged
  - success: scenario completion without collision (proxy until the runner's
    explicit success line is confirmed on a smoke run)

The regexes target the current logger formats (edge manager "tick=%d det..."
lines, detection-metrics lines, collision warnings). They are deliberately
permissive; refine against the first smoke run's log before trusting numbers.
Usage: python scripts/e6_divergence_metrics.py [sweep_dir] > e6_metrics.csv
"""
import csv
import re
import sys
from pathlib import Path

SWEEP_DIR = Path(sys.argv[1] if len(sys.argv) > 1 else
                 'paper_ispass/e6_sweep')

RE_COLLISION = re.compile(r'collision', re.IGNORECASE)
RE_DET_METRICS = re.compile(
    r'tp=(\d+)\s+fp=(\d+)\s+fn=(\d+)|true_positives=(\d+).*false_negatives=(\d+)')
# presence lines from the identity-first frame logger (B4-style):
# permissive: any line naming a track/presence for an npc/actor with a tick
RE_PRESENCE = re.compile(
    r'tick=(\d+).*(?:presence|track_on_edge|npc.*(?:present|detect))',
    re.IGNORECASE)
RE_TICK_DET = re.compile(r'tick=(\d+)\s+dets? before_filter=(\d+)')


def parse_rep(log_path: Path) -> dict:
    collisions = 0
    first_detect = None
    presence_ticks = 0
    range_ticks = 0
    recalls = []
    for line in log_path.read_text(errors='replace').splitlines():
        if RE_COLLISION.search(line) and 'warning' in line.lower():
            collisions += 1
        m = RE_PRESENCE.search(line)
        if m:
            tick = int(m.group(1))
            presence_ticks += 1
            if first_detect is None:
                first_detect = tick
        m = RE_TICK_DET.search(line)
        if m:
            range_ticks += 1
        m = RE_DET_METRICS.search(line)
        if m:
            g = [x for x in m.groups() if x is not None]
            if len(g) >= 3:
                tp, _fp, fn = int(g[0]), int(g[1]), int(g[2])
            else:
                tp, fn = int(g[0]), int(g[1])
            if tp + fn > 0:
                recalls.append(tp / (tp + fn))
    return {
        'collisions': collisions,
        'first_detect_tick': first_detect if first_detect is not None else '',
        'presence_ratio': (presence_ticks / range_ticks) if range_ticks else '',
        'det_recall': (sum(recalls) / len(recalls)) if recalls else '',
        'success': 1 if collisions == 0 else 0,
    }


def main():
    writer = csv.writer(sys.stdout)
    writer.writerow(['arm', 'weather', 'rep', 'collisions',
                     'first_detect_tick', 'presence_ratio', 'det_recall',
                     'success'])
    for cell in sorted(SWEEP_DIR.glob('*_w*/rep*')):
        log = cell / 'run.log'
        if not log.exists():
            continue
        arm_weather = cell.parent.name
        arm, _, weather = arm_weather.rpartition('_')
        rep = cell.name.replace('rep', '')
        row = parse_rep(log)
        writer.writerow([arm, weather, rep, row['collisions'],
                         row['first_detect_tick'], row['presence_ratio'],
                         row['det_recall'], row['success']])


if __name__ == '__main__':
    main()
