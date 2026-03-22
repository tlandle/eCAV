#!/usr/bin/env python3
"""
Summarize per-tick WorldFusion timing CSVs produced by WorldFusionPerceptionManager.close().

Usage:
    python tools/analyze_wf_timing.py [logs/wf_timing_*.csv ...]

When no file is given, the latest wf_timing_*.csv in logs/ is used.
Prints min/avg/p95/max for each numeric column, grouped by (mode, agent_type).
"""

import csv
import os
import sys

import numpy as np

NUMERIC_COLS = [
    'build_batch_ms',
    'to_numpy_ms',
    'pack_ms',
    'payload_bytes',
    'http_ms',
    'server_read_ms',
    'server_decode_ms',
    'server_inference_ms',
    'server_encode_ms',
    'server_total_ms',
    'response_bytes',
    'unpack_ms',
    'to_tensor_ms',
    'total_e2e_ms',
]


def summarize(path):
    groups = {}  # (mode, agent_type) -> {col: [values]}

    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            key = (row.get('mode', '?'), row.get('agent_type', '?'))
            if key not in groups:
                groups[key] = {col: [] for col in NUMERIC_COLS}
            for col in NUMERIC_COLS:
                val = row.get(col, '')
                if val not in ('', '0', None):
                    try:
                        groups[key][col].append(float(val))
                    except ValueError:
                        pass

    print(f"\n{'='*80}")
    print(f"  {os.path.basename(path)}")
    print(f"{'='*80}")

    for (mode, agent_type), data in sorted(groups.items()):
        n = len(data.get('total_e2e_ms', []))
        print(f"\n  mode={mode}  agent={agent_type}  n={n} ticks\n")
        print(f"  {'metric':<26}  {'min':>9}  {'avg':>9}  {'p95':>9}  {'max':>9}")
        print(f"  {'-'*26}  {'-'*9}  {'-'*9}  {'-'*9}  {'-'*9}")

        for col in NUMERIC_COLS:
            vals = data[col]
            if not vals:
                continue
            a = np.array(vals)
            if col in ('payload_bytes', 'response_bytes'):
                # Display in KB
                label = col.replace('_bytes', '_kb')
                a = a / 1024.0
                print(f"  {label:<26}  {a.min():>9.1f}  {a.mean():>9.1f}  "
                      f"{np.percentile(a, 95):>9.1f}  {a.max():>9.1f}")
            else:
                print(f"  {col:<26}  {a.min():>9.2f}  {a.mean():>9.2f}  "
                      f"{np.percentile(a, 95):>9.2f}  {a.max():>9.2f}")

    print()


def main():
    if len(sys.argv) < 2:
        log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        try:
            files = sorted(
                os.path.join(log_dir, f)
                for f in os.listdir(log_dir)
                if f.startswith('wf_timing_')
            )
        except FileNotFoundError:
            print(f"logs/ directory not found. Run a scenario first.")
            sys.exit(1)
        if not files:
            print("No wf_timing_*.csv files found in logs/. Pass a path explicitly.")
            sys.exit(1)
        paths = [files[-1]]
        print(f"No file specified — using latest: {paths[0]}")
    else:
        paths = sys.argv[1:]

    for path in paths:
        summarize(path)


if __name__ == '__main__':
    main()
