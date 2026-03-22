#!/usr/bin/env python3
"""
Summarize per-tick ML timing CSV produced by MLManager.dump_timing_csv().

Usage:
    python tools/analyze_ml_timing.py logs/ml_timing_<timestamp>.csv [logs/ml_timing_<other>.csv ...]

Prints min/max/avg/p95 for each numeric column, grouped by mode (local/distributed).
When multiple files are provided, each is summarized separately.
"""

import csv
import sys
import os
import numpy as np

NUMERIC_COLS = [
    'serialize_ms',
    'rpc_ms',
    'server_decode_ms',
    'server_img_prep_ms',
    'server_inference_ms',
    'server_encode_ms',
    'client_deserialize_ms',
    'parse_ms',
    'total_e2e_ms',
]


def summarize(path):
    rows_by_mode = {}

    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            mode = row['mode']
            if mode not in rows_by_mode:
                rows_by_mode[mode] = {col: [] for col in NUMERIC_COLS}
            for col in NUMERIC_COLS:
                val = row.get(col, '')
                if val != '':
                    rows_by_mode[mode][col].append(float(val))

    print(f"\n{'='*72}")
    print(f"  {os.path.basename(path)}")
    print(f"{'='*72}")

    for mode, data in sorted(rows_by_mode.items()):
        n = len(data['total_e2e_ms'])
        print(f"\n  mode={mode}  n={n} ticks\n")
        print(f"  {'metric':<24}  {'min':>8}  {'avg':>8}  {'p95':>8}  {'max':>8}")
        print(f"  {'-'*24}  {'-'*8}  {'-'*8}  {'-'*8}  {'-'*8}")

        for col in NUMERIC_COLS:
            vals = data[col]
            if not vals:
                continue
            a = np.array(vals)
            # Skip columns that are all zero (not applicable for this mode)
            if a.max() == 0.0:
                continue
            print(f"  {col:<24}  {a.min():>8.2f}  {a.mean():>8.2f}  "
                  f"{np.percentile(a, 95):>8.2f}  {a.max():>8.2f}")

    print()


def main():
    if len(sys.argv) < 2:
        # Default: latest file in logs/
        log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        files = sorted(
            [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.startswith('ml_timing_')],
        )
        if not files:
            print("No ml_timing_*.csv files found in logs/. Pass a path explicitly.")
            sys.exit(1)
        paths = [files[-1]]
        print(f"No file specified — using latest: {paths[0]}")
    else:
        paths = sys.argv[1:]

    for path in paths:
        summarize(path)


if __name__ == '__main__':
    main()
