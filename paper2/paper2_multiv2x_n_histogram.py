"""Generate per-zone and overall connected-agent histograms across Multi-V2X.

For each RSU zone in each town, reads the RSU's yaml files and counts
the size of the `conn_agents` list (N_cav, not counting the RSU itself).

Outputs:
  - paper2_figures/multiv2x_n_per_zone.csv   per-zone stats
  - paper2_figures/multiv2x_n_histogram.csv  binned histogram
  - paper2_figures/multiv2x_n_histogram.png  histogram figure

The binned histogram uses N_total = 1 + N_cav to match the profiler's
definition (RSU + connected CAVs).
"""
import argparse
import csv
import os
from collections import Counter
from pathlib import Path

import numpy as np
import yaml as pyyaml

DEFAULT_DATA_ROOT = '/data1/Datasets/Multi-V2X'
OUT_DIR = Path('paper2_figures')


def count_rsu_zone(rsu_dir: Path):
    """Return list of N_cav per frame in this RSU zone."""
    yaml_files = sorted([f for f in rsu_dir.glob('*.yaml') if f.stem.isdigit()])
    counts = []
    for f in yaml_files:
        try:
            with open(f) as fh:
                d = pyyaml.safe_load(fh)
        except Exception:
            continue
        conn = d.get('conn_agents', []) or []
        counts.append(len(conn))
    return counts


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--data-root', default=DEFAULT_DATA_ROOT)
    p.add_argument('--max-frames-per-zone', type=int, default=None,
                   help='If set, subsample up to this many frames per zone.')
    args = p.parse_args()

    data_root = Path(args.data_root)
    towns = sorted([d for d in data_root.iterdir()
                    if d.is_dir() and d.name.startswith('Town')])
    print(f"Found {len(towns)} towns")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    per_zone_rows = []
    all_counts = []  # list of N_cav across every (town, rsu, frame)

    for town_dir in towns:
        rsu_dirs = sorted([d for d in town_dir.iterdir()
                           if d.is_dir() and d.name.startswith('rsu_')])
        print(f"\n{town_dir.name}: {len(rsu_dirs)} RSUs")
        for rsu_dir in rsu_dirs:
            counts = count_rsu_zone(rsu_dir)
            if not counts:
                continue
            if args.max_frames_per_zone and len(counts) > args.max_frames_per_zone:
                stride = len(counts) // args.max_frames_per_zone
                counts = counts[::stride][:args.max_frames_per_zone]
            arr = np.array(counts)
            n_total = arr + 1  # RSU + CAVs
            per_zone_rows.append({
                'town': town_dir.name,
                'rsu': rsu_dir.name,
                'frames': len(arr),
                'ncav_mean': round(float(arr.mean()), 2),
                'ncav_median': int(np.median(arr)),
                'ncav_p75': int(np.percentile(arr, 75)),
                'ncav_p95': int(np.percentile(arr, 95)),
                'ncav_max': int(arr.max()),
                'ntotal_mean': round(float(n_total.mean()), 2),
                'ntotal_median': int(np.median(n_total)),
                'ntotal_p95': int(np.percentile(n_total, 95)),
                'ntotal_max': int(n_total.max()),
            })
            all_counts.extend(counts)
            print(f"  {rsu_dir.name}: frames={len(arr)} "
                  f"ncav mean={arr.mean():.1f} med={int(np.median(arr))} "
                  f"max={int(arr.max())}")

    # Per-zone CSV
    per_zone_csv = OUT_DIR / 'multiv2x_n_per_zone.csv'
    if per_zone_rows:
        with open(per_zone_csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(per_zone_rows[0].keys()))
            w.writeheader()
            w.writerows(per_zone_rows)
        print(f"\nWrote {per_zone_csv}")

    if not all_counts:
        print("No data collected.")
        return

    all_cav = np.array(all_counts)
    all_total = all_cav + 1

    # Overall summary
    print("\n" + "=" * 64)
    print("Overall Multi-V2X connectivity (all towns, all zones, all frames)")
    print("=" * 64)
    print(f"  Frames       : {len(all_cav)}")
    print(f"  Zones        : {len(per_zone_rows)}")
    print(f"  N_cav mean   : {all_cav.mean():.2f}")
    print(f"  N_cav median : {int(np.median(all_cav))}")
    print(f"  N_cav p75    : {int(np.percentile(all_cav, 75))}")
    print(f"  N_cav p95    : {int(np.percentile(all_cav, 95))}")
    print(f"  N_cav p99    : {int(np.percentile(all_cav, 99))}")
    print(f"  N_cav max    : {int(all_cav.max())}")
    print(f"  N_total mean : {all_total.mean():.2f}")
    print(f"  N_total p95  : {int(np.percentile(all_total, 95))}")
    print(f"  N_total max  : {int(all_total.max())}")

    # Binned histogram CSV (by N_total)
    bins = list(range(0, int(all_total.max()) + 2))
    hist, edges = np.histogram(all_total, bins=bins)
    hist_csv = OUT_DIR / 'multiv2x_n_histogram.csv'
    with open(hist_csv, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['n_total', 'count', 'frac'])
        total = hist.sum()
        for i, c in enumerate(hist):
            if c == 0:
                continue
            w.writerow([i, int(c), round(float(c) / total, 4)])
    print(f"Wrote {hist_csv}")

    # Histogram figure
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(6, 3.2))
        ax.bar(np.arange(len(hist)), hist, width=0.9, color='#2b6cb0')
        ax.set_xlabel('N_total = 1 RSU + connected CAVs')
        ax.set_ylabel('Frames')
        ax.set_title(f'Multi-V2X connected-agent distribution '
                     f'({len(per_zone_rows)} zones, {len(all_cav):,} frames)')
        # Annotate key quantiles
        for q, label in [(50, 'med'), (95, 'p95')]:
            qv = np.percentile(all_total, q)
            ax.axvline(qv, color='red' if q == 95 else 'orange',
                       linestyle='--', linewidth=1)
            ax.text(qv, ax.get_ylim()[1] * 0.9,
                    f'{label}={int(qv)}', color='red' if q == 95 else 'orange',
                    rotation=90, va='top', ha='right')
        fig.tight_layout()
        fig_path = OUT_DIR / 'multiv2x_n_histogram.png'
        fig.savefig(fig_path, dpi=140)
        print(f"Wrote {fig_path}")
    except Exception as e:
        print(f"[warn] skipped figure: {e}")


if __name__ == '__main__':
    main()
