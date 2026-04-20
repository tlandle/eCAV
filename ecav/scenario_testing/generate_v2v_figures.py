#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Generate paper figures for the V2V characterization study.

Figures:
  1. PRR vs N at different payload sizes (ns-3 NR V2X standalone sweep)
  2. Per-vehicle compute time vs N (CARLA closed-loop experiments)
  3. Collision avoidance success rate vs N per fusion level
  4. Overlaid network + compute + safety bottleneck curves

Usage:
    python -m ecav.scenario_testing.generate_v2v_figures \\
        --ns3-csv /path/to/nr_v2x_full_sweep.csv \\
        --sweep-csv evaluation_outputs/v2v_n_sweep/v2v_n_sweep_summary.csv \\
        --output-dir paper2_figures/v2v_characterization/
"""

import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Paper-quality matplotlib settings
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'axes.labelsize': 11,
    'axes.titlesize': 11,
    'legend.fontsize': 9,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'figure.figsize': (3.5, 2.5),  # single-column width
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'lines.linewidth': 1.5,
    'lines.markersize': 5,
})

PAYLOAD_LABELS = {
    300: "BSM (300B)",
    1000: "Compressed LF (1KB)",
    5000: "Late Fusion (5KB)",
    50000: "Early Fusion (50KB)",
}

PAYLOAD_COLORS = {
    300: "#2196F3",
    1000: "#4CAF50",
    5000: "#FF9800",
    50000: "#F44336",
}

FUSION_LABELS = {
    "late": "Late Fusion (5KB)",
    "early": "Early Fusion (50KB)",
    "intermediate": "Intermediate (200KB)",
}

FUSION_COLORS = {
    "late": "#4CAF50",
    "early": "#FF9800",
    "intermediate": "#F44336",
}


def load_ns3_sweep(csv_path: str):
    """Load ns-3 standalone sweep results."""
    import pandas as pd
    df = pd.read_csv(csv_path)
    # Filter out failed runs
    df = df[~df['avgPRR'].isin(['FAIL', 'NO_DATA', 'QUERY_FAIL'])]
    df['avgPRR'] = df['avgPRR'].astype(float)
    df['minPRR'] = df['minPRR'].astype(float)
    df['maxPRR'] = df['maxPRR'].astype(float)
    return df


def load_sweep_results(csv_path: str):
    """Load N-sweep CARLA experiment results."""
    import pandas as pd
    df = pd.read_csv(csv_path)
    return df


def figure1_prr_vs_n(ns3_df, output_dir: str):
    """
    Figure 1: PRR vs N at different payload sizes.

    This shows the payload-dependent communication knee. At low N, all
    payload sizes achieve near-perfect PRR. As N increases, larger
    payloads degrade first due to increased subchannel contention.
    """
    fig, ax = plt.subplots(figsize=(3.5, 2.5))

    for pkt_size in sorted(ns3_df['pktSize_bytes'].unique()):
        subset = ns3_df[ns3_df['pktSize_bytes'] == pkt_size].sort_values('totalUEs')
        label = PAYLOAD_LABELS.get(pkt_size, f"{pkt_size}B")
        color = PAYLOAD_COLORS.get(pkt_size, 'gray')

        ax.plot(subset['totalUEs'], subset['avgPRR'],
                marker='o', color=color, label=label)

        # Error band (min-max)
        ax.fill_between(subset['totalUEs'],
                        subset['minPRR'], subset['maxPRR'],
                        alpha=0.15, color=color)

    ax.set_xlabel("Number of vehicles (N)")
    ax.set_ylabel("Packet Reception Ratio (PRR)")
    ax.set_ylim(0, 1.05)
    ax.set_xlim(left=0)
    ax.axhline(y=0.95, color='gray', linestyle='--', linewidth=0.8,
               label='PRR = 0.95')
    ax.legend(loc='lower left', framealpha=0.9)
    ax.grid(True, alpha=0.3)

    fig.savefig(os.path.join(output_dir, "fig1_prr_vs_n.pdf"))
    fig.savefig(os.path.join(output_dir, "fig1_prr_vs_n.png"))
    plt.close(fig)
    print(f"  Figure 1: {output_dir}/fig1_prr_vs_n.pdf")


def figure2_compute_vs_n(sweep_df, output_dir: str):
    """
    Figure 2: Per-vehicle compute time vs N.

    Shows that per-vehicle compute is approximately constant (does not
    scale with N), while total compute grows linearly. The bottleneck
    is per-vehicle inference time, not N-dependent overhead.
    """
    fig, ax = plt.subplots(figsize=(3.5, 2.5))

    for fusion in sweep_df['fusion_level'].unique():
        subset = sweep_df[
            (sweep_df['fusion_level'] == fusion) &
            (sweep_df['status'] == 'success')
        ].sort_values('n_vehicles')

        if subset.empty:
            continue

        label = FUSION_LABELS.get(fusion, fusion)
        color = FUSION_COLORS.get(fusion, 'gray')

        if 'pipeline_ms_mean' in subset.columns:
            ax.plot(subset['n_vehicles'], subset['pipeline_ms_mean'],
                    marker='s', color=color, label=label)

    ax.set_xlabel("Number of vehicles (N)")
    ax.set_ylabel("Per-tick compute time (ms)")
    ax.set_xlim(left=0)
    ax.legend(loc='upper left', framealpha=0.9)
    ax.grid(True, alpha=0.3)

    fig.savefig(os.path.join(output_dir, "fig2_compute_vs_n.pdf"))
    fig.savefig(os.path.join(output_dir, "fig2_compute_vs_n.png"))
    plt.close(fig)
    print(f"  Figure 2: {output_dir}/fig2_compute_vs_n.pdf")


def figure3_safety_vs_n(sweep_df, output_dir: str):
    """
    Figure 3: Collision avoidance success rate vs N per fusion level.

    Shows that safety degrades as PRR drops. Late fusion maintains
    safety at higher N (smaller payloads, better PRR). Intermediate
    fusion fails immediately (200KB exceeds sidelink capacity).
    """
    fig, ax = plt.subplots(figsize=(3.5, 2.5))

    for fusion in sweep_df['fusion_level'].unique():
        subset = sweep_df[
            (sweep_df['fusion_level'] == fusion) &
            (sweep_df['status'] == 'success')
        ].sort_values('n_vehicles')

        if subset.empty:
            continue

        label = FUSION_LABELS.get(fusion, fusion)
        color = FUSION_COLORS.get(fusion, 'gray')

        if 'collision_avoidance_success' in subset.columns:
            ax.plot(subset['n_vehicles'],
                    subset['collision_avoidance_success'],
                    marker='^', color=color, label=label)

    ax.set_xlabel("Number of vehicles (N)")
    ax.set_ylabel("Collision avoidance success rate")
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlim(left=0)
    ax.legend(loc='lower left', framealpha=0.9)
    ax.grid(True, alpha=0.3)

    fig.savefig(os.path.join(output_dir, "fig3_safety_vs_n.pdf"))
    fig.savefig(os.path.join(output_dir, "fig3_safety_vs_n.png"))
    plt.close(fig)
    print(f"  Figure 3: {output_dir}/fig3_safety_vs_n.pdf")


def figure4_bottleneck_overlay(ns3_df, sweep_df, output_dir: str):
    """
    Figure 4: Overlaid network PRR + compute time + safety rate.

    Dual y-axis plot showing where network vs compute bottleneck
    occurs first for each fusion level. This is the key figure
    that motivates the edge for intermediate fusion.
    """
    fig, ax1 = plt.subplots(figsize=(4.5, 3.0))
    ax2 = ax1.twinx()

    # Network PRR (left axis) for late fusion payload (5KB)
    if 5000 in ns3_df['pktSize_bytes'].values:
        subset = ns3_df[ns3_df['pktSize_bytes'] == 5000].sort_values('totalUEs')
        ax1.plot(subset['totalUEs'], subset['avgPRR'],
                 'o-', color='#2196F3', label='PRR (5KB, V2V sidelink)')

    if 50000 in ns3_df['pktSize_bytes'].values:
        subset = ns3_df[ns3_df['pktSize_bytes'] == 50000].sort_values('totalUEs')
        ax1.plot(subset['totalUEs'], subset['avgPRR'],
                 's-', color='#F44336', label='PRR (50KB, V2V sidelink)')

    # PRR threshold
    ax1.axhline(y=0.95, color='gray', linestyle='--', linewidth=0.8)
    ax1.set_ylabel("PRR", color='#2196F3')
    ax1.set_ylim(0, 1.05)
    ax1.tick_params(axis='y', labelcolor='#2196F3')

    # Compute time (right axis)
    if sweep_df is not None and not sweep_df.empty:
        for fusion in sweep_df['fusion_level'].unique():
            subset = sweep_df[
                (sweep_df['fusion_level'] == fusion) &
                (sweep_df['status'] == 'success')
            ].sort_values('n_vehicles')

            if subset.empty or 'pipeline_ms_mean' not in subset.columns:
                continue

            color = FUSION_COLORS.get(fusion, 'gray')
            ax2.plot(subset['n_vehicles'], subset['pipeline_ms_mean'],
                     'D--', color=color,
                     label=f'Compute ({fusion})')

    ax2.set_ylabel("Compute time (ms)", color='#FF9800')
    ax2.tick_params(axis='y', labelcolor='#FF9800')

    ax1.set_xlabel("Number of vehicles (N)")
    ax1.set_xlim(left=0)

    # Combined legend
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2,
               loc='center left', framealpha=0.9, fontsize=8)

    ax1.grid(True, alpha=0.3)

    fig.savefig(os.path.join(output_dir, "fig4_bottleneck_overlay.pdf"))
    fig.savefig(os.path.join(output_dir, "fig4_bottleneck_overlay.png"))
    plt.close(fig)
    print(f"  Figure 4: {output_dir}/fig4_bottleneck_overlay.pdf")


def main():
    parser = argparse.ArgumentParser(
        description="Generate V2V characterization study figures")
    parser.add_argument("--ns3-csv", type=str,
                        default="/home/atlas/ns3_cv2x/ns-3-dev-5glena/scratch/nr_v2x_full_sweep.csv",
                        help="Path to ns-3 standalone sweep CSV")
    parser.add_argument("--sweep-csv", type=str, default=None,
                        help="Path to N-sweep CARLA results CSV")
    parser.add_argument("--output-dir", type=str,
                        default="paper2_figures/v2v_characterization/",
                        help="Output directory for figures")

    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    print("Generating V2V characterization figures...")

    # Figure 1: PRR vs N (ns-3 standalone)
    if os.path.exists(args.ns3_csv):
        ns3_df = load_ns3_sweep(args.ns3_csv)
        if not ns3_df.empty:
            figure1_prr_vs_n(ns3_df, args.output_dir)
        else:
            print("  Figure 1: skipped (no valid data in ns-3 CSV)")
    else:
        ns3_df = None
        print(f"  Figure 1: skipped (ns-3 CSV not found: {args.ns3_csv})")

    # Figures 2-4: from CARLA N-sweep
    sweep_df = None
    if args.sweep_csv and os.path.exists(args.sweep_csv):
        sweep_df = load_sweep_results(args.sweep_csv)
        if not sweep_df.empty:
            figure2_compute_vs_n(sweep_df, args.output_dir)
            figure3_safety_vs_n(sweep_df, args.output_dir)
        else:
            print("  Figures 2-3: skipped (no valid data in sweep CSV)")
    else:
        print("  Figures 2-3: skipped (sweep CSV not provided)")

    # Figure 4: overlay (needs both)
    if ns3_df is not None and not ns3_df.empty:
        figure4_bottleneck_overlay(ns3_df, sweep_df, args.output_dir)

    print("Done.")


if __name__ == "__main__":
    main()
