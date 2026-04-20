#!/bin/bash
# End-to-end reproduction of all paper data.
# Requires: opencda310 conda env, Multi-V2X at /data1/Datasets/Multi-V2X.
# Runtime: ~4 hours on an NVIDIA A10 24 GB.
set -e

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$REPO"

echo "### 1/4: Multi-V2X N distribution ###"
bash "$HERE/run_n_histogram.sh"

echo "### 2/4: Selector + K_cav sweeps ###"
bash "$HERE/run_selector_sweeps.sh"

echo "### 3/4: Joint controller profile ###"
bash "$HERE/run_joint_ablation.sh"

echo "### 4/4: Mechanism ablation (amort_only, risk_only) ###"
bash "$HERE/run_mechanism_ablation.sh"

echo "### Generating plots ###"
python paper2/paper2_plot_evaluation.py
python paper2/paper2_plot_evaluation_v2.py

echo "### Done ###"
echo "CSVs: paper2_figures/*.csv"
echo "Figures: paper2_figures/*.pdf (and .png)"
