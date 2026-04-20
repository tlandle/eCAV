#!/bin/bash
# Run the joint-controller profile on Zone A.
# Produces ablation_rsu250_joint.csv with three configs:
#   - static_all (no adaptation)
#   - joint (causal selector + divergence gating + risk-budget)
#   - pred_only_adaptive (no selector, amort + risk)
set -e

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$REPO"

DATA_ROOT="${DATA_ROOT:-/data1/Datasets/Multi-V2X}"
OUT_DIR="${OUT_DIR:-paper2_figures}"

python paper2/paper2_offline_profiler_v3.py \
    --data-root "$DATA_ROOT" \
    --town "Town05__2023_11_13_23_03_07" \
    --rsu "rsu_250" \
    --max-frames 50 \
    --filter-mode causal \
    --joint \
    --out "$OUT_DIR/ablation_rsu250_joint.csv"
