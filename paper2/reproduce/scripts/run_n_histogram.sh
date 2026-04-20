#!/bin/bash
# Generate the Multi-V2X connected-agent distribution across all 56 zones.
# Produces multiv2x_n_per_zone.csv and the N histogram figure.
set -e

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$REPO"

DATA_ROOT="${DATA_ROOT:-/data1/Datasets/Multi-V2X}"

python paper2/paper2_multiv2x_n_histogram.py \
    --data-root "$DATA_ROOT" \
    --out paper2_figures/multiv2x_n_per_zone.csv
