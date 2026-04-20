#!/bin/bash
# Run all selector + K_cav sweeps on both zones.
# Produces: ablation_{rsu250,rsu40}_{causal,random,lightweight,oracle_total,oracle_occluded}.csv
set -e

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$REPO"

DATA_ROOT="${DATA_ROOT:-/data1/Datasets/Multi-V2X}"
OUT_DIR="${OUT_DIR:-paper2_figures}"

TOWN_A="Town05__2023_11_13_23_03_07"
RSU_A="rsu_250"
TOWN_B="Town10HD__2023_11_15_00_20_38"
RSU_B="rsu_40"

FRAMES=50
K_SWEEP="0,2,4,8"

for zone_label in A B; do
    if [ "$zone_label" = "A" ]; then
        TOWN="$TOWN_A"; RSU="$RSU_A"; RSU_SHORT="rsu250"
    else
        TOWN="$TOWN_B"; RSU="$RSU_B"; RSU_SHORT="rsu40"
    fi
    for mode in causal random lightweight oracle_total oracle_occluded; do
        echo "=== Zone $zone_label ($RSU) / $mode ==="
        python paper2/paper2_offline_profiler_v3.py \
            --data-root "$DATA_ROOT" \
            --town "$TOWN" \
            --rsu "$RSU" \
            --max-frames "$FRAMES" \
            --sweep-k-cav "$K_SWEEP" \
            --filter-mode "$mode" \
            --out "$OUT_DIR/ablation_${RSU_SHORT}_${mode}.csv"
    done
done
