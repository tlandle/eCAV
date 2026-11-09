#!/usr/bin/env bash
# ISPASS Experiment C.1: two runs of one configuration with one seed, compared tick by tick.
#   scripts/ispass/fixed_seed_repeat.sh [exact|coop] [seed]
# Runs the highway merge (closing speed 10 m/s, 200 ms configured delay) twice under two stamps and
# calls fixed_seed_compare.py on the pair. 'exact' is the exact-state arm (no perception model),
# 'coop' the sensor-derived V2X2V +SBA arm (YOLO on the GPU, live stamps).
set -euo pipefail
ARM="${1:-exact}"; SEED="${2:-11}"
cd "$(dirname "$0")/../.."
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
case "$ARM" in exact) MGR=oracle;; coop) MGR=late_fusion;; *) echo "arm exact|coop"; exit 2;; esac
COMMON="--ego-counts 1 --speeds 97 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port ${CARLA_PORT:-2000} --edge-kv prediction_steps=60 closing_speed_mps=10 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 --scenario-env CLOSING_SPEED_MPS=10"
for tag in A B; do
  "$PY" test_runner.py -t highway_merge_edge_late_fusion --manager-types "$MGR" --anchoring on --latencies 200 --repetitions 1 --seed "$SEED" --stamp "ispass_repeat_${ARM}_${tag}" $COMMON
done
"$PY" scripts/ispass/fixed_seed_compare.py "ispass_repeat_${ARM}_A" "ispass_repeat_${ARM}_B"
