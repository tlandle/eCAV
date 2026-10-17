#!/usr/bin/env bash
# Crossing-phase calibration: the oracle at the listed delays, N seeds, for each
# CROSS_TRIGGER_TTA value (seconds of hero time-to-arrival at the conflict point
# at which the crossing car launches). Read each stamp with
# scripts/diag/junction_margin.py: the value to use is the largest at which the
# hero at 0 ms still has to yield (small positive margin) and the oracle fails
# between 300 and 500 ms. One stamp per value. Run only on a host with no other
# simulation.
#   CARLA_PORT=2000 nohup scripts/sensys_tta_probe.sh atlas "3.0 3.5 4.0" > logs/tta_probe_atlas.log 2>&1 &
set -euo pipefail
ROLE="${1:?atlas|cetus}"; TTAS="${2:-3.0 3.5 4.0 4.5}"
STAMP_PREFIX="${STAMP_PREFIX:-tta_probe_}"
EXTRA="${EXTRA:---behavior-kv curv_lat_accel=8}"
LATS="${LATS:-0 300 500}"
REPS="${REPS:-5}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
GEOMS="${GEOMS:-openscenario_3_edge_late_fusion}"
for v in $TTAS; do
  for sc in $GEOMS; do
    "$PY" test_runner.py -t "$sc" --manager-types oracle --anchoring on --latencies $LATS \
        --speeds 70 --repetitions "$REPS" --scenario-env "CROSS_TRIGGER_TTA=$v" $EXTRA \
        --latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port "$PORT" \
        --stamp "${STAMP_PREFIX}${v}"
    "$PY" scripts/diag/junction_margin.py "${STAMP_PREFIX}${v}" --geom "$sc" || true
  done
  "$PY" scripts/sensys_analysis/campaign_check.py "${STAMP_PREFIX}${v}" 2>&1 | grep -E "^== |^[a-z_0-9]+ +70 +oracle" || true
done
echo "tta probe done"
