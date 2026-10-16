#!/usr/bin/env bash
# Zero-delay gate of the blind-overtake geometry with the timing Khonsu
# tuned it with (ONCOMING_SPEED=12 m/s, TRIGGER_DIST=300 m): oracle and
# contract, 10 seeds, 70 km/h, stamp overtake_gate. Run only on a free host.
#   CARLA_PORT=2000 nohup scripts/sensys_overtake_gate.sh atlas > logs/overtake_gate_atlas.log 2>&1 &
set -euo pipefail
ROLE="${1:?atlas|cetus}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
ENV="${OVERTAKE_ENV:-ONCOMING_SPEED=12 TRIGGER_DIST=300}"
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port $PORT --stamp overtake_gate --speeds ${SPEEDS:-70} --latencies ${LATS:-0} --repetitions ${REPS:-10} --scenario-env $ENV"
"$PY" test_runner.py -t blind_overtake_edge_late_fusion --manager-types oracle --anchoring on $COMMON
"$PY" test_runner.py -t blind_overtake_edge_late_fusion --manager-types late_fusion --anchoring on $COMMON
"$PY" scripts/sensys_analysis/campaign_check.py overtake_gate || true
