#!/usr/bin/env bash
# Physics-stage extension: ages past the planned sweep for the arms whose
# transition the sweep did not bracket (campaign_check's bracketing list).
# Oct 16: the oracle holds P(S_op)=1.00 at every delay to 1,000 ms at all six
# speeds on scenario_3, so its boundary there lies past the sweep.
# Run only when no other simulation is on the host (one orchestration server
# port per host):
#   STAMP=sensys_nov27 CARLA_PORT=2000 nohup scripts/sensys_physics_ext.sh atlas|cetus > logs/physics_ext_<role>.log 2>&1 &
set -euo pipefail
ROLE="${1:?atlas|cetus}"
STAMP="${STAMP:-sensys_nov27}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
EXT_LATS="${EXT_LATS:-1500 2000 3000}"
SCENARIOS="${SCENARIOS:-openscenario_3_edge_late_fusion}"
case "$ROLE" in
  atlas) SPEEDS="${SPEEDS:-38 53 70}" ;;
  cetus) SPEEDS="${SPEEDS:-50 63 88}" ;;
  *) echo "role must be atlas or cetus"; exit 2 ;;
esac
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --stamp $STAMP --carla-port $PORT --per-tick-age-log"
for sc in $SCENARIOS; do
  "$PY" test_runner.py -t "$sc" --manager-types oracle --anchoring on \
      --latencies $EXT_LATS --speeds $SPEEDS --repetitions 10 $COMMON
done
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || true
