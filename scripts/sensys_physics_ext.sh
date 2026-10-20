#!/usr/bin/env bash
# Physics-stage extension: ages past the planned sweep for the oracle cells
# whose transition the sweep did not bracket. Oct 17 (stage j2): the oracle
# holds 1.00 to 700 ms at 38 and 53 km/h on scenario_3 at the same crossing
# phasing that gives 300 to 400 ms at 70, because a slower hero stops in a
# shorter distance from the same brake onset. Same configuration as the
# stage (scripts/sensys_junction.sh): EXTRA, prediction horizon, per-speed
# crossing phase maps, 3 seeds for the deterministic oracle.
# Run only when no other simulation is on the host:
#   STAMP=sensys_nov27_j2 SPEEDS="38 53" CARLA_PORT=2000 nohup scripts/sensys_physics_ext.sh atlas > logs/physics_ext_atlas.log 2>&1 &
set -uo pipefail
ROLE="${1:?atlas|cetus}"
STAMP="${STAMP:-sensys_nov27_j2}"
# SCENV: extra KEY=VALUE pairs for the scenario process (e.g. HIDE_ENV_OBJECTS=...), on the same
# --scenario-env flag as the crossing trigger because a repeated flag keeps only the last one.
SCENV="${SCENV:-}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
EXT_LATS="${EXT_LATS:-1500 2000 3000}"
SCENARIOS="${SCENARIOS:-openscenario_3_edge_late_fusion scp_edge_late_fusion}"
case "$ROLE" in
  atlas) SPEEDS="${SPEEDS:-38 53 70}" ;;
  cetus) SPEEDS="${SPEEDS:-50 63 88}" ;;
  *) echo "role must be atlas or cetus"; exit 2 ;;
esac
EXTRA="${EXTRA:---behavior-kv curv_lat_accel=8}"
PRED_STEPS="${PRED_STEPS:-60}"
OREPS="${ORACLE_REPS:-3}"
[ -f logs/tta_maps.env ] && source logs/tta_maps.env
TTA_S3="${TTA_S3:-38:5.25,50:5.0,53:5.0,63:5.5,70:7.0,88:9.0}"
TTA_SCP="${TTA_SCP:-38:5.55,50:5.4,53:5.3,63:6.5,70:8.0,88:10.17}"
tta_for() { case "$1" in openscenario_3*) echo "$TTA_S3" ;; scp_*) echo "$TTA_SCP" ;; esac; }
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --stamp $STAMP --carla-port $PORT --per-tick-age-log $EXTRA --edge-kv prediction_steps=$PRED_STEPS"
echo "$(date +%T) $ROLE stamp $STAMP speeds [$SPEEDS] lats [$EXT_LATS] tta_s3 [$TTA_S3] tta_scp [$TTA_SCP]"
for sc in $SCENARIOS; do
  "$PY" test_runner.py -t "$sc" --manager-types oracle --anchoring on \
      --latencies $EXT_LATS --speeds $SPEEDS --repetitions "$OREPS" $COMMON \
      --scenario-env "CROSS_TRIGGER_TTA=$(tta_for "$sc")" $SCENV
done
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || true
echo "$(date +%T) physics extension done on $ROLE"
