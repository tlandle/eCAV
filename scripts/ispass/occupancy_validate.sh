#!/usr/bin/env bash
# Service occupancy in closed loop: the left turn with the intermediate-fusion manager, measured
# compute, ideal delivery, service_occupancy on against off.
#   STAMP=ispass_occ REPS=2 [GEOM=lt|scp] scripts/ispass/occupancy_validate.sh [--dry-run]
set -uo pipefail
DRY="${1:-}"
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_occ}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-2}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
# GEOM=lt (left turn, default) or scp (straight crossing), with the calibrated crossing trigger
case "${GEOM:-lt}" in
  lt)  SCEN=openscenario_3_edge_late_fusion; TTA="${TTA_LT:-70:4.77}" ;;
  scp) SCEN=scp_edge_late_fusion; TTA="${TTA_SCP:-70:5.12}" ;;
  *) echo "GEOM lt|scp"; exit 2 ;; esac
for occ in ${OCCS:-off on}; do
  cmd=(test_runner.py -t "$SCEN" --baselines wf --anchoring on --latencies 0 --latency-distribution fixed --speeds 70
       --ego-counts 1 --repetitions "$REPS" --per-tick-age-log --self-id-radius 1 --carla-port "$PORT" --stamp "$STAMP"
       --edge-kv prediction_steps=60 compute_delay=measured use_ns3_lut=false lut_dl=false service_occupancy=$occ ${EDGE_KV_EXTRA:-}
       --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
       --scenario-env CROSS_TRIGGER_TTA=$TTA HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900)
  echo "$(date '+%F %T') service_occupancy=$occ reps=$REPS"
  if [[ "$DRY" == "--dry-run" ]]; then echo "$PY ${cmd[*]}"; else "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3; fi
done
if [[ "$DRY" != "--dry-run" ]]; then
  for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
  for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
  echo "$(date '+%F %T') occupancy_validate end"
fi
