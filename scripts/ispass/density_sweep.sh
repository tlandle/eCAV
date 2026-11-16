#!/usr/bin/env bash
# ISPASS: does sensor-derived perception change the closed-loop outcome, and does the difference change with
# traffic density. No edge assistance: every shared message is dropped, the vehicle drives on its own state.
#
# The straight crossing is held fixed (same encounter as fidelity_ablation.sh) and M background road
# users are added around the junction (scenario_3.py, BG_TRAFFIC). Two arms:
#   gt_vis   simulator ground truth clipped to the vehicle's LiDAR range and to line of sight from its roof sensor
#            position (test_runner --vehicle-state gt_visible): the visibility of the sensors, none of their errors
#   onboard  camera and LiDAR detection and tracking on the vehicle
# Two encounters:
#   open     the parked trucks are removed (OCCLUDERS=0): the crossing car is in the vehicle's sensor view
#   occ      the occluded junction: the crossing car is hidden until the junction
# Every LiDAR carries range noise (LIDAR_NOISE, metres, default 0.02; the scenario YAMLs carry 0).
# The arm, the encounter and M are not in the run directory name, so each cell has its own stamp:
# <STAMP>_<encounter>_<arm>_m<M>.
#
#   STAMP=ispass_den1 scripts/ispass/density_sweep.sh gate          # one run per cell
#   STAMP=ispass_den1 REPS=30 scripts/ispass/density_sweep.sh full
#   env: MS="4 8 16 32 64 128"  ARMS="gt_vis onboard"  ENCS="open occ"  REPS  LIDAR_NOISE  SPEED  EGO_Y  MAX_STEP  CARLA_PORT
set -u
STAGE="${1:?gate|full}"; DRY="${2:-}"
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_den1}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-30}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
MS="${MS:-4 8 16 32 64 128}"; ARMS="${ARMS:-gt_vis onboard}"; ENCS="${ENCS:-open occ}"; LIDAR_NOISE="${LIDAR_NOISE:-0.02}"
# SPEED (km/h): at 70 the crossing car enters the 50 m LiDAR range 1.6 s before the conflict, too late for a vehicle
# without shared state whatever its state source; at 40 it enters about 3.6 s before. EGO_Y is the start of the approach.
SPEED="${SPEED:-40}"; EGO_Y="${EGO_Y:--30}"; MAX_STEP="${MAX_STEP:-1400}"
# LOCAL_STEPS: forecast horizon of the vehicle's own predictor in 50 ms steps. The platform default is 10 (0.5 s, a
# fallback beside edge forecasts); a vehicle that drives alone gets the edge's horizon, 60 steps (3 s).
LOCAL_STEPS="${LOCAL_STEPS:-60}"
# TTA: the crossing car launches when the connected vehicle is TTA seconds from the conflict point (5.12 s is the
# 70 km/h calibration of the SenSys encounter; another speed needs its own).
TTA="${TTA:-5.12}"
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
arm_args() { case "$1" in
  gt_vis)  echo "--vehicle-state gt_visible" ;;
  onboard) echo "" ;; esac; }
enc_env() { case "$1" in
  open) echo "OCCLUDERS=0" ;;
  occ)  echo "OCCLUDERS=1" ;; esac; }
run_cell() {  # encounter arm M reps
  local cmd=(test_runner.py -t scp_edge_late_fusion --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0
             --latency-distribution fixed --speeds $SPEED --ego-counts 1 --repetitions "$4" --per-tick-age-log --self-id-radius 1
             --carla-port "$PORT" --stamp "${STAMP}_$1_$2_m$3" $(arm_args "$2") --lidar-noise-std "$LIDAR_NOISE" --local-prediction-steps "$LOCAL_STEPS"
             --edge-kv prediction_steps=60 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env CROSS_TRIGGER_TTA=$SPEED:$TTA HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=$EGO_Y EGO_DRIVE_DISTANCE=340 SCENARIO_MAX_STEP=$MAX_STEP
                            BG_TRAFFIC=$3 $(enc_env "$1"))
  echo "$(date '+%F %T') enc=$1 arm=$2 M=$3 reps=$4"
  if [[ "$DRY" == "--dry-run" ]]; then echo "$PY ${cmd[*]}"; return; fi
  local t0=$(date +%s)
  "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3
  echo "$(date '+%F %T') enc=$1 arm=$2 M=$3 wall $(( $(date +%s) - t0 )) s"
}
if [[ "$DRY" != "--dry-run" ]]; then
  if pgrep -f "^$PY test_runner.py" >/dev/null || pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
    echo "a runner or CARLA is active; aborting"; exit 1; fi
  if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; exit 1; fi
  mkdir -p experiment_results/campaign_meta
  { echo "stamp=$STAMP stage=$STAGE reps=$REPS ms=$MS arms=$ARMS encs=$ENCS lidar_noise=$LIDAR_NOISE speed=$SPEED ego_y=$EGO_Y local_steps=$LOCAL_STEPS tta=$TTA"; git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_density.txt"
fi
case "$STAGE" in
  gate) for e in $ENCS; do for m in $MS; do for a in $ARMS; do run_cell "$e" "$a" "$m" "${GATE_REPS:-1}"; done; done; done ;;
  full) for e in $ENCS; do for m in $MS; do for a in $ARMS; do run_cell "$e" "$a" "$m" "$REPS"; done; done; done ;;
  *) echo "unknown stage"; exit 2 ;;
esac
if [[ "$DRY" != "--dry-run" ]]; then
  for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
  for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
  echo "$(date '+%F %T') $STAGE end"
fi
