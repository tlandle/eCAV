#!/usr/bin/env bash
# ISPASS Section V, second geometry: a slower vehicle cuts into the lane of one sensor-equipped vehicle on a
# highway (highway_merge_edge_late_fusion, scenarios/scenario_highway_merge.py, Town06, 97 km/h). No edge
# assistance (uplink loss 100 %). The swept variable is the closing speed at the cut-in. Arms as in column_sweep.sh:
#   gt_vis   simulator ground truth clipped to the vehicle's LiDAR range and line of sight
#   onboard  camera and LiDAR detection and tracking on the vehicle
# Each cell has its own stamp, <STAMP>_<arm>_cs<closing speed>.
#
#   STAMP=ispass_cut1 REPS=1 scripts/ispass/cutin_sweep.sh        # gate: one run per cell
#   env: CS="4 7 10 13 16"  ARMS="gt_vis onboard"  REPS  SPEED  TTC_JITTER  LIDAR_Z  REAR_CAM_X  FOV  TRACE  HM_ENV
set -u
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_cut1}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-10}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
CS="${CS:-4 7 10 13 16}"; ARMS="${ARMS:-gt_vis onboard}"; SPEED="${SPEED:-97}"; LIDAR_NOISE="${LIDAR_NOISE:-0.02}"
LOCAL_STEPS="${LOCAL_STEPS:-60}"; TTC_JITTER="${TTC_JITTER:-0.15}"; TRACE="${TRACE:-1}"; FOV="${FOV:-}"
rig_args() { [[ -n "${LIDAR_Z:-}" ]] && echo -n "--lidar-z $LIDAR_Z "; [[ -n "${REAR_CAM_X:-}" ]] && echo -n "--rear-camera-x $REAR_CAM_X "; [[ -n "$FOV" ]] && echo -n "--camera-fov $FOV "; [[ -n "${CAM_RES:-}" ]] && echo -n "--camera-resolution $CAM_RES "; [[ -n "${LIDAR_DROPOFF:-}" ]] && echo -n "--lidar-dropoff $LIDAR_DROPOFF "; [[ -n "${TM:-}" ]] && echo -n "--tm-traffic $TM "; [[ -n "${DET_SIZE:-}" ]] && echo -n "--detector-input-size $DET_SIZE "; }
arm_args() { case "$1" in
  gt_vis)  echo "--vehicle-state gt_visible" ;;
  onboard) echo "" ;; esac; }
run_cell() {  # arm closing_speed reps
  local cmd=(test_runner.py -t highway_merge_edge_late_fusion --ego-counts 1 --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0
             --latency-distribution fixed --speeds "$SPEED" --repetitions "$3" --per-tick-age-log --self-id-radius 1
             --carla-port "$PORT" --stamp "${STAMP}_$1_cs$2" $(arm_args "$1") $(rig_args) --lidar-noise-std "$LIDAR_NOISE" --local-prediction-steps "$LOCAL_STEPS"
             --edge-kv prediction_steps=60 closing_speed_mps="$2" --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env CLOSING_SPEED_MPS="$2" CUTIN_TTC_JITTER_S="$TTC_JITTER" $([[ "$TRACE" == 1 ]] && echo PERC_TRACE=1) ${HM_ENV:-})
  echo "$(date '+%F %T') arm=$1 cs=$2 reps=$3"
  local t0=$(date +%s)
  "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3
  echo "$(date '+%F %T') arm=$1 cs=$2 wall $(( $(date +%s) - t0 )) s"
}
if pgrep -f "^$PY test_runner.py" >/dev/null || pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
  echo "a runner or CARLA is active; aborting"; exit 1; fi
if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; exit 1; fi
mkdir -p experiment_results/campaign_meta
{ echo "stamp=$STAMP reps=$REPS cs=$CS arms=$ARMS speed=$SPEED ttc_jitter=$TTC_JITTER lidar_noise=$LIDAR_NOISE fov=$FOV lidar_z=${LIDAR_Z:-} rear_cam_x=${REAR_CAM_X:-} hm_env=${HM_ENV:-}"; cat CODE_VERSION 2>/dev/null || git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_cutin.txt"
for cs in $CS; do for a in $ARMS; do run_cell "$a" "$cs" "$REPS"; done; done
for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
echo "$(date '+%F %T') sweep end"
