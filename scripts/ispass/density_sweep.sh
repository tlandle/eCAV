#!/usr/bin/env bash
# ISPASS: does sensor-derived perception change the closed-loop outcome, and does the difference change with
# traffic density. No edge assistance: every shared message is dropped, the vehicle drives on its own state.
#
# The straight crossing is held fixed (same encounter as fidelity_ablation.sh) and M background road
# users are added around the junction (scenario_3.py, BG_TRAFFIC). Two arms:
#   gt_vis   simulator ground truth clipped to the vehicle's LiDAR range and to line of sight from its roof sensor
#            position (test_runner --vehicle-state gt_visible): the visibility of the sensors, none of their errors
#   onboard  camera and LiDAR detection and tracking on the vehicle
#   gt_all   (optional, ARMS) every vehicle of the simulator, no range or line of sight limit: the state a simulator
#            hands a vehicle when its perception is off
# Two encounters:
#   open     the parked trucks are removed (OCCLUDERS=0): the crossing car is in the vehicle's sensor view
#   occ      the occluded junction: the crossing car is hidden until the junction
# Every LiDAR carries range noise (LIDAR_NOISE, metres, default 0.02; the scenario YAMLs carry 0).
# The arm, the encounter and M are not in the run directory name, so each cell has its own stamp:
# <STAMP>_<encounter>_<arm>_m<M>.
#
#   STAMP=ispass_den1 scripts/ispass/density_sweep.sh gate          # one run per cell
#   STAMP=ispass_den1 REPS=30 scripts/ispass/density_sweep.sh full
#   env: MS="4 8 16 32 64 128"  ARMS="gt_vis onboard"  ENCS="open occ"  REPS  LIDAR_NOISE  SPEED  EGO_Y  MAX_STEP  CARLA_PORT  TRACE  MODEL  NOISE
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
# The connected vehicle starts at EGO_Y and passes a second signalized junction (y -12 to 28) before the conflict junction.
# Its overhead signal heads are detected as a stationary vehicle in the lane (ispass_den2: 55 false stops in 180 runs at
# (-86.1, 29.3), 1.1 m from the head 6036580141546007417). The harness has hidden the four signal assemblies of the conflict
# junction for the same reason since Oct 19 (HIDE above); the four assemblies of the second junction are hidden too.
HIDE=$HIDE,6036580141546007417,3663578618683831388,12050226332585529187,4930559307141010704,4126283349862587052,7662298469676198566,1410771669709828659,8314392846370487945,585006360711733766,8374940585021404564,8209796319443785099,3273669445552054494
# Perception error models between ground truth in view and the full pipeline (ARMS gt_noisy, gt_model): MODEL is the
# table of scripts/ispass/perception_model_fit.py, fitted to the detection trace of onboard runs (TRACE=1 adds the
# trace to the onboard arm). gt_noisy takes the marginal miss probability and position spread of the same table
# (NOISE="p_miss sigma_m" overrides).
MODEL="${MODEL:-paper_ispass/perception_model.json}"; TRACE="${TRACE:-0}"
noise() { [[ -n "${NOISE:-}" ]] && { echo "$NOISE"; return; }
  "$PY" -c "import json,sys; m=json.load(open(sys.argv[1]))['independent']; print(m['p_miss'], m['pos_sigma_m'])" "$MODEL"; }
# FOV: horizontal field of view of every vehicle camera in degrees (unset: the code default of 70, which leaves 20
#      degree gaps between the four cameras). GT_CAM=1: the ground-truth arms are also clipped to the camera coverage.
# SYNC_LAUNCH=<speed_mps>: the main process launches the crossing car in simulation time, TTA seconds before the
#      vehicle reaches the conflict point, at this constant speed (the scenario runner's wall-clock trigger is off).
# LIDAR_PPS: points per second of every vehicle LiDAR (unset: the value of the scenario YAML)
FOV="${FOV:-}"; GT_CAM="${GT_CAM:-0}"; SYNC_LAUNCH="${SYNC_LAUNCH:-}"; LIDAR_PPS="${LIDAR_PPS:-}"
# BG_MODE=tm: the M background road users drive under CARLA's traffic manager (test_runner.py --tm-traffic) and none is
#      parked by the scenario. Unset: the scenario's own background (scenario_3.py, BG_TRAFFIC), mostly standing.
BG_MODE="${BG_MODE:-}"
rig_args() { [[ -n "$FOV" ]] && echo -n "--camera-fov $FOV "; [[ -n "$LIDAR_PPS" ]] && echo -n "--lidar-points-per-second $LIDAR_PPS "; [[ "$GT_CAM" == 1 && "$1" != onboard && "$1" != gt_all ]] && echo -n "--gt-camera-coverage"; }
launch_env() { if [[ -n "$SYNC_LAUNCH" ]]; then echo "CROSS_TRIGGER_TTA=$SPEED:0 CROSS_SYNC_LAUNCH=$TTA:$SYNC_LAUNCH"; else echo "CROSS_TRIGGER_TTA=$SPEED:$TTA"; fi; }
arm_args() { case "$1" in
  gt_vis)   echo "--vehicle-state gt_visible" ;;
  gt_all)   echo "--vehicle-state gt_all" ;;
  gt_noisy) echo "--vehicle-state gt_noisy --perception-noise $(noise)" ;;
  gt_model) echo "--vehicle-state gt_model --perception-model $MODEL" ;;
  onboard)  echo "" ;; esac; }
arm_env() { [[ "$1" == onboard && "$TRACE" == 1 ]] && echo "PERC_TRACE=1"; }
enc_env() { case "$1" in
  open) echo "OCCLUDERS=0" ;;
  occ)  echo "OCCLUDERS=1" ;; esac; }
run_cell() {  # encounter arm M reps
  local cmd=(test_runner.py -t scp_edge_late_fusion --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0
             --latency-distribution fixed --speeds $SPEED --ego-counts 1 --repetitions "$4" --per-tick-age-log --self-id-radius 1
             --carla-port "$PORT" --stamp "${STAMP}_$1_$2_m$3" $(arm_args "$2") $(rig_args "$2") $([[ "$BG_MODE" == tm ]] && echo "--tm-traffic $3") --lidar-noise-std "$LIDAR_NOISE" --local-prediction-steps "$LOCAL_STEPS"
             --edge-kv prediction_steps=60 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env $(launch_env) HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=$EGO_Y EGO_DRIVE_DISTANCE=340 SCENARIO_MAX_STEP=$MAX_STEP
                            BG_TRAFFIC=$([[ "$BG_MODE" == tm ]] && echo 0 || echo "$3") $(enc_env "$1") $(arm_env "$2"))
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
  { echo "stamp=$STAMP stage=$STAGE reps=$REPS ms=$MS arms=$ARMS encs=$ENCS lidar_noise=$LIDAR_NOISE speed=$SPEED ego_y=$EGO_Y local_steps=$LOCAL_STEPS tta=$TTA trace=$TRACE model=$MODEL fov=$FOV gt_cam=$GT_CAM sync_launch=$SYNC_LAUNCH"; git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_density.txt"
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
