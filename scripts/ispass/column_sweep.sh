#!/usr/bin/env bash
# ISPASS: does the difference between ground-truth state and sensor-derived perception grow with the number of
# moving, sensor-equipped vehicles that have to react to each other.
#
# N vehicles approach the straight crossing in one column (scp_column*.yaml, scripts/ispass/gen_column_scenario.py):
# vehicle 1 meets the crossing car as in the density study, vehicles 2..N follow it, 10 m apart at the start, on the
# same lane (from the ninth vehicle on in the second lane). Every vehicle drives on its own state. No edge
# assistance. Arms, applied to every vehicle of the column:
#   gt_vis   simulator ground truth clipped to the vehicle's LiDAR range and line of sight
#   onboard  camera and LiDAR detection and tracking on the vehicle
# N is the --ego-counts knob of test_runner; each cell has its own stamp, <STAMP>_<arm>_n<N>.
#
#   STAMP=ispass_col1 scripts/ispass/column_sweep.sh gate          # GATE_REPS runs per cell (default 1)
#   STAMP=ispass_col1 REPS=20 scripts/ispass/column_sweep.sh full
#   env: BG  TAIL_Y  NS="1 2 4 8 16"  ARMS="gt_vis onboard"  REPS  GATE_REPS  LIDAR_NOISE  SPEED  EGO_Y  MAX_STEP  TTA  CARLA_PORT
set -u
STAGE="${1:?gate|full}"
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_col1}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-20}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
NS="${NS:-1 2 4 8 16}"; ARMS="${ARMS:-gt_vis onboard}"; LIDAR_NOISE="${LIDAR_NOISE:-0.02}"
SPEED="${SPEED:-40}"; EGO_Y="${EGO_Y:--30}"; MAX_STEP="${MAX_STEP:-1400}"; LOCAL_STEPS="${LOCAL_STEPS:-60}"; TTA="${TTA:-4.8}"
# Variants of the hazard the column meets (each needs a stamp of its own):
#   BG=<n>       background road users of the density study around the junction (BG_TRAFFIC)
#   TAIL_Y=<y>   a stopped vehicle in the column's lane at this y, the tail of a queue (QUEUE_TAIL_Y); the column
#                stops behind it and never reaches the crossing, so the crossing car does not launch.
#                TAIL_LANES=2 also stops a vehicle in the neighbouring lane (for 16 vehicles, two lanes of 8)
BG="${BG:-0}"; TAIL_Y="${TAIL_Y:-}"
TRACE="${TRACE:-0}"   # 1: detection trace of the camera and LiDAR arm (PERC_TRACE, perception_manager)
variant_env() { [[ "$BG" != 0 ]] && echo -n "BG_TRAFFIC=$BG "; [[ -n "$TAIL_Y" ]] && echo -n "QUEUE_TAIL_Y=$TAIL_Y QUEUE_TAIL_LANES=${TAIL_LANES:-1} "; [[ "$TRACE" == 1 ]] && echo -n "PERC_TRACE=1 "; [[ "${COLUMN_TRACE:-0}" == 1 ]] && echo -n "COLUMN_TRACE=1 "; echo -n "${EXTRA_ENV:-}"; }   # EXTRA_ENV: further KEY=VALUE pairs for the scenario process
# signal assemblies of the conflict junction and of the junction the column passes before it (density_sweep.sh)
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276,6036580141546007417,3663578618683831388,12050226332585529187,4930559307141010704,4126283349862587052,7662298469676198566,1410771669709828659,8314392846370487945,585006360711733766,8374940585021404564,8209796319443785099,3273669445552054494
# FOV: horizontal field of view of every vehicle camera in degrees (unset: the code default of 70, which leaves 20
#      degree gaps between the four cameras). GT_CAM=1: the ground-truth arms are also clipped to the camera coverage.
# SYNC_LAUNCH=<speed_mps>: the main process launches the crossing car in simulation time, TTA seconds before the
#      vehicle reaches the conflict point, at this constant speed (the scenario runner's wall-clock trigger is off).
FOV="${FOV:-}"; GT_CAM="${GT_CAM:-0}"; SYNC_LAUNCH="${SYNC_LAUNCH:-}"
# GAP_JITTER=<m>: every follower starts up to this much farther behind its predecessor, drawn per repetition.
GAP_JITTER="${GAP_JITTER:-}"
# SCEN: scenario of the cell (scp_column: straight crossing; ltp_column: left turn at the same junction,
#       scripts/ispass/gen_left_turn_scenario.py). DRIVE_DIST: metres the vehicle drives before the scenario ends.
SCEN="${SCEN:-scp_column}"; DRIVE_DIST="${DRIVE_DIST:-340}"
rig_args() { [[ -n "${LIDAR_Z:-}" ]] && echo -n "--lidar-z $LIDAR_Z "; [[ -n "${REAR_CAM_X:-}" ]] && echo -n "--rear-camera-x $REAR_CAM_X "; [[ -n "$GAP_JITTER" ]] && echo -n "--spawn-gap-jitter $GAP_JITTER "; [[ -n "$FOV" ]] && echo -n "--camera-fov $FOV "; [[ -n "${CAM_RES:-}" ]] && echo -n "--camera-resolution $CAM_RES "; [[ -n "${DET_SIZE:-}" ]] && echo -n "--detector-input-size $DET_SIZE "; [[ "$GT_CAM" == 1 && "$1" != onboard && "$1" != gt_all ]] && echo -n "--gt-camera-coverage"; }
# LEAD_JITTER=<s>: with SYNC_LAUNCH, the lead of each repetition is drawn within plus or minus this value (seeded).
launch_env() { if [[ -n "$SYNC_LAUNCH" ]]; then echo "CROSS_TRIGGER_TTA=$SPEED:0 CROSS_SYNC_LAUNCH=$TTA:$SYNC_LAUNCH CROSS_SYNC_JITTER=${LEAD_JITTER:-0}"; else echo "CROSS_TRIGGER_TTA=$SPEED:$TTA"; fi; }
arm_args() { case "$1" in
  gt_vis)  echo "--vehicle-state gt_visible" ;;
  onboard) echo "" ;; esac; }
run_cell() {  # arm N reps
  local cmd=(test_runner.py -t "$SCEN" --ego-counts "$2" --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0
             --latency-distribution fixed --speeds $SPEED --repetitions "$3" --per-tick-age-log --self-id-radius 1
             --carla-port "$PORT" --stamp "${STAMP}_$1_n$2" $(arm_args "$1") $(rig_args "$1") --lidar-noise-std "$LIDAR_NOISE" --local-prediction-steps "$LOCAL_STEPS"
             --edge-kv prediction_steps=60 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env $(launch_env) HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=$EGO_Y EGO_DRIVE_DISTANCE=$DRIVE_DIST SCENARIO_MAX_STEP=$MAX_STEP OCCLUDERS=0 $(variant_env))
  echo "$(date '+%F %T') arm=$1 N=$2 reps=$3"
  local t0=$(date +%s)
  "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3
  echo "$(date '+%F %T') arm=$1 N=$2 wall $(( $(date +%s) - t0 )) s"
}
if pgrep -f "^$PY test_runner.py" >/dev/null || pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
  echo "a runner or CARLA is active; aborting"; exit 1; fi
if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; exit 1; fi
mkdir -p experiment_results/campaign_meta
{ echo "stamp=$STAMP stage=$STAGE reps=$REPS ns=$NS arms=$ARMS lidar_noise=$LIDAR_NOISE speed=$SPEED ego_y=$EGO_Y local_steps=$LOCAL_STEPS tta=$TTA bg=$BG tail_y=$TAIL_Y max_step=$MAX_STEP fov=$FOV gt_cam=$GT_CAM sync_launch=$SYNC_LAUNCH gap_jitter=$GAP_JITTER"; git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_column.txt"
case "$STAGE" in
  gate) for n in $NS; do for a in $ARMS; do run_cell "$a" "$n" "${GATE_REPS:-1}"; done; done ;;
  full) for n in $NS; do for a in $ARMS; do run_cell "$a" "$n" "$REPS"; done; done ;;
  *) echo "unknown stage"; exit 2 ;;
esac
for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
echo "$(date '+%F %T') $STAGE end"
