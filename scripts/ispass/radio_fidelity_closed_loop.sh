#!/usr/bin/env bash
# ISPASS experiment 4, closed loop: the intermediate-fusion left turn (70 km/h, measured
# compute) with the radio resolved by the stored ns-3 tables or by the live ns-3 process,
# at two sender counts. Uplink and downlink message size is fixed (DL_BYTES, 16896) for both models.
#
#   scripts/ispass/radio_fidelity_closed_loop.sh gate|full [--dry-run]
#   STAMP=ispass_rfcl REPS=10 NS="16 24" MODELS="table table_loss live" scripts/ispass/radio_fidelity_closed_loop.sh full
#
#   table       tables, delay only (what the edge managers do with use_ns3_lut)
#   table_loss  tables, delay and the table's delivery ratio (lut_loss=true)
#   live        radio=live: one ns-3 step per simulation step, lut_n senders, the connected vehicle in
#               sender slot 0. live:<k> puts it in slot k (the ns-3 scheduler serves senders by slot).
# The table arm records the execution time of every forecast per repetition
# (experiment_results/campaign_meta/<stamp>_compute_pool/n<N>) and the live arms replay the record
# (compute_delay=replay), so both radio models charge the same execution times: with the ns-3
# step ahead of the GPU pass, the measured time of a live run is lower than in a table run.
set -uo pipefail
STAGE="${1:?gate|full}"; DRY="${2:-}"
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_rfcl}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-10}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
NS="${NS:-16 24}"; MODELS="${MODELS:-table table_loss live}"; DL_BYTES="${DL_BYTES:-16896}"; LATS="${LATS:-0}"
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,800525283833
model_kv() { case "$1" in
  table)      echo "" ;;
  table_loss) echo "lut_loss=true" ;;
  live*)      local k="${1#live}"; k="${k#:}"; echo "radio=live live_idx=${k:-0}" ;; esac; }
POOL="${POOL:-$PWD/experiment_results/campaign_meta/${STAMP}_compute_pool}"
run_cell() {  # model n reps
  # live: use_ns3_lut=true is the manager's default; leaving it out keeps the run directory name under 255 bytes
  local ckv="compute_delay=measured use_ns3_lut=true"; [[ "$1" == live* ]] && ckv="compute_delay=replay"
  local cmd=(test_runner.py -t openscenario_3_edge_late_fusion --baselines wf --anchoring on --latencies $LATS
             --latency-distribution fixed --speeds 70 --ego-counts 1 --repetitions "$3" --per-tick-age-log
             --self-id-radius 1 --carla-port "$PORT" --stamp "$STAMP"
             --edge-kv prediction_steps=60 $ckv lut_n=$2 lut_plus_configured=true
                       msg_b=$DL_BYTES $(model_kv "$1")
             --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env CROSS_TRIGGER_TTA=70:4.77 HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900)
  # live ns-3 steps take minutes of wall time at start-up; the scenario runner's 10 s watchdog would end the cross traffic
  [[ "$1" == live* ]] && cmd+=(SR_TIMEOUT_S=3600 ECAV_COMPUTE_POOL="$POOL/n$2")
  [[ "$1" == table ]] && cmd+=(ECAV_COMPUTE_POOL_RECORD="$POOL/n$2")
  echo "$(date '+%F %T') $1 n=$2 reps=$3"
  if [[ "$DRY" == "--dry-run" ]]; then echo "$PY ${cmd[*]}"; return; fi
  local t0=$(date +%s)
  "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3
  echo "$(date '+%F %T') $1 n=$2 wall $(( $(date +%s) - t0 )) s"
  for p in $(pgrep -f "^$HOME/ns3_cv2x/ns-3-dev-5glena/build[a-z-]*/scratch/ns3-dev-nr_v2x_cosim_server[a-z-]* --shm-path=/dev/shm/ecav_ns3_cosim_[0-9]+_edge"); do kill -9 "$p"; done
}
if [[ "$DRY" != "--dry-run" ]]; then
  if pgrep -f "^$PY test_runner.py" >/dev/null || pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
    echo "a runner or CARLA is active; aborting"; exit 1; fi
  if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; exit 1; fi
  mkdir -p experiment_results/campaign_meta
  { echo "stamp=$STAMP stage=$STAGE reps=$REPS ns=$NS models=$MODELS msg_b=$DL_BYTES"; git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_radio_fidelity.txt"
fi
case "$STAGE" in
  gate) for m in $MODELS; do for n in $NS; do run_cell "$m" "$n" 1; done; done ;;
  full) for n in $NS; do for m in $MODELS; do run_cell "$m" "$n" "$REPS"; done; done ;;
  *) echo "unknown stage"; exit 2 ;;
esac
if [[ "$DRY" != "--dry-run" ]]; then
  for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
  for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
  echo "$(date '+%F %T') $STAGE end"
fi
