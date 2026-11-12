#!/usr/bin/env bash
# ISPASS experiment 2: does modelling the complete system change which architecture is best.
# Three architectures (I2V object sharing, V2X2V prediction with the identity rule, intermediate
# feature fusion), four evaluation conditions, all with sensor-derived state and no added delay:
#   A idealized      compute_delay=zero      ideal delivery
#   B compute-aware  compute_delay=measured  ideal delivery
#   C network-aware  compute_delay=zero      radio delay from the ns-3 table at LUT_N senders
#   D full system    compute_delay=measured  radio delay from the ns-3 table at LUT_N senders
# One live connected vehicle and two roadside units in every run; LUT_N is modelled radio load.
# Every switch is an --edge-kv pair, so it is part of the run directory and of the reducer's kv.
#
#   scripts/ispass/fidelity_ablation.sh smoke|gate|full [--dry-run]
#   STAMP=ispass_fid REPS=30 LUT_NS="16" GEOMS="lt scp" scripts/ispass/fidelity_ablation.sh full
set -uo pipefail
STAGE="${1:?smoke|gate|full}"; DRY="${2:-}"
cd "$(dirname "$0")/../.."
STAMP="${STAMP:-ispass_fid}"; PORT="${CARLA_PORT:-2000}"; REPS="${REPS:-30}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
LUT_NS="${LUT_NS:-16}"; GEOMS="${GEOMS:-lt scp}"; ARCHS="${ARCHS:-i2v v2x2v inter}"; CONDS="${CONDS:-A B C D}"
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
SPEED=70
geom_args() { case "$1" in
  lt)  echo "openscenario_3_edge_late_fusion CROSS_TRIGGER_TTA=${TTA_LT:-70:4.77}" ;;
  scp) echo "scp_edge_late_fusion CROSS_TRIGGER_TTA=${TTA_SCP:-70:5.12}" ;; esac; }
arch_args() { case "$1" in
  i2v)   echo "--manager-types infra_only --anchoring off" ;;
  v2x2v) echo "--manager-types late_fusion --anchoring on" ;;
  inter) echo "--baselines wf --anchoring on" ;; esac; }
cond_kv() { local n="$2"; case "$1" in
  A) echo "compute_delay=zero use_ns3_lut=false lut_dl=false" ;;
  B) echo "compute_delay=measured use_ns3_lut=false lut_dl=false" ;;
  C) echo "compute_delay=zero use_ns3_lut=true lut_dl=true lut_n=$n" ;;
  D) echo "compute_delay=measured use_ns3_lut=true lut_dl=true lut_n=$n" ;; esac; }
run_cell() {  # geom arch cond lut_n reps
  set -- "$@"; local g a; read -r g a <<< "$(geom_args "$1")"
  local cmd=(test_runner.py -t "$g" $(arch_args "$2") --latencies 0 --latency-distribution fixed --speeds $SPEED
             --ego-counts 1 --repetitions "$5" --per-tick-age-log --self-id-radius 1 --carla-port "$PORT" --stamp "$STAMP"
             --edge-kv prediction_steps=60 $(cond_kv "$3" "$4") --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0
             --scenario-env "$a" HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900)
  echo "$(date '+%F %T') $1 $2 $3 n=$4 reps=$5"
  if [[ "$DRY" == "--dry-run" ]]; then echo "$PY ${cmd[*]}"; else "$PY" "${cmd[@]}" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -3; fi
}
if [[ "$DRY" != "--dry-run" ]]; then
  if pgrep -f "^$PY test_runner.py" >/dev/null || pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null; then
    echo "a runner or a simulator is active on this host; aborting"; exit 1; fi
  if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; exit 1; fi
  mkdir -p experiment_results/campaign_meta logs
  { echo "stamp=$STAMP stage=$STAGE reps=$REPS lut_ns=$LUT_NS geoms=$GEOMS archs=$ARCHS conds=$CONDS"; git rev-parse HEAD; hostname; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_fidelity.txt"
fi
case "$STAGE" in
  smoke) for a in $ARCHS; do run_cell lt "$a" D 16 1; done ;;
  gate)  for a in $ARCHS; do for c in $CONDS; do run_cell lt "$a" "$c" 16 2; done; done ;;
  full)  # conditions A and B do not depend on the radio load: run them once
    for g in $GEOMS; do for a in $ARCHS; do
      for c in A B; do [[ " $CONDS " == *" $c "* ]] && run_cell "$g" "$a" "$c" 0 "$REPS"; done
      for n in $LUT_NS; do for c in C D; do [[ " $CONDS " == *" $c "* ]] && run_cell "$g" "$a" "$c" "$n" "$REPS"; done; done
    done; done ;;
  *) echo "unknown stage"; exit 2 ;;
esac
if [[ "$DRY" != "--dry-run" ]]; then
  for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
  for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
  echo "$(date '+%F %T') stage $STAGE end"
fi
