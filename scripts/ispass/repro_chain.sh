#!/usr/bin/env bash
# ISPASS Experiment 5, part A: closed-loop reproducibility with fixed seeds.
#   scripts/ispass/repro_chain.sh            (stamps ispass_rep5_*; SEED, TAG honoured)
# 1. highway cut-in, sensor-derived V2X2V +SBA, 200 ms: two runs without lock step, two with
# 2. highway cut-in, exact state, 200 ms: two runs with lock step
# 3. left turn, intermediate fusion, measured compute, service occupancy on, lock step:
#    two runs with live stamps, two that replay the stamps of the first
# 4. left turn, V2X2V +SBA, measured compute, service occupancy on, lock step: same four
set -uo pipefail
cd "$(dirname "$0")/../.."
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
SEED="${SEED:-11}"; T="${TAG:-ispass_rep5}"; PORT="${CARLA_PORT:-2000}"
TR="$PWD/experiment_results/ispass_repro"; mkdir -p "$TR"
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
run() { echo "$(date '+%F %T') $1"; shift; "$PY" test_runner.py "$@" 2>&1 | grep -E "Sweep completed|abort|Error|Traceback|void" | tail -2; }
hm() {  # stamp manager lockstep
  run "$1" -t highway_merge_edge_late_fusion --manager-types "$2" --anchoring on --latencies 200 --repetitions 1 --seed "$SEED" --stamp "$1" \
      --ego-counts 1 --speeds 97 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port "$PORT" \
      --edge-kv prediction_steps=60 closing_speed_mps=10 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 \
      --scenario-env CLOSING_SPEED_MPS=10 SR_LOCKSTEP="$3"
}
lt() {  # stamp arch-args... timing_mode
  local stamp="$1" mode="$2"; shift 2
  local kv="prediction_steps=60 compute_delay=measured use_ns3_lut=false lut_dl=false service_occupancy=on"
  [[ "$mode" == replay ]] && kv="$kv timing_mode=replay"
  run "$stamp" -t openscenario_3_edge_late_fusion "$@" --anchoring on --latencies 0 --latency-distribution fixed --speeds 70 \
      --ego-counts 1 --repetitions 1 --seed "$SEED" --per-tick-age-log --self-id-radius 1 --carla-port "$PORT" --stamp "$stamp" \
      --edge-kv $kv --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 \
      --scenario-env CROSS_TRIGGER_TTA=70:4.77 HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900 SR_LOCKSTEP=1
}
four() {  # name arch-args...
  local n="$1"; shift
  rm -f "$TR/${T}_${n}_A.jsonl" "$TR/${T}_${n}_B.jsonl"
  ECAV_TIMING_TRACE="$TR/${T}_${n}_A.jsonl" lt "${T}_${n}_liveA" live "$@"
  ECAV_TIMING_TRACE="$TR/${T}_${n}_B.jsonl" lt "${T}_${n}_liveB" live "$@"
  ECAV_TIMING_TRACE="$TR/${T}_${n}_A.jsonl" lt "${T}_${n}_replay1" replay "$@"
  ECAV_TIMING_TRACE="$TR/${T}_${n}_A.jsonl" lt "${T}_${n}_replay2" replay "$@"
}
for s in ${STEPS:-1 2 3 4}; do case "$s" in
  1) hm "${T}_coop_free_A" late_fusion 0; hm "${T}_coop_free_B" late_fusion 0
     hm "${T}_coop_lock_A" late_fusion 1; hm "${T}_coop_lock_B" late_fusion 1 ;;
  2) hm "${T}_exact_lock_A" oracle 1; hm "${T}_exact_lock_B" oracle 1 ;;
  3) four inter --baselines wf ;;
  4) four v2x2v --manager-types late_fusion ;;
esac; done
for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 "$p"; done
for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 "$p"; done
echo "$(date '+%F %T') repro chain end"
