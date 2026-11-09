#!/usr/bin/env bash
# Highway merge (cut-in) reproduction for the SenSys resubmission: the geometry the evaluation
# export calls highway_merge_edge_late_fusion (97 km/h, closing speeds 4 to 16 m/s at the cut-in,
# three arms, 24 configured-delay cells, 50 seeds). The scenario is scenarios/scenario_highway_merge.py
# on Town06; the design table is scripts/sensys_analysis/highway_merge_design.py.
#
#   scripts/sensys_highway_merge.sh smoke|validate|pilot|full [--dry-run]
#   STAMP=sensys_hm CARLA_PORT=2000 REPS=10 scripts/sensys_highway_merge.sh pilot
#
#   smoke     one exact-state run at zero configured delay, closing speed 10 m/s
#   validate  pre-launch gate (Tyler, Oct 30): every arm at zero age, one stale probe, per closing speed:
#             exact-state baseline at 0 and 1000 ms, V2X2V +SBA at 0, V2X2V no SBA at 0, and no
#             cooperation (uplink loss 100 %) at 0, two seeds each; then highway_merge_check.py prints the table
#   pilot     closing speeds 4, 10, 16 (the paper's Fig. 16 panels), 8 delay cells, REPS seeds, three arms
#   full      the export's grid: 7 closing speeds, 24 delay cells, 50 seeds, three arms (25,200 runs)
#
# The closing speed travels twice: --edge-kv closing_speed_mps=<cs> puts it in the run directory and
# the reducer's kv (the export's schema), --scenario-env CLOSING_SPEED_MPS=<cs> is what the scenario reads.
# HM_ENV adds scenario knobs (e.g. HM_ENV="OCCLUDER=1 CUTIN_DURATION_S=2.5").
set -euo pipefail
STAGE="${1:?smoke|validate|pilot|full}"; DRY="${2:-}"
cd "$(dirname "$0")/.."
STAMP="${STAMP:-sensys_hm}"
PORT="${CARLA_PORT:-2000}"
REPS="${REPS:-10}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
T=highway_merge_edge_late_fusion
CS_ALL="${CS:-4 6 8 10 12 14 16}"   # CS=10 restricts validate/full to one closing speed
CS_PILOT="${CS:-4 10 16}"
LATS_FULL="0 50 100 150 200 250 300 350 400 450 500 550 600 650 700 750 800 850 900 950 1000 1050 1150 1250"
LATS_PILOT="0 100 200 300 400 600 800 1000"
COMMON="--ego-counts 1 --speeds 97 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port $PORT --stamp $STAMP"
kv() { echo "--edge-kv prediction_steps=60 closing_speed_mps=$1 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 --scenario-env CLOSING_SPEED_MPS=$1 ${HM_ENV:-}"; }
run() { if [[ "$DRY" == "--dry-run" ]]; then echo "$PY test_runner.py $*"; else "$PY" test_runner.py "$@"; fi; }
if [[ "$DRY" != "--dry-run" ]]; then
  if pgrep -f "^$PY test_runner.py" >/dev/null; then echo "a runner is still active; aborting"; exit 1; fi
  if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; ls ecav/scenario_testing/config_yaml/*.yaml.bak; exit 1; fi
  mkdir -p experiment_results/campaign_meta
  { echo "stamp=$STAMP stage=$STAGE port=$PORT reps=$REPS hm_env=${HM_ENV:-}"; git rev-parse HEAD; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_highway_merge.txt"
fi
case "$STAGE" in
  smoke)
    run -t $T --manager-types oracle --anchoring on --latencies 0 --repetitions 1 $COMMON $(kv 10) ;;
  validate)
    for cs in $CS_ALL; do
      run -t $T --manager-types oracle --anchoring on --latencies 0 1000 --repetitions 2 $COMMON $(kv $cs)
      run -t $T --manager-types late_fusion --anchoring both --latencies 0 --repetitions 2 $COMMON $(kv $cs)
      run -t $T --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0 --repetitions 2 $COMMON $(kv $cs)
    done
    [[ "$DRY" == "--dry-run" ]] || "$PY" scripts/sensys_analysis/highway_merge_check.py "$STAMP" | tee "experiment_results/campaign_meta/${STAMP}_highway_merge_validate.txt" ;;
  pilot)
    for cs in $CS_PILOT; do for arm in "oracle on" "late_fusion on" "late_fusion off"; do set -- $arm
      run -t $T --manager-types "$1" --anchoring "$2" --latencies $LATS_PILOT --repetitions "$REPS" $COMMON $(kv $cs)
    done; done ;;
  full)
    for cs in $CS_ALL; do for arm in "oracle on" "late_fusion on" "late_fusion off"; do set -- $arm
      run -t $T --manager-types "$1" --anchoring "$2" --latencies $LATS_FULL --repetitions "${REPS:-50}" $COMMON $(kv $cs)
    done; done ;;
  *) echo "unknown stage $STAGE"; exit 2 ;;
esac
