#!/usr/bin/env bash
# j6f cetus (2026-10-30 14:4x): robustness of the recalibrated 70 km/h straight-crossing encounter one age step below the
# reference budget (1,500 ms) under imperfect information: uplink packet loss, ego GNSS noise and dropout,
# timestamp noise on the consumed messages (the planner re-indexes forecasts by the carried timestamp), and a
# 4 m beacon position fault. Reference, contract (anchoring on) and naive (anchoring off), ten seeds.
# Stage V first: every cell at zero age, one seed per arm, as the pre-launch gate (tracebacks or missing metrics
# abort the block; outcomes are printed for review). Waits for the j6 speed chain (pid 4146153).
# Log: logs/j6f_cetus.log. Stamp sensys_nov27_j6f.
set -uo pipefail
cd "$HOME/sensys_wt"
CHAIN=2784684
while kill -0 "$CHAIN" 2>/dev/null; do sleep 60; done
while pgrep -f "^/home/tlandle/anaconda3/envs/opencda310/bin/python test_runner.py" >/dev/null; do sleep 30; done
sleep 20
while pgrep -f '^[^ ]*/CarlaUE4-Linux-Shipping' >/dev/null; do sleep 10; done
if pgrep -f "^/home/tlandle/anaconda3/envs/opencda310/bin/python test_runner.py" >/dev/null; then echo "a runner is still active; aborting"; exit 1; fi
if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; ls ecav/scenario_testing/config_yaml/*.yaml.bak; exit 1; fi
PY="$HOME/anaconda3/envs/opencda310/bin/python"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
PORT=2000
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
SCP=$("$PY" -c "import json;m=json.load(open('/home/tlandle/j6_maps.json'))['scp'];print(','.join(f'{int(float(k))}:{v:.2f}' for k,v in sorted(m.items(),key=lambda x: float(x[0]))))")
ENV_SCP="CROSS_TRIGGER_TTA=$SCP HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900"
COMMON="--ego-counts 1 --speeds 70 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port $PORT --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0"
STAMP=sensys_nov27_j6f
T=scp_edge_late_fusion
echo "$(date '+%F %T') j6f cetus starts at $(git rev-parse --short HEAD); SCP map $SCP"
# one cell = extra runner arguments; the edge-kv always carries prediction_steps=60
CELLS=(
  "--packet-loss 10 --edge-kv prediction_steps=60"
  "--packet-loss 30 --edge-kv prediction_steps=60"
  "--gps-noise 1.5e-5 --edge-kv prediction_steps=60"
  "--gps-noise 3e-5 --edge-kv prediction_steps=60"
  "--gps-dropout 20 --edge-kv prediction_steps=60"
  "--gps-dropout 50 --edge-kv prediction_steps=60"
  "--timestamp-noise 100 --edge-kv prediction_steps=60"
  "--timestamp-noise 300 --edge-kv prediction_steps=60"
  "--edge-kv beacon_fault_pos_noise_m=4.0 prediction_steps=60"
)
ARMS=("oracle on" "late_fusion on" "late_fusion off")
run_cells () {  # $1 = latencies, $2 = repetitions, $3 = stage label
  LAT=$1; REP=$2; STAGE=$3
  for cell in "${CELLS[@]}"; do for arm in "${ARMS[@]}"; do set -- $arm
    echo "$(date '+%F %T') [$STAGE] arm=$1/$2 cell=[$cell]"
    "$PY" test_runner.py -t $T --manager-types $1 --anchoring $2 $COMMON --latencies $LAT --repetitions $REP --stamp "$STAMP" $cell --scenario-env $ENV_SCP
  done; done
}
run_cells 0 1 V
echo "$(date '+%F %T') j6f stage V done"
if grep -q Traceback logs/j6f_cetus.log; then echo "stage V has a traceback; block not launched"; exit 1; fi
NV=$(find experiment_results/$T/$STAMP -name simulation_metrics.json -path '*lat_0_*' | grep -v evaluation_output | wc -l)
echo "stage V metrics files: $NV of 27"
if [ "$NV" -lt 27 ]; then echo "stage V incomplete; block not launched"; exit 1; fi
PYTHONPATH=.:ecav/protos "$PY" scripts/sensys_analysis/envelope_measured.py $STAMP --out safety_envelope_paper/analysis/envelope_j6f 2>&1 | tail -3
"$PY" - <<'PY'
import csv
print("stage V outcomes (geom speed mgr anch lat cell -> collided self_ghost false_brake stalled)")
for r in csv.DictReader(open("safety_envelope_paper/analysis/envelope_j6f/runs.csv")):
    print(r['geom'][:4], r['speed'], r['mgr'], r['anch'], r['lat'], r['cell'][-70:], '->', r['collided'], r['self_ghost'], r['false_brake'], r['stalled'])
PY
run_cells 1500 10 B
echo "$(date '+%F %T') j6f cetus done"
