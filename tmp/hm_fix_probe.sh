#!/usr/bin/env bash
# Highway merge diagnosis probes (2026-11-11): exact-state arm at a few delays per closing speed.
# usage: tmp/hm_fix_probe.sh <stamp> "<cs:lat lat ...>" ["<cs:lat ...>" ...]   (HM_ENV, REPS, ARM="oracle on" honoured)
cd "${HM_WT:-$HOME/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox}"
STAMP=$1; shift
PY=$HOME/anaconda3/envs/opencda310/bin/python
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
set -- "$@"; ARM=${ARM:-oracle on}; read M A <<< "$ARM"
for spec in "$@"; do cs=${spec%%:*}; lats=${spec#*:}
  echo "$(date '+%F %T') cs $cs lats $lats arm $M $A"
  $PY test_runner.py -t highway_merge_edge_late_fusion --manager-types $M --anchoring $A --latencies $lats --repetitions ${REPS:-2} \
    --ego-counts 1 --speeds 97 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port 2000 --stamp $STAMP \
    --edge-kv prediction_steps=60 closing_speed_mps=$cs ${EDGE_KV_EXTRA:-} --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 ${BEH_KV_EXTRA:-} \
    --scenario-env CLOSING_SPEED_MPS=$cs ${HM_ENV:-} 2>&1 | grep -E "Sweep completed|abort|Error|Traceback" | tail -3
done
PYTHONPATH=.:ecav/protos $PY scripts/sensys_analysis/highway_merge_check.py $STAMP 2>&1 | grep -v "^gate\|^  cs  " | tail -40
for p in $(pgrep -f "^$HOME/carla-0.9.15/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"); do kill -9 $p; done
for p in $(pgrep -f "^/bin/sh $HOME/carla-0.9.15/CarlaUE4.sh"); do kill -9 $p; done
echo "$(date '+%F %T') probe end"
