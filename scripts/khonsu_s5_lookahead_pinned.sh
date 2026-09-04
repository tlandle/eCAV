#!/bin/bash
# Khonsu design studies S1-S4 (docs/agent_plans/khonsu_design_studies.md).
# Runs sequentially against a live CARLA; each run logs to LOGDIR and the
# extractor turns [RUNROW] + the ego eval dict into CSV rows.
# Usage: KDS_GO=1 REPS=10 bash scripts/khonsu_design_sweep.sh
set -u
cd /home/atlas/TrafficSimulator_eCloud/khonsu_v1_wt
source /home/atlas/anaconda3/etc/profile.d/conda.sh
conda activate opencda310

REPS=${REPS:-10}
LOGDIR=${LOGDIR:-evaluation_outputs/khonsu_design_sweep_$(date +%Y%m%d_%H%M%S)}
CARLA_ROOT=${CARLA_ROOT:-$HOME/carla-0.9.15}
RESTART_EVERY=${RESTART_EVERY:-1}   # fresh CARLA every N runs
mkdir -p "$LOGDIR"
echo "logdir: $LOGDIR"

carla_ok () {
  # Real liveness: the UE4 binary process (pgrep -f self-matches the
  # invoking shell, which is how the first campaign burned itself) AND
  # the RPC port accepting connections.
  pgrep -x CarlaUE4-Linux- > /dev/null 2>&1 || \
    pgrep -f "CarlaUE4/Binaries" > /dev/null 2>&1 || return 1
  timeout 5 bash -c "exec 3<>/dev/tcp/localhost/2000" 2>/dev/null || return 1
  return 0
}

carla_restart () {
  echo "[carla] restarting"
  pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
  sleep 5
  ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen > /dev/null 2>&1 < /dev/null & )
  for i in $(seq 1 24); do
    sleep 5
    if carla_ok; then
      sleep 15   # settle: the port accepts before the sim can serve clients
      echo "[carla] up"; return 0
    fi
  done
  echo "[carla] FAILED to start"; return 1
}

RUN_N=0
run_one () {
  local tag="$1"; shift
  local log="$LOGDIR/${tag}.log"
  # A prior log counts as done only if the run produced the ego eval
  # (actor_id) alongside RUNROW; RUNROW alone also emits on early crashes.
  if [ -f "$log" ] && grep -q RUNROW "$log" && grep -q "actor_id" "$log"; then
    echo "skip $tag (done)"; return
  fi
  if [ $((RUN_N % RESTART_EVERY)) -eq 0 ] || ! carla_ok; then
    carla_restart || exit 1
  fi
  RUN_N=$((RUN_N + 1))
  echo "[$(date +%H:%M:%S)] $tag"
  env "$@" timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml \
      > "$log" 2>&1 < /dev/null
  sleep 3
  if ! grep -q "actor_id" "$log"; then
    echo "WARN: $tag incomplete, retrying once"
    carla_restart || exit 1
    env "$@" timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml \
        > "$log" 2>&1 < /dev/null
    sleep 3
    grep -q "actor_id" "$log" || echo "WARN: $tag incomplete after retry"
  fi
}

if [ "${KDS_GO:-0}" != "1" ]; then
  echo "dry run: set KDS_GO=1 to execute"; exit 0
fi

for rep in $(seq 1 "$REPS"); do
  for L in 2 3 4; do
    run_one "s5_look${L}_r${rep}" MIGRATION_MODE=warm LOOKAHEAD_S="$L" ONCOMING_SPEED=12 TRIGGER_DIST=300
  done
done
echo "sweep complete: $LOGDIR"
