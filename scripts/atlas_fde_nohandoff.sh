#!/bin/bash
set +u
# FDE no-handoff REFERENCE arm (freeze-1n, atlas). Single-locale scenario
# (openscenario_1_flow_nohandoff_gt): one locale spanning the whole path with
# BOTH RSUs (rsu1@325 + rsu2@205), no crossing -> the wired runner fires no
# migration. This is the reference tab_fde quotes the history methods against at
# the 5 s horizon. It is an UPPER BOUND (continuous two-RSU coverage where the
# migrated arms' destination has one), not a like-for-like control (KB). The
# nohandoff .py is a stock copy wired via the overlay (runner_wired). Chained
# behind the FDE 60 (flock on the central lock). VALIDATE=1 -> one cell first, to
# confirm sensing coverage (MODESROW for the oncoming across the whole path, not
# just near one RSU) before the ten, since a naive suppressed version fails when
# the second sensor cannot see the far conflict.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
FIXED_SPEED=${FIXED_SPEED:-12}
if [ "${VALIDATE:-0}" = "1" ]; then SEEDS="1"; fi
exec 9>/tmp/atlas_1l_central.lock
flock -w 43200 9 || { echo "[fde-nohandoff] lock wait timed out"; exit 1; }
echo "[fde-nohandoff] lock acquired $(date +%H:%M:%S) seeds='$SEEDS' speed=$FIXED_SPEED validate=${VALIDATE:-0}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_gt.py|runner_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_nohandoff_gt.py|runner_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|payload_wired.py"
  "ecav/core/application/edge/migration/daemon.py|daemon_1l.py"
  "ecav/scenario_testing/scenarios/scenario_1.py|scenario_1_1l.py"
  "ecav/core/plan/behavior_agent.py|behavior_agent_1m.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|mtr_edge_predictor_1m.py"
)
DESTS=()
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/fdenh_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/fdenh_${i}.bak" "$d"; i=$((i+1)); done
  git checkout -q "${DESTS[@]}" 2>/dev/null; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; cp "$SP/$s" "$d"; done
grep -q "SEEDROW" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT paired seed not applied"; exit 1; }
grep -q "npc_handoff_bytes" ecav/scenario_testing/openscenario_1_flow_nohandoff_gt.py || { echo "ABORT nohandoff.py not wired"; exit 1; }
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest"; exit 1; }
R="$WT/evaluation_outputs/frozen_1n_fde"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && grep -q "\[MODESROW\]" "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300) then $@" > "$log"
    ( env ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300 "$@" EVAL_TAG=khonsu-eval-freeze-1n timeout -k 30 900 python ecav.py -t openscenario_1_flow_nohandoff_gt --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW; STOPPING"; exit 2; }
  grep -q "\[MODESROW\]" "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no MODESROW; STOPPING"; exit 2; }; }
for s in $SEEDS; do
  run "fd_nohandoff_r${s}" MIGRATION_MODE=warm ONCOMING_SPEED=$FIXED_SPEED KHONSU_SEED=$s
done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "FDE-nohandoff seeds='$SEEDS' speed=$FIXED_SPEED $(date +%H:%M:%S)" > "$R/_fde_nohandoff_done"
echo "[fde-nohandoff] DONE $(date +%H:%M:%S)"
