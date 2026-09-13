#!/bin/bash
set +u
# T25 SPEED SWEEP under freeze-1n (regions figure, TOP panel = constant-speed
# oncoming). Adapted from the canonical cetus_t25_rerun.sh (same scenario
# openscenario_1_flow_gt, same lander khonsu_t25_land.py, same defaults-BEFORE-$@
# speed-clobber fix). Differences required by freeze-1n:
#   - runs on atlas in the idfix_wt worktree at the frozen base 71c9f37e, with the
#     freeze-1n overlay applied (cp COMMON -> base, trap restore); the base is not
#     modified. This carries Defect B (coast-through-blind-window), the ghost-filter
#     fix and the accel-consumption fix, which is what lets warm beat kf/cold.
#   - passes KHONSU_SEED=$s so the "seeds" are the PAIRED spawn-phase samples
#     (identical situation across arms per index), not timing-jitter reps.
#   - fifth arm: GT-crossing trigger = MIGRATION_MODE=warm TRIGGER_MODE=oracle,
#     tagged t25_oracle_* (needs "oracle" added to the lander's migrated set to
#     read crossing_tick from HANDOFFROW; flagged to the figure owner).
# Env: SPEEDS (default "6 8 12 16 20 24"), SEEDS (default "1 2 3 4 5"),
#      ARMS (default "cold kf reactive warm oracle"), SMOKE=1 -> warm v12 s1 only.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
# Sweep floor is 8 m/s: below it the oncoming stream does not clear within the
# scenario, so 6 m/s measured the scenario (all arms fail) not the method; dropped
# (peer 2026-09-10). Already-collected v6 rows stay on disk, out of the figure.
SPEEDS=${SPEEDS:-"8 12 16 20 24"}
SEEDS=${SEEDS:-"1 2 3 4 5"}
ARMS=${ARMS:-"cold kf reactive warm oracle"}
if [ "${SMOKE:-0}" = "1" ]; then SPEEDS="12"; SEEDS="1"; ARMS="warm"; fi
exec 9>/tmp/atlas_1l_central.lock
flock -w 172800 9 || { echo "[t25-1m] lock wait timed out"; exit 1; }
echo "[t25-1m] lock acquired $(date +%H:%M:%S) speeds='$SPEEDS' seeds='$SEEDS' arms='$ARMS' smoke=${SMOKE:-0}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze-1h"; exit 1; }
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_gt.py|runner_wired.py"
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
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/t25m_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/t25m_${i}.bak" "$d"; i=$((i+1)); done
  git checkout -q "${DESTS[@]}" 2>/dev/null; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; cp "$SP/$s" "$d"; done
grep -q "SEEDROW" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT paired seed not applied"; exit 1; }
# model-integrity guard (peer 57af285a): abort if this host's weights differ from
# the manifest, so a block never runs on a divergent visual backbone / checkpoint.
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest; results from this host are not comparable"; exit 1; }
R="$WT/evaluation_outputs/${OUTDIR:-frozen_1n_t25}"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[GT INJECT DBG\]" "$lg" 2>/dev/null || return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
# defaults BEFORE "$@" so the per-run ONCOMING_SPEED overrides (the clobber fix).
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag (done)"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=12 TRIGGER_DIST=300) then $@" > "$log"
    ( env ONCOMING_SPEED=12 TRIGGER_DIST=300 "$@" EVAL_TAG=${EVAL_TAG:-khonsu-eval-freeze-1n} timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate (startup transient)"
  done
  # HARD FAIL: a cell that exits without its completion marker (RUNROW) means the
  # run was killed/crashed (e.g. a wrong-sized timeout). Stop the block loudly
  # rather than move on and silently produce zero usable rows.
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag produced no RUNROW after $attempt attempts (timeout/crash); STOPPING block"; exit 2; }; }
arm_env () { case "$1" in
    cold)     echo "MIGRATION_MODE=cold" ;;
    kf)       echo "MIGRATION_MODE=kf" ;;
    reactive) echo "MIGRATION_MODE=reactive" ;;
    warm)     echo "MIGRATION_MODE=warm LOOKAHEAD_S=1" ;;
    oracle)   echo "MIGRATION_MODE=warm TRIGGER_MODE=oracle" ;;
  esac; }
# seed-major: each completed seed gives a full speed x arm sweep, so an
# interrupted grid is a complete-but-coarse figure (n=1, then n=2, ...) rather
# than a figure missing whole speeds.
for s in $SEEDS; do for spd in $SPEEDS; do for md in $ARMS; do
  run "t25_${md}_v${spd}_s${s}" $(arm_env $md) ONCOMING_SPEED=$spd KHONSU_SEED=$s
done; done; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "T25-1m speeds='$SPEEDS' seeds='$SEEDS' arms='$ARMS' $(date +%H:%M:%S)" > "$R/_t25_1m_done"
echo "[t25-1m] DONE $(date +%H:%M:%S)"
