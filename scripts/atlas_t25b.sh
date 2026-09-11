#!/bin/bash
set +u
# T25b GEOMETRY SWEEP (freeze-1n, atlas) = the confound check for the regions
# figure. Varies ONLY the boundary/timing (ONCOMING_SHIFT_X phase-shifts the
# oncoming spawn; the ego approach is identical) at FIXED oncoming speed, so
# t_avail moves while the traffic situation is held constant. The speed sweep
# (atlas_t25_1m.sh) confounds t_avail with traffic difficulty (success falls as
# t_avail rises); T25b isolates t_avail. rules_regions_geometry requires each
# geometry point's Wilson interval to overlap the nearest speed-sweep point for
# the same arm so the two sweeps collapse onto one curve.
# Shifts {-14,-7,0,7,21} map (generated) to t_avail ~{1.0,1.6,2.0,2.5,3.6},
# spanning the 1.0-3.5 rule; realized t_avail is measured per shift, not assumed
# (CALIB=1 runs the 5-shift warm-only probe first to confirm the span).
# Env: SHIFTS, SEEDS, ARMS, FIXED_SPEED; CALIB=1 -> warm x {shifts} x seed 1.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
SHIFTS=${SHIFTS:-"-14 -7 0 7 21"}
SEEDS=${SEEDS:-"1 2 3 4 5"}
ARMS=${ARMS:-"cold kf reactive warm oracle"}
FIXED_SPEED=${FIXED_SPEED:-12}
# CALIB probe (5 runs): warm x 5 shifts x seed 1 with BEHAVIOR_DEBUG on, so it
# reports per shift: realized t_avail, the gate's ttc at the launch decision (is
# it blind to the oncoming across shifts too?), and the gate's track list ([OT
# SIGHT]/[OT PREDS], the association answer). Report all three before the 125.
BD=""
if [ "${CALIB:-0}" = "1" ]; then SHIFTS="-14 -7 0 7 21"; SEEDS="1"; ARMS="warm"; BD="BEHAVIOR_DEBUG=1"; fi
exec 9>/tmp/atlas_1l_central.lock
flock -w 43200 9 || { echo "[t25b] lock wait timed out"; exit 1; }
echo "[t25b] lock acquired $(date +%H:%M:%S) shifts='$SHIFTS' seeds='$SEEDS' arms='$ARMS' speed=$FIXED_SPEED calib=${CALIB:-0}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
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
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/t25b_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/t25b_${i}.bak" "$d"; i=$((i+1)); done
  git checkout -q "${DESTS[@]}" 2>/dev/null; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; cp "$SP/$s" "$d"; done
grep -q "SEEDROW" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT paired seed not applied"; exit 1; }
grep -q "ONCOMING_SHIFT_X" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT shift knob missing"; exit 1; }
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest"; exit 1; }
R="$WT/evaluation_outputs/frozen_1n_t25b"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
# defaults BEFORE "$@" so the per-run ONCOMING_SPEED/SHIFT overrides win.
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag (done)"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300) $BD then $@" > "$log"
    ( env ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300 $BD "$@" EVAL_TAG=khonsu-eval-freeze-1n timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate (startup transient)"
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW after $attempt attempts; STOPPING block"; exit 2; }; }
arm_env () { case "$1" in
    cold)     echo "MIGRATION_MODE=cold" ;;
    kf)       echo "MIGRATION_MODE=kf" ;;
    reactive) echo "MIGRATION_MODE=reactive" ;;
    warm)     echo "MIGRATION_MODE=warm LOOKAHEAD_S=1" ;;
    oracle)   echo "MIGRATION_MODE=warm TRIGGER_MODE=oracle" ;;
  esac; }
# seed-major: an interrupted grid is complete-but-coarse (n=1, then n=2, ...).
for s in $SEEDS; do for shift in $SHIFTS; do for md in $ARMS; do
  run "t25b_${md}_x${shift}_s${s}" $(arm_env $md) ONCOMING_SPEED=$FIXED_SPEED ONCOMING_SHIFT_X=$shift KHONSU_SEED=$s
done; done; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "T25b shifts='$SHIFTS' seeds='$SEEDS' arms='$ARMS' speed=$FIXED_SPEED $(date +%H:%M:%S)" > "$R/_t25b_done"
echo "[t25b] DONE $(date +%H:%M:%S)"
