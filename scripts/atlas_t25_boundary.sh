#!/bin/bash
set +u
# T25 BOUNDARY SWEEP under freeze-1p (regions figure x-axis replacement).
#
# WHY THIS EXISTS. The freeze-1n speed sweep cannot isolate T_avail. Oncoming
# speed sets preparation time (T_avail = d/v, faster = less) AND how fast the
# hazard clears the conflict (faster = easier), and the second effect dominates:
# measured no-collision rate for the no-transfer arm RISES 0.20 -> 1.00 from
# 8 to 24 m/s, opposite to the figure's model. At 8 m/s a perfect crossing
# trigger (oracle, 0.60) scores below a velocity snapshot (kf, 0.80), which is
# the tell that the outcome is not the trigger's.
#
# This block holds oncoming speed fixed and moves the LOCALE BOUNDARY instead.
# That changes the distance from the NPC's locale crossing to the conflict, and
# so T_avail, while vehicle speeds, spawn phase, road geometry and the conflict
# itself are untouched. Measured conflict x ~ 286 (blocked egos stall at
# 286-292 in the frozen_1n logs); the patcher refuses a boundary past it.
#
# T_avail at ONCOMING_SPEED=12: b=220 -> 5.5s, 235 -> 4.2s, 250 -> 3.0s,
# 262 -> 2.0s, 275 -> 0.9s. Widths are held fixed (see WIDTHS) so fusion cost
# does not drift with the boundary; b=250 reproduces the current configuration.
#
# Adapted from atlas_t25_1m.sh (same scenario
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
# Env: BOUNDS (default "170 200 230 255 275"), SEEDS (default "1 2 3 4 5"),
#      ARMS (default "cold kf reactive warm oracle"), ONCOMING_SPEED (default 12),
#      SMOKE=1 -> warm b250 s1 only (the baseline-reproducing point).
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
BOUNDS=${BOUNDS:-"220 235 250 262 275 277 280 282"}   # + sub-second points (Figure 5 region; design t25_boundary_extension_design.md)
MOTIONS=${MOTIONS:-"constant maneuvering"}
# maneuvering motion = the accel panel's operating point; ONCOMING_ACCEL_ONSET must be
# given (the probed onset), the run refuses a maneuvering motion without it.
MANEUVER_ENV=${MANEUVER_ENV:-"ONCOMING_CRUISE=5 ONCOMING_ACCEL=2 ONCOMING_VCAP=16"}
[ -n "$PRED_OVERLAY" ] || { echo "ABORT: PRED_OVERLAY unset (the block runs only under the verified 1q predictor)"; exit 1; }
ONC=${ONCOMING_SPEED:-12}
# WIDTHS="w1,w0" holds both locale widths fixed and translates the partition, so
# the destination's fusion cost does not drift along the sweep. Empty = simple
# sweep (shared edge moves, both locales resize). 170,120 makes b=250 reproduce
# the current configuration exactly, giving the sweep an anchor against existing data.
WIDTHS=${WIDTHS:-"170,120"}
SEEDS=${SEEDS:-"1 2 3 4 5"}
ARMS=${ARMS:-"cold kf reactive warm oracle"}
if [ "${SMOKE:-0}" = "1" ]; then BOUNDS="250"; SEEDS="1"; ARMS="warm"; fi
exec 9>/tmp/atlas_1l_central.lock
flock -w 14400 9 || { echo "[t25-1m] lock wait timed out"; exit 1; }
echo "[t25-bnd] lock acquired $(date +%H:%M:%S) bounds='$BOUNDS' widths='$WIDTHS' onc=$ONC seeds='$SEEDS' arms='$ARMS' smoke=${SMOKE:-0}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze-1h"; exit 1; }
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_gt.py|runner_wired.py"
  "ecav/scenario_testing/openscenario_1_accel_gt.py|runner_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|payload_wired.py"
  "ecav/core/application/edge/migration/daemon.py|daemon_1l.py"
  "ecav/scenario_testing/scenarios/scenario_1.py|scenario_1_1l.py"
  "ecav/core/plan/behavior_agent.py|behavior_agent_1m.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|${PRED_OVERLAY}"
)
YAML=ecav/scenario_testing/config_yaml/openscenario_1_flow_gt.yaml
YAML_ACC=ecav/scenario_testing/config_yaml/openscenario_1_accel_gt.yaml
cp "$YAML" /tmp/t25bnd_yaml.bak; cp "$YAML_ACC" /tmp/t25bnd_yaml_acc.bak
set_boundary () { cp /tmp/t25bnd_yaml.bak "$YAML"; cp /tmp/t25bnd_yaml_acc.bak "$YAML_ACC"
  python3 "$MAIN/scripts/set_locale_boundary.py" "$YAML" "$1" $WIDTHS || exit 3
  python3 "$MAIN/scripts/set_locale_boundary.py" "$YAML_ACC" "$1" $WIDTHS || exit 3; }
DESTS=()
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/t25m_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/t25m_${i}.bak" "$d"; i=$((i+1)); done
  cp /tmp/t25bnd_yaml.bak "$YAML" 2>/dev/null; cp /tmp/t25bnd_yaml_acc.bak "$YAML_ACC" 2>/dev/null
  git checkout -q "${DESTS[@]}" 2>/dev/null; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; cp "$SP/$s" "$d"; done
grep -q "SEEDROW" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT paired seed not applied"; exit 1; }
grep -q "RANKROW" ecav/core/prediction/mtr_edge_predictor.py || { echo "ABORT: predictor overlay is not the 1q fix (no RANKROW)"; exit 1; }
# model-integrity guard (peer 57af285a): abort if this host's weights differ from
# the manifest, so a block never runs on a divergent visual backbone / checkpoint.
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest; results from this host are not comparable"; exit 1; }
R="$WT/evaluation_outputs/${OUTDIR:-frozen_1q_bnd}"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[GT INJECT DBG\]" "$lg" 2>/dev/null || return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
# defaults BEFORE "$@" so the per-run ONCOMING_SPEED overrides (the clobber fix).
run () { local tag="$1"; local scen="$2"; shift 2; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag (done)"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=$ONC TRIGGER_DIST=300) boundary=$CURB scenario=$scen then $@" > "$log"
    ( env ONCOMING_SPEED=$ONC TRIGGER_DIST=300 "$@" EVAL_TAG=${EVAL_TAG:-khonsu-eval-freeze-1q} timeout -k 30 900 python ecav.py -t "$scen" --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate (startup transient)"
  done
  # HARD FAIL: a cell that exits without its completion marker (RUNROW) means the
  # run was killed/crashed (e.g. a wrong-sized timeout). Stop the block loudly
  # rather than move on and silently produce zero usable rows.
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag produced no RUNROW after $attempt attempts (timeout/crash); STOPPING block"; exit 2; }
  python "$MAIN/scripts/diag/check_mode_diversity.py" "$log" --require_rows PADROW,RANKROW || { echo "[HARD FAIL] $tag mode diversity gate; STOPPING"; exit 3; }; }
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
for s in $SEEDS; do for b in $BOUNDS; do
  CURB=$b; set_boundary "$b"
  for motion in $MOTIONS; do
    if [ "$motion" = maneuvering ]; then
      echo "$MANEUVER_ENV" | grep -q "ONCOMING_ACCEL_ONSET=" || { echo "ABORT: maneuvering motion needs ONCOMING_ACCEL_ONSET in MANEUVER_ENV (the probed onset)"; exit 1; }
      SCEN=openscenario_1_accel_gt; PFX=bndm; MENV="$MANEUVER_ENV"
    else SCEN=openscenario_1_flow_gt; PFX=bnd; MENV=""; fi
    for md in $ARMS; do
      run "${PFX}_${md}_b${b}_s${s}" "$SCEN" $(arm_env $md) $MENV ONCOMING_SPEED=$ONC KHONSU_SEED=$s
    done
  done
done; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "T25-bnd bounds='$BOUNDS' onc=$ONC seeds='$SEEDS' arms='$ARMS' $(date +%H:%M:%S)" > "$R/_t25_bnd_done"
echo "[t25-bnd] DONE $(date +%H:%M:%S)"
