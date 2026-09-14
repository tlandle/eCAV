#!/bin/bash
# freeze-1r closed-loop STATE-DEPTH block (Figure 4): vary only the transferred
# state at the handoff (none, pos_vel, hist2, hist5, hist10, no_handoff) on the
# flow scenario family, per maneuver, n >= 20 seeds per cell; same trigger,
# boundary, network, load and seeds in every arm; every arm charged the full
# record's modelled transfer delay (XFER_CHARGE_BYTES). Design:
# docs/agent_plans/state_depth_closed_loop_design.md. Runs ONLY after Tyler's
# fix-shape ruling: PRED_OVERLAY must be the verified 1q predictor.
# Usage: MANEUVERS="constant" SEEDS="$(seq 1 20)" PRED_OVERLAY=mtr_edge_predictor_1q.py scripts/atlas_state_depth.sh
set +u
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
CARLA_ROOT=/home/atlas/carla-0.9.15
SEEDS=${SEEDS:-$(seq 1 20)}
ARMS=${ARMS:-"none pos_vel hist2 hist5 hist10 no_handoff"}
MANEUVERS=${MANEUVERS:-"constant"}
FIXED_SPEED=${FIXED_SPEED:-12}
CHARGE_BYTES=${XFER_CHARGE_BYTES:-816}     # hist10's declared payload bytes on this lineage (record_wire_sizes.json)
[ -n "$PRED_OVERLAY" ] || { echo "ABORT: PRED_OVERLAY unset (the block runs only under the verified 1q predictor)"; exit 1; }
exec 9>/tmp/atlas_1l_central.lock; flock -w 43200 9 || { echo "ABORT: lock"; exit 1; }
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
cd "$WT" || exit 1
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_gt.py|runner_wired.py"
  "ecav/scenario_testing/openscenario_1_flow_single.py|runner_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|payload_wired.py"
  "ecav/core/application/edge/migration/daemon.py|daemon_1l.py"
  "ecav/core/application/edge/migration/link.py|link_1r.py"
  "ecav/scenario_testing/scenarios/scenario_1.py|scenario_1_1l.py"
  "ecav/core/plan/behavior_agent.py|behavior_agent_1m.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|${PRED_OVERLAY}"
)
BK=$(mktemp -d); DESTS=()
restore () { for d in "${DESTS[@]}"; do cp "$BK/$(echo "$d" | tr / _)" "$d"; done; }
trap restore EXIT
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; [ -f "$SP/$s" ] || { echo "ABORT missing overlay $s"; exit 1; }; cp "$d" "$BK/$(echo "$d" | tr / _)"; DESTS+=("$d"); cp "$SP/$s" "$d"; done
grep -q "SEEDROW" ecav/scenario_testing/scenarios/scenario_1.py || { echo "ABORT scenario overlay"; exit 1; }
grep -q "MODESROW" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT edge overlay"; exit 1; }
grep -q "XFER_CHARGE_BYTES" ecav/core/application/edge/migration/link.py || { echo "ABORT link overlay"; exit 1; }
grep -q "RANKROW" ecav/core/prediction/mtr_edge_predictor.py || { echo "ABORT: predictor overlay is not the 1q fix (no RANKROW)"; exit 1; }
R="$WT/evaluation_outputs/${OUTDIR:-frozen_1r_state_depth}"; mkdir -p "$R"
carla_restart () { pkill -f "^$CARLA_ROOT/CarlaUE4" 2>/dev/null; pkill -f "CarlaUE4-Linux-Shipping" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && ./CarlaUE4.sh -RenderOffScreen -nosound >/dev/null 2>&1 & ); sleep 55; }
_degenerate () { grep -qE "RuntimeError|time-out of|Traceback \(most recent call last\)|CarlaUE4 crashed" "$1" 2>/dev/null && ! grep -q RUNROW "$1" 2>/dev/null && return 0; grep -q "\[EGO-DBG\]" "$1" 2>/dev/null || return 0; return 1; }
arm_env () { case "$1" in
  none) echo "MIGRATION_MODE=cold";;
  pos_vel) echo "MIGRATION_MODE=kf";;
  hist2) echo "MIGRATION_MODE=warm MIGRATION_HIST=2 LOOKAHEAD_S=1";;
  hist5) echo "MIGRATION_MODE=warm MIGRATION_HIST=5 LOOKAHEAD_S=1";;
  hist10) echo "MIGRATION_MODE=warm LOOKAHEAD_S=1";;
  no_handoff) echo "MIGRATION_MODE=warm LOOKAHEAD_S=1";;
  *) echo "ABORT unknown arm $1" >&2; exit 2;; esac; }
maneuver_env () { case "$1" in
  constant) echo "";;                                   # the flow scenario as landed: ONCOMING_SPEED=$FIXED_SPEED, adjacent lane, blocker at x=278
  ltap_od|scp|lane_change) echo "ABORT maneuver $1 not scripted yet (feasibility report pending)" >&2; exit 2;;
  *) echo "ABORT unknown maneuver $1" >&2; exit 2;; esac; }
scenario_for () { [ "$1" = no_handoff ] && echo openscenario_1_flow_single || echo openscenario_1_flow_gt; }
run () { local tag="$1"; local scen="$2"; shift 2; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && grep -q "\[MODESROW\]" "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $scen $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300 XFER_CHARGE_BYTES=$CHARGE_BYTES) MANEUVER_SCENARIO=$scen $@" > "$log"
    ( env ONCOMING_SPEED=$FIXED_SPEED TRIGGER_DIST=300 XFER_CHARGE_BYTES=$CHARGE_BYTES "$@" EVAL_TAG=${EVAL_TAG:-khonsu-eval-freeze-1r} timeout -k 30 900 python ecav.py -t "$scen" --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW; STOPPING block"; exit 2; }
  grep -q "\[MODESROW\]" "$log" 2>/dev/null || { echo "[HARD FAIL] $tag 0 MODESROW; STOPPING"; exit 2; }
  python "$MAIN/scripts/diag/check_mode_diversity.py" "$log" --require_rows PADROW,RANKROW || { echo "[HARD FAIL] $tag mode diversity gate; STOPPING"; exit 3; }; }
echo "[state-depth] $(date '+%F %T') maneuvers: $MANEUVERS arms: $ARMS seeds: $(echo $SEEDS | wc -w) predictor: $PRED_OVERLAY charge_bytes: $CHARGE_BYTES out: $R"
for mv in $MANEUVERS; do menv=$(maneuver_env $mv) || exit 2
  for s in $SEEDS; do for arm in $ARMS; do
    run "sd_${mv}_${arm}_r${s}" "$(scenario_for $arm)" $(arm_env $arm) $menv MANEUVER=$mv KHONSU_SEED=$s
  done; done
done
touch "$R/_state_depth_done"; echo "[state-depth] DONE $(date '+%F %T')"
