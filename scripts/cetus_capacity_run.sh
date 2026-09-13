#!/bin/bash
set +u
# CAPACITY / BURST block (T8) under freeze-1n on cetus (whole block, one host).
# burst: 10 rep x 3 arms {warm, edgewarp, cold} on openscenario_1_burst_gt.
# q5:    10 rep x 3 FLOW_N {2,4,8} x 3 arms on openscenario_1_flow_gt (concurrent
#        crossings capacity). 30 + 90 = 120 runs. Runs in the cetus_1m_wt worktree
# (overlay already applied; edge is the fixed ghost-filter version). Generous
# timeout (1500s) so a heavy cell is not truncated; the FIRST cell is timed and
# reported before the rest matter, to classify flow-class vs heavy-class.
WT=$HOME/cetus_1m_wt; CARLA_ROOT=$HOME/carla-0.9.15
cd "$WT" || exit 1
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT models mismatch"; exit 1; }
grep -q "PUBGATE_SRC" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT overlay not applied"; exit 1; }
R="$WT/evaluation_outputs/frozen_1p_capacity"; mkdir -p "$R"; rm -f "$R"/_done
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"; grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0; return 1; }
_mode_args () { case "$1" in
    warm) echo "MIGRATION_MODE=warm LOOKAHEAD_S=1";;
    edgewarp) echo "MIGRATION_MODE=edgewarp_full TRIGGER_MODE=predictive LOOKAHEAD_S=1";;
    *) echo "MIGRATION_MODE=$1";; esac; }
run () { local tag="$1"; local scn="$2"; shift 2; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt t0 t1
  for attempt in 1 2; do
    carla_restart; t0=$(date +%s); echo "[$(date +%H:%M:%S)] $tag ($scn) attempt $attempt : $@"
    echo "[LAUNCHENV] ONCOMING_SPEED=12 TRIGGER_DIST=300 $@" > "$log"
    ( env "$@" ONCOMING_SPEED=12 TRIGGER_DIST=300 EVAL_TAG=khonsu-eval-freeze-1p timeout -k 30 1500 python ecav.py -t "$scn" --apply_ml >> "$log" 2>&1 ) || true
    t1=$(date +%s); echo "[CELLTIME] $tag sim_wall=$((t1-t0))s" | tee -a "$R/_timing.txt"
    _degenerate "$log" || break
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW; STOPPING block"; exit 2; }; }
# 5 seeds: the appendix density table and the burst 5/5 claim are both "of 5"; a
# finer denominator is an extension, not this block. burst 5x3 + q5 5x3x3 = 60.
SEEDS=${SEEDS:-"1 2 3 4 5"}
# burst first (its first cell is what classifies the block)
for rep in $SEEDS; do for m in warm edgewarp cold; do
  run "burst_${m}_r${rep}" openscenario_1_burst_gt $(_mode_args $m)
done; done
# q5 concurrent-crossings capacity
for rep in $SEEDS; do for fn in 2 4 8; do for m in warm edgewarp cold; do
  run "q5_n${fn}_${m}_r${rep}" openscenario_1_flow_gt $(_mode_args $m) FLOW_N=$fn
done; done; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[capacity] DONE $(date +%H:%M:%S)" > "$R/_done"
