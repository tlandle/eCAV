#!/bin/bash
set +u
# CETUS corridor RUN-ONLY. The worktree ~/cetus_1m_wt is already set up (frozen
# base 71c9f37e + code_overlay applied + corridor files + proto stubs + REAL
# models placed by the peer). This script does NOT recreate the worktree and does
# NOT touch the models (doing so would clobber the peer's real checkpoints with
# dangling symlinks). It only verifies the setup then runs the 70 routes.
WT=$HOME/cetus_1m_wt
CARLA_ROOT=$HOME/carla-0.9.15
# Chain: run LAST in the cetus sequence (capacity -> flow-arms -> t12 -> corridor)
# so the whole campaign shares the single GPU without contention. Idle-wait for
# the predecessor's _done (default the t12 age sweep; override CHAIN_WAIT; NOWAIT=1
# skips for standalone use), then for the GPU to be idle, before taking the lock.
CHAIN_WAIT=${CHAIN_WAIT:-$HOME/cetus_1m_wt/evaluation_outputs/frozen_1p_t12/_done}
if [ "${NOWAIT:-0}" != "1" ] && [ -n "$CHAIN_WAIT" ]; then
  echo "[cetus-corridor] waiting for $CHAIN_WAIT ($(date +%H:%M:%S))"
  for i in $(seq 1 17280); do   # 17280 * 10s = 48h (capacity+flow-arms+t12 ~30h)
    [ -f "$CHAIN_WAIT" ] && break
    sleep 10
  done
  [ -f "$CHAIN_WAIT" ] || { echo "[cetus-corridor] ABORT predecessor never finished after 48h"; exit 1; }
  for i in $(seq 1 60); do
    pgrep -f "ecav.py -t openscenario" >/dev/null 2>&1 || break
    sleep 10
  done
fi
exec 9>/tmp/cetus_1m.lock
flock -w 600 9 || { echo "[cetus-corridor] lock busy"; exit 1; }
echo "[cetus-corridor-run] start $(date +%H:%M:%S)"
cd "$WT" || { echo "ABORT no worktree"; exit 1; }
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD)" = "71c9f37e" ] || { echo "ABORT worktree not at base"; exit 1; }
# model-integrity guard (peer 57af285a): full size+hash check against the manifest,
# not just a size floor - catches the silent-divergence case (untracked Swin backbone
# quietly fetched from HuggingFace at a different revision than atlas).
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest; results from this host are not comparable"; exit 1; }
grep -q "PUBGATE_SRC" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT epoch edge missing"; exit 1; }
grep -q "CONSUMEDEPOCH" ecav/core/plan/behavior_agent.py || { echo "ABORT epoch behavior missing"; exit 1; }
grep -q "CORRIDORCROSS" ecav/scenario_testing/openscenario_1_corridor_gt.py || { echo "ABORT corridor runner missing"; exit 1; }
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 3
R="$WT/evaluation_outputs/frozen_1p_corridor"; mkdir -p "$R"
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] ONCOMING_SPEED=12 TRIGGER_DIST=300 $@" > "$log"
    ( env "$@" ONCOMING_SPEED=12 TRIGGER_DIST=300 EVAL_TAG=khonsu-eval-freeze-1p timeout -k 30 2100 python ecav.py -t openscenario_1_corridor_gt --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done
  # HARD FAIL: no completion marker means the route was killed/crashed (e.g. a
  # too-short timeout). A corridor route must reach RUNROW and cross >=1 boundary
  # (CORRIDORCROSS); stop the block loudly rather than churn out zero-row cells.
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW after $attempt attempts (timeout/crash); STOPPING block"; exit 2; }
  [ "$(grep -c CORRIDORCROSS "$log" 2>/dev/null)" -gt 0 ] || { echo "[HARD FAIL] $tag RUNROW but 0 CORRIDORCROSS (broken route/no handoff); STOPPING block"; exit 2; }; }
# The alternatives figure and corridor table iterate ALT_ORDER (plot_figures.py):
# 13 arms = cold, handover_snapshot, kf, kf_final, edgewarp, reactive, band20,
# band40, band80, replication, repl_final, warm, oracle. arm_of() resolves them
# from the logged (mode, trigger, band), so the env per arm must produce those:
# edgewarp = MIGRATION_MODE=edgewarp_full (arm_of maps it to 'edgewarp'); band{w}
# = warm + TRIGGER_MODE=band + BAND_W_M=w; oracle = warm + TRIGGER_MODE=oracle;
# the rest are plain modes. Running fewer draws measured points beside projected
# ones on the same axes with nothing to tell them apart. 13 x 5 = 65 + khonsu_ef0
# x5 = 70 routes at n=5 (extend to n=10 later). khonsu_ef0 is the deliberately
# unfenced control for the fencing rule, NOT in ALT_ORDER; kept, kept out of the
# figure.
_arm_env () { case "$1" in
    cold)              echo "MIGRATION_MODE=cold" ;;
    handover_snapshot) echo "MIGRATION_MODE=handover_snapshot" ;;
    kf)                echo "MIGRATION_MODE=kf" ;;
    kf_final)          echo "MIGRATION_MODE=kf_final" ;;
    edgewarp)          echo "MIGRATION_MODE=edgewarp_full" ;;
    reactive)          echo "MIGRATION_MODE=reactive" ;;
    band20)            echo "MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=20" ;;
    band40)            echo "MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=40" ;;
    band80)            echo "MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=80" ;;
    replication)       echo "MIGRATION_MODE=replication" ;;
    repl_final)        echo "MIGRATION_MODE=repl_final" ;;
    warm)              echo "MIGRATION_MODE=warm" ;;
    oracle)            echo "MIGRATION_MODE=warm TRIGGER_MODE=oracle" ;;
  esac; }
ARMS=${ARMS:-"cold handover_snapshot kf kf_final edgewarp reactive band20 band40 band80 replication repl_final warm oracle"}
SEEDS=${SEEDS:-"1 2 3 4 5"}
if [ "${SMOKE:-0}" = "1" ]; then run "co_warm_n1_r1" MIGRATION_MODE=warm EPOCH_FENCE=1; echo "smoke $(date +%H:%M:%S)"; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; exit 0; fi
for rep in $SEEDS; do
  for arm in $ARMS; do
    run "co_${arm}_n1_r${rep}" $(_arm_env "$arm") EPOCH_FENCE=1
  done
done
for rep in $SEEDS; do run "co_khonsu_ef0_r${rep}" MIGRATION_MODE=warm EPOCH_FENCE=0; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[cetus-corridor-run] DONE $(date +%H:%M:%S)" > "$R/_corridor_1m_done"
echo "[cetus-corridor-run] ALL DONE $(date +%H:%M:%S)"
