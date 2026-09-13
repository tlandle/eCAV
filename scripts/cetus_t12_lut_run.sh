#!/bin/bash
set +u
# T12 CONTROLLED-AGE sweep via the ns-3 radio plane (freeze-1n, cetus, whole
# block). Tyler's amendment 2: age at use must be produced by the ns-3 radio
# plane, not injection. NS3_LUT_N indexes the LUT load cell (the freeze-1l fix:
# it sets the cell/contention the latency distribution is drawn from while the
# max stays over the live uploader count; identical to live at N=scene). AGEROW
# already logs realized_age_ms + network_age_ms per planner decision in the
# freeze-1n edge overlay, so this is a sweep + land, no frozen-base change.
# Arms: warm WITH migration (matches the freeze-1f/1j T12 runs, confirmed in KB).
# Scenarios: blindovertake = openscenario_1_flow_gt (constant-speed oncoming),
#            accel        = openscenario_1_accel_gt (accelerating oncoming;
#            invoked exactly like flow per scratchpad/cetus_debug_accel.sh).
# Sweep: NS3_LUT_N in {4,8,12,16,20,24,31} x 10 seeds x 2 scenarios = 140 runs
# (~12 h flow-class). 10 seeds not 5: age_bins drops any 100 ms bin with < 8
# RUNS, and a lone load level contributes only its own seeds to its bin, so 5
# would silently drop levels that do not pair up (peer's call).
# Lander: scripts/khonsu_1l_land.py t12lut (per-run sweep + per-decision file).
# Chained AFTER flow-arms (idle-waits for frozen_1n_flow_arms/_done + GPU idle).
WT=$HOME/cetus_1m_wt; CARLA_ROOT=$HOME/carla-0.9.15
FA_DONE="$WT/evaluation_outputs/frozen_1p_flow_arms/_done"
cd "$WT" || exit 1
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}

if [ "${NOWAIT:-0}" != "1" ]; then
  echo "[t12] waiting for flow-arms _done ($(date +%H:%M:%S))"
  for i in $(seq 1 17280); do   # 17280 * 10s = 48h (capacity+flow-arms ~18h, wide margin)
    [ -f "$FA_DONE" ] && break
    sleep 10
  done
  [ -f "$FA_DONE" ] || { echo "[t12] ABORT flow-arms never finished after 48h"; exit 1; }
  echo "[t12] flow-arms done; waiting for GPU idle ($(date +%H:%M:%S))"
  for i in $(seq 1 60); do
    pgrep -f "ecav.py -t openscenario" >/dev/null 2>&1 || break
    sleep 10
  done
fi

[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT models mismatch"; exit 1; }
grep -q "PUBGATE_SRC" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT overlay not applied"; exit 1; }
grep -q "NS3_LUT_N indexes the LUT CELL" ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py || { echo "ABORT freeze-1l LUT-cell fix not present"; exit 1; }

R="$WT/evaluation_outputs/frozen_1p_t12"; mkdir -p "$R"; rm -f "$R"/_done
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
_scn () { case "$1" in
    blindovertake) echo "openscenario_1_flow_gt";;
    accel)         echo "openscenario_1_accel_gt";;
    *) echo "openscenario_1_flow_gt";; esac; }
# tag = t12_{scenario}_n{N}_s{seed}; run consumes NS3_LUT_N + KHONSU_SEED, warm.
run () { local scn="$1" n="$2" seed="$3"; local tag="t12_${scn}_n${n}_s${seed}"
  local log="$R/${tag}.log" runner; runner=$(_scn "$scn")
  # done = RUNROW present, not degenerate, AND at least one AGEROW (the age is the
  # whole point of this block; a run without AGEROW is unusable, not just late).
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && grep -q "\[AGEROW\]" "$log" 2>/dev/null && ! _degenerate "$log"; then
    echo "[skip] $tag"; return; fi
  local attempt t0 t1
  for attempt in 1 2; do
    carla_restart; t0=$(date +%s); echo "[$(date +%H:%M:%S)] $tag ($runner) attempt $attempt"
    echo "[LAUNCHENV] NS3_LUT_N=$n MIGRATION_MODE=warm ONCOMING_SPEED=12 TRIGGER_DIST=300 KHONSU_SEED=$seed" > "$log"
    ( env NS3_LUT_N=$n MIGRATION_MODE=warm ONCOMING_SPEED=12 TRIGGER_DIST=300 KHONSU_SEED=$seed \
        EVAL_TAG=khonsu-eval-freeze-1p timeout -k 30 900 python ecav.py -t "$runner" --apply_ml >> "$log" 2>&1 ) || true
    t1=$(date +%s); echo "[CELLTIME] $tag sim_wall=$((t1-t0))s" | tee -a "$R/_timing.txt"
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW after $attempt attempts; STOPPING block"; exit 2; }
  grep -q "\[AGEROW\]" "$log" 2>/dev/null || { echo "[HARD FAIL] $tag RUNROW but 0 AGEROW (age instrumentation silent; the block measures age); STOPPING"; exit 2; }; }

SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
LEVELS=${LEVELS:-"4 8 12 16 20 24 31"}
SCNS=${SCNS:-"blindovertake accel"}
if [ "${SMOKE:-0}" = "1" ]; then
  # validate BOTH scenarios emit RUNROW + AGEROW + a do_ov launch before the full
  # sweep (accel t12 was never run on freeze-1f; blindovertake is proven).
  run blindovertake 12 1
  run accel 12 1
  echo "[t12-smoke] blindovertake AGEROW=$(grep -c '\[AGEROW\]' "$R/t12_blindovertake_n12_s1.log" 2>/dev/null) launch=$(grep -c 'do_ov=True' "$R/t12_blindovertake_n12_s1.log" 2>/dev/null)"
  echo "[t12-smoke] accel        AGEROW=$(grep -c '\[AGEROW\]' "$R/t12_accel_n12_s1.log" 2>/dev/null) launch=$(grep -c 'do_ov=True' "$R/t12_accel_n12_s1.log" 2>/dev/null)"
  pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; exit 0
fi
# accel is unproven on t12 (freeze-1f ran blindovertake only): probe one accel
# cell first so a broken accel age path hard-fails in ~5 min instead of after the
# whole blindovertake scenario. The cell counts toward the sweep (skipped when the
# main loop reaches it).
if echo "$SCNS" | grep -qw accel; then run accel 12 1; fi
# scenario-major so scenario 1 finishes first (peer's mid-course bin check),
# then seed-major within so an interrupted scenario is n=1..k, not missing levels.
for scn in $SCNS; do
  for seed in $SEEDS; do
    for n in $LEVELS; do
      run "$scn" "$n" "$seed"
    done
  done
  echo "[t12] scenario $scn complete $(date +%H:%M:%S)" >> "$R/_timing.txt"
done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[t12] DONE $(date +%H:%M:%S)" > "$R/_done"
echo "[t12] ALL DONE $(date +%H:%M:%S)"
