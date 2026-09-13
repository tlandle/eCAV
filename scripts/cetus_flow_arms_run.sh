#!/bin/bash
set +u
# FLOW-ARMS block under freeze-1n on cetus (whole block, one host). These are the
# three "missing runner" axes that are actually ARMS inside the flow scenario
# (openscenario_1_flow_gt), driven by env params the runner already reads:
#   overlap  = TRIGGER_MODE=band + BAND_W_M in {10,20,40,80,120}   (overlap figure)
#   trigger  = TRIGGER_MODE in {predictive(baseline),computed,mtr,oracle}
#              + MTR_THETA in {0.3,0.5,0.7,0.9} for the mtr theta sweep
#   lookahead= LOOKAHEAD_S in {1(baseline),2,3,4}                   (T18 lookahead)
# No new scenario, no new runner: only cell enumeration. Tags MATCH the existing
# landers so the logs land without inventing conventions:
#   fb_band{w}_r{rep}   -> khonsu_1l_land.py band   (BAND_TAG_RE)
#   th_mtr{theta}_s{s}  -> khonsu_1l_land.py theta  (THETA_TAG_RE)
#   fa_*_s{s}           -> khonsu_1l_land.py rows   (generic RUNROW lander, added)
# Runs in cetus_1m_wt (overlay already applied = fixed ghost filter). Chained
# AFTER the capacity block: idle-waits for frozen_1n_capacity/_done and for no
# residual capacity CARLA/ecav.py before starting, so it never runs concurrently
# with capacity on the single cetus GPU. Whole-block-per-host preserved.
WT=$HOME/cetus_1m_wt; CARLA_ROOT=$HOME/carla-0.9.15
# Default gate is capacity _done; override with WAIT_FILE to insert the byte-field
# inert-proof gate (flow-arms waits for _proof_ok, written after the proof passes).
CAP_DONE="${WAIT_FILE:-$WT/evaluation_outputs/frozen_1p_capacity/_done}"
cd "$WT" || exit 1
source $HOME/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/tmp/torch_compat_1m:$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}

# --- chain: wait for capacity to finish (up to 14h), then require the GPU idle ---
if [ "${NOWAIT:-0}" != "1" ]; then
  echo "[flow-arms] waiting for capacity _done ($(date +%H:%M:%S))"
  for i in $(seq 1 17280); do   # 17280 * 10s = 48h (capacity ~11h, wide margin)
    [ -f "$CAP_DONE" ] && break
    sleep 10
  done
  [ -f "$CAP_DONE" ] || { echo "[flow-arms] ABORT capacity never finished after 48h"; exit 1; }
  echo "[flow-arms] capacity done; waiting for GPU idle ($(date +%H:%M:%S))"
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

R="$WT/evaluation_outputs/frozen_1p_flow_arms"; mkdir -p "$R"; rm -f "$R"/_done
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[EGO-DBG\]" "$lg" 2>/dev/null || return 0
  return 1; }
# defaults BEFORE "$@" so per-cell overrides win (the clobber fix).
run () { local tag="$1"; shift; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt t0 t1
  for attempt in 1 2; do
    carla_restart; t0=$(date +%s); echo "[$(date +%H:%M:%S)] $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=12 TRIGGER_DIST=300) then $@" > "$log"
    ( env ONCOMING_SPEED=12 TRIGGER_DIST=300 "$@" EVAL_TAG=khonsu-eval-freeze-1p timeout -k 30 900 python ecav.py -t openscenario_1_flow_gt --apply_ml >> "$log" 2>&1 ) || true
    t1=$(date +%s); echo "[CELLTIME] $tag sim_wall=$((t1-t0))s" | tee -a "$R/_timing.txt"
    _degenerate "$log" || break
    echo "[retry] $tag degenerate"
  done
  # HARD FAIL: a cell that exits without RUNROW was killed/crashed; stop loudly.
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW after $attempt attempts; STOPPING block"; exit 2; }; }

SEEDS=${SEEDS:-"1 2 3 4 5"}
if [ "${SMOKE:-0}" = "1" ]; then
  run "fb_band20_r1" MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=20 KHONSU_SEED=1
  echo "smoke $(date +%H:%M:%S)"; pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; exit 0
fi
# seed-major: each completed seed gives one full arm sweep, so an interrupted
# block is complete-but-coarse (n=1, then n=2, ...) rather than missing whole arms.
for s in $SEEDS; do
  # baseline shared by all three figures (warm, predictive trigger, 1 s lead)
  run "fa_base_s${s}"        MIGRATION_MODE=warm TRIGGER_MODE=predictive LOOKAHEAD_S=1 KHONSU_SEED=$s
  # overlap: band widths at BOTH speeds so the measured side matches the
  # generated frozengen_rows.csv exactly (5 widths x 2 speeds = 10 band cells/
  # seed; the overlap figure draws one curve per speed). 12 m/s uses the default
  # (no _v tag); 20 m/s is tagged _v20 so the lander reads the true speed per row
  # (std_row parses _v(\d+)) and the two lowest-width curves never mix a measured
  # 12 m/s point with a projected 20 m/s one.
  for w in 10 20 40 80 120; do
    run "fb_band${w}_r${s}"      MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=$w KHONSU_SEED=$s
    run "fb_band${w}_v20_r${s}"  MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=$w ONCOMING_SPEED=20 KHONSU_SEED=$s
  done
  # band5 is handoff-figure only (TRIGGER_ORDER wants fb_band5; overlap does not),
  # so it runs at the single contract speed and the rows lander excludes it from
  # the overlap file. The union of the two figures' widths is {5,10,20,40,80,120}.
  run "fb_band5_r${s}"           MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M=5 KHONSU_SEED=$s
  # trigger modes for the trigger figure's rows.csv (predictive = baseline,
  # band = fb_band20; mtr representative arm at the default theta 0.5, the theta
  # sweep below stays separate for the theta curve so rows.csv has one mtr arm).
  run "fa_trig_computed_s${s}" MIGRATION_MODE=warm TRIGGER_MODE=computed KHONSU_SEED=$s
  run "fa_trig_mtr_s${s}"      MIGRATION_MODE=warm TRIGGER_MODE=mtr MTR_THETA=0.5 KHONSU_SEED=$s
  run "fa_trig_oracle_s${s}"   MIGRATION_MODE=warm TRIGGER_MODE=oracle KHONSU_SEED=$s
  # trigger mtr theta sweep (th_mtr* -> theta lander)
  for th in 0.3 0.5 0.7 0.9; do
    run "th_mtr${th}_s${s}"  MIGRATION_MODE=warm TRIGGER_MODE=mtr MTR_THETA=$th KHONSU_SEED=$s
  done
  # lookahead sensitivity (fa_look* -> rows lander; 1 s = baseline above)
  for L in 2 3 4; do
    run "fa_look${L}_s${s}"  MIGRATION_MODE=warm TRIGGER_MODE=predictive LOOKAHEAD_S=$L KHONSU_SEED=$s
  done
done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
echo "[flow-arms] DONE $(date +%H:%M:%S)" > "$R/_done"
echo "[flow-arms] ALL DONE $(date +%H:%M:%S)"
