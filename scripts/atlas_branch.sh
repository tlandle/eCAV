#!/bin/bash
set +u
# BRANCHING-JUNCTION HARNESS (NSDI experiment 1), atlas, freeze-1p overlay.
# Sites k2/k3/k4 (scripts/branching/branching_sites.json), six placement arms,
# seed-major. Same central lock as the campaign runners (queues behind them).
# Env: SITES (default "k2 k3 k4"), SEEDS (1..10), ARMS, ONCOMING_SPEED (8 at EVERY site:
#      the degree sweep varies k only; 8 keeps the 2 s history window on the k4 straight),
#      SMOKE=1 -> k3, seed 1, arms "oracle khonsu replicate".
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=$MAIN/idfix_wt
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
BR=${BR:-$MAIN/scripts/branching}
CARLA_ROOT=/home/atlas/carla-0.9.15
SITES=${SITES:-"k2 k3 k4"}
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
ARMS=${ARMS:-"khonsu oracle cv top1 edgewarp replicate"}
if [ "${SMOKE:-0}" = "1" ]; then SITES=${SITES:-k3}; SITES="k3"; SEEDS="1"; ARMS="oracle khonsu replicate"; fi
exec 9>/tmp/atlas_1l_central.lock
flock -w 172800 9 || { echo "[branch] lock wait timed out"; exit 1; }
echo "[branch] lock acquired $(date +%H:%M:%S) sites='$SITES' seeds='$SEEDS' arms='$ARMS' smoke=${SMOKE:-0}"
cd "$WT" || exit 1
pkill -9 -f "CarlaUE4" 2>/dev/null; sleep 3
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:${PYTHONPATH:-}
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
# campaign overlay (identical predictor/tracker/edge build as freeze-1p) + branching files
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|$SP/pluggable_base_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|$SP/wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|$SP/tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|$SP/tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|$SP/factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|$SP/payload_wired.py"
  "ecav/core/application/edge/migration/daemon.py|$BR/daemon_branch.py"
  "ecav/core/plan/behavior_agent.py|$SP/behavior_agent_1m.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|$SP/edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|$SP/mtr_edge_predictor_1m.py"
  "ecav/scenario_testing/utils/sim_api.py|$BR/sim_api_branch.py"
)
NEWF=(
  "ecav/scenario_testing/scenarios/scenario_1_branch.py|$BR/scenario_1_branch.py"
  "ecav/scenario_testing/scenarios/branching_sites.json|$BR/branching_sites.json"
)
for k in k2 k3 k4; do
  NEWF+=("ecav/scenario_testing/openscenario_1_branch_${k}.py|$BR/runner_branch.py")
  NEWF+=("ecav/scenario_testing/config_yaml/openscenario_1_branch_${k}.yaml|$BR/openscenario_1_branch_${k}.yaml")
  NEWF+=("ecav/scenario_testing/scenarios/scenario_1_branch_${k}.xml|$BR/scenario_1_branch_${k}.xml")
done
DESTS=()
_backup () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "$d" "/tmp/br_${i}.bak"; DESTS+=("$d"); i=$((i+1)); done; }
restore () { local i=0; for p in "${COMMON[@]}"; do d=${p%%|*}; cp "/tmp/br_${i}.bak" "$d"; i=$((i+1)); done
  git checkout -q "${DESTS[@]}" 2>/dev/null
  for p in "${NEWF[@]}"; do rm -f "${p%%|*}"; done
  pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; }
trap restore EXIT
_backup
for p in "${COMMON[@]}" "${NEWF[@]}"; do d=${p%%|*}; s=${p##*|}; [ -f "$s" ] || { echo "ABORT missing overlay source $s"; exit 1; }; cp "$s" "$d"; done
grep -q "AGENTSET_ISOLATE" ecav/core/prediction/mtr_edge_predictor.py || { echo "ABORT cap predictor not applied"; exit 1; }
grep -q "member_vehicles" ecav/scenario_testing/utils/sim_api.py || { echo "ABORT sim_api overlay not applied"; exit 1; }
grep -q "payload_hash" ecav/core/application/edge/migration/daemon.py || { echo "ABORT daemon hash not applied"; exit 1; }
git fetch origin develop -q 2>/dev/null || true
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest"; exit 1; }
carla_restart () { pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null; sleep 6; ( cd "$CARLA_ROOT" && setsid nohup ./CarlaUE4.sh -RenderOffScreen >/dev/null 2>&1 9>&- & ); sleep 55; }
_degenerate () { local lg="$1"
  grep -q "is not found in your CARLA repo\|has no attribute 'world'" "$lg" 2>/dev/null && return 0
  grep -q "\[HEROROW\]" "$lg" 2>/dev/null || return 0
  return 1; }
branches_of () { python3 -c "import json,sys; s=json.load(open('$BR/branching_sites.json'))['$1']; print(' '.join(l['id'] for l in s['locales'] if l['id'].startswith('locale_branch_')))"; }
run () { local site="$1" tag="$2"; shift 2; local R="$WT/evaluation_outputs/branch_1p_${site}"; mkdir -p "$R"; local log="$R/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log" 2>/dev/null && grep -q "\[READYROW\]" "$log" 2>/dev/null && ! _degenerate "$log"; then echo "[skip] $tag"; return; fi
  local attempt
  for attempt in 1 2; do
    carla_restart; echo "[$(date +%H:%M:%S)] $site $tag attempt $attempt : $@"
    echo "[LAUNCHENV] defaults(ONCOMING_SPEED=${ONCOMING_SPEED:-8} TRIGGER_DIST=300 AGENTSET_ISOLATE=cap ISOLATE_CAP=1) then $@" > "$log"
    ( env ONCOMING_SPEED=${ONCOMING_SPEED:-8} TRIGGER_DIST=300 AGENTSET_ISOLATE=cap ISOLATE_CAP=1 \
          BRANCH_SITE=$site BRANCH_SITES_JSON=ecav/scenario_testing/scenarios/branching_sites.json "$@" \
          EVAL_TAG=${EVAL_TAG:-khonsu-eval-branch-1p} timeout -k 30 900 python ecav.py -t openscenario_1_branch_${site} --apply_ml >> "$log" 2>&1 ) || true
    _degenerate "$log" || break
    echo "[retry] $tag degenerate (startup transient)"
  done
  grep -q RUNROW "$log" 2>/dev/null || { echo "[HARD FAIL] $tag no RUNROW after $attempt attempts; STOPPING"; exit 2; }
  grep -q "\[READYROW\]" "$log" 2>/dev/null || { echo "[HARD FAIL] $tag RUNROW but no READYROW (actor never resolved); STOPPING"; exit 2; }
  # mode-diversity + trigger-fire gates (paper side 2026-09-13): the MTR trigger cannot fire under mode collapse
  case "$tag" in *_khonsu_*|*_top1_*)
    grep -q "\[FIREROW\].*rule=\(theta\|top1\)" "$log" 2>/dev/null || { echo "[HARD FAIL] $tag: the MTR trigger never fired (no FIREROW rule=theta/top1); mode collapse? STOPPING"; exit 2; }
    python3 scripts/diag/check_mode_diversity.py "$log" || { echo "[HARD FAIL] $tag: MODESROW mode diversity below the gate; STOPPING"; exit 2; } ;;
  esac; }
arm_env () { case "$1" in
    khonsu)    echo "MIGRATION_MODE=warm TRIGGER_MODE=mtr" ;;
    oracle)    echo "MIGRATION_MODE=warm TRIGGER_MODE=oracle" ;;
    cv)        echo "MIGRATION_MODE=warm TRIGGER_MODE=computed" ;;
    top1)      echo "MIGRATION_MODE=warm TRIGGER_MODE=mtr MTR_TOP1=1" ;;
    edgewarp)  echo "MIGRATION_MODE=edgewarp_full" ;;
    replicate) echo "MIGRATION_MODE=warm TRIGGER_MODE=computed REPLICATE_ALL=1" ;;
  esac; }
# seed-major across sites: seed s takes branch (s mod k) at every site, so an
# interrupted grid is a complete-but-coarse figure at every k.
for s in $SEEDS; do for site in $SITES; do
  read -ra BRS <<< "$(branches_of $site)"; k=${#BRS[@]}; taken=${BRS[$(( (s - 1) % k ))]}
  for md in $ARMS; do
    run "$site" "br_${site}_${md}_s${s}" $(arm_env $md) KHONSU_SEED=$s BRANCH_TAKEN=$taken
  done
done; done
pkill -9 -f "CarlaUE4/Binaries" 2>/dev/null
for site in $SITES; do echo "BRANCH sites='$SITES' seeds='$SEEDS' arms='$ARMS' $(date +%H:%M:%S)" > "$WT/evaluation_outputs/branch_1p_${site}/_branch_done"; done
echo "[branch] DONE $(date +%H:%M:%S)"
