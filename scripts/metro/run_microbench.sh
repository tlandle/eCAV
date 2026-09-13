#!/bin/bash
set +u
# Measured per-operation costs (metro replay step 2): create / delta / predict at
# batches 1-32 on the REAL freeze-1p paths, on the runtime device (tracker and
# predictor on cuda:0 as the edge manager constructs them). Runs in mb_wt at the
# freeze base 71c9f37e with the campaign overlay applied, opencda310.
# GUARD: refuses to run while any simulation or CARLA is alive or the host is
# loaded, because the numbers are the paper's per-operation costs.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=${WT:-$MAIN/idfix_wt}   # campaign worktree by default (compiled MTR ops present); mb_wt lacks the untracked builds
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
OUT=${OUT:-$MAIN/evaluation_outputs/metro_microbench}
mkdir -p "$OUT"
if pgrep -f "^python ecav.py" >/dev/null || pgrep -f "^/home/atlas/carla-0.9.15/CarlaUE4" >/dev/null; then echo "ABORT: a simulation or CARLA is running"; exit 1; fi
L1=$(cut -d' ' -f1 /proc/loadavg); awk -v l="$L1" 'BEGIN{exit !(l>8.0)}' && { echo "ABORT: host load $L1 > 8 (32-core host; the guard is against a running campaign, not idle chatter)"; exit 1; }
cd "$WT" || exit 1
[ "$(git rev-parse --short HEAD | cut -c1-8)" = "71c9f37e" ] || { echo "ABORT not freeze base"; exit 1; }
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate opencda310
export PYTHONPATH=/home/atlas/carla-0.9.15/PythonAPI/carla:$WT:${PYTHONPATH:-}
COMMON=(
  "ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py|pluggable_base_wired.py"
  "ecav/core/tracking/mamba3dmot/wrapper.py|wrapper_wired.py"
  "ecav/core/tracking/mamba3dmot/tracker.py|tracker_wired.py"
  "ecav/core/tracking/mamba3dmot/tracklet.py|tracklet_1l.py"
  "ecav/core/application/edge/migration/factories.py|factories_1l.py"
  "ecav/core/application/edge/migration/payload.py|payload_wired.py"
  "ecav/core/application/edge/migration/daemon.py|daemon_1l.py"
  "ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py|edge_manager_merged_1l_epoch.py"
  "ecav/core/prediction/mtr_edge_predictor.py|mtr_edge_predictor_1m.py"
)
DESTS=()
restore () { git checkout -q -- "${DESTS[@]}" 2>/dev/null; }
trap restore EXIT
for p in "${COMMON[@]}"; do d=${p%%|*}; s=${p##*|}; [ -f "$SP/$s" ] || { echo "ABORT missing overlay $s"; exit 1; }; cp "$SP/$s" "$d"; DESTS+=("$d"); done
grep -q "AGENTSET_ISOLATE" ecav/core/prediction/mtr_edge_predictor.py || { echo "ABORT cap predictor not applied"; exit 1; }
git checkout origin/develop -- scripts/verify_models.sh scripts/models.manifest 2>/dev/null || true
bash scripts/verify_models.sh "$(pwd)" || { echo "ABORT: model artefacts differ from manifest"; exit 1; }
TS=$(date +%Y%m%d_%H%M%S)
echo "[microbench] $TS host $(hostname) load $L1 gpu: $(nvidia-smi --query-gpu=name,memory.used --format=csv,noheader | head -1)"
python "$SP/metro/microbench_ops.py" --ops "${OPS:-create,delta,predict}" --device "${DEVICE:-cuda:0}" --reps "${REPS:-10}" \
  --edge_yaml ecav/scenario_testing/config_yaml/openscenario_1_flow_gt.yaml --out "$OUT/microbench_${TS}.json" 2>&1 | tee "$OUT/microbench_${TS}.log"
echo "[microbench] DONE $(date +%H:%M:%S) -> $OUT/microbench_${TS}.json"
