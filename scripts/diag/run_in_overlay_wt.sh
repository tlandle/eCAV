#!/bin/bash
set +u
# Run a diagnostic script inside mb_wt (freeze base 71c9f37e) with the freeze-1p
# overlay applied, under opencda310, then restore the worktree. Refuses to run
# while a simulation or CARLA is alive (shares the GPU with nothing).
# Usage: run_in_overlay_wt.sh <script.py> [args...]
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
WT=${WT:-$MAIN/idfix_wt}   # default: the campaign worktree (has the compiled MTR ops); mb_wt lacks the untracked .so builds
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
if pgrep -f "^python ecav.py" >/dev/null || pgrep -f "^/home/atlas/carla-0.9.15/CarlaUE4" >/dev/null; then echo "ABORT: a simulation or CARLA is running"; exit 1; fi
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
echo "[overlay-wt] $(date '+%F %T') host $(hostname) load $(cut -d' ' -f1 /proc/loadavg) gpu $(nvidia-smi --query-gpu=memory.used --format=csv,noheader | head -1)"
python "$@"
