#!/bin/bash
# Khonsu design studies S1-S4 (docs/agent_plans/khonsu_design_studies.md).
# Runs sequentially against a live CARLA; each run logs to LOGDIR and the
# extractor turns [RUNROW] + the ego eval dict into CSV rows.
# Usage: KDS_GO=1 REPS=10 bash scripts/khonsu_design_sweep.sh
set -u
cd /home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
source /home/atlas/anaconda3/etc/profile.d/conda.sh
conda activate opencda310

REPS=${REPS:-10}
LOGDIR=${LOGDIR:-evaluation_outputs/khonsu_design_sweep_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOGDIR"
echo "logdir: $LOGDIR"

run_one () {
  local tag="$1"; shift
  local log="$LOGDIR/${tag}.log"
  if [ -f "$log" ] && grep -q RUNROW "$log"; then
    echo "skip $tag (done)"; return
  fi
  echo "[$(date +%H:%M:%S)] $tag"
  env "$@" python ecav.py -t openscenario_1_flow_gt --apply_ml \
      > "$log" 2>&1
  grep -q RUNROW "$log" || echo "WARN: $tag produced no RUNROW"
}

if [ "${KDS_GO:-0}" != "1" ]; then
  echo "dry run: set KDS_GO=1 to execute"; exit 0
fi

for rep in $(seq 1 "$REPS"); do
  # S1 trigger axis: band widths (predictive warm + reactive already tabled;
  # rerun warm/reactive here too so all rows share one harness version)
  run_one "s1_warm_r${rep}"      MIGRATION_MODE=warm
  run_one "s1_reactive_r${rep}"  MIGRATION_MODE=reactive
  for w in 5 10 20 40 80; do
    run_one "s1_band${w}_r${rep}" MIGRATION_MODE=warm TRIGGER_MODE=band BAND_W_M="$w"
  done
  # S2 commit refresh
  run_one "s2_refresh_r${rep}"   MIGRATION_MODE=warm COMMIT_REFRESH=full
  # S3 mirroring rate
  for p in 0.25 0.5 1.0 2.0; do
    run_one "s3_mirror${p}_r${rep}" MIGRATION_MODE=warm MIRROR_PERIOD_S="$p"
  done
  # S4 seed top-up for the remaining headline arms
  run_one "s4_kf_r${rep}"        MIGRATION_MODE=kf
  run_one "s4_edgewarp_r${rep}"  MIGRATION_MODE=edgewarp
  run_one "s4_cold_r${rep}"      MIGRATION_MODE=cold
done
echo "sweep complete: $LOGDIR"
