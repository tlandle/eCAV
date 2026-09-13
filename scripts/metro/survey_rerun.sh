#!/bin/bash
# Recompute the five pNEUMA slot surveys under the deterministic partition
# (sorted iteration, PYTHONHASHSEED=0) into *_survey_det.json for a diff
# against the surveys reported on 2026-09-12. Two slots at a time (each parses
# a ~3 GB CSV). Log: evaluation_outputs/metro/survey_rerun.log
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
OUT=$MAIN/evaluation_outputs/metro; DATA=/home/atlas/datasets/pneuma/extract/pNEUMA_dataset
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export SUMO_HOME=$CONDA_PREFIX/lib/python3.10/site-packages/sumo; export PATH=$SUMO_HOME/bin:$PATH
export PYTHONHASHSEED=0
cd "$MAIN/scripts/metro" || exit 1
echo "[survey-rerun] $(date '+%F %T')"
run_one() { slot=$1
  echo "[start] $(date '+%T') $slot"
  python pneuma_survey.py "$DATA/20181024_dX_${slot}.csv" "$OUT/pneuma_20181024_${slot}_survey_det.json" --tile 300 --work "$SP/metro" --osm "$SP/metro/athens_pneuma_pad50.osm" > "$OUT/pneuma_20181024_${slot}_survey_det.log" 2>&1
  echo "[done rc=$?] $(date '+%T') $slot"
}
export -f run_one; export DATA OUT SP
printf "%s\n" 0830_0900 0900_0930 0930_1000 1000_1030 1030_1100 | xargs -P 2 -I{} bash -c 'run_one {}'
echo "[survey-rerun] DONE $(date '+%F %T')"
