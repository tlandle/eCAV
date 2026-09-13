#!/bin/bash
# Rerun only the single-batch-rule rows (pNEUMA admit92/admit50, extension K=16)
# after the owned-pass switch (single batch predict(K) for that row's coupling).
# Reads the jobs file written by coupled_rerun.sh. Log: coupled_rerun_single.log
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox; OUT=$MAIN/evaluation_outputs/metro
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export SUMO_HOME=$CONDA_PREFIX/lib/python3.10/site-packages/sumo; export PATH=$SUMO_HOME/bin:$PATH
cd "$MAIN/scripts/metro" || exit 1
JOBS=$OUT/coupled_rerun_single_jobs.txt; awk '$4 != "0" && $5 == "single"' "$OUT/coupled_rerun_jobs.txt" > "$JOBS"
echo "[coupled-rerun-single] $(date '+%F %T') $(wc -l < "$JOBS") jobs, engine md5 $(md5sum metro_replay.py | cut -c1-8)"
run_one() { set -- $1; T=$1; N=$2; O=$3; B=$4; R=$5
  echo "[start] $(date '+%T') $(basename "$O")"
  python metro_replay.py --trace "$T" --net "$N" --out "$O" --costs "$COSTS" --charge marginal --admit_budget_ms "$B" --admit_rule "$R" > "${O%.json}.log" 2>&1
  echo "[done rc=$?] $(date '+%T') $(basename "$O") $(grep -E '^(warm|replall|delta_ready) ' "${O%.json}.log" | sed -E 's/.*p95 load +([0-9.]+)%.*COUPLED +([0-9.]+)%.*/\1%\/coupled \2%/' | tr '\n' ' ')"
}
export -f run_one; export COSTS
xargs -P 6 -I{} bash -c 'run_one "{}"' < "$JOBS"
echo "[coupled-rerun-single] DONE $(date '+%F %T')"
