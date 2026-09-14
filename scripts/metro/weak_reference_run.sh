#!/bin/bash
# Weak-scaling reference runs: full-network replays of the top-density (t3623) and
# 800-track extension traces under the three admission rows, with the per-(locale,
# cycle) load dump for offline density-matched window selection; then the rerun of
# the landed 22 replays for the instrumentation columns (coupled_rerun.sh).
# Log: evaluation_outputs/metro/weak/weak_reference.log
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox; OUT=$MAIN/evaluation_outputs/metro/weak; EXT=$MAIN/evaluation_outputs/metro/ext
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json; NET=$EXT/athens_w3.net.xml
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export PYTHONHASHSEED=0
cd "$MAIN/scripts/metro" || exit 1
JOBS=$OUT/weak_reference_jobs.txt; : > "$JOBS"
for T in 3623 800; do for rule in "none:0" "single:92" "deployed:92"; do r=${rule%%:*}; b=${rule##*:}
  echo "$EXT/ext_L106_t${T}_s1.fcd.xml $OUT/ref_t${T}_${r}.json $b $( [ "$r" = none ] && echo single || echo $r ) $OUT/ref_t${T}_${r}.npz" >> "$JOBS"
done; done
echo "[weak-ref] $(date '+%F %T') $(wc -l < "$JOBS") jobs, engine md5 $(md5sum metro_replay.py | cut -c1-8)"
run_one() { set -- $1; T=$1; O=$2; B=$3; R=$4; D=$5
  echo "[start] $(date '+%T') $(basename "$O")"
  python metro_replay.py --trace "$T" --net "$NET" --out "$O" --costs "$COSTS" --charge marginal --admit_budget_ms "$B" --admit_rule "$R" --report_windows 2,4,6,8,10 --dump_pooled "$D" > "${O%.json}.log" 2>&1
  echo "[done rc=$?] $(date '+%T') $(basename "$O") $(grep -E '^(warm|replall) ' "${O%.json}.log" | sed -E 's/.*p95 load +([0-9.]+)%.*/\1%/' | tr '\n' ' ')"
}
export -f run_one; export COSTS NET
xargs -P 6 -I{} bash -c 'run_one "{}"' < "$JOBS"
echo "[weak-ref] DONE $(date '+%F %T')"
"$MAIN/scripts/metro/coupled_rerun.sh" > "$MAIN/evaluation_outputs/metro/coupled_rerun.log" 2>&1
echo "[landed-22 rerun] DONE $(date '+%F %T') rc=0 count $(grep -c 'rc=0' "$MAIN/evaluation_outputs/metro/coupled_rerun.log")"
