#!/bin/bash
# Rerun every metro replay (pNEUMA 09:00 slot, four admission rows; extension six
# points, three rows) under the engine with the load-coupled freshness column
# (deployed rate fallback replayed per locale). Outputs overwrite the canonical
# JSONs in place; loads are unchanged by construction (checked on one point).
# CPU only, six jobs in parallel. Log: evaluation_outputs/metro/coupled_rerun.log
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
OUT=$MAIN/evaluation_outputs/metro; EXT=$OUT/ext
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json
PN=/home/atlas/datasets/pneuma/extract/pNEUMA_dataset/20181024_dX_0900_0930.csv; PNET=$SP/metro/athens_pneuma_pad50.net.xml
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export SUMO_HOME=$CONDA_PREFIX/lib/python3.10/site-packages/sumo; export PATH=$SUMO_HOME/bin:$PATH
cd "$MAIN/scripts/metro" || exit 1
JOBS=$OUT/coupled_rerun_jobs.txt; : > "$JOBS"
echo "$PN $PNET $OUT/pneuma_0900_replay_demand.json 0 single" >> "$JOBS"
echo "$PN $PNET $OUT/pneuma_0900_replay_admit92.json 92 single" >> "$JOBS"
echo "$PN $PNET $OUT/pneuma_0900_replay_admit50.json 50 single" >> "$JOBS"
echo "$PN $PNET $OUT/pneuma_0900_replay_deployed.json 92 deployed" >> "$JOBS"
NET=$(ls "$EXT"/athens_w*.net.xml | head -1)
for F in "$EXT"/ext_L*_t*_s1.fcd.xml; do B=$(basename "$F" .fcd.xml)
  echo "$F $NET $EXT/replay_${B}_none.json 0 single" >> "$JOBS"
  echo "$F $NET $EXT/replay_${B}_single.json 92 single" >> "$JOBS"
  echo "$F $NET $EXT/replay_${B}_deployed.json 92 deployed" >> "$JOBS"
done
echo "[coupled-rerun] $(date '+%F %T') $(wc -l < "$JOBS") jobs, engine md5 $(md5sum metro_replay.py | cut -c1-8)"
run_one() { set -- $1; T=$1; N=$2; O=$3; B=$4; R=$5
  echo "[start] $(date '+%T') $(basename "$O")"
  python metro_replay.py --trace "$T" --net "$N" --out "$O" --costs "$COSTS" --charge marginal --admit_budget_ms "$B" --admit_rule "$R" > "${O%.json}.log" 2>&1
  echo "[done rc=$?] $(date '+%T') $(basename "$O") $(grep -E '^(warm|replall|delta_ready) ' "${O%.json}.log" | sed -E 's/.*p95 load +([0-9.]+)%.*COUPLED +([0-9.]+)%.*/\1%\/coupled \2%/' | tr '\n' ' ')"
}
export -f run_one; export COSTS
xargs -P 6 -I{} bash -c 'run_one "{}"' < "$JOBS"
echo "[coupled-rerun] DONE $(date '+%F %T')"
