#!/bin/bash
# Weak-scaling CROP series: run the engine with the density-matched windows chosen by
# weak_scaling_windows.py (window column of the *_windows.csv, density_matched rows),
# both traces, three admission rows; rim effects are real here (the deployment ends
# at the window) and are reported beside the reference series.
# Usage: weak_crop_run.sh   (reads evaluation_outputs/metro/weak/ref_t{3623,800}_deployed_windows.csv)
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox; OUT=$MAIN/evaluation_outputs/metro/weak; EXT=$MAIN/evaluation_outputs/metro/ext
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json; NET=$EXT/athens_w3.net.xml
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export PYTHONHASHSEED=0
cd "$MAIN/scripts/metro" || exit 1
JOBS=$OUT/weak_crop_jobs.txt; : > "$JOBS"
for T in 3623 800; do W=$OUT/ref_t${T}_deployed_windows.csv; [ -f "$W" ] || { echo "ABORT missing $W"; exit 1; }
  python3 - "$W" <<'PY' | while read k win; do
import csv, sys
seen = set()
for r in csv.DictReader(open(sys.argv[1])):
    if r["window_kind"] == "density_matched" and r["k"] not in seen: seen.add(r["k"]); print(r["k"], r["window"])
PY
    for rule in "none:0" "single:92" "deployed:92"; do r=${rule%%:*}; b=${rule##*:}
      echo "$EXT/ext_L106_t${T}_s1.fcd.xml $OUT/crop_t${T}_k${k}_${r}.json $b $( [ "$r" = none ] && echo single || echo $r ) $win" >> "$JOBS"
    done
  done
done
echo "[weak-crop] $(date '+%F %T') $(wc -l < "$JOBS") jobs, engine md5 $(md5sum metro_replay.py | cut -c1-8)"
run_one() { set -- $1; T=$1; O=$2; B=$3; R=$4; W=$5
  echo "[start] $(date '+%T') $(basename "$O") window $W"
  python metro_replay.py --trace "$T" --net "$NET" --out "$O" --costs "$COSTS" --charge marginal --admit_budget_ms "$B" --admit_rule "$R" --tile_window "$W" > "${O%.json}.log" 2>&1
  echo "[done rc=$?] $(date '+%T') $(basename "$O") $(grep -E '^partition' "${O%.json}.log" | cut -c1-80) $(grep -E '^(warm|replall) ' "${O%.json}.log" | sed -E 's/.*p95 load +([0-9.]+)%.*/\1%/' | tr '\n' ' ')"
}
export -f run_one; export COSTS NET
xargs -P 6 -I{} bash -c 'run_one "{}"' < "$JOBS"
echo "[weak-crop] DONE $(date '+%F %T')"
