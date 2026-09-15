#!/bin/bash
# Locale-sizing sweep on the Athens replay (Figure 11 rebuilt measured; design:
# docs/agent_plans/locale_sizing_replay_design.md). Warm arm, deployed row,
# marginal charging, tile side swept. SIDES="100 125 ..." (default 100..600 by
# 25), PAR parallel jobs (default 6). Skips sides whose JSON exists. CPU only.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
OUT=$MAIN/evaluation_outputs/metro/sizing; mkdir -p "$OUT"
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json
PN=/home/atlas/datasets/pneuma/extract/pNEUMA_dataset/20181024_dX_0900_0930.csv; PNET=$SP/metro/athens_pneuma_pad50.net.xml
PY=/home/atlas/anaconda3/envs/metro_env/bin/python
export PYTHONHASHSEED=0
SIDES=${SIDES:-"100 125 150 175 200 225 250 275 300 325 350 375 400 425 450 475 500 525 550 575 600"}
PAR=${PAR:-6}
cd "$MAIN/scripts/metro" || exit 1
JOBS=$OUT/_jobs.txt; : > "$JOBS"
for s in $SIDES; do [ -f "$OUT/replay_sizing_t${s}.json" ] || echo "$s" >> "$JOBS"; done
echo "[sizing-sweep] $(date '+%F %T') $(wc -l < "$JOBS") sides to run, $PAR parallel, engine md5 $(md5sum metro_replay.py | cut -c1-8)"
run_one () {
  s=$1; L=$OUT/replay_sizing_t${s}.log
  echo "start $(date '+%F %T') engine $(md5sum metro_replay.py | cut -c1-8)" > "$L"
  /usr/bin/time -f "wall_s=%e maxrss_kB=%M" "$PY" metro_replay.py --trace "$PN" --net "$PNET" --out "$OUT/replay_sizing_t${s}.json" \
    --costs "$COSTS" --charge marginal --admit_budget_ms 92 --admit_rule deployed --arms warm --tile "$s" >> "$L" 2>&1
  echo "exit=$? end $(date '+%F %T')" >> "$L"
  echo "[sizing-sweep] side $s done $(date '+%T') $(tail -1 "$L")"
}
export -f run_one; export OUT PY PN PNET COSTS
xargs -P "$PAR" -I{} bash -c 'run_one {}' < "$JOBS"
"$PY" "$MAIN/scripts/metro/sizing_rows.py" --canvas "$MAIN/evaluation_outputs/canvas_latency.csv" --out "$OUT/frozen1r_sizing_rows.csv" \
  --reference "$MAIN/evaluation_outputs/metro/pneuma_0900_replay_deployed.json" "$OUT"/replay_sizing_t*.json > "$OUT/sizing_checks.txt" 2>&1
tail -30 "$OUT/sizing_checks.txt"
echo "[sizing-sweep] DONE $(date '+%F %T')" | tee "$OUT/_done"
