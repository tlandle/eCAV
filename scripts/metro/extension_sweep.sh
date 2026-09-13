#!/bin/bash
# Step 3: SUMO extension of the real Athens network to ~100 locales at 300 m
# (3.0 km window), demand at the figure's interior grid {200,400,800,1600,3200}
# active tracks plus the top point at the pNEUMA-measured 14.9 veh/lane-km, one
# 900 s trace per point, then the replay engine on each trace under the three
# admission rows (demand / K=16 capacity / deployed rule) and marginal charging.
# CPU only. Labelled calibrated extrapolation. Output: evaluation_outputs/metro/ext/
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
SP=/tmp/claude-1000/-home-atlas-TrafficSimulator-eCloud-ecloudsim-distributed-sandbox/88f56d4f-259d-44e9-bdb2-c8ace70b6018/scratchpad
OUT=$MAIN/evaluation_outputs/metro/ext; mkdir -p "$OUT"
COSTS=$MAIN/evaluation_outputs/metro_microbench/microbench_merged_v2.json
source /home/atlas/anaconda3/etc/profile.d/conda.sh && conda activate metro_env
export SUMO_HOME=$CONDA_PREFIX/lib/python3.10/site-packages/sumo; export PATH=$SUMO_HOME/bin:$PATH
cd "$SP/metro" || exit 1
SEED=${SEED:-1}; WIN=${WIN:-3.0}
TOP=$(python - <<PY
import json,glob,os
# top point: 14.9 veh/lane-km on the cropped window's lane-km (read from a prior meta if present, else estimated from the 2.8 km dry crop scaled)
m=sorted(glob.glob("$OUT/ext_L*_t*_s*.meta.json"))
lk = json.load(open(m[0]))["lane_km"] if m else 210.7*(float("$WIN")/2.8)**2
print(int(round(14.9*lk)))
PY
)
echo "[ext-sweep] $(date '+%F %T') window $WIN km, top point target $TOP active (14.9 veh/lane-km)"
for T in 200 400 800 1600 3200 $TOP; do
  F=$(ls "$OUT"/ext_L*_t${T}_s${SEED}.fcd.xml 2>/dev/null | head -1)
  if [ -z "$F" ]; then
    python sumo_extension.py --osm athens_ext.osm --full_net athens_ext.net.xml --window_km $WIN --target_active $T --sim_s 900 --calib_s 300 --seed $SEED --out "$OUT" 2>&1 | tail -3
    F=$(ls "$OUT"/ext_L*_t${T}_s${SEED}.fcd.xml 2>/dev/null | head -1)
  fi
  [ -n "$F" ] || { echo "[ext-sweep] no trace for target $T"; continue; }
  NET=$(ls "$OUT"/athens_w${WIN}.net.xml)
  for rule in "none:0" "single:92" "deployed:92"; do r=${rule%%:*}; b=${rule##*:}
    O="$OUT/replay_$(basename "$F" .fcd.xml)_${r}.json"; [ -f "$O" ] && continue
    python metro_replay.py --trace "$F" --net "$NET" --out "$O" --costs "$COSTS" --charge marginal --admit_budget_ms $b --admit_rule $( [ "$r" = none ] && echo single || echo $r ) 2>&1 | grep -E "^partition|^N_owned|^(warm|replall|delta|reactive)" | cut -c1-200
  done
done
python "$MAIN/scripts/metro/metro_rows.py" "$OUT/fig_metro_extension_rows.csv" "$OUT"/replay_*.json | head -3
echo "[ext-sweep] DONE $(date '+%F %T')"
