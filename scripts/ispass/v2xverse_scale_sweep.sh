#!/usr/bin/env bash
# Scale sweep of V2Xverse (CollaborativePerception/V2Xverse) closed-loop driving: N communicating
# driving vehicles (--ego-num), each with LiDAR, its own roadside unit, collaborative perception
# and the end-to-end planner, in V2Xverse's single evaluator process. Town05 short route 0 (the
# platform's evaluation routes; it has no Town06 route set). CARLA 0.9.10.1, 20 Hz.
#   V2XVERSE=~/competitors/V2Xverse OUT=~/competitors/results/v2xverse_run1 scripts/ispass/v2xverse_scale_sweep.sh
# Variables: NS, METHOD (codriving|late|single, the agent configs pnp_config_<METHOD>_5_10.yaml),
# BG (default: the platform's 60 vehicles and 60 pedestrians; 0: no background traffic), TICKS, ROUTE, PORT.
set -uo pipefail
V2X="${V2XVERSE:?V2Xverse checkout}"; OUT="${OUT:?output directory}"
BENCH="$(cd "$(dirname "$0")" && pwd)/coop_sim_tick_bench.py"
PY="${PY:-$HOME/miniconda3/envs/v2xverse/bin/python}"
NS="${NS:-1 2 4 8 16 32 64}"; METHOD="${METHOD:-codriving}"; BG="${BG:-0}"; TICKS="${TICKS:-300}"; ROUTE="${ROUTE:-0}"; PORT="${PORT:-40000}"
cd "$V2X"; CARLA_ROOT="$(readlink -f external_paths/carla_root)"
CARLA_BIN="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
mkdir -p "$OUT"
stop_carla() { for p in $(pgrep -f "^$CARLA_BIN") $(pgrep -f "^/bin/sh $CARLA_ROOT/CarlaUE4.sh"); do kill -9 "$p"; done; }
start_carla() {   # 0.9.10.1 has no -RenderOffScreen; it renders off screen through SDL
  stop_carla; sleep 3
  (SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 setsid nohup "$CARLA_ROOT/CarlaUE4.sh" --world-port="$PORT" -opengl > "$OUT/carla.log" 2>&1 < /dev/null &)
  sleep 30
}
export LEADERBOARD_ROOT=simulation/leaderboard SCENARIO_RUNNER_ROOT=simulation/scenario_runner
export PYTHONPATH="$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:$LEADERBOARD_ROOT:$LEADERBOARD_ROOT/team_code:$SCENARIO_RUNNER_ROOT:$PWD:${PYTHONPATH:-}"
export CHALLENGE_TRACK_CODENAME=SENSORS DEBUG_CHALLENGE=0 DATA_ROOT=external_paths/data_root
export ROUTES="simulation/leaderboard/data/evaluation_routes/town05_short_r${ROUTE}.xml"   # the agent names its output after it
{ echo "v2xverse $(git rev-parse HEAD) method=$METHOD bg=$BG route=$ROUTE"; hostname; date -Is; } >> "$OUT/meta.txt"
for n in $NS; do
  tag="v2xverse_${METHOD}_n${n}_bg${BG}"
  [[ -s "$OUT/$tag.json" ]] && grep -q '"completed": true' "$OUT/$tag.json" && continue
  # agent and scenario configuration: copies of the platform's files with the vehicle count and the background amounts set
  "$PY" - "$n" "$BG" "$METHOD" "$OUT/$tag" <<'PYEOF'
import sys, yaml
n, bg, method, base = int(sys.argv[1]), sys.argv[2], sys.argv[3], sys.argv[4]
c = yaml.safe_load(open(f"simulation/leaderboard/team_code/agent_config/pnp_config_{method}_5_10.yaml"))
c["simulation"]["ego_num"] = n
yaml.safe_dump(c, open(base + "_agent.yaml", "w"))
s = yaml.safe_load(open("simulation/leaderboard/leaderboard/scenarios/scenario_parameter.yaml"))
if bg == "0":
    s["Background"]["vehicle_amount"] = 0; s["Background"]["pedestrian_amount"] = 0
yaml.safe_dump(s, open(base + "_scenario.yaml", "w"))
PYEOF
  export SAVE_PATH="$OUT/$tag/image" RESULT_ROOT="$OUT/$tag" CHECKPOINT_ENDPOINT="$OUT/$tag/results.json"
  mkdir -p "$SAVE_PATH"
  start_carla
  echo "$(date '+%F %T') $tag"
  LD_LIBRARY_PATH="$(dirname "$(dirname "$PY")")/lib" timeout "${RUN_TIMEOUT:-5400}" "$PY" "$BENCH" --platform v2xverse --n "$n" --ticks "$TICKS" --tag "$tag" --out "$OUT/$tag.json" -- \
    simulation/leaderboard/leaderboard/leaderboard_evaluator_parameter.py \
    --scenarios=simulation/leaderboard/data/scenarios/town05_all_scenarios_2.json --scenario_parameter="$OUT/${tag}_scenario.yaml" \
    --routes="simulation/leaderboard/data/evaluation_routes/town05_short_r${ROUTE}.xml" --repetitions=1 --track=SENSORS \
    --checkpoint="$CHECKPOINT_ENDPOINT" --agent=simulation/leaderboard/team_code/pnp_agent_e2e.py --agent-config="$OUT/${tag}_agent.yaml" \
    --debug=0 --resume=0 --port="$PORT" --trafficManagerPort=$((PORT + 5)) --carlaProviderSeed=2000 --trafficManagerSeed=2000 \
    --ego-num="$n" --timeout 600 --skip_existed=0 > "$OUT/$tag.log" 2>&1
  echo "$(date '+%F %T') $tag exit $? $(grep -o '"tick_ms": {[^}]*}' "$OUT/$tag.json" 2>/dev/null | tr -d '\n ')"
  grep -q '"completed": true' "$OUT/$tag.json" 2>/dev/null || { echo "stopping at the first vehicle count that does not complete"; break; }
done
stop_carla
echo "$(date '+%F %T') sweep end"
