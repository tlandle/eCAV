#!/usr/bin/env bash
# Scale sweep of AutoCastSim (hangqiu/AutoCastSim): the unprotected left turn scenario (Town03) with
# sensor sharing on, the rule-based agent, and N background vehicles. AutoCastSim attaches a LiDAR
# and a collaborator to every vehicle within 50 m of the ego, so the number of sharing vehicles is
# measured per tick and is not N. One vehicle plans; the others are CARLA traffic-manager vehicles.
# CARLA 0.9.11, 20 Hz, one process with one thread per collaborator, MQTT broker for the messages.
#   AUTOCAST=~/competitors/AutoCastSim OUT=~/competitors/results/autocast_run1 scripts/ispass/autocast_scale_sweep.sh
# Variables: NS (background vehicles), TICKS, PORT, SCEN (8 left turn, 6 overtake, 10 red light).
set -uo pipefail
AC="${AUTOCAST:?AutoCastSim checkout}"; OUT="${OUT:?output directory}"
BENCH="$(cd "$(dirname "$0")" && pwd)/coop_sim_tick_bench.py"
PY="${PY:-$HOME/miniconda3/envs/autocast/bin/python}"
NS="${NS:-0 8 16 32 64 128 256}"; TICKS="${TICKS:-300}"; PORT="${PORT:-2000}"; SCEN="${SCEN:-8}"
case "$SCEN" in 6) TOWN=01 ;; *) TOWN=03 ;; esac
cd "$AC"; CARLA_ROOT="$(readlink -f carla_root)"
CARLA_BIN="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
MQTT=$((PORT + 100)); TM=$((PORT + 50))
mkdir -p "$OUT"
stop_carla() { for p in $(pgrep -f "^$CARLA_BIN") $(pgrep -f "^/bin/sh $CARLA_ROOT/CarlaUE4.sh"); do kill -9 "$p"; done; }
stop_broker() { for p in $(pgrep -f "^mosquitto -p $MQTT"); do kill "$p"; done; }
start_carla() {   # 0.9.11 has no -RenderOffScreen; it renders off screen through SDL
  stop_carla; sleep 3
  (SDL_VIDEODRIVER=offscreen SDL_HINT_CUDA_DEVICE=0 setsid nohup "$CARLA_ROOT/CarlaUE4.sh" -carla-port="$PORT" -opengl > "$OUT/carla.log" 2>&1 < /dev/null &)
  sleep 30
}
export PYTHONPATH="$CARLA_ROOT/PythonAPI/carla:$(ls "$CARLA_ROOT"/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg):$PWD:${PYTHONPATH:-}"
export SDL_VIDEODRIVER=dummy     # the agent opens a pygame surface
{ echo "autocastsim $(git rev-parse HEAD) scenario=$SCEN"; hostname; date -Is; } >> "$OUT/meta.txt"
stop_broker; (setsid nohup mosquitto -p "$MQTT" > "$OUT/mosquitto.log" 2>&1 < /dev/null &); sleep 2
for n in $NS; do
  tag="autocast_scen${SCEN}_bg${n}"
  [[ -s "$OUT/$tag.json" ]] && grep -q '"ticks"' "$OUT/$tag.json" && continue
  start_carla
  echo "$(date '+%F %T') $tag"
  LD_LIBRARY_PATH="$(dirname "$(dirname "$PY")")/lib" timeout "${RUN_TIMEOUT:-5400}" "$PY" "$BENCH" --platform autocast --n "$n" --ticks "$TICKS" --tag "$tag" --out "$OUT/$tag.json" -- \
    scenario_runner.py --route "srunner/data/routes_training_town${TOWN}_autocast${SCEN}.xml" "srunner/data/towns${TOWN}_traffic_scenarios_autocast${SCEN}.json" \
    --reloadWorld --agent AVR/autocast_agents/simple_agent.py --bgtraffic "$n" --sharing --nocollider \
    --port "$PORT" --trafficManagerPort "$TM" --mqttport "$MQTT" --outputdir "$OUT/$tag/" > "$OUT/$tag.log" 2>&1
  echo "$(date '+%F %T') $tag exit $? $(grep -o '"tick_ms": {[^}]*}' "$OUT/$tag.json" 2>/dev/null | tr -d '\n ') $(grep -o '"sharing_vehicles": {[^}]*}' "$OUT/$tag.json" 2>/dev/null | tr -d '\n ')"
done
stop_carla; stop_broker
echo "$(date '+%F %T') sweep end"
