#!/usr/bin/env bash
# Scale sweep of upstream OpenCDA on the eCAV scale workload (see opencda_scale_bench.py).
# Run on the benchmark host from any directory:
#   OPENCDA=~/competitors/OpenCDA GEN=~/scale_wt/scripts/ispass/gen_scale_town06.py \
#   OUT=~/competitors/results/run1 scripts/ispass/opencda_scale_sweep.sh
# Variables: NS (vehicle counts), LAYOUTS, PERCEPTION (on|off), MAPMGR (on|off), TICKS, PY, CARLA_ROOT.
# 300 ticks up to 128 vehicles and 100 above, as in scripts/ispass/scale_sweep.sh.
set -uo pipefail
OPENCDA="${OPENCDA:?OpenCDA checkout}"; GEN="${GEN:?gen_scale_town06.py}"; OUT="${OUT:?output directory}"
BENCH="$(cd "$(dirname "$0")" && pwd)/opencda_scale_bench.py"
PY="${PY:-$HOME/miniconda3/envs/opencda_up/bin/python}"; CARLA_ROOT="${CARLA_ROOT:-$HOME/carla-0.9.15}"
NS="${NS:-8 16 32 64 128 256}"; LAYOUTS="${LAYOUTS:-dense spread}"; PERCEPTION="${PERCEPTION:-off}"; MAPMGR="${MAPMGR:-on}"
CARLA_BIN="$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping"
mkdir -p "$OUT"
# Every CARLA sensor stream is a socket in the client and OpenCDA attaches four sensors per vehicle
# without perception: 256 vehicles exceed the default limit of 1024 open files.
ulimit -n 65536 2>/dev/null || ulimit -n "$(ulimit -Hn)"
stop_carla() { for p in $(pgrep -f "^$CARLA_BIN") $(pgrep -f "^/bin/sh $CARLA_ROOT/CarlaUE4.sh"); do kill -9 "$p"; done; }
start_carla() {
  stop_carla; sleep 3
  (setsid nohup "$CARLA_ROOT/CarlaUE4.sh" -RenderOffScreen -carla-port=2000 > "$OUT/carla.log" 2>&1 < /dev/null &)
  sleep 25
}
{ echo "opencda $(git -C "$OPENCDA" rev-parse HEAD)"; hostname; date -Is; "$PY" -c 'import numpy, shapely, carla; print("numpy", numpy.__version__, "shapely", shapely.__version__)'; } >> "$OUT/meta.txt"
cd "$OPENCDA"
for layout in $LAYOUTS; do for n in $NS; do
  tag="opencda_${layout}_n${n}_perc${PERCEPTION}_map${MAPMGR}"
  [[ -s "$OUT/$tag.json" ]] && grep -q '"completed": true' "$OUT/$tag.json" && continue
  ticks=300; (( n > 128 )) && ticks=100
  [[ -n "${TICKS:-}" ]] && ticks=$TICKS     # fewer ticks where a tick takes many seconds (perception at 64 and above)
  start_carla
  echo "$(date '+%F %T') $tag ticks=$ticks"
  timeout "${RUN_TIMEOUT:-5400}" "$PY" "$BENCH" --gen "$GEN" -n "$n" --layout "$layout" --perception "$PERCEPTION" \
    --map-manager "$MAPMGR" --ticks "$ticks" --out "$OUT/$tag.json" > "$OUT/$tag.log" 2>&1
  echo "$(date '+%F %T') $tag exit $? $(grep -o '"tick_ms": {[^}]*}' "$OUT/$tag.json" 2>/dev/null | tr -d '\n ')"
done; done
stop_carla
echo "$(date '+%F %T') sweep end"
