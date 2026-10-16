#!/usr/bin/env bash
# Conflict-margin probe: the oracle at 0 and 300 ms, 10 seeds, for each
# crossing-car trigger distance (hero distance at which the crossing car
# starts; the April value is 80 m; smaller = later start = tighter conflict),
# one stamp per distance. The distance to use for the tight situation is the
# largest at which the oracle still holds at zero delay and fails between
# 300 and 500 ms. Run only on a host with no other simulation.
#   CARLA_PORT=2000 nohup scripts/sensys_margin_probe.sh atlas "60 50 40" > logs/margin_probe_atlas.log 2>&1 &
set -euo pipefail
ROLE="${1:?atlas|cetus}"; DISTS="${2:-60 45 30}"
EXTRA="${EXTRA:---behavior-kv curv_lat_accel=8}"   # the configuration under probe (April turn speed)
LATS="${LATS:-0 300 500}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
GEOMS="${GEOMS:-openscenario_3_edge_late_fusion scp_edge_late_fusion}"
for d in $DISTS; do
  for sc in $GEOMS; do
    "$PY" test_runner.py -t "$sc" --manager-types oracle --anchoring on --latencies $LATS \
        --speeds 70 --repetitions 10 --cross-trigger-dist "$d" $EXTRA \
        --latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port "$PORT" \
        --stamp "margin_probe_${d}"
  done
  "$PY" scripts/sensys_analysis/campaign_check.py "margin_probe_${d}" 2>&1 | grep -E "^== |^[a-z_]+ +70 +oracle" || true
done
