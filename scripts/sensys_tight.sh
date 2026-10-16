#!/usr/bin/env bash
# The tight-margin situation: the same two junction geometries with the
# crossing car triggered at TRIGGER_DIST (April: 80 m), so the oracle's
# physical boundary falls inside the age sweep. Stages, each under the stamp
# sensys_nov27_tight<dist>: gate (naive, contract, oracle, 6 delays, 10 seeds,
# 70 km/h), physics (oracle, this host's three speeds, 10 delays), landing
# (naive, contract, ETSI at 70 km/h, 10 delays). The gate check must pass
# before the stages run. Run only on a host with no other simulation.
#   TRIGGER_DIST=50 CARLA_PORT=2000 nohup scripts/sensys_tight.sh atlas > logs/tight_atlas.log 2>&1 &
set -euo pipefail
ROLE="${1:?atlas|cetus}"
TRIGGER_DIST="${TRIGGER_DIST:?set TRIGGER_DIST (m)}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
STAMP="sensys_nov27_tight${TRIGGER_DIST}"
case "$ROLE" in atlas) SPEEDS="38 53 70" ;; cetus) SPEEDS="50 63 88" ;; *) echo "role"; exit 2 ;; esac
GEOMS="${GEOMS:-openscenario_3_edge_late_fusion scp_edge_late_fusion}"
LATS="0 50 100 150 200 300 400 500 700 1000"
ETSI="late_fusion_etsi_self_announcement"
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port $PORT --stamp $STAMP --cross-trigger-dist $TRIGGER_DIST --repetitions 10"
run() { "$PY" test_runner.py "$@" $COMMON; }
# gate on scenario_3 at 70 km/h
run -t openscenario_3_edge_late_fusion --manager-types late_fusion --anchoring both --latencies 0 100 200 300 400 500 --speeds 70
run -t openscenario_3_edge_late_fusion --manager-types oracle --anchoring on --latencies 0 100 200 300 400 500 --speeds 70
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || { echo "tight gate FAILED, stopping"; exit 1; }
# physics: oracle, this host's speeds, both geometries
for sc in $GEOMS; do run -t "$sc" --manager-types oracle --anchoring on --latencies $LATS --speeds $SPEEDS; done
# landing: naive, contract, ETSI at 70 km/h, both geometries
for sc in $GEOMS; do
  run -t "$sc" --manager-types late_fusion --anchoring both --latencies $LATS --speeds 70
  run -t "$sc" --manager-types "$ETSI" --anchoring off --latencies $LATS --speeds 70
done
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || true
