#!/usr/bin/env bash
# Junction geometries (scenario_3 left turn, SCP straight crossing) under one
# configuration, with the V2X arms swept over speed as expected_effects E5
# requires. Stages under STAMP: gate (scenario_3, 70 km/h: naive, contract,
# oracle, ETSI, 6 delays, 10 seeds; campaign_check must pass), physics
# (oracle, this host's three speeds, 10 delays), speed landing (naive,
# contract, ETSI at this host's three speeds, 6 delays to 500 ms), then the
# 70 km/h landing of the remaining arms (Mahalanobis gate, VIPS, I2V) at 10
# delays on cetus. EXTRA holds the per-run knobs that define the
# configuration (e.g. "--behavior-kv curv_lat_accel=8"). TTA_S3 and TTA_SCP
# are the per-geometry CROSS_TRIGGER_TTA values (seconds of hero time to
# arrival at the conflict point at which the crossing car launches), from
# scripts/sensys_tta_probe.sh. Run only on a host with no other simulation.
#   STAMP=sensys_nov27_j1 EXTRA="--behavior-kv curv_lat_accel=8" TTA_S3=5.0 TTA_SCP=5.5 \
#     CARLA_PORT=2000 nohup scripts/sensys_junction.sh atlas > logs/junction_atlas.log 2>&1 &
set -uo pipefail
ROLE="${1:?atlas|cetus}"
STAMP="${STAMP:?set STAMP}"
EXTRA="${EXTRA:-}"
PORT="${CARLA_PORT:-2000}"
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
cd "$(dirname "$0")/.."
case "$ROLE" in atlas) SPEEDS="38 53 70" ;; cetus) SPEEDS="50 63 88" ;; *) echo "role"; exit 2 ;; esac
GEOMS="${GEOMS:-openscenario_3_edge_late_fusion scp_edge_late_fusion}"
LATS="0 50 100 150 200 300 400 500 700 1000"
LATS6="0 100 200 300 400 500"
ETSI="late_fusion_etsi_self_announcement"
GATE_KV="assoc_metric=m_dis assoc_threshold=3.75"
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port $PORT --stamp $STAMP --repetitions 10 $EXTRA"
TTA_S3="${TTA_S3:-}"; TTA_SCP="${TTA_SCP:-}"
tta_for() { case "$1" in openscenario_3*) echo "$TTA_S3" ;; scp_*) echo "$TTA_SCP" ;; esac; }
run() {   # run -t <scenario> ... ; appends the geometry's crossing trigger when set
  local sc="$2" tta; tta="$(tta_for "$sc")"
  if [[ -n "$tta" ]]; then "$PY" test_runner.py "$@" $COMMON --scenario-env "CROSS_TRIGGER_TTA=$tta"
  else "$PY" test_runner.py "$@" $COMMON; fi
}
echo "$(date +%T) $ROLE stamp $STAMP extra [$EXTRA] tta_s3 [$TTA_S3] tta_scp [$TTA_SCP]"
# 1. gate on scenario_3 at 70 km/h (both hosts run it: 20 seeds pooled)
run -t openscenario_3_edge_late_fusion --manager-types late_fusion --anchoring both --latencies $LATS6 --speeds 70
run -t openscenario_3_edge_late_fusion --manager-types oracle --anchoring on --latencies $LATS6 --speeds 70
run -t openscenario_3_edge_late_fusion --manager-types "$ETSI" --anchoring off --latencies $LATS6 --speeds 70
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || { echo "$(date +%T) gate FAILED, stopping"; exit 1; }
# 2. physics: oracle, this host's speeds, both geometries
for sc in $GEOMS; do run -t "$sc" --manager-types oracle --anchoring on --latencies $LATS --speeds $SPEEDS; done
# 3. speed landing: naive, contract, ETSI at this host's speeds, both geometries
for sc in $GEOMS; do
  run -t "$sc" --manager-types late_fusion --anchoring both --latencies $LATS6 --speeds $SPEEDS
  run -t "$sc" --manager-types "$ETSI" --anchoring off --latencies $LATS6 --speeds $SPEEDS
done
# 4. 70 km/h landing of the remaining arms, 10 delays (cetus)
if [[ "$ROLE" == "cetus" ]]; then
  for sc in $GEOMS; do
    run -t "$sc" --manager-types infra_only --anchoring off --latencies $LATS --speeds 70
    run -t "$sc" --manager-types late_fusion --anchoring both --latencies $LATS --speeds 70 --edge-kv $GATE_KV
    run -t "$sc" --manager-types vips --anchoring both --latencies $LATS --speeds 70
  done
fi
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || true
echo "$(date +%T) junction stages done on $ROLE"
