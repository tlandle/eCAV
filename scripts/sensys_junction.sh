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
# Edge prediction horizon in 50 ms steps (71aeae47): one --edge-kv flag per
# call, since a repeated flag keeps only the last one; the Mahalanobis arm
# therefore carries the horizon and its gate pairs in one flag below.
PRED_STEPS="${PRED_STEPS:-60}"
EKV="--edge-kv prediction_steps=$PRED_STEPS"
# Seeds: 10 for the arms with perception and timing noise; the oracle is
# deterministic (three seeds of stallfix_t1 identical to the digit), so
# ORACLE_REPS (default 3) verify it without 7 identical copies per cell.
DEF_REPS="${DEF_REPS:-10}"; OREPS="${ORACLE_REPS:-3}"
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port $PORT --stamp $STAMP $EXTRA"
TTA_S3="${TTA_S3:-}"; TTA_SCP="${TTA_SCP:-}"
tta_for() { case "$1" in openscenario_3*) echo "$TTA_S3" ;; scp_*) echo "$TTA_SCP" ;; esac; }
run() {   # run -t <scenario> ... ; appends seeds (unless given) and the geometry's crossing trigger
  local sc="$2" tta reps="$DEF_REPS"; tta="$(tta_for "$sc")"
  case " $* " in *" --repetitions "*) reps="" ;; esac
  "$PY" test_runner.py "$@" $COMMON ${reps:+--repetitions $reps} ${tta:+--scenario-env CROSS_TRIGGER_TTA=$tta}
}
echo "$(date +%T) $ROLE stamp $STAMP extra [$EXTRA] pred_steps [$PRED_STEPS] seeds [$DEF_REPS/oracle $OREPS] tta_s3 [$TTA_S3] tta_scp [$TTA_SCP]"
# 1. gate on scenario_3 at 70 km/h (both hosts run it: 20 seeds pooled)
run -t openscenario_3_edge_late_fusion --manager-types late_fusion --anchoring both --latencies $LATS6 --speeds 70 $EKV
run -t openscenario_3_edge_late_fusion --manager-types oracle --anchoring on --latencies $LATS6 --speeds 70 --repetitions $OREPS $EKV
run -t openscenario_3_edge_late_fusion --manager-types "$ETSI" --anchoring off --latencies $LATS6 --speeds 70 $EKV
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || { echo "$(date +%T) gate FAILED, stopping"; exit 1; }
# 2. physics: oracle, this host's speeds, both geometries
for sc in $GEOMS; do run -t "$sc" --manager-types oracle --anchoring on --latencies $LATS --speeds $SPEEDS --repetitions $OREPS $EKV; done
# 3. speed landing: naive, contract, ETSI at this host's speeds, both geometries
for sc in $GEOMS; do
  run -t "$sc" --manager-types late_fusion --anchoring both --latencies $LATS6 --speeds $SPEEDS $EKV
  run -t "$sc" --manager-types "$ETSI" --anchoring off --latencies $LATS6 --speeds $SPEEDS $EKV
done
# 4. 70 km/h landing of the remaining arms, 10 delays (cetus)
if [[ "$ROLE" == "cetus" ]]; then
  for sc in $GEOMS; do
    run -t "$sc" --manager-types infra_only --anchoring off --latencies $LATS --speeds 70 $EKV
    run -t "$sc" --manager-types late_fusion --anchoring both --latencies $LATS --speeds 70 --edge-kv prediction_steps=$PRED_STEPS $GATE_KV
    run -t "$sc" --manager-types vips --anchoring both --latencies $LATS --speeds 70 $EKV
  done
fi
"$PY" scripts/sensys_analysis/campaign_check.py "$STAMP" || true
echo "$(date +%T) junction stages done on $ROLE"
