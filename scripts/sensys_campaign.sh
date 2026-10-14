#!/usr/bin/env bash
# SenSys resubmission campaign driver (matrix: reviews/response_plan.md, "Campaign sizing").
# One campaign per host and per CARLA port; every invocation reuses STAMP so an
# interrupted campaign resumes in place (completed runs are skipped).
#
#   scripts/sensys_campaign.sh atlas|cetus [--dry-run]
#   STAMP=sensys_nov27 CARLA_PORT=2000 REPS=20 scripts/sensys_campaign.sh atlas
#
# Arms (manager type, anchoring):
#   late_fusion off/on   V2X2V object sharing, Naive / SBA (the contract)
#   late_fusion_etsi_self_announcement off   ETSI TR 103 562 sender-side self-announcement rule
#   vips both            V2X2V with VIPS temporal alignment, with and without the contract
#   infra_only both      I2V object sharing (RSU only), receiver-side contract on/off
#   oracle on, cip on    ground-truth identity reference and cooperative planning
# Geometries: scenario_3 left turn (ego YAMLs 2/4/8/16), SCP straight-through
# (4ego only), blind overtake on Town01 (2ego only). Mixed-traffic variants
# exist at scenario_3 4ego, SCP 4ego, blind overtake 2ego.
set -euo pipefail
ROLE="${1:?atlas|cetus}"; DRY="${2:-}"
STAMP="${STAMP:-sensys_nov27}"
PORT="${CARLA_PORT:-2000}"
REPS="${REPS:-20}"              # envelope sweep (N=1)
MIX_REPS="${MIX_REPS:-10}"      # mixed traffic (N=4 / N=2)
SCALE_REPS="${SCALE_REPS:-5}"   # scaling and faults (N=4 and above)
SIT_REPS="${SIT_REPS:-10}"      # situation study (N=1)
PY="${PY:-$HOME/anaconda3/envs/opencda310/bin/python}"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"   # CARLA agents package (needed on cetus)
LATS="0 50 100 150 200 250 300 350 400 450 500 550"
SCALE_LATS="100 200 400"
SPEEDS="50 70"
ETSI="late_fusion_etsi_self_announcement"
COMMON="--latency-distribution hybrid --self-id-radius 1.0 --stamp $STAMP --carla-port $PORT --per-tick-age-log"

run() { if [[ "$DRY" == "--dry-run" ]]; then echo "$PY test_runner.py $*"; else "$PY" test_runner.py "$@"; fi; }
# Envelope sweep and situation study run at N=1 (April density); N=4 runs take 200 s, N=1 runs 65 s
envelope_n() { echo 1; }
scaling_n()  { case "$1" in openscenario_3*) echo "2 4 8 16" ;; scp*) echo "4" ;; *) echo "2" ;; esac; }

# GATE: V2X2V with the Mahalanobis association gate (Wolf 2025 / Sensors 2026), chi-square 95% on the 7-D form
GATE_KV="assoc_metric=m_dis assoc_threshold=3.75"
case "$ROLE" in
  atlas) ARMS=("late_fusion both" "$ETSI off" "oracle on" "cip on") ;;
  cetus) ARMS=("vips both" "infra_only both" "late_fusion both $GATE_KV") ;;
  *) echo "unknown role $ROLE"; exit 2 ;;
esac
GEOMS="openscenario_3_edge_late_fusion scp_edge_late_fusion blind_overtake_edge_late_fusion"

mkdir -p experiment_results/campaign_meta
{ echo "stamp=$STAMP role=$ROLE port=$PORT reps=$REPS"; git rev-parse HEAD; date -Is; } >> "experiment_results/campaign_meta/${STAMP}_${ROLE}.txt"

# 1. Envelope sweep: 12 latencies x 2 speeds x REPS seeds at N=1
for geom in $GEOMS; do
  for arm in "${ARMS[@]}"; do
    set -- $arm; mgr=$1; anch=$2; shift 2; kv=("$@"); kvflag=(); [[ ${#kv[@]} -gt 0 ]] && kvflag=(--edge-kv "${kv[@]}")
    run -t "$geom" --manager-types "$mgr" --anchoring "$anch" --latencies $LATS --speeds $SPEEDS --ego-counts $(envelope_n "$geom") --repetitions $REPS "${kvflag[@]}" $COMMON
  done
done

# 2. Scaling: 3 latencies x SCALE_REPS seeds at 70 km/h over the ego counts each geometry has
for geom in $GEOMS; do
  for arm in "${ARMS[@]}"; do
    set -- $arm; mgr=$1; anch=$2; shift 2; kv=("$@"); kvflag=(); [[ ${#kv[@]} -gt 0 ]] && kvflag=(--edge-kv "${kv[@]}")
    run -t "$geom" --manager-types "$mgr" --anchoring "$anch" --latencies $SCALE_LATS --speeds 70 --ego-counts $(scaling_n "$geom") --repetitions $SCALE_REPS "${kvflag[@]}" $COMMON
  done
done

# 3. Situation study: oracle only, 0.75x and 1.25x of 50 and 70 km/h (cetus)
if [[ "$ROLE" == "cetus" ]]; then
  for geom in $GEOMS; do
    run -t "$geom" --manager-types oracle --anchoring on --latencies $LATS --speeds 38 63 53 88 --ego-counts 1 --repetitions $SIT_REPS $COMMON
  done
fi

# 4. Mixed traffic: V2X2V only, Naive / ETSI / SBA, at the mixed variants' ego counts (cetus)
if [[ "$ROLE" == "cetus" ]]; then
  run -t openscenario_3_edge_late_fusion_mixed --manager-types late_fusion --anchoring both --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 4 --repetitions $MIX_REPS $COMMON
  run -t openscenario_3_edge_late_fusion_mixed --manager-types "$ETSI" --anchoring off --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 4 --repetitions $MIX_REPS $COMMON
  run -t scp_edge_late_fusion_mixed --manager-types late_fusion --anchoring both --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 4 --repetitions $MIX_REPS $COMMON
  run -t scp_edge_late_fusion_mixed --manager-types "$ETSI" --anchoring off --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 4 --repetitions $MIX_REPS $COMMON
  run -t blind_overtake_edge_late_fusion_mixed --manager-types late_fusion --anchoring both --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 2 --repetitions $MIX_REPS $COMMON
  run -t blind_overtake_edge_late_fusion_mixed --manager-types "$ETSI" --anchoring off --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 2 --repetitions $MIX_REPS $COMMON
fi

# 5. Beacon faults: V2X2V Naive / ETSI / SBA, one knob at a time via --edge-kv
#    (keys of the gating track), scenario_3 and SCP at N=4, SCALE_REPS seeds, 200 ms, 70 km/h.
if [[ "$ROLE" == "atlas" ]]; then
  FAULTS="beacon_fault_loss_rate=0.1 beacon_fault_loss_rate=0.3 beacon_fault_loss_rate=0.5 beacon_fault_loss_rate=0.7 \
          beacon_fault_delay_ticks=1 beacon_fault_delay_ticks=2 beacon_fault_delay_ticks=4 beacon_fault_delay_ticks=8 \
          beacon_fault_clock_offset_ms=50 beacon_fault_clock_offset_ms=100 beacon_fault_clock_offset_ms=200 beacon_fault_clock_offset_ms=400 \
          beacon_fault_pos_noise_m=0.5 beacon_fault_pos_noise_m=1.0 beacon_fault_pos_noise_m=2.0 beacon_fault_pos_noise_m=4.0"
  for geom in openscenario_3_edge_late_fusion scp_edge_late_fusion; do
    for kv in $FAULTS; do
      run -t "$geom" --manager-types late_fusion --anchoring both --latencies 200 --speeds 70 --ego-counts 4 --repetitions $SCALE_REPS --edge-kv "$kv" $COMMON
      run -t "$geom" --manager-types "$ETSI" --anchoring off --latencies 200 --speeds 70 --ego-counts 4 --repetitions $SCALE_REPS --edge-kv "$kv" $COMMON
    done
  done
  # ValidBeacon on, same faults, SBA arm only
  for geom in openscenario_3_edge_late_fusion scp_edge_late_fusion; do
    for kv in $FAULTS; do
      run -t "$geom" --manager-types late_fusion --anchoring on --latencies 200 --speeds 70 --ego-counts 4 --repetitions $SCALE_REPS --edge-kv "$kv" beacon_validity_checks=true $COMMON
    done
  done
fi
