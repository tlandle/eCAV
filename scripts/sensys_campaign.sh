#!/usr/bin/env bash
# SenSys resubmission campaign driver. One campaign per host and per CARLA port.
# Usage: scripts/sensys_campaign.sh <host-role> [--dry-run]
#   host-role: atlas | cetus          (splits the arm list across the two machines)
# Every invocation reuses STAMP so an interrupted campaign resumes in place.
set -euo pipefail
ROLE="${1:?atlas|cetus}"; DRY="${2:-}"
STAMP="${STAMP:-sensys_nov27}"          # shared stamp, resume-safe
PORT="${CARLA_PORT:-2000}"
REPS="${REPS:-20}"
LATS="0 50 100 150 200 250 300 350 400 450 500 550"
SCALE_LATS="100 200 400"
GEOMS="openscenario_3_edge_late_fusion scp_edge_late_fusion blind_overtake_edge_late_fusion"
SPEEDS="50 70"
ETSI_MGR="${ETSI_MGR:-late_fusion_etsi_self_announcement}"   # set to the registered name of the ETSI arm
COMMON="--self-id-radius 1.0 --repetitions $REPS --stamp $STAMP --carla-port $PORT --per-tick-age-log"

run() { if [[ "$DRY" == "--dry-run" ]]; then echo "python3 test_runner.py $*"; else python3 test_runner.py "$@"; fi; }

# Arms: (manager-type, anchoring). oracle and cip carry the contract by construction.
case "$ROLE" in
  atlas) ARMS=("late_fusion both" "oracle on" "$ETSI_MGR off") ;;
  cetus) ARMS=("vips both" "infra_only both" "cip on") ;;
  *) echo "unknown role $ROLE"; exit 2 ;;
esac

for geom in $GEOMS; do
  for arm in "${ARMS[@]}"; do
    set -- $arm; mgr=$1; anch=$2
    # envelope sweep, single focal vehicle
    run -t "$geom" --manager-types "$mgr" --anchoring "$anch" --latencies $LATS --speeds $SPEEDS $COMMON
    # scaling at three latencies (ego-count variants exist for scenario_3 and scp at 4, blind overtake at 2)
    case "$geom" in
      blind_overtake_edge_late_fusion) egos="2" ;;
      *) egos="2 4 8 16" ;;
    esac
    run -t "$geom" --manager-types "$mgr" --anchoring "$anch" --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts $egos $COMMON
  done
done

# Situation study: oracle only, speed multipliers 0.75x and 1.25x of 50 and 70
if [[ "$ROLE" == "cetus" ]]; then
  for geom in $GEOMS; do
    run -t "$geom" --manager-types oracle --anchoring on --latencies $LATS --speeds 38 63 53 88 $COMMON
  done
fi

# Faults and mixed traffic: V2X2V only, Naive / ETSI / SBA (atlas)
if [[ "$ROLE" == "atlas" ]]; then
  for geom in openscenario_3_edge_late_fusion_mixed scp_edge_late_fusion_mixed; do
    run -t "$geom" --manager-types late_fusion "$ETSI_MGR" --anchoring both --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 4 $COMMON
  done
  run -t blind_overtake_edge_late_fusion_mixed --manager-types late_fusion "$ETSI_MGR" --anchoring both --latencies $SCALE_LATS --speeds $SPEEDS --ego-counts 2 $COMMON
  # beacon fault knobs are YAML keys from the gating track; add their test_runner flags here once merged
fi
