#!/bin/bash
# Full sweep commands for MobiCom 2026 paper
# Scenario: openscenario_3_edge_late_fusion
# 5 reps per config, hybrid latency distribution
# Single-ego: 10 latency points (0-500ms)
# Multi-ego: 3 latency points (0, 200, 400ms), N=2,4,8

set -e
cd /home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox

# === SINGLE-EGO ===

# 1. Late Fusion (both anchoring)
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types late_fusion \
  --anchoring both \
  --latencies 0 100 150 200 250 300 350 400 450 500 \
  --latency-distribution hybrid \
  --speeds 43 --self-id-radius 1.0 --repetitions 5

# 2. Oracle (anchoring on only — ego suppression is built-in)
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types oracle \
  --anchoring on \
  --latencies 0 100 150 200 250 300 350 400 450 500 \
  --latency-distribution hybrid \
  --speeds 43 --self-id-radius 1.0 --repetitions 5

# 3. VIPS Temporal (both anchoring)
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types vips_temporal \
  --anchoring both \
  --latencies 0 100 150 200 250 300 350 400 450 500 \
  --latency-distribution hybrid \
  --speeds 43 --self-id-radius 1.0 --repetitions 5

# === MULTI-EGO ===

# 4. Late Fusion multi-ego (both anchoring)
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types late_fusion \
  --anchoring both \
  --latencies 0 200 400 \
  --latency-distribution hybrid \
  --speeds 43 --ego-counts 2 4 8 --self-id-radius 1.0 --repetitions 5

# 5. Oracle multi-ego
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types oracle \
  --anchoring on \
  --latencies 0 200 400 \
  --latency-distribution hybrid \
  --speeds 43 --ego-counts 2 4 8 --self-id-radius 1.0 --repetitions 5

# 6. VIPS Temporal multi-ego (both anchoring)
python3 test_runner.py -t openscenario_3_edge_late_fusion \
  --manager-types vips_temporal \
  --anchoring both \
  --latencies 0 200 400 \
  --latency-distribution hybrid \
  --speeds 43 --ego-counts 2 4 8 --self-id-radius 1.0 --repetitions 5

# === SENSYS RESUBMISSION: GEOMETRY x SPEED x MIXED TRAFFIC (not launched) ===
# Geometry is the scenario name; speed is the --speeds knob and lands in the
# run directory as speed_<kmh>/ (test_runner.run_once). Baseline max_speed in
# every late-fusion config is 43 km/h; the campaign speeds are 50 and 70 km/h
# and the 0.75x/1.0x/1.25x multipliers of each: 50 -> 38 50 63, 70 -> 53 70 88.
#   LTAP/OD        : openscenario_3_edge_late_fusion (unchanged)
#   SCP            : scp_edge_late_fusion            (Town03, focal goes straight north)
#   blind overtake : blind_overtake_edge_late_fusion (Town01, Scenario_1)

# 7. Geometry x speed, single ego
for GEOM in openscenario_3_edge_late_fusion scp_edge_late_fusion blind_overtake_edge_late_fusion; do
python3 test_runner.py -t $GEOM \
  --manager-types late_fusion oracle vips_temporal \
  --anchoring both \
  --latencies 0 100 150 200 250 300 350 400 450 500 \
  --latency-distribution hybrid \
  --speeds 38 50 63 53 70 88 --self-id-radius 1.0 --repetitions 5
done

# 8. Mixed traffic: 50% of the CAVs non-connected (edge_list[0].non_connected_fraction: 0.5,
#    the last round(0.5 * N) vehicles of the list; vehicle 0 stays connected). The
#    mixed variants exist only at the multi-ego count, so --ego-counts is required.
python3 test_runner.py -t openscenario_3_edge_late_fusion_mixed \
  --manager-types late_fusion --anchoring both \
  --latencies 0 200 400 --latency-distribution hybrid \
  --speeds 50 70 --ego-counts 4 --self-id-radius 1.0 --repetitions 5
python3 test_runner.py -t scp_edge_late_fusion_mixed \
  --manager-types late_fusion --anchoring both \
  --latencies 0 200 400 --latency-distribution hybrid \
  --speeds 50 70 --ego-counts 4 --self-id-radius 1.0 --repetitions 5
python3 test_runner.py -t blind_overtake_edge_late_fusion_mixed \
  --manager-types late_fusion --anchoring both \
  --latencies 0 200 400 --latency-distribution hybrid \
  --speeds 50 70 --ego-counts 2 --self-id-radius 1.0 --repetitions 5

# 9. Planner response: behavior.hazard_response: speed_adapt (default stop) and
#    behavior.speed_adapt_target_kmh (default 15) are YAML keys of the vehicle
#    behavior block; set them in a config copy, there is no test_runner flag.
