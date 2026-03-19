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
