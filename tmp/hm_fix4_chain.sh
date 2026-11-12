#!/usr/bin/env bash
# 2026-11-11: one code version, exact-state then the V2X2V arms (stamp sensys_hm_fix4).
cd ~/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
tmp/hm_fix_probe.sh sensys_hm_fix4 "10:0 600 700 800 900" "16:0 100 200 250 300 350" "4:0 1050 1150 1250" > logs/hm_fix4.log 2>&1
ARM="late_fusion on"  tmp/hm_fix_probe.sh sensys_hm_fix4 "10:0 100 200 400 800" "16:0 100 200" > logs/hm_fix4_sba.log 2>&1
ARM="late_fusion off" tmp/hm_fix_probe.sh sensys_hm_fix4 "10:0 100 200 400" > logs/hm_fix4_nosba.log 2>&1
echo "$(date '+%F %T') chain end" >> logs/hm_fix4_nosba.log
