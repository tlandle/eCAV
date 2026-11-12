#!/usr/bin/env bash
# 2026-11-12: finish sweep3 on the A10 node: the missing sequential run, N=512 spread, then the perception part.
cd ~/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
H=azureuser@20.81.176.215
scripts/ispass/scale_sweep.sh sweep3 $H seq > logs/scale_sweep3_seq2.log 2>&1
timeout 5400 scripts/ispass/launch_scale.sh -n 512 -r 20 -p off -s 100 -g "--layout spread" -o $H -t sweep3_spread_n512 > logs/scale_n512.log 2>&1
scripts/ispass/scale_sweep.sh sweep3 $H percep > logs/scale_sweep3c.log 2>&1
echo "$(date '+%T') chain3 end" >> logs/scale_sweep3c.log
