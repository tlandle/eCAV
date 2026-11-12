#!/usr/bin/env bash
# Radio fidelity open-loop sweep, the cells ns-3 resolves faster than about real time first.
cd ~/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
PY=$HOME/anaconda3/envs/opencda310/bin/python
$PY -u scripts/ispass/radio_fidelity_bench.py --stamp rf_20261112 --append --ns 4 8 16 --payloads 300 1000 16896 --seeds 3 --ticks 600
$PY -u scripts/ispass/radio_fidelity_bench.py --stamp rf_20261112 --append --ns 24 31 --payloads 300 1000 --seeds 3 --ticks 600
echo "$(date '+%F %T') sweep A end"
