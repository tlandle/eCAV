#!/usr/bin/env bash
# Status of a fidelity ablation campaign on this host: runs finished per cell, the active runner, the last log lines.
#   scripts/ispass/fidelity_status.sh [stamp] [log]
cd "$(dirname "$0")/../.."
STAMP="${1:-ispass_fid}"; LOG="${2:-logs/fid_full.log}"
echo "host $(hostname) $(date '+%F %T') stamp $STAMP"
pgrep -af "^$HOME/anaconda3/envs/opencda310/bin/python test_runner.py" | cut -c1-160 || echo "no runner active"
find experiment_results/*/"$STAMP" -path '*/run_*' -maxdepth 12 -name simulation_metrics.json -not -path '*/evaluation_output/*' 2>/dev/null \
  | sed -E 's#.*/(openscenario_3_edge_late_fusion|scp_edge_late_fusion)(_wf)?/.*/(baseline_[a-z]+)/(mgr_[a-z_]+)/.*/(lat_[0-9]+)_.*sidr_1.0_(.*)_curv_lat.*/run_[0-9]+/.*#\1 \3 \4 \5 \6#' \
  | sort | uniq -c
tail -4 "$LOG" 2>/dev/null | cut -c1-140
