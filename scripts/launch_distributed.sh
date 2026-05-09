#!/bin/bash
# Launch distributed eCAV scenario with remote edge server
#
# Usage:
#   ./scripts/launch_distributed.sh [scenario_name] [edge_host]
#
# Example:
#   ./scripts/launch_distributed.sh openscenario_3_edge_late_fusion_4ego 20.81.176.215
#
# This creates a tmux session "ecav" with panes for:
#   - Edge server (remote, via SSH)
#   - Orchestrator (local)
#   - Vehicle clients (local, one per vehicle)
#
# Prerequisites:
#   - CARLA running locally
#   - SSH access to edge server (azureuser@edge_host)
#   - conda env "opencda310" on both machines

set -e

SCENARIO=${1:-openscenario_3_edge_late_fusion_4ego}
EDGE_HOST=${2:-20.81.176.215}
EDGE_USER=${3:-azureuser}
EDGE_PORT=${4:-50051}
LOCAL_IP=$(hostname -I | awk '{print $1}')

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
CONDA_ACTIVATE="source ~/anaconda3/etc/profile.d/conda.sh && conda activate opencda310"
EDGE_CONDA="source ~/miniconda3/etc/profile.d/conda.sh && conda activate opencda310"

echo "=== eCAV Distributed Launch ==="
echo "Scenario:    $SCENARIO"
echo "Edge server: $EDGE_USER@$EDGE_HOST:$EDGE_PORT"
echo "Local IP:    $LOCAL_IP"
echo "Repo:        $REPO_DIR"
echo ""

# Count vehicles from the YAML config
YAML_FILE="$REPO_DIR/ecav/scenario_testing/config_yaml/${SCENARIO}.yaml"
if [ ! -f "$YAML_FILE" ]; then
    echo "ERROR: Scenario YAML not found: $YAML_FILE"
    exit 1
fi

# Kill existing session if any
tmux kill-session -t ecav 2>/dev/null || true

# Create tmux session
tmux new-session -d -s ecav -n edge

# Pane 0: Edge server (remote)
tmux send-keys -t ecav:edge "ssh $EDGE_USER@$EDGE_HOST '$EDGE_CONDA && cd ~/ecav && python -u ecav/ecav2/edge_process.py -t $SCENARIO --edge-port $EDGE_PORT --orchestrator-host $LOCAL_IP --orchestrator-port 50051'" Enter

sleep 2

# Pane 1: Orchestrator (local)
tmux split-window -h -t ecav:edge
tmux send-keys -t ecav:edge.1 "cd $REPO_DIR && $CONDA_ACTIVATE && python -u ecav.py -t $SCENARIO -d --apply_ml 2>&1 | tee distributed_${SCENARIO}.log" Enter

echo ""
echo "tmux session 'ecav' created."
echo "  Pane 0: Edge server (remote)"
echo "  Pane 1: Orchestrator (local)"
echo ""
echo "Attach with:  tmux attach -t ecav"
echo "Kill with:    tmux kill-session -t ecav"
