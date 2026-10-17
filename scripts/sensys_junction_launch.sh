#!/usr/bin/env bash
# Launch the junction stage on one host under the Oct 16 configuration
# (spawn y=30, curv_lat_accel 8, collision_time_ahead 3, buffer 16,
# prediction_steps 60, per-speed crossing phase maps). The maps hold one
# CROSS_TRIGGER_TTA value per commanded speed; the scenario picks the
# nearest key. Edit TTA_S3 / TTA_SCP here when a speed is recalibrated.
#   scripts/sensys_junction_launch.sh atlas|cetus [STAMP]
set -euo pipefail
ROLE="${1:?atlas|cetus}"; STAMP="${2:-sensys_nov27_j1}"
export TTA_S3="${TTA_S3:-38:5.25,50:5.0,53:5.0,63:5.5,70:7.0,88:9.0}"
export TTA_SCP="${TTA_SCP:-38:5.55,50:5.4,53:5.3,63:6.5,70:8.0,88:10.17}"
export EXTRA="${EXTRA:---behavior-kv curv_lat_accel=8}"
export PRED_STEPS="${PRED_STEPS:-60}"
export STAMP CARLA_PORT="${CARLA_PORT:-2000}"
cd "$(dirname "$0")/.."
if pgrep -f "^${HOME}/anaconda3/envs/opencda310/bin/python test_runner.py" >/dev/null; then echo "a runner is already active on this host"; exit 2; fi
echo "$(date +%T) launching junction stage on $ROLE stamp $STAMP tta_s3 [$TTA_S3] tta_scp [$TTA_SCP]"
nohup scripts/sensys_junction.sh "$ROLE" > "logs/junction_${ROLE}.log" 2>&1 &
echo "pid $!"
