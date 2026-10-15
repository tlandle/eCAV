#!/usr/bin/env bash
# Wait for the gate stage's driver to exit; if its check passed, continue the
# campaign with every later stage (the driver skips the gate's finished runs).
#   nohup scripts/sensys_after_gate.sh atlas|cetus > logs/after_gate_<role>.log 2>&1 &
set -u
ROLE="${1:?atlas|cetus}"
STAMP="${STAMP:-sensys_nov27}"
PORT="${CARLA_PORT:-2000}"
LOG="logs/gate_${ROLE}.log"
cd "$(dirname "$0")/.."
echo "$(date +%T) waiting for the gate driver on $ROLE"
while pgrep -f "[s]ensys_campaign[.]sh $ROLE gate" >/dev/null; do sleep 120; done
echo "$(date +%T) gate driver exited"
if grep -q "== result: PASS" "$LOG"; then
    echo "$(date +%T) gate PASS; launching the remaining stages"
    STAMP="$STAMP" CARLA_PORT="$PORT" bash scripts/sensys_campaign.sh "$ROLE" all > "logs/stages_${ROLE}.log" 2>&1
    echo "$(date +%T) stages driver exited"
else
    echo "$(date +%T) gate did not pass; nothing launched"
    grep -E "FAIL|== result" "$LOG" | tail -5
fi
