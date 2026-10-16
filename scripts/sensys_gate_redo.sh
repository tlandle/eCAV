#!/usr/bin/env bash
# After the gate driver and its watcher have exited: if the watcher launched
# the stages, do nothing; otherwise resume the gate (void runs are redone,
# finished ones skipped) and start the watcher again, which launches the
# stages on a pass.
#   nohup scripts/sensys_gate_redo.sh atlas|cetus > logs/gate_redo_<role>.log 2>&1 &
set -u
ROLE="${1:?atlas|cetus}"
STAMP="${STAMP:-sensys_nov27}"
PORT="${CARLA_PORT:-2000}"
cd "$(dirname "$0")/.."
echo "$(date +%T) waiting for the gate driver and watcher on $ROLE"
while pgrep -f "[s]ensys_campaign[.]sh $ROLE gate" >/dev/null || pgrep -f "[s]ensys_after_gate[.]sh $ROLE" >/dev/null; do sleep 120; done
if grep -q "launching the remaining stages" "logs/after_gate_${ROLE}.log" 2>/dev/null; then
    echo "$(date +%T) stages already launched by the watcher; nothing to do"
    exit 0
fi
echo "$(date +%T) gate did not pass; resuming the gate (void runs redone)"
mv "logs/gate_${ROLE}.log" "logs/gate_${ROLE}_first_pass.log" 2>/dev/null
(STAMP="$STAMP" CARLA_PORT="$PORT" setsid nohup bash scripts/sensys_campaign.sh "$ROLE" gate > "logs/gate_${ROLE}.log" 2>&1 < /dev/null &)
sleep 5
mv "logs/after_gate_${ROLE}.log" "logs/after_gate_${ROLE}_first_pass.log" 2>/dev/null
(setsid nohup bash scripts/sensys_after_gate.sh "$ROLE" > "logs/after_gate_${ROLE}.log" 2>&1 < /dev/null &)
echo "$(date +%T) gate resumed and watcher restarted"
