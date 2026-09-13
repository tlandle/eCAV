#!/bin/bash
# ONE-CELL VERIFY for the freeze-1q predictor overlay (padding + kinematic mode
# selection). Runs fd_warm_r1 through atlas_fde.sh with PRED_OVERLAY swapped, into
# a scratch dir, then the three launch gates: mode diversity (>= 95%, PADROW and
# RANKROW present), forward motion (>= 95% of moving MTR RAW calls), ISOROW present.
# Exit 0 only if every gate passes. Runs only on Tyler's go; nothing else launches
# before it passes.
MAIN=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
R=$MAIN/idfix_wt/evaluation_outputs/verify_1q
rm -rf "$R"; mkdir -p "$R"
PRED_OVERLAY=mtr_edge_predictor_1q.py OUTDIR=verify_1q EVAL_TAG=khonsu-eval-freeze-1q ORDER_CHECK=1 bash "$MAIN/scripts/atlas_fde.sh" || { echo "[verify-1q] the cell did not complete"; exit 2; }
L=$(ls "$R"/fd_warm_r1.log 2>/dev/null); [ -f "$L" ] || { echo "[verify-1q] no log"; exit 2; }
python3 "$MAIN/scripts/diag/check_mode_diversity.py" "$L" --require_rows PADROW,RANKROW && \
python3 "$MAIN/scripts/diag/check_forward_motion.py" "$L" && \
grep -q "\[ISOROW\]" "$L" && grep -q "tag=khonsu-eval-freeze-1q" "$L" && { echo "[verify-1q] ALL GATES PASS $(date '+%F %T')"; exit 0; }
echo "[verify-1q] GATE FAILURE $(date '+%F %T')"; exit 2
