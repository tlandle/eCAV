#!/usr/bin/env bash
# Stage support files into the WF->MTR export dir and stream it to PACE
# PROJECT as a single plain .tar (compression barely helps this data, and
# SCRATCH is auto-purged). The training sbatch untars onto compute-node NVMe.
set -euo pipefail

REPO=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
EXPORT_DIR="${1:-$REPO/models/multiv2x_mtr_wf_translaug}"
PACE_HOST=tlandle3@login-phoenix.pace.gatech.edu
PACE_KEY=~/.ssh/pace_ed25519
PROJECT=/storage/project/ps-cavef-0/tlandle3
TAR_NAME=wf_mtr_translaug.tar

echo "=== Staging support files into $EXPORT_DIR ==="
# dataset_info.json: the loader's record list (scenario/rsu/split).
cp "$REPO/ecav/ml_manager/models/multiv2x_mtr/dataset_info.json" \
   "$EXPORT_DIR/dataset_info.json"
# lane_maps: RSU-static BEV lane rasters for the Swin lane encoder.
mkdir -p "$EXPORT_DIR/lane_maps"
cp "$REPO"/models/lane_maps/*.png "$EXPORT_DIR/lane_maps/"

# Intention points must be regenerated from the NEW (ego-frame) GT before
# upload; fail loudly if absent rather than shipping the stale world-frame one.
if [ ! -f "$EXPORT_DIR/multiv2x_cluster_64_center_dict.pkl" ]; then
  echo "ERROR: $EXPORT_DIR/multiv2x_cluster_64_center_dict.pkl missing." >&2
  echo "Run build_multiv2x_intention_points.py against the new GT first." >&2
  exit 1
fi

echo "=== Export dir contents ==="
du -sh "$EXPORT_DIR"/* 2>/dev/null

echo "=== Streaming tar to $PACE_HOST:$PROJECT/$TAR_NAME ==="
# -C parent so the tar contains the single top dir; stream so no local copy.
tar -cf - -C "$(dirname "$EXPORT_DIR")" "$(basename "$EXPORT_DIR")" \
  | ssh -o BatchMode=yes -i "$PACE_KEY" "$PACE_HOST" \
      "cat > $PROJECT/$TAR_NAME && ls -lh $PROJECT/$TAR_NAME"

echo "=== Done. On PACE: $PROJECT/$TAR_NAME ==="
