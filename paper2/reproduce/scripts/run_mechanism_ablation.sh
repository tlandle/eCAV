#!/bin/bash
# Run amort_only and risk_only predictor policies on Zone A.
# Produces ablation_mechanisms.csv.
# These two isolated predictor policies are not exposed via the main
# profiler CLI; paper2_ablation_mechanisms.py calls profile_config
# directly.
set -e

REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$REPO"

python paper2/paper2_ablation_mechanisms.py
