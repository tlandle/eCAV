#!/bin/bash
# Build every SenSys figure of one campaign stamp into OUT (PDF, PNG, CSV per figure).
#   bash scripts/sensys_analysis/build_figures.sh sensys_nov27_j3 safety_envelope_paper/analysis/figs_j3
# Inputs: the stamp's cells under experiment_results and the cetus mirror, the hosts' envelope runs.csv
# (envelope_measured.py, named envelope_<tag>_<host>), landing_ci.md, and the overtake stamps below.
# fig_landing.py reads the landing reduction and writes under safety_envelope_paper/analysis/fig_landing/.
set -e
STAMP=${1:?stamp}; OUT=${2:?out dir}
TAG=${STAMP#sensys_nov27_}
PY=/home/atlas/anaconda3/envs/opencda310/bin/python
A=safety_envelope_paper/analysis
S=scripts/sensys_analysis
ROOTS="experiment_results experiment_results/_cetus_mirror/experiment_results"
OV="sensys_nov27_ov4 sensys_nov27_ov4_onc20 sensys_nov27_ov4_onc28"   # blind overtake, three oncoming speeds
CSV="$A/envelope_${TAG}_atlas/runs.csv $A/envelope_${TAG}_cetus/runs.csv"
mkdir -p "$OUT/situation"
# reuse a previous per-run axis cache (keys are path@mtime, so stale entries are ignored)
if [ ! -f "$OUT/situation/axis_cache.json" ]; then
  prev=$(ls -t $A/fig_situation*/axis_cache.json 2>/dev/null | head -1)
  [ -n "$prev" ] && cp "$prev" "$OUT/situation/axis_cache.json"
fi
export PYTHONPATH=.:ecav/protos
$PY $S/fig_envelope.py --csv $CSV --speed 70 --out "$OUT/arch_safety_scale.pdf"
$PY $S/fig_situation.py --stamps "$STAMP" sensys_nov27_ov4 --roots $ROOTS --out "$OUT/situation" | tail -60
$PY $S/fig_safety_margin.py --tau "$OUT/situation/fig_situation_tau.csv" --landing $A/landing_ci.md --out "$OUT/safety_margin.pdf"
$PY $S/fig_failure_decomp.py --stamp "$STAMP" --roots $ROOTS --overtake-stamps $OV --out "$OUT/arch_failure_decomp.pdf" --sweep "$OUT/arch_failure_sweep.pdf"
$PY $S/fig_faults.py --stamp "$STAMP" --roots $ROOTS --geom openscenario_3_edge_late_fusion --lat 400 --out "$OUT/faults_lt.pdf"
$PY $S/fig_faults.py --stamp "$STAMP" --roots $ROOTS --geom scp_edge_late_fusion --lat 200 --out "$OUT/faults_scp.pdf"
$PY $S/fig_faults.py --csvs "$OUT/faults_lt.csv" "$OUT/faults_scp.csv" --labels "left turn, 400 ms" "straight crossing, 200 ms" \
    --grid "$OUT/faults_grid.pdf" --summary "$OUT/faults_summary.pdf"
$PY $S/fig_ablation.py --csv $CSV --geom openscenario_3_edge_late_fusion --speed 70 --out "$OUT/sba_ablation.pdf"
$PY $S/fig_trace_replay.py --runs $CSV --stamp "$STAMP" --root experiment_results --out "$OUT/trace_replay.pdf" | tail -15
$PY $S/fig_landing.py | tail -3
echo "figures under $OUT (landing under $A/fig_landing)"
