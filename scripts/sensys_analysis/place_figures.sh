#!/bin/bash
# Copy one campaign build's figures (build_figures.sh output dir) into the paper repo under the names the
# floats include, force-add the PDFs (the paper's .gitignore drops *.pdf) and list what changed.
#   bash scripts/sensys_analysis/place_figures.sh safety_envelope_paper/analysis/figs_j5 ~/repos/safety_envelope_sensys_nov27
# Rebuilding later with the full data and running this again refreshes every figure in place.
set -e
SRC=${1:?build dir}; PAPER=${2:?paper repo}
declare -A MAP=(
  [arch_safety_scale.pdf]=arch_safety_scale.pdf
  [situation/fig_situation.pdf]=situation_sensitivity.pdf
  [situation/fig_situation_ref.pdf]=situation_reference.pdf
  [situation/fig_situation_contract.pdf]=situation_contract.pdf
  [situation/fig_situation_control.pdf]=situation_control.pdf
  [safety_margin.pdf]=safety_margin.pdf
  [arch_failure_decomp.pdf]=arch_failure_decomp.pdf
  [arch_failure_sweep.pdf]=arch_failure_sweep.pdf
  [faults_grid.pdf]=faults_grid.pdf
  [faults_summary.pdf]=faults_summary.pdf
  [sba_ablation.pdf]=sba_ablation.pdf
  [trace_replay.pdf]=trace_replay.pdf
)
placed=()
for k in "${!MAP[@]}"; do
  if [ -f "$SRC/$k" ]; then cp "$SRC/$k" "$PAPER/figures/${MAP[$k]}"; placed+=("figures/${MAP[$k]}"); else echo "missing in build: $k (paper keeps figures/${MAP[$k]})"; fi
done
# the budget table rows (floats/tbl-budget.tex reads them) travel with the figures
[ -f "$SRC/tbl_budget_thresholds.tex" ] && cp "$SRC/tbl_budget_thresholds.tex" "$PAPER/floats/tbl_budget_thresholds.tex" && placed+=("floats/tbl_budget_thresholds.tex")
cd "$PAPER" && git add -f "${placed[@]}" && git status --short -- figures floats | sed 's/^/  /'
echo "placed ${#placed[@]} files from $SRC"
