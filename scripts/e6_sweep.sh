#!/bin/bash
# E6 state-model divergence sweep (ISPASS): arms x weather x repetitions.
# Design: docs/agent_plans/ispass_divergence_scenarios.md
#
# Arms (S1 blind overtake, Scenario_1 XML):
#   e6a  = openscenario_1_e6a_oracle        (GT content, no fusion compute)
#   clat = openscenario_1_e6clat_gt_latency (GT content, arm-C latency)
#   e6b  = openscenario_1_e6b_ego_local     (ego-local perception, no edge)
#   e6c  = openscenario_1_edge_worldfusion  (full sensor-derived stack)
#
# Weather variants are generated on the fly (yaml+py copies, suffix _w1/_w2);
# w0 is the base config's clear weather. Each cell runs REPS times; outputs
# are archived under $OUT/<arm>_<weather>/rep<i>/ with a DONE marker, so the
# sweep resumes idempotently.
#
# DRY RUN by default: prints the plan. Set E6_GO=1 to execute (GPU runs only
# with Tyler's go-ahead). Requires CarlaUE4 already running (start_actors.sh).
set -u
source /home/atlas/anaconda3/etc/profile.d/conda.sh
conda activate opencda310
ROOT=/home/atlas/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox
OUT=$ROOT/paper_ispass/e6_sweep
LOG=$OUT/e6_sweep.log
REPS=${REPS:-10}
cd $ROOT
mkdir -p "$OUT"

# K-ladder: e6a=oracle, clat=GT+latency, e6b=no-edge endpoint, edge_worldfusion=C-K1;
# add e6c_k2+ arms here once the observer-CAV binding is smoke-validated.
ARMS="e6a_oracle e6clat_gt_latency e6b_ego_local edge_worldfusion"
# name:cloudiness:precipitation:fog_density:wetness
WEATHERS="w0:0:0:0:0 w1:75:30:40:30 w2:100:60:80:60"

gen_variant () {  # $1 base scenario name, $2 weather tag, $3..$6 weather values
  local base=$1 tag=$2 cl=$3 pr=$4 fog=$5 wet=$6
  local name="${base}_${tag}"
  [ "$tag" = "w0" ] && { echo "$base"; return; }
  if [ ! -f "ecav/scenario_testing/config_yaml/${name}.yaml" ]; then
    python3 - "$base" "$name" "$cl" "$pr" "$fog" "$wet" <<'PYEOF'
import sys, yaml
base, name, cl, pr, fog, wet = sys.argv[1:7]
p = f'ecav/scenario_testing/config_yaml/{base}.yaml'
d = yaml.safe_load(open(p))
w = d['world']['weather']
w['cloudiness'] = float(cl); w['precipitation'] = float(pr)
w['precipitation_deposits'] = float(pr)
w['fog_density'] = float(fog); w['wetness'] = float(wet)
if 'scenario_name' in d: d['scenario_name'] = name
# anchors resolve on load; the dump is semantically identical
yaml.safe_dump(d, open(f'ecav/scenario_testing/config_yaml/{name}.yaml','w'),
               default_flow_style=False, sort_keys=False)
src = open(f'ecav/scenario_testing/{base}.py').read()
src = src.replace(f"SCENARIO_NAME = '{base}'", f"SCENARIO_NAME = '{name}'")
open(f'ecav/scenario_testing/{name}.py','w').write(src)
PYEOF
  fi
  echo "$name"
}

if [ "${E6_GO:-0}" != "1" ]; then
  echo "DRY RUN (set E6_GO=1 to execute). Plan:"
  for arm in $ARMS; do for w in $WEATHERS; do
    tag=${w%%:*}
    # e6b has no edge; weather still applies. clat/e6a weather runs are
    # controls only at w0 (GT content is weather-independent); skip w1/w2
    # for the GT arms to save GPU time.
    case "$arm" in e6a_oracle|e6clat_gt_latency) [ "$tag" != "w0" ] && continue;; esac
    echo "  openscenario_1_${arm} ${tag} x ${REPS} reps"
  done; done
  exit 0
fi

pgrep -x CarlaUE4-Linux- > /dev/null || pgrep -f CarlaUE4 > /dev/null || {
  echo "CarlaUE4 not running; start it (start_actors.sh) first." >&2; exit 1; }

echo "=== e6 sweep start $(date -u) reps=$REPS ===" >> $LOG
FAIL=0
for arm in $ARMS; do
  for w in $WEATHERS; do
    IFS=: read tag cl pr fog wet <<< "$w"
    case "$arm" in e6a_oracle|e6clat_gt_latency) [ "$tag" != "w0" ] && continue;; esac
    name=$(gen_variant "openscenario_1_${arm}" "$tag" "$cl" "$pr" "$fog" "$wet")
    for i in $(seq 1 $REPS); do
      cell="$OUT/${arm}_${tag}/rep${i}"
      [ -f "$cell/DONE" ] && continue
      mkdir -p "$cell"
      until [ "$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits | head -1)" -ge 9000 ]; do sleep 60; done
      echo "=== $(date -u) $arm $tag rep$i ($name) ===" >> $LOG
      python -u ecav.py -t "$name" --apply_ml > "$cell/run.log" 2>&1
      rc=$?
      # archive whatever evaluation output this scenario produced
      for d in evaluation_outputs/${name}*; do
        [ -e "$d" ] && mv "$d" "$cell/" ;
      done
      if [ $rc -ne 0 ]; then
        echo "=== FAILED $arm $tag rep$i rc=$rc ===" >> $LOG
        FAIL=$((FAIL+1))
      else
        touch "$cell/DONE"
        echo "=== ok $arm $tag rep$i ===" >> $LOG
      fi
    done
  done
done
echo "=== e6 sweep complete $(date -u) failures=$FAIL ===" >> $LOG
