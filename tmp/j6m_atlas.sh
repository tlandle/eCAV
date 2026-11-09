#!/usr/bin/env bash
# j6m atlas (2026-10-30 15:0x): motion uncertainty on the recalibrated 70 km/h left turn. The crossing car launches
# at 13.4 m/s and accelerates when it is D m before the conflict, so a forecast built from an observation taken
# before the change extrapolates to the wrong arrival; the lead is recalibrated per design so the changed car
# meets the ego (j6_calibrate.py --extra-env --want-after-car-in 0.15). Each design is gated: calibration converged;
# no cooperation collides 2/2 at zero age; reference and contract complete 2/2 at zero age; the reference fails at
# 1,000 ms in at least one of two seeds (the change reaches the decision, otherwise the design is uninformative).
# The first design that passes runs the block: reference and contract, ages 0 to 1,000, ten seeds. Then j6f.
# Waits for the j6 speed chain (pid 4146153). Log: logs/j6m_atlas.log.
set -uo pipefail
cd "$HOME/TrafficSimulator_eCloud/ecloudsim_distributed_sandbox"
RUNNER=4146153
while kill -0 "$RUNNER" 2>/dev/null; do sleep 30; done
sleep 20
while pgrep -f '^[^ ]*/CarlaUE4-Linux-Shipping' >/dev/null; do sleep 10; done
if pgrep -f "^/home/atlas/anaconda3/envs/opencda310/bin/python test_runner.py" >/dev/null; then echo "a runner is still active; aborting"; exit 1; fi
if ls ecav/scenario_testing/config_yaml/*.yaml.bak >/dev/null 2>&1; then echo "a patched YAML backup remains; aborting"; ls ecav/scenario_testing/config_yaml/*.yaml.bak; exit 1; fi
PY="$HOME/anaconda3/envs/opencda310/bin/python"
export PYTHONPATH="$HOME/carla-0.9.15/PythonAPI/carla:${PYTHONPATH:-}"
PORT=2000
HIDE=5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,7518384223988529276
T=openscenario_3_edge_late_fusion
COMMON="--ego-counts 1 --speeds 70 --latency-distribution hybrid --per-tick-age-log --self-id-radius 1 --carla-port $PORT --edge-kv prediction_steps=60 --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0"
echo "$(date '+%F %T') j6m atlas starts at $(git rev-parse --short HEAD)"
DESIGNS=("d40v20 CROSS_SPEED_CHANGE=40:20.0" "d30v22 CROSS_SPEED_CHANGE=30:22.0" "d50v20 CROSS_SPEED_CHANGE=50:20.0")
RAN=0
for design in "${DESIGNS[@]}"; do set -- $design; NAME=$1; CHG=$2
  echo "$(date '+%F %T') [$NAME] calibrating the lead with $CHG"
  PYTHONPATH=.:ecav/protos:$PYTHONPATH "$PY" scripts/sensys_analysis/j6_calibrate.py --out tmp/j6m_$NAME.json --speeds 70 --tests lt --extra-env $CHG --want-after-car-in 0.15 --stamp-prefix sensys_nov27_j6mc_$NAME --iters 4 --start-maps tmp/j6_maps.json --log logs/j6m_calibrate.log --port $PORT
  OK=$("$PY" -c "import json;d=json.load(open('tmp/j6m_$NAME.json'));print(int(bool(d.get('ok'))))")
  LEAD=$("$PY" -c "import json;d=json.load(open('tmp/j6m_$NAME.json'));print(d['lt'].get('70', d['lt'].get(70)))")
  echo "$(date '+%F %T') [$NAME] calibration ok=$OK lead=$LEAD"
  if [ "$OK" != "1" ]; then echo "[$NAME] calibration did not converge; next design"; continue; fi
  ENV="CROSS_TRIGGER_TTA=70:$LEAD HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900 $CHG"
  SV=sensys_nov27_j6mv_$NAME
  echo "$(date '+%F %T') [$NAME] validation"
  "$PY" test_runner.py -t $T --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0 --repetitions 2 $COMMON --stamp "$SV" --scenario-env $ENV
  "$PY" test_runner.py -t $T --manager-types oracle --anchoring on --latencies 0 1000 --repetitions 2 $COMMON --stamp "$SV" --scenario-env $ENV
  "$PY" test_runner.py -t $T --manager-types late_fusion --anchoring on --latencies 0 --repetitions 2 $COMMON --stamp "$SV" --scenario-env $ENV
  PYTHONPATH=.:ecav/protos "$PY" scripts/sensys_analysis/envelope_measured.py $SV --out safety_envelope_paper/analysis/envelope_$SV 2>&1 | tail -2
  GATE=$("$PY" - "$SV" <<'PY'
import csv, sys
rows=list(csv.DictReader(open(f"safety_envelope_paper/analysis/envelope_{sys.argv[1]}/runs.csv")))
def ok(r): return r['collided']=='0' and r['self_ghost']=='0' and r['false_brake']=='0' and r['stalled']=='0'
nc=[r for r in rows if 'loss_100' in (r.get('cell','')+r.get('path',''))]
ref0=[r for r in rows if r['mgr']=='oracle' and int(float(r['lat']))==0]
ref1=[r for r in rows if r['mgr']=='oracle' and int(float(r['lat']))==1000]
con0=[r for r in rows if r['mgr']=='late_fusion' and int(float(r['lat']))==0 and r not in nc]
for name,g in (('no-coop',nc),('ref 0',ref0),('ref 1000',ref1),('contract 0',con0)):
    print(f"  {name}: {len(g)} runs, collided {sum(r['collided']!='0' for r in g)}, ok {sum(ok(r) for r in g)}", file=sys.stderr)
good = len(nc)==2 and all(r['collided']!='0' for r in nc) and len(ref0)==2 and all(ok(r) for r in ref0) \
       and len(con0)==2 and all(ok(r) for r in con0) and len(ref1)==2 and any(r['collided']!='0' for r in ref1)
print(int(good))
PY
)
  echo "$(date '+%F %T') [$NAME] gate=$GATE"
  if [ "$GATE" != "1" ]; then echo "[$NAME] gate failed; next design"; continue; fi
  SB=sensys_nov27_j6m_$NAME
  echo "$(date '+%F %T') [$NAME] block starts (stamp $SB, lead $LEAD, $CHG)"
  "$PY" test_runner.py -t $T --manager-types oracle --anchoring on --latencies 0 100 200 300 500 700 1000 --repetitions 10 $COMMON --stamp "$SB" --scenario-env $ENV
  "$PY" test_runner.py -t $T --manager-types late_fusion --anchoring on --latencies 0 100 200 300 500 700 1000 --repetitions 10 $COMMON --stamp "$SB" --scenario-env $ENV
  echo "$(date '+%F %T') [$NAME] block done"
  RAN=1; break
done
[ "$RAN" = "1" ] || echo "$(date '+%F %T') j6m: no design passed its gate; no block run"
# Slow-downs (Tyler, Oct 30 17:5x): the car slows when it is 40 m before the conflict (to 6 m/s, then to a stop), so a
# stale forecast keeps an arrival that has moved away and the ego brakes for a conflict that no longer exists. The
# nominal lead holds (no recalibration: the slowed car must not conflict); gate: no collision without cooperation 2/2
# and none for the reference at zero age 2/2; block: reference and contract at 0, 500, 1,000 ms, five seeds. The
# measurement is the brake and stall rate per age (per-tick logs carry the brake and release ticks).
LEAD0=$("$PY" -c "import json;m=json.load(open('tmp/j6_maps.json'))['lt'];print([v for k,v in m.items() if int(float(k))==70][0])")
SLOW=("d40v6 CROSS_SPEED_CHANGE=40:6.0" "d40v0 CROSS_SPEED_CHANGE=40:0.0")
for design in "${SLOW[@]}"; do set -- $design; NAME=$1; CHG=$2
  ENV="CROSS_TRIGGER_TTA=70:$LEAD0 HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900 $CHG"
  SV=sensys_nov27_j6ov_$NAME
  echo "$(date '+%F %T') [$NAME] slow-down validation (lead $LEAD0, $CHG)"
  "$PY" test_runner.py -t $T --manager-types late_fusion --anchoring on --packet-loss 100 --latencies 0 --repetitions 2 $COMMON --stamp "$SV" --scenario-env $ENV
  "$PY" test_runner.py -t $T --manager-types oracle --anchoring on --latencies 0 --repetitions 2 $COMMON --stamp "$SV" --scenario-env $ENV
  PYTHONPATH=.:ecav/protos "$PY" scripts/sensys_analysis/envelope_measured.py $SV --out safety_envelope_paper/analysis/envelope_$SV 2>&1 | tail -2
  GATE=$("$PY" - "$SV" <<'PY2'
import csv, sys
rows=list(csv.DictReader(open(f"safety_envelope_paper/analysis/envelope_{sys.argv[1]}/runs.csv")))
nc=[r for r in rows if 'loss_100' in (r.get('cell','')+r.get('path',''))]
ref0=[r for r in rows if r['mgr']=='oracle' and int(float(r['lat']))==0]
for name,g in (('no-coop',nc),('ref 0',ref0)):
    print(f"  {name}: {len(g)} runs, collided {sum(r['collided']!='0' for r in g)}, stalled {sum(r['stalled']!='0' for r in g)}, false brakes {sum(r['false_brake']!='0' for r in g)}", file=sys.stderr)
good = len(nc)==2 and all(r['collided']=='0' for r in nc) and len(ref0)==2 and all(r['collided']=='0' for r in ref0)
print(int(good))
PY2
)
  echo "$(date '+%F %T') [$NAME] slow-down gate=$GATE"
  if [ "$GATE" != "1" ]; then echo "[$NAME] slow-down gate failed; next design"; continue; fi
  SB=sensys_nov27_j6o_$NAME
  echo "$(date '+%F %T') [$NAME] slow-down block starts (stamp $SB)"
  "$PY" test_runner.py -t $T --manager-types oracle --anchoring on --latencies 0 500 1000 --repetitions 5 $COMMON --stamp "$SB" --scenario-env $ENV
  "$PY" test_runner.py -t $T --manager-types late_fusion --anchoring on --latencies 0 500 1000 --repetitions 5 $COMMON --stamp "$SB" --scenario-env $ENV
  echo "$(date '+%F %T') [$NAME] slow-down block done"
done
echo "$(date '+%F %T') j6m atlas done; starting j6f"
bash tmp/j6f_atlas.sh >> logs/j6f_atlas.log 2>&1
