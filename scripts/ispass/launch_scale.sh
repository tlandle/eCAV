#!/usr/bin/env bash
# Launch one distributed scale_town06 run: CARLA, orchestrator, edge, N vehicle clients, R RSU clients,
# each on the host given, over ssh from this machine. Waits on log lines, not fixed sleeps, and
# collects every log into one results directory here.
#
#   scripts/ispass/launch_scale.sh -n 32 -r 2 -p off -o azureuser@20.81.176.215
#   scripts/ispass/launch_scale.sh -n 256 -p off -o user@orch -c user@carla -e user@edge \
#        -H "user@c1:0-63,user@c2:64-127,user@c3:128-191,user@c4:192-255"
#
#   -n N            ego vehicles (one client process each)
#   -r R            RSU clients (default 2)
#   -p on|off       perception (default off: exact-state edge manager, no sensors, no --apply_ml)
#   -s STEPS        ticks to run (default 200)
#   -o HOST         orchestrator host (ssh target); default host for everything else
#   -c HOST         CARLA host (default: orchestrator host). CARLA is started there if port 2000 is closed
#   -e HOST         edge host (default: orchestrator host)
#   -R HOST         RSU client host (default: edge host)
#   -H LIST         vehicle clients: "host:lo-hi,host:lo-hi" (default: orchestrator host, 0..N-1)
#   -t TAG          results directory tag (default: n<N>_r<R>_p<on|off>_<time>)
#   -g ARGS         extra generator arguments, quoted (e.g. -g "--layout spread --rsus 12")
#   -m dist|seq     dist (default): one process per actor. seq: the sequential single process
#                   (ecav.py without -d) on the orchestrator host, same scenario files
# Environment: SCALE_WT (worktree on every host, default $HOME/scale_wt), SCALE_ACT (shell line that activates the
# Python env on every host), SCALE_CARLA_ROOT (default $HOME/carla-0.9.15), BATCH (clients started per wave, 16).
#
# Every host needs: the worktree at $WT with generated proto stubs and, on the orchestrator, the
# ecloud_server binary; the opencda310 env; CARLA's PythonAPI under $CARLA_ROOT. Hosts must reach
# each other on 2000-2002 (CARLA), 50051 (orchestration server), 50061 (orchestrator push),
# 50054 (edge), 50101-50101+N (vehicle push) and 51101-51101+R (RSU push).
# cloud_config.yaml is rewritten on every host from the private IPs (hostname -I) when more than
# one host is used: carla_server_public_ip, ecloud_server_public_ip, vehicle_client_public_ip
# (the host's own address; on the orchestrator the list of client hosts), edge_server_public_ip.
set -uo pipefail
N=""; R=2; P=off; STEPS=200; ORCH=""; CARLA=""; EDGE=""; RSUH=""; VLIST=""; TAG=""; GEN=""; MODE=dist
while getopts "n:r:p:s:o:c:e:R:H:t:g:m:" o; do case $o in
  m) MODE=$OPTARG;;
  n) N=$OPTARG;; r) R=$OPTARG;; p) P=$OPTARG;; s) STEPS=$OPTARG;; o) ORCH=$OPTARG;; c) CARLA=$OPTARG;;
  e) EDGE=$OPTARG;; R) RSUH=$OPTARG;; H) VLIST=$OPTARG;; t) TAG=$OPTARG;; g) GEN=$OPTARG;; *) exit 2;; esac; done
[[ -n "$N" && -n "$ORCH" ]] || { sed -n 2,20p "$0"; exit 2; }
CARLA=${CARLA:-$ORCH}; EDGE=${EDGE:-$ORCH}; RSUH=${RSUH:-$EDGE}; VLIST=${VLIST:-$ORCH:0-$((N-1))}
# defaults are expanded on the remote host, not here
DEF_WT='$HOME/scale_wt'; DEF_CARLA='$HOME/carla-0.9.15'
DEF_ACT='source $HOME/miniconda3/etc/profile.d/conda.sh 2>/dev/null || source $HOME/anaconda3/etc/profile.d/conda.sh; conda activate opencda310'
WT=${SCALE_WT:-$DEF_WT}; CARLA_ROOT=${SCALE_CARLA_ROOT:-$DEF_CARLA}; ACT=${SCALE_ACT:-$DEF_ACT}; BATCH=${BATCH:-16}
TAG=${TAG:-n${N}_r${R}_p${P}_$(date +%m%d_%H%M%S)}
HERE=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$HERE/experiment_results/ispass_scale/$TAG; mkdir -p "$OUT"
L=logs/scale/$TAG                       # log directory inside the worktree on every host
ML=""; [[ $P == on ]] && ML="--apply_ml"
# one multiplexed connection per host: sshd rate-limits new connections (MaxStartups), and a run makes hundreds of calls
SSHO="-o BatchMode=yes -o ConnectTimeout=15 -o ControlMaster=auto -o ControlPath=/tmp/scale_ssh_%C -o ControlPersist=300"
SSH="ssh $SSHO"
log() { echo "$(date '+%T') $*" | tee -a "$OUT/launch.log"; }
# run a command line in the worktree with the env active
on() { local h=$1; shift; $SSH "$h" "ulimit -n 65536 2>/dev/null; cd $WT && $ACT && export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:\${PYTHONPATH:-} && $*"; }
# start a detached process: bg <host> <logname> <command...>
bg() { local h=$1 n=$2; shift 2; on "$h" "mkdir -p $L && (setsid nohup $* > $L/$n.log 2>&1 < /dev/null &)"; }
# wait_log <host> <logname> <regex> <timeout_s>
wait_log() { $SSH "$1" "cd $WT && timeout $4 bash -c 'until grep -Eq \"$3\" $L/$2.log 2>/dev/null; do sleep 2; done'" \
             || { log "TIMEOUT waiting for /$3/ in $2.log on $1"; return 1; }; }
HOSTS=$(printf '%s\n' "$ORCH" "$CARLA" "$EDGE" "$RSUH" $(tr ',' '\n' <<< "$VLIST" | sed 's/:.*//') | sort -u)
killall_hosts() { for h in $HOSTS; do $SSH "$h" "P=\$( { pgrep -f '^python -u ecav'; pgrep -f '^\./ecav/ecloud_server/ecloud_server'; } | sort -u); [ -n \"\$P\" ] && kill -9 \$P; true"; done; }
collect() { for h in $HOSTS; do mkdir -p "$OUT/${h##*@}"; scp -q $SSHO "$h:${WT/\$HOME\//}/$L/*.log" "$OUT/${h##*@}/" 2>/dev/null; done; }
fail() { log "FAILED: $*"; collect; killall_hosts; exit 1; }

log "scale_town06 N=$N R=$R perception=$P steps=$STEPS orch=$ORCH carla=$CARLA edge=$EDGE rsu=$RSUH vehicles=$VLIST"
killall_hosts

# addresses: one host means localhost everywhere; several hosts means private IPs and a cloud_config rewrite
ip_of() { $SSH "$1" "hostname -I | cut -d' ' -f1"; }
if [[ $(wc -l <<< "$HOSTS") -eq 1 ]]; then CARLA_IP=localhost; MULTI=0; else
  MULTI=1; CARLA_IP=$(ip_of "$CARLA"); ORCH_IP=$(ip_of "$ORCH"); EDGE_IP=$(ip_of "$EDGE")
  CLIENT_IPS=$(for h in $(tr ',' '\n' <<< "$VLIST" | sed 's/:.*//' | sort -u) "$RSUH"; do ip_of "$h"; done | sort -u | paste -sd,)
  for h in $HOSTS; do
    own=$(ip_of "$h"); vc=$own; [[ $h == "$ORCH" ]] && vc=$CLIENT_IPS
    $SSH "$h" "cd $WT && [ -f cloud_config.yaml.single_host ] || cp cloud_config.yaml cloud_config.yaml.single_host; \
      sed -i -E -e 's|^carla_server_public_ip:.*|carla_server_public_ip: \"$CARLA_IP\"|' \
                -e 's|^ecloud_server_public_ip:.*|ecloud_server_public_ip: \"$ORCH_IP\"|' \
                -e 's|^vehicle_client_public_ip:.*|vehicle_client_public_ip: \"$vc\"|' \
                -e '/^edge_server_public_ip:/d' cloud_config.yaml && echo 'edge_server_public_ip: \"$own\"' >> cloud_config.yaml" || fail "cloud_config on $h"
  done
  log "addresses: carla $CARLA_IP orchestrator $ORCH_IP edge $EDGE_IP clients $CLIENT_IPS"
fi

# file logging: every process writes a debug-level log file by default (cloud_config file_log_level), hundreds of MB
# per client per run and about 12 % of a client's step. Scale runs log warnings and above to file.
for h in $HOSTS; do $SSH "$h" "cd $WT && sed -i '/^file_log_level:/d' cloud_config.yaml && echo 'file_log_level: \"${SCALE_FILE_LOG:-warning}\"' >> cloud_config.yaml" || fail "file_log_level on $h"; done

# CARLA
if ! $SSH "$CARLA" "ss -ltn | grep -q ':2000 '"; then
  log "starting CARLA on $CARLA"
  $SSH "$CARLA" "(setsid nohup $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -carla-port=2000 > /tmp/carla_scale.log 2>&1 < /dev/null &)"
  $SSH "$CARLA" "timeout 120 bash -c 'until ss -ltn | grep -q \":2000 \"; do sleep 2; done'" || fail "CARLA did not open port 2000"
  sleep 8
fi

# scenario files: generated on the orchestrator, copied to the other hosts (clients read the XML, the YAML comes from the server)
on "$ORCH" "python scripts/ispass/gen_scale_town06.py -n $N --rsus $R --perception $P --carla-host $CARLA_IP $GEN" | tee -a "$OUT/launch.log" || fail "generator"
# the spread layout chooses its own RSU count
RG=$(grep -o '^RSUS=[0-9]*' "$OUT/launch.log" | tail -1 | cut -d= -f2); [[ -n "$RG" ]] && R=$RG
for h in $HOSTS; do [[ $h == "$ORCH" ]] && continue
  for f in ecav/scenario_testing/scenarios/scenario_scale_town06.xml ecav/scenario_testing/config_yaml/scale_town06.yaml; do
    scp -q -3 $SSHO "$ORCH:${WT/\$HOME\//}/$f" "$h:${WT/\$HOME\//}/$f" || fail "copy $f to $h"; done; done

T0=$(date +%s)
if [[ $MODE == seq ]]; then
  bg "$ORCH" orch "env SCENARIO_MAX_STEP=$STEPS python -u ecav.py -t scale_town06 -v 0.9.15 $ML"
  wait_log "$ORCH" orch "SCALE_TOWN06. tick 0 |Caught exception|Traceback" $((900 + 6 * N)) || fail "no first tick (sequential)"
  $SSH "$ORCH" "cd $WT && grep -q 'Caught exception' $L/orch.log" && fail "sequential run raised before the first tick"
  log "first tick after $(( $(date +%s) - T0 )) s"
  $SSH "$ORCH" "cd $WT && head -1 /proc/stat > $L/cpu_start.log; cat /proc/loadavg > $L/load_start.log"
  wait_log "$ORCH" orch "SCALE_TOWN06. SUMMARY" $((600 + STEPS * 30)) || fail "run did not finish"
  $SSH "$ORCH" "cd $WT && head -1 /proc/stat > $L/cpu_end.log; cat /proc/loadavg > $L/load_end.log"
  sleep 10; collect; killall_hosts
  grep -h "SUMMARY" "$OUT"/*/orch.log | tee -a "$OUT/launch.log"; log "done: $OUT"; exit 0
fi
bg "$ORCH" orch "env SCENARIO_MAX_STEP=$STEPS python -u ecav.py -t scale_town06 -v 0.9.15 -d $ML"
wait_log "$ORCH" orch "start [0-9]+ vehicle containers" 300 || fail "orchestrator did not come up"
log "orchestrator up"
EARGS="-e 0 -P 50054"; [[ $MULTI == 1 ]] && EARGS="$EARGS -O $ORCH_IP -o 50051"
bg "$EDGE" edge "python -u ecav/ecav2/edge_process.py $EARGS"
wait_log "$EDGE" edge "registered successfully" 180 || fail "edge did not register"
log "edge registered"

# vehicle client 0 spawns every ego; the others attach to theirs
host_of() { local k=$1 e h r; for e in $(tr ',' ' ' <<< "$VLIST"); do h=${e%%:*}; r=${e##*:}; (( k >= ${r%-*} && k <= ${r#*-} )) && { echo "$h"; return; }; done; }
# vehicles lo..hi that live on one host start from one remote loop
veh_range() { local h=$1 lo=$2 hi=$3
  on "$h" "mkdir -p $L && for i in \$(seq $lo $hi); do (setsid nohup python -u ecav.py $ML -v 0.9.15 -d -i \$i -T \$((8000 + i)) > $L/veh_\$i.log 2>&1 < /dev/null &); done"; }
veh_range "$(host_of 0)" 0 0
wait_log "$(host_of 0)" veh_0 "spawning Ecav2ActorClient" $((120 + N)) || fail "client 0 did not spawn the egos"
log "client 0 spawned the egos"
k=1; while (( k < N )); do
  h=$(host_of $k); hi=$(( k + BATCH - 1 < N - 1 ? k + BATCH - 1 : N - 1 ))
  while [[ $(host_of $hi) != "$h" ]]; do hi=$(( hi - 1 )); done
  veh_range "$h" $k $hi
  wait_log "$h" "veh_$hi" "spawning Ecav2ActorClient|Traceback" 600 || fail "client $hi did not start"
  log "clients $k..$hi started on $h"; k=$(( hi + 1 ))
done
for j in $(seq 0 $((R - 1))); do bg "$RSUH" "rsu_$j" "python -u ecav/ecav2/ecloud_actor_client.py $ML -v 0.9.15 -i $j"; done
log "all clients started after $(( $(date +%s) - T0 )) s; waiting for the run"
wait_log "$ORCH" orch "SCALE_TOWN06. tick 0 |Caught exception" $((600 + 4 * N)) || fail "no first tick"
$SSH "$ORCH" "cd $WT && grep -q 'Caught exception' $L/orch.log" && fail "orchestrator raised before the first tick: $($SSH "$ORCH" "cd $WT && grep -m1 'Caught exception' $L/orch.log")"
log "first tick after $(( $(date +%s) - T0 )) s"
# CPU counters on every host at the first tick and at the end: scale_breakdown.py reports the busy fraction
for h in $HOSTS; do $SSH "$h" "cd $WT && head -1 /proc/stat > $L/cpu_start.log; cat /proc/loadavg > $L/load_start.log"; done
wait_log "$ORCH" orch "SCALE_TOWN06. SUMMARY" $((600 + STEPS * 20)) || fail "run did not finish"
for h in $HOSTS; do $SSH "$h" "cd $WT && head -1 /proc/stat > $L/cpu_end.log; cat /proc/loadavg > $L/load_end.log"; done
sleep 15
collect; killall_hosts
grep -h "SUMMARY" "$OUT"/*/orch.log | tee -a "$OUT/launch.log"
log "done: $OUT"
