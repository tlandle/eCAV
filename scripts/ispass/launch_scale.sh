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
#   -p on|off|shared  perception (default off: exact-state edge manager, no sensors, no --apply_ml).
#                   on: every client loads its own detector. shared: clients call the YOLO inference
#                   service at ml_manager.yolo_endpoint (--apply_ml -l); start the service first
#   -s STEPS        ticks to run (default 200)
#   -o HOST         orchestrator host (ssh target); default host for everything else
#   -c HOST         CARLA host (default: orchestrator host). CARLA is started there if port 2000 is closed
#   -e HOST         edge host (default: orchestrator host)
#   -R HOST         RSU client host (default: edge host)
#   -H LIST         vehicle clients: "host:lo-hi,host:lo-hi" (default: orchestrator host, 0..N-1)
#   -y HOST         host of the shared YOLO inference service (default: orchestrator host). With more than
#                   one host and -p shared, the scenario YAML's yolo_endpoint is set to that host's private address
#   -Y LIST         perception service replicas, "ip:port,ip:port,...": vehicle client k and RSU client j call
#                   endpoint k mod R and j mod R (ECAV_YOLO_ENDPOINT in each client's environment). Implies the
#                   service is already running at those addresses (scripts/ispass/perception_replicas.sh);
#                   the scenario YAML's yolo_endpoint is left alone. Without -Y every client uses the YAML's endpoint
#   -G LIST         CARLA secondary servers (multi-GPU sensor rendering), "host:n,host:n": n secondaries on that host,
#                   one per GPU index 0..n-1. The CARLA host then runs the primary without rendering (-nullrhi) and
#                   the secondaries render the sensors. Off by default (one server renders everything).
#                   Every secondary host needs CARLA under $SCALE_CARLA_ROOT and reaches the primary on port 2002
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
N=""; R=2; P=off; STEPS=200; ORCH=""; CARLA=""; EDGE=""; RSUH=""; VLIST=""; TAG=""; GEN=""; MODE=dist; PERC=""; EPS=""; SECS=""
while getopts "n:r:p:s:o:c:e:R:H:t:g:m:y:Y:G:" o; do case $o in
  y) PERC=$OPTARG;; Y) EPS=$OPTARG;; G) SECS=$OPTARG;;
  m) MODE=$OPTARG;;
  n) N=$OPTARG;; r) R=$OPTARG;; p) P=$OPTARG;; s) STEPS=$OPTARG;; o) ORCH=$OPTARG;; c) CARLA=$OPTARG;;
  e) EDGE=$OPTARG;; R) RSUH=$OPTARG;; H) VLIST=$OPTARG;; t) TAG=$OPTARG;; g) GEN=$OPTARG;; *) exit 2;; esac; done
[[ -n "$N" && -n "$ORCH" ]] || { sed -n 2,20p "$0"; exit 2; }
CARLA=${CARLA:-$ORCH}; EDGE=${EDGE:-$ORCH}; PERC=${PERC:-$ORCH}; RSUH=${RSUH:-$EDGE}; VLIST=${VLIST:-$ORCH:0-$((N-1))}
# defaults are expanded on the remote host, not here
DEF_WT='$HOME/scale_wt'; DEF_CARLA='$HOME/carla-0.9.15'
# a host with a conda install activates opencda310; a client VM prepared by setup_scale_vm.sh has only the env
# directory (no conda base), so its bin directory goes on PATH
DEF_ACT='if [ -f $HOME/miniconda3/etc/profile.d/conda.sh ]; then source $HOME/miniconda3/etc/profile.d/conda.sh; conda activate opencda310; elif [ -f $HOME/anaconda3/etc/profile.d/conda.sh ]; then source $HOME/anaconda3/etc/profile.d/conda.sh; conda activate opencda310; else export PATH=$HOME/miniconda3/envs/opencda310/bin:$PATH; fi'
WT=${SCALE_WT:-$DEF_WT}; CARLA_ROOT=${SCALE_CARLA_ROOT:-$DEF_CARLA}; ACT=${SCALE_ACT:-$DEF_ACT}; BATCH=${BATCH:-16}
# numeric libraries start one thread per core in every process; with one process per actor that
# oversubscribes the host, so each process gets SCALE_THREADS threads (default 1, 0 leaves the libraries alone)
TH=${SCALE_THREADS:-1}; THREADS=""
[[ $TH != 0 ]] && THREADS="OMP_NUM_THREADS=$TH OPENBLAS_NUM_THREADS=$TH MKL_NUM_THREADS=$TH NUMEXPR_NUM_THREADS=$TH"
TAG=${TAG:-n${N}_r${R}_p${P}_$(date +%m%d_%H%M%S)}
HERE=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$HERE/experiment_results/ispass_scale/$TAG; mkdir -p "$OUT"
L=logs/scale/$TAG                       # log directory inside the worktree on every host
ML=""; [[ $P == on ]] && ML="--apply_ml"; [[ $P == shared ]] && ML="--apply_ml -l"
GP=$P; [[ $P == shared ]] && GP=on
# one multiplexed connection per host: sshd rate-limits new connections (MaxStartups), and a run makes hundreds of calls
SSHO="-o BatchMode=yes -o ConnectTimeout=15 -o ControlMaster=auto -o ControlPath=/tmp/scale_ssh_%C -o ControlPersist=300"
SSH="ssh $SSHO"
log() { echo "$(date '+%T') $*" | tee -a "$OUT/launch.log"; }
# run a command line in the worktree with the env active
# SCALE_CLIENT_ENV: extra NAME=value words exported to every process this launcher starts (e.g. ECAV_SENSOR_TIMING=1)
on() { local h=$1; shift; $SSH "$h" "ulimit -n 65536 2>/dev/null; cd $WT && $ACT && export PYTHONPATH=$CARLA_ROOT/PythonAPI/carla:\${PYTHONPATH:-} $THREADS ${SCALE_CLIENT_ENV:-} && $*"; }
# start a detached process: bg <host> <logname> <command...>
bg() { local h=$1 n=$2; shift 2; on "$h" "mkdir -p $L && (setsid nohup $* > $L/$n.log 2>&1 < /dev/null &)"; }
# wait_log <host> <logname> <regex> <timeout_s>
wait_log() { $SSH "$1" "cd $WT && timeout $4 bash -c 'until grep -Eq \"$3\" $L/$2.log 2>/dev/null; do sleep 2; done'" \
             || { log "TIMEOUT waiting for /$3/ in $2.log on $1"; return 1; }; }
HOSTS=$(printf '%s\n' "$ORCH" "$CARLA" "$EDGE" "$RSUH" $(tr ',' '\n' <<< "$VLIST" | sed 's/:.*//') | sort -u)
killall_hosts() { for h in $HOSTS; do $SSH "$h" "P=\$( { pgrep -f '^python -u ecav'; pgrep -f '^\./ecav/ecloud_server/ecloud_server'; } | sort -u); [ -n \"\$P\" ] && kill -9 \$P; true"; done; }
collect() { for h in $HOSTS; do mkdir -p "$OUT/${h##*@}"; scp -q $SSHO "$h:${WT/\$HOME\//}/$L/*.log" "$OUT/${h##*@}/" 2>/dev/null; done; }
# a failed run can leave the simulator unusable (see the send-buffer note below): the next run restarts it
fail() { log "FAILED: $*"; collect; killall_hosts; $SSH "$CARLA" "rm -f /tmp/scale_carla_layout"; exit 1; }

log "scale_town06 N=$N R=$R perception=$P steps=$STEPS orch=$ORCH carla=$CARLA edge=$EDGE rsu=$RSUH vehicles=$VLIST"
if [[ -n "$EPS" ]]; then
  [[ $P == shared && $MODE == dist ]] || { log "FAILED: -Y needs -p shared and the distributed mode"; exit 2; }
  NEP=$(tr ',' '\n' <<< "$EPS" | grep -c .); log "perception replicas ($NEP): $EPS"
fi
killall_hosts

# host firewall rule used below: simulator ports (2000-2002, secondaries 3000-3999), client RPC and push ports
NET=${SCALE_PRIVATE_NET:-10.0.0.0/8}; RULE="INPUT ! -i lo -p tcp -m multiport --dports 2000:2002,3000:3999,8000:8999,50051:51300 ! -s $NET -j DROP"
# addresses: one host means localhost everywhere; several hosts means private IPs and a cloud_config rewrite
ip_of() { $SSH "$1" "hostname -I | cut -d' ' -f1"; }
if [[ $(wc -l <<< "$HOSTS") -eq 1 ]]; then CARLA_IP=localhost; MULTI=0
  # a multi-host run rewrote cloud_config.yaml on this host: put the single-host file back
  $SSH "$ORCH" "cd $WT && if [ -f cloud_config.yaml.single_host ]; then cp cloud_config.yaml.single_host cloud_config.yaml; fi" || fail "cloud_config restore on $ORCH"
else
  MULTI=1; CARLA_IP=$(ip_of "$CARLA"); ORCH_IP=$(ip_of "$ORCH"); EDGE_IP=$(ip_of "$EDGE")
  CLIENT_IPS=$(for h in $(tr ',' '\n' <<< "$VLIST" | sed 's/:.*//' | sort -u) "$RSUH"; do ip_of "$h"; done | sort -u | paste -sd,)
  for h in $HOSTS; do
    own=$(ip_of "$h"); vc=$own; [[ $h == "$ORCH" ]] && vc=$CLIENT_IPS
    $SSH "$h" "cd $WT && [ -f cloud_config.yaml.single_host ] || cp cloud_config.yaml cloud_config.yaml.single_host; \
      sed -i -E -e 's|^carla_server_public_ip:.*|carla_server_public_ip: \"$CARLA_IP\"|' \
                -e 's|^ecloud_server_public_ip:.*|ecloud_server_public_ip: \"$ORCH_IP\"|' \
                -e 's|^vehicle_client_public_ip:.*|vehicle_client_public_ip: \"$vc\"|' \
                -e '/^edge_server_public_ip:/d' -e '/^actor_client_ip:/d' cloud_config.yaml && echo 'edge_server_public_ip: \"$own\"' >> cloud_config.yaml && echo 'actor_client_ip: \"$own\"' >> cloud_config.yaml" || fail "cloud_config on $h"
  done
  log "addresses: carla $CARLA_IP orchestrator $ORCH_IP edge $EDGE_IP clients $CLIENT_IPS"
  # Every vehicle client runs a CARLA traffic manager RPC server on port 8000+k, and the push servers listen on
  # 50101+k. A VM whose security group admits any port from the internet receives scanner connections on these
  # ports (8000, 8080, ...). The traffic manager's RPC server does not survive a non-RPC message: the client
  # process terminates with a msgpack parse or type error. Each host therefore drops connections to these ports
  # that do not come from the private network (SCALE_PRIVATE_NET, default 10.0.0.0/8) or from loopback.
  # The rule is not persistent (gone after a restart of the VM).
  : # RULE is defined before this block
  for h in $HOSTS; do $SSH "$h" "sudo iptables -C $RULE 2>/dev/null || sudo iptables -I $RULE" || fail "port guard on $h"; done
  # CARLA 0.9.15 in synchronous mode: a stream write that has not completed when the next frame's message is queued
  # for the same client makes a streaming worker spin forever (ServerSession::Write waits inside the strand that
  # must run the completion). On loopback a write completes at once. To a remote client it completes only when the
  # kernel send buffer takes the whole message, and the default initial buffer is 16 KB, less than the world state
  # of a few hundred actors. Two frames in quick succession (clients tick once each at start) then stop every stream.
  # A 4 MB initial send buffer on the CARLA host lets each write complete at once, as on loopback.
  $SSH "$CARLA" "sudo sysctl -q -w net.ipv4.tcp_wmem='4096 4194304 16777216'" || fail "tcp_wmem on $CARLA"
fi

# file logging: every process writes a debug-level log file by default (cloud_config file_log_level), hundreds of MB
# per client per run and about 12 % of a client's step. Scale runs log warnings and above to file.
for h in $HOSTS; do $SSH "$h" "cd $WT && sed -i '/^file_log_level:/d' cloud_config.yaml && echo 'file_log_level: \"${SCALE_FILE_LOG:-warning}\"' >> cloud_config.yaml" || fail "file_log_level on $h"; done

# CARLA. The server keeps the traffic manager registrations (address and port) of the clients of the last run.
# When the set of client hosts changes, a new client that asks for its traffic manager port is pointed at the
# old host and fails with a bind error, so the server is restarted whenever the host layout differs from the last run's.
# (the layout is the vehicle index ranges per host; on one host it does not depend on N)
LAYOUT="$VLIST"; [[ $(wc -l <<< "$HOSTS") -eq 1 ]] && LAYOUT="single:$ORCH"
# secondaries hold sensor state of the previous run: with -G the servers are always started fresh
[[ -n "$SECS" ]] && LAYOUT="secondaries:$SECS:$VLIST:$(date +%s)"
SEC_HOSTS=$(tr ',' '\n' <<< "$SECS" | sed 's/:[0-9]*$//' | sort -u)
kill_carla() { $SSH "$1" "for p in \$(pgrep -f \"^$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping\") \$(pgrep -f \"^/bin/sh $CARLA_ROOT/CarlaUE4.sh\"); do kill -9 \$p; done; true"; }
# secondaries of an earlier run are stopped whether or not this run uses them
for h in $SEC_HOSTS $($SSH "$CARLA" "cat /tmp/scale_carla_secondary_hosts 2>/dev/null"); do [[ $h == "$CARLA" ]] || kill_carla "$h"; done
if $SSH "$CARLA" "ss -ltn | grep -q ':2000 '" && ! $SSH "$CARLA" "[ \"\$(cat /tmp/scale_carla_layout 2>/dev/null)\" = \"$LAYOUT\" ]"; then
  log "host layout changed: restarting CARLA on $CARLA"
  $SSH "$CARLA" "for p in \$(pgrep -f \"^$CARLA_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping\") \$(pgrep -f \"^/bin/sh $CARLA_ROOT/CarlaUE4.sh\"); do kill -9 \$p; done; timeout 60 bash -c 'while ss -ltn | grep -q \":2000 \"; do sleep 1; done'" || fail "CARLA did not stop"
fi
$SSH "$CARLA" "echo \"$LAYOUT\" > /tmp/scale_carla_layout"
$SSH "$CARLA" "echo '$SEC_HOSTS' > /tmp/scale_carla_secondary_hosts"
if [[ -n "$SECS" ]]; then
  # A secondary renders one frame per frame the primary sends, but it keeps its own time step: with the default
  # variable step its sensors integrate over the wall-clock time between frames (the LiDAR sweeps many rotations per
  # frame and frame time stamps follow the wall clock). Its RPC port does not serve a client (get_world times out),
  # so the fixed step of the simulation is given on the command line (-benchmark -fps, SCALE_SEC_FPS, default 20).
  # primary: simulation only, no rendering; it listens for secondaries on 2002 and turns synchronous mode on when one connects
  PRIM_IP=$(ip_of "$CARLA")
  log "starting CARLA primary (no rendering) on $CARLA, secondaries: $SECS"
  $SSH "$CARLA" "(setsid nohup $CARLA_ROOT/CarlaUE4.sh -nullrhi -carla-port=2000 -carla-primary-port=2002 > /tmp/carla_scale.log 2>&1 < /dev/null &)"
  $SSH "$CARLA" "timeout 120 bash -c 'until ss -ltn | grep -q \":2000 \" && ss -ltn | grep -q \":2002 \"; do sleep 2; done'" || fail "CARLA primary did not open ports 2000 and 2002"
  sleep 8
  for e in $(tr ',' ' ' <<< "$SECS"); do h=${e%:*}; n=${e##*:}
    [[ $h == "$CARLA" ]] && ph=127.0.0.1 || ph=$PRIM_IP
    [[ $h == "$CARLA" ]] || { $SSH "$h" "sudo iptables -C $RULE 2>/dev/null || sudo iptables -I $RULE" || fail "port guard on $h"; }
    for g in $(seq 0 $((n - 1))); do rp=$((3000 + 10 * g))
      $SSH "$h" "(setsid nohup $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -carla-rpc-port=$rp -carla-primary-host=$ph -carla-primary-port=2002 ${SCALE_SEC_FLAGS:--ini:[/Script/Engine.Engine]:bUseFixedFrameRate=True -ini:[/Script/Engine.Engine]:FixedFrameRate=20.0} -ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=$g > /tmp/carla_secondary_$g.log 2>&1 < /dev/null &)"
    done
    $SSH "$h" "timeout 240 bash -c 'for g in \$(seq 0 $((n - 1))); do until ss -ltn | grep -q \":\$((3000 + 10 * g)) \"; do sleep 2; done; done'" || fail "secondaries on $h did not start"
  done
  sleep 15
  log "secondaries up: $($SSH "$CARLA" "ss -tn state established '( sport = :2002 )' | grep -c ':2002'") connections on the primary port"
elif ! $SSH "$CARLA" "ss -ltn | grep -q ':2000 '"; then
  log "starting CARLA on $CARLA"
  $SSH "$CARLA" "(setsid nohup $CARLA_ROOT/CarlaUE4.sh -RenderOffScreen -carla-port=2000 > /tmp/carla_scale.log 2>&1 < /dev/null &)"
  $SSH "$CARLA" "timeout 120 bash -c 'until ss -ltn | grep -q \":2000 \"; do sleep 2; done'" || fail "CARLA did not open port 2000"
  sleep 8
fi

# scenario files: generated on the orchestrator, copied to the other hosts (clients read the XML, the YAML comes from the server)
on "$ORCH" "python scripts/ispass/gen_scale_town06.py -n $N --rsus $R --perception $GP --carla-host $CARLA_IP $GEN" | tee -a "$OUT/launch.log" || fail "generator"
# the spread layout chooses its own RSU count
RG=$(grep -o '^RSUS=[0-9]*' "$OUT/launch.log" | tail -1 | cut -d= -f2); [[ -n "$RG" ]] && R=$RG
if [[ $MULTI == 1 && $P == shared && -z "$EPS" ]]; then
  PERC_IP=$(ip_of "$PERC")
  $SSH "$ORCH" "cd $WT && sed -i -E \"s|yolo_endpoint: *'?localhost:([0-9]+)'?|yolo_endpoint: '$PERC_IP:\\1'|\" ecav/scenario_testing/config_yaml/scale_town06.yaml && grep -q \"yolo_endpoint: '$PERC_IP:\" ecav/scenario_testing/config_yaml/scale_town06.yaml" || fail "yolo_endpoint rewrite"
  log "perception service at $PERC_IP"
fi
for h in $HOSTS; do [[ $h == "$ORCH" ]] && continue
  for f in ecav/scenario_testing/scenarios/scenario_scale_town06.xml ecav/scenario_testing/config_yaml/scale_town06.yaml; do
    scp -q -3 $SSHO "$ORCH:${WT/\$HOME\//}/$f" "$h:${WT/\$HOME\//}/$f" || fail "copy $f to $h"; done; done

T0=$(date +%s)
if [[ $MODE == seq ]]; then
  bg "$ORCH" orch "env SCENARIO_MAX_STEP=$STEPS ECAV_CARLA_TIMEOUT_S=${SCALE_CARLA_TIMEOUT:-60} python -u ecav.py -t scale_town06 -v 0.9.15 $ML"
  wait_log "$ORCH" orch "SCALE_TOWN06. tick 0 |Caught exception|Traceback" $((900 + 6 * N)) || fail "no first tick (sequential)"
  $SSH "$ORCH" "cd $WT && grep -q 'Caught exception' $L/orch.log" && fail "sequential run raised before the first tick"
  log "first tick after $(( $(date +%s) - T0 )) s"
  $SSH "$ORCH" "cd $WT && head -1 /proc/stat > $L/cpu_start.log; cat /proc/loadavg > $L/load_start.log"
  wait_log "$ORCH" orch "SCALE_TOWN06. SUMMARY" $((600 + STEPS * 30)) || fail "run did not finish"
  $SSH "$ORCH" "cd $WT && head -1 /proc/stat > $L/cpu_end.log; cat /proc/loadavg > $L/load_end.log"
  sleep 10; collect; killall_hosts
  grep -h "SUMMARY" "$OUT"/*/orch.log | tee -a "$OUT/launch.log"; log "done: $OUT"; exit 0
fi
bg "$ORCH" orch "env SCENARIO_MAX_STEP=$STEPS ECAV_CARLA_TIMEOUT_S=${SCALE_CARLA_TIMEOUT:-60} python -u ecav.py -t scale_town06 -v 0.9.15 -d $ML"
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
  # with -Y, client i gets endpoint i mod R in its environment; without it the variable stays unset
  local pre=""; [[ -n "$EPS" ]] && pre="EP=($(tr ',' ' ' <<< "$EPS")); "
  local envp=""; [[ -n "$EPS" ]] && envp="env ECAV_YOLO_ENDPOINT=\${EP[\$((i % \${#EP[@]}))]} "
  on "$h" "mkdir -p $L && ${pre}for i in \$(seq $lo $hi); do (setsid nohup ${envp}python -u ecav.py $ML -v 0.9.15 -d -i \$i -T \$((8000 + i)) > $L/veh_\$i.log 2>&1 < /dev/null &); done"; }
# wait until every client lo..hi on one host has either reached its actor client or raised
batch_wait() { $SSH "$1" "cd $WT && timeout 600 bash -c 'until ( for i in \$(seq $2 $3); do grep -Eq \"spawning Ecav2ActorClient|Traceback\" $L/veh_\$i.log 2>/dev/null || exit 1; done ); do sleep 2; done'" \
               || { log "TIMEOUT waiting for clients $2..$3 on $1"; return 1; }; }
veh_range "$(host_of 0)" 0 0
wait_log "$(host_of 0)" veh_0 "spawning Ecav2ActorClient" $((120 + N)) || fail "client 0 did not spawn the egos"
log "client 0 spawned the egos"
k=1; while (( k < N )); do
  h=$(host_of $k); hi=$(( k + BATCH - 1 < N - 1 ? k + BATCH - 1 : N - 1 ))
  while [[ $(host_of $hi) != "$h" ]]; do hi=$(( hi - 1 )); done
  veh_range "$h" $k $hi
  batch_wait "$h" $k $hi || fail "clients $k..$hi did not start"
  # Every client is a remote user of client 0's traffic manager (the scenario YAML gives all of them port 8000),
  # and that RPC has a fixed 2 s timeout. With many sensors in the world the simulator answers slowly and a client
  # can die on set_random_device_seed before it has created anything. Such a client is started again.
  for try in 1 2 3; do
    bad=$($SSH "$h" "cd $WT && for i in \$(seq $k $hi); do grep -q 'spawning Ecav2ActorClient' $L/veh_\$i.log || { grep -q 'rpc::timeout' $L/veh_\$i.log && echo \$i; }; done" | tr '\n' ' ')
    [[ -z "${bad// }" ]] && break
    log "clients ${bad}timed out on the traffic manager RPC: starting them again (try $try)"
    for i in $bad; do $SSH "$h" "cd $WT && mv $L/veh_$i.log $L/veh_$i.log.rpc_timeout$try"; veh_range "$h" $i $i; done
    batch_wait "$h" $k $hi || fail "clients $k..$hi did not start"
  done
  log "clients $k..$hi started on $h"; k=$(( hi + 1 ))
done
EPA=($(tr ',' ' ' <<< "$EPS"))
for j in $(seq 0 $((R - 1))); do
  rpre=""; [[ -n "$EPS" ]] && rpre="env ECAV_YOLO_ENDPOINT=${EPA[$((j % ${#EPA[@]}))]} "
  bg "$RSUH" "rsu_$j" "${rpre}python -u ecav/ecav2/ecloud_actor_client.py $ML -v 0.9.15 -i $j"; done
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
