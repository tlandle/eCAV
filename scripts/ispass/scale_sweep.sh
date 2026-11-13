#!/usr/bin/env bash
# Single-host scale sweep for the ISPASS scaling section, one code version for the whole table.
#   scripts/ispass/scale_sweep.sh <stamp> <host> [parts]      parts: dense spread seq percep (default: all)
# Multi-host parts (not in the default): mhdense mhspread mhpercep mhstrong mhequiv. <host> keeps CARLA, the orchestrator, the edge,
# the RSU clients and the inference service; the vehicle clients are split evenly over the hosts in CLIENT_HOSTS:
#   CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 "mhdense mhspread"
# Perception replicas (not in the default): mhprepl. The inference service runs as R replicas, one per GPU, on
# PERC_HOST (scripts/ispass/perception_replicas.sh); client k calls replica k mod R. R in MH_RS (default 1 2 4 8),
# N in MH_RN (default 8 16 32 64 128); runs are <stamp>_mhpr<R>_n<N>:
#   CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" PERC_HOST=azureuser@20.172.248.156 \
#       scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 mhprepl
# Runs land in experiment_results/ispass_scale/<stamp>_<config>_n<N>; scale_report.py <stamp> builds the table and figure.
# 300 ticks up to 128 vehicles, 100 ticks above (a tick takes seconds there). A failed run is retried once.
set -uo pipefail
STAMP=$1; HOST=$2; PARTS=${3:-"dense spread seq percep"}
cd "$(dirname "$0")/../.."
L=scripts/ispass/launch_scale.sh
steps() { (( $1 > 128 )) && echo 100 || echo 300; }
one() { local tag=$1; shift
  [[ -f experiment_results/ispass_scale/$tag/launch.log ]] && grep -q "done:" experiment_results/ispass_scale/$tag/launch.log && { echo "$tag exists"; return; }
  for try in 1 2; do rm -rf experiment_results/ispass_scale/$tag
    if timeout 5400 $L "$@" -o $HOST -t $tag > /dev/null 2>&1; then break; fi; echo "$(date '+%T') $tag attempt $try failed"; done
  echo "$(date '+%T') $tag: $(grep -h SUMMARY experiment_results/ispass_scale/$tag/launch.log | tail -1)"; }
{ git rev-parse HEAD; date -Is; } > experiment_results/ispass_scale/${STAMP}_meta.txt
# vehicle index ranges for launch_scale.sh -H: N clients split evenly over CLIENT_HOSTS
split() { local n=$1 hs=($CLIENT_HOSTS) k=${#hs[@]} i lo=0 hi out=""
  for (( i = 0; i < k; i++ )); do hi=$(( lo + n / k - 1 + (i < n % k ? 1 : 0) )); out+="${hs[i]}:$lo-$hi,"; lo=$(( hi + 1 )); done; echo "${out%,}"; }
perc_start() { ssh $HOST 'cd $HOME/scale_wt && source $HOME/miniconda3/etc/profile.d/conda.sh && conda activate opencda310 && (PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True PYTHONUNBUFFERED=1 setsid nohup python ecav/ml_manager/yolo_grpc_server.py > /tmp/yolo_grpc_scale.log 2>&1 < /dev/null &); timeout 180 bash -c "until ss -ltn | grep -q \":18001 \"; do sleep 2; done"'
  ssh $HOST '(setsid nohup nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits -l 1 > /tmp/gpu_scale.csv 2>/dev/null < /dev/null &)'; }
perc_stop() { ssh $HOST 'for p in $(pgrep -f "^nvidia-smi --query-gpu=utilization.gpu,memory.used"); do kill $p; done; for p in $(pgrep -f "^python ecav/ml_manager/yolo_grpc_server.py"); do kill -9 $p; done'
  scp -q $HOST:/tmp/gpu_scale.csv experiment_results/ispass_scale/$1/gpu.csv; scp -q $HOST:/tmp/yolo_grpc_scale.log experiment_results/ispass_scale/$1/yolo_grpc.log; }
[[ "$PARTS" == *mh* && -z "${CLIENT_HOSTS:-}" ]] && { echo "multi-host parts need CLIENT_HOSTS"; exit 2; }
for part in $PARTS; do case $part in
  mhdense)  for n in ${MH_NS:-64 128 256 512}; do one ${STAMP}_mhdense_n$n -n $n -r 2 -p off -s $(steps $n) -H "$(split $n)"; done ;;
  mhspread) for n in ${MH_NS:-64 128 256 512}; do one ${STAMP}_mhspread_n$n -n $n -r 20 -p off -s $(steps $n) -g "--layout spread" -H "$(split $n)"; done ;;
  mhstrong) # strong scaling at one vehicle count: every client on <host>, on the first client host, on all client hosts
    n=${MH_SN:-256}; hs=($CLIENT_HOSTS)
    one ${STAMP}_dense_n$n -n $n -r 2 -p off -s $(steps $n)
    one ${STAMP}_h1dense_n$n -n $n -r 2 -p off -s $(steps $n) -H "${hs[0]}:0-$((n - 1))"
    one ${STAMP}_mhdense_n$n -n $n -r 2 -p off -s $(steps $n) -H "$(split $n)" ;;
  mhequiv) # the same scenario with every client on <host> and with the clients on the client hosts
    n=${MH_EN:-16}
    one ${STAMP}_eqsh_n$n -n $n -r 2 -p off -s 300
    one ${STAMP}_eqrep_n$n -n $n -r 2 -p off -s 300     # repeat on one host: the run-to-run difference (GNSS noise is not seeded)
    one ${STAMP}_eqmh_n$n -n $n -r 2 -p off -s 300 -H "$(split $n)" ;;
  mhprepl) [[ -n "${PERC_HOST:-}" ]] || { echo "mhprepl needs PERC_HOST"; exit 2; }
    for r in ${MH_RS:-1 2 4 8}; do for n in ${MH_RN:-8 16 32 64 128}; do tag=${STAMP}_mhpr${r}_n$n
      [[ -f experiment_results/ispass_scale/$tag/launch.log ]] && grep -q "done:" experiment_results/ispass_scale/$tag/launch.log && { echo "$tag exists"; continue; }
      eps=$(scripts/ispass/perception_replicas.sh start $PERC_HOST $r) || { echo "$tag: replicas did not start"; continue; }
      one $tag -n $n -r 2 -p shared -s 300 -H "$(split $n)" -Y "$eps"
      scripts/ispass/perception_replicas.sh collect $PERC_HOST experiment_results/ispass_scale/$tag
      scripts/ispass/perception_replicas.sh stop $PERC_HOST; done; done ;;
  mhpercep) for n in ${MH_PN:-8 16 32 64}; do perc_start; one ${STAMP}_mhpdense_n$n -n $n -r 2 -p shared -s 300 -H "$(split $n)"; perc_stop ${STAMP}_mhpdense_n$n; done ;;
  dense)  for n in 8 16 32 64 128 256; do one ${STAMP}_dense_n$n -n $n -r 2 -p off -s $(steps $n); done ;;
  spread) for n in 8 16 32 64 128 256 512; do one ${STAMP}_spread_n$n -n $n -r 20 -p off -s $(steps $n) -g "--layout spread"; done ;;
  seq)    for n in 8 16 32; do one ${STAMP}_seqdense_n$n -n $n -r 2 -p off -s 300 -m seq
                               one ${STAMP}_seqspread_n$n -n $n -r 20 -p off -s 300 -m seq -g "--layout spread"; done ;;
  percep) # shared YOLO inference service on the GPU host, restarted per run so its log covers one run;
          # GPU utilisation and memory sampled once per second
    for n in 8 16 32; do
      # the service command line must not start with "python -u ecav": the launcher kills those at the start of a run
      ssh $HOST 'cd $HOME/scale_wt && source $HOME/miniconda3/etc/profile.d/conda.sh && conda activate opencda310 && (PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True PYTHONUNBUFFERED=1 setsid nohup python ecav/ml_manager/yolo_grpc_server.py > /tmp/yolo_grpc_scale.log 2>&1 < /dev/null &); timeout 180 bash -c "until ss -ltn | grep -q :18001; do sleep 2; done"' || echo "YOLO service did not start"
      ssh $HOST '(setsid nohup nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits -l 1 > /tmp/gpu_scale.csv 2>/dev/null < /dev/null &)'
      one ${STAMP}_pdense_n$n -n $n -r 2 -p shared -s 300
      ssh $HOST 'for p in $(pgrep -f "^nvidia-smi --query-gpu=utilization.gpu,memory.used"); do kill $p; done; for p in $(pgrep -f "^python ecav/ml_manager/yolo_grpc_server.py"); do kill -9 $p; done'
      scp -q $HOST:/tmp/gpu_scale.csv experiment_results/ispass_scale/${STAMP}_pdense_n$n/gpu.csv
      scp -q $HOST:/tmp/yolo_grpc_scale.log experiment_results/ispass_scale/${STAMP}_pdense_n$n/yolo_grpc.log
    done ;;
esac; done
echo "$(date '+%T') sweep $STAMP parts [$PARTS] end"
