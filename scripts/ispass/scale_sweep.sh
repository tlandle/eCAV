#!/usr/bin/env bash
# Single-host scale sweep for the ISPASS scaling section, one code version for the whole table.
#   scripts/ispass/scale_sweep.sh <stamp> <host> [parts]      parts: dense spread seq percep (default: all)
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
for part in $PARTS; do case $part in
  dense)  for n in 8 16 32 64 128 256; do one ${STAMP}_dense_n$n -n $n -r 2 -p off -s $(steps $n); done ;;
  spread) for n in 8 16 32 64 128 256 512; do one ${STAMP}_spread_n$n -n $n -r 20 -p off -s $(steps $n) -g "--layout spread"; done ;;
  seq)    for n in 8 16 32; do one ${STAMP}_seqdense_n$n -n $n -r 2 -p off -s 300 -m seq
                               one ${STAMP}_seqspread_n$n -n $n -r 20 -p off -s 300 -m seq -g "--layout spread"; done ;;
  percep) # shared YOLO inference service on the GPU host, GPU sampled once per second during each run
    ssh $HOST 'cd $HOME/scale_wt && source $HOME/miniconda3/etc/profile.d/conda.sh && conda activate opencda310 && (PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True setsid nohup python -u ecav/ml_manager/yolo_grpc_server.py > /tmp/yolo_grpc_scale.log 2>&1 < /dev/null &); timeout 120 bash -c "until ss -ltn | grep -q :18001; do sleep 2; done"' || echo "YOLO service did not start"
    for n in 8 16 32; do
      ssh $HOST '(setsid nohup nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits -l 1 > /tmp/gpu_scale.csv 2>/dev/null < /dev/null &)'
      one ${STAMP}_pdense_n$n -n $n -r 2 -p on -s 300
      ssh $HOST 'for p in $(pgrep -f "^nvidia-smi --query-gpu=utilization.gpu,memory.used"); do kill $p; done'
      scp -q $HOST:/tmp/gpu_scale.csv experiment_results/ispass_scale/${STAMP}_pdense_n$n/gpu.csv
      scp -q $HOST:/tmp/yolo_grpc_scale.log experiment_results/ispass_scale/${STAMP}_pdense_n$n/yolo_grpc.log
    done
    ssh $HOST 'for p in $(pgrep -f "^python -u ecav/ml_manager/yolo_grpc_server.py"); do kill -9 $p; done' ;;
esac; done
echo "$(date '+%T') sweep $STAMP parts [$PARTS] end"
