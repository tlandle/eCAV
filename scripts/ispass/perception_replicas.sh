#!/usr/bin/env bash
# Start or stop R replicas of the shared YOLO perception service on one host, one replica per GPU
# (replica g on GPU g mod #GPUs, port BASE+g). Prints the endpoint list (private address) on start.
#
#   scripts/ispass/perception_replicas.sh start <ssh host> <R> [base port, default 18001]
#   scripts/ispass/perception_replicas.sh stop  <ssh host>
#   scripts/ispass/perception_replicas.sh collect <ssh host> <local dir>     logs and the GPU sample file
#
# Environment: SCALE_WT (default $HOME/scale_wt on the host), YOLO_GRPC_WORKERS (threads per replica, default 4).
# The host needs the worktree with yolov5/ and yolov5m.pt, the opencda310 env, and, where the env lacks
# grpcio, the packages under $HOME/scale_pydeps (put on PYTHONPATH when the directory exists).
# The service command line must not start with "python -u ecav": the launcher kills those at the start of a run.
set -uo pipefail
CMD=${1:?start|stop|collect}; HOST=${2:?ssh host}
SSH="ssh -o BatchMode=yes -o ConnectTimeout=15"
ENVL='if [ -f $HOME/miniconda3/etc/profile.d/conda.sh ]; then source $HOME/miniconda3/etc/profile.d/conda.sh; conda activate opencda310; else export PATH=$HOME/miniconda3/envs/opencda310/bin:$PATH; fi; [ -d $HOME/scale_pydeps ] && export PYTHONPATH=$HOME/scale_pydeps:${PYTHONPATH:-}'
WT=${SCALE_WT:-'$HOME/scale_wt'}
stop() { $SSH "$HOST" 'for p in $(pgrep -f "^nvidia-smi --query-gpu=index,utilization.gpu,memory.used") $(pgrep -f "^python ecav/ml_manager/yolo_grpc_server.py"); do kill -9 $p; done; true'; }
case $CMD in
  start) R=${3:?replicas}; BASE=${4:-18001}; stop
    # replicas bind the host's private address only: a VM may carry a security group that admits any port from the internet
    $SSH "$HOST" "cd $WT && $ENVL; G=\$(nvidia-smi -L | wc -l); IP=\$(hostname -I | cut -d' ' -f1); rm -f /tmp/yolo_grpc_r*.log
      for g in \$(seq 0 $((R - 1))); do
        (CUDA_VISIBLE_DEVICES=\$((g % G)) YOLO_GRPC_WORKERS=${YOLO_GRPC_WORKERS:-4} PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True PYTHONUNBUFFERED=1 \
          setsid nohup python ecav/ml_manager/yolo_grpc_server.py --host \$IP --port \$(($BASE + g)) > /tmp/yolo_grpc_r\$g.log 2>&1 < /dev/null &); done
      timeout 300 bash -c 'for g in \$(seq 0 $((R - 1))); do until ss -ltn | grep -q \":\$(($BASE + g)) \"; do sleep 2; done; done' || { echo 'replicas did not start' >&2; exit 1; }
      (setsid nohup nvidia-smi --query-gpu=index,utilization.gpu,memory.used --format=csv,noheader,nounits -l 1 > /tmp/gpu_replicas.csv 2>/dev/null < /dev/null &)
      for g in \$(seq 0 $((R - 1))); do printf '%s:%s\n' \$IP \$(($BASE + g)); done | paste -sd," ;;
  stop) stop ;;
  collect) D=${3:?local dir}; mkdir -p "$D"
    scp -q -o BatchMode=yes "$HOST:/tmp/yolo_grpc_r*.log" "$D/" 2>/dev/null; scp -q -o BatchMode=yes "$HOST:/tmp/gpu_replicas.csv" "$D/gpu_replicas.csv" 2>/dev/null
    cat "$D"/yolo_grpc_r*.log > "$D/yolo_grpc.log" 2>/dev/null; true ;;
  *) echo "unknown command"; exit 2 ;;
esac
