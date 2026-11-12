#!/usr/bin/env bash
# Prepare one client VM for the multi-host scale run. Run from atlas.
#
#   scripts/ispass/setup_scale_vm.sh <vm ssh target> [source ssh target, default azureuser@20.81.176.215]
#
# The source is a host that already runs the scale scenario (the A10 node). Files are streamed
# source -> atlas -> VM with tar over ssh, so the source and the VM need no trust in each other
# and the VM needs no GitHub access. Steps:
#   1. worktree $HOME/scale_wt (symlinked submodules sort and ecav/worldfusion are copied as real directories;
#      logs, results and model checkpoints are left out: clients without perception load no model)
#   2. CARLA PythonAPI under $HOME/carla-0.9.15/PythonAPI
#   3. Python env opencda310: copied from the source (without the CUDA toolkit) to the same path and used
#      through PATH; built on the VM (miniconda + pip; needs internet) only if the import test fails there.
#      The source runs Ubuntu 24.04 and the client VMs Ubuntu 20.04; the import test decides.
#   4. reserved service ports (sysctl), file descriptor limit, clock check (chrony)
#   5. import test and a dry generator run
set -euo pipefail
VM=${1:?vm ssh target}; SRC=${2:-azureuser@20.81.176.215}
SSHO="-o BatchMode=yes -o ConnectTimeout=15"
say() { echo "$(date '+%T') [$VM] $*"; }
stream() { # stream <source dir relative to the source HOME> <tar options...>
  local d=$1; shift
  # tar exits 1 for a dangling symlink or a file that changed while it was read; neither is fatal here
  ssh $SSHO "$SRC" "cd \$HOME && tar -c${DEREF-h}f - --ignore-failed-read $* $d; [ \$? -le 1 ]" | ssh $SSHO "$VM" "cd \$HOME && tar -xf -"; }

say "worktree"
stream scale_wt --exclude=scale_wt/logs --exclude=scale_wt/log --exclude=scale_wt/experiment_results \
       --exclude='*.pth' --exclude='*.pt' --exclude=scale_wt/.git --exclude='__pycache__' \
       --exclude=scale_wt/preprocessed_data --exclude=scale_wt/evaluation_outputs --exclude=scale_wt/docs
ssh $SSHO "$VM" 'mkdir -p $HOME/scale_wt/logs $HOME/scale_wt/log'
say "CARLA PythonAPI"
ssh $SSHO "$VM" 'mkdir -p $HOME/carla-0.9.15'
stream carla-0.9.15/PythonAPI

TEST='import carla, grpc, numpy, scipy, cv2, yaml, shapely, py_trees, omegaconf, torch, ecloud_pb2; print("imports ok")'
ENVD='$HOME/miniconda3/envs/opencda310'
RUN="export PATH=$ENVD/bin:\$PATH && cd \$HOME/scale_wt && PYTHONPATH=\$HOME/carla-0.9.15/PythonAPI/carla:\$HOME/scale_wt/ecav/protos"
say "Python env: copy from the source"
# The env is copied to the same absolute path (same user name on every host), so it runs without a conda
# base install: the launcher puts its bin directory on PATH. The CUDA toolkit (targets, nsight-compute) is
# left out: 18 GB becomes about 10 GB, which fits the 30 GB root disk of a client VM. The temporary disk
# at /mnt is not used because a deallocation erases it.
if ! ssh $SSHO "$VM" "test -x $ENVD/bin/python"; then
  ssh $SSHO "$VM" 'mkdir -p $HOME/miniconda3/envs'
  # symlinks are kept inside the env (DEREF empty): following them copies every shared library twice (13 GB)
  DEREF="" stream miniconda3/envs/opencda310 --exclude='*.pyc' --exclude='__pycache__' --exclude=miniconda3/envs/opencda310/targets \
         --exclude='miniconda3/envs/opencda310/nsight-compute*' --exclude=miniconda3/envs/opencda310/pkgs
fi
if ! ssh $SSHO "$VM" "$RUN python -c '$TEST'"; then
  say "copied env does not import on this OS; building the env on the VM"
  ssh $SSHO "$VM" 'test -x $HOME/miniconda3/bin/conda || { curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/mc.sh && bash /tmp/mc.sh -b -u -p $HOME/miniconda3; }'
  ssh $SSHO "$VM" 'source $HOME/miniconda3/etc/profile.d/conda.sh && rm -rf $HOME/miniconda3/envs/opencda310 && conda create -y -n opencda310 python=3.10 && conda activate opencda310 && cd $HOME/scale_wt && pip install -r requirements_3_10.txt && pip install carla==0.9.15'
  ssh $SSHO "$VM" "$RUN python -c '$TEST'" || { say "env build failed"; exit 1; }
fi

say "ports, limits, clock"
ssh $SSHO "$VM" 'echo "net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300" | sudo tee /etc/sysctl.d/90-scale.conf >/dev/null; sudo sysctl -q -p /etc/sysctl.d/90-scale.conf; \
  grep -q "nofile 65536" /etc/security/limits.d/scale.conf 2>/dev/null || echo "* - nofile 65536" | sudo tee /etc/security/limits.d/scale.conf >/dev/null; \
  (chronyc tracking 2>/dev/null | grep -E "System time|Leap status") || (timedatectl | grep -E "synchronized|NTP service") || echo "NO CLOCK SYNC SERVICE FOUND: install chrony"; nproc; free -g | sed -n 2p'
say "generator dry run"
ssh $SSHO "$VM" "$RUN python scripts/ispass/gen_scale_town06.py -n 8 --xodr \$HOME/scale_wt/Town06.xodr 2>&1 | tail -2" || say "generator needs Town06.xodr only on the orchestrator host; not fatal on a client"
say "ready. The network must allow TCP between the hosts on 2000-2002, 50051-50070, 50101-51300 and the ephemeral range (a client can fall back to an OS-assigned port)."
