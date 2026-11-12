#!/usr/bin/env bash
# Prepare one client VM for the multi-host scale run. Run from atlas; NOT yet run on any VM (2026-11-12:
# the client VMs accept only a password and atlas has no key on them).
#
#   scripts/ispass/setup_scale_vm.sh <vm ssh target> [source ssh target, default azureuser@20.81.176.215]
#
# The source is a host that already runs the scale scenario (the A10 node). Files are streamed
# source -> atlas -> VM with tar over ssh, so the source and the VM need no trust in each other
# and the VM needs no GitHub access. Steps:
#   1. worktree $HOME/scale_wt (symlinked submodules sort and ecav/worldfusion are copied as real directories;
#      logs, results and model checkpoints are left out: clients without perception load no model)
#   2. CARLA PythonAPI under $HOME/carla-0.9.15/PythonAPI
#   3. Python env opencda310: copied from the source when the import test passes on the VM, otherwise
#      built on the VM (miniconda + pip; needs internet). The source runs Ubuntu 24.04 and the client VMs
#      Ubuntu 20.04, so the copied env can fail on glibc; the import test decides.
#   4. reserved service ports (sysctl), file descriptor limit, clock check (chrony)
#   5. import test and a dry generator run
set -euo pipefail
VM=${1:?vm ssh target}; SRC=${2:-azureuser@20.81.176.215}
SSHO="-o BatchMode=yes -o ConnectTimeout=15"
say() { echo "$(date '+%T') [$VM] $*"; }
stream() { # stream <source dir relative to the source HOME> <tar options...>
  local d=$1; shift
  ssh $SSHO "$SRC" "cd \$HOME && tar -chf - $* $d" | ssh $SSHO "$VM" "cd \$HOME && tar -xf -"; }

say "worktree"
stream scale_wt --exclude=scale_wt/logs --exclude=scale_wt/log --exclude=scale_wt/experiment_results \
       --exclude='*.pth' --exclude='*.pt' --exclude=scale_wt/.git --exclude='__pycache__'
ssh $SSHO "$VM" 'mkdir -p $HOME/scale_wt/logs $HOME/scale_wt/log'
say "CARLA PythonAPI"
ssh $SSHO "$VM" 'mkdir -p $HOME/carla-0.9.15'
stream carla-0.9.15/PythonAPI

TEST='import carla, grpc, numpy, scipy, cv2, yaml, shapely, py_trees, omegaconf, ecloud_pb2; print("imports ok")'
RUN='source $HOME/miniconda3/etc/profile.d/conda.sh && conda activate opencda310 && cd $HOME/scale_wt && PYTHONPATH=$HOME/carla-0.9.15/PythonAPI/carla'
say "Python env: copy from the source"
if ! ssh $SSHO "$VM" 'test -x $HOME/miniconda3/bin/conda'; then
  ssh $SSHO "$VM" 'curl -fsSL https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -o /tmp/mc.sh && bash /tmp/mc.sh -b -p $HOME/miniconda3'
fi
stream miniconda3/envs/opencda310 --exclude='*.pyc' || true
if ! ssh $SSHO "$VM" "$RUN python -c '$TEST'"; then
  say "copied env does not import on this OS; building the env on the VM"
  ssh $SSHO "$VM" 'source $HOME/miniconda3/etc/profile.d/conda.sh && rm -rf $HOME/miniconda3/envs/opencda310 && conda create -y -n opencda310 python=3.10 && conda activate opencda310 && cd $HOME/scale_wt && pip install -r requirements_3_10.txt && pip install carla==0.9.15'
  ssh $SSHO "$VM" "$RUN python -c '$TEST'" || { say "env build failed"; exit 1; }
fi

say "ports, limits, clock"
ssh $SSHO "$VM" 'sudo sysctl -w net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300 >/dev/null; \
  grep -q "nofile 65536" /etc/security/limits.d/scale.conf 2>/dev/null || echo "* - nofile 65536" | sudo tee /etc/security/limits.d/scale.conf >/dev/null; \
  (chronyc tracking 2>/dev/null | grep -E "System time|Leap status") || (timedatectl | grep -E "synchronized|NTP service") || echo "NO CLOCK SYNC SERVICE FOUND: install chrony"; nproc; free -g | sed -n 2p'
say "generator dry run"
ssh $SSHO "$VM" "$RUN python scripts/ispass/gen_scale_town06.py -n 8 --xodr \$HOME/scale_wt/Town06.xodr 2>&1 | tail -2" || say "generator needs Town06.xodr only on the orchestrator host; not fatal on a client"
say "ready. The network must allow TCP between the hosts on 2000-2002, 50051-50070, 50101-51300 and the ephemeral range (a client can fall back to an OS-assigned port)."
