#!/bin/bash
#SBATCH --job-name=mtr-wf-2gpu
#SBATCH --account=paceship-cavef
#SBATCH --partition=gpu-h200
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --gres=gpu:h200:2
#SBATCH --cpus-per-task=16
#SBATCH --mem=384G
#SBATCH --time=02:00:00
#SBATCH --output=%x_%j.out
#SBATCH --error=%x_%j.err
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --mail-user=tlandle3@gatech.edu
set -euo pipefail

NGPUS=2
EPOCHS=3
PROJECT=/storage/project/ps-cavef-0/tlandle3
CONDA=$PROJECT/miniconda3
ENV_PREFIX=$CONDA/envs/opencda310
MTR_CODE_TAR=$PROJECT/mtr_code.tar
DATA_TAR=$PROJECT/wf_mtr_translaug.tar
CFG_REL=tools/cfgs/multiv2x/multiv2x_multiego_worldfusion.yaml

LOCAL=${TMPDIR:-/tmp}/mtr_wf_$SLURM_JOB_ID
mkdir -p "$LOCAL"
echo "=== node $(hostname) | ${NGPUS} GPU | $(date) ==="
nvidia-smi -L

export CONDA_NO_PLUGINS=true CONDA_SOLVER=classic
export PIP_CACHE_DIR=$PROJECT/pip_cache XDG_CACHE_HOME=$PROJECT/.cache
source $CONDA/etc/profile.d/conda.sh
conda activate "$ENV_PREFIX"
export LD_LIBRARY_PATH="$ENV_PREFIX/lib/python3.10/site-packages/nvidia/nvjitlink/lib:${LD_LIBRARY_PATH:-}"

tar -xf "$MTR_CODE_TAR" -C "$LOCAL"; MTR=$LOCAL/MTR
tar -xf "$DATA_TAR" -C "$LOCAL"; DATA_ROOT=$LOCAL/multiv2x_mtr_wf_translaug
# Build MTR CUDA ops: load matching CUDA toolkit for nvcc, no build isolation.
source /etc/profile.d/z00_lmod.sh 2>/dev/null || true
module load cuda/12.1.1
export CUDA_HOME="${CUDA_HOME:-$(dirname "$(dirname "$(command -v nvcc)")")}"
# CUPTI is in extras/, not lib64/: with the module on LD_LIBRARY_PATH torch
# resolves cudart from the module and then fails on libcupti.so.12 at import.
export LD_LIBRARY_PATH="$CUDA_HOME/extras/CUPTI/lib64:${LD_LIBRARY_PATH:-}"
export TORCH_CUDA_ARCH_LIST="9.0"
cd "$MTR"
pip install -e . --no-build-isolation 2>&1 | tee "$LOCAL/mtr_build.log" | tail -20
python -c "from mtr.ops.knn import knn_utils; from mtr.ops.attention import attention_utils; print('MTR CUDA ops OK')" \
  || { echo 'MTR ops FAILED'; cat "$LOCAL/mtr_build.log"; exit 1; }

python - "$MTR/$CFG_REL" "$DATA_ROOT" <<'PY'
import sys, re
cfg, root = sys.argv[1], sys.argv[2]
s = open(cfg).read()
s = re.sub(r"root_dir:\s*'[^']*'", f"root_dir: '{root}'", s)
s = re.sub(r"INTENTION_POINTS_FILE:\s*'[^']*'",
           f"INTENTION_POINTS_FILE: '{root}/multiv2x_cluster_64_center_dict.pkl'", s)
open(cfg, 'w').write(s)
PY

PORT=$(( (RANDOM % 20000) + 20000 ))
echo "=== DDP ${NGPUS} GPU, ${EPOCHS} epochs, port ${PORT} at $(date) ==="
cd "$MTR"
torchrun --nproc_per_node=${NGPUS} --master_port=${PORT} \
  tools/train_multiego.py \
  --cfg_file "$CFG_REL" \
  --batch_size ${NGPUS} \
  --epochs ${EPOCHS} \
  --workers 8 \
  --extra_tag wf_2gpu \
  --launcher pytorch

echo "=== DONE $(date) ==="
