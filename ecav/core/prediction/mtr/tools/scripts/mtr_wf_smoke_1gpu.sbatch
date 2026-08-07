#!/bin/bash
#SBATCH --job-name=mtr-wf-smoke
#SBATCH --account=paceship-cavef
#SBATCH --partition=gpu-h200
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --gres=gpu:h200:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=256G
#SBATCH --time=01:00:00
#SBATCH --output=%x_%j.out
#SBATCH --error=%x_%j.err
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --mail-user=tlandle3@gatech.edu
set -euo pipefail

# --- Paths ---
PROJECT=/storage/project/ps-cavef-0/tlandle3
CONDA=$PROJECT/miniconda3
ENV_NAME=opencda310
ENV_PREFIX=$CONDA/envs/$ENV_NAME
# MTR code tar (upload once): tar -cf - -C <repo>/ecav/core/application/v2v/baselines/cmp/CMP MTR | ssh ... 'cat > $PROJECT/mtr_code.tar'
MTR_CODE_TAR=$PROJECT/mtr_code.tar
DATA_TAR=$PROJECT/wf_mtr_translaug.tar        # from upload_wf_mtr_to_pace.sh
CFG_REL=tools/cfgs/multiv2x/multiv2x_multiego_worldfusion.yaml

# --- Claim-lock: submitted in parallel to gpu-h200 and gpu-h100; whichever
#     node starts first claims the run, the sibling yields cleanly. ---
LOCK=$PROJECT/mtr_wf_smoke.lock
if ! ( set -o noclobber; echo "$SLURM_JOB_ID @ $(hostname)" > "$LOCK" ) 2>/dev/null; then
  echo "smoke already claimed by $(cat "$LOCK" 2>/dev/null); yielding."; exit 0
fi
trap 'rm -f "$LOCK"' EXIT

# --- Compute-node local NVMe (fast; Lustre is slow for many small files) ---
LOCAL=${TMPDIR:-/tmp}/mtr_wf_$SLURM_JOB_ID
mkdir -p "$LOCAL"
echo "=== node $(hostname) | LOCAL=$LOCAL | $(date) ==="
nvidia-smi

# --- Conda ---
export CONDA_NO_PLUGINS=true CONDA_SOLVER=classic
export PIP_CACHE_DIR=$PROJECT/pip_cache XDG_CACHE_HOME=$PROJECT/.cache
source $CONDA/etc/profile.d/conda.sh
conda activate "$ENV_PREFIX"
NVJIT="$ENV_PREFIX/lib/python3.10/site-packages/nvidia/nvjitlink/lib"
export LD_LIBRARY_PATH="$NVJIT:${LD_LIBRARY_PATH:-}"
python -c "import torch;print('torch',torch.__version__,'cuda',torch.cuda.is_available(),torch.cuda.device_count())"

# --- Stage code + data onto local NVMe ---
echo "=== untar MTR code ==="
tar -xf "$MTR_CODE_TAR" -C "$LOCAL"        # -> $LOCAL/MTR
MTR=$LOCAL/MTR
echo "=== untar data ($(du -h $DATA_TAR | cut -f1)) ==="
tar -xf "$DATA_TAR" -C "$LOCAL"            # -> $LOCAL/multiv2x_mtr_wf_translaug
DATA_ROOT=$LOCAL/multiv2x_mtr_wf_translaug
ls "$DATA_ROOT"

# --- Build MTR CUDA ops on the node (knn, attention) ---
# torch is cu121 wheel (runtime only, no nvcc), so load a matching CUDA
# toolkit module for nvcc, and build with --no-build-isolation so torch is
# importable at build time (setup.py uses torch.utils.cpp_extension).
echo "=== building MTR ops ==="
source /etc/profile.d/z00_lmod.sh 2>/dev/null || true
module load cuda/12.1.1
export CUDA_HOME="${CUDA_HOME:-$(dirname "$(dirname "$(command -v nvcc)")")}"
# CUPTI is in extras/, not lib64/: with the module on LD_LIBRARY_PATH torch
# resolves cudart from the module and then fails on libcupti.so.12 at import.
export LD_LIBRARY_PATH="$CUDA_HOME/extras/CUPTI/lib64:${LD_LIBRARY_PATH:-}"
export TORCH_CUDA_ARCH_LIST="9.0"   # Hopper: H100 + H200
echo "nvcc=$(command -v nvcc)  CUDA_HOME=$CUDA_HOME"
nvcc --version | tail -2
cd "$MTR"
pip install -e . --no-build-isolation 2>&1 | tee "$LOCAL/mtr_build.log" | tail -25
python -c "from mtr.ops.knn import knn_utils; from mtr.ops.attention import attention_utils; print('MTR CUDA ops import OK')" \
  || { echo 'MTR ops build/import FAILED — full log:'; cat "$LOCAL/mtr_build.log"; exit 1; }

# --- Point config at local data root + intention pkl ---
CFG=$MTR/$CFG_REL
python - "$CFG" "$DATA_ROOT" <<'PY'
import sys, re
cfg, root = sys.argv[1], sys.argv[2]
s = open(cfg).read()
s = re.sub(r"root_dir:\s*'[^']*'", f"root_dir: '{root}'", s)
s = re.sub(r"INTENTION_POINTS_FILE:\s*'[^']*'",
           f"INTENTION_POINTS_FILE: '{root}/multiv2x_cluster_64_center_dict.pkl'",
           s)
s = re.sub(r"LANE_ENCODER:\s*'[^']*'",
           "LANE_ENCODER: 'pretrained/swin-base-patch4-window7-224'", s)
open(cfg, 'w').write(s)
print('patched', cfg)
PY
grep -E "root_dir|INTENTION_POINTS_FILE" "$CFG"

# --- Smoke: 1 GPU, 1 epoch ---
echo "=== training (1 GPU, 1 epoch) at $(date) ==="
cd "$MTR"
python tools/train_multiego.py \
  --cfg_file "$CFG_REL" \
  --batch_size 1 \
  --epochs 1 \
  --workers 8 \
  --extra_tag wf_smoke_1gpu \
  --launcher none

echo "=== DONE $(date) ==="
ls -lh "$MTR"/output/*/wf_smoke_1gpu/ckpt/*.pth 2>/dev/null || true
