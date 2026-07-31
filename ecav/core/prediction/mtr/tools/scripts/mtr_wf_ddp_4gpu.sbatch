#!/bin/bash
#SBATCH --job-name=mtr-wf-4gpu
#SBATCH --account=paceship-cavef
#SBATCH --partition=gpu-h200
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --gres=gpu:h200:4
#SBATCH --cpus-per-task=32
#SBATCH --mem=512G
#SBATCH --time=12:00:00
#SBATCH --output=%x_%j.out
#SBATCH --error=%x_%j.err
#SBATCH --mail-type=BEGIN,END,FAIL
#SBATCH --mail-user=tlandle3@gatech.edu
set -euo pipefail

# Two-stage CMP recipe on WorldFusion features:
#   stage 1: base MTR, no aggregator (_no_agg yaml), from scratch
#   stage 2: Transformer aggregator over the WF fused BEV feature,
#            initialized from the stage-1 best_model (lowest eval MR)
NGPUS=4
EPOCHS=30
PROJECT=/storage/project/ps-cavef-0/tlandle3
CONDA=$PROJECT/miniconda3
ENV_PREFIX=$CONDA/envs/opencda310
MTR_CODE_TAR=$PROJECT/mtr_code.tar
DATA_TAR=$PROJECT/wf_mtr_translaug.tar
CFG1_REL=tools/cfgs/multiv2x/multiv2x_multiego_worldfusion_no_agg.yaml
CFG2_REL=tools/cfgs/multiv2x/multiv2x_multiego_worldfusion.yaml
RESULTS=$PROJECT/mtr_wf_runs/${SLURM_JOB_ID}

# Sibling of the 8-GPU job: yield if it is already running.
if squeue -h -u tlandle3 -n mtr-wf-8gpu -t R | grep -q .; then
  echo "8-GPU sibling already running; yielding."; exit 0
fi

LOCAL=${TMPDIR:-/tmp}/mtr_wf_$SLURM_JOB_ID
mkdir -p "$LOCAL" "$RESULTS"
echo "=== node $(hostname) | ${NGPUS} GPU | $(date) ==="
nvidia-smi -L

# Node NVMe is wiped at job end: always copy training output back to
# PROJECT, also on failure/timeout, so partial checkpoints survive.
copy_back() {
  echo "=== copying output back to $RESULTS at $(date) ==="
  # -T so repeated calls overwrite rather than nesting output/output
  [ -d "$LOCAL/MTR/output" ] && cp -rT "$LOCAL/MTR/output" "$RESULTS/output" || true
  cp "$LOCAL"/mtr_build.log "$RESULTS/" 2>/dev/null || true
}
trap copy_back EXIT

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

STAGE1_BEST="$MTR/output/multiv2x_multiego_worldfusion_no_agg/wf_stage1/ckpt/best_model.pth"
python - "$MTR/$CFG1_REL" "$MTR/$CFG2_REL" "$DATA_ROOT" "$STAGE1_BEST" <<'PY'
import sys, re
cfg1, cfg2, root, stage1_best = sys.argv[1:5]
for cfg in (cfg1, cfg2):
    s = open(cfg).read()
    s = re.sub(r"root_dir:\s*'[^']*'", f"root_dir: '{root}'", s)
    s = re.sub(r"INTENTION_POINTS_FILE:\s*'[^']*'",
               f"INTENTION_POINTS_FILE: '{root}/multiv2x_cluster_64_center_dict.pkl'", s)
    if cfg == cfg2:
        s = re.sub(r"PRETRAINED_MOTION_TRANSFORMER:\s*'[^']*'",
                   f"PRETRAINED_MOTION_TRANSFORMER: '{stage1_best}'", s)
    open(cfg, 'w').write(s)
print('configs patched')
PY

PORT=$(( (RANDOM % 20000) + 20000 ))
cd "$MTR"

echo "=== STAGE 1 (no aggregator): ${NGPUS} GPU, ${EPOCHS} epochs, port ${PORT} at $(date) ==="
torchrun --nproc_per_node=${NGPUS} --master_port=${PORT} \
  tools/train_multiego.py \
  --cfg_file "$CFG1_REL" \
  --batch_size ${NGPUS} \
  --epochs ${EPOCHS} \
  --workers 8 \
  --extra_tag wf_stage1 \
  --launcher pytorch
copy_back   # persist stage-1 checkpoints before stage 2 runs

# Fall back to the newest epoch checkpoint if no best_model was recorded.
if [ ! -f "$STAGE1_BEST" ]; then
  FALLBACK=$(ls -t "$MTR"/output/multiv2x_multiego_worldfusion_no_agg/wf_stage1/ckpt/checkpoint_epoch_*.pth 2>/dev/null | head -1 || true)
  [ -n "$FALLBACK" ] || { echo 'stage 1 produced no checkpoint'; exit 1; }
  echo "no best_model.pth; stage 2 will start from $FALLBACK"
  python - "$MTR/$CFG2_REL" "$FALLBACK" <<'PY'
import sys, re
cfg, ckpt = sys.argv[1], sys.argv[2]
s = open(cfg).read()
s = re.sub(r"PRETRAINED_MOTION_TRANSFORMER:\s*'[^']*'",
           f"PRETRAINED_MOTION_TRANSFORMER: '{ckpt}'", s)
open(cfg, 'w').write(s)
PY
fi

PORT=$(( (RANDOM % 20000) + 20000 ))
echo "=== STAGE 2 (Transformer aggregator): ${NGPUS} GPU, ${EPOCHS} epochs, port ${PORT} at $(date) ==="
torchrun --nproc_per_node=${NGPUS} --master_port=${PORT} \
  tools/train_multiego.py \
  --cfg_file "$CFG2_REL" \
  --batch_size ${NGPUS} \
  --epochs ${EPOCHS} \
  --workers 8 \
  --extra_tag wf_stage2 \
  --launcher pytorch

echo "=== DONE $(date) ==="
