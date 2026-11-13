#!/usr/bin/env bash
# Prepare the V2Xverse and AutoCastSim scale baselines on the benchmark host. Installs only:
# no simulator and no GPU process is started. Idempotent; each step is skipped when done.
#
#   scripts/ispass/setup_competitors.sh carla      # CARLA 0.9.10.1 and 0.9.11 under $CARLA_STORE
#   scripts/ispass/setup_competitors.sh v2xverse   # conda env v2xverse, packages, checkpoints
#   scripts/ispass/setup_competitors.sh autocast   # conda env autocast, packages, MQTT broker
#
# CARLA_STORE defaults to /mnt/competitors, the Azure temporary disk: a deallocation erases it,
# so the carla step must be rerun after the VM is started again (about 16 GB of downloads).
set -uo pipefail
STEP="${1:?carla|v2xverse|autocast}"
ROOT="${COMP_ROOT:-$HOME/competitors}"; STORE="${CARLA_STORE:-/mnt/competitors}"
CONDA="${CONDA:-$HOME/miniconda3/bin/conda}"; ENVS="$(dirname "$(dirname "$CONDA")")/envs"
REL=https://carla-releases.s3.us-east-005.backblazeb2.com/Linux
fetch_carla() {  # version tarball
  local d="$STORE/carla-$1"
  [[ -x "$d/CarlaUE4.sh" ]] && { echo "carla $1 present"; return; }
  mkdir -p "$d" && cd "$d" && wget -q -c "$REL/$2" && tar -xf "$2" && rm "$2" && echo "carla $1 extracted: $(du -sh "$d" | cut -f1)"
}
case "$STEP" in
  carla)
    [[ -d "$STORE" ]] || { sudo mkdir -p "$STORE" && sudo chown "$USER" "$STORE"; }
    fetch_carla 0.9.10.1 CARLA_0.9.10.1.tar.gz      # V2Xverse (Town05 is in the base package)
    fetch_carla 0.9.11 CARLA_0.9.11.tar.gz          # AutoCastSim (Town01 and Town03 are in the base package)
    mkdir -p "$ROOT/V2Xverse/external_paths"
    ln -sfn "$STORE/carla-0.9.10.1" "$ROOT/V2Xverse/external_paths/carla_root"
    ln -sfn "$STORE/carla-0.9.11" "$ROOT/AutoCastSim/carla_root"
    ;;
  v2xverse)
    cd "$ROOT/V2Xverse"
    [[ -d "$ENVS/v2xverse" ]] || "$CONDA" create -y -q -n v2xverse python=3.7
    PIP="$ENVS/v2xverse/bin/pip"; PY="$ENVS/v2xverse/bin/python"
    "$PIP" install -q torch==1.10.1+cu113 torchvision==0.11.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
    # The instructions build spconv 1.2.1 from source. The preprocessor also accepts spconv 2.x
    # (opencood/data_utils/pre_processor/sp_voxel_preprocessor.py), which has a binary wheel.
    "$PIP" install -q spconv-cu113
    "$PIP" install -q -r opencood/requirements.txt
    grep -v '^setuptools' simulation/requirements.txt > /tmp/v2xverse_sim_req.txt   # the setuptools pin is only for easy_install
    "$PIP" install -q -r /tmp/v2xverse_sim_req.txt
    "$PIP" install -q efficientnet_pytorch==0.7.0 python-lzf psutil huggingface_hub
    "$PIP" install -q timm==0.6.13 --no-deps          # newer timm needs a Rust build on Python 3.7
    "$CONDA" install -y -q -n v2xverse -c conda-forge "libtiff=4.4"   # libtiff.so.5 for the CARLA 0.9.10 client on Ubuntu 24.04
    "$PIP" install -q "git+https://github.com/klintan/pypcd.git"
    "$PIP" install -q -e . --no-deps
    "$PY" opencood/utils/setup.py build_ext --inplace
    "$PY" - <<'PYEOF'
from huggingface_hub import snapshot_download
snapshot_download("gjliu/v2xverse", local_dir="checkpoints", allow_patterns=["codriving/*", "no_collaboration/*", "late_fusion/*"])
PYEOF
    "$PY" -c "import torch, spconv, opencood, cv2, pygame; print('v2xverse env', torch.__version__, torch.cuda.is_available())"
    ;;
  autocast)
    cd "$ROOT/AutoCastSim"
    dpkg -s mosquitto >/dev/null 2>&1 || sudo apt-get install -y -q mosquitto libopenblas-dev
    sudo systemctl disable --now mosquitto 2>/dev/null   # the bench starts its own broker on its own port
    [[ -d "$ENVS/autocast" ]] || "$CONDA" create -y -q -n autocast python=3.7
    PIP="$ENVS/autocast/bin/pip"; PY="$ENVS/autocast/bin/python"
    "$PIP" install -q torch==1.7.1+cu110 torchvision==0.8.2+cu110 -f https://download.pytorch.org/whl/torch_stable.html
    "$PIP" install -q paho-mqtt==1.5.1 scipy pygame py-trees==0.8.3 networkx==2.2 xmlschema numpy shapely imageio tqdm numba pandas \
      scikit-image scikit-learn opencv-python h5py pydot decorator psutil ephem tabulate six dictor open3d
    # libtiff.so.5 for the CARLA 0.9.11 client; a compiler, crypt.h and OpenBLAS for MinkowskiEngine
    "$CONDA" install -y -q -n autocast -c conda-forge "libtiff=4.4" gxx_linux-64=9.5 ninja libxcrypt openblas
    # MinkowskiEngine, CPU version, as the AutoCastSim instructions ask (AVR/PCProcess.py imports it)
    if ! LD_LIBRARY_PATH="$ENVS/autocast/lib" "$PY" -c "import MinkowskiEngine" 2>/dev/null; then
      [[ -d "$ROOT/MinkowskiEngine" ]] || git clone -q --depth 1 --branch v0.5.4 https://github.com/NVIDIA/MinkowskiEngine.git "$ROOT/MinkowskiEngine"
      E="$ENVS/autocast"
      (cd "$ROOT/MinkowskiEngine" && PATH="$E/bin:$PATH" CC="$E/bin/x86_64-conda-linux-gnu-gcc" CXX="$E/bin/x86_64-conda-linux-gnu-g++" \
        CPATH="$E/include" MAX_COMPILATION_THREADS=8 LDSHARED="$E/bin/x86_64-conda-linux-gnu-g++ -pthread -shared -L$E/lib -Wl,-rpath=$E/lib" \
        "$PY" setup.py install --cpu_only --blas=openblas --blas_include_dirs="$E/include" --blas_library_dirs="$E/lib")
    fi
    LD_LIBRARY_PATH="$ENVS/autocast/lib" "$PY" -c "import torch, paho.mqtt.client, py_trees, numba, MinkowskiEngine; print('autocast env', torch.__version__, MinkowskiEngine.__version__)"
    ;;
  *) echo "unknown step"; exit 2 ;;
esac
echo "$(date '+%F %T') setup $STEP end"
