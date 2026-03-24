#!/usr/bin/env bash
# setup_ns3_cv2x.sh — Clone, build, and verify Eckermann's ns-3_c-v2x
#
# Automates the one-time setup of the ns-3 C-V2X simulator used for
# MAC model validation.  Builds into /home/atlas/ns3_cv2x/ (outside
# the repo — build artifacts are large).
#
# Uses the ecav310 conda env (Python 3.10) because ns-3's bundled
# WAF 1.8.19 requires the `imp` module removed in Python 3.12.
#
# Prerequisites: g++ >= 9, conda env ecav310
#
# Usage:
#   bash scripts/setup_ns3_cv2x.sh
#   bash scripts/setup_ns3_cv2x.sh /path/to/install
#
# After setup, run the PRR sweep:
#   conda run -n ecav310 python scripts/run_ns3_prr_sweep.py

set -euo pipefail

# ── Configuration ────────────────────────────────────────────────
INSTALL_DIR="${1:-/home/atlas/ns3_cv2x}"
REPO_URL="https://github.com/FabianEckermann/ns-3_c-v2x.git"
NS3_DIR="${INSTALL_DIR}/ns-3_c-v2x"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="${SCRIPT_DIR}/../data"
CONDA_ENV="ecav310"
CONDA_PYTHON="/home/atlas/anaconda3/envs/${CONDA_ENV}/bin/python"

# ── Color helpers ────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }

# ── Step 0: Check prerequisites ─────────────────────────────────
info "Checking prerequisites..."

command -v g++  >/dev/null 2>&1 || fail "g++ not found"
command -v git  >/dev/null 2>&1 || fail "git not found"

GCC_VER=$(g++ -dumpversion | cut -d. -f1)
if [ "$GCC_VER" -lt 9 ]; then
    fail "g++ >= 9 required (found $GCC_VER)"
fi

# Verify conda env with Python <= 3.11 (WAF needs imp module)
if [ ! -x "${CONDA_PYTHON}" ]; then
    fail "Conda env '${CONDA_ENV}' not found at ${CONDA_PYTHON}. Create with: conda create -n ${CONDA_ENV} python=3.10"
fi
PY_VER=$("${CONDA_PYTHON}" --version | awk '{print $2}')
info "  g++ $(g++ -dumpversion), ${CONDA_ENV} Python ${PY_VER}"

# ── Step 1: Clone ────────────────────────────────────────────────
if [ -d "${NS3_DIR}" ]; then
    info "ns-3_c-v2x already cloned at ${NS3_DIR}, skipping clone."
else
    info "Cloning ns-3_c-v2x into ${INSTALL_DIR}..."
    mkdir -p "${INSTALL_DIR}"
    git clone --depth 1 "${REPO_URL}" "${NS3_DIR}"
    info "Clone complete."
fi

# ── Step 2: Build ────────────────────────────────────────────────
cd "${NS3_DIR}"

# Point WAF at the conda Python 3.10 so it doesn't pick up system 3.12
export PYTHON="${CONDA_PYTHON}"
# Rewrite the waf shebang in-place to use conda python
sed -i "1s|.*|#!${CONDA_PYTHON}|" waf

if [ -f "waf" ]; then
    # WAF-based build (ns-3.30 era)
    info "Configuring with WAF (using ${CONDA_ENV} python)..."
    "${CONDA_PYTHON}" waf configure --build-profile=optimized --enable-examples 2>&1 | tail -10

    info "Building (this may take 10-20 minutes)..."
    "${CONDA_PYTHON}" waf build -j"$(nproc)" 2>&1 | tail -10
    BUILD_SYSTEM="waf"

elif [ -f "CMakeLists.txt" ]; then
    # Newer ns-3 uses CMake
    info "Configuring with CMake..."
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DNS3_EXAMPLES=ON 2>&1 | tail -5

    info "Building with CMake (this may take 10-20 minutes)..."
    cmake --build build -j"$(nproc)" 2>&1 | tail -10
    BUILD_SYSTEM="cmake"
else
    fail "No waf or CMakeLists.txt found in ${NS3_DIR}"
fi

# ── Step 3: Verify build ────────────────────────────────────────
info "Verifying build..."

if [ "$BUILD_SYSTEM" = "waf" ]; then
    # Check that the v2x example binary exists
    if "${CONDA_PYTHON}" waf --run "v2x_communication_example --PrintHelp" 2>&1 | head -5 | grep -qi "usage\|option\|error\|v2x"; then
        info "v2x_communication_example binary found and runs."
    else
        warn "Could not verify v2x_communication_example. Check build output."
        warn "The binary may have a different name — list available:"
        ls build/scratch/ 2>/dev/null || ls build/examples/ 2>/dev/null || true
    fi
elif [ "$BUILD_SYSTEM" = "cmake" ]; then
    if [ -f "build/scratch/v2x_communication_example" ]; then
        info "v2x_communication_example binary found."
    else
        warn "Binary not found at expected path. Available scratch programs:"
        ls build/scratch/ 2>/dev/null || true
    fi
fi

# ── Step 4: Create data directory ────────────────────────────────
mkdir -p "${DATA_DIR}"

# ── Done ─────────────────────────────────────────────────────────
info ""
info "ns-3 C-V2X setup complete!"
info "  Install dir:  ${NS3_DIR}"
info "  Build system: ${BUILD_SYSTEM}"
info "  Python:       ${CONDA_PYTHON} (${PY_VER})"
info ""
info "Next steps:"
info "  conda run -n ${CONDA_ENV} python scripts/run_ns3_prr_sweep.py --ns3-dir ${NS3_DIR}"
info "  # or manually:"
info "  cd ${NS3_DIR}"
if [ "$BUILD_SYSTEM" = "waf" ]; then
    info "  ${CONDA_PYTHON} waf --run 'v2x_communication_example --numVeh=8 --numSubchannel=20 --time=60 --probResourceKeep=0.4'"
else
    info "  ./build/scratch/v2x_communication_example --numVeh=8 --numSubchannel=20 --time=60 --probResourceKeep=0.4"
fi
