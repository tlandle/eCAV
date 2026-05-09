#!/usr/bin/env bash
# Setup script for Multi-V2X evaluation.
#
# Clones RadetzkyLi's OpenCOOD fork to HDD and installs it in the ecav
# conda env. The fork has the Multi-V2X database manager, PR config
# loader, and key_objects machinery that our in-repo
# ecav/worldfusion/opencood lacks. We keep the two opencood checkouts
# separate: our in-repo copy is frozen for WorldFusion training, and
# the RadetzkyLi fork is used only for Multi-V2X loading and when
# fine-tuning WorldFusion on Multi-V2X.
#
# Usage:
#   bash paper2_multiv2x_setup.sh
#
# After setup, export the env var (add to your shell rc):
#   export MULTIV2X_OPENCOOD_ROOT=/media/atlas/HDD/Repos/Multi-V2X-OpenCOOD

set -euo pipefail

OPENCOOD_HOME="${MULTIV2X_OPENCOOD_ROOT:-/media/atlas/HDD/Repos/Multi-V2X-OpenCOOD}"
MULTIV2X_HOME="${MULTIV2X_HOME:-/media/atlas/HDD/Datasets/Multi-V2X}"

echo "=== Multi-V2X setup ==="
echo "  OpenCOOD fork:     $OPENCOOD_HOME"
echo "  Dataset root:      $MULTIV2X_HOME"
echo ""

# 1. Clone or update the RadetzkyLi OpenCOOD fork
mkdir -p "$(dirname "$OPENCOOD_HOME")"
if [[ ! -d "$OPENCOOD_HOME/.git" ]]; then
    echo "[1/4] Cloning RadetzkyLi/OpenCOOD to $OPENCOOD_HOME"
    git clone https://github.com/RadetzkyLi/OpenCOOD "$OPENCOOD_HOME"
else
    echo "[1/4] RadetzkyLi/OpenCOOD already present at $OPENCOOD_HOME"
    git -C "$OPENCOOD_HOME" fetch --all --quiet
fi

# 2. Install opencood in the ecav conda env (develop mode so edits apply)
echo ""
echo "[2/4] Installing RadetzkyLi OpenCOOD in develop mode"
echo "    (requires the ecav conda env to be active)"
if [[ -z "${CONDA_DEFAULT_ENV:-}" ]] || [[ "$CONDA_DEFAULT_ENV" != "ecav" ]]; then
    echo "    WARNING: ecav env is not active. Run:"
    echo "      conda activate ecav && bash paper2_multiv2x_setup.sh"
    exit 1
fi
pushd "$OPENCOOD_HOME" > /dev/null
python setup.py develop --quiet || {
    echo "    setup.py develop failed; trying pip install -e ."
    pip install -e . --quiet
}
popd > /dev/null

# 3. Verify dataset root
echo ""
if [[ -d "$MULTIV2X_HOME" ]]; then
    echo "[3/4] Multi-V2X dataset root exists: $MULTIV2X_HOME"
    ls "$MULTIV2X_HOME" | head -5
else
    echo "[3/4] Dataset root $MULTIV2X_HOME not yet present."
    echo "      Download from https://opendatalab.com/Rongsong/Multi-V2X"
fi

# 4. Export hint
echo ""
echo "[4/4] Add to your shell rc:"
echo "    export MULTIV2X_OPENCOOD_ROOT=$OPENCOOD_HOME"
echo "    export MULTIV2X_HOME=$MULTIV2X_HOME"
echo ""
echo "Done."
