# Hardcoded Absolute Path Remediation

This document tracks known hardcoded absolute paths in the repository that break upon sync to a new machine, and the plan to resolve them. It is the output of an audit conducted 2026-03-23.

---

## Step 2: `local.env` Pattern for Runtime Configuration

### Problem

Seven tracked files hardcode the CARLA installation path and/or a specific user's conda prefix:

| File | Hardcoded Path |
|------|---------------|
| `start_actors.sh` | `/opt/carla-simulator` (8×), `/home/jordan/anaconda3` (2×) |
| `generate_spawn_positions.sh` | `/opt/carla-simulator` |
| `ecav.py` | `/opt/carla-simulator/PythonAPI/carla` |
| `opencda.py` | `/opt/carla-simulator/PythonAPI/carla` |
| `ecav/ecav2/edge_process.py` | `/opt/carla-simulator/PythonAPI/carla` |
| `ecav/ecav2/ecloud_actor_client.py` | `/opt/carla-simulator/PythonAPI/carla` |
| `ecav/distributed_client/distributed_actor_client.py` | `/opt/carla-simulator/PythonAPI/carla` |

### Plan

Introduce two files:

**`local.env.template`** — tracked in git; documents all required variables:
```bash
# Copy this file to local.env and fill in values for your machine.
# local.env is gitignored and must not be committed.

CARLA_HOME=/opt/carla-simulator
CONDA_HOME=/home/$USER/anaconda3
ECAV_HOME=/home/$USER/eCAV
```

**`local.env`** — gitignored (already excluded via `.env` and explicit `local.env` entries in `.gitignore`); populated by each developer after cloning.

**Shell scripts**: update to source `local.env` at the top, substituting `$CARLA_HOME` for `/opt/carla-simulator` and `$CONDA_HOME` for the conda prefix. Provide a fallback to the current default for backward compatibility:
```bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$SCRIPT_DIR/local.env" ] && source "$SCRIPT_DIR/local.env"
CARLA_HOME="${CARLA_HOME:-/opt/carla-simulator}"
CONDA_HOME="${CONDA_HOME:-/home/$USER/anaconda3}"
```

**Python entry points (`ecav.py`, `opencda.py`, and the distributed client files)**: resolve the CARLA PythonAPI path via environment variable with fallback:
```python
import os
CARLA_HOME = os.environ.get("CARLA_HOME", "/opt/carla-simulator")
carla_path = os.path.join(CARLA_HOME, "PythonAPI", "carla")
sys.path.append(carla_path)
```

Optionally, add a `scripts/setup_local_env.sh` that auto-generates `local.env` from the template by substituting the current user's detected conda prefix and checking for a CARLA installation at common paths (`/opt/carla-simulator`, `~/carla`).

---

## Step 3: Analysis Scripts in `scripts/`

### Problem

The following scripts were committed with hardcoded absolute paths from specific developers' machines (atlas). They are not usable by any other contributor without manual path editing:

| File | Hardcoded Paths |
|------|----------------|
| `scripts/sweep_commands.sh` | Atlas project root |
| `scripts/extract_metrics.py` | Specific experiment result directories |
| `scripts/reclassify_sweep.py` | Specific evaluation output directories |
| `scripts/paper1_real_aligned_plots.py` | Base evaluation output directory |
| `scripts/paper1_real_data_plots.py` | Project root (in comment) |
| `scripts/analyze_multi_ego_sweep.py` | Experiment and output directories |
| `debug_model_load.py` (repo root) | Full path to a model file on atlas's machine |

### Plan

**Option A (preferred if scripts are reusable tools):** Replace all hardcoded paths with `argparse` arguments. Example for `extract_metrics.py`:
```python
parser = argparse.ArgumentParser()
parser.add_argument("--results-dir", required=True)
parser.add_argument("--output-dir", required=True)
args = parser.parse_args()
```

**Option B (if scripts are one-off paper artifacts):** Move to a `scripts/scratch/` subdirectory that is gitignored, preventing further accumulation. The `paper1_*` naming convention suggests these are paper-specific and do not belong in the main repository.

`debug_model_load.py` in the repo root is unambiguously a one-off debugging script and should be deleted or moved to `scripts/scratch/`.

---

## Step 4: `ecav/core/application/edge/` Files

### Problem

Three files in the edge application layer contain hardcoded paths from specific developers' machines that affect load-bearing logic (not just analysis output):

| File | Owner | Hardcoded Paths |
|------|-------|----------------|
| `ecav/core/application/edge/collab_sandbox.py` | chandramouli | Output directories for clustering/greedy comparison (lines 456, 475–478, 513, 534–537) |
| `ecav/core/application/edge/a_star_algorithm/collab_sandbox.py` | chandramouli | Same output directories |
| `ecav/core/application/edge/a_star_algorithm/astar_edge_manager.py` | chattsgpu | Full project root path (line 33) |

### Plan

- **`astar_edge_manager.py`**: Replace the hardcoded project root with a path resolved relative to the file itself using `pathlib`:
  ```python
  from pathlib import Path
  PROJECT_ROOT = Path(__file__).resolve().parents[4]  # adjust depth as needed
  ```

- **`collab_sandbox.py` (both copies)**: Replace hardcoded output directories with `argparse` arguments or construct paths relative to a `--output-dir` argument. If these files are experimental/non-production, consider whether they belong in `scripts/scratch/` instead of `ecav/core/`.

---

## Step 5: `python_3_10_complete.yml` Conda Prefix

### Problem

`python_3_10_complete.yml` contains:
```yaml
prefix: /home/jordan/anaconda3/envs/opencda
```

This line is auto-generated by `conda env export` and records the exporting user's conda prefix. While `conda env create` ignores the `prefix:` field when recreating the environment, it is misleading to other contributors and pollutes diffs when the environment is re-exported.

### Plan

Strip the `prefix:` line before committing any future conda environment export:
```bash
conda env export --no-builds | grep -v "^prefix:" > python_3_10_complete.yml
```

As a one-time fix, remove the existing `prefix:` line from `python_3_10_complete.yml`.

Consider adding a note to the repo `README` or onboarding docs that environment exports must be stripped of the `prefix:` line before committing.

---

## Known Non-Actionable Issues (Submodule Upstream Code)

The following files in git submodules contain hardcoded paths inherited from upstream projects. Modifying them creates divergence from upstream and is not recommended unless the submodule is already forked:

| File | Hardcoded Path |
|------|---------------|
| `ecav/worldfusion/opencood/visualization/vis_data_sequence_dairv2x.py` | `/home/test_vis_result/` |
| `ecav/worldfusion/opencood/visualization/vis_data_sequence.py` | `/home/data_vis/v2x_2.0_new/train` |
| `ecav/BM2CP/opencood/visualization/vis_data_sequence_dairv2x.py` | `/home/test_vis_result/` |
| `ecav/BM2CP/opencood/visualization/vis_data_sequence.py` | `/home/data_vis/v2x_2.0_new/train` |

These files are visualization utilities not exercised in the simulation path. No action required unless visualization workflows are needed.
