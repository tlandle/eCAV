# eCAV

eCAV is a simulation platform for evaluating edge-assisted connected autonomous vehicles (CAVs). The primary research scenario: a camera at a blind intersection fuses perception data with an approaching vehicle's own sensors, allowing the vehicle to detect an obstacle it cannot see directly.

## Architecture

eCAV runs in two orthogonal dimensions of distribution:

**Actor distribution** — sequential or distributed:
- **Sequential**: all actors (vehicles, RSUs, edge) run in the main process. Simple, good for algorithm development.
- **Distributed** (`-d`): each actor runs in its own Docker container. Uses gRPC + a C++ orchestration server for inter-process coordination.

**Perception inference** — local or remote:
- **Local** (`--apply_ml`): perception models (YOLOv5, WorldFusion) load in-process.
- **gRPC server** (`-l`): feature extraction is offloaded to standalone servers. Avoids loading the same model per-actor. Requires starting the appropriate server(s) before running.

These are independent. You can run distributed actors with local inference, or sequential actors with gRPC inference.

### Process Topology

```
Sequential:
  ecav.py (main) ──► ScenarioRunner subprocess (spawns CARLA actors)
                  ──► Edge / VehicleManager / RSUManager (in-process)
                  ──► [optional] worldfusion_grpc_server.py (port 18002)
                  ──► [optional] yolo_grpc_server.py (port 18001)

Distributed:
  ecav.py (main) ──► ecloud_server (C++, gRPC orchestration)
                  ──► Edge / RSUManager (in-process)
                  ──► Docker: vehicle container (actor + ScenarioRunner)
                  ──► Docker: RSU container (actor + ScenarioRunner)
                  ──► [optional] worldfusion_grpc_server.py (port 18002)
                  ──► [optional] yolo_grpc_server.py (port 18001)
```

### Perception Backends

| Flag | Backend | What runs where |
|------|---------|-----------------|
| `--apply_ml` | Local | Model loads in main process |
| `--apply_ml -l` | gRPC server | Feature extraction in server process; main process is client |
| `-d` | Local (per container) | Each container loads its own model copy |
| `-d -l` | gRPC server | All containers share one server |

### Active Scenarios

| Scenario | Fusion | Notes |
|----------|--------|-------|
| `openscenario_3_edge_late_fusion` | Late (YOLO + AB3DMOT) | Ego detects Lincoln via RSU |
| `openscenario_3_edge_worldfusion` | Intermediate (WorldFusion) | Feature-level fusion via Where2comm |

The scenario name maps to matching `.py` and `.yaml` files under `ecav/scenario_testing/` and `ecav/scenario_testing/config_yaml/`. The YAML's `scenario_runner.scenario` field points to a CARLA scenario class (`.py` + `.xml`) under `ecav/scenario_testing/scenarios/`.

---

## Requirements

- Ubuntu 20.04 / 22.04
- CARLA 0.9.12+
- NVIDIA GPU (RTX 3080+ recommended)
- Docker + NVIDIA Container Toolkit (for distributed actor mode)
- Conda

---

## Setup

```bash
conda env create -f environment.yml   # or: conda create -n ecav310 python=3.10
conda activate ecav310
pip install -r requirements_3_10.txt
```

Compile gRPC stubs (required after editing `.proto` files, or on first setup):

```bash
python ecav.py --build
```

---

## Running Scenarios

### Sequential — local inference

All actors and models run in one process. No servers or containers needed.

```bash
# Late fusion
python ecav.py -t openscenario_3_edge_late_fusion --apply_ml

# WorldFusion (intermediate fusion)
python ecav.py -t openscenario_3_edge_worldfusion --apply_ml
```

### Sequential — gRPC inference server

Offloads model inference to a dedicated process. Start the server first, then run the scenario.

**Late fusion (YOLO, port 18001):**
```bash
# Terminal 1
python ecav/ml_manager/yolo_grpc_server.py

# Terminal 2
python ecav.py -t openscenario_3_edge_late_fusion -l
```

**WorldFusion (port 18002):**
```bash
# Terminal 1
python ecav/ml_manager/worldfusion_grpc_server.py

# Terminal 2
python ecav.py -t openscenario_3_edge_worldfusion -l
```

### Distributed actors

`start_actors.sh` handles the full lifecycle: CARLA, gRPC servers (if requested), Docker containers, and readiness synchronization. It prompts for configuration interactively.

```bash
bash start_actors.sh
```

The script will ask for:
- Scenario name
- Number of ego vehicles and RSUs
- Whether to use ML / WorldFusion gRPC / YOLO gRPC
- Whether to rebuild Docker containers
- Whether to start CARLA locally (headless or with display)

Once everything is running, start the orchestrator in a separate terminal:

```bash
python ecav.py -t openscenario_3_edge_late_fusion -d
# or with gRPC inference servers (start_actors.sh already started them):
python ecav.py -t openscenario_3_edge_worldfusion -d -l
```

To tear down:
```bash
bash stop_actors.sh
```

---

## Code Structure

```
ecav/                         # Active development target
  ecav2/                      # Actor client (runs inside containers)
    ecloud_actor_client.py    # gRPC client connecting actor to orchestrator
  ecloud_server/              # C++ gRPC orchestration server
  ml_manager/
    ml_manager.py             # Model loader (local or distributed mode)
    worldfusion_grpc_server.py  # Standalone WorldFusion feature server (port 18002)
    yolo_grpc_server.py         # Standalone YOLO detection server (port 18001)
    litserve_models.py          # LitServe model wrappers (legacy HTTP path)
  core/
    common/
      cav_world.py            # Shared simulation state
      vehicle_manager.py
      rsu_manager.py
    sensing/perception/
      worldfusion_perception_manager.py
      bm2cp_perception_manager.py
      perception_manager.py   # Base / YOLO path
    application/edge/
      edge_manager/           # Edge fusion logic (late fusion, WorldFusion)
    prediction/               # SMART trajectory predictor
  protos/
    ecloud.proto              # Orchestration RPC definitions
    perception.proto          # Perception RPC definitions (YOLO, WorldFusion)
  scenario_testing/
    openscenario_3_edge_late_fusion.py
    openscenario_3_edge_worldfusion.py
    config_yaml/              # Scenario YAML configs
    scenarios/                # CARLA scenario classes + XML trigger files
    evaluations/              # Evaluation managers and metrics

opencda/                      # Legacy upstream code (mostly superseded)
scenario_runner/              # CARLA ScenarioRunner (git submodule)
ecav.py                       # Main entry point
start_actors.sh               # Distributed launch script
stop_actors.sh                # Distributed teardown script
```

The `ecav/` directory is the active development target. Before editing anything in `opencda/`, check whether an `ecav/` counterpart exists.

---

## gRPC Stubs

Proto files live in `ecav/protos/`. Generated stubs (`*_pb2.py`, `*_pb2_grpc.py`) live alongside them and are added to `sys.path` at startup. Recompile after any `.proto` change:

```bash
python ecav.py --build
```

---

## Supported Models

### Cooperative Perception
- [x] [CMP (CoBEVT + MTR) [RA-L2025]](https://ieeexplore.ieee.org/document/10908648) - V2V cooperative perception and prediction (32KB payload, 256x compression)
- [x] [BM2CP [CoRL2023]](https://proceedings.mlr.press/v229/zhao23a.html) - V2V multi-modal intermediate fusion (~200KB payload)
- [x] [AutoCast [MobiSys2022]](https://dl.acm.org/doi/10.1145/3498361.3538925) - V2V early fusion with MCKP scheduling (~10-50KB payload)
- [x] Late Fusion (YOLO + LiDAR) - Edge-assisted detected object fusion (~5KB payload)
- [x] WorldFusion - Edge-assisted BEV feature fusion with Where2Comm attention (eCAV, ~17KB payload)
- [ ] [Where2comm [NeurIPS2022]](https://proceedings.neurips.cc/paper_files/paper/2022/hash/1f5c5cd01b864d53cc5fa0a3472e152e-Abstract-Conference.html)
- [ ] [V2X-ViT [ECCV2022]](https://link.springer.com/chapter/10.1007/978-3-031-19842-7_7)
- [ ] [EMP [MobiCom2021]](https://dl.acm.org/doi/10.1145/3447993.3483242)

### OPV2V 3D Detection (100ms delay)

| Method | Compression | Payload | AP@0.5 | AP@0.7 | Weights |
|--------|-------------|---------|--------|--------|---------|
| No Cooperation | N/A | 0 | 0.79 | 0.65 | [download](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/point_pillar_sinbevt/) |
| [V2VNet [ECCV2020]](https://link.springer.com/chapter/10.1007/978-3-030-58536-5_36) | None | 82.5 MB/s | 0.83 | 0.66 | [download](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/point_pillar_v2vnet_multiego/) |
| [CoBEVT/CMP [RA-L2025]](https://ieeexplore.ieee.org/document/10908648) | 256x | **0.32 MB/s** | **0.92** | **0.82** | [download](ecav/core/application/v2v/baselines/cmp/CMP/pretrained/opv2v/corpbevtlidar_delay_1_frame_aug_c256/) |
| WorldFusion (Multi-V2X) | 256x | ~17KB/msg | 0.627 | - | ecav/ml_manager/models/worldfusion_multiv2x_caronly_ndm/ |

### OPV2V Cooperative Motion Prediction

| Method | Perception Frontend | Cooperation | minADE@5s | minFDE@5s |
|--------|-------------------|-------------|-----------|-----------|
| No Cooperation | SinBEVT | None | 2.2217 | 5.1853 |
| [CMP [RA-L2025]](https://ieeexplore.ieee.org/document/10908648) | [V2VNet](https://link.springer.com/chapter/10.1007/978-3-030-58536-5_36) | Perception + Prediction | 2.1174 | 4.9037 |
| **[CMP [RA-L2025]](https://ieeexplore.ieee.org/document/10908648)** | **CoBEVT (256x)** | **Perception + Prediction** | **1.8578** | **4.1628** |

### Tracking
- [x] AB3DMOT (vectorized Kalman, 1.8x speedup over original)
- [x] MambaTrack3D (Mamba-based 3D MOT with trained weights)

### Trajectory Prediction
- [x] MTR (Motion Transformer, edge single-call mode, 57ms @ 25 objects)
- [x] SMART (NeurIPS 2024, 143ms constant, Waymo-trained)
- [x] Linear KF predictor (fallback)

---

## Attributions

- [OpenCDA](https://github.com/ucla-mobility/OpenCDA) — baseline coordination and planning
- [OpenCOOD](https://github.com/ucla-mobility/OpenCOOD) — collaborative perception (BM2CP, WorldFusion)
- [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT) — 3D multi-object tracking
- [SMART](https://github.com/rainmaker22/SMART) — trajectory prediction
- [CARLA](https://github.com/carla-simulator/carla) — physics and sensor simulation

## Citation

```bibtex
@misc{landle2025ecav,
  title={eCAV: An Edge-Assisted Evaluation Platform for Connected Autonomous Vehicles},
  author={Tyler Landle and others},
  year={2025},
  eprint={2506.16535},
  archivePrefix={arXiv},
  primaryClass={cs.RO},
  url={https://arxiv.org/abs/2506.16535},
}
```
