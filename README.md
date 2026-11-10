# eCAV

eCAV is a simulation platform for evaluating edge-assisted connected autonomous vehicles (CAVs). Vehicles, roadside units (RSUs), and an edge service run as separate actors against a CARLA world. An orchestration server steps every actor in lock step, so a run is deterministic in world evolution and message order while compute and network delay are measured or modeled per message.

The representative scenario is an occluded conflict. A vehicle approaches a junction where a crossing vehicle is hidden. RSUs observe the crossing lane, the edge fuses their observations with the vehicle's, and the vehicle receives trajectory forecasts in time to brake.

Last reviewed 2026-11-10. Scenario status is in [docs/scenario_status.md](docs/scenario_status.md).

## What the platform supports

| Capability | Options | Selected by |
|---|---|---|
| Actor execution | Sequential (one process) or distributed (one process or container per actor, gRPC to a C++ orchestration server) | `-d` |
| Perception inference | In process, or a shared inference server | `--apply_ml`, `-l` |
| State model | Ground-truth state (`oracle`) or sensor-derived state (every other manager) | `manager_type` in the scenario YAML |
| Edge service | Late fusion, intermediate (feature) fusion, infrastructure only, cooperative planning, multi-edge handoff | `manager_type` |
| Delivery path | Injected delay and loss, latency tables from ns-3 5G-LENA, live ns-3 co-simulation | `test_runner.py` flags and the YAML latency block |
| Sweeps | Speed, delay, loss, vehicle count, seeds, arbitrary config keys | `test_runner.py` |

### Edge manager types

`scenario.edge_list[].manager_type` selects the edge pipeline. The registry is `ecav/core/application/edge/edge_manager/__init__.py`. Names are case insensitive and an unknown name raises `KeyError`. A missing key means `late_fusion`.

| `manager_type` | Pipeline |
|---|---|
| `late_fusion` | YOLO and LiDAR detections from each agent, fused at the edge, AB3DMOT tracking, linear prediction |
| `late_fusion_etsi_self_announcement` | Late fusion with vehicle self-announcement for identity |
| `oracle` | Ground-truth state through the same tracker, predictor, and delivery path. The exact-state baseline |
| `infra_only` | RSU observations only |
| `cip` | Cooperative planning. The edge returns the trajectory the vehicle follows |
| `perception` | Shared perception without prediction |
| `worldfusion`, `worldfusion_pred` | Intermediate BEV feature fusion (WorldFusion), AB3DMOT, linear prediction |
| `worldfusion_mtr` | WorldFusion, AB3DMOT, MTR prediction |
| `worldfusion_adaptive` | WorldFusion, AB3DMOT, MTR, with the deadline-aware controller |
| `worldfusion_mamba_adaptive`, `worldfusion_mamba_mtr` | WorldFusion, MambaTrack, MTR. Used by the multi-locale handoff scenarios |
| `sota`, `sota_edge` | Pluggable edge with the SMART predictor |
| `adaptive`, `vips`, `vips_pred`, `vips_temporal`, `maneuver` | Registered. Not exercised in 2026 |

`bm2cp`, `harbor`, `v2v_coop`, `v2v_autocast`, and `cmp` appear in older YAMLs and are not registered on `develop`. Scenarios that use them fail at edge creation.

## Requirements

- Ubuntu 20.04 or 22.04
- CARLA 0.9.15
- NVIDIA GPU. A 16 GB card runs one to four full-sensor vehicles with an edge service
- Python 3.10 environment from `requirements_3_10.txt`. On the project hosts the conda environment is named `opencda310`
- Docker and the NVIDIA Container Toolkit, only for containerized distributed actors

## Setup

```bash
conda create -n opencda310 python=3.10
conda activate opencda310
pip install -r requirements_3_10.txt
python ecav.py --build        # generate the gRPC stubs, once per checkout and after any .proto change
make -C ecav                  # build the C++ orchestration server, needed for -d
```

The generated stubs are not in git. A fresh checkout fails with `No module named 'ecloud_pb2'` until `--build` has run.

## How a scenario is resolved

`python ecav.py -t X` needs two files:

- `ecav/scenario_testing/X.py`, the runner, with `run_scenario` and, for distributed mode, `run_vehicle`
- `ecav/scenario_testing/config_yaml/X.yaml`, merged over `config_yaml/default.yaml`

The YAML must contain a `scenario_runner` section. Its `configFile` (.xml) and `additionalScenario` (.py) name a CARLA scenario under `ecav/scenario_testing/scenarios/`, as paths relative to the repository root, and `scenario` must match the `name` in the XML. A YAML without this section fails at startup. That is the reason most pre-2026 scenarios no longer run.

## Running scenarios

### One run, sequential

Start CARLA, then:

```bash
python ecav.py -t openscenario_3_edge_late_fusion --apply_ml
python ecav.py -t openscenario_3_edge_worldfusion --apply_ml
```

### Sweeps with the test runner

`test_runner.py` is how every 2026 campaign was run. It starts CARLA itself, patches the scenario YAML for each cell, runs `ecav.py`, and restores the YAML at the end.

```bash
python test_runner.py -t highway_merge_edge_late_fusion \
    --speeds 97 --latency-distribution hybrid --per-tick-age-log \
    --edge-kv prediction_steps=60 --seed 1 --stamp my_sweep
```

- Results go to `experiment_results/<scenario>/<stamp>/...`, one directory per run.
- `--manager-types` overrides `manager_type` per run, so one scenario serves several arms.
- `--ego-counts N` selects the `_<N>ego` variant of the scenario.
- `--edge-kv k=v` and `--behavior-kv k=v` set arbitrary edge and planner keys. `--scenario-env K=V` passes environment variables to the CARLA scenario.
- If a sweep is killed, the patched YAML stays in place. Restore `config_yaml/<scenario>.yaml.bak` and compare with `git diff` before the next run.
- Reference drivers: `scripts/sensys_highway_merge.sh`, `scripts/sweep_commands.sh`.

### Distributed actors

In distributed mode the orchestrator and each actor are separate processes:

```bash
python ecav.py -t <scenario> -d            # orchestrator, starts the orchestration server
python ecav.py -d -i 0                     # actor 0, fetches the scenario from the server
python ecav.py -d -i 1                     # actor 1, and so on
```

Actors find the orchestration server and CARLA through `cloud_config.yaml` (`ecloud_server_public_ip`, `carla_server_public_ip`, `vehicle_client_public_ip`). Set these to real addresses for a multi-host run. They are `localhost` in git.

`start_actors.sh <scenario> --auto` automates the single-host containerized case: CARLA, inference servers, and one container per vehicle, RSU, and edge. Two warnings:

- `start_actors.sh` and `stop_actors.sh` stop and remove every Docker container on the host and kill every CARLA process, not only the ones they started. Do not run them on a host with other work.
- `start_actors.sh` defaults to a conda environment named `ecav310`. Set `CONDA_ENV=opencda310` on the project hosts.

Multi-host scaling (hundreds of vehicle clients across machines) was last run with the 2023 `ecloud_4lane_*` scenarios, which no longer start. The replacement is tracked in `docs/agent_plans/ispass_scaleout_run.md`.

### Inference servers

```bash
python ecav/ml_manager/yolo_grpc_server.py          # YOLO detection, port 18001
python ecav/ml_manager/worldfusion_grpc_server.py   # WorldFusion features, port 18002
python ecav.py -t openscenario_3_edge_late_fusion -l
```

## Scenario families

Status has three levels. Supported means the family ran in a 2026 campaign. Unverified means the scenario passes static checks (files exist, YAML loads, manager registered) and has not been run recently. Deprecated means the scenario fails static checks and does not start.

### Supported

| Family | Scenarios | Map | What it tests |
|---|---|---|---|
| Occluded left turn | `openscenario_3_edge_late_fusion`, `_2ego`, `_4ego`, `_8ego`, `_mixed_4ego`, `_boundary`, smoke variants | Town03 | A connected vehicle turns left across a crossing vehicle hidden by parked cars. Two RSUs. The main harness for delivered-age sweeps, identity, and the edge architectures (`late_fusion`, `oracle`, `infra_only`, `cip`, `perception`) |
| Occluded straight crossing | `scp_edge_late_fusion`, `_2ego`, `_4ego`, `_8ego`, `_mixed_4ego` | Town03 | The same junction with the connected vehicle going straight |
| Blind overtake | `blind_overtake_edge_late_fusion`, `_2ego`, `_mixed_2ego` | Town01 | Overtaking a stopped truck with oncoming traffic occluded. Three RSUs |
| Highway cut-in | `highway_merge_edge_late_fusion` | Town06 | A slower vehicle cuts in at 97 km/h. Three RSUs. Scenario knobs through `--scenario-env`. Under validation as of 2026-11-10 |
| Multi-locale handoff | `openscenario_1_flow_gt`, `_flow_single`, `_flow_nohandoff_gt`, `_flow_visible_gt`, `_accel_gt`, `_burst_gt`, `_corridor_gt` | Town01 | Several edge services in one simulation. Track state moves between edges ahead of a road user's crossing (`worldfusion_mamba_adaptive`, ground-truth detection injection) |
| Intermediate fusion at the left turn | `openscenario_3_edge_worldfusion`, `_smoke`, `_4ego` | Town03 | WorldFusion BEV feature fusion at the edge, with linear or MTR prediction |
| Multi-edge | `openscenario_3_multi_edge_late_fusion`, `openscenario_multi_edge_right_merge` | Town03, Town06 | Two edges with sequential handoff |

The multi-vehicle variants (`_2ego`, `_4ego`, `_8ego`, `_mixed_*`) are generated by `scripts/generate_multi_ego_scenario.py`.

### Unverified

These pass static checks and have no run on record since they were added: `openscenario_20` to `openscenario_23` intersection variants (dense occlusion, blind-side approach, dense convergence, stale prediction), `openscenario_3_sota_*`, `openscenario_3_edge_route`, `openscenario_3_edge_worldfusion_mtr_live`, `openscenario_1_accel_visible_gt`, `openscenario_1_accel_worldfusion`, `openscenario_1_two_locale_worldfusion`, `openscenario_3_multi_edge_mamba`, `openscenario_multi_edge_right_merge_worldfusion`, the `_16ego` variants, `openscenario_3` (single vehicle, no edge), and `mv2x_datadump_town01`.

### Deprecated

84 of the 144 scenarios do not start on `develop`. The files stay in the tree for reference. Do not base new work on them.

| Family | Count | Why it fails |
|---|---|---|
| eCloud four-lane scaling, `ecloud_4lane_*` | 23 | No `scenario_runner` section, runner signature predates the current entry point, two YAMLs do not parse |
| eCloud legacy edge, `ecloud_edge_*` | 12 | No `scenario_runner` section, old `members:` schema |
| OpenCDA demos, `single_*`, `platoon_*`, `multi_2lanefree_carla`, `v2xp_datadump_town06_carla` | 12 | No `scenario_runner` section. The co-simulation ones also need SUMO |
| Legacy `openscenario_N` (1, 2, 4 to 9, 11 to 18, `_9_edge`, `_13_edge`, `_1_e6b_ego_local`) and the fog, rain, and light variants | 19 | Absolute paths to scenario files on machines that no longer exist |
| V2V baselines, `openscenario_3_v2v_*`, `openscenario_3_harbor`, `openscenario_3_edge` | 19 | `manager_type` (`v2v_coop`, `v2v_autocast`, `cmp`, `harbor`, `bm2cp`) not registered on `develop` |

The per-scenario list with the specific reason is in [docs/scenario_status.md](docs/scenario_status.md).

## Deployment tooling status

| Tool | Status |
|---|---|
| `test_runner.py` | Current. Used by every 2026 campaign |
| `start_actors.sh`, `stop_actors.sh` | Work for single-host containers. Host-wide container and process cleanup, see the warnings above |
| `Dockerfile` (root) | Current image for actor containers. No entry point, the launcher passes the command |
| `cloud_config.yaml` | Current. Holds host addresses and per-module log levels |
| `ansible/` | Out of date. Targets a 2023 image name and branch, and its vault file is missing. Do not use without rework |
| `docker/Dockerfile.ecav`, `docker/Dockerfile.torchserve` | Out of date. Python 3.7 image, and a copy of a directory that no longer exists |
| `opencda.py` | Dead. The `opencda/` directory is gone and all code is under `ecav/` |

## Code structure

```
ecav/
  ecav2/                      # Actor client and edge process for distributed mode
  ecloud_server/              # C++ gRPC orchestration server
  ml_manager/                 # Model loading, YOLO and WorldFusion inference servers
  core/
    common/                   # cav_world, vehicle_manager, rsu_manager
    sensing/perception/       # Perception managers
    application/edge/         # Edge managers, latency tables, tracking, prediction glue
    plan/, actuation/         # Behavior planner and controllers
  protos/                     # ecloud.proto, perception.proto
  scenario_testing/
    <scenario>.py             # Runners
    config_yaml/              # Scenario YAMLs, default.yaml
    scenarios/                # CARLA scenario classes and XML
    evaluations/              # Evaluation managers and metrics
scenario_runner/              # CARLA ScenarioRunner
scripts/                      # Campaign drivers and analysis
docs/                         # Plans (agent_plans) and the knowledge base (kb)
ecav.py                       # Entry point
test_runner.py                # Sweep harness
start_actors.sh, stop_actors.sh
```

---

## Supported Models

### Cooperative Perception
- [ ] [CMP (CoBEVT + MTR) [RA-L2025]](https://ieeexplore.ieee.org/document/10908648) - V2V cooperative perception and prediction (32KB payload, 256x compression). Code is in the tree; the manager type is not registered on `develop`.
- [ ] [BM2CP [CoRL2023]](https://proceedings.mlr.press/v229/zhao23a.html) - V2V multi-modal intermediate fusion (~200KB payload). Not registered on `develop`.
- [ ] [AutoCast [MobiSys2022]](https://dl.acm.org/doi/10.1145/3498361.3538925) - V2V early fusion with MCKP scheduling (~10-50KB payload). Not registered on `develop`.
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

- [OpenCDA](https://github.com/ucla-mobility/OpenCDA): baseline coordination and planning
- [OpenCOOD](https://github.com/ucla-mobility/OpenCOOD): collaborative perception (BM2CP, WorldFusion)
- [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT): 3D multi-object tracking
- [SMART](https://github.com/rainmaker22/SMART): trajectory prediction
- [CARLA](https://github.com/carla-simulator/carla): physics and sensor simulation

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
