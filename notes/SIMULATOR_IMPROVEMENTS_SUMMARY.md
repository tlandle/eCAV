# eCloudSim Distributed Traffic Simulator: Comprehensive Improvements Summary (Nov 2025 - Feb 2026)

## Overview

eCloudSim is a CARLA-based cooperative autonomous driving simulator built on eCAV. Over the past 3 months, the simulator has been substantially extended from a monolithic single-process edge manager into a modular, distributed simulation platform supporting multiple cooperative perception backends, comprehensive metrics collection, and GPU-offloaded ML inference. The major areas of improvement are:

1. **Modular Edge Manager Architecture** (replacing a single 1864-line monolith)
2. **WorldFusion Cooperative Perception** (new end-to-end learned perception backend)
3. **Distributed Simulation Mode** (vehicle/RSU clients in containers via gRPC)
4. **LitServe ML Offloading** (GPU model inference decoupled from simulation)
5. **Comprehensive Metrics Architecture** (detection, tracking, prediction, planning, safety)
6. **VIPS Baseline** for infrastructure-only perception benchmarking
7. **Multi-Ego Vehicle and RSU Support** with perception enabled per agent

---

## 1. Modular Edge Manager Architecture

The original simulator had a single `edge_manager.py` monolith (1864 lines) handling all edge processing. This has been decomposed into a pluggable package (`ecav/core/application/edge/edge_manager/`) with a factory registry and a base class:

**Base Class** (`_BaseEdgeManager`): Shared functionality including:
- Vehicle and RSU manager registration
- Configurable latency simulation with multiple distribution models: `fixed`, `normal`, `lognormal`, and `hybrid` (C-V2X radio traces + log-normal backhaul)
- Configurable uplink/downlink packet loss percentages
- Shared `LinearPredictorManager` (constant-velocity, 25 future steps at world dt)
- `EdgeDebugHelper` for telemetry collection

**Six concrete edge manager variants**, selected at runtime via YAML config and a factory registry (`get_edge_class(name)`):

| Variant | Key | Pipeline | Use Case |
|---------|-----|----------|----------|
| **PerceptionEdge** | `PERCEPTION` | Raw detection merge → latency buffer → distribute to vehicles | Baseline V2X perception fusion |
| **ManeuverEdge** | `MANEUVER` | Vehicle poses → lane-graph transform → A* cooperative planning → waypoint buffers | Centralized lane-change/merge decisions |
| **LateFusionEdge** | `LATE_FUSION` | Late detection fusion → AB3DMOT tracking (min_hits=3, max_age=2) → linear prediction | Traditional MOT + prediction |
| **BM2CPEdge** | `BM2CP` | Intermediate BEV features → PointPillarBM2CP fusion_net → AB3DMOT (min_hits=1, max_age=10) → linear prediction | Learned BEV-space cooperative perception |
| **WorldFusionEdge** | `WORLDFUSION` | Intermediate features → PointPillarWorldFusion backbone + Where2comm attention (world frame) → WorldVoxelPostprocessor → self-beacon filter → AB3DMOT (min_hits=3, max_age=3) → ghost track filter → linear prediction | World-coordinate learned cooperative perception with attention |
| **VIPSEdge** | `VIPS` | RSU-only detections (no vehicle sensors, no self-beacon anchoring) → AB3DMOT → linear prediction | Infrastructure-only baseline for benchmarking |

The VIPS edge manager was added specifically to answer the research question: "Can infrastructure-only perception (VIPS) achieve what self-beacon anchoring provides?" It uses only RSU sensor detections with no vehicle contribution.

---

## 2. WorldFusion Cooperative Perception

A new end-to-end cooperative perception pipeline was added, integrating the WorldFusion model from the BM2CP submodule:

**WorldFusionPerceptionManager** (`worldfusion_perception_manager.py`):
- Processes LiDAR point clouds through `SpVoxelPreprocessor` (voxel size 0.4m)
- Processes 4 RGB camera images with camera-to-LiDAR extrinsic transforms
- Extracts intermediate spatial features (before backbone) for edge transmission
- Supports two execution modes:
  - **Local**: Runs `PointPillarWorldFusion` sensor encoder directly on GPU
  - **LitServe**: Sends compressed batch (pickle+zlib) to remote HTTP endpoint for GPU inference, reducing per-container GPU memory

**WorldFusion Edge Manager** (`edge_manager_worldfusion_ab3dmot_linear_predictor.py`, 1560 lines):
- Collects `feature_dict` (spatial_features) from all vehicles and RSUs
- Computes world-coordinate pairwise transformation matrices via `transformation_utils.x1_to_x2()` to align all agents to a world anchor frame
- Runs per-agent backbone on spatial features, then Where2comm attention-based fusion in world coordinates
- Runs classification and regression detection heads on the fused BEV feature map
- Post-processes through `WorldVoxelPostprocessor` yielding detections directly in world frame
- Filters self-detections using vehicle beacon poses (removes ego vehicle from its own detections)
- Feeds through persistent AB3DMOT tracker with ghost track filtering (removes static false positives based on velocity consistency over 5 frames)
- Linear constant-velocity prediction (25 future steps)
- Latency-aware evaluation: compares predictions against a CARLA snapshot captured at feature extraction time, providing ground-truth position/velocity for all actors at the moment sensors were read

**BM2CP Submodule Updates**:
- Added `PointPillarWorldFusion` model combining BM2CP sensor encoder with Where2comm attention mechanism
- Added `WorldVoxelPostprocessor` for world-frame anchor-free detection
- Updated dataset loaders for V2XSim2 format

---

## 3. Distributed Simulation Mode

The simulator now supports running vehicle and RSU actors in separate processes/containers that communicate with a central server through a C++ gRPC orchestrator:

**Architecture**:
```
Scenario Manager (Server)
    ├── CARLA World (direct access)
    ├── Edge Manager(s)
    ├── VehicleManagerProxies (lightweight state mirrors)
    └── gRPC Client → C++ Orchestrator (:50051)
                            ↕
                    Vehicle Client 1 (container)
                    Vehicle Client 2 (container)
                    RSU Client (container)
```

**C++ Orchestrator** (`ecloud_server.cc`):
- Multi-threaded, callback-based gRPC service
- Handles vehicle registration, tick broadcast (parallel to all clients), vehicle update collection and aggregation
- Stores and distributes edge predictions and waypoint buffers
- Signals server when all clients have responded (barrier synchronization)

**Data Flow Per Tick (Distributed)**:
1. Server advances CARLA world (`tick_world()`)
2. Server broadcasts command (TICK or PULL_OBJECTS_AND_TICK) via orchestrator
3. Orchestrator pushes command to all client push servers in parallel
4. Each client independently:
   - Runs localization (`update_info()`)
   - Runs perception (YOLO for object detection, WorldFusion/BM2CP for feature extraction)
   - Runs planning and control (`run_step()`, `apply_control()`)
   - Serializes and sends back: `pickled_agent_objects` (detected vehicles), `pickled_features` (compressed spatial features, ~1.7MB per RSU), `transform` (localized position), `velocity`
5. Orchestrator collects all responses, signals server
6. Server unpacks vehicle updates: sets proxy vehicle transforms/velocities, extracts features into perception managers
7. Edge manager runs server-side: fusion, tracking, prediction
8. Server pushes edge predictions back; clients pull on next tick via `Client_GetObjects()`

**What Distributed Mode Enables**:
- Parallel sensor processing across separate GPUs/containers
- Realistic network latency measurement (gRPC overhead captured in metrics)
- Scalability testing with 8-128+ vehicles
- Separation of concerns: vehicle perception runs on client hardware, edge computation runs on server
- Feature-level data exchange: only intermediate features are transmitted, not raw sensor data

**Key Proto Messages** (`ecloud.proto`):
- `Tick` (tick_id, command, timing)
- `VehicleUpdate` (tick_id, vehicle_index, actor_type, vehicle_state, transform, velocity, pickled_agent_objects, pickled_features)
- `ObjectBuffer` / `EdgeObstacleObject` (edge predictions with bbox, location, velocity, heading)
- `WaypointBuffer` / `EdgeWaypoints` (cooperative planning waypoints)

---

## 4. LitServe ML Offloading

A combined LitServe server (`litserve_models.py`) serves both YOLO and WorldFusion ML inference on a single port (18000), decoupling GPU model inference from the simulation process:

**Endpoints**:
- `/predict_msgpack` — YOLOv5 object detection (input: msgpack-encoded images, output: detections per camera, ~25ms)
- `/extract_features` — WorldFusion spatial feature extraction (input: compressed pickle batch with LiDAR voxels + camera images, output: compressed spatial features, ~150ms)

**Architecture**:
- Both endpoints served on port 18000 via a single FastAPI/uvicorn process
- YOLO model loaded via lazy `load_global_model()` in main process (not LitServe worker, which runs in a subprocess)
- WorldFusion model loaded via lazy `load_wf_model()` in main process
- WorldFusion uses binary pickle+zlib serialization (not JSON) for performance with large tensors
- Half-precision (float16) for spatial features to reduce bandwidth

**MLManager Dual-Mode** (`ml_manager.py`):
- **Local mode**: Models loaded in-process, direct inference
- **Distributed mode**: HTTP endpoints configured per model type, serialization/deserialization handled transparently
- Configurable per YAML: `yolo_endpoint`, `worldfusion_endpoint`, `bm2cp_vehicle_endpoint`, `bm2cp_edge_endpoint`

---

## 5. Comprehensive Metrics Architecture

The metrics system was refactored from scattered "debug helpers" into a structured metrics architecture with dedicated modules for each evaluation domain:

**Detection Metrics** (`detection_metrics.py`):
- Per-frame TP/FP/FN counts using distance-based matching
- Precision and recall per frame
- Average Precision (AP) at multiple distance thresholds (AP@0.3=4.0m, AP@0.5=2.0m, AP@0.7=1.0m)
- Accumulated metrics over full simulation for evaluation

**Tracking Metrics** (`tracking_metrics.py`):
- Standard MOT metrics: MOTA (Multi-Object Tracking Accuracy), MOTP (Multi-Object Tracking Precision)
- IDF1 Score (ID F1 = 2*IDTP / (2*IDTP + IDFP + IDFN))
- ID switch count (how often a track ID changes for the same GT object)
- Track fragmentation count (how often a track is interrupted)
- Per-frame and accumulated statistics

**Prediction Metrics** (`prediction_metrics.py`):
- ADE (Average Displacement Error) at 1s, 2s, 3s horizons
- FDE (Final Displacement Error) at prediction endpoint
- Miss rate (fraction of predictions > threshold from GT)
- minADE / minFDE for multi-modal predictions
- Configurable dt and prediction horizons

**Planning Metrics** (`planning_metrics.py`):
- Speed (m/s), acceleration (m/s^2), jerk (m/s^3) statistics
- Time-to-collision (TTC) tracking
- Lateral deviation from planned path (m)
- Per-vehicle serializable to protobuf for distributed collection

**Safety Manager** (`safety_manager.py`):
- Collision sensor events
- Stuck detection (vehicle not moving)
- Off-road detection
- Traffic light violation detection
- `get_metrics()` API for structured metrics extraction

**Evaluation Manager** (`evaluate_manager.py`):
- Orchestrates all metrics sources into a single evaluation pass
- Produces `simulation_metrics.json` with hierarchical structure:
  - Global metrics (success_rate, collision_count, edge_config)
  - Per-vehicle metrics (avg_speed, avg_acceleration, avg_ttc, perception/tracking times, collisions)
  - Per-edge metrics (algorithm_time, tracking_time, prediction_time, latency)
- Generates plots (EPS + PNG): kinematics, localization error curves, perception/tracking latency profiles
- Text summary report

**Additional Metrics from Distribution**:
- Client-side timing: `update_info_time`, `perception_time`, `tracking_time`, `localization_time`, `control_time` per vehicle
- Network timing: overall step time, step latency (orchestrator overhead), per-client tick duration
- Feature transfer metrics: bytes sent per client per tick
- Simulation timing: startup time, cumulative wall-clock time, per-tick profiling (broadcast, world_tick, spectator, edge processing)

---

## 6. Multi-Ego Vehicle and RSU Support

The simulator supports multiple perception-enabled ego vehicles and RSUs within a single scenario:

**Configuration** (YAML):
```yaml
edge_list:
  - manager_type: worldfusion
    edge_dt: 0.2
    vehicles:
      - name: cav1
        spawn_position: [x, y, z, roll, pitch, yaw]
        destination: [x, y, z]
        sensing:
          perception:
            activate: true
            backend: worldfusion  # or bm2cp, default
            camera: { num: 4, ... }
            lidar: { channels: 32, range: 120, ... }
    rsus:
      - name: rsu1
        spawn_position: [x, y, z]
        sensing:
          perception:
            activate: true
            backend: worldfusion
```

**Perception Backend Selection** (per vehicle/RSU independently):
- `default` → Standard YOLOv5-based PerceptionManager
- `bm2cp` → BM2CPPerceptionManager (intermediate BEV feature extraction)
- `worldfusion` → WorldFusionPerceptionManager (world-frame intermediate features)

**RSU Manager** (`rsu_manager.py`):
- Runtime backend selector picks perception class based on YAML config
- In distributed mode, server-side RSU managers are proxies using base PerceptionManager (no GPU models loaded on server)
- Client-side RSU loads full perception pipeline (potentially via LitServe)
- RSU features are sent to server alongside vehicle features for edge-level fusion

**Vehicle Manager Proxy** (`vehicle_manager_proxy.py`):
- Lightweight proxy on the server that mirrors the client vehicle's state
- Receives transform and velocity updates via gRPC each tick
- Stores `agent.objects` (detected vehicles) and perception features from client
- Edge manager reads features from all proxies for cooperative fusion

**Scaling**: Tested configurations from 1 to 128+ vehicles with distributed architecture. Edge managers handle arbitrary numbers of vehicles and RSUs per edge instance.

---

## 7. Scenario System

**70+ scenario test scripts** covering:
- OpenSCENARIO XML-based scenarios (1-18): overtake, lane change, intersection crossing, pedestrian scenarios
- Edge-integrated variants with RSUs and cooperative perception
- Distributed variants at multiple scales (8, 16, 24, 32, 48, 64, 128 vehicles)
- Weather/lighting variants (fog, rain, cloud cover at 25/50/75/100%)
- Platoon formation and stability tests

**OpenSCENARIO + ScenarioRunner Integration**:
- Scenarios defined in XML (spawn positions, routes, weather, actor models)
- Python scenario classes implement behavior trees via py_trees
- ScenarioRunner runs as a subprocess (sequential) or in a vehicle container (distributed)
- Route-based navigation support via RouteScenario

---

## 8. What Information is Available from the Distributed Sim that the Sequential Sim Cannot Provide

The distributed architecture enables collection of several metrics categories that are impossible or meaningless in sequential mode:

- **True network latency**: gRPC round-trip time between vehicle clients and server, including serialization/deserialization overhead
- **Per-client processing time breakdown**: How long each independent vehicle/RSU takes for perception, planning, and control — measured in actual wall-clock time rather than sequential accumulation
- **Feature transfer overhead**: Size of compressed intermediate features transmitted per client per tick (e.g., ~1.7MB compressed per RSU for WorldFusion)
- **Parallel processing efficiency**: Whether the orchestrator achieves true parallel execution across clients or serializes them
- **Container startup/registration time**: Time from vehicle container launch to first tick
- **Barrier synchronization overhead**: How much time is spent waiting for the slowest client each tick
- **Realistic LitServe inference latency**: ML model inference served over HTTP with realistic queueing and serialization delays

---

## 9. Network Simulation Capabilities

The edge manager base class provides configurable network simulation independent of the actual distributed gRPC network:

- **Fixed latency**: Constant delay in ms
- **Normal distribution**: Gaussian jitter with configurable std
- **Log-normal distribution**: Realistic heavy-tailed latency modeling
- **Hybrid C-V2X model**: Real radio RTT traces (sampled from CSV) + log-normal backhaul (mu=2.9957, sigma=0.6556) + base latency
- **Packet loss**: Independent uplink and downlink packet loss percentages
- **Latency-aware processing**: Edge managers maintain frame history buffers and index into delayed snapshots based on sampled latency, replaying historical detections/features with realistic staleness

---

## Summary of New/Modified Files

**New files** (not in original codebase):
- `ecav/core/application/edge/edge_manager/` (entire package: 7 files, 4060 lines)
- `ecav/core/sensing/perception/worldfusion_perception_manager.py`
- `ecav/core/sensing/perception/detection_metrics.py`
- `ecav/core/sensing/tracking/tracking_metrics.py`
- `ecav/core/prediction/prediction_metrics.py`
- `ecav/core/plan/planning_metrics.py`
- `ecav/core/application/edge/edge_manager/edge_manager_vips_ab3dmot_linear_predictor.py`
- `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py`
- `ecav/ml_manager/litserve_models.py`
- `ecav/core/prediction/linear_predictor_manager.py`
- `ecav/scenario_testing/openscenario_3_edge_worldfusion.py`

**Substantially modified files**:
- `ecav/scenario_testing/utils/sim_api.py` (distributed mode, feature unpacking, edge object pushing)
- `ecav/ecav2/ecloud_actor_client.py` (feature sending, transform/velocity population)
- `ecav/ml_manager/ml_manager.py` (dual-mode local/distributed, WorldFusion endpoint)
- `ecav/core/common/rsu_manager.py` (backend selection, distributed proxy support)
- `ecav/core/common/vehicle_manager.py` (perception backend selection)
- `ecav/scenario_testing/evaluations/evaluate_manager.py` (edge eval, JSON output, multi-source metrics)
- `ecav/core/safety/safety_manager.py` (`get_metrics()` API)
- `ecav/protos/ecloud.proto` (pickled_features, transform, velocity fields)
