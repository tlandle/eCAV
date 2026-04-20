# Paper 2: Jordan Rapp Task Assignment

## Scope

Distributed simulation infrastructure for the SEC 2026 dense-scale evaluation.
Your work lives below the edge manager API boundary. You should not need to
modify the edge manager, predictor, tracker, or fusion internals.

Tyler handles: ML pipeline (SMART predictor, WorldFusion, runtime controller),
paper writing, evaluation analysis.

## Architecture Overview

4-node Azure deployment:

1. **Node 1 (CARLA + Sim Manager)**: Runs CARLA world and ScenarioManager.
   Orchestrates tick synchronization.
2. **Node 2 (Vehicle Clients)**: Dsv-series VMs. Each vehicle client is a
   separate process. Sends sensor data to edge via gRPC, receives predictions.
3. **Node 3 (Perception Front-End)**: NV12 or similar with GPU. Runs LitServe
   for feature extraction (WorldFusion spatial features, YOLOv5). Or runs
   CPU-local on vehicle clients if LitServe adds too much latency.
4. **Node 4 (Edge Server)**: Standard_NV36ads_A10_v5 (36 vCPU, 440GB RAM,
   1x A10 24GB). Runs edge_manager in-process with all models loaded.

## Tasks

### 1. Validate Distributed Mode End-to-End

**Priority: Highest**

Get the distributed simulation running with WorldFusion feature transmission.
The gRPC data flow for features is implemented but untested at scale.

Key files:
- `ecav/protos/ecloud.proto` (VehicleUpdate, IntermediateFeatures protos)
- `ecav/distributed_client/distributed_actor_client.py`
- `ecav/scenario_testing/utils/sim_api.py` (server orchestration)

Verify:
- Vehicle clients send `pickled_features` (spatial_features tensor) in VehicleUpdate
- Server unpacks into `vm_proxy.perception_manager.feature_dict`
- Edge manager collects features from all VMs and runs fusion
- Predictions flow back via `push_edge_objects` / `Client_GetObjects`
- Tick synchronization works across nodes

### 2. Scale Vehicle Client Count (N=4 to N=64)

**Priority: High**

Paper 2 evaluates at N = {4, 8, 16, 32, 48, 64}. The system currently works
at low N. Scaling requires:

- Spawning N vehicle client processes (containers or separate processes)
- Managing gRPC connections at scale (connection pooling, timeouts)
- Ensuring CARLA can handle 64 vehicles (may need to reduce rendering quality,
  use no-rendering mode, or batch spawning)
- Monitoring per-client resource usage (CPU, memory, network)

### 3. Per-Stage Latency Instrumentation

**Priority: High**

Add `time.perf_counter()` instrumentation around each pipeline stage in the
distributed data path. We need wall-time measurements for:

- Vehicle client: feature extraction time, serialization time, gRPC send time
- Server: gRPC receive time, deserialization time
- Edge manager: fusion time, tracking time, prediction time (Tyler will also
  instrument these internally, but we need the outer timing too)
- Return path: prediction serialization, gRPC push, client receive

Output per-tick timing to a structured log (JSON lines or CSV) that can be
parsed for the paper's latency breakdown figures.

Also track:
- Queue depth (jitter buffer size at each tick)
- GPU memory: `torch.cuda.memory_allocated()` on the edge node
- Network bytes per message

### 4. Azure Deployment Scripts

**Priority: Medium**

Create deployment automation for the 4-node setup:
- VM provisioning (or document manual steps)
- Docker containers or conda environments for each node
- Network configuration (ports, firewall rules for gRPC)
- CARLA server launch script (headless mode)
- Vehicle client batch launcher (spawn N clients pointing at CARLA + edge)
- Edge server launcher (loads all models, connects to CARLA for world state)

### 5. LitServe Perception Front-End Scaling

**Priority: Medium**

Evaluate whether LitServe on Node 3 is faster than CPU-local feature extraction
on vehicle clients. Prior testing showed LitServe adds latency overhead.

Test:
- LitServe with 1 model instance serving N clients
- LitServe with auto-scaling (multiple model instances)
- CPU-local feature extraction on vehicle clients (no GPU)
- Measure per-client feature extraction latency for each configuration

### 6. gRPC Performance Profiling

**Priority: Low**

Profile gRPC throughput and latency under load:
- Message sizes at different N (how big are serialized features?)
- Throughput ceiling (messages/sec at N=64)
- Whether we need streaming RPCs vs unary calls
- Compression options (gzip, snappy) for feature tensors

## What NOT to Modify

- Edge manager internals (`edge_manager_worldfusion_*.py`)
- Predictor managers (`linear_predictor_manager.py`, `smart_predictor_manager.py`)
- AB3DMOT tracker or KF tuning
- WorldFusion model weights or fusion logic
- Collision avoidance / behavior agent logic

If you need changes to any of these, coordinate with Tyler first.

## Communication

- Commit to the `distributed-integration` branch
- Keep commits focused on infrastructure (not ML or paper)
- Flag any issues with the gRPC proto definitions early. Proto changes affect
  both client and server and need coordination.
