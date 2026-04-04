---
updated: 2026-04-04
---
# System Architecture

Compiled from [.claude/ARCHITECTURE.md](../../.claude/ARCHITECTURE.md), which is the authoritative source. This article adds synthesis and fills in documented TODO sections.

## Research Context

eCAV characterizes the operational safety envelope of edge-assisted autonomous vehicles. Representative scenario: a camera at a blind intersection. The edge fuses perception from that camera with perception from approaching vehicles, allowing detection of obstacles that would otherwise be occluded.

Primary evaluation axes: network latency, jitter, multi-source state inconsistency, closed-loop safety impact.

## Process Topology

### Sequential Mode (algorithm debugging)

Three processes:
1. **CarlaUE4** — world physics and sensor simulation (`/opt/carla-simulator/CarlaUE4.sh`)
2. **ecav.py** — scenario orchestrator + all vehicle/RSU actors in one process
3. **scenario_runner.py** — spawned as subprocess; executes scenario behavior tree

### Distributed Mode (`-d` flag)

Five process types:
1. **CarlaUE4** — world simulation
2. **ecav.py (orchestrator)** — manages ticks, metrics, spawns C++ comms server
3. **ecloud_server (C++)** — central comms; true multithreaded actor coordination (bypasses GIL)
4. **scenario_runner.py** — spawned by orchestrator; spawns ego vehicle `Ecav2ActorClient` instances
5. **ecloud_actor_client** — one per vehicle/RSU; optionally in Docker container

The first ego vehicle process to register spawns all CARLA actors. All non-ego vehicles are spawned by a single `ecav.py -i -1` call.

### LitServe Mode (`-l` flag, orthogonal to `-d`)

An additional optional process:
6. **LitServe server** — ML inference; port 18000 (WorldFusion HTTP), port 18001 (YOLO gRPC)

## Key Components

### Root Orchestrator (`ecav.py`)
Parses `-t <scenario>`, `-d`, `-l`, `-i <index>` flags. Calls `run_scenario()` or `run_vehicle()` on the scenario module. Manages CARLA world ticking. Spawns C++ comms server via `ecav/scenario_testing/utils/sim_api.py`.

### C++ Comms Server (`ecav/ecloud_server/ecloud_server`)
Written in C++ to achieve true multithreading, bypassing the Python GIL. Receives scenario config from orchestrator and distributes to registering actors. Aggregates per-tick actor responses; pushes collated state to orchestrator once all actors have acked. With edge processes: waits for all edges (not individual vehicles) before notifying orchestrator.

This replaced a prior Python asyncio design where GIL contention was a significant bottleneck.

### Scenario Runner (`scenario_runner/scenario_runner.py`)
CARLA scenario_runner fork (git submodule; `tlandle/scenario_runner`). Executes pytrees behavior tree logic (e.g., occluded left-turn). In distributed mode, spawns `Ecav2ActorClient` instances for ego vehicles.

### Vehicle Actors
- Sequential: `vehicle_manager` instances within orchestrator process
- Distributed: `Ecav2ActorClient` in separate processes/containers (`ecav/ecav2/ecloud_actor_client.py`)
- Per-tick sequence: `update_info()` → `run_step()` → `apply_control()`

### Edge / RSU
- Sequential: actor class instances within orchestrator
- Distributed: standalone processes, typically Docker containers
- Role: fuse camera perception with vehicle perception; publish reconciled obstacle state

### LitServe Perception Server

Two independent inference services:

| Service | Transport | Port | Endpoint | Model |
|---------|-----------|------|----------|-------|
| YOLO late fusion | gRPC | 18001 | `PerceptionService.Detect` | YOLOv5 |
| WorldFusion intermediate fusion | HTTP | 18000 | `/extract_features` | WorldFusion |

Late fusion: each vehicle actor calls LitServe directly.
Intermediate fusion: edge calls LitServe with merged multi-vehicle feature data.

## Communication

| Link | Protocol | Notes |
|------|----------|-------|
| Orchestrator ↔ CARLA | HTTP | CARLA's native REST API |
| Actors ↔ C++ comms server | gRPC | `ecloud.proto` |
| Vehicle/edge ↔ LitServe (YOLO) | gRPC | `perception.proto`; migrated from HTTP |
| Edge ↔ LitServe (WorldFusion) | HTTP | gRPC migration planned |

Proto files: `ecav/protos/ecloud.proto`, `ecav/protos/perception.proto`
Recompile stubs: `python ecav.py --build`

## Simulation Tick Lifecycle

```
1. ecav.py ticks the CARLA world
2. CARLA applies previous-step actor controls (brake/steer/throttle)
3. Orchestrator signals C++ comms server: new tick
4. Comms server notifies actors (direct) or edges (when edge processes exist)
5. Each actor: update_info() → run_step() → apply_control()
   - Perception (LitServe call) happens inside run_step()
   - Edge actors: collect vehicle sensor data, call LitServe for intermediate fusion
6. Actors ack comms server (or their edge, which acks comms server)
7. Comms server: all acks received → push collated state to orchestrator
8. Orchestrator processes state → next tick or scenario end
```

## Actor State Machine

States defined in `ecav/protos/ecloud.proto`:

```
REGISTERING → CARLA_UPDATE → TICK_OK (repeated) → TICK_DONE
```

Full enum:
```
REGISTERING = 0   # Initial registration with comms server
CARLA_UPDATE = 1  # Received actor info from CARLA
UPDATE_INFO_OK = 2
GET_DESTINATION = 3
TICK_OK = 4       # Normal per-tick ack
TICK_DONE = 5     # Vehicle reached destination; dumps diagnostics
OK = 6
ERROR = 7
DEBUG_INFO_UPDATE = 8  # Terminal: dump all stored debug diagnostics
```

## Scenario Configuration

Each scenario requires a file triplet:

| File | Location | Purpose |
|------|----------|---------|
| `<scenario>.py` | `ecav/scenario_testing/` | Python entry: `run_scenario()` / `run_vehicle()` |
| `<scenario>.yaml` | `ecav/scenario_testing/config_yaml/` | World properties, actor config, YAML schema |
| `<scenario_runner>.py` | `ecav/scenario_testing/scenarios/` | Behavior tree logic (pytrees) |
| `<scenario_runner>.xml` | `ecav/scenario_testing/scenarios/` | Scenario parameters for CARLA |

Invocation: `python ecav.py -t <scenario_name>` (names must match across `.py` and `.yaml`).

## Deployment

| Mode | Command |
|------|---------|
| Sequential local | `python ecav.py -t <scenario>` |
| Distributed local | `bash start_actors.sh` |
| With LitServe | Add `-l`; start LitServe process separately |
| Cloud (Azure) | Same scripts; CARLA in headless mode |

Docker: single `Dockerfile` for all actor types. `start_actors.sh` / `stop_actors.sh` manage lifecycle.

Conda env: `opencda`. Must be activated before running orchestrator.

## Architecture TODOs (open in ARCHITECTURE.md)

- §6.2: YAML schema documentation
- §6.3: Distributed mode YAML configuration
- §7: Distributed perception end-to-end description
- §9: Metrics and evaluation
- §10: Known limitations

The edge architecture proposal ([edge_architecture_proposal.md](../../agent_plans/edge_architecture_proposal.md)) addresses several of these.

## Related

- [.claude/ARCHITECTURE.md](../../.claude/ARCHITECTURE.md) — authoritative source
- [Distributed Simulation concept](concepts/distributed_simulation.md)
- [Collaborative Perception concept](concepts/collaborative_perception.md)
- [gRPC dependency](dependencies/grpc.md)
- [CARLA dependency](dependencies/carla.md)
- [LitServe dependency](dependencies/litserve.md)
