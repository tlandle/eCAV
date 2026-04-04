---
updated: 2026-04-04
---
# Distributed Simulation

## Two Distinct Meanings of "Distributed"

A persistent source of confusion in eCAV is that "distributed" refers to two orthogonal concepts:

| Concept | Flag | Meaning |
|---------|------|---------|
| Distributed simulation | `-d` / `--distributed` | Actors run as separate OS processes, optionally in Docker containers |
| Distributed ML inference | `-l` / `--litserve` | ML perception offloaded to LitServe server (not in each actor process) |

These are independent. You can run distributed simulation with local perception (each container runs its own YOLO), or sequential simulation with LitServe perception, or any combination.

## Distributed Simulation (`-d`)

### Motivation
In sequential mode, all actors run in a single Python process. The GIL means only one actor runs its perception stack at a time, even on multi-core hardware. For N actors, per-tick time scales roughly linearly.

In distributed mode, each actor is a separate OS process. Python's GIL is per-interpreter, so N actor processes can truly run in parallel on N cores (or via Docker on a multi-node cluster).

### How It Works

1. **Orchestrator** (`ecav.py -t <scenario> -d`): spawns C++ comms server; waits for all actors to register; manages CARLA ticks
2. **Ego vehicles** (`ecav.py -d -i <N>`): each spawned by scenario_runner; registers with comms server; runs as `Ecav2ActorClient`
3. **Non-ego vehicles** (`ecav.py -d -i -1`): single call spawns all background traffic in CARLA
4. **RSU/Edge**: standalone process or Docker container; registers as an RSU or edge actor type

The first process to register (`-i 0` typically) is responsible for spawning all CARLA world actors. Subsequent processes discover their actor by querying CARLA.

### The C++ Comms Server

Central aggregation point. Written in C++ to bypass the Python GIL. Handles:
- Actor registration: distributes scenario config JSON to each registering actor
- Per-tick aggregation: waits until all expected actors have acked, then pushes collated state to orchestrator
- Edge mode: waits for edges rather than individual actors

Location: `ecav/ecloud_server/ecloud_server` (build with `make` in `ecav/`)

### Docker Containerization

All actors share a single `Dockerfile`. Each container runs an actor process.

Lifecycle:
- `start_actors.sh` — builds image if needed, starts containers
- `stop_actors.sh` — stops and removes all actor containers

Vehicle containers require NVIDIA runtime (`--runtime nvidia`) for GPU access if running local perception.

### Mode Combinations

| `-d` | `-l` | Description |
|------|------|-------------|
| ❌ | ❌ | Sequential, local perception — debugging only |
| ❌ | ✅ | Sequential, LitServe perception — useful for testing LitServe stack |
| ✅ | ❌ | Distributed actors, local perception — each container runs own YOLO |
| ✅ | ✅ | Distributed actors, LitServe perception — production/research mode |

### Configuration

Execution mode is CLI-only — not in YAML. YAML describes scenario content; CLI describes execution mode. The `distributed` field should be removed from all YAML files (pending edge architecture Phase 0 cleanup).

## Related

- [Architecture](../architecture.md)
- [Edge-Assisted Perception concept](edge_assisted_perception.md)
- [CARLA dependency](../dependencies/carla.md)
- [gRPC dependency](../dependencies/grpc.md)
- [edge_architecture_proposal.md](../../../agent_plans/edge_architecture_proposal.md)
