# eCAV Knowledge Base

Master index. All wiki articles are LLM-maintained. Raw source material lives in `raw/`.

## Core

| Article | Description |
|---------|-------------|
| [Current State](current_state.md) | Active branch, in-flight work, next steps — start here after a gap |
| [Architecture](architecture.md) | System overview and process topology |
| [Research](research.md) | Research direction, hypotheses, experiment results |
| [Decisions](decisions.md) | Architectural decision log with rationale |
| [Plans Index](plans_index.md) | Status of all implementation plans |

## Concepts

| Article | Description |
|---------|-------------|
| [Edge-Assisted Perception](concepts/edge_assisted_perception.md) | The blind intersection scenario and what "edge-assisted" means |
| [Distributed Simulation](concepts/distributed_simulation.md) | `-d` flag: actors as separate processes/containers |
| [Collaborative Perception](concepts/collaborative_perception.md) | Late fusion vs. intermediate fusion |
| [Latency Modeling](concepts/latency_modeling.md) | Network latency, C-V2X, 5G trace-driven models |

## Dependencies

| Article | Description |
|---------|-------------|
| [CARLA](dependencies/carla.md) | Physics and sensor simulator |
| [OpenCDA](dependencies/opencda.md) | Upstream baseline; what we forked and diverged |
| [gRPC](dependencies/grpc.md) | Inter-process communication layer |
| [LitServe](dependencies/litserve.md) | ML model serving framework |
| [YOLOv5](dependencies/yolov5.md) | Object detection (late fusion path) |
| [WorldFusion](dependencies/worldfusion.md) | Intermediate fusion with world-model reconciliation |
| [BM2CP](dependencies/bm2cp.md) | OpenCOOD-based intermediate feature fusion |
| [SORT / AB3DMOT](dependencies/sort_ab3dmot.md) | Multi-object tracking stack |
| [Scenario Runner](dependencies/scenario_runner.md) | CARLA behavior tree execution engine |

## Raw Sources

- [sessions/](../raw/sessions/) — End-of-session logs (written by Claude at session end)
- [notes/](../raw/notes/) — Research discussion notes (jrapp contributes)
- [papers/](../raw/papers/) — Paper and reference markdown
