---
updated: 2026-04-04
---
# OpenCDA

## What It Is

OpenCDA is an open cooperative driving automation framework built on top of CARLA. It provides vehicle management, planning, perception, and co-simulation utilities for testing cooperative driving algorithms.

**Upstream:** https://github.com/ucla-mobility/OpenCDA
**Citation:** Xu et al., arXiv:2107.06260

## Role in eCAV

eCAV derives from OpenCDA. The `opencda/` directory contains the legacy upstream code. The `ecav/` directory contains eCAV's active development, which supersedes much of `opencda/`.

The core vehicle planning and control logic in `ecav/core/` originated in OpenCDA and has been progressively modified. The simulation entry point (`ecav.py` vs. the original `opencda.py`) and the distributed actor architecture are eCAV additions.

## What eCAV Changed

- **Distribution:** OpenCDA is single-process. eCAV added gRPC-based distributed actors (C++ comms server, `Ecav2ActorClient`).
- **Perception serving:** Added LitServe integration for offloading YOLO inference.
- **Collaborative perception:** Added BM2CP and WorldFusion intermediate fusion.
- **Scenario runner integration:** Tighter integration with CARLA's scenario_runner.
- **Network modeling:** Added C-V2X and 5G latency injection.
- **Metrics:** Added per-tick timing instrumentation and evaluation framework.

## Directory Split

Before editing any file, check whether an `ecav/` counterpart exists:

| Legacy location | Active location |
|----------------|----------------|
| `opencda/scenario_testing/` | `ecav/scenario_testing/` |
| `opencda/core/` | `ecav/core/` |
| `opencda/customize/` | `ecav/customize/` |
| `opencda.py` | `ecav.py` |

The `opencda/` directory is kept for reference and potential upstream sync, but new work never goes there.

## Related

- [CARLA dependency](carla.md)
- [Architecture](../architecture.md)
- [eCAV paper](../research.md)
