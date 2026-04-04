---
updated: 2026-04-04
---
# CARLA Scenario Runner

## What It Is

CARLA Scenario Runner is a tool for executing complex driving scenarios using behavior trees (py_trees). Scenarios are defined in Python (behavior tree logic) and XML (parameters). The runner interfaces with the CARLA Python API to spawn actors, trigger conditions, and evaluate scenario success.

**Submodule:** `scenario_runner/` (fork: `tlandle/scenario_runner`)
**Upstream:** https://github.com/carla-simulator/scenario_runner (CARLA's official runner)

## Why a Fork

The `tlandle/scenario_runner` fork extends the upstream runner to integrate with eCAV's distributed actor architecture. Key modifications:
- Spawns `Ecav2ActorClient` instances for ego vehicles in distributed mode
- Integrates with eCAV's gRPC registration flow
- Supports eCAV-specific scenario parameters

## Role in eCAV

The scenario runner is spawned as a subprocess by `ecav.py`. It executes the behavior tree logic for a given scenario (e.g., the occluded left-turn sequence), triggering conditions and monitoring completion.

Relationship to eCAV scenarios:
- **eCAV scenario** (`.py` + `.yaml` in `ecav/scenario_testing/`) — defines world properties, actor configs, calls `run_scenario()`
- **Scenario runner scenario** (`.py` + `.xml` in `ecav/scenario_testing/scenarios/`) — defines behavior tree; executed by `scenario_runner.py`

The eCAV scenario YAML contains a `scenario_runner` block that names the associated scenario runner scenario.

## Key Scenario

**`scenario_3`** / **`openscenario_3_edge`** — occluded left-turn at a blind intersection. The primary evaluation scenario for edge-assisted perception research.

Associated files:
- `ecav/scenario_testing/scenarios/scenario_3.py` — behavior tree
- `ecav/scenario_testing/scenarios/scenario_3.xml` — parameters
- `ecav/scenario_testing/openscenario_3_edge.py` — eCAV entry point
- `ecav/scenario_testing/config_yaml/openscenario_3_edge.yaml` — world config

## Related

- [CARLA dependency](carla.md)
- [Architecture](../architecture.md) — §2.3 Scenario Runner
- [Distributed Simulation concept](../concepts/distributed_simulation.md)
