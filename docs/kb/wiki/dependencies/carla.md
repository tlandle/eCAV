---
updated: 2026-04-04
---
# CARLA

## What It Is

CARLA (Car Learning to Act) is an open-source urban driving simulator built on Unreal Engine 4. It provides physics simulation, sensor simulation (cameras, LiDAR, radar, IMU, GNSS), and a Python API for programmatic control of vehicles and world state.

**Version in use:** 0.9.12+
**License:** MIT
**Upstream:** https://github.com/carla-simulator/carla
**Citation:** Dosovitskiy et al., CoRL 2017

## Role in eCAV

CARLA serves as the physics and sensor ground truth. Every simulated vehicle, camera frame, and LiDAR scan is produced by CARLA. eCAV does not model physics directly — it uses CARLA as a black-box simulator and interfaces via CARLA's Python client API.

Key CARLA responsibilities in eCAV:
- World state (vehicle positions, velocities, orientations — ground truth)
- Sensor simulation (camera images, LiDAR point clouds per configured actor)
- Actor spawning and control application (`apply_control()`)
- Map provision (Town05, Town06, custom freeway maps in `ecav/assets/`)

## How eCAV Interfaces with CARLA

- **Communication:** HTTP (CARLA's built-in REST API via Python client library)
- **Ticking:** Synchronous mode — `ecav.py` calls `world.tick()` to advance simulation; all actors must have applied controls before the tick
- **Actor management:** `ecav.py` creates/destroys CARLA actors; spawned via `carla.ActorBlueprint`

CARLA actors are distinct from eCAV actors. A "CARLA actor" is a simulated entity (has physics, sensors). An "eCAV actor" is the Python process that controls and reads from the CARLA actor.

## Running CARLA

**Headless (cloud/CI):**
```bash
/opt/carla-simulator/CarlaUE4.sh -RenderOffScreen
```

**Visual (local debugging):**
```bash
/opt/carla-simulator/CarlaUE4.sh
```

**Synchronous mode** is required for eCAV — CARLA must not advance the simulation until ecav.py calls `tick()`.

## Maps

Custom maps for eCAV scenarios are in `ecav/assets/`:
- `2lane_freeway_complete` — full freeway map
- `2lane_freeway_simplified` — simplified version for faster loading
- `Town05`, `Town06` — standard CARLA maps used for intersection scenarios

## Known Constraints

- CARLA 0.9.12 requires Ubuntu 20.04 or 22.04; Windows support is limited
- GPU required (Unreal Engine rendering); RTX 3080+ recommended for non-headless
- CARLA process is not containerized in eCAV (requires access to map files and GPU directly)
- Python client library version must match server version

## Related

- [Architecture](../architecture.md)
- [Scenario Runner dependency](scenario_runner.md)
- [OpenCDA dependency](opencda.md)
