## Method and differences

Host: the A10 node of scale_sweep3 (36 cores, 432 GB, one A10 24 GB), CARLA 0.9.15 started as in the eCAV sweep (`CarlaUE4.sh -RenderOffScreen`), 50 ms tick, synchronous mode, one CARLA at a time, restarted for every cell.

System under test: upstream OpenCDA, ucla-mobility/OpenCDA commit 72b17e7b (main, version string 0.1.3, 81 commits after tag v0.1.2), unmodified source. Bench: `scripts/ispass/opencda_scale_bench.py`, sweep: `scripts/ispass/opencda_scale_sweep.sh`.

Workload: N OpenCDA vehicle managers with the `single` application in OpenCDA's own loop (world tick, then for every vehicle `update_info`, `run_step`, `apply_control`), on Town06 at the spawn slots and destinations that `scripts/ispass/gen_scale_town06.py` gives eCAV (dense and spread layouts, same slot order), 40 km/h, overtaking off. 300 ticks up to 128 vehicles and 100 ticks at 256, first 5 ticks dropped, as in the eCAV sweep. The tick time is the wall-clock time of one loop iteration.

Differences from the eCAV rows:
- OpenCDA has no edge service, no roadside units and no forecasts in this workload. Each vehicle reads the vehicles within its range from the simulator and plans on them. The eCAV rows include the edge step (exact-state forecasts without perception, late fusion with perception), the roadside units and the delivery of forecasts to every vehicle. OpenCDA therefore does less work per tick than eCAV in every row.
- eCAV sequential was measured to 32 vehicles only.
- OpenCDA defaults are kept (localization on, map manager on, safety manager on) with every visualization off. The column "map manager off" is a tuning in OpenCDA's favour: the per-vehicle rasterized map is disabled.
- Perception rows: four cameras and one 64-channel LiDAR per vehicle as in eCAV's scale scenario. OpenCDA does not set the camera resolution, so its cameras render at CARLA's default 800x600 against 640x480 in eCAV. OpenCDA runs its YOLOv5m detector in the vehicle loop of the single process, one call per vehicle per tick. eCAV sends each vehicle's frames to the shared perception service.
- Percentiles: OpenCDA rows give the 95th percentile, the eCAV table on record gives the 99th. The JSON has the 99th percentile for OpenCDA as well.

Compatibility measures (OpenCDA source not edited):
- OpenCDA states support for CARLA 0.9.11 and 0.9.12. It ran on 0.9.15 with its 0.9.12 blueprint names (`--version 0.9.12`) and the `carla==0.9.15` client package. No patch to OpenCDA was needed.
- OpenCDA does not run in eCAV's Python 3.10 environment (NumPy 2 and Shapely 2 break its map manager). It ran in a separate Python 3.8 environment with its pinned requirements (NumPy 1.20.0, Shapely 1.8.4) and Pillow 9.5.0. The perception rows use a copy of that environment with torch 2.4.1 (CUDA 12.1) and NumPy 1.23.5.
- OpenCDA loads `ultralytics/yolov5` from the default branch through torch.hub, which no longer imports on Python 3.8. The hub cache was populated with the yolov5 tag v7.0 (915bbf29).
- Open-file limit raised to 65536 for the sweep. With the default 1024, the 256-vehicle run loses sensor streams (four sensors per vehicle) and OpenCDA's localization fails on a missing GNSS sample.
- Headless host: with perception active OpenCDA opens an Open3D window per vehicle even with LiDAR visualization off. The bench replaces the window constructor with a no-op.
- OpenCDA's local planner raises IndexError (empty waypoint or history buffer) for some vehicles held in a queue; without a guard the 32 and 128 vehicle dense runs abort. The bench catches the exception, applies full brake to that vehicle for the tick and counts it (column "planner exceptions", vehicle-ticks). A vehicle in that state does less planning work, which favours OpenCDA slightly.
- OpenCDA aborts at interpreter teardown (sensor callbacks on destroyed actors) after the measurements are written; the bench exits with `os._exit`.

Cells rerun: spread layout, map manager off, 32, 64 and 128 vehicles. Their first measurements overlapped another worker's software installation on the host (23:55 to 00:30 UTC, Nov 12 to 13) and are kept under `_contaminated_install_window/` (means 762, 1856 and 3967 ms against 735, 1867 and 3979 ms in the reruns). The first sweep attempt, made before the planner-exception guard, is not used.

## Other platforms the paper positions against

- V2Xverse, AutoCast and Coopernaut, EI-Drive: closed loop, but each evaluates one ego (or a fixed small set of sharing vehicles in three scripted scenarios) driven by its own model on older CARLA releases (0.9.10.1 to 0.9.11). None exposes N independently planning cooperative vehicles, so there is no matching scale axis. Not benchmarked here.
- OpenCOOD, V2X-Sim, DAIR-V2X, Multi-V2X: open-loop datasets and model toolkits. No closed loop. Not applicable.
- ms-van3t-CARLA: bridges OpenCDA to ns-3. Its vehicle loop is OpenCDA's, so the OpenCDA rows are a lower bound on its tick time. Not built here.
- OpenCAMS (SUMO, CARLA, OMNeT++), Veins, Eclipse MOSAIC: traffic and network co-simulation without per-vehicle perception or planning stacks of the kind measured here. Not benchmarked.
- Pylot and D3: one ego. Not applicable to a vehicle-count sweep.
- CADET: distributed cooperative autonomy. Code availability was not checked in this pass.
