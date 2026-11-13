OpenCDA: tick wall mean / p50 / p95 (ms). eCAV: tick wall mean / p50 / p99 (ms), from scale_sweep3.


### dense layout, perception off

| N | OpenCDA, defaults | OpenCDA, map manager off | eCAV sequential | eCAV distributed | OpenCDA defaults / eCAV distributed | OpenCDA client RSS / CARLA RSS (GB) | GPU memory (MB) | planner exceptions |
|---|---|---|---|---|---|---|---|---|
| 8 | 162 / 160 / 179 | 98 / 97 / 110 | 201 / 202 / 412 | 66 / 60 / 274 | 2.5 | 0.57 / 3.8 | 7226 | 0 / 0 |
| 16 | 378 / 380 / 393 | 248 / 248 / 269 | 483 / 474 / 781 | 104 / 89 / 334 | 3.6 | 0.75 / 3.57 | 7225 | 0 / 0 |
| 32 | 918 / 919 / 945 | 610 / 619 / 670 | 1377 / 1348 / 1773 | 210 / 159 / 460 | 4.4 | 1.1 / 3.47 | 7224 | 0 / 229 |
| 64 | 2155 / 2151 / 2221 | 1441 / 1439 / 1533 | - | 435 / 389 / 792 | 4.9 | 1.82 / 3.51 | 7192 | 0 / 229 |
| 128 | 4913 / 4888 / 5292 | 3212 / 3210 / 3310 | - | 1066 / 1078 / 1548 | 4.6 | 3.27 / 3.05 | 6674 | 2 / 2 |
| 256 | 14038 / 13954 / 14652 | 9876 / 9827 / 10475 | - | 2383 / 2398 / 3485 | 5.9 | 6.09 / 3.08 | 6602 | 74 / 44 |

### spread layout, perception off

| N | OpenCDA, defaults | OpenCDA, map manager off | eCAV sequential | eCAV distributed | OpenCDA defaults / eCAV distributed | OpenCDA client RSS / CARLA RSS (GB) | GPU memory (MB) | planner exceptions |
|---|---|---|---|---|---|---|---|---|
| 8 | 293 / 287 / 363 | 156 / 150 / 212 | 135 / 129 / 188 | 55 / 53 / 249 | 5.4 | 0.57 / 3.78 | 7225 | 0 / 0 |
| 16 | 521 / 513 / 609 | 357 / 349 / 439 | 280 / 273 / 553 | 91 / 80 / 307 | 5.7 | 0.75 / 3.47 | 7225 | 0 / 0 |
| 32 | 1062 / 1051 / 1179 | 735 / 722 / 873 | 687 / 674 / 1006 | 177 / 144 / 397 | 6.0 | 1.11 / 3.44 | 7224 | 0 / 0 |
| 64 | 2579 / 2566 / 2790 | 1867 / 1862 / 2081 | - | 382 / 347 / 764 | 6.8 | 1.83 / 3.48 | 7192 | 0 / 0 |
| 128 | 5760 / 5759 / 6178 | 3979 / 4028 / 4407 | - | 997 / 1039 / 1573 | 5.8 | 3.27 / 3.03 | 6674 | 298 / 298 |
| 256 | 15313 / 15273 / 16057 | 10991 / 10975 / 11709 | - | 2774 / 2910 / 4384 | 5.5 | 6.09 / 3.07 | 6607 | 162 / 162 |

### dense layout, perception on

| N | OpenCDA, defaults | OpenCDA, map manager off | eCAV sequential | eCAV distributed | OpenCDA defaults / eCAV distributed | OpenCDA client RSS / CARLA RSS (GB) | GPU memory (MB) | planner exceptions |
|---|---|---|---|---|---|---|---|---|
| 8 | 1612 / 1596 / 1743 | - | - | 463 / 462 / 704 | 3.5 | 2.23 / 4.12 | 9127 | 0 / - |
| 16 | 3342 / 3323 / 3543 | - | - | 828 / 827 / 1141 | 4.0 | 2.65 / 4.29 | 10470 | 0 / - |
| 32 | 7237 / 7213 / 7625 | - | - | 1591 / 1616 / 1903 | 4.5 | 3.45 / 5.17 | 13164 | 0 / - |
| 64 | 15245 / 15180 / 15571 | - | - | - | - | 4.73 / 7.02 | 18538 | 0 / - |

Run metadata:

```
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-12T20:30:46+00:00
numpy 1.20.0 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-12T22:53:02+00:00
numpy 1.20.0 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T00:46:43+00:00
numpy 1.20.0 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T01:31:56+00:00
numpy 1.20.0 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T02:44:35+00:00
numpy 1.20.0 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T03:42:20+00:00
numpy 1.23.5 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T03:48:42+00:00
numpy 1.23.5 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T17:41:43+00:00
numpy 1.23.5 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T18:12:08+00:00
numpy 1.23.5 shapely 1.8.4
opencda 72b17e7b7fa0d67da1bf11a4083c90737eb1225f
edge-server
2026-11-13T18:28:20+00:00
numpy 1.23.5 shapely 1.8.4
```

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

Perception rows at 64 and 128 vehicles (added 2026-11-13, same commit, bench and settings as the 8 to 32 rows):
- 64 vehicles: 45 ticks, the first 5 dropped, 40 measured (a tick takes 15.2 s). Set-up 229 s. Peak GPU memory 18.5 GB.
- 128 vehicles: does not start on this host. The CARLA server ends with a segmentation fault (signal 11) while OpenCDA attaches the sensors, with 22.1 GB of the A10's 24.5 GB in use, in two of two attempts (10 and 12 minutes after launch). No tick is measured. Logs: `_perc128_attempt1`, `_perc128_attempt2` under the results directory on the benchmark host. eCAV at 128 vehicles holds the same 22.1 GB with 640x480 cameras and started in four of six attempts.

Cells rerun: spread layout, map manager off, 32, 64 and 128 vehicles. Their first measurements overlapped another worker's software installation on the host (23:55 to 00:30 UTC, Nov 12 to 13) and are kept under `_contaminated_install_window/` (means 762, 1856 and 3967 ms against 735, 1867 and 3979 ms in the reruns). The first sweep attempt, made before the planner-exception guard, is not used.

## Other platforms the paper positions against

- V2Xverse, AutoCast and Coopernaut, EI-Drive: closed loop, but each evaluates one ego (or a fixed small set of sharing vehicles in three scripted scenarios) driven by its own model on older CARLA releases (0.9.10.1 to 0.9.11). None exposes N independently planning cooperative vehicles, so there is no matching scale axis. Not benchmarked here.
- OpenCOOD, V2X-Sim, DAIR-V2X, Multi-V2X: open-loop datasets and model toolkits. No closed loop. Not applicable.
- ms-van3t-CARLA: bridges OpenCDA to ns-3. Its vehicle loop is OpenCDA's, so the OpenCDA rows are a lower bound on its tick time. Not built here.
- OpenCAMS (SUMO, CARLA, OMNeT++), Veins, Eclipse MOSAIC: traffic and network co-simulation without per-vehicle perception or planning stacks of the kind measured here. Not benchmarked.
- Pylot and D3: one ego. Not applicable to a vehicle-count sweep.
- CADET: distributed cooperative autonomy. Code availability was not checked in this pass.
