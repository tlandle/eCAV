# Closed-loop cooperative driving platforms: capabilities and scale baselines

Prepared 2026-11-12. Every cell names its source. "code" means the upstream repository at the
commit pinned below, read on the benchmark host (`~/competitors`). "unknown" means the source
does not say. No benchmark of V2Xverse or AutoCastSim has been run yet. The OpenCDA benchmark is
reported separately in `scale_competitor.md` when the run finishes.

Pinned commits: OpenCDA `72b17e7b` (v0.1.2 + 81), V2Xverse `b6271753` (2025-09-11),
AutoCastSim `5b6a5bdc` (2022-09-23), MinkowskiEngine `9f81ae66` (v0.5.4, AutoCastSim dependency).

## Capability table

| Platform | Distributed execution | Measured compute in the loop | Radio model in the loop | Roadside and edge services | Sensor-derived state | Largest published or documented vehicle count | CARLA version |
|---|---|---|---|---|---|---|---|
| eCAV | Yes. Vehicles, roadside units and edge services are separate processes in lock step, on one or several hosts (ISPASS paper, platform section; `scripts/ispass/launch_scale.sh`) | Yes. Measured service time enters the age at use, replayable stamps (`compute_delay=measured`, `service_occupancy=on`) | Yes. ns-3 5G-LENA tables per message, or live co-simulation (`use_ns3_lut`, `radio=live`) | Roadside units and an edge fusion and prediction service | Yes. Camera and LiDAR pipelines, and exact state | 256 clients on one host without perception, 32 with perception (`paper_ispass/scale_sweep3.md`) | 0.9.15 |
| OpenCDA | No. One Python process steps every vehicle in turn (code: `opencda/scenario_testing/*.py` loops) | No. The simulator waits for each vehicle's step | No. Range check, position noise and a fixed lag in ticks (code: `opencda/core/common/v2x_manager.py` lines 56 to 86) | Roadside unit manager (code: `opencda/core/common/rsu_manager.py`). No edge service | Yes. Per-vehicle camera and LiDAR detection, optional | unknown as a limit. Shipped scenarios are single vehicles and platoons (code: 15 YAML files in `scenario_testing/config_yaml`) | 0.9.11 and 0.9.12 (docs `installation.md` line 48) |
| V2Xverse (CoDriving) | No. One evaluator process, one agent object that holds all communicating vehicles (code: `leaderboard_evaluator_parameter.py`, `pnp_agent_e2e.py`) | No. Inference runs every 4 ticks and the simulator waits (code: `skip_frames: 4` in `agent_config/pnp_config_*.yaml`) | No radio model. An optional constant delay in frames (code: `comm_latency`, `pnp_infer_action_e2e.py` lines 494 to 515). The paper studies a bandwidth budget on the shared features | One roadside unit per communicating vehicle, respawned along the route every 5 frames (code: `RSU` block of the agent config, `spawn_rsu`). No edge service | Yes. LiDAR collaborative perception feeding an end-to-end planner | `--ego-num` is a free parameter. The evaluation script uses 1 and data generation uses 2 (code: `scripts/eval_driving_e2e.sh`, `data_collection/base_script.sh`). Perception fuses at most 5 agents (`max_cav: 5`, `codriving_multiclass_config.yaml`) | 0.9.10.1 (README, step 2) |
| AutoCastSim (AutoCast, Coopernaut) | No for one scenario. One process, one thread per sharing vehicle joined every tick, MQTT broker for messages (code: `agent_wrapper.py` `tick_collaborators`, `AVR/Comm.py`). The README runs several independent scenario instances on several GPUs | No. Synchronous tick. Coopernaut reports inference time offline (ISPASS related work) | Partly. Scheduled sharing at a fixed rate of 7.2 Mbit/s within a 50 m half range, with a distance-based delivery probability (code: `AVR/__init__.py` `Utils.Rate`, `HalfRadioRange`; `AVR/Sched.py` `get_success_prob`) | None. Vehicle to vehicle only | Yes. LiDAR point clouds shared between vehicles. One vehicle plans, the others are traffic-manager vehicles | Three scenarios. Sharing vehicles are those within 50 m of the ego (code: `scenario_manager.py` `check_sensors_on_other_actors`) | 0.9.11 (README) |
| ms-van3t with CARLA (VaN3Twin-CARLA) | No. ns-3 and one OpenCDA process joined by a gRPC adapter (README, CARLA extension section) | No (README does not describe one) | Yes. ns-3 802.11p and NR-V2X per message (README examples `v2v-carla-80211p`, `v2v-carla-nrv2x`) | unknown | Through OpenCDA's perception module, which the README marks as failing in some configurations | unknown | 0.9.12 (README line 207) |
| EI-Drive | unknown here. Not cloned. The ISPASS platform table lists it as single host with modeled latency and error | unknown | unknown | unknown | unknown | unknown | unknown |

## Is a scale measurement a fair comparison with the eCAV sweep

- **OpenCDA.** Fair for the no-perception and the per-vehicle perception workloads: N vehicles, each with its own planner and controller, same town and spawn layout. Being run by the OpenCDA worker.
- **V2Xverse.** Fair only against eCAV's perception rows. Every communicating vehicle carries LiDAR, a roadside unit, collaborative perception and a learned planner, and there is no mode without perception. Common workload: N communicating driving vehicles with sensors, 20 Hz, wall-clock per tick. Differences that must be stated: Town05 (the platform's routes) against Town06, CARLA 0.9.10.1 against 0.9.15, one roadside unit per vehicle against shared roadside units, inference every fourth tick, and fusion limited to 5 agents per vehicle, so cost per vehicle should stay near constant while eCAV's edge service fuses all contributors.
- **AutoCastSim.** Not a like-for-like scale comparison. The number of sharing vehicles is not a parameter. It is the number of vehicles within 50 m of the single planning vehicle, and the other vehicles do not plan. What can be reported is wall-clock per tick against the measured number of sharing vehicles as background traffic grows, labelled as a sensor-sharing workload with one planner.
- **ms-van3t with CARLA.** Its vehicle layer is OpenCDA, so its client scaling is the OpenCDA result plus the ns-3 step. A separate measurement needs an ns-3 build and CARLA 0.9.12 and is not prepared. The eCAV radio experiment already reports the cost of live ns-3 per simulated second.
- **Open-loop tools (OpenCOOD, V2X-Sim).** They replay recorded data and run no closed loop, so a tick-time comparison does not apply.

## Installed on the benchmark host (A10 node), ready to run

- `~/competitors/V2Xverse`, env `~/miniconda3/envs/v2xverse` (Python 3.7, torch 1.10.1+cu113). Checkpoints `codriving`, `late_fusion`, `no_collaboration` from the platform's Hugging Face repository under `checkpoints/`. Scenario manager and agent import without a simulator.
- `~/competitors/AutoCastSim`, env `~/miniconda3/envs/autocast` (Python 3.7, torch 1.7.1+cu110, MinkowskiEngine 0.5.4 CPU build), `mosquitto` installed and its system service disabled. Runner and rule-based agent import without a simulator.
- CARLA 0.9.10.1 (7.6 GB) and 0.9.11 (8.5 GB) under `/mnt/competitors`, the Azure temporary disk. A deallocation erases them. `scripts/ispass/setup_competitors.sh carla` restores them (about 16 GB of downloads, about 10 minutes).
- Deviations from the platforms' instructions: spconv 2.x binary wheel in place of a source build of 1.2.1 (the V2Xverse preprocessor supports both), `timm` 0.6.13 without dependencies, `libtiff` 4.4 from conda for the old CARLA clients on Ubuntu 24.04, `LD_LIBRARY_PATH` set to the env for the Python process only, MinkowskiEngine built with a conda GCC 9.

## Commands once the GPU is free

One CARLA at a time. Both sweeps start and stop their own simulator and stop at the end.

```
ssh azureuser@20.81.176.215
cd ~/competitors && bash setup_competitors.sh carla        # only after a deallocation
V2XVERSE=~/competitors/V2Xverse OUT=~/competitors/results/v2xverse_run1 NS="1 2" TICKS=100 bash v2xverse_scale_sweep.sh   # smoke
V2XVERSE=~/competitors/V2Xverse OUT=~/competitors/results/v2xverse_run1 bash v2xverse_scale_sweep.sh                     # 1 to 64, stops at the first failure
AUTOCAST=~/competitors/AutoCastSim OUT=~/competitors/results/autocast_run1 NS="0 8" TICKS=100 bash autocast_scale_sweep.sh  # smoke
AUTOCAST=~/competitors/AutoCastSim OUT=~/competitors/results/autocast_run1 bash autocast_scale_sweep.sh                    # 0 to 256 background vehicles
```

The scripts on the host are copies of `scripts/ispass/{setup_competitors.sh,v2xverse_scale_sweep.sh,autocast_scale_sweep.sh,coop_sim_tick_bench.py}`.
Expected time: V2Xverse about 5 to 15 minutes per vehicle count if the tick time stays under 2 s (seven counts, one to two hours). AutoCastSim about 5 minutes per count (under one hour). Both are estimates, since neither has run.

## Not verified

- Neither CARLA 0.9.10.1 nor 0.9.11 has been started on this host. They have no `-RenderOffScreen` option. The sweeps start them with `SDL_VIDEODRIVER=offscreen` and `-opengl`, which is the documented off-screen method for those versions. If the server does not start on Ubuntu 24.04 with the GRID driver, the smoke run shows it in `carla.log`.
- Whether V2Xverse places N vehicles on a short Town05 route for N above 8. Extra vehicles are placed at the nearest map spawn points to the route start (code: `route_scenario.py` lines 360 to 400), and the sweep stops at the first count that fails.
- Whether the V2Xverse route lasts 340 ticks at every N. The bench records the ticks achieved and marks a shorter run as not completed.
- The bench wrapper has been exercised only up to the connection to the simulator on both platforms.
