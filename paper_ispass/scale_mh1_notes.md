# Multi-host scale run (stamp mh1), 2026-11-13

## Deployment

- A10 node (edge-server, 36 cores, 432 GB, one A10 24 GB, private 10.1.0.4): CARLA 0.9.15, the orchestrator, the C++ orchestration server, the edge process, the roadside unit clients.
- carla-client3 and carla-client4 (64 cores, 251 GB each, no GPU, private 10.0.0.11 and 10.0.0.12): the vehicle clients, split evenly by index.
- carla-ndm (8 V100 32 GB, private 10.2.0.6): perception service replicas, one per GPU, for the perception rows only.
- Round trip A10 to carla-client3 1.5 ms, to carla-client4 3.6 ms, to carla-ndm 5.3 ms. Clocks in step by chrony.
- Scenario, tick counts and reduction as in scale_sweep3: scale_town06, 50 ms tick, 300 ticks up to 128 vehicles and 100 above, first 5 ticks dropped. Tables: `scale_mh1.md`, `scale_mh1_compare.md`. Figures: `fig_scale_mh1.pdf`, `fig_scale_mh1_compare.pdf`.

## Code versions

- No-perception rows, strong scaling and equivalence: hosts hold develop at 7c3a49b7 (launcher, generator and every tracked file under `ecav/`). All of these cells ran in one sweep started 01:59 EST. The single-host sweep3 rows ran on the older worktree (c4e0e7db plus the scale fixes).
- Perception rows: develop at ec2331d0 (7c3a49b7 plus the one-line client change that lets a client with the shared service start without a GPU) and the launcher of b355d528. Clients are started four at a time (`BATCH=4`).
- Earlier attempts are kept under `experiment_results/ispass_scale/_mh1_prefix`, `_mh1_prefix2` and `_mh1_debug` and are not used.

## Defects found and fixed

| defect | effect | fix | commit |
|---|---|---|---|
| Client VMs lacked `libGL.so.1` and the `opencood` package that the env installs in editable mode from a path outside the worktree | no vehicle client imports | `apt install libgl1 libglib2.0-0`, package sources copied to the same path (host setup, not in the repository) | host change |
| RSU clients on the orchestrator host registered the comma-separated list of client hosts as their push address | the edge never reaches the RSUs and every tick waits for its 30 s timeout | the launcher writes `actor_client_ip`, the host's own address, and the actor client prefers it | 6bbb3e4a |
| A single-host run after a multi-host run kept the rewritten `cloud_config.yaml` | wrong addresses | the single-host run restores the saved file | 630e0079 |
| CARLA keeps the traffic manager registration (address and port) of the previous run's clients | a client on another host layout fails with a traffic manager bind error | the launcher restarts CARLA when the layout of client hosts changes | b786e346 |
| CARLA 0.9.15 stream server in synchronous mode: a write that has not completed when the next frame's message is queued for the same client makes a streaming worker spin forever. With remote clients the 16 KB initial send buffer is smaller than the world state at a few hundred actors | at 256 vehicles all 11 streaming workers spun, no client received a frame, start-up hung | 4 MB initial TCP send buffer on the CARLA host (`net.ipv4.tcp_wmem`), CARLA restarted after a failed run | 536e1794 |
| The client VMs' security group admits any port from the internet. The CARLA traffic manager RPC server of client 0 (port 8000) terminates the process on a non-RPC message (`clmdep_msgpack` type or parse error; reproduced with a TLS hello and with byte 0xc1) | client 0 died twice in mid-run and every tick then waited 30 s | each host drops connections to 8000-8999 and 50051-51300 that are not from 10.0.0.0/8 or loopback. The rule dropped 203 and 149 packets on the two clients in 1 h 45 min. The security group is unchanged | 403bb2b7 |
| `ecav.py` initialised CUDA whenever perception was on, also with the shared service | a client on a host without a GPU did not start | CUDA is initialised only when the client runs its own model | ec2331d0 |
| Every client is a remote user of client 0's traffic manager (the scenario YAML gives all clients port 8000; the `-T` option is parsed and never applied). The RPC has a fixed 2 s timeout, and with many sensors in the world the simulator answers slowly | from 48 vehicles with perception some clients die at start | the launcher starts such a client again, up to three times. The dead `-T` option is not changed (it would change every distributed run on develop) | b355d528 |
| The daily 01:00 EST auto-shutdown of carla-client3 and carla-client4 deallocated both in mid-sweep | two cells lost | VMs started again, cells rerun. The schedule is unchanged | none |

## Vehicle-count scaling without perception

Mean tick wall time in ms (median in parentheses). One host: scale_sweep3. Three hosts: this run.

| layout | deployment | 64 | 128 | 256 | 512 |
|---|---|---|---|---|---|
| dense | one host (sweep3) | 435 (389) | 1066 (1078) | 2383 (2398) | not run |
| dense | three hosts | 338 (278) | 698 (712) | 1504 (1547) | 4001 (4150) |
| spread | one host (sweep3) | 382 (347) | 997 (1039) | 2774 (2910) | not run |
| spread | three hosts | 303 (276) | 709 (733) | 1731 (1870) | 4507 (4780) |

Strong scaling at 256 vehicles, dense, one code version (7c3a49b7):

| vehicle clients on | tick mean / p50 / p99 (ms) | clients | edge step | transport | world | CPU busy |
|---|---|---|---|---|---|---|
| the A10 node with everything else | 2302 / 2340 / 3281 | 1127 | 830 | 292 | 53 | A10 64 % |
| carla-client3 | 2080 / 2119 / 3104 | 986 | 712 | 338 | 44 | A10 5 %, client 54 % |
| carla-client3 and carla-client4 | 1504 / 1547 / 2361 | 524 | 730 | 207 | 43 | A10 6 %, clients 37 % and 22 % |

The single-host cell confirms the sweep3 value (2383 ms) within 3.4 % under today's code.

Largest vehicle count: 512 completes in both layouts (4.0 s and 4.5 s per tick, 100 ticks). No larger count was tried: at 512 each client host holds 256 clients and 242 of 251 GB (about 0.94 GB per client), so 512 is the memory limit of two client VMs of this size.

What limits the tick at 512, dense (critical path): edge step 2091 ms (52 %), client step 1375 ms (34 %), client reply 296 ms (7 %), CARLA world tick 102 ms (2.6 %), edge fan-out 88 ms, the remaining transport under 10 ms. The edge step is one Python process and grows with vehicles times neighbours. The client hosts are 21 % and 39 % busy and the A10 node 6 %, so the client step is not limited by client CPU. Start-up takes 669 s at 512 vehicles.

## Equivalence of the two deployments (16 vehicles, dense, 300 ticks)

| pair | position source | largest difference | first tick that differs |
|---|---|---|---|
| one host, run twice | ground truth, 0.1 m | 0.0 m | none |
| one host against three hosts | ground truth, 0.1 m | 11.1 m (RMS 1.50 m) | 41 |
| one host against three hosts, exact localization | planner ego position, 0.01 m | 0.00 m over 4194 vehicle-ticks | none |

With the GNSS model on, the two deployments differ, and each is repeatable (two three-host runs agree with each other, two one-host runs agree with each other). The GNSS noise sequence a vehicle draws is the same within a deployment and differs between them from the first sample, because the sensors are created in a start-up order that depends on how the clients are batched per host. With localization taken from the simulator's transform (`gen_scale_town06.py --exact-localization`) every vehicle follows the same trajectory to 0.01 m for 300 ticks (vehicles travel up to 137 m). The lock step, the edge and the planners therefore produce the same result on one host and on three. The noise sequence is not controlled across deployments.

## Perception scaling with service replicas

Vehicle clients on the two client hosts (no GPU), CARLA and its sensor rendering on the A10 node, R replicas of the shared YOLOv5m service on carla-ndm (one V100 each), client k calls replica k mod R. Four cameras and one LiDAR per vehicle. 300 ticks up to 32 vehicles, 100 above. Mean tick wall time in ms:

| replicas | 8 | 16 | 32 | 64 | 128 |
|---|---|---|---|---|---|
| 1 | 372 | 632 | 1226 | 2393 | 4197 |
| 2 | 259 | 450 | 868 | 1804 | 4221 |
| 4 | 258 | 451 | 871 | 1772 | 4144 |
| 8 | 258 | 452 | 873 | 1801 | 4198 |
| one host, service on the A10 (sweep3) | 463 | 828 | 1591 | not run | not run |

Service requests per wall-clock second (vehicles and the two RSUs, one request each per tick): 27 to 31 with one replica at every vehicle count, 37 to 40 with two or more replicas up to 64 vehicles, 31 to 32 at 128 vehicles for every replica count. The standalone ceiling of one replica is 37 requests per s.

What limits each cell:
- One replica, 8 to 64 vehicles: the replica. It serves 27 to 31 requests per s in the run, the tick grows by about 37 ms per vehicle, and a second replica removes 25 to 30 % of the tick.
- Two or more replicas, 8 to 64 vehicles: the simulator. The tick is the same for 2, 4 and 8 replicas (258, 451, 871, 1790 ms, 27 to 32 ms per vehicle). The replicas' GPUs are 1 to 20 % utilised, the client hosts 2 to 8 % busy, the edge step is 26 to 520 ms. Clients wait in their step for the simulator, whose game thread runs at about 90 % of one core while the A10 GPU is 38 to 47 % utilised at every vehicle count. The limit is the serial issue of sensor renders in CARLA, not GPU throughput.
- 128 vehicles: the simulator for every replica count (4.1 to 4.2 s per tick, 31 requests per s against a ceiling of 37 per replica). CARLA holds 21.5 to 22.1 GB of the A10's 24 GB with 512 cameras and 128 LiDARs, so 128 vehicles is also the sensor memory limit of this GPU. Start-up takes 26 minutes (15 for the clients, started four at a time, and 11 for the orchestrator's vehicle proxies), and 21 to 30 client starts per run are repeated after the traffic manager RPC timeout (none up to 64 vehicles).
- The client hosts never limit a perception cell.

Against the single-host rows of sweep3 (service on the A10 beside the simulator): 463 to 258 ms at 8 vehicles, 828 to 451 ms at 16, 1591 to 871 ms at 32 with two or more replicas.

## Why more inference replicas do not shorten the perception tick: measured decomposition

One instrumented run, 32 vehicles, 4 replicas, clients on the two client hosts (`ECAV_SENSOR_TIMING=1`, stamp mh1dec, tick 874 ms against 871 ms without the instrumentation). Reduction: `scripts/ispass/perception_step_breakdown.py`, a 60 s stack sample (py-spy, 100 Hz, wall clock) of one client per host, and 1 s samples on the A10 node.

How the client uses sensors and the service:
- A client sends one inference request per tick: the four camera frames as JPEG in one call (412 KB). The LiDAR is not sent. Nothing is sent concurrently because there is one call.
- A client does not wait for the sensor frames of the tick. It reads its sensor buffers 10 ms after the world tick returns (p90 12 ms). The camera frames of that tick arrive a median 571 ms after the world tick (p10 77 ms, p90 906 ms), the LiDAR frame 204 ms (p90 586 ms). Detection therefore runs on camera frames that are one world tick old in 88 % of client ticks, two ticks old in 10 % and current in 2 %. LiDAR: one tick old in 98 %. This is the existing behaviour of the distributed perception path, not an effect of the multi-host deployment being measured here.
- Raw frames: 1.23 MB per camera frame (640x480 BGRA), 0.82 MB per LiDAR frame, 5.74 MB per vehicle per tick.

Client step, mean per client tick (9433 client ticks): 619 ms in total.

| stage | ms | share |
|---|---|---|
| wait for sensor frames | 0 | 0 % (the client does not wait) |
| colour conversion before the call | 0.6 | 0.1 % |
| JPEG encoding of four frames | 5.2 | 0.8 % |
| inference call round trip | 174.5 | 28 % |
| of which the handler in the replica | 97.5 (JPEG decode 6.6, model 90.4, encode 0.4) | |
| of which network and wait for a handler thread | 77 | |
| rest of detection (LiDAR and camera fusion, vehicle-side tracker) | 41 | 7 % |
| planner and controller (`run_step`) | 352 | 57 % |
| reply to the edge | 30 (median 10) | 5 % |

Inside the replica the model takes 25 to 27 ms for a request that runs alone and 90 ms here, because 3.5 handler threads on average run the model on the same GPU at once. Per-replica throughput is unchanged by that (37 requests per s), so with 8 clients per replica the call is not the bound.

Inside `run_step` the stack samples put 17 to 19 % of the client's wall time (150 to 165 ms per tick) on one statement, `behavior_agent.py:814`, which calls `vehicle.get_traffic_light()` twice. Each is a synchronous request to the simulator and is answered only between simulator frames. 7 to 8 % (about 65 ms) is the unpickling of the forecast message (`pred_blobs.unpack`). The planning arithmetic itself is under 5 %. Without perception the whole client step at 32 vehicles is 76 ms, so the same two simulator requests cost a few ms there.

Simulator host during the ticks: outbound 227 MB/s median (1.8 Gbit/s, max 248 MB/s) against a measured ceiling of 23.5 Gbit/s on one TCP stream from the A10 node to carla-client3 (28.9 Gbit/s on eight streams). CARLA's game thread 85 % of one core, RHI thread 47 %, render thread 42 %, 11 task-graph threads about 35 % each. GPU 43 % utilised.

Which stage bounds the tick with two or more replicas: the simulator's game thread. After a world tick it spends most of the next tick period reading back and sending 160 sensor frames (the last camera frame leaves 0.9 s after the tick), and while it does so it answers client requests slowly, which is where the client's step goes. It is not GPU rendering throughput (43 %), not the network transfer of the raw frames (8 % of the link), and not the inference service.

What would move it:
- CARLA secondary servers (multi-GPU) take the sensor readback off the primary's game thread. This is the change that addresses the bound.
- One simulator request instead of two in `traffic_light_manager` removes about 75 ms per client tick at 32 vehicles (9 % of the tick) without changing the planner's input.
- Clients on the simulator host would not help: the network is at 8 % of its ceiling, and the single-host rows are slower.
- Compressed or lower-rate sensor streams are excluded (no change to quality, resolution or rate in a measured row).
- Concurrent per-camera requests would not help: there is one request per tick and the call is 28 % of the step, bounded by GPU sharing inside the replica, not by serial sending.
