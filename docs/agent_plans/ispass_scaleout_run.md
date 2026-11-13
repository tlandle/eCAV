# ISPASS scale-out run (distributed deployment and overhead)

Status 2026-11-10: plan and feasibility probe. No client VM started yet.

## Goal
Measure eCAV fully distributed on the current code: every actor a separate process on
separate machines in lock step, with a real edge node. Replace the scaling numbers the
ISPASS draft carries from the 2023 evaluation (256 clients on four machines, 45 ms p99
overhead; 64 clients with perception, 35 ms).

## Azure inventory (all in subscription eCloudSim_Tyler_Landle)
| VM | size | role | state 2026-11-10 |
|---|---|---|---|
| edge-server | NV36ads A10 v5, 36 cores, 440 GB | edge node (Tyler: keep for edge testing) | running |
| carla-server | NV12s v3 | world engine and orchestration server | deallocated |
| carla-client1, carla-client4 | D64s v5 | vehicle clients, no perception | deallocated |
| carla-client2, carla-client3 | D64as v4 | vehicle clients, no perception | deallocated |
| carla-gpu-client1, carla-gpu-client2 | NV12s v3 | vehicle clients with perception | deallocated |
| carla-ndm | ND40rs v2, 8 V100 | perception clients, many per host | deallocated |
Quota allows all of them at once. cloud_config.yaml still holds the 256-car address layout
(carla-server plus ndm and client1 to client4).

## Findings from the probe on the A10 node (worktree ~/scale_wt at develop c4e0e7db)
1. The A10 node cannot fetch from GitHub and has no git-lfs. Code goes there with
   `git push --no-verify ssh://azureuser@<ip>/home/azureuser/ecloudsim_distributed_sandbox develop:refs/heads/scale_develop`.
2. The 2023 scaling scenarios (`ecloud_4lane_*`, 21 YAMLs) do not run on the current entry
   point. Several fail to parse (undefined YAML alias `vehicle_base`). The ones that parse
   have no `scenario_runner` section, which ecav.py now requires
   (`scene_dict.scenario_runner.distributed`), and their .py files have no `run_vehicle`.
3. The current distributed path is `ecav.py -t <scenario> -d` for the server and
   `ecav.py -d -i <k>` per actor, where the scenario module provides `run_vehicle` and the
   scenario comes from the scenario runner. Clients fetch the scenario from the
   orchestration server at `ecloud_server_public_ip` in cloud_config.yaml.
4. Protocol stubs are not in git: run `python ecav.py --build` once per checkout.
5. ansible/ playbooks target a `vehicle-sim` image and a vault password from 2023. Treat as
   out of date. Use plain ssh scripts first.

## Work items
1. New scenario `scale_town06` on the current harness: Town06 long eastbound lanes, N
   connected vehicles from a generated spawn grid, scenario-runner based, `run_vehicle`
   present, per-actor timing breakdown written per run. N as a parameter (generator
   script, not 20 hand-written YAMLs).
2. Smoke on the A10 node alone, N = 8 and 32, host processes, no perception.
3. Multi-host launcher script (ssh, no ansible): starts server on carla-server, k clients
   per client host, edge service on the A10 node. Collects logs and the per-tick
   breakdown.
4. Sweeps: no perception N = 16 to 512 on 1, 2, 4 client hosts. With perception N = 8 to
   128 using the GPU clients and NDm. One run with the edge service on the A10 node and
   vehicles elsewhere (real edge deployment).
5. Report per tick: barrier wait, serialization, network, client step, world step. p50 and
   p99. Weak and strong scaling figures for the paper.
6. Deallocate every VM in the block that ends the sweep. The agent's `az vm deallocate`
   was permission-blocked on 2026-11-09, so Tyler runs it or adds a permission rule.

## Cost note
Pay-as-you-go list prices, approximate: D64s v5 3.07 USD/h, D64as v4 3.07, NV12s v3 1.14,
ND40rs v2 22.03, A10 3.11 (observed). All clients, server and the A10 node without NDm is
about 20 USD/h. NDm doubles it. Start NDm only for the perception sweep.

## Probe log, 2026-11-10 12:29 (single-host distributed smoke on the A10 node)
Scenario openscenario_3_edge_late_fusion_4ego, host processes, script ~/dist_smoke.sh on the node.
Setup defects found and handled on a fresh checkout:
- scenario_runner is a submodule and is empty in a new worktree. Linked from ~/sensys_wt.
- PYTHONPATH must include ~/carla-0.9.15/PythonAPI/carla (module agents).
- ecav.py --build compiled only ecloud.proto. Fixed on develop: compiles every proto.
- ecav/ecloud_server/ecloud_server in git is an LFS pointer and the node has no gRPC C++ toolchain. The atlas binary runs there (both Ubuntu 24.04).
- edge_process.py takes no -t. scripts/launch_distributed.sh is stale on that point and on the conda env name.
State reached: orchestration server up, four vehicles registered, edge process registered actor 0 of 6 and waits ("Waiting for all edges to confirm actors ready"). Vehicle 0's scenario runner exited and dumped core. The two RSU actors were not launched by the smoke script.
Next: launch the RSU actors the way start_actors.sh does, find why vehicle 0's scenario runner exits, get one full distributed run with ticks, then build scale_town06.

## Probe log, 2026-11-11 21:52 (scale_town06 built, three single-host smokes on the A10 node)
Work items 1 to 3 are done on one host.

Files:
- `scripts/ispass/gen_scale_town06.py -n N --rsus R --perception on|off [--carla-host IP]` writes `config_yaml/scale_town06.yaml` and `scenarios/scenario_scale_town06.xml` (overwritten per configuration; the committed copy is N=8, perception off). Slots come from Town06.xodr offline: straight non-junction lanes, 12 m apart, nearest to (350, 145) first, each with a straight route of at least 150 m. 459 slots at 12 m. Above that the generator falls back to 10 m and 120 m (606 slots), so N=512 fits. Blocks come from `config_yaml/scale_town06_base.yaml` (a copy of the highway merge YAML, because test_runner patches the live one during campaigns).
- `ecav/scenario_testing/scenarios/scenario_scale_town06.py` (no scripted actors) and `ecav/scenario_testing/scale_town06.py` (runner: `run_scenario`, `run_vehicle`; runs SCENARIO_MAX_STEP ticks, default 200; logs `[SCALE_TOWN06] tick k wall= clients= world=` and a SUMMARY line).
- `scripts/ispass/launch_scale.sh -n N -r R -p on|off -o ORCH [-c CARLA] [-e EDGE] [-R RSUHOST] [-H "host:lo-hi,..."]` starts CARLA if port 2000 is closed, generates on the orchestrator, copies YAML and XML to the other hosts, starts orchestrator, edge, client 0 (spawns every ego), the other clients in waves of 16, RSU clients, waits on log lines, collects logs to `experiment_results/ispass_scale/<tag>/<host>/`. Env knobs SCALE_WT, SCALE_ACT, SCALE_CARLA_ROOT (atlas exports CARLA_ROOT, so the knob has its own name).
- `scripts/ispass/scale_breakdown.py <results dir>` prints the per-tick breakdown.

Defects fixed:
- scenario_runner submodule (527c028, pushed to tlandle/scenario_runner distributed-integration, pointer bumped): `scenario_manager.py` created the actor client only for index 0, so clients k >= 1 never drove a vehicle. Every client now drives `ego_vehicles[k]`. `scenario_runner.py` cleanup destroyed every ego from every client. Only client 0 destroys them now.
- `ecav/ecav2/edge_process.py`: the edge pushed ticks to `localhost:<port>` for every actor, so edge and actors had to share a host. It uses the actor's registered address now.

Results, A10 node (36 cores), everything on one host, 200 ticks, first 5 excluded, ms:

| run | tick wall mean / p50 / p99 | server wait_clients mean | vehicle client compute mean (update_info + run_step) | world tick | time to first tick |
|---|---|---|---|---|---|
| N=8, 2 RSUs, no perception | 76 / 68 / 300 | 71 | 31 (9 + 22) | 3 | 86 s |
| N=32, 2 RSUs, no perception | 398 / 465 / 948 | 384 | 105 (45 + 60) | 8 | 208 s |
| N=8, 2 RSUs, perception | 471 / 458 / 709 | 464 | 392 (288 + 103) | 3 | 87 s |

All clients ticked 200 times in every run, no collision, orchestrator exit 0. Server send_cmd 1 to 3 ms, unpack 1 ms. RSU clients 3 ms without perception, 276 ms with.

Open points for the sweep:
- At N=32 the barrier wait (384 ms) is 3.6 times the client compute (105 ms), and client compute itself tripled against N=8 on a 36-core host. The edge process and CPU contention are the candidates. The edge logs no per-tick time. Add one before the sweep, otherwise the breakdown cannot attribute the wait.
- Orchestrator start-up builds one proxy VehicleManager per vehicle, about 3 s each at N=32 (first tick after 208 s). N=512 would take about half an hour to start unless that loop is trimmed.
- Push ports: vehicle k listens on 50101+k and RSU j on 50201+j, so vehicle 100 and RSU 0 want the same port on one host. Keep RSU clients on a host with fewer than 100 vehicles, or change the RSU base.
- sshd on the node resets connections when many start at once. The launcher uses one multiplexed connection per host.
- Teardown noise, harmless: edge_process core dump after cleanup, grpc aio `__del__` TypeError in RSU clients.

Multi-host: every VM needs the worktree at $HOME/scale_wt (git push to the VM, or rsync from the node, since VMs have no GitHub access), the scenario_runner submodule content at 527c028, sort and ecav/worldfusion submodule content, `python ecav.py --build` for the stubs, the opencda310 env, CARLA 0.9.15 PythonAPI under $HOME/carla-0.9.15 (clients need only PythonAPI/carla and Town06.xodr on the orchestrator), and the ecloud_server binary on the orchestrator. Ports between VMs: 2000 to 2002, 50051, 50061, 50054, 50101 to 50101+N, 50201 to 50201+R. The launcher rewrites cloud_config.yaml on every host from private IPs (carla_server_public_ip, ecloud_server_public_ip, vehicle_client_public_ip, edge_server_public_ip) and keeps the original as cloud_config.yaml.single_host. The multi-host path of the launcher is written and not yet run.

## Probe log, 2026-11-11 23:34 (scale defects closed, single-host baseline sweep on the A10 node)

Code version for the whole sweep: develop 8effe0e7 (node files checked against it by checksum).
Results: `experiment_results/ispass_scale/sweep2_n{8,16,32,64,128}` on atlas. No perception,
exact-state edge manager, 2 RSUs, 300 ticks, 30 km/h, everything on the A10 node (36 cores).
No vehicle reported done early and no collision was logged in any run.

Tick wall time and its partition along the critical path (the client that reports last), ms, means
over ticks 5 to 299. `scripts/ispass/scale_breakdown.py <dir>` prints one run, `--row` one row.

| N | tick wall mean / p50 / p99 | orch to edge | edge fan-out | client loop gap | client compute (last reporter) | client reply | edge step | edge to orch | unpack | world tick | unattributed | client compute, all clients | first tick (s) | CPU busy | load avg |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 8 | 72 / 64 / 294 | 2 | 9 | 2 | 37 | 2 | 15 | 2 | 1 | 3 | 0 | 31 | 52 | 21 % | 17 |
| 16 | 127 / 110 / 330 | 2 | 17 | 2 | 63 | 4 | 31 | 2 | 1 | 4 | 0 | 49 | 49 | 29 % | 8 |
| 32 | 283 / 257 / 687 | 3 | 14 | 2 | 123 | 25 | 106 | 2 | 1 | 8 | 0 | 107 | 68 | 47 % | 20 |
| 64 | 888 / 877 / 1463 | 5 | 13 | 4 | 435 | 48 | 366 | 2 | 1 | 13 | 0 | 369 | 103 | 56 % | 38 |
| 128 | 3910 / 4096 / 5099 | 7 | 31 | 6 | 2249 | 126 | 1462 | 2 | 1 | 27 | 0 | 2055 | 176 | 66 % | 73 |

(The load average of the N=8 row still carries the run before it. N=256 was not run: the host is
past its core count at N=64.)

Reading:
- The orchestration path itself (orchestrator to edge, fan-out, edge to orchestrator, unpack) is 14 ms
  at N=8 and 41 ms at N=128, about 1 % of the tick at N=128. CARLA's world tick is 3 to 27 ms.
- The tick is two serial phases: the clients compute in parallel, then the edge process runs one
  single-threaded step. Client compute per vehicle grows with N in this scenario because the fleet
  is one dense block (12 m slots, 5 lanes): every client tracks and plans against most of the other
  vehicles, and beyond 36 clients they share cores. The edge step (exact-state manager: tracker,
  forecast, one pickled forecast list per recipient) grows faster than linearly for the same reason.
- CPU busy stays below 70 % at N=128 because the clients idle during the edge step and the edge
  process idles during the client phase.
- More hosts will cut the client phase only. The edge step (1.5 s at N=128) is one Python process and
  will not move with client hosts. That is the next bottleneck for the multi-host sweep, and the
  place to look is per-recipient serialization in the edge manager's run_step (py-spy at N=32:
  pickle.dumps of each recipient's list and the tracker).

Defects closed in this block:
1. Barrier wait attributed. Orchestrator (`sim_api.py` server_do_tick), edge (`edge_process.py`
   process_tick) and clients (`ecloud_actor_client.py` tick) log absolute times per tick, and the
   breakdown follows the last-reporting client. Unattributed time is under 1 ms at every N. The
   earlier 384 ms wait against 105 ms client compute at N=32 was the edge step, which no timer covered.
2. Real inefficiency in the edge step: `edge_process.py` run_edge_step unpickled every recipient's
   forecast buffer to count predictions for one log line, 59 % of the edge step at N=32 (py-spy).
   It now counts the first buffer only. N=32 before and after, same day, 200 ticks: tick wall 400 to
   271 ms, edge step 238 to 102 ms (runs probe1_n32 and probe4_n32).
3. Start-up. `MapManager.__init__` sampled every Town06 lane at 0.1 m for each VehicleManager, 2 s
   each, in the orchestrator's and the edge's per-vehicle proxy loops (78 s and 64 s at N=32). The
   result is cached per process (`ecav/core/map/map_manager.py`). The orchestrator's per-vehicle dump
   of every actor (`sim_api.py` SPAWN TICK, quadratic) now runs only up to 8 vehicles. Time to first
   tick at N=32: 223 s to 68 s. At N=128 it is 176 s, of which 130 s is starting the clients in waves
   of 16; the estimate for N=512 on one host is about 10 minutes, less with clients on more hosts.
4. File descriptors. The orchestrator opened one CARLA client per proxy VehicleManager and failed at
   the 98th vehicle (epoll: Too many open files). Proxies now share the orchestrator's world
   (`vehicle_manager.py`), and the launcher raises the limit to 65536 and stops when the orchestrator
   logs an exception before the first tick.
5. Push ports. RSU j now listens on 51101+j (vehicle k on 50101+k). A second fault showed at N=32:
   the push ports are inside the kernel's ephemeral range (32768 to 60999), an outgoing connection
   held 50118, client 17 fell back to port + 1 and shared client 18's port (SO_REUSEPORT), and the
   run stalled at 30 s per tick. `ecloud_comms.py` now disables port sharing, falls back to a free
   OS-assigned port, and the client registers the port the server bound.

Multi-host run must know:
- On every host: `sudo sysctl -w net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300` (set on the
  A10 node, not persisted across reboot) so outgoing connections never take a service port. With the
  fallback a client can still land on an OS-assigned port, so hosts need all TCP ports open to each
  other inside the virtual network, not only the listed ranges.
- scale_breakdown.py compares time.time() across processes: hosts need synchronised clocks (chrony)
  or the cross-host rows (fan-out, reply) are off by the clock offset. Sums stay correct.
- py-spy in its default blocking mode stalls a run (14 s ticks); use `--nonblocking -r 50`.
- With perception on, each client sleeps index + 1 seconds before building its VehicleManager
  (`ecloud_actor_client.py:159`), 128 s at N=128; not changed.
- Node-only state: worktree files delivered by scp, py-spy installed in opencda310, ~/spy.sh,
  ~/spy_top.py, ~/prof_vm.py. CARLA and all run processes on the node are stopped.

## Probe log, 2026-11-12 03:05 (superlinear terms fixed, two layouts, sequential baseline, perception rows; sweep3 on the A10 node)

Code: develop b5a0d526 for the sweep (dense N=8 to 128 ran at d90af688/c32fa70f; the later commits change the
generator only for slot lists that contain overlapping slots, the N<=128 dense lists are byte-identical, checked
by checksum, the CARLA timeout, and the sweep script). Results: experiment_results/ispass_scale/sweep3_<config>_n<N>,
table paper_ispass/scale_sweep3.md, data paper_ispass/scale_sweep3.json, figure paper_ispass/fig_scale_sweep3.pdf
(scripts/ispass/scale_report.py sweep3). Host: A10 node, 36 vCPUs (18 cores), 432 GB, everything on one host.
300 ticks up to N=128, 100 ticks at 256. Times are means in ms over the ticks after the first 5.

| configuration | N | RSUs | in sensing range | in forecast range | tick wall mean / p50 / p99 (ms) | sim s per wall s | clients | edge step | transport | world | client compute, all (ms) | first tick (s) | CPU busy | GPU util / mem | inference service p50 / p95 / p99 (ms), requests per wall s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| distributed, dense | 8 | 2 | 7.0 | 7.0 | 66 / 60 / 274 | 0.759 | 40 | 11 | 12 | 3 | 32 | 50 | 22 % | - | - |
| distributed, dense | 16 | 2 | 15.0 | 15.0 | 104 / 89 / 334 | 0.480 | 58 | 19 | 22 | 4 | 44 | 50 | 31 % | - | - |
| distributed, dense | 32 | 2 | 25.2 | 31.0 | 210 / 159 / 460 | 0.239 | 99 | 50 | 53 | 8 | 76 | 59 | 46 % | - | - |
| distributed, dense | 64 | 2 | 30.4 | 62.4 | 435 / 389 / 792 | 0.115 | 211 | 119 | 91 | 14 | 173 | 89 | 59 % | - | - |
| distributed, dense | 128 | 2 | 29.3 | 108.6 | 1066 / 1078 / 1548 | 0.047 | 515 | 350 | 174 | 27 | 428 | 144 | 63 % | - | - |
| distributed, dense | 256 | 2 | 34.7 | 146.2 | 2383 / 2398 / 3485 | 0.021 | 1130 | 858 | 340 | 55 | 1006 | 295 | 64 % | - | - |
| distributed, spread | 8 | 6 | 0.0 | 1.8 | 55 / 53 / 249 | 0.916 | 32 | 13 | 7 | 3 | 24 | 53 | 23 % | - | - |
| distributed, spread | 16 | 9 | 0.0 | 3.9 | 91 / 80 / 307 | 0.551 | 42 | 27 | 17 | 4 | 31 | 54 | 30 % | - | - |
| distributed, spread | 32 | 10 | 1.4 | 9.0 | 177 / 144 / 397 | 0.282 | 55 | 64 | 49 | 8 | 50 | 70 | 39 % | - | - |
| distributed, spread | 64 | 11 | 3.7 | 18.1 | 382 / 347 / 764 | 0.131 | 130 | 150 | 88 | 14 | 114 | 94 | 48 % | - | - |
| distributed, spread | 128 | 14 | 7.9 | 35.9 | 997 / 1039 / 1573 | 0.050 | 391 | 426 | 153 | 28 | 326 | 155 | 53 % | - | - |
| distributed, spread | 256 | 14 | 16.8 | 71.9 | 2774 / 2910 / 4384 | 0.018 | 1153 | 1194 | 358 | 69 | 940 | 300 | 58 % | - | - |
| sequential, dense | 8 | - | 7.0 | 7.0 | 201 / 202 / 412 | 0.248 | - | 199 (with vehicles) | - | 3 | - | 19 | 9 % | - | - |
| sequential, dense | 16 | - | 15.0 | 15.0 | 483 / 474 / 781 | 0.104 | - | 479 (with vehicles) | - | 4 | - | 21 | 9 % | - | - |
| sequential, dense | 32 | - | 25.2 | 31.0 | 1377 / 1348 / 1773 | 0.036 | - | 1370 (with vehicles) | - | 7 | - | 23 | 9 % | - | - |
| sequential, spread | 8 | - | 0.0 | 1.8 | 135 / 129 / 188 | 0.371 | - | 133 (with vehicles) | - | 2 | - | 19 | 9 % | - | - |
| sequential, spread | 16 | - | 0.0 | 3.9 | 280 / 273 / 553 | 0.178 | - | 277 (with vehicles) | - | 3 | - | 19 | 9 % | - | - |
| sequential, spread | 32 | - | 1.4 | 9.0 | 687 / 674 / 1006 | 0.073 | - | 680 (with vehicles) | - | 6 | - | 21 | 9 % | - | - |
| distributed, dense, perception | 8 | 2 | 7.0 | 7.0 | 463 / 462 / 704 | 0.108 | 407 | 29 | 23 | 3 | 312 | 56 | 30 % | 47 % / 9.8 GB | 144 / 174 / 207, 21.7 |
| distributed, dense, perception | 16 | 2 | 15.0 | 15.0 | 828 / 827 / 1141 | 0.060 | 720 | 73 | 29 | 5 | 490 | 67 | 32 % | 57 % / 11.0 GB | 146 / 169 / 199, 21.8 |
| distributed, dense, perception | 32 | 2 | 25.2 | 31.0 | 1591 / 1616 / 1903 | 0.031 | 1331 | 211 | 40 | 9 | 849 | 117 | 35 % | 58 % / 13.1 GB | 146 / 166 / 192, 21.4 |

Columns "clients", "edge step", "transport", "world" partition the tick along the last-reporting client.
"In sensing range" and "in forecast range" are the mean number of other vehicles within 50 m and 150 m of a vehicle.

### What was fixed (before to after, dense layout)

| N | tick wall before (sweep2) | after (sweep3) | edge step | client compute, last reporter |
|---|---|---|---|---|
| 32 | 283 | 210 | 106 to 50 | 123 to 97 |
| 64 | 888 | 435 | 366 to 119 | 435 to 203 |
| 128 | 3910 | 1066 | 1462 to 350 | 2249 to 510 |

1. Forecasts serialised once per tick (ecav/ecav2/pred_blobs.py, exact-state manager distribution step). Each
   recipient's buffer was pickle.dumps of its own list, N*(N-1) forecast objects per tick, 48 % of the edge step at
   N=64. A recipient's buffer is now the concatenation of forecasts pickled once. The client reads both forms.
   N=64 dense, 200 ticks: tick 562 to 445 ms, edge step 266 to 125 ms (fixA_n64, fixB_n64).
2. Sensing range for vehicles without sensors (perception YAML deactivate_range_m, default unchanged: unlimited).
   deactivate_mode listed every vehicle in the world (hard-coded 100000 m), so each client tracked, uploaded and
   collision-checked N-1 objects. The scale YAML sets the vehicle lidar range of the base YAML, 50 m, which is what
   a vehicle with sensors sees. N=32 dense: client compute, all clients, 111 to 79 ms (prof1_n32, fixA_n32).
3. Recipient range at the edge (edge config recipient_range_m, default unset: every forecast to every vehicle).
   The scale YAML sets 150 m: the 54 m planned path plus two vehicles closing at the speed limit over the 3 s
   forecast, with margin. Not matched against a sequential campaign setting: the junction campaigns have no gate
   because a locale there is smaller than 150 m. Both gates are in the YAML, so the sequential rows use them too.
4. File logging: every process wrote a debug-level log file (cloud_config file_log_level), hundreds of MB per
   client per run, about 12 % of a client step. The launcher sets warning. N=64 dense: 445 to 422 ms.
5. Spawn slots that overlap where roads join are dropped (CARLA refused one spawn at N=256).
6. Thread limit per process (OMP/OPENBLAS/MKL = 1): no measurable change (prof2/prof3 spread N=64), kept.

### What limits each workload class (single host)

- No perception, distributed: two serial phases. Client phase: per-client work is about 25 to 30 ms when a
  vehicle has few neighbours (spread N=8, 16) and grows with the neighbours in range in the dense layout; beyond
  18 clients the clients share 18 cores, so client compute per client rises with N in both layouts (spread: 24, 31,
  50, 114, 326, 940 ms). More client hosts remove that term. Edge phase: one single-threaded process; its
  largest part after the fix is the tracker over all N ground-truth boxes (association is quadratic in N), then
  the forecast generation and the per-tick metrics. The edge step does not shrink with client hosts and is
  larger than the client phase from N=64 in the spread layout.
- The spread layout is only 10 to 20 % faster than dense: neighbours in range are not the main cost at this
  scale, core sharing and the single edge are.
- Sequential single process: 201, 483, 1377 ms (dense) and 135, 280, 687 ms (spread) at N=8, 16, 32. Distributed on
  the same host is 3.0, 4.6, 6.6 times faster (dense) and 2.5, 3.1, 3.9 times faster (spread).
- Perception, shared YOLO service: the tick is the client phase (88 % at N=8). The service delivers about 21.6
  requests per wall second at every N (4 worker threads, p50 145 ms per 4-camera request under load, 72 ms
  unloaded), GPU utilisation 47 to 58 %, 9.8 to 13.1 GB. The service's worker threads, not the GPU, cap the rate,
  so the tick grows as (N + 2 RSUs) / 21.6 s.
- Start-up: 50 to 300 s for N=8 to 256, linear in N (clients start in waves of 16).
- N=512 does not run on this host. All 512 vehicle and 15 RSU clients started (623 s) and registered, then the
  host ran out of memory (about 0.8 GB per client process, 415 of 432 GB used) and the kernel killed CARLA. Three
  attempts, the first two ended on the orchestrator's 10 s CARLA timeout (now ECAV_CARLA_TIMEOUT_S, 60 s in scale
  runs), the third showed the out-of-memory kill in dmesg.
- More than one edge is not supported in distributed mode: the C++ server keeps vehicle-to-edge mappings, but
  vehicle_manager.py (lines 204 to 266), ecloud_actor_client.py (157 to 183), sim_api.py (746 to 749) and
  scale_town06.py (80) read edge_list[0]. The sequential path runs the multi-edge scenarios. One edge per region
  is the change that would cut the edge step.

### Multi-host run, prepared, not run

Blocked on access (client VMs accept only a password; adding a key was refused) and on a network path (the A10
node is alone in edge-server-vnet 10.1.0.0/16; carla-client3 and carla-client4 are in
carla-server_group_10111734-vnet 10.0.0.0/16; no peering). Steps once access exists:

1. Network: peer the two virtual networks both ways, for example
   `az network vnet peering create -g CARLA-SERVER_GROUP -n edge-to-clients --vnet-name edge-server-vnet --remote-vnet <id of carla-server_group_10111734-vnet> --allow-vnet-access`
   and the reverse from resource group CARLA-SERVER_GROUP_10111734. The launcher uses private addresses
   (hostname -I) and rewrites cloud_config.yaml on every host.
2. Per VM: `scripts/ispass/setup_scale_vm.sh azureuser@<vm>` (worktree, CARLA PythonAPI and the Python env
   streamed from the A10 node through atlas, reserved ports, file limit, clock check, import test). Untested.
3. On the A10 node once per boot: `sudo sysctl -w net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300`.
4. Run, A10 node as CARLA, orchestrator and edge, clients on the two D64 VMs (64 vCPUs each):
   `scripts/ispass/launch_scale.sh -n 128 -r 20 -p off -s 300 -g "--layout spread" -o azureuser@20.81.176.215 -H "azureuser@<client3>:0-63,azureuser@<client4>:64-127" -t mh_spread_n128`
   then N=256 with 0-127 and 128-255, and the dense layout without -g. Report with scale_report.py after naming the
   tags <stamp>_<config>_n<N> (a multi-host configuration name must be added to CONFIGS there).
5. Expect the client phase to fall to the per-client cost (25 to 80 ms) and the edge step to remain: 426 ms at
   N=128 and 1.2 s at N=256 (spread), which then is the tick.

## Multi-host run, hosts prepared, 2026-11-12 15:22 (not run: the A10 node is busy with the competitor benchmark)

### State
- Peering edge-server-vnet (10.1.0.0/16) with carla-server_group_10111734-vnet (10.0.0.0/16) is Connected and in sync in both directions (peerings edge-to-clients and clients-to-edge).
- carla-client3 (172.200.91.133, private 10.0.0.11) and carla-client4 (20.242.82.41, private 10.0.0.12): 64 cores, 251 GB, Ubuntu 20.04, atlas's key installed. Each has
  - `~/scale_wt`, a copy of the A10 node's worktree (sweep3 code) without logs, results, checkpoints and docs,
  - `~/carla-0.9.15/PythonAPI`,
  - `~/miniconda3/envs/opencda310`, a copy of the A10 node's env without the CUDA toolkit (9.7 GB, symlinks kept). There is no conda base install. The launcher puts the env's bin directory on PATH. The env imports on Ubuntu 20.04 (carla, grpc, numpy, scipy, cv2, torch, shapely, py_trees, omegaconf, the proto stubs).
  - reserved ports 50051-50070 and 50101-51300 in /etc/sysctl.d/90-scale.conf (persistent), file limit 65536, chrony in sync.
  - Root disk 30 GB, 6.6 GB free after the install. client4 has no temporary disk. client3 has one at /mnt, not used (a deallocation erases it).
- A vehicle client started on each VM (`python -u ecav.py -v 0.9.15 -d -i 1 -T 8001`) reaches its scenario request to 10.1.0.4:50051 and is refused, as expected with no orchestrator running.

### Defect found and fixed
`ecav.py` requested the scenario from a hard-coded localhost:50051, so a vehicle client on another machine could never start. The address now comes from cloud_config.yaml's ecloud_server_public_ip (the key the actor clients already read). The fixed file is on the A10 node and both clients. The multi-host rows therefore differ from sweep3 in this one file; on one host the address is localhost as before.

### Ports and directions (verified in code; TCP tested both ways across the peering on unused ports 47001 to 47003)
| listener | port | host | who connects |
|---|---|---|---|
| CARLA | 2000 to 2002 | A10 | every vehicle and RSU client, orchestrator, scenario runner |
| orchestration server (C++) | 50051 | A10 | every client, edge process |
| orchestrator push API | 50061 | A10 | orchestration server (local) |
| edge process | 50054 | A10 | clients it serves |
| vehicle push | 50101 + k | host of client k | orchestration server, edge process |
| RSU push | 51101 + j | RSU host (A10) | orchestration server, edge process |
| shared YOLO inference | 18001 | A10 | every client with `-p shared` |
- No rule was added. edge-server-nsg allows only 22, 80 and 443 from the internet, and the default AllowVnetInBound rule covers the peered range, which the listener test confirmed (client to A10, A10 to client, client to client). No host firewall is active on any of the three (ufw inactive).
- carla-client3-nsg and carla-client4-nsg carry a rule AllowAnyCustomAnyInbound (priority 310, any source, any port). The rule predates this work. It exposes every port of both clients to the internet while they run. Not changed; Tyler's call.

### Launcher changes
- `launch_scale.sh`: a host without a conda install uses the env through PATH; `-y HOST` names the inference service host, and with several hosts and `-p shared` the scenario YAML's yolo_endpoint is rewritten to that host's private address (it was localhost, which a remote client cannot use). Remote start by ssh, per-host index ranges (`-H`), the cloud_config rewrite from private addresses and cleanup by PID (`^python -u ecav`, `^./ecav/ecloud_server/ecloud_server`) already existed.
- `scale_sweep.sh`: parts mhdense, mhspread, mhpercep split the vehicle clients evenly over CLIENT_HOSTS. `scale_report.py` knows the three configurations.
- `setup_scale_vm.sh`: fixed (tar warnings no longer abort it, env copied without the CUDA toolkit and with symlinks kept, persistent sysctl).

### Run (once the A10 node has no simulator running)
```
ssh azureuser@20.81.176.215 'sudo sysctl -w net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300'
CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" MH_NS="64" scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 mhdense      # smoke, one run
CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 "mhdense mhspread mhpercep"
/home/atlas/anaconda3/envs/opencda310/bin/python scripts/ispass/scale_report.py mh1
```
Defaults: 64, 128, 256, 512 vehicles without perception, 8 to 64 with the shared service. The sweep exits early for a tag that already finished.

### Not yet verified (needs the first run)
- The orchestration server and the edge process pushing to a client on another host (the address path exists and was used in 2023; untested on this code).
- Memory at 512 vehicles: 256 clients per VM against 251 GB.
- Client index 0 spawns every ego and runs on carla-client3, so spawn time includes the 1 to 2 ms path to CARLA.
- After a VM is deallocated and started again the install persists (root disk). The A10 node's sysctl does not persist; the first command above sets it.

## Perception replicas on carla-ndm, 2026-11-12 21:38 (set up and measured standalone; the multi-host sweep has not run)

### State
- carla-ndm (azureuser@20.172.248.156, private 10.2.0.6, 8 V100 32 GB, 40 cores, Ubuntu 20.04) holds `~/scale_wt` (a copy of the A10 node's worktree with `yolov5/` and `yolov5m.pt`, sha256 61d93336..., the same file as on the A10) and `~/scale_pydeps` (grpcio 1.80.0, protobuf 6.33.6, seaborn, psutil, setuptools 75.8.0 installed with `pip --target`; the training env `opencda310` on that host is not modified and lacks grpcio). Both are on the root disk and survive a deallocation. Nothing is on a temporary disk.
- Its virtual network was re-addressed to 10.2.0.0/16 and peered with the A10 node's network (10.1.0.0/16) and the client network (10.0.0.0/16). TCP to 10.2.0.6:18001 and :18008 connects from the A10 node and from carla-client3; round-trip time about 5.3 ms.
- The service listens on every interface on ports 18001 to 18008. The NSG on the NIC (carlandmnsg981) allows inbound from the internet only on 22, 8000, 8787, 50051-50053 and 50101-50512, so the replicas are reachable from the peered networks only. No rule was added.

### Standalone ceiling (paper_ispass/perception_replicas.md, .json)
One replica per GPU, four worker threads, the request of a scale_town06 client (four 640x480 JPEG frames, 412 KB), eight closed-loop clients per replica: 36.6 requests per s on 1 GPU, 73.8 on 2, 146.9 on 4, 291.1 on 8, p50 218 to 222 ms at that load. A replica saturates at four clients. From carla-client3 across the peering: 36.6 and 290.7 requests per s. The A10 node's standalone ceiling is not measured (the 21.6 requests per s of sweep3 is an in-run rate).

### Launcher changes
- `ecav/ml_manager/ml_manager.py`: `ECAV_YOLO_ENDPOINT` in a client's environment overrides the scenario YAML's `yolo_endpoint`. Unset, the behaviour is unchanged.
- `scripts/ispass/launch_scale.sh -Y "ip:port,ip:port"`: vehicle client k and RSU client j use endpoint k mod R and j mod R. Without `-Y` nothing changes.
- `scripts/ispass/perception_replicas.sh start|stop|collect`: R replicas on one host, replica g on GPU g, port 18001+g, GPU sampling once per second.
- `scripts/ispass/scale_sweep.sh` part `mhprepl`: R in MH_RS (1 2 4 8) by N in MH_RN (8 16 32 64 128), runs `<stamp>_mhpr<R>_n<N>`.
- `scripts/ispass/scale_report.py`: configurations mhpr1, mhpr2, mhpr4, mhpr8, a service replicas column, GPU utilisation from the replicas' sample file.
- `scripts/ispass/perception_service_bench.py`: the standalone load test.

### Run (the A10 node must have no simulator running; start carla-ndm, carla-client3 and carla-client4 first)
```
# the client VMs were set up before these changes: deliver the one changed client file
for h in azureuser@172.200.91.133 azureuser@20.242.82.41; do rsync -aR ecav/ml_manager/ml_manager.py $h:scale_wt/; done
ssh azureuser@20.81.176.215 'sudo sysctl -w net.ipv4.ip_local_reserved_ports=50051-50070,50101-51300'
# smoke: 8 vehicles, 2 replicas
CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" PERC_HOST=azureuser@20.172.248.156 MH_RS="2" MH_RN="8" scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 mhprepl
# full: R = 1 2 4 8 by N = 8 16 32 64 128
CLIENT_HOSTS="azureuser@172.200.91.133 azureuser@20.242.82.41" PERC_HOST=azureuser@20.172.248.156 scripts/ispass/scale_sweep.sh mh1 azureuser@20.81.176.215 mhprepl
/home/atlas/anaconda3/envs/opencda310/bin/python scripts/ispass/scale_report.py mh1
# standalone ceiling of the A10 node, for the per-GPU comparison (GPU must be idle)
scripts/ispass/perception_replicas.sh start azureuser@20.81.176.215 1
ssh azureuser@20.81.176.215 'cd ~/scale_wt && source ~/miniconda3/etc/profile.d/conda.sh && conda activate opencda310 && python scripts/ispass/perception_service_bench.py --endpoints localhost:18001 --clients 8 --seconds 25'
scripts/ispass/perception_replicas.sh stop azureuser@20.81.176.215
```

### Not yet verified
- No closed-loop run has used `-Y`. The endpoint assignment was tested with the launcher's own command string in a local shell, and the override in the client is a single environment read.
- With the service on another host, each CARLA-side sensor frame still reaches the client first. The RSU clients run on the A10 node and call the replicas across the peering.
- At 128 vehicles with perception the limit may move to the client hosts (64 clients each with four cameras and a LiDAR) or to CARLA rendering on the A10 node's single GPU, not to the service. The breakdown columns will show which.
- carla-client1 and carla-client2 are in the re-addressed network (10.2.0.4, 10.2.0.5) and reachable by peering once started, but hold no setup and no key of mine.
