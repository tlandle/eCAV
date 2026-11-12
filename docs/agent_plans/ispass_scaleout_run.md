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
