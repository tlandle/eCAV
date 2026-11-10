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
