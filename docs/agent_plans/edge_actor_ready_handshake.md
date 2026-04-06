# Plan: Edge Actor-Ready Handshake

## Problem

The orchestrator's tick loop starts before actors finish initializing. The
sequence of events:

1. `run_comms()` returns after edges register with orchestrator (push_q.get #1).
2. Scenario code calls `create_edge_manager_from_scenario_runner()`.
3. Tick loop starts → `Server_DoTick` → C++ pushes tick to edge → edge pushes
   to actors.
4. Actors are still in `run()` initialization (VehicleManager creation, etc.)
   when tick_id:1 arrives.

The `push_q.empty()` assert in actor clients fires because the tick arrives
before `send_carla_data_to_ecav()` is called.

A second problem: `send_carla_data_to_ecav()` calls `Client_RegisterVehicle`
on the orchestrator. When all vehicles register, the C++ server pushes another
`TICK_ID_INVALID` to sim_api (the "duplicate tick_id:-1"). This extra push
pollutes the push_q and would corrupt the ordering of any second push_q.get().

## Desired Flow (jrapp, 2026-04-04)

1. Edges register with orchestrator → get actor list
2. Actors register with orchestrator → get edge assignment
3. Actors register with edge (`Edge_ActorRegister`) — early, for push port + scenario config
4. Actors complete initialization (VehicleManager, carla data)
5. Actors call `Edge_ActorReady` on edge — signals "I'm done"
6. Edge: once all actors ready → calls `Edge_ActorsReady` on orchestrator
7. Orchestrator: `run_comms()` push_q.get #2 → unblocks → returns
8. Tick loop starts → `Server_DoTick` → C++ pushes tick_id:1 to edge
9. Edge pushes tick to actors — actors are guaranteed ready
10. Actors call `Edge_ActorSendUpdate` → edge → `Edge_TickComplete` → orchestrator
11. Repeat from 8

The `push_q.empty()` assert in actors holds because the tick cannot arrive
before step 7, which requires step 5 for all actors.

## Changes

### Proto (`ecav/protos/ecloud.proto`)

- New message: `EdgeReadyNotification { int32 edge_index = 1; int32 num_actors = 2; }`
- New RPC (actor → edge): `Edge_ActorReady(RegistrationInfo) returns (Empty)`
- New RPC (edge → orchestrator): `Edge_ActorsReady(EdgeReadyNotification) returns (Empty)`
- New `Command` enum value: `ACTORS_READY = 5`

### C++ server (`ecav/ecloud_server/ecloud_server.cc`)

- Add `std::atomic<int> numEdgesActorReady_{0}` field
- Reset in `Server_StartScenario`
- Handle `Edge_ActorsReady`: increment counter; when all edges ready → push `TICK_ID_INVALID` with `Command::ACTORS_READY` to sim_api
  - **Important**: must use `ACTORS_READY` (not `TICK`) to avoid the `ecloud_comms` duplicate-filter dropping the push. The first push (edge registration) uses `Command::TICK`; if actors-ready also used `TICK`, the second push would be silently dropped.

### `edge_process.py`

- Add `num_actors_ready = 0` to `EdgeProcess.__init__`
- `EdgeServer.Edge_ActorReady` handler: store actor_id/vid, increment `edge_process.num_actors_ready`; when count == `expected_num_actors` → call `edge_process.notify_actors_ready()`
- `EdgeProcess.notify_actors_ready()`: calls `Edge_ActorsReady` on orchestrator

### `ecloud_actor_client.py`

- `send_carla_data_to_ecav()`: if `connected_to_edge`, call `Edge_ActorReady` on
  edge instead of `Client_RegisterVehicle` on orchestrator.
  - Eliminates the "duplicate tick_id:-1" push entirely.
  - The actor's actor_id and vid (now set after VehicleManager) are included in the request.
- Restore `assert self.push_q.empty()` before `push_q.get()` in `run()`.

### `sim_api.py`

- `run_comms()`: after `server_start_scenario` returns, add:
  ```python
  logger.info("Waiting for all edges to confirm actors ready...")
  assert self.push_q.empty(), ...
  await self.push_q.get()
  self.push_q.task_done()
  logger.info("All edges actor-ready")
  ```

### Build

- `python ecav.py --build` (regenerate Python proto stubs)
- `cd ~/eCAV/ecav/ecloud_server && make -j4` (rebuild C++ server)

## Push_q Ordering After Fix

| Event | push_q source | consumed by |
|-------|--------------|-------------|
| All edges `Edge_Register` | C++ `Edge_Register` handler | `server_start_scenario` push_q.get #1 |
| All edges `Edge_ActorsReady` | C++ `Edge_ActorsReady` handler | `run_comms()` push_q.get #2 |
| All edges `Edge_TickComplete` per tick | C++ `Edge_TickComplete` handler | `server_do_tick` push_q.get |

No extra pushes. Asserts hold at each get().

## Checklist

- [x] Proto: add `EdgeReadyNotification`, `Edge_ActorReady`, `Edge_ActorsReady`
- [x] C++: add `numEdgesActorReady_`, reset in `Server_StartScenario`, handle `Edge_ActorsReady`
- [x] Build: regenerate stubs + `make -j4`
- [x] `edge_process.py`: `Edge_ActorReady` handler + `notify_actors_ready()`
- [x] `ecloud_actor_client.py`: `send_carla_data_to_ecav()` → `Edge_ActorReady` in edge mode, restore assert
- [x] `sim_api.py`: second `push_q.get()` in `run_comms()`
- [ ] Smoke test: `docker logs` — confirm "All edges actor-ready" before first tick
