---
updated: 2026-04-26
---
# Current State

Primary context-switching artifact. Read this first after a gap.

## Active Branch

`distributed-integration` → PR target: `ecav_2_distributed`

---

## Milestone: Distributed Perception Fast and Fully Operational (2026-04-06)

Both fusion modes confirmed working end-to-end in distributed mode, with production-quality latency.

### Late Fusion (BM2CP)
- Actor client → gRPC → C++ server → sim_api → edge_process → BM2CP model
- YOLO detection: `yolo_grpc_server.py` (port 18001), standalone, in-process inference
- Verified working.

### Intermediate Fusion (WorldFusion)
- Same actor path + feature extraction via `worldfusion_grpc_server.py` (port 18002)
- `grpc_ms ≈ server_total_ms ± 5-8ms` — IPC overhead fully eliminated
- Vehicle: ~130ms total e2e. RSU: ~140ms best case, ~200ms when GPU-contended with vehicle.
- RSU bimodal is pure CUDA serialization in distributed mode (two independent batch=1 calls). Not a transport issue.

### Infrastructure
- `start_actors.sh`: log-based container readiness (replaces static sleeps)
  - Edge: waits for `"registered successfully"` in docker logs
  - Vehicle/RSU: waits for `"Registered with"` (covers both edge and direct-to-orchestrator paths)

---

## Milestone: Sequential WorldFusion Unblocked (2026-04-06)

`openscenario_3_edge_worldfusion -l` (without `-d`) was deadlocked at "waiting for actors".

**Root cause**: `scenario_runner/scenario_runner.py:_prepare_ego_vehicles` only spawned the ego vehicle when `vehicle_index == 0` (the distributed-mode non-ego container convention). In sequential mode `vehicle_index = -2`, so the subprocess fell into the `else` branch and looped forever waiting for an ego nobody would spawn. This broke when `vehicle_index` was changed from `0` to `-2` in commit `3b9dcdb`.

**Fix**: Also spawn when `distributed=False`, regardless of `vehicle_index`. Distributed behavior unchanged. Committed to scenario_runner submodule (`34488e0`) and bumped in main repo (`925d526`).

**Status**: Proceeds past actor discovery. Secondary CARLA `rpc::timeout` errors on `_initialize_actors` appear on a dirty CARLA session (leftover state from the stuck run); clears on fresh CARLA restart.

**Verified**: `python ecav.py -t openscenario_3_edge_worldfusion --apply_ml` runs clean end-to-end. O5 batch=2 sequential path confirmed working.

---

## WIP: WorldFusion Live Detection Debugging (2026-04-26)

Active branch: `distributed-integration`. Running `openscenario_3_edge_worldfusion --apply_ml` standalone (sequential mode, no `-d`).

### Two Separate Bugs in `edge_manager_worldfusion_ab3dmot_linear_predictor.py`

**Tyler's fix (commit `c36e6084`, 2026-04-20)** — "align pairwise transform with Multi-V2X dataset, correct detection origin":
- Pairwise transform destination was wrong: `x1_to_x2(pose_j, self.world_anchor)` → now `x1_to_x2(pose_j, [0,0,0,0,0,0])`. The destination must be true world origin, not the RSU's CARLA position.
- `lidar_pose` and `world_anchor` passed to post-processor were `[self.world_anchor]` → now `[[0,0,0,0,0,0]]`. Post-processor must see origin as the reference to decode boxes in RSU-local frame.
- Final coordinate offset: was hardcoded `+= self.world_anchor[0/1/2]` → now reads from RSU localizer's actual position each tick.

**Our fix (2026-04-26, uncommitted)** — agent ordering:
- Original code collected vehicles first, RSUs second → vehicle was agent 0, RSU was agent 1.
- WorldFusion's fusion layer is ego-centric: it warps all other agents' BEV features into agent 0's frame. Output is in agent 0's coordinate frame.
- Post-processor uses `lidar_pose=[0,0,0,0,0,0]` (origin) as the reference. For this to be consistent, agent 0 must be the entity we define as origin — the RSU.
- Fix: collect RSUs first (agent 0), vehicles after (agents 1..N). Also corrected `vehicle_poses = poses[num_rsus:]` in the self-beacon filter (was `poses[:num_vehicles]`).
- Tyler's fix was necessary but not sufficient: correct math on wrong-ordered agents still produces wrong output.

### Detection Status (clean CARLA session, score_threshold=0.5)
- Tick 13: Lincoln teleports to z=0.7 (road surface) ✓
- Ticks 29-35: Firetruck at RSU-local (0.4, -7.8), psm=0.54–0.76 ✓
- Ticks 99-100: **Lincoln detected** at RSU-local (7.8, 0.2) and (8.0, 0.8), score=0.871/0.907, matching Lincoln's actual position to within ~2m ✓
- Ticks 146+: Ego vehicle at RSU-local (24–39, 8.8), psm=0.90–0.94 → self-filtered ✓
- Ego does not collide with Lincoln → prediction pipeline (detect → track → predict → ego behavior agent) confirmed working end-to-end ✓

WorldFusion effective detection range is ~7m from RSU for this scenario. This is appropriate: at 40m Lincoln hasn't committed to the intersection; at 7m it has, and the ego still has reaction time. Detection range is not a bug.

### Open Question: Why Does WorldFusion Not Detect Lincoln Until ~7m From RSU?

Lincoln enters the RSU's voxel range (±40m) at tick 43 and is not detected until tick 99 — when it is only ~7m east of the RSU. That is 56 ticks in-range with zero detections. At ticks 99-100, WorldFusion fires with high confidence (score=0.87-0.91) and the detection coordinates match Lincoln's position to within ~2m.

The effective detection range appears to be ~7m, not 40m. Possible causes:
- **LiDAR point density**: Lincoln is approaching the RSU head-on along the road. At 30-40m, the front cross-section of the vehicle may produce too few RSU LiDAR returns to activate the voxel.
- **RSU sensor coverage**: The RSU may be oriented toward the intersection center rather than the approach road, giving good coverage at close range but sparse coverage further along the road.
- **Model sensitivity**: WorldFusion trained on V2XSim may have lower recall for head-on approaching vehicles vs. vehicles already in the intersection.

See "Questions for Tyler" section below.

### Dirty CARLA Session Hygiene
`start_actors.sh` restarts CARLA automatically. For standalone runs (`python ecav.py ...`), CARLA must be manually restarted first. Symptom: actors remain at z=-500 past tick 15 despite `ActorTransformSetter` in the behavior tree.

---

## WIP / Exploratory

### Azure Distributed Deployment (2026-04-06)

Planning phase. See [azure_deploy.md](../../agent_plans/azure_deploy.md).

**Topology**: 5-node split — node-0 (CARLA + ecav.py), node-1 (ecloud_server), node-2 (inference), node-3 (GPU actors), node-4 (CPU actors).

**Key finding**: codebase is already ~90% wired for multi-node. `sim_api.py:580` already skips spawning `ecloud_server` when `ECLOUD_IP != 'localhost'`. Only code change needed: `ecav.py:43` hardcodes `ECLOUD_SERVER_ADDRESS = "localhost:50051"` — must read from `cloud_config.yaml` for actor containers on remote nodes.

**Approach**: Ansible for cluster orchestration; per-node startup scripts extracted from `start_actors.sh`; `cloud_config.yaml` rendered per-node from Jinja2 template.

**Status**: Plan written, not yet implemented.

---

## Questions for Tyler

From live WorldFusion detection run on 2026-04-26:

**WorldFusion effective detection range appears to be ~7m from RSU, not 40m.** Lincoln enters the voxel range (±40m) at tick 43 but is not detected until tick 99 when it is ~7m east of the RSU. First confirmed detections: score=0.871 at rsu-local=(7.8, 0.2), score=0.907 at rsu-local=(8.0, 0.8) — both matching Lincoln's actual position to within ~2m.

Questions:
1. Is the narrow effective detection range (~7m vs. ±40m voxel range) expected given the RSU sensor placement and orientation in scenario_3? Or does this suggest a problem with how RSU LiDAR returns are being voxelized for the approach road?
2. From ticks 107 onward, `dets before_filter=1 after_filter=0` consistently — something is detected but the self-beacon filter removes it. Is this the ego vehicle being detected and correctly filtered, or is it Lincoln being incorrectly removed because the ego is nearby?
3. Is there a known limitation in WorldFusion's recall for head-on approaching vehicles (front cross-section is small for LiDAR)?

---

## Backlog

---

## Related

- [WorldFusion Performance](worldfusion_performance.md) — full optimization history, measured results
- [Architectural Decisions](decisions.md) — D12 (gRPC migration), D13 (standalone servers), D14 (log-based readiness)
- [Architecture](architecture.md) — process topology, ML server ports
- [Plans Index](plans_index.md)
- [Research](research.md)
