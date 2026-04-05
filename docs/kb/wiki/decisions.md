---
updated: 2026-04-04
---
# Architectural Decisions

Record of key decisions with rationale. Forward-looking plans live in [docs/agent_plans/](../../agent_plans/).

---

## D1: C++ Comms Server Instead of Python asyncio

**Decision:** Central orchestration comms server written in C++ (`ecav/ecloud_server/`), not Python asyncio.

**Rationale:** Python's GIL prevents true parallel execution of per-actor tick responses. With 4–16 actors calling in simultaneously, the asyncio approach serialized responses and became the dominant bottleneck. C++ with native threads handles concurrent actor registrations and tick acks without GIL contention.

**Trade-off:** C++ build dependency; requires CMake setup; adds complexity for contributors unfamiliar with the mixed-language stack.

---

## D2: gRPC for All Simulation IPC

**Decision:** All inter-process communication (orchestrator ↔ comms server ↔ actors) uses gRPC with protobuf (`ecloud.proto`).

**Rationale:** Typed, versioned message contracts across Python and C++ processes. Supports both streaming and unary RPCs. Well-supported in both languages with code generation.

**See also:** [grpc dependency](dependencies/grpc.md)

---

## D3: gRPC for YOLO Perception (Migrated from HTTP/msgpack)

**Decision:** YOLO LitServe transport migrated from HTTP + msgpack to gRPC (`perception.proto`).

**Rationale:** HTTP required a new TCP connection per request without `requests.Session`, adding ~15ms overhead. msgpack provided binary serialization but required an extra deserialization step. gRPC provides persistent connection, binary framing, and typed messages in one.

**Plan:** [grpc_perception_migration.md](../../agent_plans/grpc_perception_migration.md)

---

## D4: Native 640×480 Camera Resolution for YOLO

**Decision:** CARLA camera configured at 640×480 natively, matching YOLOv5 inference resolution.

**Rationale:** The camera was originally configured at a higher resolution and downsampled before encoding for LitServe. Setting native resolution to match inference input eliminated the resize step, which was the dominant overhead: e2e latency dropped from ~70ms → ~22ms (−69%). This was the single largest optimization.

**Lesson:** Profile before optimizing transport. The bottleneck was at the sensor, not the network.

---

## D5: uint8 for WorldFusion `imgs` Transport (O4)

**Decision:** `imgs` tensor sent as uint8 to LitServe instead of float32.

**Rationale:** CARLA camera output is uint8 (8-bit per channel). Converting to float32 before transport tripled the payload with no information gain — normalization happens inside the model. Sending uint8 reduces request payload by 53% (8021 → 3803 KB) losslessly. On loopback, savings are neutral (latency-dominated); on Azure, saves ~4 MB/s bandwidth per intersection.

**Cost:** 0.31ms encode (negligible).

**Plan:** [worldfusion_litserve_plan.md](../../agent_plans/worldfusion_litserve_plan.md)

---

## D6: float16 for WorldFusion `spatial_features` Response (O1)

**Decision:** WorldFusion LitServe server returns `spatial_features` as float16, not float32.

**Rationale:** `spatial_features` is an intermediate feature tensor used as input to downstream fusion. float16 halves the response payload (~10 MB → ~5 MB) and reduced http_ms by 40% (291 → 175ms). Quality impact on final detection not observed to be significant.

**Risk acknowledged:** float16 precision loss could affect detection quality in edge cases. No regression observed in tested scenarios.

---

## D7: TurboJPEG Codec Reverted (O4b)

**Decision:** libjpeg-turbo JPEG codec reverted; default PIL JPEG retained.

**Rationale:** Initial hypothesis was that TurboJPEG would reduce encode/decode time on full-resolution images. After applying O4b, measured gains were marginal and unstable. Root cause: the performance gains originally attributed to TurboJPEG were actually from the camera resolution change (D4). Once the camera was at 640×480, encode/decode time was negligible regardless of codec.

**Lesson:** Attribute performance gains to specific changes before building on them.

---

## D8: `distributed` Flag Scope Is CLI Only (not YAML)

**Decision:** The `-d` / `--distributed` flag that enables distributed actor mode is a CLI argument only. The `distributed` field must be removed from all YAML files.

**Rationale:** YAML files describe scenario content (world properties, actor configurations). Execution mode (distributed vs. sequential, LitServe vs. local perception) is an operator choice at runtime, not a scenario property. Mixing them creates ambiguity about which controls behavior.

**Corollary:** A new `--litserve` / `-l` flag controls distributed ML inference, also CLI-only.

**Plan:** [edge_architecture_proposal.md](../../agent_plans/edge_architecture_proposal.md) Phase 0

---

## D9: WorldFusion Depth Map Placeholder Not Eliminated (O2)

**Decision:** Attempted to eliminate the zero-valued `depth_map` tensor from WorldFusion transport payload; reverted.

**Rationale:** The WorldFusion model accepts a depth map as an optional input. The eCAV implementation passes a zero tensor placeholder (~2.8 MB float32). Hypothesis was that removing it would reduce payload. Result: no measurable benefit; the model still expected the tensor; payload savings were smaller than anticipated.

**Status:** Reverted. Zero depth map placeholder retained.

---

## D11: Edge Index and Port Are Dynamically Assigned, Not in YAML

**Decision:** `edge_index` and `edge_port` are not declared in scenario YAML files. Instead, `start_actors.sh` assigns edge indices sequentially (0, 1, 2...) and ports sequentially from base port 50054.

**Rationale:** Edge identity is an execution-time property, not a scenario property. Hardcoding ports in YAML would create conflicts across concurrent simulation runs. Dynamic assignment keeps YAML files describing what actors exist (vehicles, RSUs, edge type/manager), while the shell script controls how they are deployed.

**Trade-off:** Cannot specify custom non-sequential edge indices or ports via YAML. Single-edge scenarios (the current research focus) are unaffected.

---

## D10: Edge Process Owns Tick Barrier for Its Actors

**Decision:** In the planned edge architecture, each edge process manages its own tick barrier (waits for all its actors to ack before sending up to the comms server).

**Rationale:** The C++ comms server currently waits for N actors. With edge processes, having the server track individual actors through the edge would require the server to have knowledge of the edge-actor topology. Instead, each edge presents a single interface to the comms server: "my actors are done." This keeps the comms server topology-agnostic.

**Plan:** [edge_architecture_proposal.md](../../agent_plans/edge_architecture_proposal.md) §3.2
