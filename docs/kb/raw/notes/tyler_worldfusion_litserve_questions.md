# WorldFusion / LitServe Architecture — Discussion Notes for Tyler

**Date:** 2026-04-05  
**Context:** Investigating distributed performance bottlenecks in `openscenario_3_edge_worldfusion`  
**Status:** Discussed with Tyler 2026-04-05. All questions answered.

---

## Current Architecture (as implemented)

Each actor container (vehicle, RSU) calls LitServe **independently**:

```
Vehicle container:  build_batch → POST /extract_features → spatial_features → gRPC → Edge
RSU container:      build_batch → POST /extract_features → spatial_features → gRPC → Edge
Edge:               spatial_features (all agents) → Where2comm backbone → detection → predictions
```

- LitServe runs the **sensor encoder only** (voxel encoder + LSS camera encoder → BEV `spatial_features`)
- The edge runs the **backbone + Where2comm fusion + detection head** in-process
- `pickled_features` in the gRPC message carries the *output* of LitServe (spatial_features), not the raw batch

## The Distributed Bottleneck

In a distributed run (actors in separate containers), both containers call LitServe independently.
LitServe serializes requests on a single CUDA stream, so the two calls run **de facto sequentially**:

- Vehicle container: `http=165ms` (served first)
- RSU container: `http=400ms` (queued behind vehicle — ~320ms queue wait + ~80ms inference)

This means the actor with the worse queue position always pays ~2× the inference cost per tick.
No amount of tuning eliminates this without changing the call structure.

## Proposed Alternative (edge-centric)

Move the LitServe call to the **edge**:

```
Vehicle container:  build_batch → gRPC (raw voxels + camera tensors) → Edge
RSU container:      build_batch → gRPC (raw voxels + camera tensors) → Edge
Edge:               batch=2 → POST /extract_features → all spatial_features
                    → Where2comm backbone → detection → predictions
```

---

## Questions and Answers (from Tyler, 2026-04-05)

### 1. Custom FastAPI endpoint vs. LitServe native request path

**Q:** Was the custom `@server.app.post(...)` route using `asyncio.to_thread()` a deliberate choice?

**A:** It was just a way to make it a remote process. The blocking behavior was not intentional — Tyler had no way to test the distributed flow when he built it, so parallelism was never addressed.

**Implication:** No architectural constraint here. The custom endpoint can be replaced or made parallel. Phase 2 (strip YOLO, `workers_per_device=N`) is the correct path.

---

### 2. Per-actor LitServe calls vs. edge-centric call

**Q:** Was the per-actor LitServe call pattern intentional, or lifted from sequential mode as-is?

**A:** It is intentional and must stay. Sending raw sensor data to the edge would be **early fusion** — an entirely separate research area that has already been studied. Early fusion is a known bandwidth problem: raw sensor data is orders of magnitude larger than the intermediate tensors we send. WorldFusion is specifically *intermediate* fusion: each actor runs the sensor encoder (LiDAR voxelizer + camera encoder → BEV features) locally, and only the features are transmitted.

**Implication:** Edge-centric LitServe calls are ruled out. The per-actor call architecture is the correct design. Parallelism must be achieved within the current model, not by restructuring where the encoder runs.

---

### 3. RTT implications in distributed mode

**Q:** Was the additional RTT cost of per-actor LitServe calls modeled before the distributed design was set?

**A:** No — the distributed path was not tested at the time.

**Implication:** The CUDA stream serialization bottleneck (~320ms) was a known risk that was not caught before distribution. The parallelism plan (Phase 1: CUDA streams, Phase 2: `workers_per_device=N`) is the fix.

---

### 4. Multi-worker LitServe

**Q:** Was `workers_per_device=N` considered?

**A:** No — same reason as above. Not tested distributed.

**Implication:** `workers_per_device=N` where N = `min(gpu_headroom, num_actors_with_sensors)` is the target. The only structural blocker was YOLO initializing CUDA in the parent process. Stripping YOLO from `litserve_models.py` removes that constraint.

---

### 5. Compression bug (raised by Tyler)

**Q/Finding:** Tyler noted a compression bug beyond the gRPC `pickled_features` fix. The actor→LitServe HTTP path (`_extract_features_remote`) sends raw `msgpack.packb()` with no compression, despite the gRPC path using `zlib`. The asymmetry is unintentional.

**Fix (2026-04-05):** Added `zlib.compress()` on client side (`worldfusion_perception_manager.py`, `edge_manager_worldfusion_ab3dmot_linear_predictor.py`) and `zlib.decompress()` on server side (`litserve_models.py`, gated on `Content-Encoding: zlib` header).

---

## Summary of Trade-offs

| Approach | Pro | Con |
|----------|-----|-----|
| Current (per-actor calls) | Correct for intermediate fusion; actors process own sensors | Serialized at LitServe in distributed mode; 2× RTT per agent |
| Edge-centric batched call | Single LitServe call, no contention | **Ruled out** — would be early fusion, not intermediate |
| LitServe multi-worker | Transparent to actors; no protocol change | Requires YOLO removal to unblock process mode |
| LitServe auto-batching (native path) | Server-side, transparent to actors | Requires restructuring endpoint; defer until after Phase 2 |
