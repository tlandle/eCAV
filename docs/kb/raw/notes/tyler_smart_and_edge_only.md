# Notes: Tyler — SMART Predictor Status & Edge-Only Distributed Mode

**Date:** 2026-05-09  
**Source:** Slack conversation (Jordan + Tyler)

---

## Summary

Four topics covered:

1. **Distributed edge is now real** — the edge manager (WorldFusion/LateFusion fusion logic) was previously a NOP relay in `edge_process.py`. It is now wired up and running correctly in both modes.

2. **SMART predictor is broken for this use case** — trained on Waymo Open Dataset, doesn't generalize to CARLA. Requires HD map input that was never wired in; predictions are off by ~17m on average. MTR (from CMP) is the path forward. Tyler is retraining it on WorldFusion inputs.

3. **Edge never offloads to litserve — this is architectural** — the edge is hardware-in-the-loop and always runs its own local GPU inference (fusion heads, tracking, prediction). Vehicles can offload perception to litserve; the edge cannot and should not. This is a firm design invariant, not a configuration option.

4. **Edge-only distributed mode is motivated by systems profiling** — Tyler cannot fit CARLA and edge GPU work on the same node. The edge must be its own isolated node to get accurate timing and GPU utilization measurements. Litserve is not a factor on the edge side.

---

## Topic 1: Distributed Edge — What Was Missing and What's Fixed

The first pass at distributed simulation distributed the actors (ego vehicle, RSU, NPC container) but not the edges. Edge processes were then added, comms worked, but the edge manager logic itself (the actual fusion pipeline) was left as a stub/NOP — masked by the fact that the WorldFusion scenario appeared to "run" (no crash, no obvious failure).

After going back and auditing what was and wasn't done, the edge manager logic is now correctly wired into `edge_process.py`. Both WorldFusion and LateFusion run real fusion in the edge container.

**Current test results:**

- WorldFusion (with and without litserve): collision. SMART predictor sees the Lincoln too late to brake. This is a predictor problem, not a pipeline problem.
- LateFusion (YOLO locally): no collision — falls back to linear predictor, which is fast enough.
- LateFusion (YOLO via litserve): collision — litserve path uses SMART instead of linear predictor, hitting the same issue as WorldFusion.

---

## Topic 2: SMART Predictor — Why It Doesn't Work

SMART was chosen for its scene-level prediction (one forward pass produces predictions for all vehicles in a locale — scales well). MTR was avoided because it predicts one vehicle per forward pass (expensive at scale).

In practice, SMART cannot be used without retraining from scratch:

- **Generalization failure** — trained on Waymo Open Dataset, does not transfer to CARLA's coordinate system or our scenarios.
- **Missing HD map input** — SMART requires lane markings and map structure on every inference call. This was never correctly wired in.
- **Result quality** — even partially wired, predictions are ~17m off on average and often point in the wrong direction.

**Current path: MTR from CMP.** The pretrained MTR model was already path-cleared for offline use on OPV2V data. Tyler tested it for scene-level prediction (it was designed for ego-centric inputs, not scene-level), found it workable, and is currently retraining it on WorldFusion inputs. Should work for our purposes once retrained.

---

## Topic 3: Edge Inference Architecture — Edge Never Uses litserve

Both late fusion and WorldFusion follow the same high-level flow; only the components differ:

- **Vehicles** run perception locally or offload to litserve (YOLO for late fusion; spatial feature extraction for WorldFusion). litserve exists to scale across large numbers of vehicles.
- **Edge** always runs locally — GPU fusion heads, classification, tracking (AB3DMOT / Mamba3DMOT), and prediction (MTR / SMART). No litserve, ever.

The edge is **hardware-in-the-loop** by design. Its purpose is to be profiled as a real system under realistic load. Offloading inference would defeat that purpose.

**What the edge sends back to vehicles:** trajectory predictions (not fused features, not raw detections). The pipeline is cooperative motion prediction, edge-focused. The output is the same in both fusion modes — only the input differs.

---

## Topic 4: Edge-Only Distributed Mode — Motivation

Tyler's immediate need: profile the edge as a standalone node.

- Cannot fit CARLA + edge GPU work on a single machine.
- litserve is not relevant for the edge (see above).
- For multi-edge work, the starting scenario is: one vehicle + 2 RSUs + 2 edges, vehicle drives between edge-owned locales.
- Each edge must spawn everything locally (fusion, tracking, prediction) so timing and GPU utilization measurements are accurate.

This is the direct motivation for the edge-only distributed mode in `docs/agent_plans/edge_only_distributed_mode.md`.
