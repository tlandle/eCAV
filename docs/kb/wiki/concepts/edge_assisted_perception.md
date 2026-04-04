---
updated: 2026-04-04
---
# Edge-Assisted Perception

## The Core Scenario

A vehicle approaches a blind intersection. A cross-street vehicle is occluded by a building; the ego vehicle's onboard sensors cannot detect it. A camera mounted at the intersection — connected to an edge server — can see both vehicles.

The edge server:
1. Runs perception on its own camera feed (detecting the cross-street vehicle)
2. Receives perception data from the ego vehicle (via C-V2X or 5G)
3. Fuses both into a unified obstacle map
4. Publishes fused state back to the ego vehicle

The ego vehicle can then detect the cross-street vehicle and brake or yield before entering the intersection. Without the edge, this requires the vehicle to creep into the intersection to get line-of-sight — by which point avoidance may not be possible.

## Why This Is Non-Trivial

**Latency:** The edge pipeline introduces network round-trip delay. If the edge publishes state with 200ms latency, the ego vehicle is acting on where obstacles *were*, not where they *are*. At 30 mph, a vehicle travels ~2.7 meters in 200ms.

**State inconsistency:** The ego vehicle has its own local perception running simultaneously with the edge perception. These two streams see different things at different times. Naively merging them can introduce conflicts (e.g., the ego perceives itself and the edge also perceives the ego, causing a ghost detection).

**Self-ghosting:** The edge detects the ego vehicle and publishes it as an obstacle. The ego vehicle then treats itself as an external obstacle — this can cause emergent braking or avoidance behavior. Self-Beacon Anchoring (SBA) addresses this by enforcing an ego-uniqueness invariant at the publish boundary.

## Self-Beacon Anchoring (SBA)

Each vehicle broadcasts a unique identifier in its V2X beacon. The edge uses this identifier to exclude the publishing vehicle from its own perceived obstacle list before transmitting back. SBA is an invariant at the edge publish boundary: a vehicle's own ID never appears in the obstacle list it receives from its edge.

Without SBA, self-ghosting failure rates increase with vehicle count and with edge detection quality (paradoxically, better edge detection → more reliable self-ghosting).

## Fusion Architectures

Two distinct fusion approaches are implemented and evaluated:

### Late Fusion
Each perception source independently completes detection (bounding boxes). Boxes are merged by the edge at the output stage. Simple, but loses spatial correlation between sources.

In eCAV: vehicle runs YOLO locally (or via LitServe), edge runs its own detection, results are merged. Tracking via AB3DMOT or VIPS.

### Intermediate Fusion
Feature representations (not final detections) from multiple cameras are merged before the detection head runs. Requires sharing raw feature tensors between agents — higher bandwidth, but retains spatial context for the fusion step.

In eCAV: WorldFusion and BM2CP implement intermediate fusion. The edge collects feature tensors from multiple vehicles and its own camera, fuses them via learned attention or concatenation, then runs the detection head once on the fused representation.

## Related

- [Collaborative Perception concept](collaborative_perception.md)
- [Latency Modeling concept](latency_modeling.md)
- [Distributed Simulation concept](distributed_simulation.md)
- [WorldFusion dependency](../dependencies/worldfusion.md)
- [Research](../research.md)
