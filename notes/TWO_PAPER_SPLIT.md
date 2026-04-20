# Two-Paper Split: Implementation Map & Stakeholder Summary

## Context

This document maps our simulator implementations to two papers and their
evaluation needs.  It is intended for co-authors and stakeholders who need
to understand what exists, what each paper requires, and where the gaps are.

---

## Paper 1: "Characterizing and Increasing the Safety Envelope for Edge-Assisted Control of Autonomous Vehicles"

**Venue target:** MobiCom or equivalent top-tier systems venue
**Core claim:** Network latency induces a *state-consistency* failure
(self-ghosting) distinct from the physics-based braking limit.  The main
contribution is the **measurement and decomposition** of the safety envelope
into logic-induced (state consistency) vs. physics-induced (stopping distance)
boundaries, plus a minimal correctness invariant (Ego-Uniqueness) that shifts
the boundary toward the physical limit.

**Framing guidance:** This is a **correctness + measurement** paper.  The
self-beacon anchoring protocol is the mechanism, not the primary contribution.
Any planner stabilization (RSS latch, TTC gating) should be presented as
"experimental apparatus to isolate perception/tracking consistency," not as
additional contributions.

### Reviewer Concerns & Our Responses

The TPC summary stated: *"limited novelty … missed potential is a thorough
evaluation in more complex scenarios … unjustified heuristics … writing
unnecessarily complex."*  Five reviewers converged on three themes:

#### 1. Single-ego assumption does not scale (Reviewers A, B, D, E, TPC)

> *"It is unclear whether the proposed tracking method can scale to
> multi-ego vehicle environments, where identity association can be much
> more complex."* — Reviewer A

**Response — Multi-ego cascading evaluation (2, 4, 8, 16 vehicles):**

| Component | File(s) | Status |
|-----------|---------|--------|
| Multi-ego vehicle + RSU support in sim | `sim_api.py`, vehicle/RSU manager, YAML configs | Done |
| Multi-ego sweep in test_runner | `test_runner.py` (`--num-egos`) | In progress |
| BeaconIdManager scales per-vehicle | `beacon_id_manager.py` | Done |
| Ego-Uniqueness Monitor (per-tick duplicate tracking) | `ego_uniqueness_monitor.py` | Done |
| Cross-camera NMS (prevents multi-camera duplicates) | `perception_manager.py`, `nms.py` | Done |
| AB3DMOT NMS center-distance check (cross-sensor dedup) | `nms.py` | Done |

**Planned evaluation:** Latency sweep x ego-count sweep (2, 4, 8, 16 egos)
on LTAP/OD and SCP scenarios.  Show that anchoring success rate holds as
ego count increases while naive-edge degrades.

#### 2. No comparison with VIPS or cooperative perception (Reviewer A, TPC)

> *"The proposed system is only compared with a naive baseline. There is no
> comparison with existing cooperative perception systems, such as VIPS
> [20], V2X-ViT [27]."* — Reviewer A

**Response — Two-part defense: VIPS-style baseline + oracle experiment**

##### 2a. VIPS-style V2I temporal alignment baseline (Required)

VIPS (MobiCom'22) is a **V2I fusion** system — it fuses both vehicle and
infrastructure sensor data, using graph matching to align the
infrastructure's perception with each vehicle's local view under
unpredictable communication/compute delays.  It is NOT infrastructure-only.

EMP (MobiCom'21) is also edge-assisted: vehicles upload sensor data to an
edge server that merges views.  Harbor (SenSys'24) is V2I-centric and
opportunistically uses V2V as a helper/relay.  None of these are V2V-only.

Full reproduction of EMP/Harbor/VIPS end-to-end is expensive and easy for
reviewers to attack on "fidelity" grounds.  For Paper 1, we need the parts
that matter for our claim: time-rectification + association under
delay/jitter.

| Component | File(s) | Status |
|-----------|---------|--------|
| VIPS-style V2I fusion with temporal alignment | `edge_manager_vips_ab3dmot_linear_predictor.py` | **Needs reimplementation** |
| Infrastructure-only baseline (RSU-only, no vehicle data) | Current VIPSEdge code | Done (rename to "Infra-Only") |
| Same latency module + jitter buffer | `edge/latency/` | Done |

**What VIPS reimplementation requires:**
- Fuse vehicle detections + RSU detections (like late fusion)
- Add trajectory-based time-rectification + spatiotemporal graph matching
  to handle async arrivals under delay/jitter
- NO self-beacon anchoring (key difference — VIPS uses spatiotemporal
  matching, not identity-based anchoring)
- Object/track-level implementation is acceptable if described precisely
  as "VIPS-inspired temporal alignment" (not "full VIPS reproduction")

**Critical for MobiCom:** Self-ghosting must persist (or manifest
differently) under VIPS-like temporal alignment.  VIPS is a MobiCom
paper — reviewers WILL check.  If graph matching already prevents
self-ghosting, we pivot to: anchoring provides a deterministic invariant
(ego identity → one track, by construction) vs. probabilistic matching
that can fail under high jitter, multi-ego confusion, or GPS drift.

##### 2b. Oracle front-end experiment (Required — kill-shot for "SOTA fixes it")

Inject **ground-truth 3D boxes** from the CARLA simulator into the edge
pipeline (same tracking, association, prediction, latency/jitter model).
Rerun the full latency sweep.

**What this proves:** If the logic-induced safety cliff persists with
*perfect* perception, then no amount of SOTA perception improvement can
fix it.  The failure is rooted in delayed identity/association, not
detection quality.  This directly neutralizes both "intermediate fusion is
resistant to latency" and "late fusion isn't SOTA."

| Component | File(s) | Status |
|-----------|---------|--------|
| Oracle detection front-end (GT boxes from CARLA) | New: oracle mode in edge manager | **Needs implementation** |
| Same tracking + association + planner pipeline | Existing late fusion back-half | Done |
| Same latency/jitter model | `edge/latency/` | Done |

**Paper text (2-4 sentences):**
> *"To isolate state-consistency faults from perception errors, we repeat
> key experiments with oracle detections from the simulator.  The
> logic-induced cliff persists, confirming that the failure mode is rooted
> in delayed identity/association rather than detection quality."*

##### 2c. Additional baselines (low-effort, high-value)

| System | Architecture | Status |
|--------|-------------|--------|
| Vehicle-only | No cooperation, local perception only | Easy (disable edge) |
| V2V late fusion | Peer-to-peer, same object format, latency model | Exists in `v2v/` (wire into test_runner) |
| Infra-only | RSU sensors only | Done (current VIPSEdge, rename) |

##### Full comparison set for Paper 1

| System | Architecture | Anchoring | Temporal align | Oracle |
|--------|-------------|-----------|---------------|--------|
| Vehicle-only | No cooperation | N/A | N/A | No |
| V2V late fusion | Peer-to-peer, same objects | No | No | No |
| Infra-only | RSU sensors only | No | No | No |
| Naive-Edge | V2I late fusion | No | No | No |
| VIPS-style | V2I late fusion + spatiotemporal matching | No | **Yes** | No |
| Beacon-Edge | V2I late fusion + identity anchoring | **Yes** | No | No |
| Oracle-Edge | V2I ground-truth boxes | No | No | **Yes** |
| Oracle + Anchoring | V2I ground-truth boxes + anchoring | **Yes** | No | **Yes** |

This set isolates:
- (a) Value of edge assistance (vehicle-only vs edge)
- (b) Whether temporal alignment alone suffices (VIPS-style vs Naive-Edge)
- (c) Whether identity anchoring is necessary beyond temporal alignment
- (d) Whether the failure depends on perception quality (Oracle-Edge)

A convincing result: self-ghosting appears even with oracle detections
(ruling out perception quality), VIPS-style alignment reduces but does not
eliminate it under high jitter/multi-ego, and anchoring removes it across
all configurations.

##### Positioning intermediate fusion in Paper 1

**Do NOT implement V2X-ViT or WorldFusion as baselines in Paper 1.**
WorldFusion is a Paper 2 contribution.  V2X-ViT is ego-centric intermediate
fusion — it does not instantiate the same failure surface because:

- Ego-centric fusion projects all agents into the ego vehicle frame and
  fuses for a designated ego (one fusion instance per ego)
- It typically does NOT require an edge-side tracker that returns delayed
  ego state for planning/control
- The self-ghosting mechanism requires an offboard service that (a) runs
  tracking/association over time and (b) returns a state that can be stale
  relative to the ego's current pose

**Handle in Related Work, not evaluation:**

Intermediate fusion "latency resistance" targets perception metrics
(detection mAP under bounded delay), not closed-loop safety.  Even the
intermediate-fusion literature acknowledges vulnerability under adverse
communication — NeurIPS'23 work on asynchrony-robust collaboration notes
that delay-aware encodings do not use historical temporal information for
compensation.  The oracle experiment in our evaluation directly addresses
this: perfect perception does not prevent the consistency failure.

**Avoid** phrases like "compare against V2X-ViT" unless we run the model.
**Use:** *"We evaluate a VIPS-like temporal alignment baseline and an oracle
front-end to demonstrate that the safety cliff is independent of perception
quality."*

**How to answer "late fusion isn't SOTA":**
- Late fusion is a *controlled instrument*, not a claim of best perception
- The paper asks "when does edge assistance become unsafe under delay?" not
  "which detector wins mAP"
- The oracle experiment is the kill-shot: perfect perception doesn't fix it
- Tighten scope: paper evaluates edge-assisted control architectures where
  an offboard service returns delayed state used for planning/control

#### 3. Unjustified heuristics (Reviewer A, TPC)

> *"Why reject any 3D bounding box that is unrealistically thin? … Simply
> eliminating these 'less perfect' objects may increase false negative
> detections."* — Reviewer A

**Response — Removed all heuristic filters:**

| Removed | Replacement |
|---------|-------------|
| `_is_sliver` (volume/dimension filter) | None — let tracker handle noise |
| Ego proximity filter | Self-beacon anchoring (identity, not proximity) |
| `_aabb_iou_2d` overlap filter | AB3DMOT NMS (center-distance + containment) |
| Speed < 3 filter | Collision check with TTC gating |
| Speed < 5 filter | KF velocity gating (`_MIN_KF_SPEED_MPS = 1.0`) |

The paper's revised pipeline relies only on: (1) self-beacon anchoring for
identity, (2) AB3DMOT data association for tracking, (3) TTC-based
collision check for safety decisions.  No ad-hoc geometric filters.

#### 4. GPS dropouts and robustness (Reviewer B, E)

> *"How would the system handle GPS dropouts?"* — Reviewer B
> *"No evaluation of the robustness of the correlation of ID+ground truth
> (e.g. location)."* — Reviewer E

**Response — GPS noise + dropout simulation:**

| Component | File(s) | Status |
|-----------|---------|--------|
| Bursty GPS dropout (Markov-chain model) | `localization_manager.py` (GnssSensor) | Done |
| GPS noise sweep in test_runner | `test_runner.py` (`--gps-noise`) | Done |
| GPS dropout sweep in test_runner | `test_runner.py` (`--gps-dropout`) | Done |

**Planned evaluation:** Anchoring success under GPS noise (0.5m, 1m, 2m,
5m std) and dropout (5%, 10%, 20% burst probability).  Show that anchoring
degrades gracefully — forced match succeeds as long as beacon drift <
association gate.

#### 5. BSM temporary ID and privacy (Reviewer D)

> *"The BSM standardized by SAE J2945 includes a temporary ID that is
> updated … A simple mitigation is to detect the temporary ID change."*
> — Reviewer D

**Response — BeaconIdManager with bounded anchoring epoch:**

| Component | File(s) | Status |
|-----------|---------|--------|
| BeaconIdManager (temp_id rotation, reverse mapping) | `beacon_id_manager.py` | Done |
| Bounded anchoring epoch (age counter in KF, forced expiry) | `kalman_filter.py`, `matching.py`, `model.py` | Done |
| Temp ID → CARLA ID reverse mapping for ego filter | Late fusion + WorldFusion edge managers | Done |

The paper's revised Section 4 discusses SAE J2735/J2945 BSM compatibility.
Per the NHTSA Federal Register (2017-01-12) and 5GAA STiCAD report,
pseudonym/temporary ID rotation is tied to **certificate changes** (typical
interval: every 5 minutes), not distance traveled.  Our bounded anchoring
epoch argument: ID rotation *can* occur within the reconciliation window
under some configurations, so the protocol needs an epoch/linkage mechanism
(or kinematic continuity rule) to preserve Ego-Uniqueness across a
pseudonym boundary.  The edge maintains a reverse-mapping table; bounded
anchoring epoch ensures stale identity associations expire.

#### 6. Computational cost of protocol (Reviewer B)

> *"The computational and communication energy cost of the protocol could
> be worth quantifying."* — Reviewer B

**Response — Edge profiler + per-stage timing:**

| Component | File(s) | Status |
|-----------|---------|--------|
| EdgeProfiler (per-stage wall-clock timing, GPU util) | `edge_profiler.py` | Done |
| Per-frame metrics (feature_collection, tracking, prediction, distribution) | All edge managers via `frame.time()` | Done |
| Bandwidth measurement (Table 3 in paper: 9.9 KB/s per vehicle) | Already in paper | Done |

**Planned evaluation:** Show anchoring adds < 0.2ms per frame (already
measured).  Profile tracker latency vs. N objects for multi-ego scenarios.

#### 7. Realistic latency modeling (Reviewer B, trace-driven validation)

| Component | File(s) | Status |
|-----------|---------|--------|
| Pluggable latency module (fixed, normal, lognormal, hybrid) | `edge/latency/latency_model.py` | Done |
| Jitter buffer (per-packet arrival, source-tick ordering) | `edge/latency/jitter_buffer.py` | Done |
| Persistent tracker (O(D) per frame, not O(H×D) replay) | Late fusion + VIPS edge managers | Done |
| Jitter sweep in test_runner | `test_runner.py` (`--jitter-std`, `--latency-distribution`) | Done |

#### 8. Experimental apparatus: planner stabilization

These are NOT contributions — they stabilize the planner so the experiment
isolates perception/tracking consistency.  Present as methodology.

| Component | File(s) | Purpose |
|-----------|---------|---------|
| RSS proper response latch in behavior_agent | `behavior_agent.py` | Prevent braking oscillation from masking real collision avoidance |
| TTC-based collision check (time-reparametrized paths) | `collision_check.py` | Deterministic collision detection for reproducible envelope measurement |
| KF velocity for prediction (not regression) | `linear_predictor_manager.py` | Remove depth-convergence artifacts from prediction |

### Paper 1 — What's Left To Do

**Implementation:**
1. **Reimplement VIPS-style edge manager** — V2I fusion with temporal
   alignment (trajectory-based time-rectification + spatiotemporal matching
   between vehicle + RSU detections under async arrivals), NO self-beacon
   anchoring.  Describe as "VIPS-inspired," not "full VIPS reproduction."
2. **Implement oracle front-end** — GT boxes from CARLA injected into the
   same tracking/association/prediction pipeline with same latency model.
   This is the kill-shot against "SOTA perception fixes it."
3. **Rename current VIPSEdge** → "InfraOnlyEdge" (RSU-only baseline)
4. **Wire V2V late fusion baseline** into test_runner (BM2CP V2V exists in
   `v2v/` directory but isn't in sweep infrastructure)
5. **Finish multi-ego sweep in test_runner** (`--num-egos`)

**Evaluation runs:**
6. **Oracle experiment:** Latency sweep with GT boxes, with and without
   anchoring.  If cliff persists → perception quality is irrelevant.
7. **Multi-ego cascading:** 2, 4, 8, 16 ego sweeps × latency sweep × both
   scenarios (LTAP/OD, SCP) × 10 repetitions.  Must show **qualitative
   transition** (scaling law, duplicate-track rate growth vs ego count,
   anchoring flattening the curve) — not just "2x safer region"
8. **VIPS-style comparison:** Same sweep — does spatiotemporal alignment
   prevent self-ghosting?  If it reduces but doesn't eliminate → anchoring
   provides the stronger guarantee (deterministic invariant).
9. **Full comparison set:** Vehicle-only, V2V, Infra-only, Naive-Edge,
   VIPS-style, Beacon-Edge, Oracle-Edge, Oracle+Anchoring
10. **GPS noise/dropout runs:** Anchoring under degraded localization
11. **BSM rotation runs:** Verify bounded anchoring epoch under rapid ID
    rotation

**Paper revision:**
12. **Rewrite Sections 3.5.1** — remove all heuristic justifications
13. **Rewrite Section 4** — BSM/SAE J2735 compatibility, certificate-based
    rotation (cite NHTSA Federal Register, 5GAA), bounded anchoring epoch
14. **Rewrite Section 5** — oracle results, multi-ego results, VIPS
    comparison, full comparison set, GPS robustness
15. **Reframe contributions:**
    - "Characterizes a planner-visible safety cliff induced by
      state-consistency faults under delay"
    - "Separates physics-only braking limit from distributed consistency
      limit"
    - "Introduces an invariant (Ego-Uniqueness) and a minimal enforcement
      mechanism that is orthogonal to fusion model choice"
    - Avoid "first" claims — use narrow scope claims
16. **Related work taxonomy:** (a) CVP architecture work (EMP/Harbor/
    EdgeCooper), (b) temporal alignment work (VIPS), (c) learning fusion
    work (V2X-ViT), then place Ego-Uniqueness + safety-envelope
    characterization in a non-overlapping contribution slot
17. **Simplify writing** — TPC: "unnecessarily complex"
18. **New figures:** Oracle cliff, multi-ego scaling law, VIPS comparison,
    full comparison set, GPS degradation

---

## Paper 2: "Conductor: A Resource-Aware Edge Service for Scalable Cooperative Perception"

**Venue:** TBD (systems conference)
**Core claim:** As vehicle density scales from 10 to 200, the edge hits a
*computational performance cliff* — processing delays exceed the safety
envelope not because of the network but because of compute overload.
Conductor dynamically adapts to prevent this.

### What Conductor Needs (from the draft)

The Conductor draft (Section 4) specifies a SOTA pipeline + two adaptive
mechanisms:

#### SOTA Pipeline

| Component | Draft Reference | Simulator Status |
|-----------|----------------|-----------------|
| Cooperative perception (BM2CP / OpenCOOD) | Section 4.1 | **Done** — BM2CP submodule, BM2CPEdge manager |
| WorldFusion (Where2comm attention, world-frame) | Section 4.1 | **Done** — WorldFusionEdge manager |
| MambaTrack (learning-based tracker, SSMs) | Section 4.1, ref [4] | **Not implemented** |
| SMART predictor (multi-modal, next-token) | Section 4.1, ref [3] | **Not implemented** |

#### Adaptive Mechanisms

| Mechanism | Draft Reference | Simulator Status |
|-----------|----------------|-----------------|
| Adaptive edge fusion (input-prioritizing selector) | Section 4.2.1 | **Not implemented** |
| Fidelity adaptation (reduce prediction horizon/modes) | Section 4.2.2 | **Not implemented** |
| Rate adaptation (reduce update frequency for low-priority vehicles) | Section 4.2.2 | **Not implemented** |
| Model adaptation (MambaTrack → AB3DMOT fallback) | Section 4.2.2 | **Not implemented** |

#### Infrastructure (carries forward from Paper 1 work)

| Component | Status | Notes |
|-----------|--------|-------|
| Modular edge architecture (base + 6 backends) | Done | Factory registry, shared base class |
| Compute contention model | Done | `compute_budget_ms`, `per_vehicle_compute_ms` in base |
| Edge profiler (per-stage timing, GPU util) | Done | `edge_profiler.py` |
| Pluggable latency module + jitter buffer | Done | `edge/latency/` |
| LitServe ML offloading (YOLO + WorldFusion) | Done | `litserve_models.py` |
| Distributed simulation mode (gRPC orchestrator) | Done | Vehicle/RSU containers |
| Multi-ego + RSU support | Done | Arbitrary vehicle/RSU count per edge |
| Comprehensive metrics (detection, tracking, prediction, planning, safety) | Done | Full metrics architecture |

### Conductor Evaluation Plan (from draft Section 6)

- **Testbed:** Azure NVads A10 v5
- **Scenarios:** Vehicle density sweep (10–200), scenario complexity sweep,
  GPU Buster + Memory Hog stress tests
- **Compared systems:** Conductor (all policies), Static SOTA Pipeline
  (policies disabled), Harbor baseline
- **Metrics:** System performance (latency, CPU/GPU, energy), Application
  QoS (mAP, HOTA, minADE/minFDE), Safety (collision rate, TTC)

### Paper 2 — What's Left To Do

1. **Implement MambaTrack** integration (new edge manager backend or
   swappable tracker in existing backends)
2. **Implement SMART predictor** integration (replace linear predictor for
   SOTA pipeline)
3. **Implement adaptive fusion module** (input-prioritizing selector before
   BM2CP/WorldFusion)
4. **Implement fidelity controller** (state machine: normal → fidelity
   adaptation → rate adaptation → model adaptation)
5. **Dense scenario configs** (10, 25, 50, 100, 150, 200 vehicles)
6. **Harbor baseline** re-implementation for comparison
7. **Azure testbed** profiling runs

---

## Implementation Summary: What Belongs Where

### Paper 1 Only (Safety Envelope revision)

- Self-beacon anchoring + BeaconIdManager (+ BSM temp IDs, bounded epoch)
- Multi-ego cascading evaluation (2, 4, 8, 16)
- VIPS-style V2I temporal alignment baseline (**needs reimplementation**)
- Infrastructure-only baseline (current VIPSEdge, renamed)
- V2V late fusion baseline (same object format, peer-to-peer)
- GPS noise/dropout simulation
- Removed heuristic filters (sliver, proximity, IoU, speed)
- Planner stabilization (RSS latch, TTC gating — methodology, not contribution)
- Ego-Uniqueness Monitor

### Shared Infrastructure (both papers use)

- Modular edge manager architecture (base class + factory)
- AB3DMOT tracker (KF tuning, KITTI↔CARLA coord swap)
- Linear predictor (KF velocity extrapolation)
- Pluggable latency module (4 models + jitter buffer)
- Edge profiler + metrics architecture
- Cross-camera NMS
- Multi-ego + RSU support
- Distributed simulation mode
- LitServe ML offloading
- test_runner.py sweep infrastructure

### Paper 2 Only (Conductor)

- WorldFusion cooperative perception (as a pipeline component)
- BM2CP cooperative perception (as a pipeline component)
- Compute contention model (budget enforcement)
- MambaTrack integration (NOT YET IMPLEMENTED)
- SMART predictor integration (NOT YET IMPLEMENTED)
- Adaptive fusion module (NOT YET IMPLEMENTED)
- Fidelity controller (NOT YET IMPLEMENTED)
- Dense scenario evaluation (10–200 vehicles)
- Harbor baseline comparison

---

## Immediate Priority: Paper 1 Revision

The Safety Envelope paper has concrete reviewer feedback and a path to
MobiCom.  Priorities in order:

1. **Oracle front-end** — lowest implementation cost, highest defensive
   value.  If the cliff persists with perfect perception, the entire
   "SOTA would fix it" argument collapses.  Implement first.
2. **VIPS reimplementation** — highest-risk item.  If VIPS-style temporal
   alignment already prevents self-ghosting, we need to know NOW because it
   changes the paper's thesis.  VIPS is a MobiCom paper — reviewers will
   check our comparison.
3. **Multi-ego runs** — addresses the #1 reviewer concern (scalability)
   shared by all five reviewers.  Must show **qualitative transition**
   (scaling law, duplicate-track rate growth), not just parameter sweep.
4. **Full comparison set** — Vehicle-only, V2V, Infra-only, Naive-Edge,
   VIPS-style, Beacon-Edge, Oracle, Oracle+Anchoring.  Transforms the paper
   from "bug fix" to "measurement study with correctness invariant."
5. **GPS robustness** — addresses Reviewer B and E concerns about real-world
   conditions.
6. **Paper rewrite** — reframe contributions (logic vs physics decomposition
   is the main result; Ego-Uniqueness invariant is orthogonal to fusion
   choice), add BSM/privacy discussion (cite NHTSA, 5GAA), restructure
   related work into clean taxonomy, simplify writing.

**Key risks:**
- If VIPS temporal alignment prevents self-ghosting → pivot to: anchoring
  provides deterministic invariant vs. probabilistic matching; test whether
  VIPS matching breaks under high jitter, multi-ego, or GPS drift.
- If oracle experiment shows cliff disappears with GT boxes → self-ghosting
  IS a perception artifact, and the paper's thesis changes fundamentally.
  (This is unlikely given the mechanism, but must be verified.)

**Novelty framing (avoid reviewer traps):**
- Avoid "first" claims
- Use: "characterizes a planner-visible safety cliff induced by
  state-consistency faults under delay"
- Use: "separates physics-only braking limit from distributed consistency
  limit"
- Use: "introduces an invariant and minimal enforcement mechanism
  orthogonal to fusion model choice"
- Late fusion is a controlled instrument, not a claim of best perception
- Cite V2X-ViT as SOTA for detection robustness, but emphasize detection
  robustness ≠ edge tracker identity continuity under delayed beacons

---

## Timeline

### Paper 1 — MobiCom 2026
- **Abstract registration: March 6, 2026 (AoE)**
- **Paper submission: March 13, 2026 (AoE)**
- ~5 weeks from today (Feb 5, 2026)
- Must have: oracle experiment, VIPS-style baseline, multi-ego scaling
  curves, GPS robustness — all as first-class results, not "future work"

### Paper 2 — Conductor
- **SEC 2026**: Abstract April 24, Paper May 1, 2026
- **SenSys 2026**: Merged flagship with IPSN/IoTDI (audience shift)
- SEC aligns best with "edge compute cliff + resource-aware adaptation"
- Requires significant new implementation (MambaTrack, SMART, adaptive
  mechanisms)

### Assessment
The outlined fixes make Paper 1 *plausibly competitive* for MobiCom, but
accept probability is bounded by:
- MobiCom's ~13-21% base acceptance rate
- Whether the final draft contains a decisive novelty result beyond "use ID"
- Specifically: oracle + alignment-baseline evidence must be conclusive
