# Edge World-Model Pipeline: Deadline-Aware Adaptation Architecture

## Overview

Two complementary mechanisms enable the edge cooperative world-model pipeline (WorldFusion + AB3DMOT + MTR) to meet planner deadlines as agent count N grows beyond the compute cliff (~N=10 on RTX 4080 SUPER).

Both mechanisms exploit the edge's unique properties:
- Persistent cross-tick track state (AB3DMOT maintains track history)
- Global scene graph (all agents' positions, velocities, headings)
- Centralized compute (single GPU, sequential pipeline stages)

---

## Mechanism 1: Temporal Amortization with Divergence Gating

### Problem

MTR prediction costs 47ms at 22 tracks and 77ms at 44 tracks. This runs every tick (50ms budget at 20Hz). Most objects follow predictable trajectories between ticks. Re-predicting every object every tick wastes compute on objects whose futures haven't changed.

### Insight

The edge maintains persistent track state via AB3DMOT. Between ticks, most tracked objects move along their previously predicted trajectories. Only objects whose observed state diverges from the cached prediction need fresh MTR inference.

### Architecture

```
Per-tick pipeline:
  1. Fusion (WorldFusion)        -> detections
  2. Tracking (AB3DMOT)          -> updated track states
  3. Divergence check            -> classify tracks as STALE or FRESH
  4. MTR prediction (STALE only) -> new predictions for divergent tracks
  5. Merge cached + new          -> complete prediction set
  6. Publish to vehicles
```

### Divergence Check

For each tracked object i with cached prediction P_i(t) from the previous tick:

```
observed_pos  = track_state[i].position          # from AB3DMOT
predicted_pos = P_i(t).interpolate(current_time)  # cached prediction at current time
heading_obs   = track_state[i].heading
heading_pred  = P_i(t).heading(current_time)

position_error = ||observed_pos - predicted_pos||_2
heading_error  = |wrap(heading_obs - heading_pred)|

divergent = (position_error > THRESH_POS) or (heading_error > THRESH_HEAD)
```

**Thresholds (tunable):**
- THRESH_POS = 1.0 m (half a lane width)
- THRESH_HEAD = 10 degrees (beginning of a turn)

**Cost:** O(N) for N tracks. One numpy vector subtraction + norm. Negligible (~0.1ms).

### Expected Behavior

At an intersection with 44 tracked objects (N=16 egos):
- ~30 objects are driving straight or parked: position_error < 0.3m per tick. FRESH (use cache).
- ~10 objects are approaching but predictable: position_error < 0.8m. FRESH.
- ~4 objects are turning, braking, or newly appeared: position_error > 1.0m. STALE (re-predict).

MTR on 4 objects: ~30ms instead of 77ms on 44 objects. Savings: 47ms.

### Staleness Bound

Cached predictions have bounded staleness. If an object is FRESH for K consecutive ticks, its cached prediction is K * 50ms old. The divergence threshold ensures the cached prediction is never more than THRESH_POS meters wrong at the current time. If the planner's safety envelope requires < 2m prediction accuracy, THRESH_POS = 1.0m guarantees the cache is always within the planner's tolerance.

A maximum cache age (e.g., MAX_CACHE_TICKS = 10 = 500ms) forces re-prediction regardless of divergence, preventing unbounded staleness.

### Data Flow

```
Tick t-1:
  MTR predicts all 44 objects -> cache[i] = prediction_i for all i
  Total prediction time: 77ms

Tick t:
  AB3DMOT updates 44 tracks
  Divergence check: 4 objects divergent, 40 fresh
  MTR predicts 4 objects -> cache[divergent_i] = new_prediction_i
  40 objects use cache[i] from tick t-1
  Total prediction time: 30ms (saved 47ms)

Tick t+1:
  AB3DMOT updates 44 tracks
  Divergence check: 6 objects divergent (includes 2 newly turning)
  MTR predicts 6 objects
  Total prediction time: 32ms
```

### Integration Points

- **AB3DMOT wrapper:** After `track()`, expose per-track state (position, velocity, heading) for divergence check.
- **MTR predictor:** Accept a subset of track IDs to predict (not all). Build batch_dict with only divergent tracks.
- **Prediction cache:** Dictionary mapping track_id -> (prediction_tensor, tick_generated, cache_age). Stored in the edge manager.
- **Edge manager tick loop:** Insert divergence check between tracking and prediction stages.

---

## Mechanism 2: Risk-Budgeted Prediction Scheduling

### Problem

When N is large enough that even temporal amortization can't keep prediction under the deadline (e.g., 20 divergent objects at N=32), the system must decide which objects to predict. Predicting all of them equally wastes budget on objects that don't affect any vehicle's planner.

### Insight

The edge has the global scene graph. It knows every vehicle's trajectory and every tracked object's trajectory. It can compute which objects are on a collision course with which vehicles. Objects that are not on any vehicle's conflict path don't need accurate predictions for the planner to make safe decisions.

### Architecture

```
Per-tick pipeline (after divergence gating):
  1. Divergence check -> STALE objects (need re-prediction)
  2. Risk scoring     -> assign risk r_i to each STALE object
  3. Budget check     -> available_ms = deadline - (fusion_time + tracking_time + overhead)
  4. Knapsack select  -> choose subset S of STALE objects to predict within budget
  5. MTR prediction   -> predict objects in S
  6. Linear fallback  -> extend cached predictions for STALE objects not in S
  7. Merge all        -> complete prediction set
```

### Risk Score Computation

For each tracked object i, compute risk relative to all ego vehicles:

```
risk_i = max over all ego vehicles v:
    r_iv = f(ttc_iv, speed_iv, occlusion_iv, lane_conflict_iv)

where:
    ttc_iv     = time-to-collision between object i and vehicle v
                 (geometric, based on current positions + velocities)
    speed_iv   = closing speed between object i and vehicle v
    occlusion_iv = 1 if object i is occluded from vehicle v's perspective
    lane_conflict_iv = 1 if object i and vehicle v are on conflicting lanes
```

**Risk function (simple, fast):**

```
r_iv = (1 / max(ttc_iv, 0.5)) * speed_iv * (1 + occlusion_iv) * lane_conflict_iv
```

- Low TTC = high risk (inverse relationship, clamped at 0.5s)
- High closing speed = high risk (linear)
- Occluded objects get 2x risk boost (the vehicle can't see them, so the edge prediction is their only information)
- Non-conflicting lanes get zero risk (lane_conflict = 0)

**Cost:** O(N_stale * N_ego). For 20 stale objects and 16 egos: 320 evaluations. Each is arithmetic on scalars. Total: ~0.5ms.

### Budget Allocation (Greedy Knapsack)

```
available_budget_ms = deadline_ms - fusion_ms - tracking_ms - divergence_ms - overhead_ms
cost_per_object_ms  = estimated from MTR scaling curve (e.g., 30ms base + 0.5ms/object)

Sort STALE objects by risk_i / cost_i (descending)
Greedily add objects to prediction set S until budget exhausted
Remaining STALE objects get linear extrapolation from cached prediction
```

This is O(N_stale * log(N_stale)) for the sort. Negligible.

### Linear Fallback

Objects not selected for MTR prediction get their cached prediction extended by one tick using constant-velocity extrapolation:

```
fallback_position(t) = cached_position(t-1) + velocity * dt
fallback_heading(t)  = cached_heading(t-1) + yaw_rate * dt
```

Cost: O(1) per object. Total for all fallback objects: ~0.01ms.

The planner receives these as lower-confidence predictions (confidence score reduced proportional to cache age and fallback count).

### Composition with Mechanism 1

The two mechanisms compose naturally:

```
All N tracked objects
  |
  v
[Divergence Gate] -----> FRESH objects (use cache, 0ms)
  |
  v
STALE objects (K << N)
  |
  v
[Risk Scoring] ---------> risk_i for each STALE object
  |
  v
[Budget Knapsack] -------> Selected (predict with MTR)
  |                   \--> Not selected (linear fallback)
  v
[MTR Prediction] -------> New predictions for selected subset
  |
  v
[Merge Cache] -----------> Complete prediction set for all N objects
```

### Example: N=32, 128 detections, 89 tracks

Without adaptation:
- Fusion: 84ms, Tracking: 61ms, MTR (89 tracks): 155ms
- Total: 300ms (3x over 100ms deadline)

With temporal amortization:
- 89 tracks, ~15 divergent
- MTR (15 tracks): ~35ms
- Total: 84 + 61 + 35 = 180ms (still over deadline)

With temporal amortization + risk budgeting:
- Budget for prediction: 100 - 84 - 61 - 2 = -47ms (negative, fusion+tracking alone exceed deadline)
- At N=32, the system needs rate adaptation too (reduce to 10Hz, budget becomes 100ms for compute)

With 10Hz rate + temporal amortization + risk budgeting:
- Fusion: 84ms, Tracking: 61ms (these don't change with rate)
- Available for prediction: 200ms - 84 - 61 - 2 = 53ms
- Risk-budget selects top-8 risk objects for MTR: ~33ms
- Remaining 7 divergent objects get linear fallback: ~0.01ms
- Total: 84 + 61 + 33 = 178ms at 10Hz (fits in 200ms budget)
- 64 FRESH objects use cached predictions from the previous tick

### What This Looks Like at Different N

| N | Tracks | Divergent | Risk-selected | MTR time | Total | Budget | Fits? |
|---|--------|-----------|--------------|----------|-------|--------|-------|
| 4 | 11 | 3 | 3 | 28ms | 44ms | 100ms (20Hz) | Yes |
| 8 | 22 | 5 | 5 | 30ms | 63ms | 100ms (20Hz) | Yes |
| 16 | 44 | 8 | 8 | 33ms | 102ms | 100ms (20Hz) | Tight |
| 16 | 44 | 8 | 5 | 30ms | 99ms | 100ms (20Hz) | Yes |
| 32 | 89 | 15 | 8 | 33ms | 178ms | 200ms (10Hz) | Yes |
| 64 | 179 | 25 | 8 | 33ms | 327ms | 500ms (4Hz) | Yes |

The system adapts across three knobs simultaneously:
1. **Temporal amortization** reduces the set of objects needing prediction
2. **Risk budgeting** selects which of those objects get expensive prediction
3. **Rate adaptation** extends the tick budget when compute exceeds deadline

---

## Evaluation Plan

### Metrics

1. **Deadline compliance rate:** fraction of ticks where total compute <= deadline
2. **Safety-critical coverage:** fraction of high-risk objects (r_i > threshold) that received MTR prediction
3. **Prediction error (ADE/FDE):** measured separately for MTR-predicted vs cached vs fallback objects
4. **Planner outcome:** collision rate, false brake rate, success rate in blind intersection scenario

### Baselines

1. **Static full pipeline:** MTR on all objects every tick (current, exceeds deadline at N~10)
2. **Static linear-only:** Linear prediction on all objects (always meets deadline, poor quality)
3. **Rate-only adaptation:** Reduce tick rate when compute exceeds deadline, no amortization or risk budgeting
4. **Temporal amortization only:** Divergence gating without risk budgeting
5. **Full adaptive:** Temporal amortization + risk budgeting + rate adaptation

### Experiments

1. N sweep {4, 8, 16, 24, 32, 48, 64} on RTX 4080 SUPER and Azure A10
2. Threshold sensitivity: THRESH_POS = {0.5, 1.0, 2.0, 5.0} meters
3. Ablation: each mechanism independently vs composed
4. Closed-loop: run in eCAV simulator with blind intersection scenario, measure collision and false brake rates

---

## Implementation Priority

1. **Prediction cache** in edge manager (dictionary, trivial)
2. **Divergence check** after tracking (numpy vector ops, ~20 lines)
3. **Subset MTR call** (modify MTR batch construction to accept track ID subset)
4. **Risk scoring** (geometric computation from track state, ~50 lines)
5. **Knapsack selection** (sort + greedy, ~10 lines)
6. **Rate adaptation** (existing edge_dt parameter, monitor p95 latency)
