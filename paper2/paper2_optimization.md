# Edge World-Model Controller: Optimization Formulation

## Problem Statement

At each tick t, the edge receives fused perception from N cooperative agents and must produce trajectory predictions for M tracked objects within a deadline D (e.g., 100ms). The full pipeline (fusion + tracking + prediction) exceeds D as N grows. The controller must select, per tick, how to allocate compute across pipeline stages and across objects to maximize planner-relevant safety while meeting the deadline.

## Notation

| Symbol | Definition |
|--------|-----------|
| D | Deadline (ms), e.g. 100 |
| N_t | Number of cooperative agents at tick t |
| M_t | Number of tracked objects at tick t |
| O_i | Tracked object i, i in {1, ..., M_t} |
| r_i | Risk score of object i (higher = more safety-critical) |
| c^f_j | Compute cost of fusion tier j |
| c^p_i | Compute cost of predicting object i with MTR |
| c^T | Tracking compute cost (function of M_t) |
| x_i | Binary decision: 1 = predict O_i with MTR, 0 = use cache/linear |
| f | Fusion tier selection: f in {high, low} |
| k | Rate multiplier: ticks between updates (1 = every tick, 2 = every other) |

## Per-Tick Compute Model

Total compute at tick t:

```
C(f, x, k) = c^f_f + c^T(M_t) + c^base_MTR * 1[sum(x) > 0] + c^marginal * sum(x_i)
```

Where:
- c^f_high = Where2Comm fusion cost (measured: ~1ms * N)
- c^f_low = Maxout fusion cost (measured: ~0.02ms * N)
- c^T(M) = AB3DMOT tracking cost (measured: ~0.7ms * M)
- c^base_MTR = MTR base overhead, ~25ms (model load, batch setup)
- c^marginal = MTR marginal per-object cost, ~1.5ms/object

## Objective: Safety-Critical Coverage

The planner needs accurate predictions for objects on conflict trajectories. Define utility:

```
U(x) = sum_{i=1}^{M_t} r_i * q_i(x_i)
```

Where q_i is the prediction quality for object i:
- q_i(x_i = 1) = 1.0 (fresh MTR prediction)
- q_i(x_i = 0, cached, age a) = max(0, 1 - a * alpha) where alpha is a decay rate
- q_i(x_i = 0, linear fallback) = beta (constant, e.g. 0.3)

## Optimization Problem

```
maximize    U(x) = sum_i r_i * q_i(x_i)

subject to  C(f, x, k) <= D * k          (deadline constraint)
            x_i in {0, 1}                 (binary prediction decision)
            f in {high, low}              (fusion tier)
            k in {1, 2, 4, 5, 10}        (rate multiplier)
            sum(x_i) >= 1 if any r_i > r_critical  (must predict at least one critical object)
```

The deadline constraint uses D * k because at rate multiplier k, the effective budget per update is D * k milliseconds (e.g., at 10Hz with 100ms deadline, budget is 200ms).

## Solution: Greedy Decomposition

The problem decomposes into three sequential decisions:

### Step 1: Rate Selection

Choose the minimum rate multiplier k such that fixed costs fit within the budget:

```
k* = min { k in {1,2,4,5,10} : c^f_high + c^T(M_t) + c^base_MTR <= D * k }
```

If even c^f_high + c^T exceeds D*1, try k=2, etc. If c^f_low + c^T fits at a lower k, prefer that (Step 2 decides fusion tier).

### Step 2: Fusion Tier Selection

Given k*, compute remaining budget after tracking:

```
B_fuse = D * k* - c^T(M_t) - c^base_MTR
```

If B_fuse >= c^f_high: use Where2Comm (better detection quality)
Else if B_fuse >= c^f_low: use maxout
Else: increase k and retry

### Step 3: Prediction Budget Allocation (Knapsack)

Remaining budget after fusion and tracking:

```
B_pred = D * k* - c^f_{f*} - c^T(M_t) - c^base_MTR
```

Maximum number of objects to predict:

```
n_pred = floor(B_pred / c^marginal)
```

Among M_t objects, select the n_pred with highest risk scores. Specifically, among the divergent objects (those whose tracked state has diverged from cached prediction):

```
D_t = { i : ||pos_i - cached_pos_i|| > thresh_pos  or  cache_age_i > max_age }
```

Sort D_t by risk r_i descending. Select top min(n_pred, |D_t|). Remaining divergent objects get linear fallback. Non-divergent objects use cached predictions.

### Algorithm

```
function ADAPTIVE_PREDICT(M_t, D, measurements):
    # Divergence gating
    divergent = {i in M_t : DIVERGED(i) or NO_CACHE(i)}
    fresh = M_t \ divergent

    # Risk scoring
    for i in divergent:
        r_i = RISK_SCORE(O_i, ego_vehicles)

    # Rate selection (greedy, ascending k)
    for k in [1, 2, 4, 5, 10]:
        budget = D * k

        # Try high fusion first
        for f in [high, low]:
            fixed = c^f_f + c^T(M_t)
            if fixed > budget:
                continue

            remaining = budget - fixed - c^base_MTR
            if remaining < 0:
                continue

            n_pred = floor(remaining / c^marginal)
            if n_pred >= 1:
                # Feasible: select top-risk divergent objects
                selected = TOP_K(divergent, n_pred, key=r_i)
                pruned = divergent \ selected

                return {
                    rate: 1/k * base_rate,
                    fusion: f,
                    mtr_objects: selected,
                    cache_objects: fresh,
                    linear_objects: pruned,
                }

    # Fallback: all linear, minimum rate
    return {rate: base_rate/10, fusion: low, mtr_objects: {},
            cache_objects: fresh, linear_objects: divergent}
```

### Complexity

- Divergence check: O(M_t) vector operations
- Risk scoring: O(M_t * N_ego) arithmetic
- Sorting: O(M_t log M_t)
- Total controller overhead: < 1ms for M_t < 500

## Risk Score Function

```
r_i = speed_i * proximity_factor_i * occlusion_factor_i * lane_conflict_i
```

Where:
- speed_i: object's speed in m/s (from Kalman filter state)
- proximity_factor_i = max over ego vehicles v of:
    - 10.0 if dist(O_i, v) < 5m
    - 3.0 if dist(O_i, v) < 20m
    - 1.5 if dist(O_i, v) < 50m
    - 0.1 if dist(O_i, v) >= 50m
- occlusion_factor_i = 2.0 if O_i is not directly visible to any ego vehicle, else 1.0
- lane_conflict_i = 1.0 if O_i is on a lane that intersects any ego vehicle's planned path within 5s, else 0.0

Objects with lane_conflict = 0 get zero risk regardless of other factors. This is the key pruning: most tracked objects (parked cars, vehicles on non-conflicting roads) are irrelevant to any ego vehicle's planner.

## Properties

1. **Feasibility:** The algorithm always returns a feasible solution (worst case: all linear, minimum rate).

2. **Monotonicity:** As budget increases (lower k or fewer agents), more objects get MTR prediction. The system gracefully degrades as load increases and recovers as load decreases.

3. **Safety guarantee:** Objects with r_i > r_critical are always predicted first. The minimum constraint ensures at least one critical object gets MTR prediction if any exist.

4. **Bounded staleness:** Cached predictions are at most max_cache_age * tick_dt seconds old. The divergence threshold bounds the position error of cached predictions to thresh_pos meters.

5. **Online adaptation:** The cost estimates (c^base_MTR, c^marginal) are updated each tick from measured execution time, allowing the controller to adapt to GPU load variations.

## Evaluation Metrics

1. **Deadline compliance rate:** fraction of ticks where C <= D * k
2. **Safety-critical coverage:** fraction of objects with r_i > r_critical that received MTR prediction
3. **Prediction ADE/FDE:** measured separately for MTR vs cached vs linear objects
4. **Effective update rate:** actual Hz delivered to planners (lower k = lower rate)
5. **Planner outcome:** collision rate, false brake rate, TTC distribution in closed-loop evaluation

## Comparison Baselines

| Policy | Description | Knobs |
|--------|-------------|-------|
| Static-Full | MTR on all objects, Where2Comm, max rate | None |
| Static-Linear | Linear on all objects, Where2Comm, max rate | None |
| Rate-Only | MTR on all, reduce rate when over deadline | k only |
| Amort-Only | Divergence gating, MTR on divergent subset | x only |
| Full-Adaptive | All knobs: f, x, k jointly optimized | f, x, k |
