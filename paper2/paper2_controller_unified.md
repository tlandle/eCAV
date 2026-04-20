# Unified Adaptive Controller Definition

This is the single canonical definition of the controller for Paper 2. It supersedes the earlier descriptions in `paper2_architecture.md` and `paper2_optimization.md` which presented inconsistent formulations. All plots, code, and paper text must match this document.

## Scope

The controller is a greedy scheduler that runs once per tick on the edge. It does not claim global optimality. It claims:

1. **Empirical deadline compliance.** Under the measured workloads in our evaluation (N up to 64 agents, M up to 256 tracked objects, realistic scene distributions), the selected allocation keeps total edge compute under the policy-specified deadline in nearly all ticks. We report the miss rate and its tail distribution explicitly rather than asserting a worst-case guarantee.
2. **Priority preservation.** Policy-defined critical objects (those with high risk score) are scheduled for fresh prediction whenever compute budget permits, subject to the accuracy of the estimated cost model.
3. **Graceful degradation.** When the estimated budget cannot accommodate all critical objects, the scheduler preserves the highest-risk subset and assigns cached or linear predictions to the remainder.

The controller does not solve the allocation problem exactly. It searches over a small discrete set of candidate agent counts K and selects the one that maximizes estimated utility, then runs a greedy knapsack over the tracked objects at the selected K. The gap between this scheduler and an ILP oracle is measured and reported in the evaluation.

## Decision Variables

Exactly two primary decisions per tick:

| Variable | Type | Meaning |
|----------|------|---------|
| K | integer in [1, N] | Number of cooperative agents retained after input contribution filtering |
| x_i | binary, i in 1..M | Whether to predict tracked object i with fresh MTR inference |

Rate adaptation (tick-skipping) is a fallback mechanism applied only when no (K, x) pair is feasible. It is not a first-class decision variable and is not part of the core contribution. Fusion tier swapping (Where2Comm vs cheaper alternatives) is excluded from the controller because our measurements show the cost difference is small relative to the total pipeline cost (12 ms vs 1 ms at N=16 on post-backbone features).

## Cost Model

Total edge compute per tick, as a function of decision variables:

```
C(K, x) = c_filter(N) + c_fusion(K) + c_detection(K) + c_tracking(M(K)) + c_predict(x) + c_overhead
```

Where:
- `c_filter(N)`: input contribution scoring cost. O(N) agents through cls_head, O(N^2) for pairwise FOV overlap. Measured, typically < 2 ms for N up to 128.
- `c_fusion(K)`: Where2Comm attention fusion on K agents. Approximately linear in K on post-backbone features.
- `c_detection(K)`: post-processing of fused output to bounding boxes. Approximately constant.
- `c_tracking(M)`: AB3DMOT tracking. Approximately linear in the number of detections (which scales with K).
- `c_predict(x) = c_base * 1[sum(x) > 0] + c_marginal * sum(x_i)`: MTR prediction with base overhead plus per-object marginal cost.
- `c_overhead`: fixed overhead for serialization, distribution, profiling. Approximately constant.

The separability assumption (each stage cost depends only on its own input count) must be validated by measurement. This is a first-class paper deliverable: a table or figure showing predicted vs measured latency for each stage.

## Utility Function

The scheduler maximizes a risk-weighted prediction quality utility:

```
U(K, x) = sum_{i in M} r_i * q_i(x_i, cache_age_i)
```

Where:
- `r_i` is the risk score of tracked object i (defined below). Smooth continuous function, not binary.
- `q_i(x_i, a)` is the prediction quality of object i given its decision and cache age:
  - If `x_i = 1` (fresh MTR): `q_i = 1.0`
  - If `x_i = 0` and cached (age a ticks): `q_i = max(0, 1 - a / max_cache_age)`
  - If `x_i = 0` and linear fallback: `q_i = q_linear` (constant, empirically around 0.3)

When rate adaptation is active (k > 1), the utility decays to penalize staleness:

```
U_with_rate(K, x, k) = (1/k) * sum_i r_i * q_i(x_i, cache_age_i)
```

This makes k > 1 strictly worse than k = 1 in utility terms, so the scheduler prefers maintaining 20 Hz. Rate adaptation is chosen only when no k = 1 allocation is feasible.

## Risk Score (Smooth Continuous)

The risk score is continuous and degrades gracefully rather than collapsing to zero:

```
r_i = speed_i * proximity_i * occlusion_i * conflict_proximity_i
```

Where:

- `speed_i`: object's speed from Kalman filter state, in m/s
- `proximity_i = max_v exp(-dist_iv / d_scale)` where `d_scale = 20 m`. Smooth decay with distance to the nearest ego vehicle.
- `occlusion_i`: 1.0 if visible to any ego, 2.0 if visible only to edge infrastructure (roadside units). Continuous boost for objects the egos cannot see directly.
- `conflict_proximity_i = max(0, 1 - closest_approach_dist_i / safety_radius)` where `safety_radius = 5 m`. Path-conflict score based on the closest-approach distance between object i's predicted trajectory and any ego vehicle's planned path over the next T seconds. This is a smooth heuristic score in [0, 1], not a statistical probability. No calibration to a physical event rate is claimed.

`conflict_proximity_i` replaces the earlier binary `lane_conflict`. It never collapses to zero for an object that is near but not currently intersecting a planned path. A sudden ego replan shifts the score continuously instead of causing an on-off transition.

## Divergence Gating (Time-Domain Tolerance)

The divergence threshold is a constant time-domain tolerance rather than a constant spatial tolerance. Objects are flagged as divergent if their observed state deviates from the cached prediction by more than what the object could plausibly move in `tau_pos` seconds:

```
thresh_pos_i = max(0.5, speed_i * tau_pos)   # meters
thresh_head_i = max(5_deg, angular_vel_limit)  # degrees
```

Where `tau_pos = 0.1 s` sets the spatial tolerance equivalent to 100 ms of motion at the object's current speed. At 2 m/s the threshold is 0.5 m; at 30 m/s it is 3 m. The floor of 0.5 m prevents spurious divergence flags on stationary objects due to tracker noise.

Interpretation: we tolerate up to 100 ms of unmodeled velocity drift before forcing a fresh prediction, regardless of absolute speed. A slow object's small spatial error triggers re-prediction if it exceeds the floor; a fast object's larger spatial error is acceptable as long as it corresponds to less than 100 ms of drift. This is a constant time-domain threshold, not a tighter spatial tolerance for faster objects.

An object is marked divergent and re-predicted if:
- Position error from cached prediction exceeds `thresh_pos_i`, OR
- Heading error exceeds `thresh_head_i`, OR
- Cache age exceeds `max_cache_age` (force refresh)

## Input Contribution Filtering

The input filter selects K agents from N by ranking each agent's contribution to the ego vehicles' safety zones. The score per agent:

```
contribution_i = (1 - alpha - beta) * conflict_zone_score_i
                 + alpha * unique_coverage_i
                 + beta * occluded_region_coverage_i
```

Where:

- `conflict_zone_score_i = sum(psm_i * conflict_zone_mask)`. Integral of the agent's detection confidence over the union of ego vehicles' planned paths extended forward by 5 seconds.
- `unique_coverage_i`: fraction of the agent's FOV not covered by any other agent's FOV. Computed from agent poses and LiDAR range. Agents covering regions no one else sees get a uniqueness bonus.
- `occluded_region_coverage_i`: 1 if the agent is the only source of information for any region inside the conflict zone, else 0. Protects the "singular node observing a heavily occluded region" edge case.

Weights: `alpha = 0.2`, `beta = 0.3`. These are empirical and can be tuned.

Select top-K agents by contribution. K is bounded by the fusion budget: K_max is the largest value for which `c_fusion(K) + c_tracking(M(K))` fits inside the deadline, minus overhead and a prediction reserve.

## Tracked-Object Count Estimator

Before fusion and tracking actually run, the controller must estimate how many tracked objects will result from selecting K agents. The estimator uses a rolling window over recent ticks:

```
M_hat(K) = M_base + mean_K / mean_N_observed * K + slack
```

Where:
- `mean_K / mean_N_observed` is the empirical tracks-per-agent ratio from the last W ticks (default W=20).
- `M_base` accounts for persistent tracks already in the tracker that do not depend on the current tick's agent selection.
- `slack` is a conservative over-estimation (10% of the mean, or at least 2 tracks) that prevents the pre-tracking budget from being optimistic.

After fusion and tracking actually run, the controller recomputes the residual budget from the measured M and adjusts the MTR subset selection. The pre-tracking estimate is used only to pick K; the final MTR selection uses the post-tracking measured count. This eliminates the circular dependency in the earlier formulation.

The estimator's accuracy is reported in the evaluation as part of the cost model fit validation.

## Scheduler Algorithm

The scheduler searches over candidate K values and selects the one that maximizes estimated utility, rather than the largest feasible K. Since K has a small discrete domain (e.g., {4, 8, 16, 24, 32, N}), the search is inexpensive.

```
function SCHEDULE(N_agents, tracked_objects, deadline, cost_model):
    # Step 1: Score agents by contribution (done once, reused per K)
    contribution_scores = compute_contribution_scores(N_agents)

    # Step 2: Search over candidate K values for best estimated utility
    best = None
    candidate_Ks = [k for k in [4, 8, 16, 24, 32, N_agents] if k <= N_agents]
    for K in candidate_Ks:
        # Estimate tracked-object count at this K (with slack)
        M_hat = estimate_M(K)

        # Estimate fixed costs under this K (filter, fusion, detection, tracking)
        fixed_est = c_filter(N) + c_fusion(K) + c_detection(K) + \
                    c_tracking(M_hat) + c_overhead

        if fixed_est > deadline:
            continue  # infeasible, skip

        # Remaining budget for prediction
        budget_pred = deadline - fixed_est
        if budget_pred < c_base:
            continue  # cannot even run MTR once

        max_mtr_slots = floor((budget_pred - c_base) / c_marginal)

        # Estimate utility: select top-risk divergent objects up to max_mtr_slots
        # Use cached risk scores and divergence flags from previous tick
        # as a proxy for what this tick's risk landscape will look like
        divergent_hat = estimate_divergent_set(K)
        risks = [compute_risk_smooth(t) for t in divergent_hat]
        top_slots = sorted(risks, reverse=True)[:max_mtr_slots]

        estimated_U = sum(r * 1.0 for r in top_slots) + \
                      sum(r * q_linear for r in risks[max_mtr_slots:]) + \
                      sum(r * cache_quality(t.cache_age) for t in fresh_hat(K))

        if best is None or estimated_U > best['U']:
            best = {'K': K, 'U': estimated_U, 'max_mtr': max_mtr_slots}

    if best is None:
        return schedule_with_rate_fallback(...)   # extreme case

    K_star = best['K']

    # Step 3: Select top-K agents and run fusion + tracking
    selected_agents = top_k(N_agents, contribution_scores, K_star)
    tracks = run_fusion_tracking(selected_agents)   # real GPU work
    M_actual = len(tracks)

    # Step 4: Recompute residual budget from measured M
    actual_fixed = measured_fusion_ms + measured_detection_ms + measured_tracking_ms
    actual_residual = deadline - actual_fixed - c_overhead - c_base
    max_mtr_final = max(0, floor(actual_residual / c_marginal))

    # Step 5: Divergence gating with velocity-normalized thresholds
    divergent = [t for t in tracks if diverged(t)]
    fresh = [t for t in tracks if t not in divergent]

    # Step 6: Smooth risk scoring on divergent set
    for t in divergent:
        t.risk = compute_risk_smooth(t, ego_vehicles)

    # Step 7: Final knapsack over actual divergent objects at measured budget
    sorted_by_risk = sorted(divergent, key=lambda t: t.risk, reverse=True)
    selected_for_mtr = sorted_by_risk[:max_mtr_final]
    pruned = sorted_by_risk[max_mtr_final:]

    return {
        'K_star': K_star,
        'agents': selected_agents,
        'mtr_tracks': selected_for_mtr,
        'cached_tracks': fresh,
        'linear_tracks': pruned,
    }
```

Key differences from the earlier "max feasible K" version:

1. **K is chosen to maximize estimated utility, not to be maximal.** A smaller K may reduce fusion and tracking cost enough to fund more high-risk MTR predictions, yielding higher total utility. The search evaluates all candidate K values.

2. **M is estimated with slack before fusion runs, then recomputed from the measured value.** The pre-tracking estimate is only used for K selection. After fusion and tracking produce the actual track count, the MTR subset is chosen from the measured residual budget. This removes the circular dependency.

3. **Budget accounting distinguishes estimated and actual.** The K selection uses estimated costs. The final MTR knapsack uses measured costs. Reported deadline compliance is against actual totals.

Complexity:
- Contribution scoring: O(N^2) for FOV overlap + O(N) for cls_head passes. Small constants.
- K search: at most 6 candidate values, each requires estimator call and utility computation. O(6 * (M_hat + N)).
- Final knapsack: O(M log M).
- Total controller overhead: under 3 ms for N up to 64 and M up to 256. Validated by measurement.

Since the scheduler uses estimated utility for K selection and measured costs for the final MTR knapsack, the utility it achieves per tick is approximate. We report the gap between the estimated and realized utility and between the scheduler and an ILP oracle for small instances.

## Calibration of Quality and Risk Weights

The quality function `q_i`, the linear-fallback constant `q_linear`, the cache decay rate, and the risk score factors (`d_scale`, `safety_radius`, `tau_pos`, the risk exponents) are not derived from first principles. They are control-policy surrogates with the following calibration procedure:

1. **Fit against offline prediction error.** Run the full pipeline on a held-out set of OPV2V or CARLA scenes. For each tracked object, record the actual ADE of fresh MTR, cached predictions at various ages, and linear extrapolation. Fit `q_linear` and the cache decay so that `q_i` approximately matches the inverse of observed error. Report the fit R-squared.

2. **Correlate with closed-loop planner outcomes.** In the closed-loop CARLA evaluation, for each tick record the per-object risk score and the planner's resulting action (brake event, lane change, collision avoidance). Compute the correlation between the risk score distribution and planner safety events (TTC degradation, false brakes, collisions). Risk-score factors are tuned so that high-risk objects account for the majority of planner-relevant events.

3. **Sensitivity analysis.** Report the effect of perturbing each weight by +/- 50% on deadline compliance and on high-risk ADE. Weights that cause non-monotonic behavior under perturbation are flagged as fragile.

This subsection is not a derivation. It is a calibration procedure. Reviewers should expect the weights to generalize within the data regimes we evaluate and degrade outside them.

## What the Controller Does NOT Claim

1. **Not globally optimal.** The scheduler uses a two-level greedy decomposition (K selection followed by a knapsack over divergent tracks) with estimated costs and estimated utilities. The gap between this and a joint ILP optimum is measured on small instances and reported.
2. **Not a worst-case deadline guarantee.** Compliance depends on the accuracy of the cost model and the tracked-object count estimator. The paper reports empirical deadline compliance, tail latency, and the distribution of prediction errors from the estimator rather than claiming a hard bound.
3. **Not a hard safety guarantee.** Priority preservation is best-effort. An object with high risk may miss fresh prediction if the cost estimator underestimates load or the risk score is noisy.
4. **Not the only viable architecture.** The paper argues that sidelink intermediate fusion is infeasible (demonstrated with ns-3 measurements showing PRR collapse under realistic payloads) and that an edge-hosted pipeline is a viable alternative. We do not claim it is the only option.
5. **Quality surrogates are calibrated, not derived.** The quality function `q_i(x_i, cache_age_i)`, the linear-fallback constant `q_linear`, and the risk score weights are control-policy surrogates. They are calibrated against offline prediction error on held-out data and correlated with closed-loop planner outcomes. They are not derived from a first-principles safety model.

## Changes from Earlier Drafts

Earlier drafts are inconsistent and should be ignored where they conflict with this document.

| Item | Earlier draft | This document |
|------|--------------|---------------|
| Decision variables | k, f, x (three) | K, x (two) |
| Rate adaptation | Primary mechanism | Fallback only, not in core contribution |
| Fusion tier | First-class knob | Excluded (cost difference too small) |
| Risk score | Binary lane_conflict | Smooth P_conflict with continuous distance falloff |
| Divergence thresholds | Static (1 m, 10 deg) | Velocity-normalized (scales with speed) |
| Claims | "Globally optimal", "safety guarantee", "sole viable topology" | "Deadline-feasible greedy", "priority preservation", "viable alternative" |
| Input filter | Not in earlier draft | Primary mechanism, replaces rate adaptation |
| Utility | No rate penalty | Rate adaptation carries AoI decay penalty |

## Required Validations

The paper must include the following measurements to substantiate the controller design:

1. **Cost model fit.** Predicted vs measured latency for `c_fusion`, `c_tracking`, `c_base`, `c_marginal`. Report fit error and R-squared.
2. **Controller overhead.** Wall-clock time spent in the controller itself (contribution scoring, risk scoring, knapsack) as a fraction of total tick time.
3. **Workload disentanglement.** Latency vs N, latency vs M (tracked object count), latency vs divergent count. Separate these axes to show the cliff is characterized along the right dimension.
4. **Conflict-zone retention.** Fraction of safety-critical GT objects observed by at least one agent in the filtered K-subset. Must stay high as N grows, or the filter is too aggressive.
5. **Controller stability over time.** Time-series of K, MTR count, cache hit rate, per-tick latency over 10+ seconds. Demonstrate no oscillation or cache thrash.
6. **Greedy gap to oracle.** For small instances (N <= 8), compare greedy allocation utility to ILP optimal. Show gap is bounded.
7. **Fixed-GT evaluation.** ADE, FDE, and miss rate against a fixed ground truth set. Missing predictions are penalized, not ignored.
8. **Risk-stratified quality.** ADE split by risk tier (high-risk vs low-risk). High-risk ADE must remain stable across N; low-risk ADE can degrade.
9. **Closed-loop safety correlation.** At least one experiment showing that high-risk ADE or miss rate correlates with planner safety outcomes (collision, false brake, TTC degradation).

Items 1-6 are implementation + measurement, achievable offline. Items 7-8 require the evaluation methodology fix already planned. Item 9 requires closed-loop CARLA evaluation.
