# Plan: Infra-only (I2V) baseline ladder + self-ghosting as a multi-source result

## Context

MobiCom rejected the safety-envelope paper partly because reviewers A/B read
self-ghosting as an artifact of a self-designed V2X2V pipeline, and asked whether
it applies to VIPS/VI-Eye/VRF/CIP-style systems. The SenSys resubmission must
reframe the result: self-ghosting is not caused by vehicles uploading object
lists, it is a publish-boundary identity failure that can arise whenever
multiple edge-side sources observe the ego and the edge republishes an
object-level world model back to that ego.

The infra-only (I2V) baseline is the evidence for that reframing. It reproduces
self-ghosting with **zero vehicle object uplink**, using only infrastructure
sensors, which the Oracle case cannot show. This makes the discriminator
"number of independent sources observing the ego," not "did a vehicle
participate."

## The run ladder (architecture discriminator)

| # | Run | Sources | Purpose / expected lesson |
|---|-----|---------|---------------------------|
| 1 | Oracle | single | physics floor under AoI; no multi-source identity ambiguity. Already established. |
| 2 | 1-RSU infra-only / I2V | single (infra) | direct I2V envelope without multi-source ambiguity |
| 3 | 2-RSU infra-only / I2V | multi (infra only) | multi-source infra fusion can create duplicate ego-consistent tracks **without vehicle uplink** |
| 4 | 2-RSU infra-only + ego-uniqueness | multi (infra only) | the invariant removes multi-source infra duplication too |
| (5) | LF / VIPS | vehicle+infra | vehicle+infra multi-source edge-published envelope (existing) |
| (6) | LF / VIPS + SBA | vehicle+infra | ego-uniqueness removes the logic boundary (existing) |

VRF (early fusion) is a separate baseline tracked in `vrf_baseline.md`; it is the
disagreement-free corner (single detector on pooled raw data) and its cost.

## Claims (carefully phrased)

- Single-source still has AoI and still reaches the physics-limited boundary at
  high staleness. Oracle can fail there. What single-source lacks is
  **multi-source identity ambiguity**. Do NOT write "single source does not
  cause AoI."
- Multi-source infra-only exhibits **higher duplicate / ego-uniqueness pressure**
  than single-source under the same AoI model, and the ego-uniqueness invariant
  removes that logic failure while leaving the shared physics floor unchanged.
  Do NOT assert a nonzero "ghosting as AoI -> 0" intercept before the data shows
  it.
- Reframed top-line: the age-at-use safety-envelope method applies across I2V and
  V2X2V edge assistance. Self-ghosting is not universal to all I2V designs; it
  appears when an edge-published world model can include duplicate ego-consistent
  tracks. SBA (ego-uniqueness) is the invariant for that architecture class.

## Self-ghosting mechanism (code-accurate)

The collision checker is `_check_trajectory_intersection` in
`ecav/core/plan/behavior_agent.py:42-78`. It is time-synchronized trajectory
intersection, NOT radius proximity:

- `PATH_RESOLUTION = 0.1` m/point; `lateral_threshold = 2.0` m; `max_steps = 25`.
- At each horizon step `i`, ego future position is `ego_speed_mps * (i*dt)`
  indexed along the planned path (`ego_path_x/y`); obstacle predicted position is
  `pred_traj[i]`. Brake triggers iff their separation `< 2.0` m at the same
  time index.

Consequence (this is the sharpened claim, faithful to the code):

- A delayed ego track that trails the vehicle with **consistent** velocity
  predicts forward along the ego's own path, stays behind, and never enters the
  collision set. Benign.
- Self-ghosting arises when **multi-source disagreement** yields an unlabeled
  ego-consistent track whose predicted kinematics (velocity/heading) differ
  enough that its trajectory intersects the ego's time-indexed plan within the
  25-step horizon.

Paper sentence to add near "When does self-ghosting occur?":

> The collision checker compares time-indexed positions: at each horizon step it
> tests whether the ego's future position along its planned path falls within a
> lateral threshold of the obstacle's predicted position. A delayed ego track
> that trails the vehicle with consistent velocity predicts forward along the
> ego's own path and never enters this set. Self-ghosting arises when
> multi-source disagreement yields an unlabeled ego-consistent track whose
> predicted kinematics differ enough that its trajectory intersects the ego's
> time-indexed plan.

## RSS framing (physics floor only)

Use RSS to answer reviewer B's "why 100 ms" by grounding the **physics-limited**
boundary, not to explain self-ghosting (RSS assumes correct perception; a
self-ghost is a false positive). One paragraph, citable (IEEE 2846-2022 for
parameters):

> We interpret the high-AoI boundary through an RSS-style braking margin: a
> track is admissible only if the believed gap minus staleness uncertainty
> remains above the minimum safe distance implied by ego speed, actuation delay,
> and guaranteed braking. This explains why the acceptable \(\Delta_{use}\)
> budget shrinks with speed and weaker braking. RSS bounds the physics-limited
> region; it does not explain self-ghosting, which is a logic-level identity
> failure.

Longitudinal RSS min safe distance (rear ego r behind front obstacle f):

```
d_min = v_r*rho + 0.5*a_accel*rho^2 + (v_r + rho*a_accel)^2 / (2*a_brake_min)
        - v_f^2 / (2*a_brake_max),  clamped >= 0
```

Dominant term `v_r^2 / (2*a_brake_min)` grows quadratically with ego speed ->
source of the speed dependence. Optional validation overlay: plot
`tau_max(v_ego)` (largest admissible age) over the measured S_op vs Delta_use
envelope; if the empirical physics floor matches the RSS curve, the envelope is
shown to match a formal safety model. Caveat: longitudinal RSS is car-following;
LTAP is crossing geometry. Apply RSS from the moment cross-traffic becomes an
in-path obstacle (the approach phase), or cite the RSS intersection extension.
Do not claim literal car-following RSS for the crossing.

## Implementation status and remaining work

Done:
- `InfraOnlyEdge` and `CIPEdge` implemented and registered
  (`edge_manager/__init__.py`: `INFRA_ONLY`, `CIP`).
- `rsu2` added at `[-40.0, 140.0, 7.0]` to all six scenario_3 late-fusion
  configs.
- 2-RSU infra-only smoke config + runner created:
  `config_yaml/openscenario_3_edge_infra_only_smoke.yaml` (manager_type:
  infra_only, 2 RSUs, 1 CAV) and `openscenario_3_edge_infra_only_smoke.py`
  (delegates to the late-fusion runner; class chosen from YAML).

Remaining:
1. **1-RSU infra-only config** (ladder rung 2): copy the infra-only smoke config,
   remove rsu2. `openscenario_3_edge_infra_only_1rsu_smoke.yaml` + runner.
2. **Confirm 2-RSU geometry produces a surviving duplicate** in the smoke run.
   This is the gating check: if rsu2's FOV does not overlap the conflict zone,
   no duplicate, no data point -> reposition rsu2. Nothing downstream is valid
   until this holds.
3. **Ego-uniqueness for infra-only (ladder rung 4).** SBA as implemented is
   beacon-based and `InfraOnlyEdge` forces `enable_sba=False`
   (`edge_manager_infra_only_...py:45-47`); there is no vehicle beacon to anchor.
   NOT trivial. But the edge already holds each managed vehicle's true pose via
   `vehicle_manager_list`, so ego-uniqueness can be enforced against that known
   pose instead of an uploaded beacon. Design note: add an infra-side
   ego-uniqueness path that suppresses/merges tracks matching a managed vehicle's
   known pose. Same invariant, different anchor source. Scope this only if rung 3
   shows duplicates worth removing.
4. **Fix eval output tag**: the infra-only runner delegates to late-fusion's
   `run_scenario`, which reads its own module `SCENARIO_NAME`, so eval artifacts
   are tagged `openscenario_3_edge_late_fusion`. Cosmetic; fix after data.

## Paper change set (settled direction)

- Replace "representative late-fusion V2X pipeline" with an architecture-class
  framing: "edge fusion" as the short repeated term, defined once as "an
  edge-fusion design that reconciles detections from multiple independent
  sources into a world model published back to vehicles." Drop "cooperative"
  (implies vehicle cooperation; false for the 2-RSU infra case) and avoid
  "world-model design point" as too wordy.
- Add the architecture applicability table (Oracle / 1-RSU I2V / 2-RSU I2V /
  LF+VIPS / +SBA, plus VRF early-fusion row).
- Reorder eval: envelope method first, then architecture comparison, then
  failure-mode isolation, then SBA.
- Add the duplicate-source explanation (viewpoint/extent/timestamp/localization
  noise -> non-identical boxes -> conservative NMS -> residual duplicate tail).
- Fix VRF bib + the `system_architecture.tex:8` claim that VRF co-locates compute
  at the RSU (it fuses on the vehicle). See `vrf_baseline.md`.

## Verification

1. Smoke run rung 3 (2-RSU infra-only): confirm a physical actor (cross-traffic
   Tesla and/or ego) gets two surviving tracks in a reconciled frame at low/zero
   injected delay. Log per-frame track count per physical actor.
2. Smoke run rung 2 (1-RSU infra-only): confirm single track per actor, no
   duplicate pressure.
3. Confirm the existing collision checker fires on the duplicate only when its
   predicted trajectory intersects the ego's time-indexed plan (instrument the
   brake attribution already present in `behavior_agent`).
4. Closed-loop ladder across rungs 1-4 under matched AoI; compare S_op and
   self-ghost / brake-attribution events.

## Launch

`./start_actors.sh openscenario_3_edge_infra_only_smoke` (interactive; needs
CARLA up, which it now is). The full distributed stack (C++ orchestration, YOLO
gRPC, scenario_runner, actors) is brought up by the script.
