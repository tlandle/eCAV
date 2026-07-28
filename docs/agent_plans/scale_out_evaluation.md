# Scale-Out (Paper 3, NSDI) — Predictive Latent Migration Evaluation Plan

Paper repo: `github.com/tlandle/scale_out_nsdi` (approach: predictive latent migration,
uncodenamed). This plan lives
in the eCAV repo because the evaluation runs in the eCAV CARLA + ns-3 sim. It maps
the paper's six evaluation questions to concrete sim experiments, lists the build
work needed to produce each result, and defines the knobs, metrics, and sequencing.

Related: [[project_paper3_relay_review]], [[project_paper3_nsdi]], paper sections
`contents/evaluation.tex`, `contents/architecture.tex`.

## 0. Grounded current state (2026-06-30 audit)

What is REAL and measured:
- `migration/harness.py` — offline synthetic B/B1/B0 microbenchmark, no CARLA. Drives
  two `Mamba3DTracker` instances (source A, dest B) sharing weights, migrates the
  full Mamba latent (memo_bank + diff_memo_bank) by pickle round-trip, and reports
  post-handoff prediction error for full-latent (B) vs Reactive-Kalman
  (B1, history_depth=1) vs cold (B0). CHECK 1 state parity byte-equal, CHECK 2
  prediction parity < 1e-4, CHECK 3 cold delta. This is the paper's Table 1 (Q2 corner).

What EXISTS but is NOT wired into the live path:
- AB3DMOT KFState export/import (`edge_manager_pluggable_base.py:144-279`) — only on
  `_PluggableEdgeBase` (SOTA/Adaptive edges). The live late_fusion edge never calls it.
  Minor bug: uses `np.*` with no `import numpy` (would NameError if invoked).
- Full Mamba latent transfer (`factories.latent_from_tracklet` / `inject_latent_into_tracker`)
  — used only by `harness.py`.
- Locale / registry / binding-with-hysteresis (`locale.py`, `registry.py`, `binding.py`
  VehicleLocaleTracker) — only exercised by `smoke_test.py`. No YAML polygons.

What the LIVE multi-edge scenario actually does today
(`openscenario_3_multi_edge_late_fusion.py`):
- Edge = `late_fusion` (`PredictionLateFusionEdge` : `_BaseEdgeManager`) running
  AB3DMOT + linear predictor.
- Trigger = hardcoded `HANDOFF_TICK=60`, fires once (`binding.py:216`), not geometry.
- Handoff = single synchronous swap: `relinquish(vid)` pops the VM, `import_vehicle_state`
  is a base no-op (`edge_manager_base.py:767-771`), `accept(vm)` appends it. Export is the
  minimal base payload (`hidden_state=zeros(1)`, no kf/latent). So only VM ownership moves.
- Destination COLD-STARTS its tracker. No learned state migrates live.
- Link computes a `TransferCost` but never applies delay (`link.py:9-11`); YAML sets
  latency=0, jitter=0.
- Metrics = log lines only (`[HANDOFF]`, `[TRANSFER_COST]`); not persisted; no
  warm-vs-cold prediction-error measurement.

Bottom line: the microbenchmark is the only measured result. Every live claim (Q1
closed-loop gap, Q3 triggers, Q4 safety, Q5 scale, Q6 impairment) is unbuilt.

## 0b. Structural prerequisite: the learned stack must run on the evaluation edge

The gap only exists if the edge runs a stateful predictor/tracker. AB3DMOT + linear
predictor has no multi-frame learned state to lose, so a handoff on that stack shows
no gap. The evaluation edge must run the learned stack:
- Tracker: `Mamba3DTracker` (ssm3dmot) — hidden state = memo_bank/diff_memo_bank.
- Predictor: MTR (multiv2x worldfusion cfgs) — per-track context; needs 10+ frames.
The AB3DMOT KFState path is the paper's Kalman-snapshot BASELINE arm, not the system.
The linear predictor is the stateless control (no gap).

The two learned states are separable and have very different readiness:
- TRACKER latent (Mamba hidden state) is READY. It is validated in `harness.py`
  (CHECK 1/2 pass) and is MTR-independent. It is the paper's Table 1 result and the
  first live learned-state arm to wire.
- PREDICTOR context (MTR) is NOT ready on the WorldFusion/edge (intermediate-fusion)
  setup. Per [[project_mtr_intermediate_fusion_underpredict]] MTR works on late-fusion
  + GT-injection but under-predicts on WorldFusion intermediate fusion; the WF→MTR
  training data pipeline is still mid-build (KB 2026-06-29). See P0 below.

## 0c. HARD PREREQUISITE P0: train + validate MTR on WorldFusion (Multi-V2X)

Any LIVE experiment that uses MTR as the predictor (the full WF-fusion -> tracker ->
MTR -> planner service: Q1, Q4, Q5, Q6, and the predictor-context arm of Q2) is BLOCKED
until MTR is trained on WorldFusion fused features and validated (sane AP + FDE on a held
locale). Running the migration eval before P0 gives numbers dominated by predictor error,
not migration quality. P0 is NOT part of the migration work; it is a gate in front of it.
- Status: WF->MTR (CMP-style) data generation in progress (`ecav/core/prediction/mtr/tools/
  export_wf_for_mtr.py`, `datasets/multiv2x_multiego_dataset.py`, intention points DONE,
  translaug fine-tune model pulled). Frame-decode / RSU-ego-frame association being fixed.
- Validation gate before P0 is "done": MTR trained on WF features reproduces reasonable
  FDE on held frames AND the live `WorldFusionEdge` + MTR produces sane forecasts on a
  boundary scenario (no systematic under-prediction) — otherwise Q1/Q4 are meaningless.
- MTR-INDEPENDENT work can proceed in parallel: B0.1 (done), the tracker-latent live arm
  (B0.2 restricted to the Mamba tracker), and the harness ablations that use only the tracker.

## 1. The six claims and verdict conditions

- Q1 Prediction gap is real and hurts driving. Verdict: cold destination shows a
  multi-frame post-handoff error spike vs a warm reference, and a collision/brake delta.
- Q2 The learned state closes it, not the last box or a Kalman snapshot. Verdict:
  full-latent << Kalman-snapshot ~= one-frame ~= cold in the post-handoff window.
- Q3 Trajectory trigger beats radio/distance proxies. Verdict: lower late-migration and
  wrong-destination at equal or lower false-migration rate.
- Q4 Predictive latent migration improves closed-loop safety end to end. Verdict:
  ours < generic-snapshot < cold on collisions/km, no worse on false brakes, low fallback rate.
- Q5 It scales with density. Verdict: prepare+import stays within the 300 ms budget up to
  the target concurrent-crossing count; payload grows linearly in crossing tracks.
- Q6 It degrades safely under impairment. Verdict: no silent wrong forecasts; bounded
  safety loss; fallback fires when state is late/stale/mismatched.

## 2. Build backlog (engineering work to produce the results)

Ordered by dependency. Each item names the files to touch.

- B0 LIVE STATE TRANSFER (linchpin). Make the live evaluation edge migrate real tracker state.
  - B0.1 MECHANISM + KALMAN ARM [DONE 2026-06-30]. Fixed the `import numpy` bug in
    `edge_manager_pluggable_base.py`. Gave the live `PredictionLateFusionEdge` real
    `export_vehicle_state`/`import_vehicle_state` (AB3DMOT KFState snapshot: mean, covariance,
    hits>=min_hits, velocity). Unit-verified round-trip under opencda310 (scratchpad
    `test_kf_migration.py`): destination KF resumes warm, mean/cov/hits/velocity match source,
    payload ~1.0 KB. This is the Reactive-Kalman baseline arm and proves export->serialize->
    import->warm end to end. NOTE: on this edge the predictor is LINEAR, so KF state is
    sufficient here; the learned-latent advantage requires B0.2.
  - B0.2 LEARNED TRACKER ARM (MTR-independent) [DONE 2026-07-05, unit level]. Mamba3DMOT is now
    a registered tracker (`get_tracker('mamba3dmot', cfg)`, lazy torch import); the wrapper carries
    carla_id through tracking (nearest-det association, 2 m gate) and exposes `.tracker`;
    `_PluggableEdgeBase` export/import now DISPATCHES on backend: Mamba -> full latent via
    `factories.latent_from_tracklet`/`inject_latent_into_tracker`, AB3DMOT -> KFState snapshot
    (also fixes the pre-existing wrapper-indirection bug: `self.tracker.trackers` never existed
    on the wrapper). Schema-mismatch records fall back to cold start (contract behavior).
    Verified under opencda310 (scratchpad `test_mamba_edge_migration.py`): both branches pass;
    Mamba banks byte-identical post-import, track_id preserved, ~1.3 KB payload. REMAINING for
    B0.2-live: a pluggable-edge YAML with `tracker: mamba3dmot` + weights path, exercised in the
    two-edge scenario (part of B0.3).
  - B0.2b MTR PREDICTOR-CONTEXT ARM [BLOCKED on P0]. Add MTR predictor-context migration once
    MTR is trained+validated on WorldFusion (P0). Until then do not report any live number that
    runs MTR as the predictor.
  - B0.3 INTEGRATION [DONE 2026-07-05, mechanism level]. New scenario
    `openscenario_3_multi_edge_mamba` (SOTA pluggable edges, tracker: mamba3dmot, late-fusion
    backend, linear predictor, visualize off for headless). LIVE RESULT: at HANDOFF_TICK=60 the
    full learned latent (10-frame memo bank, 776 B) exported from edge0, transferred, and
    injected warm at edge1 (`_import_track_latent: carla_id=333 -> mamba tid=1 (memo=10
    frames)`); injected tracklet survived subsequent ticks; zero post-handoff exceptions.
    Two live bugs fixed to get here: (1) `late_fusion_backend.detect` KeyError'd on a
    freshly-migrated VM whose beacon wasn't in pre-handoff frames (now tolerated); (2) live
    identities are rotating BSM temp ids, so the pluggable export now resolves tracklet ids
    through `BeaconIdManager.get_carla_id_for_temp` (same reverse map anchoring uses).
    REMAINING for full B0.3: warm-vs-cold prediction-error DELTA needs B4 metrics (persisted
    per-frame post-handoff error), and `SOTAEdge` lacks `evaluate()` (NotImplementedError at
    cleanup, non-fatal). Run recipe: CarlaUE4 -RenderOffScreen; then under opencda310
    `python ecav.py -t openscenario_3_multi_edge_mamba --apply_ml`.
  - Acceptance: destination resumes warm; a warm-vs-cold prediction-error delta is
    observable in one live boundary crossing (needs B0.2 for the learned-latent arm).
- B1 REAL TRIGGER + LOCALES. Wire `Locale`/`LocaleRegistry`/`VehicleLocaleTracker`
  (with hysteresis) into the live loop; feed locale polygons from YAML. Implement the
  trajectory-crossing trigger (predicted path intersects boundary within horizon) and,
  for the Q3 ablation, distance-to-boundary and radio-proxy triggers behind a selector.
  - Files: `binding.py` (replace tick trigger), `locale.py`, `registry.py`, YAML (add
    `locales:` polygons), scenario runner.
- B2 APPLY LINK COST AS DELAY / AoI. Make `TransferCost.total_ms` gate destination
  readiness (the migrated state is not usable until transfer completes in sim time), so
  backhaul latency and jitter change the outcome. Add knobs: base latency, jitter, packet
  loss, bandwidth. Reuse the ns-3 / SEE-V2X latency path used elsewhere for realism.
  - Files: `link.py`, scenario loop, latency model wiring.
- B3 TWO-PHASE + FALLBACK. Prepare/commit/abort with the source authoritative until
  commit (soft handoff), the not-yet-warm label, and the explicit-history/planner-default
  fallback. Per architecture `contents/architecture.tex` (import rule, failure model).
  - Files: `daemon.py`, `binding.py`, edge import path, planner fallback hook.
- B4 MIGRATION-AWARE METRICS. Persist per-handoff records to CSV/JSON: warm-vs-cold
  post-handoff ADE/FDE by frame index, payload bytes, prepare/transfer/commit timing,
  fallback rate, trigger outcome (early/late/false/wrong-dest), lead time. Add a
  warm-reference run mode (no handoff) so warm-vs-cold is a paired comparison.
  - Files: new `migration/metrics.py`, hook into `EvaluationManager` output_dir, scenario runner.
- B5 BOUNDARY SCENARIOS. Author the NHTSA set with an embedded locale boundary so the
  safety-critical actor crosses just before the conflict: LTAP-OD, SCP, LVD, LVS,
  pedestrian, lane-merge, plus a dense multi-boundary route. Extend the existing
  `openscenario_3_edge_late_fusion_boundary` fixture.
  - Files: `scenario_testing/scenarios/`, config_yaml, scenario runners.
- B6 SWEEP HARNESS. A `test_runner`-style driver over the knob matrix (payload x trigger x
  link x scene x fault x scenario), N>=30 seeds, that collects the B4 metrics and emits the
  paper tables/figures. Model on `scripts/verify_sweep_run.py` and the existing latency sweep.

## 3. Experiment specifications

### Q1 Does the gap exist (live + trace replay)
- Setup: B0 + B4 + B5. Boundary scenario, learned stack. Force one crossing. Run two arms:
  warm reference (no handoff, single edge owns throughout) vs cold destination (handoff,
  no migration).
- Also: open-loop trace replay on DAIR-V2X / V2X-Seq for a sensor-independent gap curve.
- Metrics: post-handoff ADE/FDE by frame 1..20, minADE/minFDE, miss rate, ID switches,
  time-to-recover; collision/km and brake/min delta warm vs cold.
- Depends: B0, B4, B5.

### Q2 Which state matters (payload ablation)
- Already measured (harness) for full-latent / one-frame / cold. Extend the ablation arms:
  none, latest-box, real Kalman mean+cov snapshot, N-frame history, tracker-latent-only,
  predictor-context-only, tracker+predictor, oracle.
- Two venues: keep the offline harness for the clean mechanism curve; add the live-edge
  version once B0 lands so Q2 holds in closed loop too.
- Metric: post-handoff FDE per arm. Verdict as in section 1.
- Depends: harness (done) + B0 for live confirmation. Add the real-KF arm to `factories`.

### Q3 Trigger ablation
- Setup: B1 + B4. Fix payload = full-latent. Sweep trigger: radio-proxy, distance,
  trajectory-crossing, probabilistic-trajectory, oracle.
- Metrics: early/late migration rate, false-migration rate, wrong-destination rate,
  lead-time distribution, stale-state expiry rate.
- Depends: B0, B1, B4.

### Q4 Closed-loop safety
- Setup: B0-B5. Boundary scenarios. Fix trigger = probabilistic-trajectory. Compare
  cold / generic-snapshot / full-latent (ours) / oracle.
- Metrics: collisions/km, min TTC, hard-brake/min, false-brake and stall rate, planner
  fallback rate, prediction error at use time.
- Depends: B0-B5.

### Q5 Scale
- Setup: B2 + B6. Sweep active agents 4/8/16/32 and concurrent boundary crossings/min.
- Metrics: serialize/deserialize time, backhaul bandwidth spike, migrated tracks/s,
  payload bytes, GPU sync overhead, destination memory; check against 300 ms budget.
- Depends: B0, B2, B4, B6.

### Q6 Impairment
- Setup: B2 + B3 + B6. Sweep backhaul latency/jitter/loss, clock skew, pose/frame error,
  schema mismatch, delayed/aborted/ping-pong commit.
- Metrics: fallback rate and planner cost when degraded; confirm no silent wrong forecasts.
- Depends: B0, B2, B3, B4, B6.

## 4. Knob matrix

| Axis | Values | Used by |
|------|--------|---------|
| Payload | none, latest-box, Kalman mean+cov, N-frame history, tracker-latent, predictor-context, both, oracle | Q2 |
| Trigger | radio-proxy, distance-to-boundary, trajectory-crossing, probabilistic-trajectory, oracle | Q3 |
| Link | base latency, jitter std, packet loss, bandwidth | Q5, Q6 |
| Scene | active agents {4,8,16,32}; boundary crossings/min | Q5 |
| Fault | clock skew, pose/frame error, schema mismatch, delayed/aborted/ping-pong commit | Q6 |
| Scenario | LTAP-OD, SCP, LVD, LVS, pedestrian, lane-merge, dense route (each with boundary) | Q1, Q4 |
| Stack (control) | Mamba+MTR (system), AB3DMOT+linear (stateless control) | Q1, Q2 |

Discipline: hold everything fixed and vary one axis, per the paper's "one design choice per
neighbor" scaffold. The shared substrate is the multi-edge boundary scenario + B4 metrics.

## 5. Metrics and logging

Persist one row per handoff and one row per frame post-handoff:
- Prediction: ADE, FDE, minADE, minFDE, miss rate, NLL (if modes), ID switches, frame index.
- Planner: collision flag (use `simulation_metrics.json: focal_collisions`, NOT the dead
  `conflict_kinematics collision_flag`), min TTC, brake events, false brake, fallback flag.
- Migration: payload bytes, prepare/transfer/commit timestamps, lead time, trigger outcome
  class, fallback reason.
Write under `opt.output_dir` alongside `EvaluationManager` output; a new `migration/metrics.py`
owns the schema. Warm-reference mode (no handoff) gives the paired baseline.

## 6. Sequencing / milestones

- M1 (mechanism, mostly done): harness B/B1/B0 microbenchmark. DONE. Add real-KF arm to
  the harness for the honest Q2 baseline.
- M2 (live warm handoff): B0. First live warm-vs-cold delta on one boundary crossing. This
  is the critical path; nothing else live matters until state actually migrates.
- M3 (metrics): B4. Persisted warm-vs-cold prediction error + handoff timing.
- M4 (Q1): run boundary scenario, produce the live gap curve. First live paper result.
- M5 (triggers): B1, then Q3.
- M6 (safety): B5 boundary set + B3 fallback, then Q4. Headline safety result.
- M7 (realism): B2 link delay, then Q6 impairment.
- M8 (scale): B6 sweep, then Q5.

All of B0-B6 and all of Q1-Q6 are REQUIRED for the paper. Nothing here is optional or
"minimum viable." The milestone order is a build dependency order, not a priority cut. A
strong NSDI paper needs every claim proven: the live gap (Q1), the payload ablation with a
real Kalman arm (Q2), the trigger ablation (Q3), the closed-loop safety delta (Q4), the
density scaling within budget (Q5), and the impairment/fallback behavior (Q6). Ship the full
suite. The sequencing exists only so each experiment runs on the machinery the prior build
step produced.

## 7. Risks and honest notes

- All six questions ship. There is no reduced-scope fallback. The sequencing below is a
  dependency order for building the machinery, not a ranking of which results to keep.
- The single biggest risk is M2/B0. Until the live edge runs the learned stack AND actually
  injects migrated state, there is no live gap to measure and no live claim in the paper is
  backed. Do B0 first and prove one crossing before building sweeps.
- The live edge today is AB3DMOT+linear (stateless predictor); a gap will not appear on that
  stack. Do not "measure" Q1 on it and conclude no gap. The learned stack is the object.
- Keep the paper honest: the microbenchmark is synthetic-trajectory and offline. It stays the
  mechanism result. Q1/Q4 must come from the live closed loop or trace replay, not the harness.
  Do not present harness numbers as closed-loop safety.
- Reuse the measured ns-3 / SEE-V2X latency path for B2 so backhaul is measured, not an
  injected constant (matches the safety-envelope methodology).
- Collision signal: use `focal_collisions`, the `conflict_kinematics collision_flag` is dead.
