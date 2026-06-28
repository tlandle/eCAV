# Plan: Full safety-envelope matrix at paper density (all architectures)

## FRAMING (governing): "How much cooperative perception is USABLE?"

Not "we found self-ghosting." The paper answers: how much cooperative perception
is actually usable by the planner, not just accurate or low-latency. (Reviewer B
called this the exciting question; Reviewer A pushed the generalization side.)

Usable CP iff ALL three gates pass:
  (1) info-value:  d_coop - eps(Delta_use) > d_los          (edge sees early enough)
  (2) physics:     M(t,Delta_use) >= 0                       (planner can still act)
        M = d_e - [v_e(rho+Delta_use) + v_e^2/2a_b + d_buf]
  (3) consistency: identity/freshness contract holds         (no logic failure)

Four boundaries: physics-usability (AoI vs speed/brake), information-value
(Delta* = (d_coop-d_los)/v_c), logic (multi-source self-ghost before physics
limit), architecture (early-fusion/I2V/MEC/object-world-model differ).

MAIN PLOT = usable-region, not just success curve:
  x = p99(Delta_use); y = P(S_op) or "usable CP fraction"; overlay analytic M=0
  boundary; curves Oracle / CIP / 2-RSU infra / LF / LF+SBA; MARK local-only.
  CP gain = S_op(edge admitted) - S_op(local only). Usable region = CP gain > 0
  AND identity/freshness gates pass.

REQUIRES a LOCAL-ONLY baseline arm (vehicle uses only its own perception via the
already-wired VehicleSideTracker; edge publishes nothing to the planner). This is
the CP-gain reference. NOT yet configured -> add to the matrix.

LATENCY MODEL SPLIT (do not change two variables at once):
- PRIMARY matrix (architecture discriminator, physics envelope, SBA, failure
  decomposition): CONTROLLED/deterministic latency = the single independent
  variable. ns-3 LUT OFF (it is off by default, cfg use_ns3_lut=False).
- SECONDARY validation only: ns-3 Uu LUT (payload/N-aware) scatter for a small
  subset (LF, LF+SBA, maybe CIP/2-RSU) at N in {1,4,8,16}, plotted on the SAME
  measured-Delta_use envelope to show realistic radio maps to the same coord.
- RULE: every architecture-comparison figure uses ONE latency model; every
  latency-model figure fixes architecture. Never mix models within a comparison.

## CIP STATUS (off critical path, fix in parallel)

CIP runs (apply_control tuple bug FIXED: now converts via vm.controller; DL
command delay FIXED: delivered after controlled-latency/ns-3-DL delay, never
zero). BUT the vehicle stalls ~17m in (avg_speed 0.08 m/s, distance 0.6m,
total_brake_events 0, 234 collisions). Root cause: CIP's `_advance_actors`
override reimplements the advance loop INCOMPLETELY. It calls
`vm.agent.update_information(ego_pos, ego_spd, {})` directly with empty objects,
bypassing `vm.update_info(tick)` which does map_manager.update_information,
perception merge, and local-planner waypoint-buffer population. Without the
map/route updated each tick the planner can't generate a forward trajectory, so
the vehicle never drives. Contrast: working infra_only uses the PARENT
`_advance_actors` (vm.update_info + vm.run_step = full vehicle pipeline) and the
ego drives (avg_speed 2.3, distance 20m, brakes 24).

CIP fix (scoped): the override must run the full vehicle pipeline (localize +
map update + planner) with edge-sourced obstacles, then ship the resulting
control over the DL delay, rather than calling agent.update_information with an
empty dict. Likely: call vm.update_info(tick) (or the parts it needs) so the
map/route is current, feed edge predictions as the obstacle set, then run_step +
controller + DL-delayed apply. NOT on the 26h critical path; 5 clean arms launched.

## Context

The SenSys resubmission needs clean, paper-grade envelope data across ALL
architecture arms, matching the quality of the existing reproduction sweep
(`ecav/scenario_testing/evaluation_outputs/20260311_230618`: 20 cells x 5 reps =
100 runs, latency 0-500/50ms). The existing sweep covers late_fusion / oracle /
vips x anchoring on/off. The NEW arms for the architecture-disambiguation result
(answering the MobiCom "is self-ghosting a V2X2V artifact?" critique) are
infra_only (1-RSU and 2-RSU I2V) and cip. We run the full matrix at the
calibrated 40 km/h operating point with the conflict-kinematics logger on, and
verify continuously so a 26h job is not discovered to be garbage at the end.

Clean-data discipline (why the existing plots look smooth, not binary):
- Metrics come through `scripts/recompute_metrics.compute_run_metrics`: brake
  EPISODES (contiguous same-track ticks collapse to one), not raw tick counts.
- S_op is plotted as BINNED MEAN over many runs vs MEASURED p99(Delta_use)
  (40ms bins), giving a smooth 0->1 probability envelope. Requires ~5 reps/cell.
- Continuous metrics: ego_uniqueness_violation_tick_fraction, brake_episodes/km,
  prediction_fde, AoI CDF/percentiles. Never raw total_ghost_brake_gt.
- Plotter: `scripts/paper1_real_aligned_plots.py <sweep_dirs...>`.

## Matrix

Operating point: scenario_3 LTAP, ego 40 km/h (calibrated; brake fires at margin
zero-crossing), cross-traffic Tesla 13.4 m/s, conflict (-84.8,127.7).

| Arm | manager | anchoring | RSUs | configs |
|-----|---------|-----------|------|---------|
| LF        | late_fusion | on, off | 2 | openscenario_3_edge_late_fusion_smoke |
| Oracle    | oracle      | n/a     | 2 | (--manager-types oracle on LF config) |
| VIPS      | vips_temporal | on, off | 2 | (--manager-types vips_temporal) |
| I2V-1RSU  | infra_only  | n/a     | 1 | openscenario_3_edge_infra_only_1rsu_smoke |
| I2V-2RSU  | infra_only  | n/a     | 2 | openscenario_3_edge_infra_only_smoke |
| CIP       | cip         | n/a     | 2 | openscenario_3_edge_cip_smoke |

Latencies: 0,50,100,150,200,250,300,350,400,450,500 ms (11, matches repro).
Reps: 5 per cell.

**CRITICAL latency-model correction (2026-05-31):** the full matrix MUST run with
the SEE-V2X trace driving the hybrid model:
  test_runner ... --see-v2x-trace data/see_v2x/merged_latency.csv
Without it, fixed latency + jitter_std=0 collapses AoI-at-use to a single tick
value (stairstep), e.g. lat200 -> aoi hist [34@4t, 179@5t, 2@6t], p50=p95=p99=5t.
The smooth AoI CDF / binned-mean envelope in the paper figures comes from
per-packet sampling of the real C-V2X RTT trace (latency_ms median 14.3, p5 6.4,
p95 23.1) + backhaul lognormal. --latencies then sets the hybrid base_ms offset;
the trace adds realistic jitter on top. This is both the paper's methodology and
the correct one (measured network behavior, not an injected constant).
The shakedown ran on fixed latency (fine, it only validates arms RUN); the FULL
matrix switches to the trace.
Arms x anchoring: LF{on,off}, VIPS{on,off}, Oracle, I2V-1RSU, I2V-2RSU, CIP = 8.
Total: ~8 x 11 x 5 = ~440 runs, ~26h sequential at ~3.5min/run.

NOTE infra_only/cip force enable_sba=False (no beacons), so they have no
anchoring axis. Oracle has no identity ambiguity by construction.

## Sequence (verify before committing 26h)

1. **Config prep [DONE]:** infra_only 1/2-RSU and cip configs set to 40 km/h with
   conflict_kinematics block; cip config+runner created; conflict logger
   auto-uniquifies output path; eval output tags read scenario_name from YAML.

2. **Shakedown (~2h):** 1 rep x {0, 200, 400ms} x all 6 manager arms (~30 runs).
   Run `scripts/verify_sweep_run.py` after; confirm every arm: recall>0,
   AoI p99 tracks latency, episodes sane, ego-uniqueness in [0,1]. ABORT and fix
   if any arm produces garbage (e.g. infra_only recall 0, cip no actuation).

3. **Full matrix (~26h):** only after shakedown is clean. Run each arm as its own
   test_runner invocation (latencies 0-500/50, reps 5). Stagger so CARLA restarts
   between arms. Continuous verification via Monitor on the live sweep dir.

4. **Plot:** `paper1_real_aligned_plots.py` over all sweep dirs + the existing
   repro sweep (merge so all curves are equally smooth). Produces sop_vs_aoi,
   ego_uniqueness, brake_episodes_km, envelope_boundary, aoi_cdf.

## Continuous verification (the "data behaves as expected" requirement)

`scripts/verify_sweep_run.py <sweep_dir>` flags per run:
- run incomplete (no edges{}), recall=0 (cross-traffic missed),
- measured AoI p99 not tracking injected latency,
- ghost episodes exploding (>30), ego-uniqueness fraction out of [0,1].
Tested on existing data: recall ~0.6, AoI p99 rises 1.4->3->5->8->9t with lat
0->400, euniq ~0.26-0.32. Behaves correctly.
Run it via Monitor against the live sweep dir during the full matrix.

## Expected results (to check against, per arm)

- Oracle: high S_op until physics limit (~400-500ms measured AoI), zero
  ego-uniqueness violations (single source).
- I2V-1RSU: low ego-uniqueness (single infra source), envelope near Oracle.
- I2V-2RSU: ego-uniqueness violations WITHOUT vehicle uplink (the key result);
  envelope degrades below Oracle.
- LF/VIPS off: sharp cliff ~150-200ms measured AoI; ego-uniqueness grows.
- LF/VIPS on (SBA): tracks Oracle curve; envelope expands logical->physical.
- CIP: analyze at the plan/command interface; same publish-boundary identity
  exposure as infra fusion.

## Out of scope
- VRF (related-work contrast only, not a run baseline).
- Cascading/string failures (single-focal-ego envelope only).
- Speeds other than 40 km/h for the full matrix; a SEPARATE compact Oracle
  speed sweep (30/40/50) covers the physics-boundary-moves-with-speed figure.
