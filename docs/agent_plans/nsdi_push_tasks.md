# NSDI push: task list (hand to a working session)

Context anchors, read first:
- `docs/kb/wiki/current_state.md` top three entries (provenance alert, design-sweep campaign, defense outcome).
- `docs/agent_plans/khonsu_design_studies.md` (sweep matrix, arm semantics).
- Memory: `project_khonsu_budget_contribution` (the reframed contribution),
  `project_flow_table_provenance` (DO NOT cite q4_flow_table.csv numbers).

Hard constraints:
- Sims run under conda env `opencda310` (`source ~/anaconda3/etc/profile.d/conda.sh && conda activate opencda310`), CARLA 0.9.15 local. The sweep script manages CARLA itself; never manage CARLA by hand while it runs.
- `pkill -f` self-matches your own command line; use bracketed patterns like `pkill -f 'khonsu_design_swee[p]'`.
- Runs are self-describing: every flow run ends with a `[RUNROW]` line; extract with `python scripts/khonsu_design_extract.py <logdir> -o rows.csv`. A log without an ego `actor_id` dict is a crashed run, RUNROW or not.
- Between-arm comparisons from the uniform sweep are valid; absolute collision rates are NOT paper-grade until the scenario grind (every arm 10/10 collided, buffer-capped 30 contact ticks, stalls at the truck) is diagnosed. Do not present absolute safety rates from flow runs.
- Writing rules: memory `feedback_writing_simply` (define terms at first use, no personification, no flourishes), `feedback_no_invented_terms`, `feedback_eval_ground_up_no_letter_arms` (named arms, never B0/B1), no em dashes.

## T1. Lookahead-matched warm arms (no new code, run first)
Run `MIGRATION_MODE=warm LOOKAHEAD_S={2,3,4}` x 10 reps on
`openscenario_1_flow_gt` by extending `scripts/khonsu_design_sweep.sh`
(add s1_warm_la{2,3,4} entries mirroring the existing pattern) and
relaunching with the same `LOGDIR=evaluation_outputs/khonsu_design_sweep_v1`
(resumable; done runs skip). Acceptance: 30 new rows in the extract with
transfers > 0; a lead-matched trigger table (forecast at 2/3/4 s vs bands
20/40 m vs at-crossing) added to
`docs/kb/data/relay_eval_2026_08/` with a KB note.

## T2. Scenario grind diagnosis (blocks all absolute numbers)
Symptom: every arm sustains one 30+-tick contact episode at the stopped
truck; median run times out at 50 s around x~67 m. Known-good behavior
existed on 2026-08-16 (KB "STALL ROOT FIXED" entry) but no longer
reproduces from any commit (bisection log: KB raw/sessions/2026-08-31.md).
Approach: single warm run with `BEHAVIOR_DEBUG=1`, trace whether the
stopped-lead car-following branch (behavior_agent.py, `_find_blocking_lead`)
engages on approach; if it does, trace the overtake commit gate. Compare
tick traces against the documented 08-16 fix semantics. Acceptance: named
root cause + minimal fix + 5 warm reps with 0-episode majority, THEN rerun
the full sweep (script is ready) for paper-grade absolute rates.

## T3. Computed trigger (the paper's mechanism claim)
Implement `TRIGGER_MODE=computed` in
`ecav/scenario_testing/openscenario_1_flow_gt.py`: fire when remaining
time-to-crossing <= predicted destination warm-up (backhaul transfer time
+ import + destination refresh-queue delay estimate) + margin. The
destination's refresh cadence is observable from the edge manager
(adaptive controller, deadline 130 ms). Baselines already exist (fixed
leads, bands). Acceptance: computed-trigger arm x 10 reps; lead-time
distribution logged per fire; comparison table vs fixed leads.

## T4. Platoon-burst scenario (feasible-region claim)
Variant of the flow scenario where N in {2,4,8} NPCs cross the boundary
within one 2 s window (edit `scenario_1_flow.xml` spawn spacing or add
`scenario_1_burst.xml`). Measure per-track transfer-to-first-refresh
latency at the destination (RUNROW extension or frames CSV) vs N, with
and without the existing risk-ordered admission. Acceptance: a
first-refresh-latency-vs-burst-size table; the crossing-load feasibility
boundary stated with numbers.

## T5. Paper updates — OWNED BY THE WRITING SESSION, skip (~/repos/scale_out_nsdi)
Port from the dissertation (~/repos/dissertation
contents/paper5-scaleout.tex, already written): the delivery/authority
overlap decomposition (6.1.1), the trigger-as-measured-axis subsection,
and the ownership-handshake reframe (one send + two-step authority;
"two-phase" only as EdgeWarp lineage). Update evaluation.tex arms to the
uniform-sweep names. Cite canvas_latency.csv for the locale-size cap.
Do NOT import q4 numbers; T1/T2 outputs replace them. Compile must stay
at the venue page budget; check before committing.

## T6. Figures — OWNED BY THE WRITING SESSION once T1/T2 data lands, skip
Regenerate the headline figures from the new rows.csv once T1/T2 land:
lead-time curve (exists: see scratchpad story deck pattern), trigger
comparison, burst feasibility. Style: matplotlib, NAVY/GOLD palette used
in the story deck, captions state the finding
(memory: figures must have findings).

## Review-driven additions (2026-09-03). Priority: T7, T8, T9 before T3/T4; T10, T11 after.
Source: external NSDI review. Deadlines: abstract Sep 10, full paper Sep 17.
The writing session is reframing the paper around these; do not edit ~/repos/scale_out_nsdi.

## T7. Ownership epochs + fencing + failure injection (protocol correctness)
The paper claims exactly one publisher per track, and its failure model admits
"briefly both claim a track". Make the invariant true. In
`ecav/core/application/edge/migration/payload.py` add `epoch: int` to
`MigrationPayload` (monotonic per track, source increments at PREPARE, commit
carries the epoch). Every published forecast carries `(actor_id, epoch)`;
consumers (the ego eval path in `openscenario_1_flow_gt.py`) accept only the
highest epoch seen and drop lower. Add `FAULT_MODE` knob: `lost_prepare`,
`lost_ack`, `lost_commit`, `dup_commit`, `reorder`, `dst_crash`, each injected
once per run at the transfer. Acceptance: 6 fault arms x 5 reps; a table of
(fault, transfers, stale-epoch drops, double-publish windows in ticks,
completion). Double-publish windows must be 0 with fencing on; report the
number without fencing as the baseline.

## T8. Real two-process transfer over gRPC + netem (replace the parametric link)
Today `MigrationPayload.serialize()` is pickle and transfer time is computed,
not sent. Build a minimal `migration.proto` (bytes payload + actor_id + epoch +
schema_version), a `MigrationRelay` gRPC servicer running in a second process
on Atlas, and have the flow scenario's PREPARE/COMMIT path send real bytes.
Backhaul delay/loss via `tc qdisc add dev lo root netem delay 5ms 20ms loss 1%`
(needs sudo; document the exact commands). Acceptance: p50/p95/p99 for
PREPARE, delta-at-commit, import, and COMMIT under netem {0, 5, 20, 50} ms
and loss {0, 1, 5}%; bytes on the wire per transfer; a `[RUNROW]` extension
with these fields. Keep `TRANSFER_MODE=parametric` as the fallback.

## T9. Post-handoff prediction metrics (minADE/minFDE), not only tracker displacement
From existing microbench and flow runs, compute post-handoff minADE@3s,
minFDE@5s, and miss rate (2 m) for the migrated tracks at the destination,
per arm (cold, kf, warm, warm+delta), for the first 5 refreshes after commit.
Prediction outputs are in the frames CSV / eval logs (see
`scripts/khonsu_design_extract.py` for paths). Acceptance: one table, arms x
metrics, with 95% CIs over seeds; CSV under `docs/kb/data/relay_eval_2026_08/`.

## T10. Cross-locale association for unconnected actors (staged; may not finish)
Today `stable_actor_id` is ground-truth injected. Implement an overlap-region
track-continuation step at import: match the migrated track (source token +
last bbox transformed into destination frame) against destination detections
in the sensing overlap using temporal gating (predicted position at import
time) + Mahalanobis distance on the KF covariance / bbox size; ambiguity
(two candidates inside the gate) forces the explicit cold fallback. Knob
`ASSOC_MODE={gt, gated}`; add pose-error injection `ASSOC_POSE_ERR_M` and a
distractor NPC spawned alongside the crossing actor. Acceptance: ID-switch
rate and false-association rate vs pose error {0, 0.5, 1, 2} m and distractor
{off, on}, connected ego vs unconnected actor separately, 10 reps each.

## T11. Trigger study on real trajectories (V2X-Seq), staged
Offline: take V2X-Seq trajectory sequences (dataset path: ask Tyler; not on
Atlas yet), draw synthetic locale boundaries every 250-300 m along the
corridor, run the constant-velocity and MTR forecast triggers with lead
{1, 2, 3, 4} s and bands {20, 40} m, and compute per trigger: crossing recall,
wrong-destination rate, false-prefetch rate, late-prefetch rate, lead-time
CDF, wasted bytes, and probability-threshold sensitivity. Acceptance: one
table + lead-time CDF figure data CSV. This replaces the CARLA-only trigger
comparison as the paper's trigger evidence if it finishes.

## Reframe additions (2026-09-04). The paper now asks: how does cooperative prediction scale from one locale (Conductor) to a metropolitan deployment? Four RQs: scale-out, state continuity, safe handoff, multi-locale operation.

## T12. Re-measure the safe-age envelope inside the Khonsu scenarios (RQ3 needs it; the envelope paper is unpublished)
For each closed-loop handoff scenario (blind overtake, acceleration, LTAP, SCP, stopped lead), inject a controlled forecast age at the planner (delay the consumed forecast by d ms, d in {100..600} step 50) with NO migration effects, and find the largest d at which the maneuver still completes without collision across seeds. That d is the scenario's safe-age limit tau(u). Acceptance: a table scenario x tau(u) with CIs, 5 seeds per point; the paper cites these instead of the 220-450 ms figure. Reuse the envelope harness knobs if they still exist (AOI_INJECT / forecast delay); otherwise add a consume-side delay knob in the ego eval path.

## T13. Multi-locale corridor (RQ4, the capstone; biggest new build)
Scenario: 3 to 5 consecutive locales along an arterial (A | B | C | D), each with its own canvas, RSU at its conflict anchor, and a service instance; adjacent sensing overlap sufficient for handoff. One ego drives the whole corridor. Actors cross boundaries independently, some accelerating or braking; conflicts at intersection centers AND traffic-generated between intersections (queue tail, stopped vehicle, overtake, merge); several tracks cross one boundary together; some migrations land on a destination already busy. Arms over the WHOLE route: cold, reactive/boundary, EdgeWarp timing, geometric overlap, continuous replication, Khonsu, oracle.
Metrics per route: continuity (fraction of handoffs warm before use, post-handoff ADE/FDE, track resets/ID switches, fallbacks per km); planner safety (age at use vs tau(u), min TTC, collisions, hard/false braking, route completion); system (migrations/s, concurrent migrations, bytes/s between locales, import and first-refresh latency, per-locale compute utilization, prepared-but-unpublished tracks, failure point vs crossing rate); repeated handoffs (metrics by crossing index 1st/2nd/3rd, stale epochs, ping-pong).
Build path: generalize the two-locale flow runner to N locales (registry already maps areas to locales), extend the RUNROW to per-handoff rows with crossing index and locale ids, and a corridor XML. Start with 3 locales and the existing ego route stitched; 5 reps per arm. Acceptance: one route-level table (arms x metrics) and one figure (metric vs crossing index). Priority: after v3 lands and T7-live; ahead of T10/T11 if time is short. Flag to Tyler if it cannot land by Sep 15.

## T13 concrete spec (writing session proposal, adapt as needed)
Map: the arterial used by the flow scenario. Four locales of ~300 m centered on consecutive intersections; boundaries at the midpoints between anchors (~150 m from each). Ego route ~1.2 km at the 12 m/s contract speed, three handoffs. Oncoming stream FLOW_N in {4, 8} (~2-4 crossings/min per boundary), connected share 50%. Conflicts: crossing path at anchors 1 and 3; queue tail on segment 1; stopped vehicle + overtake on segment 2; merge on segment 3; the 5-vehicle platoon crossing boundary 2 together. Success per run: ego completes without collision; age at use <= tau(u) at every planner decision; >= 95% of handoffs warm before first use; no stale-epoch forecast consumed. 5 seeds per arm. Descoping order as you proposed.

## T14. Overlap in three meanings, over overlap width (the professors' question)
Arms: sensing overlap only (both locales observe the band, state still migrates on the forecast trigger); compute overlap (destination starts processing the actor at overlap entry, geometric prefetch); dual authority (both locales publish; planner rule = highest epoch, record disagreements). Sweep overlap width {20, 40, 80, 120} m. Metrics: fraction of handoffs warm before first use, bytes per crossing, fraction of planner ticks with two disagreeing forecasts for one actor (dueling rate, disagreement threshold 0.5 m at 1 s), age at use, collisions. 5 seeds per cell on the two-locale scenario. Priority: after T12, before T10/T11.

## Metric rule (Tyler): report collided-or-not per run (fraction of runs with any ego contact) and completed-without-collision. Do not report mean collision episodes per run.

Order: T1 (tonight, safe) and T2 (the gate) first; T3/T4 build while T2's rerun goes. T5/T6 belong to the parallel writing session; do not touch ~/repos/scale_out_nsdi. Commit style: one subject
line, no co-author trailers (user rule). Update
docs/kb/wiki/current_state.md after each block.
