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

Order: T1 (tonight, safe) and T2 (the gate) first; T3/T4 build while T2's rerun goes. T5/T6 belong to the parallel writing session; do not touch ~/repos/scale_out_nsdi. Commit style: one subject
line, no co-author trailers (user rule). Update
docs/kb/wiki/current_state.md after each block.
