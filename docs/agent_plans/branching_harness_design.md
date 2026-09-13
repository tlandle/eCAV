# Branching-Junction Harness Design (freeze-1p follow-on, NSDI experiment 1 of 4)

Status: DESIGN FOR PEER REVIEW before any build. 2026-09-12.

## Purpose

Feeds the paper's two branching claims: readiness against destination certainty,
and speculative placement cost against junction degree, for six placement
policies. Also carries the predictor destination-confidence measurement (peer):
the distribution of MTR destination confidence on branching trajectories and a
calibration check, so the paper can shade the operating range of the deployed
predictor onto the synthetic sweep in Figure 8.

## Scenario

A junction whose outgoing roads define k branches, one locale per branch plus
one approach locale. A scripted road user approaches, commits to a branch late
(its route is fixed per seed but not inferable from geometry until the turn
begins), and meets a downstream conflict shortly after entry into whichever
branch it takes. The conflict is what makes readiness measurable: state must be
ready at the taken branch's locale before the recipient CAV's first use.

Junction degree k in {2, 3, 4, 6}. Map survey is build step 1:

- k=2: any fork / T-junction approached on the stem (Town01/02 have many).
- k=3: crossroads approached on one arm (Town01/03/05).
- k=4: full crossroads with U-turn permitted, or a 5-arm junction minus one
  (Town03/05).
- k=6: RISK. The largest physical junction in the available maps
  (Town01-07,10) is Town03's roundabout (~5 arms). If the survey confirms
  nothing reaches 6, the honest options are (a) sweep {2,3,4,5} and say so, or
  (b) count the roundabout's bidirectional arms as distinct destinations if
  the road network genuinely permits both. We will NOT fabricate a degree by
  partitioning locales finer than the physical branches, because destination
  uncertainty must correspond to physically distinct branches. Decision goes
  to you after the survey, before the scenario is built.

## Arms (six; placement decision is the ONLY thing that varies)

| Arm | Mechanism | Exists today? |
|---|---|---|
| Khonsu | computed trigger consuming multimodal MTR destination probabilities (per-mode boundary crossing, probability summed by destination; `last_mtr_modes` stash) | TRIGGER_MODE=computed + mtr consumption path exists |
| Oracle | knows the taken branch at trigger time | TRIGGER_MODE=oracle exists |
| Velocity projection | CV extrapolation picks the single projected branch | TRIGGER_MODE=predictive exists |
| Top-1 mode only | argmax MTR mode's branch, ignore the rest of the distribution | small variant of the mtr trigger (new flag) |
| EdgeWarp radio trigger | radio-level trigger, destination = the cell being entered | MIGRATION_MODE=edgewarp_full exists |
| Replicate-to-all | on trigger, send the identical record to ALL k branch locales | NEW arm (the brute-force comparison) |

Replicate-to-all build: a MIGRATION_MODE (or TRIGGER_MODE) variant that, at the
same trigger point as Khonsu's computed trigger, issues the same prepare to
every adjacent branch locale instead of the selected subset. Cost surfaces as
imports at k destinations; readiness is trivially met at the taken one. This is
deliberately unoptimised in DECISION but not in transport: it uses the same
migration path, serialization, and payload as every other arm.

## Fixed transferred state (the identity requirement)

The payload is the FULL record (warm) for all six arms. The arms differ only in
when/where prepares are issued, never in what is sent.

## What would make the comparison meaningless (invalidation conditions)

1. Non-identical transferred state across arms. Guard: HANDOFFROW already
   carries bytes=; add a payload content hash to the emit. Per (seed, handoff),
   the hash must be IDENTICAL across all six arms or the run set is invalid.
   Checked by the lander, not by eye.
2. Non-identical scenes. Guard: the branching actor's route (including which
   branch it takes) is scripted per seed and identical across arms; paired
   spawn-phase seeds as in the flow scenario. Verify per (seed): the actor's
   trajectory polyline in the log matches across arms within CARLA jitter
   (existing host-effect method, dist/time distributions). An arm whose
   presence changes the actor's behaviour (e.g. replicate-to-all import load
   slowing the sim and shifting spawn phase) would silently break this; the
   trajectory check catches exactly that.
3. Readiness measured against different conflict timelines. Guard: readiness
   and lead time are computed against the taken branch's conflict tick from
   that run's own log (per-run realised, never nominal), same rule as the
   T_avail lander.
4. Destination confidence measured from a different model than the policies
   consume. Guard: CONFROW is emitted inside the same predictor instance the
   trigger reads (`last_mtr_modes`), not a side model.

## Metrics and logging (all lander-parsed, no by-eye)

- READYROW per handoff: taken branch, prepare tick per destination, commit
  tick, first-use tick at the taken destination, ready-before-first-use bool,
  lead time s. Feeds readiness vs certainty.
- PLACEROW per handoff: k, set of locales prepared, count (speculative
  placement cost). Feeds placement vs junction degree. Replicate-to-all
  gives count=k by construction; Khonsu gives the probability-selected subset.
- CONFROW at T-minus {2.0, 1.0, 0.5} s before branch entry: per-branch
  probability (mode scores summed by destination), argmax branch, taken
  branch. Feeds the distribution + calibration (fraction correct at each
  confidence bin). Also answers where on the 0.35-0.95 synthetic axis MTR
  actually operates.
- ISOROW/MTRTIME stay on (same predictor build as the campaign).

## Grid and cost

k in {2,3,4,6 or survey-max} x 6 arms x 10 seeds = 240 routes (if 4 degrees).
At ~4-7 min/route: ~16-28 h single host. Runs AFTER the current campaign
completes; whichever host frees first. Smoke first: one k=3 route, Khonsu +
replicate-to-all only, verifying READYROW/PLACEROW/CONFROW parse and the
payload-hash identity across the two arms, before the full grid.

## Build order

1. Map survey (junction candidates per k; the k=6 answer). Report to peer.
2. Scenario files (yaml/xml/py) for the chosen junctions; scripted late-commit
   actor; conflict actor per branch.
3. Replicate-to-all arm + top-1 flag + payload hash in HANDOFFROW.
4. CONFROW emit in the predictor.
5. Smoke (step 5 gate: peer reviews smoke output before the grid).
6. Full grid + lander + report (distribution and calibration reported plainly,
   including if confidences cluster >=0.9 making the sweep a stress region).
