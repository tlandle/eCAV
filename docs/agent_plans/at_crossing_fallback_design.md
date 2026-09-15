# At-crossing fallback for the Khonsu arm (design, 2026-09-14)

Tyler's ruling on the never-confident track: build an at-crossing fallback
first, measure how often it fires, then decide on a deadline guess.

## 1. Fallback (both runners: code_overlay/runner_wired.py flow family;
scripts/branching/runner_branch.py)

For MIGRATION_MODE=warm under TRIGGER_MODE=mtr (and computed, which can also
fail to fire): when the road user enters a candidate locale and no PREPARE has
fired for that track into that locale, fire the reactive rule for it: the full
record to the entered locale with COMMIT, imported on arrival, publishable
after the ack, same record and transport as every other arm (the existing
reactive branch: fire on `locale_by_id[entered].contains(nxy)`). Khonsu is then
never worse than history at crossing. Marker, one line per fallback:
  [FALLBACKROW] tick=%d npc=%d entered=%s reason=%s argmax=%s p_argmax=%.3f
                lead_opened=%d
reason in {no_candidate_at_theta, lead_never_opened, trigger_never_fired};
argmax and its probability from the last mtr sample before entry (what a
guess would have done). Readiness and the [HANDOFFROW] bookkeeping treat it
as a handoff prepared at lead zero (prepare_tick = crossing tick). Smoke: the
two-locale flow scenario and the k3 site, once the predictor fix lets the mtr
arm fire (gated on 1q).

## 2. Count from the landed logs (scripts/khonsu_unprepared_crossings.py)

For every Khonsu-arm crossing in the landed blocks on this host (flow family;
the corridor's logs are on cetus, its landed CSVs give preparations and
completed crossings only): crossings with no prepare into the entered locale,
by reason. The count sets the rerun scope (the Khonsu arm of every closed-loop
block with an unprepared crossing, whole blocks) and decides rule 3. Result
recorded below when the extractor lands.

## 3. Deadline guess (only if 2 shows the case is not rare)

Env knob DEADLINE_GUESS=1: at the tick the lead window closes (time to the
projected exit at or below L) with no candidate at or above theta, fire the
argmax candidate alone; at most one prepared destination still holds; the
fallback of 1 stays underneath it.

## Invalidation conditions

- A fallback that fires while a PREPARE for the same track and locale exists
  is a bookkeeping error (the guard is `npc not in npc_handoff_done` for that
  destination); the smoke fails on any such line.
- A fallback whose import is not visible to the destination within the same
  tick (the reactive path is synchronous) is a transport error; smoke fails.
- If the fallback fires on every crossing under the mtr trigger after the 1q
  fix, the trigger is not firing at all (mode collapse not fixed, or theta too
  high); the block stops and the trigger is checked before any rerun.

## Queue

Behind the state block and the Figure 5 sweep unless the count says the
corridor numbers move, in which case first. Paper text claims nothing until
the reruns land.

## Built (2026-09-14, unsmoked; gated on 1q)

- Flow family, code_overlay/runner_wired.py (and the scratchpad copy the
  runners copy from): FALLBACK_AT_CROSSING (default on for warm) fires the
  normal transfer from the previous locale's edge into the entered locale when
  the actor's locale changes with no handoff done; bookkeeping
  npc_handoff_done = (tick, entered), so [HANDOFFROW] carries prepare_tick =
  crossing tick and the final update follows at commit as for every warm
  handoff; [FALLBACKROW] tick, npc, entered, from, reason
  (no_forecast | no_candidate_at_theta | lead_never_opened under mtr;
  trigger_never_fired otherwise), trigger, argmax, p_argmax, last_eval_tick,
  spawn_dist_m, bytes; the [SCENB] PREDICTIVE OBSTACLE HANDOFF line is emitted
  with fallback=1 appended so every lander keeps working. The mtr branch records
  the last (tick, p_dst, dst) per track; DEADLINE_GUESS (default off) fires when
  the lead window is closing below theta and logs [GUESSROW].
- Branching harness, scripts/branching/runner_branch.py: the fallback rides
  the existing fire loop (rule 'fallback') when the actor is inside a
  candidate's polygon (the mouth's overlap band) with no handoff done, so
  [FIREROW], prepare ticks and hashes apply uniformly; [FALLBACKROW] as above
  with taken=; a locale change that skipped the band logs [FALLBACK-MISS]
  (invalidation: any such line means the band is too thin at the run speed).
  The mtr branch records (tick, p_max, argmax, lead_ok) per track;
  DEADLINE_GUESS fires the argmax with rule 'guess' and [GUESSROW].
- Both runners append dist_to_boundary_m to the "[SCENB] NPC ... resolved"
  line (distance at resolution to the nearest other locale), the paper side's
  spawn-near-boundary field; is_gating comes from the landers' gating rule.
- Counter scripts/khonsu_unprepared_crossings.py gains is_gating and the
  resolve-to-crossing interval; spawn distance from the marker once runs
  carry it.

Smoke (when 1q lets the mtr arm fire): one flow cell per trigger mode
(predictive, mtr) with the fallback forced by a late release, and one k3 cell;
pass = a [FALLBACKROW] per unprepared crossing, its [HANDOFFROW] with
prepare_tick = crossing tick, no [FALLBACK-MISS], no fallback while a PREPARE
for the same track and locale exists.

