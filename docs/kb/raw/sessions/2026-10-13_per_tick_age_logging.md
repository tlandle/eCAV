# Session 2026-10-13: per-tick age-at-use logging (branch sensys/per-tick-age-logging)

Branch `sensys/per-tick-age-logging` off develop 3fe4e0da, worktree `.claude/worktrees/agent-a7c8f014855cd9bef`.

## Touched

- `ecav/core/application/edge/per_tick_age_logger.py` (new): one CSV row per tick per consumed prediction per focal vehicle; provenance, age at use, publish sequence, conflict-actor flag, planner decision.
- `ecav/core/application/edge/edge_manager/edge_manager_base.py`: constructs the logger from the `per_tick_age_log` edge block, calls it from `_record_time_to_events`, numbers edge publishes through a `lin_pred` property setter.
- `ecav/scenario_testing/config_yaml/openscenario_3_edge_late_fusion_smoke.yaml`: knob block, off.
- `test_runner.py`: `--per-tick-age-log`.
- `safety_envelope_paper/reduce_per_tick_age.py` (new, gitignored dir, added with -f): per-run and per-group reduction with bootstrap CIs.
- Paper repo `reviews/ns3_payload_and_table2.md` (ba5515f, not pushed).

## Commits

c6002ae3, d44b1bd6, cea65cce, 90c77656; KB commit follows.

## Decisions

- Hook at `_record_time_to_events` so late-fusion, oracle and VIPS log without subclass edits.
- Publish numbering via a property setter, because every SenSys manager replaces `self.lin_pred` after the base constructor (found on the first live smoke, where publish_seq was blank).
- Conflict actor picked by the ConflictKinematicsLogger rule; anonymous tracks within match_radius_m count as the actor.
- Smoke runs serialised on the shared CARLA (port 2000) by message with the peer agent after two collided launches.

## Results

See the 2026-10-13 section of `docs/kb/wiki/current_state.md`.
