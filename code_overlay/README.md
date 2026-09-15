# code overlay (settled non-scenario code)

Version-neutral, so the directory states its own build rather than implying it by
name (it was `freeze_1m_overlay`; renamed because it now holds a later build).

**Current build: freeze-1n.** Corresponds to run tag `khonsu-eval-freeze-1n` and
measured CSVs `frozen1n_*`. The one code difference from freeze-1m is the
ghost-filter fix (location-keyed removal of static unmapped `carla_id=-1`
phantoms), commit `206192bc`. freeze-1m runs (pre-fix) are abandoned; their rows
are quarantined out of the paper repo's `mock_data/`.

**MID-CAMPAIGN LOGGING ADDITION (2026-09-10, behavior-inert):** `runner_wired.py`
gained a `bytes=` field on the `[HANDOFFROW]` line (per-npc prepare
`payload_bytes`, stored in `npc_handoff_bytes` and emitted at scenario end) so the
trigger figure's wasted-bytes-per-crossing axis reads measured per-handoff bytes
instead of blank. This strictly bends one-freeze-one-code-state, allowed because
the field only reads a value already computed and already summed into `RUNROW by=`,
is stored in a dict never read during the sim, and is emitted only at scenario
end, so no running block's result can change (regions does not feed the handoff
figures and is NOT given this build). Tag does NOT move; still freeze-1n.
Commit `1ed3a4ba`. APPLIED to cetus 2026-09-11.

GATE OVERRIDE, recorded so the basis is visible (peer-agreed). The original gate
required a paired same-host/same-seed before/after run with IDENTICAL outcome
fields (episodes/contact_ticks/dist_m/time_s). That gate is unsatisfiable here:
the CARLA flow sim is not reproducible to a tenth of a metre. Three host-test reps
(identical code, same cell/seed, cold_v20_s1) span dist_m 92.5-94.9 (2.4 m) and
time_s 20.3-24.3 (4.0 s); the byte-field before/after diff was 0.5 m / 0.3 s, well
inside that. So the gate was overridden and the field applied on the CONSTRUCTION
argument, verified in code not asserted: `npc_handoff_bytes` is written in the tick
loop (runner_wired.py:677) and read only once, at the scenario-end `[HANDOFFROW]`
emit inside the post-loop `finally` block (line 866); it is never read during the
run, so it cannot influence the simulation regardless of any run pair. The noise
comparison is a sanity check, not the evidence.

INERTNESS METHOD (rule for future changes, both sessions): on a non-deterministic
simulator, inertness is established by (1) inspecting the code path to confirm the
change is never consumed during the run, and (2) if needed, comparing DISTRIBUTIONS
over many runs. NEVER by field equality between two runs; the flow sim jitters
~2.4 m / 4 s run-to-run at a fixed seed, so exact-match gates only ever fail.

Version-controlled snapshot of the settled code fixes so the set is reproducible
from the repo rather than living only in a scratchpad on one host. These files
OVERLAY the frozen base (commit `71c9f37e`, freeze-1h) at runtime: the campaign
scripts `cp` each overlay onto its base path, run, then `git checkout` the base
back (trap on EXIT). The frozen base is not modified. Bump this header's build
line whenever the overlay contents change to a new tag.

## Overlay map (overlay file -> base path it replaces)

| overlay file | base path |
|---|---|
| `edge_manager_merged_1l_epoch.py` | `ecav/core/application/edge/edge_manager/edge_manager_worldfusion_ab3dmot_linear_predictor.py` |
| `ns3_bridge_wired.py` | `ecav/core/networking/ns3_cosim/ns3_bridge.py` (co-simulation bridge: reads both Uu rows per UE, `shm_tag` for several bridges in one process; used only with `NS3_COSIM=1`) |
| `factories_1l.py` | `ecav/core/application/edge/migration/factories.py` |
| `mtr_edge_predictor_1m.py` | `ecav/core/prediction/mtr_edge_predictor.py` |
| `behavior_agent_1m.py` | `ecav/core/plan/behavior_agent.py` |
| `pluggable_base_wired.py` | `ecav/core/application/edge/edge_manager/edge_manager_pluggable_base.py` |
| `runner_wired.py` | `ecav/scenario_testing/openscenario_1_flow_gt.py` |
| `wrapper_wired.py` | `ecav/core/tracking/mamba3dmot/wrapper.py` |
| `tracker_wired.py` | `ecav/core/tracking/mamba3dmot/tracker.py` |
| `tracklet_1l.py` | `ecav/core/tracking/mamba3dmot/tracklet.py` |
| `payload_wired.py` | `ecav/core/application/edge/migration/payload.py` |
| `daemon_1l.py` | `ecav/core/application/edge/migration/daemon.py` |
| `scenario_1_1l.py` | `ecav/scenario_testing/scenarios/scenario_1.py` |

## Settled fixes captured here

- Defect B (migrated forecast invisible during the destination-blind window):
  coast-through-blind-window rebuild of the imported track's trajectory from the
  memo_bank translated to the coasted pose, sourced from tracked+lost, recreate
  if pruned (`edge_manager_merged_1l_epoch.py`).
- Ghost-filter fix: `_filter_ghost_tracks` falls back to `obstacle.kf_speed_mps`
  for Mamba tracks (AB3DMOT `.trackers` absent) so moving migrated tracks are not
  deleted as static.
- Acceleration-consumption fix: the gate uses the transferred `_migrated_accel_mps2`
  directly while the destination has `< LOCAL_ACCEL_THRESH` of its own local
  frames, then hands over to the local estimate; `[GATE_ACCEL]` logs used/src/
  migrated/local_est and the handover (`edge_manager_merged_1l_epoch.py`,
  `factories_1l.py`).
- Locale tag on `[MODESROW]`/`[MIG_SEAM]`, identity-settling `[MIG_IDENT]`.
- Round-trip relay transport (loud `[XFER_FALLBACK]` guard, `[XFERROW]`) in
  `daemon_1l.py`.

`scenario_1_1l.py` carries the paired spawn-phase seed (settled; SEEDROW,
deterministic per-index oncoming spawn-x offset, paired across arms) and an
endogenous acceleration-onset branch (ONCOMING_CRUISE + ONCOMING_ACCEL_ONSET).
The onset branch is inert unless ONCOMING_ACCEL_ONSET is set, so the file is safe
for the headline / record-ablation / corridor blocks that do not set it.

## Panel runner (parked)

`atlas_maneuver_panel.sh` is the maneuvering success-rate / FDE panel runner
(across-seed situational spread plus a same-seed repetition cell for run-to-run
jitter; log names match ACC_TAG_RE so `scripts/khonsu_1l_land.py acc` lands it).
It is versioned here so it is not lost, but it is NOT wired to a finalized
geometry. The closed-loop maneuvering collision panel was found not to be
load-bearing for the record-depth claim: that claim rests on the closed-loop
record ablation (the large separation is carrying velocity at all) and the
offline forced-handoff depth benchmark, not on a closed-loop collision. Whether
the projected-acceleration panel stays in the paper is Tyler's call.
