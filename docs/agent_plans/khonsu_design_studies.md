# Khonsu Design Studies (post-defense, 2026-08-28)

Committee-mandated experiments (defense 2026-08-27) plus the two design
questions raised while writing the dissertation chapter. All run on the
existing flow harness (`openscenario_1_flow_gt`, GT-based two-locale blind
overtake with steady oncoming flow), which produced the Q3/Q4 tables in
`docs/kb/data/relay_eval_2026_08/`.

## Existing arm structure (unchanged)

`MIGRATION_MODE` selects payload x timing on a clean 2x2 + control:

| mode | payload | timing |
|---|---|---|
| warm | full context | LOOKAHEAD_S ahead (predictive) |
| reactive | full context | at crossing |
| kf | depth-1 snapshot | LOOKAHEAD_S ahead |
| edgewarp | depth-1 snapshot | at crossing |
| cold | none | never |

## New knobs (implemented 2026-08-28 in openscenario_1_flow_gt.py)

- `TRIGGER_MODE=predictive|band`, `BAND_W_M`: geometric trigger fires when
  the NPC is within W meters of the source boundary (signed_distance), no
  trajectory projection. The dissertation's trigger-axis claim.
- `COMMIT_REFRESH=none|full`: one full re-send at the actual crossing.
  The "phase 2" question: does a commit refresh buy anything at 1.3 KB.
- `MIRROR_PERIOD_S`: periodic re-send after first trigger until crossing.
  The mirroring-rate axis (trigger-once -> standby replication).

## Sweep matrix

| Study | Arms | Reps | Answers |
|---|---|---|---|
| S1 trigger axis | warm@LOOKAHEAD_S=1.0 (exists), band W in {5,10,20,40,80}, reactive (exists) | 10 | band vs forecast trigger; dissertation 6.3.1 claim |
| S2 commit refresh | warm x REFRESH in {none (exists), full} | 10 | phase-2 value; protocol subsection claim |
| S3 mirroring rate | warm + MIRROR_PERIOD_S in {0.25, 0.5, 1.0, 2.0} | 10 | continuous replication vs trigger-once; overlap-section arm |
| S4 seeds top-up | all five MIGRATION_MODE arms | to 20 total | headline table robustness (committee) |

Metrics per run (existing): collision episodes, contact ticks, distance,
time, completed-within-deadline, transfer count + bytes
(record_handoff_cost). Derived: binary collided/not-collided (episodes>0),
per committee metric directive. Bytes totals matter for S3 (the cost side
of the mirroring axis).

## Analysis notes

- Binary metric re-derivation also applies retroactively to
  q4_flow_table.csv and q3_lookahead_sweep.csv: clean = episodes==0.
- S1 selectivity caveat: this scenario's traffic crosses the boundary, so
  the boundary-parallel over-firing cost of bands is NOT exercised here.
  The dissertation's selectivity claim rests on the transfer-count
  comparison and the geometry argument; a boundary-parallel traffic
  variant is future work if a reviewer demands it measured.
- S2 interpretation guard: the destination observes the NPC during the
  lead (range-based sensing), so a null refresh result means the
  destination's own observations already cover the tail, which is the
  honest expected outcome.
- Dual-service overlap arm: in this GT harness the destination already
  observes across the boundary by range, so the cold arm IS
  rebuild-from-own-observation, and its measured penalty bounds what
  overlap-dwell warming can deliver. Report cold's first-observation-to-
  crossing window from logs alongside its penalty instead of building a
  separate arm.
