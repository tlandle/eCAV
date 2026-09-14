# State frontier by maneuver (offline forced-handoff block), design 2026-09-13

Requested by the paper side (Tyler's ruling): Figure 4 becomes scenario-aware;
measured rows with columns maneuver {straight, brake, turn, lane_change},
representation {none, position, pos_vel, kalman_state, hist2, hist5, hist10,
no_handoff}, kb (serialized record size), fde5_m, rep. Lander name
frozen1r_state_frontier_rows.csv. Offline: the forced-handoff harness
(ecav/core/application/edge/migration/harness.py), no closed loop, no network.

## What the harness measures, and what it cannot

- The harness has NO predictor. Its forecast is the Mamba tracker's own motion
  head rolled out open-loop: the learned one-step model when the memo bank
  holds >= enable_time_thresh frames (5 in the harness config), else a
  constant-velocity coast on the record's vel_mps or on the memo-bank
  endpoints (tracklet.py:112-165). The deployed edge sets enable_time_thresh to
  1000000, so the deployed head is constant velocity. MTR is not in the loop:
  the block is NOT gated on the freeze-1q fix shape.
- Standing KB decision (current_state.md, "MICROBENCH (Table 6) DIAGNOSIS"):
  a 5 s open-loop FDE from this harness is model drift with no predictor in
  the loop and is not a paper quantity (measured 2026-08: 13.7 m straight for
  the learned rollout vs 0.32 m constant velocity; turn and brake inverted,
  cold ahead of history, because the maneuvers begin at the handoff and a
  static hold moves less than a coast). Table 6 and the depth replay (Fig 5)
  landed on the one-step tracking-recovery error instead.
- The anchors the request names (1.43 pos+vel, 0.65 ten frames, 0.60
  reference) are the closed-loop FDE block's MTR numbers (gen_numbers.tex
  nFdeKf / nFdeFullhistory / nFdeNohandoff), a different quantity; the offline
  harness cannot land near them by construction.
- The four maneuvers begin AT the handoff frame in the harness (turn, brake,
  lane change at handoff=10 after a 20 m/s cruise), so no transferred state can
  encode the maneuver; transferred depth can only change the velocity estimate.
  On noise-free trajectories every velocity-carrying arm has the exact velocity.
- no_handoff is the source tracker continuing; by the harness's own parity
  checks it is identical to hist10 (state parity, prediction parity).

## Arms as built (scripts/frontier/state_frontier.py)

Source edge A tracks the pre-handoff frames; src = its tracklet. Destination:
- none: fresh tracker, nothing injected.
- position: record with history_depth=1, no vel_mps (the MIGRATION_MODE=kf
  export as it lands on a Mamba destination); static hold until the first
  local detection, then constant velocity over the two frames.
- pos_vel: history_depth=1 plus vel_mps from the source memo endpoints (the
  full-depth export's velocity); cfg _spf_live = dt so the coast is scaled to
  the harness cadence.
- kalman_state: the AB3DMOT filterpy KF (10-state constant velocity,
  AB3DMOT_libs/kalman_filter.py) run over the pre-handoff detections; the
  record carries KFState (mean, covariance) beside a depth-1 box; the Mamba
  destination is seeded from the mean (box + velocity); the covariance is
  carried in the bytes and unused by this destination (stated).
- hist2 / hist5 / hist10: history_depth = 2 / 5 / 10, no vel_mps (velocity from
  the memo endpoints, as the deployed coast does).
- no_handoff: edge A continues over the post-handoff frames.
kb = len(MigrationPayload.serialize()) / 1000 for the arm's record (0 for
none and no_handoff).

Metrics per (maneuver, arm, rep), window = first 5 post-handoff issue frames
(the harness gap window and the 2026-08 recompute):
- fde5_m, fde3_m: mean over the window of the open-loop rollout endpoint
  error at 5 s / 3 s against the trajectory's true position (recompute rule;
  a destination without a forecast scores the static hold).
- one_step_err_m: mean over the window of the tracker's one-step-ahead error
  (Table 6 quantity, harness FrameResult.error_xy).
- frames_to_first_forecast: post-handoff frames until the destination can
  roll out (0 for injected arms; 1 for cold, which needs two detections).
- coverage: fraction of window frames with a forecast.
Options: --head cv (deployed, default) | mamba (harness config); --onset
(maneuver onset frame; 10 = at the handoff, the harness definition; smaller =
the maneuver begins before the handoff so history can encode it);
--noise_sigma and --reps (seeded detection noise; the harness is deterministic,
so rep = 1 without noise and rows are exact repeats).

## Predictions written before the smoke

1. Under the deployed head (cv), noise-free, onset at the handoff: every
   velocity-carrying arm (pos_vel, kalman_state, hist2/5/10, no_handoff) gives
   the same rollout; position differs only at issue frame 0; none differs by
   the missing first frame. On turn and brake at 5 s, none and position
   (static hold) score BELOW the coast arms (the 2026-08 inversion).
2. Under the harness head (mamba): hist5, hist10, no_handoff roll the learned
   model (about 14 m at 5 s on straight); the shallow arms coast at constant
   velocity until they hold five frames, so they score better on straight.
3. Depth becomes a mechanism only with detection noise (longer windows average
   noise) or with a maneuver onset before the handoff (then only a head with
   turn-rate or acceleration state could use it; constant velocity cannot, and
   the chord bias makes longer windows worse on a turn).

## Invalidation conditions (block-level)

- If prediction 1 holds, the fde5 column carries no depth information and the
  block cannot check the paper's per-maneuver claims on that quantity; the
  block is reported as such and the paper side chooses the quantity
  (one-step recovery error, or MTR in the loop, which reopens the Q1 ruling
  and the fix-shape gate).
- If no_handoff differs from hist10 by more than the parity tolerance, the
  injection is lossy and the harness itself is broken; stop.
- If kalman_state's mean velocity differs from the endpoint velocity by more
  than the KF's own steady-state error on the straight cruise, the KF seeding
  is wrong; stop.

Smoke before any grid: rep = 1, both heads, onset 10 and onset 5, sent to the
paper side with this note.
