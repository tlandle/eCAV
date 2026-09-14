# Tracker window and predictor window microbenches (design, 2026-09-13)

Requested by the paper side (Tyler): what each extra frame of history costs
inside the tracker per tick, and what a longer predictor window costs in MTR
forward time. Offline, same code paths and GPU setup as the deployed stack,
timing only, outputs discarded, no retrain. Generator placeholders exist on the
paper side (linear tracker cost; predictor cost anchored at the measured 10.75
ms single-record predict at ten frames, superlinear above); measured values
replace them.

## Block A: tracker per-tick update cost vs window (frozen1r_tracker_window_rows.csv)

- Tracker: Mamba3DTracker with the deployed config (the corridor/flow yaml
  block: max_window, enable_time_thresh 1000000, coast_window 40, etc.), on
  cuda:0, opencda310. max_window varied over {1, 2, 5, 10, 20}; the memo and
  diff banks are filled to that depth before timing (tracklets driven with
  max_window frames of detections at 0.05 s cadence).
- Two sizes: 1 track and 8 tracks in the same tracker (the batch effect).
- Timed quantity: one tracker.update(detections, scores) call per tick with
  one detection per track (association + per-tracklet update at the window
  depth), wall time via perf_counter with cuda synchronize before and after,
  divided by n_tracks: update_ms_per_track. Warm-up 5 calls, then >= 20 reps.
- Also recorded (same run, separate rows in a sibling file): the learned
  head's predict() per tracklet at the same depths with enable_time_thresh at
  the harness value (5), so the paper side has the cost of the Mamba motion
  head if it ever turns it on; labelled not deployed.
- Schema: window_frames, n_tracks, update_ms_per_track, rep (+ update_ms
  total, head, enable_time_thresh, device, git head).

## Block A2: import cost of a record of depth K (same run)

- create(K) = latent_from_tracklet(history_depth=K) + one MigrationPayload
  serialize + deserialize + inject_latent_into_tracker into a fresh tracker,
  for K in {1, 2, 5, 10}, one record per call (the per-record cost; the batch
  curve is in the metro microbench). Warm-up 5, >= 20 reps.
- Schema: depth_frames, import_ms, bytes, rep.
- Comparison values on file: full record 4.01 ms vs delta 0.40 ms (the paper
  side's numbers); the metro microbench measured create at 0.10-0.19 ms per
  record on this host, so a disagreement is expected and will be stated with
  the code path each number comes from.

## Block B: MTR forward time vs past_frames (frozen1r_predictor_window_rows.csv)

- Predictor: the overlay MTREdgePredictor as the campaign builds it
  (scripts/metro/microbench_ops.py build_predictor, freeze-1p overlay in
  idfix_wt), past_frames in {10, 20, 40} passed to the constructor; history
  inputs replicated/padded so the tensor shapes follow (the model's encoder
  takes past_frames x features per agent); the checkpoint is fixed; outputs
  discarded.
- Agent count fixed at the corridor's typical count (the [ISOROW] N in the
  campaign logs; N = 2 with the cap pattern), single track and batch of 8
  tracks; >= 20 reps after warm-up; cuda synchronize around each call.
- Schema: past_frames, n_tracks, predict_ms_per_track, rep (+ predict_ms
  total, agents, cap, device).
- Invalidation: if the model rejects past_frames other than its trained
  value (fixed positional embeddings or a hard-coded history length in the
  encoder), the block reports the rejection with the file and line rather
  than substituting a padded call; if the forward time at 20 and 40 is within
  the 10-frame spread, the "attention grows with the window" premise is not
  supported on this model and is reported as such.

## Host and ordering

One GPU job at a time. Order: frontier smoke (running) -> Block A + A2 ->
Block B. Worktree: idfix_wt with the freeze-1p overlay for Block B (the
compiled MTR ops live there); Blocks A and A2 run from the main checkout.
