# Table 3: destination compute of the replication arm (design, 2026-09-13)

Status: design only. Build waits on Tyler's freeze-1q ruling; the markers are
bundled into the 1q overlay set so the 1q corridor rerun produces the columns
(fix-and-rerun-whole: no separate corridor block).

## What the table needs

Duplicated compute per crossing for the full-record replication arm
(repl_final), in ms per crossing, reported the same way as for the overlap
arms, with tracking and prediction separate (the paper will describe
prediction at the destination as local work under option B for Fig 4(b)).

## Why it cannot come from landed logs (verified 2026-09-13)

- dual_fuse_ms / dual_predict_ms (scripts/khonsu_corridor_land.py:84-95) are the
  unweighted mean over every [DUALROW] line (one per tick x edge x non-ego
  track) of the edge-wide fusion-block and prediction-block ms: a per-tick edge
  block cost, not a per-crossing increment; cold carries 62.2 ms.
- both_locales_hold_track is always 0 in the corridor: the overlay
  (code_overlay/edge_manager_merged_1l_epoch.py:878-882) reads tracked_tracklets
  off the Mamba wrapper; the tracklets live on wrapper.tracker.
- The tracking block is untimed; the destination import
  (migration/daemon.py:277) is untimed; [OBSTACLE_HANDOFF] total_ms is the
  modelled link cost; the MTR line "[MTR Edge] N tracks predicted in X ms" has
  no edge id.
- [CORRIDORCROSS] is emitted per PREPARATION (openscenario_1_corridor_gt.py:
  823-842); crossing_tick = -1 marks a preparation that never crossed (11 of 32
  landed rows); bytes_per_crossing (lander :387) divides by preparations.
- No corridor logs on atlas; they are on cetus.

## Markers (additive; overlay files, mirrored in-tree)

1. Time the tracking block like fuse and predict (overlay ~684):
   _dual_track_ms.
2. Imported carla-id dict at inject (edge_manager_pluggable_base.py ~311 and
   ~336): replica identity today is inferred from _migrated_vel_mps, which the
   tracklet clears on its first fresh observation. Also fixes the always-0 flag.
3. One line per tick per edge, emitted beside the [DUALROW] loop:
   [REPLCOST] tick=%d edge=%s n_tracks=%d n_replicas=%d n_shadow=%d
   fuse_ms=%.2f track_ms=%.2f predict_ms=%.2f
4. import_ms=%.3f appended to the existing [OBSTACLE_HANDOFF] line (prefix
   regexes keep working).

## Lander columns (scripts/khonsu_corridor_land.py, appended; nothing landed
changes meaning)

- dual_fuse_ms, dual_predict_ms: unchanged, described in the notes as per-tick
  edge block costs (paper side's request).
- n_preparations, n_completed_crossings (crossing_tick != -1), n_false_preparations.
- bytes_per_completed_crossing and bytes_per_preparation (both denominators;
  the paper defines transfer per completed crossing and counts false
  preparations separately).
- dst_track_ms_per_crossing = sum over (tick, edge) of
  n_replicas / max(n_tracks, 1) x track_ms, plus sum of import_ms,
  divided by completed crossings.
- dst_predict_ms_per_crossing = sum over (tick, edge) of
  n_replicas / max(n_tracks, 1) x predict_ms, divided by completed crossings.
- dst_track_ms_per_preparation, dst_predict_ms_per_preparation (the other
  denominator).
- Populated for every arm; cold is the built-in zero; overlap arms get the
  same columns so the two definitions can be compared.
- Model-side cross-check printed beside: the metro microbench
  (scripts/metro/microbench_ops.py: create N = record import, predict N) on the
  same code path, charged as imports x create(1) + replicas x marginal
  predict, so the corridor number and the microbench number sit side by side.
- Optional (same logs): wasted destination compute per false preparation =
  the replica share of track_ms + predict_ms over the false preparation's
  life (first_dst_track_tick to its last [PUBGATE] line) plus its import_ms.

## Attribution assumption and its check

The replica share n_replicas / n_tracks assumes a uniform per-track cost and
a shared fixed per-call overhead. Check: the MTR per-track cost from the
"[MTR Edge]" line and the microbench marginal must agree with the share within
the microbench's own batch-size spread (predict(N) envelope); if the corridor
share exceeds the microbench marginal by more than 2x at the observed N, the
attribution is invalid and the per-replica cost is measured directly by a
counterfactual pass (predict with and without replicas on the same tick's
tracks), which is a second marker, not a re-definition.

## Invalidation conditions

- If n_replicas is 0 on every tick of a repl_final run, the imported-id dict
  is not wired to the edge class the corridor uses (WorldFusionMambaAdaptiveEdge
  via _PluggableEdgeBase); the block is invalid until the marker shows
  replicas.
- If dst_track_ms per crossing for cold is not ~0, the share attribution leaks
  local work; invalid.
- If the completed-crossing count differs from the [SCENB] crossing markers,
  the denominator is wrong; invalid.

## Dependencies

- Tyler's 1q ruling (fix shape); the markers ride the 1q overlay set.
- A host that can run the corridor (cetus unreachable since 2026-09-12 21:16;
  atlas can run it at ~27 min per route class).
- Paper side changes (theirs): plot_figures.py overlap gate at 1104-1105 and the
  dup aggregation at 1021; figure_rules.py 362-368.

## Additions (2026-09-14, Alex's comments via the paper side)

### Added destination compute per crossing, all arms (dest_added_ms, dest_added_frac_slack)

Table 3's cost column becomes "added destination compute per crossing" as a
fraction of the 70 ms available cycle time, from the destination edge's own
timing, per arm:
- Khonsu (warm): import_ms of the PREPARE record + import_ms of the final
  update at the crossing + the replica share of the prediction block over the
  cycles the track is held as a replica before the crossing (the added
  record's prediction step, summed per crossing).
- history at crossing (reactive): import_ms at the crossing + the replica
  share of the prediction block in the cycle of the crossing.
- EdgeWarp-style trigger (edgewarp, edgewarp_full): both records' import_ms +
  the replica share of prediction over the pre-copy's life.
- full-record replication (repl_final): every import_ms (each resend) + the
  replica share of the prediction block over every cycle the track is held in
  the boundary band, summed per crossing.
- overlap arms (band20/40/80): the dual fuse and predict already measured.
Per-arm sums come from the same additive markers already designed: [REPLCOST]
(n_tracks, n_replicas, fuse_ms, track_ms, predict_ms per tick per edge, with
the imported-id set so a replica stays attributed after its first fresh
observation) and import_ms on [OBSTACLE_HANDOFF]. The lander column
dest_added_ms = that sum divided by completed crossings (a per-preparation
variant beside it), and dest_added_frac_slack = dest_added_ms / 70. The
tracking share (track_ms x n_replicas / n_tracks) is reported in its own
column and included in dest_added_ms, since the paper counts tracking as
destination work. No new run set: a column on the 1q corridor landing.

### Wire bytes per crossing (wire_bytes_per_crossing)

The landed bytes_per_crossing (938 for Khonsu in frozen1m) is the declared
payload model (TransferCost.payload_bytes = payload.payload_bytes()), not the
wire size. A completed Khonsu crossing is two complete records (the PREPARE
and the final update at the crossing, COMMIT_REFRESH full) plus the commit
token; on the gRPC path the token is a TransferChunk with an empty payload,
on the parametric path the commit carries no bytes. Marker (v):
wire_bytes=len(payload.serialize()) appended to the [OBSTACLE_HANDOFF] line on
the parametric path (the blob is already produced on the gRPC path and logged
as [XFERROW] bytes=). Lander: wire_bytes_per_completed_crossing and
wire_bytes_per_preparation beside the existing bytes_per_crossing, with the
token counted at its TransferChunk size when the gRPC path is on and zero
otherwise, stated in the notes. The 1247 versus 1462 B question is Tyler's.

