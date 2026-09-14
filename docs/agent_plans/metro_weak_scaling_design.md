# Metro replay: weak scaling over locale count + instrumentation (design, 2026-09-14)

Two items from Tyler, both on the landed replay machinery (scripts/metro/
metro_replay.py), neither gated on the predictor fix.

## 1. Weak scaling over locale count at fixed density

Replay L = 4, 16, 36, 64, 100 locales as CROPS of the Athens extension at the
same road-user density per locale: the top point (ext_L106_t3623, 3470 active on
106 locales, 32.7 tracks per locale, 14.3 veh/lane-km, the pNEUMA peak) and the
800-track point (7.5 per locale) as the lower density. All five arms, the
three admission rows, the landed charging rule (monotone envelope, marginal on
top of the admitted pass, deployed rate fallback for freshness). Output: p95
destination load per locale per arm (the landed dest_load_p95 definition),
schema = the metro rows + tracks_per_locale + the instrumentation below.

Crop mechanism (engine flag --tile_window kx0,ky0,kx1,ky1 or --tile_square k):
the 300 m tiling of junction anchors is kept; only the tiles inside a k x k
window centred on the tiling's centre keep their locale ids (remapped to
0..L-1); every other anchor is unlabelled (-1), so sample points outside the
window are off-network for the replay, a track leaving the window stops being
owned (no crossing recorded), and boundaries to unlabelled tiles are dropped
(no replication to outside; no crossing counted). The trace is unchanged, so
the density inside the window is the full network's density. Actual L is the
number of tiles in the window that hold anchors (reported; the 106-tile
tiling is not a full square).

Expected: about flat for Khonsu with locale count and, by the same locality
argument, about flat for the replication arms; the arms differ by a constant
factor set by local load, not by L. The block is not designed to make
replication explode: replication fan-out is bounded by the observed degree,
and a crop reduces degree at its rim.

## 2. Instrumentation on both sweeps (per row, aggregated over (locale, cycle))

- prepares_per_dest_cycle_{mean,p95}: record imports into a destination per
  cycle (warm: the computed trigger's prepares; replication arms: their
  imports/resends), over locale-cycles with at least one.
- predicted_to_cross_per_locale_{mean,p95}: distinct tracks prepared into a
  locale over the run (warm), per locale; with false_prep_frac = the share
  of those that never crossed into that destination.
- refreshes_per_dest_cycle_{mean,p95}: replica prediction refreshes per
  destination per cycle, over locale-cycles with at least one.
- crossing_burst_per_locale_cycle_{mean,p95,max}: actual crossings into a
  locale per cycle, over locale-cycles with at least one.
- tracks_per_locale: median active tracks per cycle / L (the paper's relabelled
  axis: 200 to 3200 over 100 locales = 2 to 32 per locale).
Same columns on the landed 100-locale density sweep (rerun of the 22 replays;
loads unchanged by construction, checked on one point).

## Tyler's sanity check (to be answered with numbers)

The landed demand rows show Khonsu's dest_load_p95 flat at 0.117 from 800 to
3200 tracks. The accounting: warm's charge per locale-cycle = create(imports)
+ [pattern(admitted + replicas) - pattern(admitted)]; the p95 locale-cycle
carries one added replica, so the charge is one marginal prediction record on
the envelope (about 8 ms = 0.117 x 70 ms) plus a 0.1 to 0.2 ms import. If the
instrumentation shows prepares per destination-cycle rising with density while
the charged compute stays flat, the accounting is wrong (invalidation). If it
shows the p95 prepares per destination-cycle at one and the marginal cost of
one record flat on the envelope, the flatness is the count saturating at one
prepared track per destination-cycle at p95, said as such.

## Invalidation conditions

- Loads on the landed points must be identical after the engine change (p95
  to 1e-9): else the change touched the charging and the block is invalid.
- Crop rim: if the observed degree inside a window is more than 20 percent
  below the full network's, the replication arms are understated by the crop;
  windows are then taken interior-only and the degree reported per L.
- If Khonsu's per-locale p95 changes by more than the seed-free replay's
  quantisation (one envelope step) across L, locality is not holding and the
  cause is reported (rim effects or admission ratio drift), not smoothed.
- If tracks per locale inside a window deviates more than 15 percent from the
  full-network value, the window is not at fixed density; move the window.

## Order

Engine patch -> identity check on ext t3200 deployed -> smoke (t3623, deployed
row, 2x2 and 10x10 windows) to the paper side -> grid (2 densities x 5 windows
x 3 rows = 30 replays) -> rerun of the landed 22 for the columns -> landers.
