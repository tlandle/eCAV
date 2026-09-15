# Locale sizing on the Athens replay (Figure 11 / §7.2.2 rebuilt measured)

Tyler's ruling (2026-09-15, via the paper side): the sizing figure is rebuilt
on the Athens replay tiling so the paper has two setups only, the closed loop
and the replay over the pNEUMA district. Design first; grid and cost to the
paper side before the grid runs.

## What is replayed

The pNEUMA 2018-10-24 09:00 to 09:30 slot, all drones (the measured point of
Figure 10: 6,988 tracks, 91.1 lane-km, 14.9 veh/lane-km), on the same network
(athens_pneuma_pad50.net.xml) with the Q7 partition (locales anchored on
junctions, boundaries at road midpoints between junctions of different
locales), the deployed admission row (K from the deployed cycle {1, 2}, 92 ms
pass budget), marginal charging on the monotone envelope of the measured
batch costs, and load-coupled freshness, exactly as the landed deployed row.
The only swept input is the tile side that groups junction anchors into
locales (metro_replay.py --tile). Warm (Khonsu) arm only: the figure draws
Khonsu's load and freshness; the other arms are Figure 10's business.

The tile side is the nominal locale size. A locale is the union of the road
points nearest to the junctions of one tile, so its realized extent is
junction-shaped, not square; tracks per locale and boundaries per locale are
reported beside the side as the realized size. The tiling origin is the
anchors' bounding-box corner (fixed; no reason to randomize it: the district
has 482 junctions and every side from 100 to 300 m leaves many per locale).

## Quantities per tile side (lander schema, frozen1r_sizing_rows.csv)

| column | definition | source |
|---|---|---|
| tile_side_m | nominal locale side | argument |
| locales | number of locales the tiling produces | replay JSON L |
| tracks_per_locale | active tracks p50 / locales | replay JSON |
| fusion_util | canvas latency sweep at 8 contributors (canvas_latency.csv, measured on the edge GPU) interpolated at canvas side = tile side, over the 130 ms pass allocation; blank beyond 563 m (the sweep's last measured side; fusion_measured = 0) | canvas_latency.csv |
| dest_load_p95 | Khonsu's destination processing load, p95 over locale-cycles, as a fraction of the 70 ms that remain of the 200 ms cycle after the 130 ms allocated to fusion, tracking and prediction (Figure 10's definition and charging) | warm p95_load_frac |
| within_limit | forecasts within the age limit, load-coupled, post-crossing window (Figure 10's within_limit) | warm within_limit_postcross_coupled |
| crossings_per_track_per_min | crossings per track-second on the 5 Hz grid x 60 | trace_stats |
| prepares_per_locale_cycle | prepares per locale-cycle | registry prepares_per_dest_cycle |
| never_crossed_frac | prepared tracks that never enter the prepared locale | registry never_crossed_frac |
| late_prepare_frac | crossings whose prepare landed fewer than late_cycles = transfer + fold-in = 4 cycles (0.8 s) before the crossing, or that had no prepare | registry (new: late_prepare_frac) |
| unprepared_crossing_frac | crossings with no prepare at all (subset of late) | registry (new) |
| boundaries, degree_used, dest_load_mean, within_limit_mechanism, fusion_measured, notes | context | replay JSON |

The feasible range is where fusion_util < 1 and dest_load_p95 < 1. If the
handoff term never crosses 1 at small sides, that is the result; the lower
bound then comes from freshness or from the lead exceeding the locale transit
time, which late_prepare_frac and never_crossed_frac per side show.

## Grid (from the partition alone, no trace; 2026-09-15)

| side m | locales | neighbour pairs | degree | fusion_util |
|---|---|---|---|---|
| 100 | 233 | 456 | 3.91 | 0.083 |
| 125 | 170 | 350 | 4.12 | 0.125 |
| 150 | 127 | 261 | 4.11 | 0.184 |
| 175 | 101 | 212 | 4.20 | 0.273 |
| 200 | 76 | 163 | 4.29 | 0.362 |
| 225 | 61 | 124 | 4.07 | 0.462 |
| 250 | 51 | 109 | 4.27 | 0.563 |
| 275 | 46 | 96 | 4.17 | 0.665 |
| 300 | 36 | 67 | 3.72 | 0.814 |
| 325 | 33 | 66 | 4.00 | 0.983 |
| 350 | 30 | 56 | 3.73 | 1.151 |
| 375 | 26 | 49 | 3.77 | 1.319 |
| 400 | 20 | 40 | 4.00 | 1.488 |
| 425 | 20 | 40 | 4.00 | 1.727 |
| 450 | 19 | 33 | 3.47 | 1.966 |
| 475 | 18 | 33 | 3.67 | 2.205 |
| 500 | 14 | 26 | 3.71 | 2.444 |
| 525 | 13 | 21 | 3.23 | 2.683 |
| 550 | 12 | 20 | 3.33 | 2.922 |
| 575 | 12 | 21 | 3.50 | beyond the sweep |
| 600 | 12 | 19 | 3.17 | beyond the sweep |

(Neighbour pairs = adjacent locale pairs; the engine's `boundaries` column counts boundary roads, 566 at 100 m to 99 at 600 m, 214 at 300 m.) Facts of the tiling: the anchors span 1581 x 1988 m, so from 550 m the grid is
3 x 4 tiles and 575 and 600 m give the same twelve locales (reported as
duplicates of 550); 400 and 425 m both give 20 locales with different
groupings. Fusion utilization crosses 1 between 325 and 350 m on the measured
sweep. 25 m steps are the paper's original grid; cost decides between 25 and
50 (see below).

## Invalidation conditions (lander-checked)

1. The 300 m run of this sweep reproduces the landed deployed row (warm
   p95_load_frac, within_limit_postcross_coupled, prepares_per_dest_cycle,
   never_crossed_frac) to the printed digits; otherwise the sweep is not the
   figure's replay and stops.
2. locales decreases and tracks_per_locale increases monotonically with the
   side (equal at the duplicate sides); a violation is a tiling defect.
3. crossings_per_track_per_min decreases with the side; a rise is a labelling
   defect (junction-anchored locales can interleave at small sides).
4. fusion_util is read from canvas_latency.csv only; sides beyond 563 m carry
   no value and are infeasible by the last measured point (2.9 at 563 m).
5. late_prepare_frac and never_crossed_frac at 300 m equal the landed
   instrumentation (never_crossed 0.0076).
6. Duplicate partitions (575, 600 = 550) are flagged in notes, not drawn as
   independent points.
7. PYTHONHASHSEED=0 and the deterministic partition; the engine md5 is
   recorded in every JSON's log.

## Smoke (300 m, 2026-09-15 18:05 to 18:10)

Warm-only deployed run, engine md5 9539c570: 314 s wall, 2.5 GB. Condition 1
holds: dest_load_p95 0.135, within_limit 0.995, prepares per locale-cycle
0.0944, never_crossed 0.00758, all equal to the landed deployed row. New
registry fields at 300 m: late_prepare_frac 0.0336 (late_cycles 4),
unprepared_crossing_frac 0.0012, crossings 14,892, crossings per track per
minute 0.740. Grid cost at this rate: 21 sides x 5.3 min = 110 CPU-min; six in
parallel about 20 min wall and 15 GB.

## Cost and order

One warm-only deployed run at 300 m is the smoke: it gives the reproduction
check (condition 1) and the wall time per side. Grid = 21 sides (or 11 at 50 m
steps if a side costs more than about ten minutes), run six in parallel on
atlas's CPUs (no GPU; the replay charges measured costs from the microbench
table). Then scripts/metro/sizing_rows.py writes frozen1r_sizing_rows.csv,
the paper side's compare script is run read-only against the generated rows,
and the report goes to the paper side.

## Sweep landed (2026-09-15 18:12 to 18:34; evaluation_outputs/metro/sizing/)

21 sides, 314 to 351 s each, six in parallel, all exit 0; rows in
frozen1r_sizing_rows.csv, checks in sizing_checks.txt. Condition 1 holds
(300 m equals the landed deployed row). Condition 2 holds. Condition 3 fails
as written: crossings per track per minute fall from 1.67 (100 m) to 0.39
(600 m) but reverse at four adjacent pairs by 0.02 to 0.03 per minute (325
to 350, 425 to 450, 475 to 500, 525 to 550), about 3 to 5 percent. The
fixed-origin tiling moves its edges across specific roads as the side
changes, so a step can put boundaries on busier roads than the step before;
the reversals are a property of the tiling on a real network, not a
labelling defect, and the condition is restated as "monotone trend, local
reversals under 5 percent". The paper side decides whether the figure draws
the rows as measured (recommended) or averages over tiling origins.

Findings: Khonsu's p95 destination load is 0.134 to 0.135 of the slack at
every side (the p95 locale-cycle carries one to two added tracks at every
side, and the charging envelope's isolated-plus-pad step costs the same
whatever the side; the mean load carries the trend). Forecasts within the
age limit 0.993 to 0.999 everywhere. Late prepares 1.5 to 7.3 percent of
crossings with no trend in the side (the 0.99 s lead never exceeds the
transit time of a 100 m locale at urban speed); unprepared crossings 0.01 to
0.64 percent; never-crossed 0.2 to 2.3 percent, no trend. Fusion utilization
crosses 1 between 325 and 350 m. Feasible range on this trace: from the
grid's smallest side (100 m) to 325 m, bounded above by fusion only; no
handoff-side quantity bounds it from below within the grid.

