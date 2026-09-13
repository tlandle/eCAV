# Metropolitan Replay Design (NSDI experiment 2 of 4)

Status: DRAFT v2, 2026-09-12. Spec from the paper side (fig:metro, Figure 14
in 5.9, and appendix C batch-cost microbenchmark). v1's bytes-per-second
currency was wrong: the paper's claim is that the cost separating Khonsu
from replication is destination PREDICTION processing, not bandwidth. Open
items marked OPEN. Nothing built; order after the branching harness.

## Claim the experiment feeds

fig:metro: x = active tracks {200, 400, 800, 1600, 3200} at 100 locales, and
a second sweep over locales {16, 36, 64, 100} (OPEN: at which track count).
Left y = p95 destination processing load per edge cycle as a fraction of the
70 ms cycle slack (200 ms cycle minus the 130 ms allocation). Right y =
fraction of forecasts within the age limit. Paper numbers at 100 locales and
3200 tracks: delta_state 7% load, Khonsu 18%, delta_ready 139%, full-record
290%; Khonsu 93% within limit. Finding: keyed-delta replication is cheaper
than Khonsu but its forecasts are stale; making its replicas forecast-ready
costs 139% of the slack. Bytes/s is a secondary column only.

The draft's numbers come from a generator with assumed traffic statistics
(near-boundary fraction 0.22 of active tracks, neighbour degree
2.6 + 0.4 ln(locales/16 + 1), crossing rate 0.013 per track per second
calibrated to the corridor). The replay MEASURES these from SUMO
trajectories on real city geometry and reports measured against generator
side by side; it never substitutes the generator's constants.

## Arms (five, fixed by the paper; EdgeWarp is not in this figure)

| Arm | Replay rule per 200 ms edge cycle | Destination processing charged |
|---|---|---|
| warm (Khonsu) | computed trigger on the CV projection (transfer EMA + fold-in + margin, capped); one prepare to the projected destination; final update at the crossing | import + prediction only for the tracks it prepares, at the batch size it prepares them in |
| replall (full record) | every cycle, retransmit and import the complete record of every near-boundary track to every neighbour | full import + prediction for every replica every cycle |
| delta_state | install a replica once, then every cycle apply only the newest observation through a stable replica id; no prediction until the crossing | delta apply for every replica every cycle; prediction only at the crossing |
| delta_ready | delta_state plus a prediction refresh for every replica every cycle | delta apply + prediction for every replica every cycle |
| reactive | history transferred at the crossing, no lead | import + prediction at the crossing |

Periodic at the 200 ms edge cycle ("each cycle"), not event-driven; no
period sweep (extra if ever added, not the figure).

## Cost model (appendix C, measured, batches 1 to 32)

Per operation: full import 3.1 ms per track; delta apply 0.15 ms per track;
prediction 8.0 ms base + 2.5 ms per track per batch. OPEN: the replay wants
the measured per-batch-size curve points (1 to 32) rather than the linear
fit, so the per-record cost at batch N is read from the batch-N point.
"Genuinely batch": one serialization and one import call per N records per
cycle per locale, with the per-record ms taken from the batch-N point of the
microbench curve, never N times the batch-1 cost. Load per locale per cycle
= sum of charged operations in ms; the figure reports p95 over locale-cycles
(OPEN: confirm the aggregation) divided by 70 ms.

Freshness: OPEN, needed before the engine is written: the age limit value
and the per-arm definition of a forecast's age (for delta_state the replica
holds fresh observations but no forecast until the crossing; whether the
count is per track per cycle at the serving locale, or per crossing).

## Method

1. Map: an OSM district extract to OpenDRIVE with carla.Osm2Odr (client
   side) and to a SUMO network with netconvert. The Q7 partition rule
   (locales anchored on junctions, boundaries at inter-junction midpoints)
   runs on the OpenDRIVE road graph through the client-side carla.Map, same
   function as locale_partition.py; the Town03 result must reproduce the
   committed Q7 numbers (invalidation 4). The district needs at least 100
   junction anchors; the locale sweep uses nested sub-windows with 16, 36,
   64 and 100 anchors (OPEN: or the paper's own meaning of the locale axis).
   District choice (OPEN, Tyler may have a preference): chosen for
   calibration-matchability; junction degree distribution and per-boundary
   crossing rate reported against Town01/Town03 and against the corridor's
   0.013 per track per second.
2. Traffic: SUMO with randomTrips/duarouter, demand levels set so the
   active track count in the deployment reaches each x value; floating car
   data at 20 Hz (position, speed, heading, lane). One trace per (demand,
   seed) is the input to every arm.
3. Replay engine: walks the trace by 200 ms cycles through the locale
   registry (LocaleRouter, Locale, sticky assignment as in the runner);
   per locale per cycle it knows owned tracks, near-boundary tracks,
   replicas per arm, prepares, crossings; it charges the arm's operations
   from the cost model; it records freshness per the OPEN definition.
   Trigger is CV on the trace (destination + crossing time); MTR is not in
   the loop; the branching harness carries the MTR destination question.
4. Measured statistics reported against the generator: near-boundary
   fraction, neighbour degree, replicas per locale, crossing rate per track
   per second, per-boundary crossings per minute, CV wrong-destination rate
   by junction degree.

## Invalidation conditions (lander-checked, in ms of destination processing)

1. Trace differs across arms: every arm replays the identical FCD file
   (hash in every run row).
2. Costs not the measured ones: the per-operation table (appendix C file
   and its hash) is logged per run; any other constant flags the run.
3. Batching violated: per locale per cycle at most one import call and one
   prediction call per arm, N records each, per-record ms from the batch-N
   point; a lander check counts calls against locales x cycles.
4. Partition rule differs from Q7: Town03 through the same function must
   reproduce locale_partition.csv.
5. Crossing time modeled: the crossing is the trace's own entry into the
   destination polygon, per actor, per cycle.
6. District unlike the calibration: degree distribution and crossing rate
   reported next to the towns and the corridor; a district whose crossing
   rate differs from 0.013 by more than a factor the paper side sets is
   reported as such, not silently used.
7. Generator constants substituted for measurements anywhere in the replay.

## Outputs

fig_metro_tracks.csv (x tracks, per arm: p95 load fraction, within-limit
fraction, bytes/s secondary), fig_metro_locales.csv, metro_stats.csv
(measured vs generator statistics), district_q7.csv (separation table on
the district), cv_destination_by_degree.csv, checks.txt.

## Tooling and build order

- Separate conda env metro_env (python 3.10, eclipse-sumo user level,
  shapely, numpy, the carla 0.9.15 wheel for Osm2Odr and carla.Map).
  opencda310 is not touched while the campaign runs in it (paper side and
  Tyler flagged). CPU only, no CARLA server, no GPU; runs beside the
  campaign.
- Build: (1) district extract + OpenDRIVE + SUMO net + partition and the
  Town03 reproduction check; (2) SUMO demand and FCD export; (3) engine;
  (4) lander; (5) smoke at the smallest sweep point (16 locales, 200
  tracks), all five arms, output to the paper side before the sweep.
