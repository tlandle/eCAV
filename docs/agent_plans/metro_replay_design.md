# Metropolitan Replay Design (NSDI experiment 2 of 4)

Status: DRAFT v3, 2026-09-12 (the four definitions below resolved by the paper side; district and the measured batch curves still open). Spec from the paper side (fig:metro, Figure 14
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

## Definitions resolved by the paper side (2026-09-12 late)

The generator STIPULATES two of the four (freshness and p95 are constants
with noise, not computed), so the replay's choice becomes the paper's; the
paper side writes that up when the data lands.

1. Freshness. Age at use Delta_use = t_c - t_o (paper 5.1): t_o the timestamp
   of the newest observation the forecast used, t_c the cycle it is consumed;
   within limit iff Delta_use < tau. tau = 300 ms (the tighter measured limit,
   acceleration; 400 ms overtake carried as a secondary column). Population:
   one consumption per owning-locale-cycle per tracked road user (no CAV
   consumers are modelled; stated in the output). Per arm: warm, replall and
   delta_ready produce a forecast every cycle at every locale holding the
   track (owner or replica), so age = cycle length + import/refresh latency;
   delta_state produces no forecast at a replica before the crossing and the
   first destination forecast lands one prediction cycle after it, so the
   consumptions in that window are late by construction (the generator's
   0.79 encodes state-ready, not forecast-ready); reactive has nothing at the
   destination until crossing + transfer + import.
2. Locale sweep. fig_metro plots both panels at 100 locales only; the
   {16, 36, 64} rows are CSV-only and mean the SAME district partitioned
   coarser (one fixed area cut into L locales, track count held), never
   nested sub-windows. The figure needs the 100-locale partition; coarser
   partitions are optional CSV rows.
3. p95. Over all (locale, cycle) pairs POOLED at a given track count (the
   hot-locale tail the generator's docstring names); per-locale max as a
   secondary column.
4. Batch curves. The appendix C numbers are generated placeholders
   (total_ms = FIXED + PER x batch; FIXED create 0.9, delta 0.25, predict
   8.0; PER create 3.1, delta 0.15, predict 2.5; batches {1,2,4,8,16,32}).
   figure_rules pins only the ordering (create per record > predict per
   record > delta). STEP 1 of experiment 2 is therefore a MEASURED
   microbenchmark of the three operations at batches 1 to 32 on the real
   paths: record import (export + serialize + deserialize + import on the
   tracker), delta apply (newest observation onto an installed replica), and
   prediction (the MTR call; batch 1 already anchored by the campaign's
   36 ms per call on atlas). Import and delta are CPU paths; the prediction
   sweep needs the GPU and waits for the campaign to finish. The replay
   charges the measured curve and reports the generator's fits beside it;
   a non-linear predict curve is a finding, not a defect.

Build order update: (0) microbenchmark harness (import/delta now on CPU,
predict after the campaign); (1) district partition at 100 locales; (2)
SUMO demand and FCD; (3) engine; (4) lander; (5) smoke at 200 tracks, five
arms; output to the paper side before the sweep.

## District: pNEUMA (Athens), chosen 2026-09-12 under Tyler's criterion "find one based on real data"

Selection criterion (paper side, in order): real vehicle trajectories with
CONTINUOUS coverage across a district so a locale crossing is an observed
event; a real road network; enough area for a 300 m partition; public and
citable. pNEUMA (open-traffic.epfl.ch, Zenodo 10491409, CC BY 4.0): 1.3 km2
of central Athens, >100 km of lanes, ~100 intersections, 10 drones hovering
simultaneously (continuous district coverage; major arterials fully, some
minor streets partially), five weekday mornings x 2.5 h in 30 min slots,
>0.5 M trajectories, CSV per drone/date/slot plus merged all-drone files,
one row per vehicle with lat/lon, speed, accelerations every 0.04 s.
Rejected: V2X-Seq (KB: zero observations within 50 m of candidate
boundaries; intersection-centric sensing), inD/rounD (site-level), nuPlan
(fallback only, not cited). Traffic source is therefore REAL for the
calibration; SUMO is only the tool that extends the Athens network to the
100-locale point with demand calibrated to pNEUMA's measured statistics,
stated plainly.

Measured on pNEUMA at the 300 m partition (replaces the generator's
constants; reported side by side): crossings per track per second (0.013),
near-boundary fraction (0.22), neighbour degree at L (2.6 + 0.4 ln(L/16+1)),
junction degree distribution, per-boundary crossings per minute, and
per-boundary observation coverage (fraction of the boundary band with
trajectories, so an unobserved minor street cannot read as a zero crossing
rate). Vehicle classes: cars, taxis, buses, trucks are tracked road users;
powered two-wheelers reported separately (lane-splitting inflates crossings).

Tooling: conda env metro_env (python 3.10; carla 0.9.15 from PyPI; eclipse-
sumo 1.27.1 with `SUMO_HOME=$CONDA_PREFIX/lib/python3.10/site-packages/sumo`
and its bin on PATH; sumolib, traci, shapely, numpy, lxml). opencda310
untouched. The OSM -> Osm2Odr -> carla.Map -> junction survey pipeline was
exercised on a Midtown Atlanta extract (scratchpad metro/district_survey.py):
685 OSM junction nodes in 3 x 3 km, degree histogram {1:93, 2:148, 3:265,
4:173, 5:5, 6:1}, median inter-junction road 206 m; that extract is not the
district (no real trajectories) and is kept only as the pipeline test.

Tyler clarification (2026-09-12 late): "or a dataset". The real trajectory
dataset IS the district: replay the partition and the five arms DIRECTLY on
pNEUMA's trajectories, no SUMO in the loop. SUMO is used only if the
100-locale point is not reachable on the dataset's own area, and then only
as a calibrated extension of the same real network, marked as extrapolation.
Open until the survey lands: locale count depends on the partition rule
(per-junction Q7 anchors give ~100 locales of ~100 m on central Athens;
300 m grouping gives 12-16); both partitions are reported with their
calibration statistics and the paper side picks the figure's rows. The
x axis comes from subsampling a slot's real active tracks (200 to 3200,
ceiling = the slot's full count), never from generated demand.

Paper-side additions (2026-09-12, pNEUMA approved): (1) CADENCE: subsample
every trajectory to the 0.2 s edge-cycle grid BEFORE partitioning and
counting, so a crossing is a cycle-granular event and at most one crossing
per track per cycle (lane-splitting and jitter cannot double-count); report
the crossing rate at 25 Hz once beside the 5 Hz value. (2) X-AXIS: the real
point's x is the instantaneous active-track count (vehicles present in the
partitioned area per cycle, median and p95 over the session) for cars,
taxis, buses and trucks; also vehicles per km of lane; pNEUMA is a morning
peak and the paper says so. Any SUMO extension sweeps active tracks up to
3200 with demand scaled from that measured density.

CORRECTION (paper side, 2026-09-12 late): the generator's locale sweep is
FIXED locale size (~300 m) over a GROWING area, not one area cut finer
(near-boundary fraction constant at 0.22 across L; degree rising toward the
interior value is the boundary effect). fig:metro's 100-locale point is 100
locales of ~300 m, about 9 km2. Consequences for pNEUMA (1.3 km2):
- 300 m partition (12 to 16 locales) = the MEASURED corridor-scale point of
  fig:metro, x = subsampled active tracks (ceiling = the slot's full count).
- per-junction partition (~100 locales of ~100 m) is a different regime
  (below the paper's practical minimum, 2.1 and 5.11) and becomes the first
  REAL row of fig:sizing (Figure 16, 5.11: small locales push handoff
  frequency and import load past the cycle slack); report crossings per
  track per second and import load per cycle there.
- fig:metro's 100-locale point: Tyler's choice between (a) a SUMO extension
  of the same real Athens network to ~9 km2 with demand calibrated to
  pNEUMA's measured density, labelled calibrated extrapolation, or (b) the
  paper restates its deployment-scale claim at the real L. Until he
  answers: measure and report BOTH partitions; do not build the extension.
The survey script (scratchpad metro/pneuma_survey.py) already produces the
per-junction level and k-means groupings; the 300 m level is added as a
fixed-size tiling of the covered area (not k-means over anchors), so the
locale size, not the count, is what is held.

## SCOPE NARROWED (paper side, Tyler's steer, 2026-09-12 ~23:10): match the generated figure exactly

TARGET = fig:metro as generated: 100 locales of ~300 m (~9 km2), active
tracks {200, 400, 800, 1600, 3200}, five arms (warm, replall, delta_state,
delta_ready, reactive), y = p95 destination load as a fraction of the 70 ms
slack and within-limit fraction. Same axes, every input measured.
DROPPED (paper-side additions, not Tyler's ask): the per-junction ~100 m
partition, the fig:sizing row, the two-partition survey, option (b).
KEEP, in order:
1. pNEUMA: one all-drone slot, fixed-size 300 m tiling, 5 Hz subsample,
   cars+taxis+buses+trucks; measure crossings per track per second (gen
   0.013), near-boundary fraction (gen 0.22), neighbour degree (gen
   2.6 + 0.4 ln(L/16+1)), vehicles per lane-km, instantaneous active tracks.
   That is the whole survey.
2. Per-operation costs at batches 1 to 32 on the real import (create),
   delta-apply and predict paths, replacing gen_batchcost's linear
   placeholders; predict anchored at the campaign's 36 ms per call at batch 1.
3. HELD until the paper side confirms option (a): SUMO on the Athens OSM
   network extended to ~9 km2 -> 100 locales at 300 m under the Q7 rule,
   demand scaled from the measured pNEUMA density so active tracks reach
   3200, five arms charged with the measured curve; report the extension's
   crossing rate / near-boundary / degree against pNEUMA's and the
   generator's so the calibration is shown.
4. compare_measured.py against frozengen_metro_rows.csv.

Step 2 detail (2026-09-12 late, paper side agreed): delta-apply = deserialize
of the keyed update + the tracklet's own observation update on the replica
(appendix C's "applying a keyed update to one replica"); no separate
replication path exists in the runner and none is claimed. Device: the
campaign runtime places the Mamba tracker on cuda:0 (wrapper default when
available; edge manager passes 'cuda') and the predictor on cuda, so create,
delta and predict are measured TOGETHER after the campaign, in one process,
with the tracker and predictor constructed as the edge manager constructs
them, from mb_wt (freeze base 71c9f37e + the freeze-1p overlay). Harness:
scripts/metro/microbench_ops.py; runner scripts/metro/run_microbench.sh
(refuses to run while a simulation or CARLA is alive or load > 4). The
device and host load are recorded beside the curve. Survey and district
scripts: scripts/metro/pneuma_survey.py, district_survey.py.

Step 3 RELEASED (Tyler confirmed option (a), 2026-09-12 late): Athens OSM network extended to ~9 km2 so the Q7 partition gives 100 locales at 300 m, SUMO demand scaled from the pNEUMA-measured vehicles per lane-km so active tracks reach 3200, five arms charged with the measured appendix C curves; the extension's own crossing rate, near-boundary fraction and neighbour degree reported beside pNEUMA's and the generator's. Order: step 1 (pNEUMA slot) -> step 2 (microbench after the campaign) -> step 3 -> compare_measured.py vs frozengen_metro_rows.csv.

Partition implementation (2026-09-12 late): ONE function for steps 1 and 3,
the Q7 rule on a SUMO network built by netconvert from the OSM extract
(--geometry.remove so way splits are not nodes; --junctions.join 20 m so a
dual-carriageway crossing is one anchor; roads merged across pass-through
nodes; boundary at each inter-anchor road midpoint; fixed-size tiling of
anchors for the 300 m locales). scripts/metro/pneuma_survey.py. Checks run
before any pNEUMA data: (a) invalidation 4, Town03 through netconvert's
OpenDRIVE import: 41 inter-anchor roads (Q7 committed 40), median
separation 21.1 m (Q7 21-25 m), 25 anchors vs Q7's 31 because the 20 m
junction join merges Town03's paired dual-carriageway junction nodes (see
the no-join rerun in the KB); (b) pipeline test on the Midtown Atlanta
extract (no trajectories, not the district): 3 x 3 km, 425 anchors, 669
roads, separation median 50.7 m; at the 300 m tiling L = 97 locales with
mean neighbour degree 3.33 against the generator's 2.6 + 0.4 ln(97/16 + 1)
= 3.38. A 3 x 3 km area at 300 m gives the figure's 100 locales, and the
generator's degree law is what fixed-size tiling produces on a real grid.

Invalidation condition 4, PINNED (paper side, 2026-09-12 late): the check is
the boundary-to-conflict SEPARATION MEDIAN and the INTER-ANCHOR ROAD COUNT,
not the anchor count. The anchor count depends on node representation, and
netconvert's 20 m junction join is an intended normalisation: a
dual-carriageway crossing is one conflict zone, which is what Athens needs.
Reproduction target verified: the repository's locale_partition.py code run
UNCHANGED on the client-side carla.Map reproduces the committed Q7 numbers
exactly (Town03: 31 junctions, 40 roads, median 21.0 m; Town01: 12, 26,
23.5 m), so the target is the committed table. The netconvert partition on
Town03: 41 roads, median 21.1 m, with the differing road named in the KB.

Extended Athens network (3.5 x 3.5 km OSM box around the pNEUMA centre,
network only): separation median 23.8 m (the CARLA towns' 21 to 25 m band,
a real-city confirmation of the Q7 separation regime), lane-km 361; at the
300 m tiling the mean neighbour degree is 3.9 to 4.1 on every window from
2.4 to 3.5 km, against the generator's 3.2 to 3.5, so on the Athens grid the
degree constant sits about 15 percent above the law (Midtown landed on it).
Window for the figure's 100 locales: 2.8 km (L = 97) or 3.0 km (L = 95);
the exact L is reported, never rounded to 100.

Closed and pre-registered (paper side, 2026-09-13 ~00:20):
- Invalidation 4 closed as reproduced with identified differences: the three
  named merges (Q7 roads 23+22, and 52 and 60 into their chains) are the
  rule's text ("one boundary per inter-junction road") applied to a network
  representation, not losses; the 4 m minimum is Q7's own.
- Window 2.8 km, L = 97 (matches Midtown's L = 97, close to 100); the exact L
  is printed on the figure.
- Athens degree 3.9 to 4.1 against the law's 3.2 to 3.5 is a real property of
  a dense irregular grid (Midtown's regular lattice landed on the law) and is
  NOT tuned away: the replay uses the measured Athens degree and reports it
  beside the law and beside Midtown. Predicted direction, stated before the
  run: replicas per locale = near-boundary fraction x degree / L, so the
  three replication arms' destination load lands about 15 percent ABOVE the
  generated curves and warm stays where it is (its cost follows crossings);
  Khonsu's relative advantage widens. compare_measured.py showing that shape
  means the degree constant moved and nothing else; anything else means
  something else moved and is looked for.
- The 23.8 m Athens separation median (3.5 x 3.5 km window) earns one
  sentence in the deployability section: a real city inside the CARLA
  towns' 21 to 25 m band.
Tooling: scripts/metro/sumo_extension.py (crop to the window, randomTrips
+ duarouter, calibration run to set the departure period for a target
active-track count, 0.2 s FCD, meta JSON labelled calibrated extrapolation);
dry run on the cropped network in progress to validate the pipeline and the
replay engine end to end (generator costs, flagged; not a result).

