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

FRESHNESS AXIS DECIDED (paper side, 2026-09-13 ~00:55): the figure's right
axis is the fraction of consumptions within the age limit in the W = 10
cycle (2 s) window after a locale crossing. W = 2 s on principle, not by
fit: it is the window the paper already identifies as where destination
state quality matters (about 2 s rebuilds the ten-frame predictor input,
S1/S2.4; the branching conflicts sit about 2 s after entry). Stated in the
caption. W = 5 and W = 15 cycles are CSV sensitivity columns, not on the
figure. The pooled owner-cycle number (~0.98 for every arm) stays in the
text beside the windowed one. The windowed measurement replaces the
generator's stipulated within values and is not tuned to them. The
corridor's within-limit numbers are per crossing for the same reason.
Window widened to 3.0 km so L lands at 97 to 100 after netconvert's
boundary drop; exact L printed.

## Step 1 MEASURED (pNEUMA 2018-10-24 09:00-09:30, all drones, 2026-09-13 ~00:40)

Population 10,402 trajectories: 4,509 cars, 1,645 taxis, 248 buses, 586
medium/heavy, 3,411 motorcycles (kept separate), 3 others. Alignment to the
network: median 3.0 m, 91% within 10 m. Covered network (50 m pad): 482
anchors, 791 inter-anchor roads, 91 lane-km, separation median 25.3 m. At
300 m: L = 36, 214 boundaries. Measured constants (generator in brackets):
crossings per track-second 0.0123 at 5 Hz, 0.0124 at 25 Hz (0.013);
near-boundary fraction 0.208 / 0.318 / 0.548 within 20 / 30 / 50 m (0.22,
which matches a 21 m band; proposal nb = 20 m); neighbour degree 3.72 at
L = 36 (law 3.07), the same +15 to +20% as the extended Athens network.
x coordinate: active tracks median 1,359, p95 1,468; 14.9 vehicles per
lane-km; morning peak. Motorcycles: 0.0181 crossings per track-second,
0.217 near 30 m. Guard: 40% of boundaries had a tracked vehicle within 30 m
in the slot (29% saw a crossing; 6.2 crossings/min median on those); the
crossing rate is per observed track-second so empty boundaries do not bias
it; degree restricted to observed boundaries to be reported as a second
number. JSON: evaluation_outputs/metro/pneuma_20181024_0900_pad50_survey.json
(and the 200 m-pad run, L = 48, 0.0112, 0.160/0.248/0.460, degree 4.21).

Step 1 across the five morning slots of 2018-10-24 (0830 to 1100, same
network and tiling, L = 36): crossings per track-second 0.0148 / 0.0123 /
0.0121 / 0.0123 / 0.0122 (median 0.0123; generator 0.013); near-boundary
fraction at 20 m 0.207 / 0.209 / 0.196 / 0.176 / 0.180 (median 0.196;
generator 0.22, nb = 20 m is the corridor's own band); active tracks median
1181 / 1359 / 1418 / 1356 / 1259 (median 1356) and 13.0 / 14.9 / 15.6 /
14.9 / 13.8 vehicles per lane-km (median 14.9; the real point's density,
38 tracks per 300 m locale); neighbour degree 3.72 over all boundaries and
3.09 / 3.00 / 2.87 / 2.87 / 2.91 over observed boundaries (median 2.91,
on the law's 3.07); 40 to 43 percent of boundaries observed per slot.
JSONs: evaluation_outputs/metro/pneuma_20181024_<slot>_survey.json.

CORRECTION on the degree (paper side, 2026-09-13 ~02:00): the +21 percent
attributed to "dense irregular grid" was a coverage artifact. Restricted to
OBSERVED boundaries the pNEUMA degree is 3.09 / 3.00 / 2.87 / 2.87 / 2.91
across the morning, on the law's 3.07 at L = 36; the all-boundaries 3.72
counts side streets the drones do not observe and that carry no tracks.
The generator's degree law HOLDS on real peak traffic. Consequences:
1. The measured pNEUMA point uses the degree that falls out of the crossings
   themselves: the engine's replication neighbours are, by default, the
   locales across boundaries that carry traffic in the trace (--nbr_mode
   observed); 3.72 is reported beside it as the topological degree with the
   coverage fraction. Near-boundary band nb = 20 m (the corridor's band).
2. The SUMO extension routes demand onto every street, so its effective
   degree is near the topological 3.9 to 4.1, a real and stated difference,
   not an error. PREDICTION, rewritten before step 3 runs: the +15 percent on
   the three replication arms (warm unchanged) applies to the EXTENSION
   curves only; the measured pNEUMA point should land on the generated curve
   for all arms.
3. The figure/CSV report the full slot spread: crossings 0.0121 to 0.0148
   (median 0.0123 vs 0.013), near-boundary at 20 m 0.176 to 0.209 (vs 0.22,
   drifting down late morning), active tracks 1181 to 1418, 13.0 to 15.6
   vehicles per lane-km, coverage 40 to 43 percent. Two of three constants
   match within the slot spread and the third matches once coverage is
   accounted for.
Retroactive fact for the plan: the freeze-1n batched block scores 0.498 on
the mode-diversity gate (its single-track calls collapsed), so its
mode-derived columns were half collapsed; goes beside the pre-cap caveat.

Engine on the real pNEUMA 0900 slot (2026-09-13, placeholder costs, flagged): freshness columns are cost-independent and stand (post-crossing 2 s window: warm 99.8, replall 99.8, delta_ready 99.8, delta_state 89.7, reactive 79.8 percent; pooled 99.1-99.5); load columns wait for step 2's measured curves. Step 2 (microbench) can run on the now-idle GPU once the predictor fix shape is decided, because the padded isolated call changes the runtime's prediction call pattern (cost(2) + cost(N-1) under cap=1), not the per-batch curve.

METRO STEP 2 MEASURED (2026-09-12 23:36 EDT, cuda:0, idle GPU during the T25 pause, load 7.4 caveat; evaluation_outputs/metro_microbench/microbench_20260912_233553.json): create 0.19 ms (N=1) .. 2.06 ms (N=32), fit 0.27 + 0.057/record (placeholder 0.9 + 3.1); delta 0.012 .. 0.17 ms, fit 0.007 + 0.005/record (placeholder 0.25 + 0.15); predict 32.7 (N=1), 35.2 (2), 37.8 (4), 41.0 (8), 66.5 (16), 225 (32) ms: flat to 8 then superlinear, the linear fit 9.6 + 6.0/record misdescribes it; the replay interpolates the measured points. figure_rules' pinned ordering create/record > predict/record FAILS (0.06-0.19 vs 2-7 ms); surfaced, not reconciled. Padding the isolated call (N=1 -> 2) costs 2.5 ms. Engine now charges predict(2) for padded isolated calls (--pad_isolated 1); replay of the real slot with the measured curve running.

Paper-side reading of the ordering-rule failure (2026-09-13, for the record): the pinned "create per record > predict per record" encoded the generator's artifact (a 3.1 ms import placeholder), not the paper's claim; the paper argues prediction dominates and the measurement strengthens that by two orders of magnitude (0.06-0.19 ms/record import vs 2-7 ms/record prediction); appendix C's order of magnitude between delta and create survives (0.005 vs 0.057). Correcting the rule to the paper's actual claim (predict > create > delta per record) is a rule change that waits for Tyler; figure_rules is not edited from this side. Predict is charged by interpolation over the measured points (the knee above N=16 is real: a 30-replica locale pays ~200 ms of a 70 ms slack) and reported as points, not a fit, in the appendix table. The load-7.4 caveat stays on the current JSON until a quiet-moment rerun of the create/delta CPU rows; predict is GPU-bound and stands.

DESTINATION LOAD = MARGINAL (paper side, 2026-09-13 ~00:00 EDT; the figure's definition because it is what the runtime does): a replica or prepared track JOINS the destination's per-cycle prediction pass; under cap=1 the owned pass is predict(2) for the isolated+pad track plus predict(N_owned-1) for the batched remainder, added tracks join the remainder, so marginal = predict(N_owned-1+N_added) - predict(N_owned-1) (the pad belongs to the owned pass, charged to no scheme; a locale with no owned tracks runs its pass only for the replicas). Standalone kept as a CSV column, never on the figure. N_owned per locale-cycle and owned+added batch sizes reported beside the load. PREDICTED before the rerun: Khonsu drops from 97% to low single digits of slack (below the generator's 18%, which charged an 8 ms base per prepare batch); delta_state ~1%; delta_ready and replall come down from ~900% but stay far over the slack (~23 replicas push the remainder past the N=16 knee); ordering intact, Khonsu's advantage wider than generated; replall or delta_ready under the slack would mean a composition error. Paper side owns the 5.9 sentence and the abstract's 18%. Cost file for the replay: evaluation_outputs/metro_microbench/microbench_merged.json (predict rows from the 23:35 GPU run, create/delta from the quiet 23:57 rerun: create 0.10 ms at N=1 / 1.97 at 32, delta 0.011 / 0.17).

MARGINAL REPLAY ON THE REAL SLOT (2026-09-13 ~00:05 EDT, measured merged costs, cap=1+pad, L=36; pre-fix on the trigger side): N_owned per locale-cycle p50 55 / p95 190 / max 272. warm 25.9% of slack marginal (standalone 97.1; batches owned+added p50/p95 83/227; 2 s window 99.8), replall 854% (910), delta_state 0.9% (window 89.7), delta_ready 847% (902), reactive 0.1% (window 79.8). Against the pre-registered prediction: ordering and the replication arms as predicted; Khonsu 26% not single digits because the owned batches sit past the N=16 knee (a prepared track costs the linear-part ~6 ms/record there, not the flat-region ~1 ms). FINDING: at pNEUMA peak density on 300 m locales the destination's own pass (predict(54) at the median, ~330 ms by the linear fit) exceeds the 200 ms cycle before any replication; the 70 ms slack framing assumes the 130 ms allocation covers the owned pass, which it does not at this density (the generator's 32 tracks x 2.5 ms hid it). CAVEAT: batches above N=32 were charged on the linear extrapolation; the predict microbench is being extended to N=64/128/256. JSONs: evaluation_outputs/metro/pneuma_0900_replay_measured{,_marginal}.json.

ADMISSION PROVENANCE (2026-09-13 ~00:30 EDT, from the code the campaign ran): adaptive_controller.deadline_ms = 130 is the deployed allocation (WorldFusionAdaptiveEdge in edge_manager_worldfusion_ab3dmot_mtr_adaptive.py: line 79 reads it; 146-152 pick k_cav so pre-prediction fits deadline - 30 ms reserve; 193-194 set predictor.budget_ms = max(5, deadline - pre_pred_ms) EVERY tick, ~92 ms after ~38 ms fusion+tracking; 213-222 halve the tick rate on a p95 overrun; every campaign log opens with "causal controller (deadline=130.0ms, ...)"). The yaml's mtr_predictor.budget_ms = 50 is a constructor default overwritten per tick. The MTR stage admits max_tracks = int((budget - 25) / avg_ms_per_track) (overlay _risk_budget_select 497-525), avg = mean of (elapsed - 25)/n over the last 50 calls, only when > 2 tracks diverge. Under cap=1 isolation every call is n=1, so the estimate is 14 ms/track (atlas FDE), 21 (T25 v8), 42 (cetus corridor): K = 4 / 3 / 1, not 16; never bound on the atlas flow blocks (ISOROW N <= 2), almost certainly bound on the corridor. Under the 1q pad (n=2 isolated calls) the estimate is ~5 ms/track -> K ~13. Three K values on the table for the figure (16 curve capacity, 3-4 deployed under isolation, ~13 under the pad); paper side decides; the estimator reading the measured curve instead of the linear model is a candidate one-line pipeline change (stated if taken).

ADMISSION, VERIFIED DEPLOYMENT (2026-09-13 ~00:45 EDT): WorldFusionMambaAdaptiveEdge does not override run_step; WorldFusionAdaptiveEdge.run_step (repo file, not overlaid) is every campaign tick: compute_ms is measured around the WHOLE pass including prediction (line 168) and predictor.budget_ms = max(5, 130 - compute_ms) is set AFTER it (193-194), i.e. the NEXT tick's prediction budget = 130 - (fusion + tracking + prediction of this tick), a feedback value of ~15-55 ms under cap=1's two-call pattern; the yaml's 50 is an overwritten default (paper side's "50 ms sub-budget" refinement WITHDRAWN; 5.1 stands: 130 ms deployed pass deadline, the pipeline selects how many tracks receive full prediction). Iterating the runtime's rule (K = max(int((budget - 25)/avg), 1), avg = (elapsed - 25)/n over issued calls) on the measured curve gives a 2-cycle K in {1, 2} with the pad and {1, 3} without: the deployed admission self-limits to 1-3 because the two-call pattern plus the stale 25 ms base eats the deadline. Never bound on the atlas flow blocks (<= 2 mature tracks); at pNEUMA's 55 owned tracks 1-3 per cycle would receive MTR. Two runtime constants flagged for Tyler, not changed: base_ms = 25 (measured floor 33.7) and the budget-after-prediction ordering. Paper side's K decision (crossed messages): the estimator's fixed point under the 1q pad with the actual batch mix = the figure's "deployed" row; the engine's --admit_rule deployed computes exactly that (run queued); 16 (curve capacity at 92 ms) and 3-4 (1p/1n isolation) as labelled rows. Estimator fix (read the measured curve) flagged as a candidate for Tyler, NOT for 1q (one pipeline change per rerun). Demand row on the extended curve (no admission, marginal): warm 19.8%, replall 735%, delta_state 0.9%, delta_ready 728%, reactive 0.1% of slack.

ADMISSION: SEQUENCE OF CORRECTIONS RECORDED IN ORDER (2026-09-13 ~00:55 EDT). (1) Paper side: the adaptive chain IS deployed, 130 ms is the pass deadline, the yaml's 50 is an overwritten default, the "50 ms sub-budget" refinement to 5.1 withdrawn. (2) Eval side refinement, accepted: the budget is set AFTER the whole pass, so budget_{t+1} = 130 - (38 + prediction_t) hovers 15-55 ms and the runtime's rule with base_ms = 25 oscillates K between 1 and 2 with the pad (1 and 3 without): K_deployed = 1-3 under cap=1. TWO RUNTIME DEFECTS, one line each, together under-admit ~5x against the GPU's capacity (16 at 92 ms on the measured curve): (a) base_ms = 25 vs the measured 33.7 floor + a linear estimator calibrated on isolated calls; (b) the next tick's budget computed from this tick's total INCLUDING prediction. At pNEUMA density the deployed pipeline gives MTR to 1-3 of 55 owned tracks per cycle, the rest fall to the linear fallback: the base pipeline's admission at peak, not Khonsu's. DECISION IS TYLER'S (paper side putting it to him): (i) run 1q as-is, figure shows K=1-3 deployed with 16 as capacity, paper states the 5x under-admission; or (ii) fix both constants for 1q (estimator reads the measured curve; budget from this tick's pre-prediction time), K ~13-16, two more stated pipeline changes beside the reweighting rule. NOTHING BUILT. Engine: --admit_rule deployed implements the feedback fixed point (pre-pred 38, base 25, estimator (total-25)/N on the issued batches, budget from the previous cycle, pad on) and reports the cycle; the figure's "deployed" row beside 16 and 9 (50 ms by the curve).

ADMISSION ROWS, REAL SLOT (2026-09-13 ~01:05 EDT, extended measured curve, marginal above the admitted pass): K=16 (92 ms capacity; admission ratio p50 0.29 / 5th pct 0.08): warm 20.1% of slack on top (batches 17/20, displacement 0.025 mean / 0.063 p95), replall 617% (batches 56/124, displacement 1.25 / 3.44), delta_state 0.9%, delta_ready 609%, reactive 0.1%. K=9 (50 ms by the curve; ratio 0.16 / 0.05): warm 16.6%, replall 636% (displacement 1.48 / 3.87), delta_ready 628%, others unchanged. Displacement > 1 = replicas alone exceed every admitted slot. Deployed-rule row (cycle {1,2}, K=2, ratio 0.04 / 0.01) running. JSONs: evaluation_outputs/metro/pneuma_0900_replay_admit{92,50}.json.

DEPLOYED-RULE ROW (2026-09-13 ~01:15 EDT; cycle {1,2}, K=2, admission ratio p50 0.04 / 5th pct 0.01): warm 13.4% of slack on top (batches 3/6, displacement 0.029/0.079), replall 611% (batches 43/110, displacement 1.68/3.89), delta_state 0.9%, delta_ready 603%, reactive 0.1%; freshness unchanged across K. All four rows (demand, K=16, K=9, K=2) assembled by scripts/metro/metro_rows.py into evaluation_outputs/metro/fig_metro_measured_rows.csv for compare_measured.py; trigger side flagged pre-fix until 1q.

COMPARATOR ON THE MEASURED POINT (2026-09-13 ~01:40 EDT; paper repo 481bd1e, compare_measured.py --axis tracks, read-only; output evaluation_outputs/metro/compare_measured_metro.txt; schema CSVs pneuma_0900_metro_rows_{demand,admit92,admit50,deployed}.csv). By-density bracket at L=36 (generated 800 = 22/locale, 1600 = 44/locale; measured 1359 = 38/locale), dest_load_p95 fraction of slack, measured demand/K16/K9/K2: warm gen 0.155-0.182 vs 0.198/0.201/0.166/0.134 (on the curve); replall gen 1.68-3.57 vs 7.35/6.17/6.36/6.11 (1.7-2x above); delta_ready gen 0.92-1.92 vs 7.28/6.09/6.28/6.03 (3-4x above); delta_state gen 0.046-0.081 vs 0.009 (5-9x below); reactive gen 0.004-0.008 vs 0.001. within_limit (2 s window) higher than the stipulated values for every arm, ordering preserved (warm 0.998, replall 0.998, delta_ready 0.998, delta_state 0.897, reactive 0.798). Reading: traffic statistics matched the generator, so the deviations are the cost curve (prediction 4-8 ms/record above the N=16 knee vs 2.5; delta 0.005 vs 0.15; import 0.06-0.19 vs 3.1). Generator's composition at L=36/1600 (replicas/locale 30.0, degree 3.07, 20.8 crossings/s) is near the measured one.

ENGINE CHARGING RULE (2026-09-13 ~01:10 EDT): the measured predict medians are not monotone at small N (43.0 ms at N=2 vs 38.8 at N=4, 5-rep noise on a flat region), so a marginal charge could go negative; charging now uses the MONOTONE ENVELOPE (cumulative max) of the measured points and clips the marginal at 0; the raw points remain the appendix table. Extension calibration: a 300 s calibration run undershoots ~2x on the 3 km network (target 400 gave 797 active); calibration now on full-length 900 s runs, up to 4 corrections, 10 percent acceptance, the accepted run is the trace (200 -> 198, 400 -> 405). The first two extension points' replays and the four pNEUMA rows are recomputed under the envelope at the end of the sweep so every row shares one charging rule.
