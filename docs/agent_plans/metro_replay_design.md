# Metropolitan Replay Design (NSDI experiment 2 of 4)

Status: DRAFT 2026-09-12, awaiting the paper side's claim/figure spec
(question sent; the open items are marked OPEN). Nothing built. Order after
the branching harness (experiment 1), whose k3 smoke is queued on atlas.

## Purpose

Q7 (deployability) measured boundary geometry on CARLA towns and left two
things queued: the same rule on real city geometry (an OpenStreetMap
district) and per-boundary crossing rates from traffic flow, which tie the
geometry study to load. This experiment supplies both and adds the cost
comparison the reviewers asked for at city scale: per-actor predictive
migration (Khonsu) against replication baselines, measured as bytes and
messages per locale per second and as readiness at the crossing, under
replayed metropolitan traffic. CARLA cannot run a district with hundreds of
vehicles and one edge stack per locale, so the replay is trace-driven and
offline; the closed-loop claims stay with the campaign and the branching
harness.

## Method

1. Map: an OSM district extract converted to OpenDRIVE with carla.Osm2Odr
   (client side, no server) and to a SUMO network with netconvert. The
   partition rule of Q7 (locales anchored on junctions, boundaries at
   inter-junction midpoints) is applied to the OpenDRIVE road graph with the
   same code path as locale_partition.py, moved to the client-side carla.Map
   so no CARLA server is needed. OPEN: which district (candidate: a 2 x 2 km
   grid-like urban district with arterials and residential streets so both
   Q7 speed classes appear; the junction degree distribution is reported
   next to the CARLA towns' so the reader can see how it differs).
2. Traffic: SUMO on the district with randomTrips/duarouter at a set of
   demand levels (vehicles per hour per km of road), floating car data at
   20 Hz (position, speed, heading, lane). One trace per (demand, seed) is
   the input to EVERY arm (invalidation condition 1).
3. Replay: an offline engine walks each trace at 20 Hz through the locale
   registry (LocaleRouter as in the runner), keeps per-actor sticky locale
   assignment, and runs each arm's trigger on the actor's constant-velocity
   projection (the runner's computed L: transfer EMA + fold-in + margin,
   capped) and each arm's transfer accounting. Per-transfer bytes and time
   are the campaign's measured record cost (HANDOFFROW bytes, TransferCost
   total_ms distribution), not modeled constants. Readiness per crossing:
   prepared at the entered locale with first-import time (prepare tick +
   measured transfer time + fold-in) before the crossing tick; wrong
   destination and late prepare decomposed as in the branching READYROW.
   No perception and no MTR in the replay: destination inference is CV on
   the trace (the branching harness carries the MTR destination question in
   closed loop; the replay reports how often CV picks the wrong branch at
   each junction degree, which is the link between the two).
4. Load and geometry outputs: per boundary crossings per minute, per locale
   concurrent tracked actors, prepares per second, imports per second,
   bytes per second in and out; the Q7 separation table on the district.

## Arms (placement and timing policy varies; the record does not)

| Arm | Rule in the replay |
|---|---|
| Khonsu | per actor, computed trigger, one prepare to the CV-projected destination, final update at the crossing |
| Replicate-to-neighbours | per actor, same trigger tick, one prepare per adjacent locale |
| EdgeWarp | per actor, pre-copy at predicted attachment + final sync at the crossing (edgewarp_full timing) |
| Reactive | per actor, one transfer at the crossing, no lead |
| Batched replication (period T_b) | every T_b each locale sends ONE message per neighbour carrying ALL its current track records; cost = per-message header + N x record bytes, serialized once per message; readiness = a crossing is ready if the last batch before it left at least transfer + fold-in before the crossing |
| Cold | nothing sent; readiness by re-acquisition (Q7 T_recover) |

"Genuinely batch" (paper side): the batched baseline must pay one message
per neighbour per period with N records inside, never N per-record
messages, and its record bytes must be the same measured record as Khonsu's.
Its readiness improves as T_b shrinks and its bytes/s grows as N x k / T_b;
the sweep over T_b is what puts it on the same axes as Khonsu. OPEN: T_b
sweep values (proposal {0.2, 0.5, 1, 2, 5} s) and whether an event-driven
batched variant (send when a batch fills) is wanted.

## Figures (OPEN: which the paper takes and their axes)

- Cost vs load: bytes/s and messages/s per locale against demand, one line
  per arm (batched at each T_b), city-scale replay.
- Readiness vs cost: readiness fraction against bytes/s per locale, points
  per arm and T_b (the batched frontier against Khonsu's point).
- Crossing rate vs separation per boundary on the district (feeds Q7's
  "crossing rates tie to load").
- CV wrong-destination rate against junction degree on the district (the
  link to the branching harness).

## Invalidation conditions (checked by the lander, not by eye)

1. Trajectory source differs across arms: every arm replays the identical
   trace file (hash of the FCD file logged in every run row).
2. Record cost not measured: per-transfer bytes and time must come from the
   campaign's HANDOFFROW / TransferCost samples (file and its hash logged);
   a run with modeled constants is flagged.
3. Batched baseline not batching: every batch message must carry N >= 1
   records with one header; a lander check counts messages against
   period x neighbours and flags any per-record emission.
4. Locale rule differs from Q7: the partition is produced by the same
   function on the district and on Town03, and the Town03 result must
   reproduce the committed locale_partition.csv numbers.
5. Readiness measured against modeled crossings: the crossing tick is the
   trace's own entry into the destination locale polygon, per actor.
6. District unlike the towns without saying so: the junction degree and
   separation distributions of the district are reported next to Town01
   and Town03 in the same table.

## Tooling and build order

- Install eclipse-sumo into opencda310 with pip (user level, no root; OPEN
  unless Tyler objects). carla.Osm2Odr is present. netconvert from the SUMO
  package; the repo's scripts/netconvert_carla.py converts OpenDRIVE to a
  SUMO net with the CARLA traffic-light landmarks.
- Build: (1) district extract + OpenDRIVE + SUMO net + partition (offline,
  CPU only, can run during the campaign); (2) SUMO demand generation and FCD
  export at three demand levels x 5 seeds; (3) replay engine reusing
  LocaleRouter/Locale and the runner's trigger code paths; (4) lander with
  the six checks; (5) smoke: one demand level, one seed, Khonsu + batched
  (T_b = 1 s) + reactive, output to the paper side before the sweep.
- Cost: CPU only. The replay of a 20 Hz trace with ~500 actors over 10 min
  is minutes per arm. No host contention with the campaign.
