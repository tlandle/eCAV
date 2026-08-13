# Paper 3 draft section: locale design and deployability (Q7)

Draft prose for the paper, written to be lifted into the design and
evaluation sections. Figures: paper_partition_map.{pdf,png} and
paper_separation.{pdf,png} (scratchpad figs/, regenerate with
q7_paper_figs.py from locale_partition.py output).

## Design: locales and their boundaries

A locale is the region of the map that one edge serves. We anchor each
locale on a conflict zone: a junction, a merge point, or another road
segment where vehicle paths cross. The edge that owns a locale is
responsible for prediction at its conflict. Boundaries between locales
therefore never sit at a conflict. We place each boundary in the plain
stretch between two adjacent conflict zones, at the midpoint of the road
that connects them. This is the farthest point from both conflicts, so a
vehicle that crosses a boundary has the longest possible run before it
reaches the next conflict.

The distance from a boundary to its conflict determines whether a
migration protocol works there. When a tracked vehicle crosses into a new
locale, the destination edge must re-establish prediction state before
the vehicle reaches the conflict. Each protocol needs a recovery time
T_recover. A blocking transfer at the crossing followed by track
re-maturation needs about 2 s. Re-acquiring the track from scratch needs
about 4 s. Carrying the full tracker state ahead of the crossing needs
only the transfer itself, under 60 ms measured. At entry speed v, a
protocol can serve a boundary only if the separation D between the
boundary and the conflict satisfies D >= v * T_recover. We write the
minimum separation as D*.

This inverts the sizing question. With a reactive or cold protocol, D*
dictates how small a locale can be: at 14 m/s a reactive handoff needs
29 m of clear road after every boundary, and a cold start needs 56 m.
With predictive latent migration D* is under one meter, so locale size is
set by load and radio coverage instead of by the migration protocol.

## Evaluation: measured map geometry

We partition two CARLA towns with the rule above and measure every
boundary. Town01 has 12 conflict anchors and 26 inter-conflict roads;
Town03, the densest town, has 31 anchors and 40 roads. The median
boundary-to-conflict separation is 21 to 25 m.

The separation distribution answers the deployability question directly.
A boundary is deployable for a protocol if its separation exceeds that
protocol's D*. At residential speed (8.3 m/s), reactive migration can
serve 65 to 73 percent of boundaries and cold start 25 to 42 percent. At
arterial speed (14 m/s), reactive serves 33 percent and cold start 18
percent across both towns; on Town03 alone the shares fall to 27.5 and
12.5 percent. Predictive latent migration serves every boundary at both
speeds. Most of a real map's boundaries sit exactly in the separation
band where only predictive migration is safe.

Two caveats bound the claim. The recovery times for the baseline arms
come from our measured tracker maturation and transfer latencies, not
from a reimplementation of a full service-migration system. And CARLA
towns are small; an OpenStreetMap district extract (queued) tests the
rule on real city geometry, and per-boundary crossing rates from traffic
flow tie this study to the load evaluation (Q5).

## Figure captions (draft)

- Fig. partition_map: Town03 partitioned by the locale rule. Roads are
  colored by owning locale, dots mark conflict anchors, ticks mark the
  boundary at each inter-conflict midpoint.
- Fig. separation: Boundary-to-conflict separations across Town01 and
  Town03 (n=66) against the minimum separation D* each protocol needs at
  14 m/s. The median boundary (21-25 m) is below the reactive requirement
  (29 m); only predictive latent migration serves the full map.

## Notes for integration

- Numbers regenerate with: locale_partition.py --towns Town01 Town03,
  then q7_paper_figs.py. CSV: locale_partition.csv.
- T_recover constants and their provenance live in
  scale_out_evaluation.md Q7; keep them consistent with the protocol
  timeline figure (P2) when the live numbers replace projections.
- Style: no coined terms; "deployable" is used as a plain adjective with
  its condition stated inline.
