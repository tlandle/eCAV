# Branching-Junction Harness Design (freeze-1p follow-on, NSDI experiment 1 of 4)

Status: shape APPROVED by paper side 2026-09-12 (changes a-g applied). Survey done, harness BUILT (scripts/branching, scripts/atlas_branch.sh, scripts/khonsu_branch_land.py), NOT yet run: the k3 smoke waits for a free host. See the final sites section at the end; it supersedes the earlier site paragraphs.

## Purpose

Feeds the paper's two branching claims: readiness against destination certainty,
and speculative placement cost against junction degree, for six placement
policies. Also carries the predictor destination-confidence measurement (peer):
the distribution of MTR destination confidence on branching trajectories and a
calibration check, so the paper can shade the operating range of the deployed
predictor onto the synthetic sweep in Figure 8.

## Scenario

A junction whose outgoing roads define k branches, one locale per branch plus
one approach locale. A scripted road user approaches, commits to a branch late
(its route is fixed per seed but not inferable from geometry until the turn
begins), and meets a downstream conflict shortly after entry into whichever
branch it takes. The conflict is what makes readiness measurable: state must be
ready at the taken branch's locale before the recipient CAV's first use.

Junction degree k in {2, 3, 4, 5}. Map survey is build step 1:

- k=2: any fork / T-junction approached on the stem (Town01/02 have many).
- k=3: crossroads approached on one arm (Town01/03/05).
- k=4: full crossroads with U-turn permitted, or a 5-arm junction minus one
  (Town03/05).
- k=6: not available. The largest physical junction in the available maps
  (Town01-07,10) is Town03's roundabout (~5 arms). Decision (peer review):
  sweep {2,3,4,5} and say so. Bidirectional roundabout arms are NOT counted
  as distinct destinations: direction of travel is resolved from heading, so
  that would be a fabricated degree. Locales are never cut finer than the
  physical branches. The survey max k is reported to the paper side before
  any scenario is built; the paper side owns the Figure 9 x-axis change.

## Arms (six; placement decision is the ONLY thing that varies)

| Arm | Mechanism | Exists today? |
|---|---|---|
| Khonsu | computed trigger consuming multimodal MTR destination probabilities (per-mode boundary crossing, probability summed by destination; `last_mtr_modes` stash) | TRIGGER_MODE=computed + mtr consumption path exists |
| Oracle | knows the taken branch at trigger time | TRIGGER_MODE=oracle exists |
| Velocity projection | CV extrapolation picks the single projected branch | TRIGGER_MODE=predictive exists |
| Top-1 mode only | argmax MTR mode's branch, ignore the rest of the distribution. Shares Khonsu's lead computation L EXACTLY: same trigger tick, same L; only destination selection differs, never timing. Enforced in code, not by convention. | small variant of the mtr trigger (new flag) |
| EdgeWarp radio trigger | radio-level trigger, destination = the cell being entered | MIGRATION_MODE=edgewarp_full exists |
| Replicate-to-all | on trigger, send the identical record to ALL k branch locales | NEW arm (the brute-force comparison) |

Replicate-to-all build: a MIGRATION_MODE (or TRIGGER_MODE) variant that, at the
same trigger point as Khonsu's computed trigger, issues the same prepare to
every adjacent branch locale instead of the selected subset. Cost surfaces as
imports at k destinations; readiness is trivially met at the taken one. This is
deliberately unoptimised in DECISION but not in transport: it uses the same
migration path, serialization, and payload as every other arm.

## Fixed transferred state (the identity requirement)

The payload is the FULL record (warm) for all six arms. The arms differ only in
when/where prepares are issued, never in what is sent.

## What would make the comparison meaningless (invalidation conditions)

1. Non-identical transferred state across arms. Guard: HANDOFFROW already
   carries bytes=; add a payload content hash to the emit, computed over the
   SERIALIZED bytes that crossed the link, not the in-memory record. Per
   (seed, handoff), the hash must be IDENTICAL across all six arms or the run
   set is invalid. Checked by the lander, not by eye.
2. Non-identical scenes. Guard: the branching actor's route (including which
   branch it takes) is scripted per seed and identical across arms; paired
   spawn-phase seeds as in the flow scenario. Verify per (seed): the actor's
   trajectory polyline in the log matches across arms within CARLA jitter
   (existing host-effect method, dist/time distributions). An arm whose
   presence changes the actor's behaviour (e.g. replicate-to-all import load
   slowing the sim and shifting spawn phase) would silently break this; the
   trajectory check catches exactly that.
3. Readiness measured against different conflict timelines. Guard: readiness
   and lead time are computed against the taken branch's conflict tick from
   that run's own log (per-run realised, never nominal), same rule as the
   T_avail lander.
4. Destination confidence measured from a different model than the policies
   consume. Guard: CONFROW is emitted inside the same predictor instance the
   trigger reads (`last_mtr_modes`), not a side model.

## Metrics and logging (all lander-parsed, no by-eye)

- READYROW per handoff: taken branch, prepare tick per destination, commit
  tick, first-use tick at the taken destination, ready-before-first-use bool,
  lead time s, plus two failure-decomposition bools: wrong_destination (state
  was prepared, but not at the taken branch) and right_destination_too_late
  (prepared at the taken branch, but after first use). Readiness is the
  conjunction; the destination x time figure cannot be validated unless the
  lander can split them. Feeds readiness vs certainty.
- PLACEROW per handoff: k, set of locales prepared, count (speculative
  placement cost). Feeds placement vs junction degree. Replicate-to-all
  gives count=k by construction; Khonsu gives the probability-selected subset.
- CONFROW at the TRIGGER TICK (the confidence the policy actually consumed;
  this is the sample calibration is built on) and additionally at T-minus
  {2.0, 1.0, 0.5} s before branch entry: per-branch probability (mode scores
  summed by destination), argmax branch, taken branch. Feeds the distribution
  + calibration (fraction correct at each confidence bin). Also answers where
  on the 0.35-0.95 axis MTR actually operates.
- ISOROW/MTRTIME stay on (same predictor build as the campaign).

## Grid and cost

k in {2,3,4,5} x 6 arms x 10 seeds = 240 routes. At ~4-7 min/route: ~16-28 h
single host. Runs AFTER the current campaign completes; whichever host frees
first. Grid order is SEED-MAJOR (as atlas_t25_1m.sh) so an interrupted grid
leaves a complete coarse figure. Smoke first: one k=3 route, three arms:
Khonsu, replicate-to-all, AND Oracle. Oracle must be ~100% ready at k=3; if
it is not, the conflict timeline is wrong and nothing downstream measures the
trigger (the T25 tell: oracle below kf at 8 m/s). Smoke also verifies
READYROW/PLACEROW/CONFROW parse and the payload-hash identity across arms.
Smoke output goes to the paper side before the grid runs.

## Build order

1. Map survey (junction candidates per k; the k=6 answer). Report to peer.
2. Scenario files (yaml/xml/py) for the chosen junctions; scripted late-commit
   actor; conflict actor per branch.
3. Replicate-to-all arm + top-1 flag + payload hash in HANDOFFROW.
4. CONFROW emit in the predictor.
5. Smoke (step 5 gate: peer reviews smoke output before the grid).
6. Full grid + lander + report (distribution and calibration reported plainly,
   including if confidences cluster >=0.9 making the sweep a stress region).

## Map survey result (2026-09-12, OpenDRIVE parse of all loadable towns)

Method: per junction, count external non-junction roads; cluster their outward
headings at the junction end within 25 degrees so dual carriageways (two
OpenDRIVE roads ~8 m apart, same heading) count as one physical arm; exclude
roundabout ring connector stubs (0.2-2.4 m arcs). Scripts:
scratchpad junction_survey.py, junction_headings.py.

| Town | max physical arms | note |
|---|---|---|
| 01, 02 | 3 | T-junctions only |
| 04, 05, 06, 07, 10HD, 11, 12, 13 | 4 | crossroads |
| 15 | 3 | two nominal 5-arm junctions (877, 322) are dual carriageways |
| 03 | 5 | junction 861 at (-80,10): N, S, W, SW arms + 22 m E link into the roundabout. Nominal 6-arm junction 1469 is the roundabout cluster (ring stubs), disqualified |

Destinations per approach: 3 arms -> k=2, 4 arms -> k=3, 5 arms -> k=4.
k=5 needs a physical 6-arm junction; none exists. Sweep is {2, 3, 4}.
U-turn destinations are not used to reach k=5 (not a branch through traffic
would accept; same reasoning as the roundabout arms). k=4 site geometry: the
east branch enters the roundabout after 22 m, so its locale is short and its
downstream conflict sits at the roundabout entry yield; branch locales are not
symmetric at k=4 and the paper must say so.

Candidate sites (to be confirmed at scenario build): k=2 Town01 T-junction;
k=3 Town05 crossroads (flat, four symmetric arms); k=4 Town03 junction 861.
Waiting on the paper side's Figure 9 axis decision before building scenarios.

## Build decisions (2026-09-12, after the survey and code reads)

Sites (CARLA frame, polygons in scratchpad branching_sites.json):
- k=2: Town01 junction 87, approach on the 109 m stem (road 24), branches
  road 5 (70 m) and road 6 (224 m).
- k=3: Town05 junction 207, approach on the 88 m arm (road 41), branches
  roads 21 (54 m), 22 (46 m), 40 (40 m).
- k=4: Town03 junction 861, approach on the 114 m north arm (road 3),
  branches roads 4 (92 m), 62 (37 m), 44 (35 m), 61 (22 m, into the roundabout).
Approach locale = the arm rectangle extended 14 m through the junction box, so
the trigger's exit test fires at the branch mouths, not a junction-transit
early. Branch locales start at their own arm mouth, 35 m long. Conflict point
= 20 m after branch entry on every branch (19.7 m on the 22 m link).

Recipient CAV and conflict, uniform on every branch: one CAV per branch facing
the junction behind a stopped blocker vehicle, so it must overtake into the
NPC's lane at the conflict point. The NPC becomes the oncoming for whichever
branch it takes; the overtake gate, first-use bookkeeping, and the binary
collision metric carry over from the flow scenario unchanged. This makes the
conflict type identical across branches, including the roundabout-side east
branch at k=4 (removes the residual noted earlier), and matches T_avail by
construction. The branch is fixed per seed by the runner script (seed s takes
branch s mod k, so each branch is taken equally) and passed to both the
scenario and the runner as BRANCH_TAKEN.

Code shape (all as new overlay files under a branching/ workspace; the
freeze-1p campaign overlay is not edited because the queued atlas T25 block
copies it at start):
- runner_branch.py: candidate set per source locale (yaml locale.neighbors,
  default all others), per-arm fire set, prepares issued to every locale in
  the fire set, bookkeeping keyed (tick, src, prepared set), READYROW /
  PLACEROW / CONFROW (per tick while in the approach locale; the lander picks
  the trigger tick and the T-2/1/0.5 s samples), payload hash in HANDOFFROW.
- daemon_branch.py: cost.payload_hash = sha256 of the serialized bytes that
  crossed the link.
- sim_api overlay: vehicles bound to heroes by a GLOBAL index across edges
  (today a per-edge index binds hero[0] to every edge's first vehicle) and
  vehicle_count summed over edges. Behavior-preserving for every existing
  yaml (all of which list vehicles under edge 0 only).
- scenario_1_branch.py: NPC route via GlobalRoutePlanner to the taken branch,
  WaypointFollower(plan=...); k egos + k blockers from the XML.
GPU: each edge loads its own WorldFusion model; a 2-edge cell uses ~8.2 GB of
16 GB. k=3 (4 edges) is expected to fit; k=4 (5 edges) is measured at the k=3
smoke and, if it does not fit, WorldFusion weights get a shared cache like the
MTR one.

## Final sites and build (2026-09-12 late, supersedes the site paragraphs above)

Lane survey before building (OpenDRIVE lane sections + junction connection
tables; the arm count alone was not enough):

- The junction degree that matters is the number of destinations reachable
  from the actor's own approach lane without a lane change. On a dual
  carriageway the lane already tells the destination (inner lane: left or
  straight; outer: right or straight), so Town05 j207 (all arms 2 lanes per
  direction) is an effective k=2 from any one lane and was REJECTED for k3.
  Same standard at k4 (paper side): Town03 j861 approached from road 3 (dual)
  reaches {4,44,62} from the outer lane and {4,61} from the inner, never 4.
- Sites: k2 Town01 j87 approach road 24 (109 m, single lane), branches 5
  (70 m) and 6 (224 m). k3 Town04 j1452 approach road 18 (72 m, single lane,
  curved), branches 9 (50 m), 10 (36 m), 17 (57 m), all single lane, 3
  reachable from the one lane. k4 Town03 j861 approach road 62 (single lane,
  37 m straight, reaches 3, 4, 44, 61 from its one lane: a genuine k=4). Road
  62's run-in is preceded by a 90 degree turn at T-junction 1820; the actor
  spawns 65 m back on road 20 (south feeder). At 12 m/s the 2 s history window
  at a trigger 2.5 s before the mouth includes that turn; at 8 m/s (37 m =
  4.6 s) it is straight. Proposal to the paper side: ONCOMING_SPEED=8 at every
  site so k is the only thing that changes (8 is inside the T25 envelope).
- Conflict per branch: blocker 19 m after the mouth in the inward lane,
  recipient CAV 60 m behind the blocker along the lane (through the upstream
  junction where the branch is shorter; the route planner brings it in),
  conflict point 15 m after the mouth in the outward lane, RSU 10 m beside
  the conflict point. Lane rules: two-way single lane = oncoming overtake
  (k2, k3, k4 branch 61); two-way dual (k4 branches 3, 4) = oncoming overtake
  with the outer inward lane also blocked; one-way outward (k4 branch 44) =
  same-direction overtake (CAV and blocker in the outer lane, actor in the
  inner). Conflict type only affects the binary collision outcome; readiness,
  placement count and destination confidence are conflict-independent. k4
  feeds every figure under the genuine-k=4 outcome of the paper side's check.
- Polygons from the client-side carla.Map (OpenDRIVE string, no server, no
  GPU): approach locale = 8 m half-width corridor along the inward lane from
  the spawn to the mouth, union the padded convex hull of the arm mouths (the
  exit test fires at the branch mouths after the turn); branch locale = 60 m
  corridor along the outward lane (long enough for MTR mode endpoints at the
  trigger tick). Adjacent locales overlap only in the 3 m hull pad.

Files (scripts/branching, the scratchpad copies are the same bytes):
build_sites.py -> branching_sites.json (polygons, placements, lanes, conflict
points, route ends, per-site cav_destination); gen_branch_scenario.py ->
scenario_1_branch_{k}.xml + openscenario_1_branch_{k}.yaml (all CAV configs
under edge 0 `vehicles`, each edge lists `member_vehicles` by global index,
`locale.neighbors`, `locale.conflict_point`, RSU ids unique, gt injection
off); scenario_1_branch.py (actor routed by GlobalRoutePlanner to the taken
branch's route end, released at start, k egos + blockers from the XML,
CollisionTest per ego); runner_branch.py (copied per site as
openscenario_1_branch_{k}.py: candidate set per source, per-arm fire set,
prepares to every locale in the set, CONFROW per tick, READYROW / PLACEROW /
HANDOFFROW with hash / FIREROW / ENTERROW / NPCROW / HEROROW); daemon_branch.py
(cost.payload_hash = sha256 of the serialized bytes that cross the link);
sim_api_branch.py (member_vehicles binding, behavior-preserving for every
existing yaml). scripts/atlas_branch.sh runs the grid seed-major on the
campaign lock with the freeze-1p predictor/tracker overlay (cap=1, perception
active); scripts/khonsu_branch_land.py parses the rows and runs the checks.

Arms as run (env): khonsu = warm + TRIGGER_MODE=mtr (theta 0.5, computed L);
oracle = warm + oracle; velocity projection = warm + computed; top-1 = warm +
mtr + MTR_TOP1=1 (same tick and L as khonsu, enforced in code: one gate, two
selections); EdgeWarp = edgewarp_full; replicate-to-all = warm + computed +
REPLICATE_ALL=1 (fire set = all candidates at the computed L gate).

Lander checks (computed): payload byte identity where the tick is shared by
construction (replicate-to-all's k prepares within a run; khonsu vs top-1
across runs, plus identical trigger tick); the literal "hash identical across
all six arms" cannot hold because arms fire at different ticks and the record
carries the tracker state at the fire tick, so across those arms the check
is record size plus scene identity (NPC polyline max deviation over common
ticks, 1 m gate); route (actor entered BRANCH_TAKEN in every arm); oracle
ready (else the conflict timeline is wrong). Confidence samples: CONFROW at
the FIREROW tick and at T-2/1/0.5 s before ENTERROW; calibration = fraction
argmax==taken per p_argmax bin.

Smoke: `SMOKE=1 bash scripts/atlas_branch.sh` = k3, seed 1, arms oracle,
khonsu, replicate (+ ONCOMING_SPEED=8 once the paper side confirms). Output
to the paper side before any grid. GPU for 4 edges measured there.

Paper-side decisions after the k4 check (2026-09-12 ~22:15): road 62 k4 is
genuine and feeds every figure; ONCOMING_SPEED=8 at EVERY site (the degree
sweep varies k only; mixing 12 and 8 would put T_avail back on the axis); if
Oracle is not ~100% ready at 8 in the k3 smoke, tune the actor spawn distance
first and move speed only if that fails, at all three sites together. Hash
refinement accepted as proposed, plus one field on every prepare emit: the
record's history depth (memo frame count; daemon `cost.history_depth`,
`depth=` in [OBSTACLE_HANDOFF], the prepare line and HANDOFFROW). Lander
rule: depth must equal the configured record depth (10, RECORD_DEPTH env) on
every prepare or the run is flagged (an early trigger can ship a partial
record; size equality cannot see it). scripts/atlas_branch.sh defaults to 8.

BLOCKER (2026-09-13): the Khonsu and top-1 arms consume MTR destination
probabilities from the mode stash. Under the campaign's agent-set isolation
the stash carries six identical modes at 1/64 (mode collapse at batch size
1), so the trigger cannot reach theta; with a second key the scoring head
puts a stationary mode on top. The smoke now hard-fails on a khonsu/top1 cell
without a FIREROW and on the diversity gate; it is out of the lock queue
until the predictor fix (Tyler's choice, see current_state.md) passes a
one-cell verify on both gates. The certainty axis of Figure 8 depends on
which fix is chosen; under option (2) it does not exist for this model.

