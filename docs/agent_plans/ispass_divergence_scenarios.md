# ISPASS E6: state-model divergence scenarios

Goal: measure how closed-loop outcomes change as sensor-derived perception
replaces ground truth (GT), and attribute each change to a mechanism. This is
the evidence for the dual-state-model claim: architectures that look
equivalent under GT separate under real perception. Scenarios must be chosen
so the divergence is large, mechanistic, and sweepable, not incidental.

## Arms

| Arm | Configuration | What it isolates |
|---|---|---|
| A | GT injection everywhere (world-queried state, zero perception compute) | the oracle baseline |
| B | Ego-only local perception; all other actors GT/scripted | single-vehicle sensing error on the ego's own loop |
| C | All vehicles + RSU sensor-derived, full edge stack (fusion, tracking, prediction) | the deployed configuration |
| C-lat (control) | GT content injected but stamped with arm-C measured latencies | separates the two divergence channels |

The C-lat control matters: sensor-derived state diverges from GT through two
channels, information (what is known: misses, late detections, false
positives) and timing (when it is known: perception compute enters the AoI).
Without the control, a reviewer cannot tell whether outcome changes come from
information loss or added latency. GT-injection already exists as a
perception backend; C-lat is GT content + arm-C latency stamps.

Optional arm B-all (all vehicles local perception, no edge) distinguishes
cooperative recovery from mere perception degradation; run if time allows.

## Scenario catalog

Each scenario names its divergence mechanism and the expected arm ordering.
All reuse existing infrastructure; config variants that already exist are
noted.

### S1: blind overtake (occlusion at the conflict)
- Base: openscenario_1 family (accel_gt / accel_worldfusion variants exist;
  two_locale variant exists).
- Mechanism: oncoming vehicle hidden behind the stopped truck. GT sees
  through the truck; ego-local perception cannot see the oncoming until the
  commit point; the RSU vantage + fusion recovers it earlier.
- Expected: A succeeds with early aborts when needed; B commits blind or
  aborts late (failures/collisions); C recovers most of A's outcome.
  The A-B gap is the cost of local sensing; the B-C gap is the value of the
  cooperative stack. Both gaps are the paper's point.

### S2: LTAP / cross traffic (occlusion + range)
- Base: openscenario_3 family (edge / late_fusion / cip variants exist;
  cross actor is vehicle.tesla.model3, lookups must use live pose).
- Mechanism: crossing vehicle occluded by the corner building until late;
  beyond that, sparse LiDAR returns at range delay first detection even
  unoccluded. Sweep approach speed (target_speed already a config knob) to
  move the required detection distance.
- Expected: divergence grows with approach speed; A flat, B degrades
  steeply, C degrades slowly.

### S3: stop-sign crossing (SCP, sub-threshold evidence)
- Base: the SCP geometry from the Conductor closed-loop family.
- Mechanism: the crossing actor is visible to several viewpoints only as
  weak, sub-threshold returns. Ego-only perception never clears threshold in
  time; intermediate fusion combines the weak evidence. This is the one
  scenario where B-all (all local, no edge) also fails, so it isolates the
  fusion contribution specifically, not just more sensors.

### S4: weather sweep (graded divergence dial)
- Base: S1 and S2 re-run at fog/rain/cloud settings (weather already
  parameterized in config names: fog_0_rain_0_clouds_75...).
- Mechanism: sensor attenuation degrades perception continuously while GT is
  unaffected. This turns divergence into a curve, outcome vs weather
  severity per arm, rather than a binary scenario result. For ISPASS this is
  the strongest figure: a sweepable x-axis with mechanistic interpretation.

### S5: dense clutter (association divergence)
- Base: openscenario_20 12cav config (exists).
- Mechanism: even when detection succeeds, dense traffic causes association
  errors and ID switches in the tracking stack; prediction conditions on
  corrupted history. Divergence here appears in tracking/prediction metrics
  before it appears in outcomes.
- Expected: B and C separate from A on ID switches and FDE at densities
  where detection recall is still high, showing the stack, not just the
  sensor, is a divergence source.

### S6: false-positive direction (phantom braking)
- Not a separate scenario: instrument S1-S5 for false brakes. Divergence has
  two directions, missed threats (under-caution) and hallucinated obstacles
  (over-caution). GT never false-brakes; perception arms do. Report both
  directions; reviewers notice when only misses are counted.

## Metrics (per arm, per scenario)

1. Closed-loop: scenario success, collisions, min TTC, false-brake count.
2. Perception: recall on the safety-critical actor vs GT, first-detection
   time/distance (detection latency in meters of approach).
3. Tracking: ID switches, track continuity on the critical actor.
4. Prediction: FDE vs GT future for the critical actor.
5. Timing: per-arm planner-side AoI (arm A near-zero compute; arm C full
   stack), reported alongside so the C-lat control can decompose.

## Run matrix

S1, S2 x arms A/B/C/C-lat x 3 weather points x >=10 seeds.
S3 x arms A/B/B-all/C x >=10 seeds.
S5 x arms A/C x >=10 seeds (B is ill-defined at 12 CAVs; ego-only still runs
its own loop, include if cheap).
Order of construction: S1 first (most infra exists, gt + worldfusion configs
already side by side), then S4 on top of it (config-name weather knob), then
S2, S5, S3.

## What needs building

1. Arm B configs: ego-only perception (per-actor perception blocks already in
   the YAMLs; flip everyone else to GT injection).
2. C-lat: latency-stamped GT injection (GT backend exists; needs the stamp
   pass wired to arm-C measured latencies).
3. First-detection / recall-vs-GT logging on the critical actor (B4 frame
   logger from the Khonsu work does identity-first presence matching; reuse).
4. The weather-sweep driver (config templating already encodes weather in
   names; a sweep script like sweep_remaining.sh).

## Caution

- GPU runs queue behind Tyler's usage; no runs without go-ahead.
- Multi-V2X z-range and RSU mast height lessons apply to any new RSU
  placement (z=3, not 7; training z-clip [0,2]).
- Local perception must run BOTH pipelines where the edge is present (edge
  enhances, never replaces); S3's B-all arm must not accidentally disable
  the local stack.
