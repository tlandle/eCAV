# Figure 5 sub-second points: boundary-moved sweep at fixed speed (design, 2026-09-13)

Request (Tyler via the paper side): points with T_avail below one second in the
region the figure labels "preparation required", by moving the locale boundary
7 to 13 m before the conflict at fixed speed, five seeds per point, the t25
arms and success rule, both motions, the t25b schema (shift_x_m, t_avail_s,
motion).

## Facts that shape the block (verified 2026-09-13)

- The landed "t25b" (idfix_wt/evaluation_outputs/frozen_1n_t25b) is five
  warm-only, one-seed SPAWN-shift runs (ONCOMING_SHIFT_X = 21, 7, 0, -7, -14),
  tag khonsu-eval-freeze-1n, no [ISOROW], mode diversity 0.44 (gate 0.95):
  it reruns under 1q with the rest of the predictor-consuming blocks, and so
  does this extension (gated on the fix shape).
- By the t25 lander's definition (t_avail_s = (conflict_tick - crossing_tick)
  x 0.05, oncoming crossing of the destination edge to its arrival at x = 278)
  those runs give 3.0 to 3.15 s at every shift: the spawn shift does not move
  the boundary-to-conflict distance. The sub-second values in the KB use the
  per-run definition crossing -> the focal CAV's decision arrival.
- The polygon-moving harness (scripts/atlas_t25_boundary.sh, widths held) has
  never run (no frozen_1p_bnd directory). It is the clean T_avail knob.

## Geometry

Widths held (170, 120): locale_1 = [b - 170, b], locale_0 = [b - 10, b + 110];
guard: 100 < b < 286 (measured stall point); RSU1 at 205 and RSU0 at 325 stay
inside their locales for every admissible b; locale_0 always contains the
conflict. The oncoming crosses the destination edge at b - 10 at 12 m/s, so
T_avail (lander definition) = (288 - b) / 12: b = 282 / 280 / 277 -> 0.50 /
0.67 / 0.92 s; smallest admissible about 0.17 s at b = 286. Geometric
estimates: the crossing tick is the tracker's locale assignment with
hysteresis, confirmed by the smoke.

## Block

- Bounds: 220 235 250 262 275 (the harness's set) + 277 280 282 (new).
- Arms: cold kf reactive warm oracle (the t25 set); seeds 1..5 (KHONSU_SEED,
  paired spawn phase); ONCOMING_SPEED 12.
- Motions: constant (openscenario_1_flow_gt.yaml) and maneuvering
  (openscenario_1_accel_gt.yaml patched with the same set_locale_boundary call;
  the maneuvering operating point is the accel panel's: CRUISE 5, ACCEL 2,
  VCAP 16, with ONSET as probed for the landed maneuvering block; MANEUVER_ENV
  in the harness; the run refuses a maneuvering motion without ONSET).
- Predictor: the 1q overlay (RANKROW present) after the fix ruling;
  mode-diversity gate on every log; EVAL_TAG khonsu-eval-freeze-1q; OUTDIR
  frozen_1q_bnd.
- Lander: khonsu_t25_land.py extended: tags bnd_<mode>_b<b>_s<seed>;
  crossing edge per run = b - 10 from the [LAUNCHENV] boundary= field;
  shift_x_m = b - 250; boundary_x = b; motion from the launch env (ONCOMING_ACCEL
  present -> maneuvering); t_avail_s (lander definition) and
  t_avail_decision_s = (do_ov_tick - crossing_tick) x 0.05 both emitted. The
  paper side rules which one is the figure's axis.

## Invalidation conditions

- If the smoke's measured crossing tick at b = 282 does not give T_avail
  within 0.15 s of the 0.5 s estimate, the hysteresis assumption is wrong and
  the bounds are re-derived from the smoke before the grid.
- If the oncoming's conflict tick precedes its crossing tick (negative
  T_avail), the boundary is past the interaction; the point is dropped, not
  clipped, and the smallest feasible T_avail is reported.
- If the no-transfer arm's completion does not fall below one second of
  T_avail while warm holds, the region label is not supported by measurement
  and is reported as such.
- If the maneuvering motion's decision arrival differs from the constant
  motion's by more than the seed spread at the same b, the two motions are
  not on the same axis and are plotted separately.

## Gates

Fix-shape ruling (1q) -> smoke (warm, b = 250 and b = 282, one seed, both
motions) sent to the paper side -> grid with the t25 rerun.

## Maneuvering operating point (ruling 2026-09-13, late)

The maneuvering pass uses exactly the operating point the t25 maneuvering
rerun uses under 1q, because the figure's rule collapses both sweeps onto one
curve (a boundary-moved point must sit inside the Wilson interval of the
nearest speed-sweep point for the same motion and arm). The harness therefore
carries no default MANEUVER_ENV and refuses a maneuvering pass without one.
Record of what the host holds: the Sep 10 maneuvering probes
(frozen_1m_{maneuver_smoke,accremap,accelrecheck,lowaccel,accelmag}) used spawn
shifts (ONCOMING_SHIFT_X -10 to -70) with ONCOMING_ACCEL 1.5 to 6 and cap 16
and no explicit onset; the explicit-onset launch lines (245 in four logs, 235
in one) belong to the onset probe, which the KB declares void. Whichever point
the t25 rerun adopts (the panel point cruise 5 / accel 2 / cap 16 / onset 245,
or the endogenous-onset branch with the onset placed about 0.5 s before the
crossing) is written here when ruled, so the 5.2.2 caption can name it.
Arithmetic the ruling should see: at cruise 5 m/s the road user reaches the
conflict from the crossing edge in (288 - b) / 5 s, so the maneuvering motion's
T_avail at b = 282 is about 1.2 s, not 0.5; sub-second maneuvering points need
b >= 284 under the 286 guard (crossing edge 4 m before the conflict), or an
onset that has the road user already accelerating at the crossing. The
constant pass needs nothing further.

