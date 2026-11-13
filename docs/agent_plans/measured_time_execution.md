# Measured-time execution of the tracker and the vehicle step

Branch `measured-time` (from develop at 0cf9e926). Written 2026-11-12. Tyler's directive: run
the control and the tracker on the time they take, and arbitrate that time against the fixed
step.

## The rule

One rule for every component. The edge service already follows the rule
(`ecav/core/application/edge/service_occupancy.py`).

1. Admission. An invocation starts at a step boundary, and only when the component is free
   in simulated time.
2. Occupancy. An invocation that starts at `t_s` with duration `d` occupies the component
   until `t_s + d`.
3. Latest input. Inputs that arrive while the component is busy are superseded and counted.
   The next invocation reads the newest input.
4. Visibility. The output is visible to its consumer at the first step boundary at or after
   `t_s + d`. A duration of zero is the same step. With a 50 ms step, 4 ms and 50 ms are one
   step, 55 ms and 100 ms are two steps, 160 ms is four steps.
5. Elapsed time. The component is told how many steps passed since its previous invocation,
   and its internal dynamics use that time.

Durations come from measurement in the run, from a recorded trace, or from a constant. A
scale factor per stage models a computer slower or faster than the simulation host.

## What is implemented

### Vehicle pipeline

`ecav/core/common/vehicle_compute.py` (`StepOccupancy`, `DurationSource`, `VehicleCompute`).
`VehicleManager` creates one `VehicleCompute` per vehicle. Every caller of
`vm.update_info()` and `vm.run_step()` is covered without a change to the caller: the
late-fusion, infrastructure-only, exact-state, VIPS and intermediate-fusion managers, and the
distributed vehicle client.

The pipeline is one component with three stages that run back to back, as the vehicle manager
executes them.

| Stage | What is timed | Where |
|---|---|---|
| perceive | localization, onboard detection, the vehicle's own tracker, state updates | `VehicleManager.update_info` |
| plan | `BehaviorAgent.run_step` and the map manager step | `VehicleManager._run_step_measured` |
| control | `ControlManager.run_step` | same |

At a step where the pipeline is free, the planner and the controller run, the three stage
durations are added, and the command is queued for the step given by rule 4. At a step where
the pipeline is busy, the planner and the controller do not run, and the vehicle applies the
command it applied last. Before the first command is visible the vehicle applies a neutral
command (no throttle, no brake).

Elapsed time:

- Planner. `BehaviorAgent.advance_time(k)` adds the skipped steps to the planner's step
  counter, so the start hold, the oncoming hold and the overtake timers count simulated time.
  Forecast re-indexing already uses the global tick of the invocation.
- Controller. `Controller.run_step(..., elapsed_s=)` uses the time since the previous
  invocation for the derivative term and each sample's own interval for the integral term.

`update_info` still runs at every step, because localization, the V2X manager, the safety
manager and the uplink of the vehicle's detections read its results. Its wall time is charged
only at a step where an invocation starts. A separate occupancy for the uplink of the
vehicle's own detections is not modelled (open question 2).

### Cooperative planning manager

`CIPEdge._advance_actors` does not call `vm.run_step()`. The planner runs on the edge and the
controller on the vehicle, so they are two components.

- Edge planner: `VehicleCompute.edge_plan`. A plan leaves the edge after the planner's
  duration, then takes the downlink delay as before. While the planner is busy no plan is
  queued.
- Vehicle controller: `VehicleCompute.control`. The command applies after the controller's
  duration, and the vehicle holds the previous command meanwhile.

### Tracker

The other worker's commit 0cf9e926 indexes the tracker's history, velocity window and
half-second rules by source tick, and predicts the state over the stride. This branch
completes the predict step: the process noise of a predict over `s` ticks is the noise of `s`
one-tick predicts, `Q_s = sum_{k<s} F^k Q (F^k)^T`. One predict over `s` ticks then leaves the
same state and covariance as `s` one-tick predicts.

What stays in tracker calls: `min_hits` and `max_age`. A track needs that many observations at
any frame rate. With `max_age = 3` a track is removed after three calls without a match:
150 ms at one tick per call, 450 ms at three, 600 ms at four.

The tracker's own duration (1 to 5 ms per frame, see below) is part of the edge service's
measured compute, which `compute_delay=measured` already charges, and which
`service_occupancy=on` already arbitrates. No separate tracker occupancy is added.

The planner's `_blocked_ticks` and `_rss_lateral_growing_ticks` also count invocations. They
are consecutive-observation counts, like `min_hits`, and stay in invocations.

## Options

All are behavior options (`test_runner --behavior-kv`), so they are part of the run directory
name. Without `vehicle_compute` nothing changes.

| Option | Values | Meaning |
|---|---|---|
| `vehicle_compute` | `zero` (default) | the step runs inside the tick at no cost, as before |
| | `measured` | wall time of each stage, measured in the run |
| | `trace` | durations replayed from a recorded trace |
| | `<ms>` | constant duration of the whole pipeline |
| `vehicle_compute_scale` | number, default 1 | factor on every stage |
| `vehicle_perceive_scale`, `vehicle_plan_scale`, `vehicle_control_scale` | number | a stage's own factor |
| `vehicle_compute_trace` | path | trace file. Default: `ECAV_TIMING_TRACE` with `.veh<index>` appended |

With `measured` and a trace path the run records the trace. With `trace` the run replays the
trace, and the delivery steps do not depend on the replaying host.

## Logs

Per step, per vehicle, JSON lines in `vehicle_compute_<vehicle id>.jsonl` in the run directory
(when the edge has `run_output_dir`), or at `ECAV_VEHICLE_COMPUTE_LOG` + `.veh<index>.jsonl`.
Fields: `tick`, `invoked`, `start_tick`, `perceive_ms`, `plan_ms`, `control_ms`,
`duration_ms`, `apply_tick`, `superseded` (inputs superseded since the previous invocation),
`applied_start_tick`, `command_age_ms` (age of the applied command at this step). The
cooperative planning rows carry `component` (`edge_planner` or `controller`).

`simulation_metrics.json` gets `vehicles/<id>/vehicle_compute` with stage means and 95th
percentiles, invocations, superseded fraction, and command age.

## What is charged and what is not

| Charged | Not charged |
|---|---|
| localization, onboard detection, local tracker | CARLA's world step and sensor rendering |
| planner step | the edge service (charged by `compute_delay` and `service_occupancy`) |
| controller step | actuation delay inside the vehicle (CARLA applies the command in the next physics step) |
| edge planner (cooperative planning) | radio delay (charged by the latency model or the ns-3 tables) |

Measured wall time is the time on the simulation host. A shared perception service adds its
queueing to the vehicle's detection time, so `measured` with `-p shared` charges the service's
load to each vehicle. `vehicle_compute_scale` or a constant removes that dependence.

## Age at use

Before: age at use = uplink + edge compute (+ occupancy wait) + downlink + configured delay,
each rounded up to the step, and the planner used the forecast in the step in which the
forecast arrived.

With `vehicle_compute`: the planner still reads the newest delivered forecast at its
invocation step `t_s`, and the command acts on the vehicle at `t_s + ceil(d / dt)`. Two terms
are added to the age of the forecast at the time the command acts:

- the wait for the pipeline to be free, up to `ceil(d / dt) - 1` steps,
- the pipeline's duration, `ceil(d / dt)` steps.

`per_tick_age.csv` keeps reporting the age at the planner's invocation. The age at actuation
is that age plus `command_age_ms` from the vehicle compute log, joined on the tick.

## Durations on record

From the `simulation_metrics.json` files of `ispass_fid6` as mirrored on atlas on Nov 12
(60 cells, means of per-run means). The intermediate-fusion cells present are conditions A
and C.

| Component | Object-level arms (atlas, RTX 4080 Super) | Intermediate fusion (cetus) |
|---|---|---|
| vehicle localization | 0.2 ms | 0.2 ms |
| vehicle onboard perception | 27 ms (per-run means 27 to 29) | 80 ms (80 to 82) |
| vehicle local tracker | 2.2 ms | 2.9 ms |
| edge tracker, per frame | 3.6 ms (I2V), 4.8 ms (V2X2V) | 1.0 ms |
| edge service, measured compute | 20 ms | 145 ms (fusion 125 ms) |

The planner and controller step times are not in `simulation_metrics.json`. `ClientMetrics`
records them per step but the evaluator does not write them, so no number is on record. The
first `vehicle_compute=measured` run reports them in the vehicle compute log.

At a 50 ms step:

- Object-level arms: perception plus local tracking is about 30 ms. The pipeline fits one
  step if the planner and controller take under 20 ms together. The command then acts one
  step (50 ms) after the planner read its inputs, and the planner runs every step.
- Intermediate fusion on cetus: perception alone is 80 ms, so an invocation takes two steps
  if the planner takes under 20 ms. The planner runs every second step (10 Hz), and the
  applied command is 100 to 150 ms old.
- The edge tracker takes well under one step in every arm. Its cost reaches the planner
  through the edge service's compute, not through a step of its own.

## Effect on the ISPASS ablation

The compute-aware conditions (B and D) charge edge compute only. With `vehicle_compute`,
every arm also charges the vehicle pipeline, at least one step. The I2V and V2X2V cells of
conditions B and D would change and need a rerun with the option on. Conditions A and C stay
at `vehicle_compute=zero` by definition. The decision to add the option to B and D is Tyler's
(open question 1). One code version per table: if adopted, all B and D cells of all three
arms run again from this branch after the merge.

## Evidence

- `tests/test_vehicle_compute.py`: the visibility step for durations of zero, under one step,
  one step, straddling, and several steps. Hold of the previous command. Newest input wins
  and superseded inputs are counted. Elapsed steps reach the planner and the controller.
  Scale. Constant duration. Replay applies every command at the step the recording run
  applied it. Cooperative planning components. The controller without an elapsed time is
  identical, bit for bit, to the controller on develop over 400 random steps.
- `tests/test_tracker_predict_interval.py`: one predict over 2, 3, 4 and 7 ticks against that
  many one-tick predicts (state difference under 2e-14 m, relative covariance difference
  under 3e-16). Published rows and filter states at a one-tick stride identical to 0cf9e926
  on 24 noisy sequences (with and without edges, dropped frames, time rules on and off).
  `min_hits` and `max_age` in calls at strides 1, 3 and 4.
- With the option off, `VehicleManager.run_step` takes the unchanged code path
  (`vehicle_compute.enabled` is false), the planner's step counter advances by one, the
  controller receives no elapsed time, and the cooperative planning manager calls the planner
  and controller as before. No closed-loop run has confirmed that yet (validation run 1).

Not verified: nothing here has run in CARLA. `tests/test_edge_velocity.py` fails on develop
before this branch (rest-hold assertion at ticks 15 to 22) and fails the same way here.

## Validation runs (not launched)

Run on a free host from the merged tree. `HIDE` is the object list in
`scripts/ispass/fidelity_ablation.sh`.

```
COMMON="-t openscenario_3_edge_late_fusion --latency-distribution fixed --speeds 70 --ego-counts 1 \
  --per-tick-age-log --self-id-radius 1 --edge-kv prediction_steps=60 compute_delay=measured \
  --scenario-env CROSS_TRIGGER_TTA=70:4.77 HIDE_ENV_OBJECTS=$HIDE EGO_SPAWN_Y=-90 EGO_DRIVE_DISTANCE=400 SCENARIO_MAX_STEP=900 SR_LOCKSTEP=1"

# 1. Options off is unchanged: exact state, lock step, two repeats on this branch against two on develop.
#    Compare with scripts/ispass/repeat_first_diff.py (expected: identical over all ticks).
python test_runner.py $COMMON --manager-types oracle --anchoring on --latencies 0 --repetitions 2 --stamp mt_off \
  --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0

# 2. Measured: stage durations and command age for each arm, three seeds.
for arm in "--manager-types infra_only --anchoring off" "--manager-types late_fusion --anchoring on" "--baselines wf --anchoring on"; do
  python test_runner.py $COMMON $arm --latencies 0 1000 --repetitions 3 --stamp mt_measured \
    --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 vehicle_compute=measured
done

# 3. Constant sweep: where the vehicle pipeline's duration starts to cost outcomes (exact state).
for ms in 50 100 150 200 300; do
  python test_runner.py $COMMON --manager-types oracle --anchoring on --latencies 0 --repetitions 5 --stamp mt_const \
    --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 vehicle_compute=$ms
done

# 4. Replay: record, then replay with the stamps of the first run. The two runs must apply every command at the same tick.
ECAV_TIMING_TRACE=$PWD/tmp/mt_trace python test_runner.py $COMMON --manager-types oracle --anchoring on --latencies 0 \
  --repetitions 1 --stamp mt_rec --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 vehicle_compute=measured
ECAV_TIMING_TRACE=$PWD/tmp/mt_trace python test_runner.py $COMMON --manager-types oracle --anchoring on --latencies 0 \
  --repetitions 1 --stamp mt_rep --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 vehicle_compute=trace

# 5. Cooperative planning with a slow edge planner and a scaled vehicle computer.
python test_runner.py $COMMON --manager-types cip --anchoring on --latencies 0 --repetitions 3 --stamp mt_cip \
  --behavior-kv curv_lat_accel=8 self_exclusion_m=1.0 vehicle_compute=measured vehicle_compute_scale=4
```

Pass criteria: run 1 identical. Run 2 completes at 0 ms with a command age of one step for
the object-level arms and two to three steps for intermediate fusion, and the vehicle compute
log shows no step without an applied command after the first invocation. Run 3 is monotone in
the constant. Run 4 has identical `vehicle_compute_*.jsonl` apply ticks.

## Open questions for Tyler

1. Should the ablation's compute-aware conditions charge the vehicle pipeline, or should the
   paper report vehicle compute as a fifth condition? Charging it changes the I2V and V2X2V
   cells that are final today.
2. Onboard perception runs at every step even while the pipeline is busy, because the uplink
   of the vehicle's detections and the vehicle's own localization depend on it. Should the
   perceive stage have its own occupancy, so that a vehicle with 80 ms perception uploads at
   10 Hz?
3. One pipeline or two rates. Vehicles in practice run the controller at a higher rate than
   the planner. The implementation runs planner and controller as one invocation, which is
   what the code does today. A two-rate variant (controller every step on the latest visible
   plan) is a small change to `VehicleCompute.step`. Which should the paper describe?
4. Latency compensation. The planner re-indexes forecasts to its invocation step. A planner
   that knows its own latency would re-index to the step at which its command acts. Not
   implemented.
5. Measured vehicle perception differs by host (27 ms on atlas, 80 ms on cetus) and by
   perception manager. For a table, a recorded trace or a constant per arm may be preferable
   to live measurement on mixed hosts.
6. `ClientMetrics` step-time lists are not updated on the measured path. The vehicle compute
   log replaces them there.
