"""Measured-time execution of the vehicle step (ecav/core/common/vehicle_compute.py).

The rule: an invocation that starts at step t with duration d occupies the component until
t + d. Its output is visible at the first step boundary at or after t + d. Inputs that arrive
while the component is busy are superseded and the next invocation reads the newest one. The
component is told the steps that passed since its previous invocation.

Checked here:
 1. visibility step for durations of zero, below one step, exactly one step, straddling a
    step, and several steps.
 2. hold: while an invocation is in progress the vehicle applies the last visible command,
    and the idle command before the first one.
 3. newest wins: the invocation after a busy period reads the input of its own step, and the
    inputs of the busy steps are counted as superseded.
 4. elapsed time: the planner and the controller receive the steps since their last invocation.
 5. scale: a factor on the durations moves the visibility step as the scaled duration does.
 6. replay: a run that replays a recorded trace applies every command at the same step as the
    run that recorded it, whatever the wall time of the replaying host.
 7. cooperative planning: the edge planner and the vehicle's controller are occupied separately.
 8. options off: the component is disabled, and the PID controller's output without an elapsed
    time is identical, bit for bit, to the controller on develop (skipped without git or carla).
    With an elapsed time of two steps the derivative term is half and the integral doubles.

Runs standalone."""
import json
import os
import subprocess
import sys
import tempfile
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from ecav.core.common.vehicle_compute import (  # noqa: E402
    DurationSource, StepOccupancy, VehicleCompute, steps_for)

DT = 0.05
PREV = "origin/develop"


class Scripted(VehicleCompute):
    """VehicleCompute whose stage durations come from a script instead of the wall clock."""

    def __init__(self, cfg, durations):
        super().__init__(cfg, DT, vehicle_index=0)
        self._script = list(durations)
        self._k = 0
        real = self.source.resolve

        def resolve(tick, measured):
            d = self._script[min(self._k, len(self._script) - 1)]
            self._k += 1
            return real(tick, {"perceive": 0.0, "plan": float(d), "control": 0.0})
        self.source.resolve = resolve


def run(vc, n_ticks):
    """Drive the pipeline for n_ticks. The plan is the tick it was planned at, and the command
    carries that tick. Returns (applied command per tick, planner calls, controller calls)."""
    plans, ctls, applied = [], [], []
    for t in range(n_ticks):
        vc.note_perceive(0.0)
        cmd = vc.step(t, lambda el, t=t: (plans.append((t, el)), ("plan", t))[1],
                      lambda tgt, el_s, t=t: (ctls.append((t, round(el_s / DT))), ("cmd", tgt[1]))[1],
                      idle_command="idle")
        applied.append(cmd)
    return applied, plans, ctls


def check(name, cond, detail=""):
    print(f"{name}: {'ok' if cond else 'FAIL'} {detail}")
    return bool(cond)


def previous_controller():
    try:
        import carla  # noqa: F401
        src = subprocess.run(["git", "-C", ROOT, "show", f"{PREV}:ecav/core/actuation/pid_controller.py"],
                             capture_output=True, text=True, timeout=20, check=True).stdout
    except Exception:
        return None
    m = types.ModuleType("pid_controller_prev")
    exec(compile(src, "pid_controller_prev", "exec"), m.__dict__)
    return m.Controller


def main():
    ok = True

    # 1. visibility step
    cases = ((0.0, 0), (0.4, 1), (49.9, 1), (50.0, 1), (50.1, 2), (55.0, 2), (100.0, 2), (160.0, 4))
    ok &= check("visibility step", all(steps_for(d, 50.0) == s for d, s in cases),
                str([(d, steps_for(d, 50.0)) for d, _ in cases]))

    # 2 to 4. a duration of 120 ms: three steps per invocation
    vc = Scripted({"vehicle_compute": "measured"}, [120.0])
    applied, plans, ctls = run(vc, 13)
    ok &= check("hold: idle until the first command, then each command for three steps",
                applied == ["idle"] * 3 + [("cmd", 0)] * 3 + [("cmd", 3)] * 3 + [("cmd", 6)] * 3 + [("cmd", 9)],
                str(applied))
    ok &= check("newest wins: invocations at steps 0, 3, 6, 9, 12 read the input of their own step",
                [p[0] for p in plans] == [0, 3, 6, 9, 12])
    ok &= check("superseded inputs counted", vc.pipeline.superseded == 8 and vc.pipeline.invoked == 5,
                str(vc.pipeline.metrics()))
    ok &= check("elapsed steps passed to the planner and the controller",
                [p[1] for p in plans] == [1, 3, 3, 3, 3] and [c[1] for c in ctls] == [1, 3, 3, 3, 3])
    row = [r for r in vc.rows if r["tick"] == 5][0]
    ok &= check("log row: command age at apply", row.get("applied_start_tick") == 0
                and row.get("command_age_ms") == 250.0 and row["invoked"] == 0, str(row))
    row = [r for r in vc.rows if r["tick"] == 3][0]
    ok &= check("log row: invocation start, duration, apply step, superseded",
                row["invoked"] == 1 and row["start_tick"] == 3 and row["duration_ms"] == 120.0
                and row["apply_tick"] == 6 and row["superseded"] == 2, str(row))

    # below one step: every step invokes, the command applies one step later
    vc = Scripted({"vehicle_compute": "measured"}, [4.0])
    applied, plans, _ = run(vc, 6)
    ok &= check("below one step: invoked every step, applied one step later",
                applied == ["idle"] + [("cmd", t) for t in range(5)] and len(plans) == 6, str(applied))

    # straddling one step: every second step
    vc = Scripted({"vehicle_compute": "measured"}, [55.0])
    applied, plans, _ = run(vc, 8)
    ok &= check("straddling a step: invoked every second step, applied two steps later",
                [p[0] for p in plans] == [0, 2, 4, 6]
                and applied == ["idle", "idle", ("cmd", 0), ("cmd", 0), ("cmd", 2), ("cmd", 2), ("cmd", 4), ("cmd", 4)],
                str(applied))

    # mixed durations
    vc = Scripted({"vehicle_compute": "measured"}, [30.0, 130.0, 30.0, 60.0, 30.0])
    applied, plans, _ = run(vc, 10)
    ok &= check("mixed durations", [p[0] for p in plans] == [0, 1, 4, 5, 7, 8, 9]
                and applied[:9] == ["idle", ("cmd", 0), ("cmd", 0), ("cmd", 0), ("cmd", 1), ("cmd", 4),
                                    ("cmd", 4), ("cmd", 5), ("cmd", 7)], str((plans, applied)))

    # 5. scale
    vc = Scripted({"vehicle_compute": "measured", "vehicle_compute_scale": 3.0}, [40.0])
    applied, plans, _ = run(vc, 7)
    ok &= check("scale 3 on 40 ms: three steps per invocation", [p[0] for p in plans] == [0, 3, 6])
    src = DurationSource({"vehicle_compute": "measured", "vehicle_compute_scale": 2.0, "vehicle_plan_scale": 0.5})
    d = src.resolve(0, {"perceive": 10.0, "plan": 10.0, "control": 10.0})
    ok &= check("a stage's own scale overrides the common one", d == {"perceive": 20.0, "plan": 5.0, "control": 20.0}, str(d))

    # constant and zero
    vc = VehicleCompute({"vehicle_compute": "100"}, DT, 0)
    applied, plans, _ = run(vc, 7)
    ok &= check("constant 100 ms: two steps per invocation, whatever the wall time",
                [p[0] for p in plans] == [0, 2, 4, 6] and applied[2] == ("cmd", 0), str(applied))
    for off in ({}, {"vehicle_compute": "zero"}, {"vehicle_compute": "off"}, {"vehicle_compute": 0}):
        ok &= check(f"disabled with {off}", not VehicleCompute(off, DT, 0).enabled)

    # 6. replay: record with one set of durations, replay on a host whose wall time differs
    with tempfile.TemporaryDirectory() as tmp:
        trace = os.path.join(tmp, "run.trace.veh0")
        rec = VehicleCompute({"vehicle_compute": "measured", "vehicle_compute_trace": trace}, DT, 0)
        script = [12.0, 70.0, 3.0, 140.0, 51.0, 49.0, 0.2, 99.0, 101.0, 8.0] * 4
        k = [0]
        real = rec.source.resolve

        def recording(tick, measured, real=real):
            d = script[k[0]]
            k[0] += 1
            return real(tick, {"perceive": d * 0.5, "plan": d * 0.4, "control": d * 0.1})
        rec.source.resolve = recording
        a_applied, a_plans, _ = run(rec, 40)
        rep = VehicleCompute({"vehicle_compute": "trace", "vehicle_compute_trace": trace}, DT, 0)
        b_applied, b_plans, _ = run(rep, 40)       # wall time here is microseconds, not the script
        ok &= check("replay: the same invocation steps and the same command at every step",
                    a_plans == b_plans and a_applied == b_applied and len(a_plans) > 10,
                    f"{len(a_plans)} invocations")
        ok &= check("replay: per-step rows identical",
                    json.dumps(rec.rows, sort_keys=True) == json.dumps(rep.rows, sort_keys=True))

    # 7. cooperative planning: planner on the edge, controller on the vehicle
    vc = VehicleCompute({"vehicle_compute": "120"}, DT, 0)
    got = [vc.edge_plan(t, lambda el, t=t: ("plan", t, el)) for t in range(7)]
    ok &= check("edge planner: invoked at steps 0, 3, 6, three steps to leave the edge, busy between",
                [g is not None for g in got] == [True, False, False, True, False, False, True]
                and got[0] == (("plan", 0, 1), 3) and got[3] == (("plan", 3, 3), 3), str(got))
    cmds = [vc.control(t, lambda el_s, t=t: ("cmd", t), idle_command="idle") for t in range(4)]
    ok &= check("controller under a constant planner duration costs nothing: applied in its own step",
                cmds == [("cmd", t) for t in range(4)], str(cmds))

    # generic occupancy
    g = StepOccupancy(50.0)
    g.start(0, 120.0, "a")
    ok &= check("occupancy: busy at steps 1 and 2, free at step 3",
                (g.free(1), g.free(2), g.free(3)) == (False, False, True)
                and g.visible(2) is None and g.visible(3) == (3, 0, "a"))

    # 8. controller identity without an elapsed time
    Prev = previous_controller()
    if Prev is None:
        print("controller identity: skipped (no git source or no carla module)")
    else:
        import carla
        import numpy as np
        from ecav.core.actuation.pid_controller import Controller
        args = {"max_brake": 1.0, "max_throttle": 1.0, "max_steering": 0.3, "dt": DT, "dynamic": False,
                "lon": {"k_p": 0.37, "k_d": 0.024, "k_i": 0.032}, "lat": {"k_p": 0.75, "k_d": 0.02, "k_i": 0.4}}
        new, old = Controller(args), Prev(args)
        rng = np.random.default_rng(1)
        same = True
        for i in range(400):
            tr = carla.Transform(carla.Location(x=float(i) * 0.5, y=float(rng.normal(0, 0.5))),
                                 carla.Rotation(yaw=float(rng.normal(0, 5))))
            spd = float(30 + rng.normal(0, 5))
            tgt = carla.Location(x=float(i) * 0.5 + 6.0, y=float(rng.normal(0, 1.0)))
            for c in (new, old):
                c.update_info(tr, spd)
            a, b = new.run_step(40.0, tgt), old.run_step(40.0, tgt)
            same &= (a.throttle, a.brake, a.steer) == (b.throttle, b.brake, b.steer)
        ok &= check("controller without an elapsed time: identical to develop over 400 steps", same)
        c1, c2 = Controller(args), Controller(args)
        for c, el in ((c1, DT), (c2, 2 * DT)):
            c.current_speed = 0.0
            c._elapsed_s = el
            c.lon_run_step(10.0)
            c.current_speed = 1.0
            c.lon_run_step(10.0)
        de = [(c._lat_ebuffer[-1] - c._lat_ebuffer[-2]) / c._lat_dtbuffer[-1] for c in (c1, c2)]
        ie = [sum(e * d for e, d in zip(c._lat_ebuffer, c._lat_dtbuffer)) for c in (c1, c2)]
        ok &= check("controller with two steps elapsed: derivative halves, integral doubles",
                    abs(de[1] - de[0] / 2) < 1e-12 and abs(ie[1] - 2 * ie[0]) < 1e-12, str((de, ie)))

    if not ok:
        sys.exit(1)
    print("PASS")


def test_vehicle_compute():
    main()


if __name__ == "__main__":
    main()
