"""Forecast displacement per second must not depend on how often the tracker is stepped.

A car at a constant 13 m/s is detected without noise and fed to AB3DMOT every tick, every third
tick, every fourth tick, and on the mixed 3 and 4 tick strides that service occupancy produces at
145 ms of compute. The manager's conversion (TrackerStepClock and velocity_per_tick) must publish
kf_vx in metres per simulation tick, so that the linear predictor's extrapolation
(position + kf_vx * (i + 1) per 50 ms step) covers 13 m in 20 steps for every stride. Before
2026-11-12 the per-tracker-step velocity was sent as per-tick, and the forecast ran at 3 to 4
times the car's speed with occupancy on. A one-tick stride must give a conversion factor of
exactly 1.0, so that runs without occupancy are unchanged. Runs standalone."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402
from ecav.core.application.edge.tracker_step import TrackerStepClock, velocity_per_tick  # noqa: E402

DT = 0.05
V = 13.0
TICKS = 400
PRED_STEPS_PER_S = int(round(1.0 / DT))


def ticks_for(strides):
    out, t, i = [], 1, 0
    while t <= TICKS:
        out.append(t)
        t += strides[i % len(strides)]
        i += 1
    return out


def run(strides):
    """Forecast displacement over one second (m) and the clock, after the last tracker call."""
    trk = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})._tracker
    clock = TrackerStepClock(DT, 0.2)
    row = None
    for k in ticks_for(strides):
        x = V * k * DT
        # no measured box edges, as in the intermediate-fusion manager: the tracker publishes
        # the slope of its own output positions
        bundle = {"dets": np.array([[1.8, 2.0, 4.5, x, 0.0, 127.7, 0.0]], dtype=float),
                  "info": np.array([[k, 100 + k, -1]], dtype=np.int64)}
        clock.note(k)
        out, _ = trk.track(bundle, k)
        if out and len(out[0]):
            row = out[0][0]
    assert row is not None, "no track published"
    kf_vx, kf_vy, speed = velocity_per_tick(float(row[10]), float(row[12]), False,
                                            clock.step_s(), DT, clock.measured)
    disp = (kf_vx ** 2 + kf_vy ** 2) ** 0.5 * PRED_STEPS_PER_S
    return disp, speed, clock, row


def main():
    ok = True
    for name, strides, tol in (("stride 1", [1], 1e-6), ("stride 3", [3], 1e-6),
                               ("stride 4", [4], 1e-6), ("strides 3,4", [3, 4], 1e-6),
                               ("strides 3,3,4", [3, 3, 4], 5e-3)):
        # uneven strides: the filter's positions, predicted with a unit step, sit slightly off
        # the line, so the slope is exact only to a fraction of a percent
        disp, speed, clock, row = run(strides)
        good = abs(disp - V) <= tol * V and abs(speed - V) <= tol * V
        ok &= good
        print(f"{name:14s} displacement per second {disp:8.4f} m  speed {speed:8.4f} m/s  "
              f"step {clock.step_s() * 1000:6.1f} ms  {'ok' if good else 'FAIL'}")
    # a one-tick stride is passed through bit for bit
    disp, speed, clock, row = run([1])
    kf_vx, kf_vy, _ = velocity_per_tick(float(row[10]), float(row[12]), False, clock.step_s(), DT, True)
    same = (kf_vx == float(row[10]) and kf_vy == float(row[12]) and clock.step_s() == 1 * DT)
    print("one-tick stride unchanged:", "ok" if same else "FAIL")
    # before two calls the row passes through and the speed uses the fallback step
    c = TrackerStepClock(DT, 0.2)
    c.note(5)
    a = velocity_per_tick(0.3, -0.4, False, c.step_s(), DT, c.measured)
    young = (a[0] == 0.3 and a[1] == -0.4 and abs(a[2] - 0.5 / 0.2) < 1e-12)
    print("unmeasured step passes through:", "ok" if young else "FAIL")
    # rows already in m/s are scaled by the tick
    m = velocity_per_tick(13.0, 0.0, True, 0.2, DT, True)
    mps = (m[0] == 13.0 * DT and m[2] == 13.0)
    print("m/s rows:", "ok" if mps else "FAIL")
    if not (ok and same and young and mps):
        sys.exit(1)
    print("PASS")


def test_tracker_step_velocity():
    main()


if __name__ == "__main__":
    main()
