"""Published velocity of an accelerating vehicle under noisy, intermittent detections.

A crossing car launches from rest at 7 m/s^2 to 13.5 m/s (CARLA Tesla at full throttle), its box
edges carry 0.3 m of noise per frame, and 30% of the frames have no detection (dropped YOLO frames).
The tracker's published speed (row columns 10 and 12, m per frame) must follow the car: once the car
is above 10 m/s and the track has ten matched frames, the published speed may not lag the true
speed at the last matched frame by more than 2.0 m/s on average nor by more than 3.0 m/s in any
frame at 7 m/s^2 (the half-second slope lags a constant acceleration by a quarter second, 1.75 m/s),
by more than 1.0 m/s on average at 2 m/s^2 (the acceleration left at the planner's decision in the
left turn at 70 km/h), and at constant speed it must stay within 1.0 m/s on average. Before
2026-10-26 the estimate was the displacement over the last 21 matched frames (1.5 s with dropped
frames) capped by the slower of its two halves: 4.9 m/s low in this test, and 9.3 m/s for a car at
12 to 13 m/s in the j4 left turn at 70 km/h and 500 ms (collisions in 8 of 10 seeds). A parked vehicle whose visible face slides is
still published at rest (tests/test_edge_velocity.py). Runs standalone."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402

DT = 0.05
ACC = 7.0
VMAX = 13.5
SIGMA = 0.3
DROP = 0.3
FRAMES = 140


def true_state(k, acc=ACC, v0=0.0, acc2=None, v_knee=10.0):
    """Position and speed at frame k: acceleration acc from v0, then acc2 (if given) from v_knee,
    then constant VMAX. The two-phase profile mirrors a CARLA launch: strong at first, tapering."""
    t = k * DT
    x, v, t0 = 0.0, v0, 0.0
    phases = [(acc, v_knee if acc2 is not None else VMAX)]
    if acc2 is not None:
        phases.append((acc2, VMAX))
    for a, v_end in phases:
        t_end = t0 + (v_end - v) / a
        if t <= t_end:
            return x + v * (t - t0) + 0.5 * a * (t - t0) ** 2, v + a * (t - t0)
        x += v * (t_end - t0) + 0.5 * a * (t_end - t0) ** 2
        v, t0 = v_end, t_end
    return x + VMAX * (t - t0), VMAX


def det_from_box(b):
    xmin, xmax, ymin, ymax = b
    return [1.8, ymax - ymin, xmax - xmin, (xmin + xmax) / 2, 0.0, (ymin + ymax) / 2, 0.0]


def run(seed=0, sigma=SIGMA, drop=DROP, acc=ACC, v0=0.0, acc2=None):
    rng = np.random.default_rng(seed)
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    rows = []   # (frame, true_speed_at_last_match, published_speed, matched_frames)
    matched = 0
    v_last = 0.0
    for k in range(1, FRAMES + 1):
        x, v = true_state(k, acc, v0, acc2)
        if rng.random() < drop:
            bundle = {"dets": np.zeros((0, 7)), "info": np.zeros((0, 3), dtype=np.int64), "edges": np.zeros((0, 4))}
        else:
            box = (x - 2.25 + rng.normal(0, sigma), x + 2.25 + rng.normal(0, sigma),
                   126.7 + rng.normal(0, sigma), 128.7 + rng.normal(0, sigma))
            bundle = {"dets": np.array([det_from_box(box)], dtype=float),
                      "info": np.array([[k, 100 + k, -1]], dtype=np.int64),
                      "edges": np.array([box], dtype=float)}
            matched += 1
            v_last = v
        out, _ = trk.track(bundle, k)
        r = out[0] if out and len(out[0]) else np.zeros((0, 16))
        if len(r):
            pub = math.hypot(float(r[0][10]), float(r[0][12])) / DT
            rows.append((k, v_last, pub, matched))
    return rows


def stats(rows, lo, hi):
    err = [pub - vt for k, vt, pub, m in rows if m >= 10 and lo <= vt <= hi]
    assert err, "no frames in range"
    return float(np.mean(err)), float(np.std(err)), float(min(err)), float(max(err)), len(err)


def main():
    worst_mean, worst_min = 0.0, 0.0
    for seed in range(5):
        rows = run(seed)
        m_acc, s_acc, mn_acc, mx_acc, n_acc = stats(rows, 10.0, VMAX - 1e-6)
        m_cv, s_cv, mn_cv, mx_cv, n_cv = stats(rows, VMAX, VMAX)
        print(f"seed {seed}: accelerating 10 to 13.5 m/s: mean err {m_acc:+.2f} std {s_acc:.2f} min {mn_acc:+.2f} max {mx_acc:+.2f} (n={n_acc}); "
              f"constant 13.5 m/s: mean err {m_cv:+.2f} std {s_cv:.2f} min {mn_cv:+.2f} max {mx_cv:+.2f} (n={n_cv})")
        worst_mean = min(worst_mean, m_acc)
        worst_min = min(worst_min, mn_acc)
        assert m_acc > -2.0, f"seed {seed}: mean lag {m_acc:.2f} m/s while accelerating at 7 m/s^2"
        assert mn_acc > -3.0, f"seed {seed}: worst lag {mn_acc:.2f} m/s while accelerating at 7 m/s^2"
        assert abs(m_cv) < 1.0, f"seed {seed}: mean error {m_cv:.2f} m/s at constant speed"
        rows2 = run(seed, acc2=2.0)
        m2, s2, mn2, mx2, n2 = stats(rows2, 10.5, VMAX - 1e-6)
        print(f"seed {seed}: launch at 7 m/s^2 to 10 m/s then 2 m/s^2 to 13.5: mean err {m2:+.2f} std {s2:.2f} min {mn2:+.2f} (n={n2}) over 10.5 to 13.5 m/s")
        assert m2 > -1.0, f"seed {seed}: mean lag {m2:.2f} m/s while accelerating at 2 m/s^2"
    print(f"ok: worst mean lag {worst_mean:+.2f} m/s, worst single-frame lag {worst_min:+.2f} m/s")


if __name__ == "__main__":
    main()
