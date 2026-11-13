"""The tracker's Kalman predict covers the interval between its invocations.

Under service occupancy the tracker is called every third or fourth source tick. Its predict
then advances the state over that many ticks, with the process noise of that many one-tick
predicts. Checked here:

1. One predict over s ticks leaves the same state and covariance as s one-tick predicts
   (s = 2, 3, 4, 7), to floating-point rounding.
2. At a one-tick stride the tracker's published rows and filter states are identical, bit for
   bit, to the tracker before the process noise was scaled (commit 0cf9e926), on noisy
   sequences with dropped frames, with and without measured edges, with time_rules on and off.
   Skipped when git cannot provide the earlier source.
3. min_hits and max_age remain counts of tracker calls: the call at which a track is first
   published and the number of calls without a match after which it is removed are the same
   at strides 1, 3 and 4.

Runs standalone."""
import copy
import importlib.util
import os
import subprocess
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402

PREV_COMMIT = "0cf9e926"

_spec = importlib.util.spec_from_file_location(
    "tracker_time_rules_helpers", os.path.join(ROOT, "tests", "test_tracker_time_rules.py"))
H = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(H)


def previous_model():
    try:
        src = subprocess.run(["git", "-C", ROOT, "show", f"{PREV_COMMIT}:AB3DMOT_libs/model.py"],
                             capture_output=True, text=True, timeout=20, check=True).stdout
    except Exception:
        return None
    m = types.ModuleType("ab3dmot_model_prev_q")
    m.__file__ = os.path.join(ROOT, "AB3DMOT_libs", "model.py")
    exec(compile(src, "ab3dmot_model_prev_q", "exec"), m.__dict__)
    return m


def seeded_tracker():
    """A tracker holding two tracks with non-trivial state and covariance."""
    trk = H.new_tracker()
    rng = np.random.default_rng(3)
    for k in range(1, 9):
        rows = [[1.8, 2.0, 4.5, 13.0 * k * H.DT + rng.normal(0, 0.2), 0.0, 127.7 + rng.normal(0, 0.2), 0.0],
                [1.8, 2.0, 4.5, 30.0 + rng.normal(0, 0.2), 0.0, 140.0 - 6.0 * k * H.DT, 0.0]]
        trk.track({"dets": np.array(rows, dtype=float),
                   "info": np.array([[k, 100, -1], [k, 101, -1]], dtype=np.int64)}, k)
    assert len(trk.trackers) == 2
    return trk


def predict_equivalence(s):
    """Largest absolute difference in state and covariance between one predict over s ticks
    and s one-tick predicts."""
    base = seeded_tracker()
    a, b = copy.deepcopy(base), copy.deepcopy(base)
    a._stride = s
    a.prediction()
    b._stride = 1
    for _ in range(s):
        b.prediction()
    dx = max(float(np.abs(ta.kf.x - tb.kf.x).max()) for ta, tb in zip(a.trackers, b.trackers))
    dp = max(float(np.abs(ta.kf.P - tb.kf.P).max()) for ta, tb in zip(a.trackers, b.trackers))
    scale = max(float(np.abs(tb.kf.P).max()) for tb in b.trackers)
    return dx, dp / scale


def call_counts(stride):
    """(calls until the track is first published, calls without a match until it is removed)."""
    trk = H.new_tracker()
    first, k, calls = None, 1, 0
    while first is None and calls < 20:
        out, _ = trk.track(H.bundle(13.0 * k * H.DT, k, False), k)
        calls += 1
        if out and len(out[0]):
            first = calls
        k += stride
    for _ in range(30):                      # a mature track
        trk.track(H.bundle(13.0 * k * H.DT, k, False), k)
        k += stride
    missed = 0
    while trk.trackers and missed < 20:
        trk.track({"dets": np.zeros((0, 7)), "info": np.zeros((0, 3), dtype=np.int64)}, k)
        missed += 1
        k += stride
    return first, missed


def main():
    ok = True
    for s in (2, 3, 4, 7):
        dx, dp = predict_equivalence(s)
        good = dx < 1e-9 and dp < 1e-12
        ok &= good
        print(f"predict over {s} ticks against {s} one-tick predicts: state diff {dx:.2e}, "
              f"relative covariance diff {dp:.2e}  {'ok' if good else 'FAIL'}")
    ref = call_counts(1)
    for s in (1, 3, 4):
        got = call_counts(s)
        good = got == ref
        ok &= good
        print(f"stride {s}: published at call {got[0]}, removed after {got[1]} calls without a match "
              f"({got[0] * s * 50} ms and {got[1] * s * 50} ms)  {'ok' if good else 'FAIL'}")
    prev = previous_model()
    if prev is None:
        print("identity at one tick: skipped (no git source for", PREV_COMMIT + ")")
    else:
        for seed in range(6):
            for edges in (False, True):
                for time_rules in (True, False):
                    bad = H.identical_at_one_tick(prev, seed, edges, 0.3 if seed % 2 else 0.0,
                                                  time_rules=time_rules)
                    good = bad is None
                    ok &= good
                    print(f"identity at one tick against {PREV_COMMIT}, seed {seed}, "
                          f"{'edges' if edges else 'no edges'}, time_rules {'on' if time_rules else 'off'}: "
                          f"{'identical' if good else 'DIFFERS at tick ' + str(bad)}")
    if not ok:
        sys.exit(1)
    print("PASS")


def test_tracker_predict_interval():
    main()


if __name__ == "__main__":
    main()
