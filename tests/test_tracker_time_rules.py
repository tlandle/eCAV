"""The tracker's time rules hold in time, whatever the stride between the frames it is fed.

AB3DMOT is fed every tick by the object-level managers and every third or fourth tick by the
intermediate-fusion manager under service occupancy (145 ms of compute). Its published velocity
(row columns 10 and 12) is in metres per source tick, and the rules its comments state in time
(half a second of history before a velocity is published, the half-second velocity window) are
counted in source ticks. Checked here:

1. A car at a constant 13 m/s is published at 13 m/s at strides 1, 3, 4 and the mixed strides
   that occupancy produces, so the linear predictor (position + kf_vx * (i + 1) per tick) covers
   13 m in one second. Before 2026-11-12 the velocity was per call and the forecast ran at 3 to 4
   times the car's speed.
2. A car at rest that launches at 13 m/s is published as moving after the same time at strides
   1, 3 and 4, to within one stride. Before, the ten-entry rule took ten calls: 0.5 s at one tick
   per call, 1.5 to 2 s at three or four.
3. With measured box edges (the object-level managers, one tick per call) the constant and the
   launched car are published as before.
4. At a one-tick stride the output is identical, bit for bit, to the tracker before this change
   (commit 6b11eb18), on noisy sequences with dropped frames, with and without edges. Skipped when
   git cannot provide the earlier source. The time index is an option of the tracker (time_rules),
   set by the intermediate-fusion manager. With the option off the tracker is the earlier one for
   any frame numbering, including frames ingested out of source-tick order.

Runs standalone."""
import math
import os
import subprocess
import sys
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import numpy as np  # noqa: E402
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402
import AB3DMOT_libs.model as model_now  # noqa: E402

DT = 0.05
V = 13.0
PREV_COMMIT = "6b11eb18"


def new_tracker(time_rules=True):
    return AB3DMOTWrapper({"max_age": 3, "min_hits": 3, "anchoring": False,
                           "time_rules": time_rules})._tracker


def ticks_for(strides, last):
    out, t, i = [], 1, 0
    while t <= last:
        out.append(t)
        t += strides[i % len(strides)]
        i += 1
    return out


def bundle(x, k, edges):
    b = {"dets": np.array([[1.8, 2.0, 4.5, x, 0.0, 127.7, 0.0]], dtype=float),
         "info": np.array([[k, 100 + k, -1]], dtype=np.int64)}
    if edges:
        b["edges"] = np.array([[x - 2.25, x + 2.25, 126.7, 128.7]], dtype=float)
    return b


def speed_of(out):
    """Largest published speed (m/s) over the rows of one call, or None without rows."""
    if not out or not len(out[0]):
        return None
    return max(math.hypot(float(r[10]), float(r[12])) for r in out[0]) / DT


def constant(strides, edges):
    trk = new_tracker()
    sp = None
    for k in ticks_for(strides, 400):
        out, _ = trk.track(bundle(V * k * DT, k, edges), k)
        sp = speed_of(out)
    return sp


def launch(strides, edges, t0=101):
    """Ticks from the launch (first tick the car has moved) to the first call that publishes a
    track faster than 2 m/s."""
    trk = new_tracker()
    for k in ticks_for(strides, 400):
        x = 0.0 if k < t0 else V * (k - t0 + 1) * DT
        out, _ = trk.track(bundle(x, k, edges), k)
        sp = speed_of(out)
        if k >= t0 and sp is not None and sp > 2.0:
            return k - t0
    return None


def previous_model():
    try:
        src = subprocess.run(["git", "-C", ROOT, "show", f"{PREV_COMMIT}:AB3DMOT_libs/model.py"],
                             capture_output=True, text=True, timeout=20, check=True).stdout
    except Exception:
        return None
    m = types.ModuleType("ab3dmot_model_prev")
    m.__file__ = os.path.join(ROOT, "AB3DMOT_libs", "model.py")
    exec(compile(src, "ab3dmot_model_prev", "exec"), m.__dict__)
    return m


def identical_at_one_tick(prev, seed, edges, drop, time_rules=True, shuffle=False):
    """First tick at which the published rows or the filter states differ from the earlier
    tracker, or None. With shuffle the frame numbers passed to the tracker are out of order
    now and then, as when the object-level managers ingest frames in arrival order; the
    option is off there."""
    rng = np.random.default_rng(seed)
    a = new_tracker(time_rules)
    b = new_tracker(time_rules)
    b.__class__ = prev.AB3DMOT
    x2 = 40.0
    for k in range(1, 260):
        v = 0.0 if k < 60 else min(13.5, 7.0 * (k - 60) * DT)
        x2 += v * DT
        rows, eds = [], []
        for cx, cz in ((x2, 127.7), (12.0, 140.0), (0.6 * k * DT * 20 * DT, 150.0)):
            if rng.random() < drop:
                continue
            box = (cx - 2.25 + rng.normal(0, 0.3), cx + 2.25 + rng.normal(0, 0.3),
                   cz - 1.0 + rng.normal(0, 0.3), cz + 1.0 + rng.normal(0, 0.3))
            rows.append([1.8, box[3] - box[2], box[1] - box[0], (box[0] + box[1]) / 2, 0.0,
                         (box[2] + box[3]) / 2, 0.0])
            eds.append(box)
        bd = {"dets": np.array(rows, dtype=float).reshape(-1, 7),
              "info": np.array([[k, 100 + i, -1] for i in range(len(rows))], dtype=np.int64).reshape(-1, 3)}
        if edges:
            bd["edges"] = np.array(eds, dtype=float).reshape(-1, 4)
        import copy
        fr = k + (int(rng.integers(-1, 3)) if shuffle and rng.random() < 0.1 else 0)
        oa, _ = a.track(copy.deepcopy(bd), fr)
        ob, _ = b.track(copy.deepcopy(bd), fr)
        ra = oa[0] if oa and len(oa[0]) else np.zeros((0, 16))
        rb = ob[0] if ob and len(ob[0]) else np.zeros((0, 16))
        if np.asarray(ra).shape != np.asarray(rb).shape or not np.array_equal(np.asarray(ra), np.asarray(rb)):
            return k
        for ta, tb in zip(a.trackers, b.trackers):
            if not (np.array_equal(ta.kf.x, tb.kf.x) and np.array_equal(ta.kf.P, tb.kf.P)):
                return k
    return None


def main():
    ok = True
    for edges in (False, True):
        tag = "edges" if edges else "no edges"
        # with measured edges the young-track gate of the association (1.5 m per call) rejects a
        # 13 m/s car at three ticks per call; that path runs at one tick per call
        cases = (("stride 1", [1]),) if edges else (("stride 1", [1]), ("stride 3", [3]), ("stride 4", [4]),
                                                    ("strides 3,4", [3, 4]), ("strides 3,3,4", [3, 3, 4]))
        for name, strides in cases:
            sp = constant(strides, edges)
            good = sp is not None and abs(sp - V) <= 0.005 * V
            ok &= good
            print(f"constant 13 m/s, {tag:8s} {name:14s} published {sp if sp is None else round(sp, 3)} m/s, "
                  f"forecast {0 if sp is None else round(sp * 1.0, 3)} m per second  {'ok' if good else 'FAIL'}")
        base = launch([1], edges)
        for name, strides in ((("stride 1", [1]),) if edges else (("stride 1", [1]), ("stride 3", [3]), ("stride 4", [4]))):
            lag = launch(strides, edges)
            good = lag is not None and base is not None and abs(lag - base) <= strides[0]
            ok &= good
            print(f"launch, {tag:8s} {name:9s} first moving publish after "
                  f"{'never' if lag is None else str(lag * 50) + ' ms'}  {'ok' if good else 'FAIL'}")
    prev = previous_model()
    if prev is None:
        print("identity at one tick: skipped (no git source for", PREV_COMMIT + ")")
    else:
        for seed in range(6):
            for edges in (False, True):
                bad = identical_at_one_tick(prev, seed, edges, 0.3 if seed % 2 else 0.0)
                good = bad is None
                ok &= good
                print(f"identity at one tick, seed {seed}, {'edges' if edges else 'no edges'}: "
                      f"{'identical' if good else 'DIFFERS at tick ' + str(bad)}")
                bad = identical_at_one_tick(prev, seed, edges, 0.3 if seed % 2 else 0.0,
                                            time_rules=False, shuffle=True)
                good = bad is None
                ok &= good
                print(f"identity with the option off, frames out of order, seed {seed}, "
                      f"{'edges' if edges else 'no edges'}: {'identical' if good else 'DIFFERS at tick ' + str(bad)}")
    if not ok:
        sys.exit(1)
    print("PASS")


def test_tracker_time_rules():
    main()


if __name__ == "__main__":
    main()
