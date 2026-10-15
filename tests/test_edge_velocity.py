"""Published velocity from the slower-moving edge of the measured box: a
parked truck whose visible side grows as a sensor closes on it is published at
rest, a car that moves both edges is published at its speed, and a bundle
without edges keeps the position-displacement estimate. Runs standalone."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np  # noqa: E402
from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper  # noqa: E402

DT = 0.05


def truck_box(k):
    """Rear face fixed at y=88.5, visible side growing north 0.36 m per frame."""
    ymin, ymax = 88.5, min(96.9, 89.5 + 0.36 * k)
    return (-82.45, -79.55, ymin, ymax)


def car_box(k):
    x = -37.0 - 0.65 * k
    return (x - 2.25, x + 2.25, 126.7, 128.7)


def det_from_box(b):
    xmin, xmax, ymin, ymax = b
    # [h, w, l, x, y, z, theta]: KITTI y is height, KITTI z is CARLA y
    return [1.8, xmax - xmin, ymax - ymin, (xmin + xmax) / 2, 0.0, (ymin + ymax) / 2, 0.0]


def run(with_edges, frames=24):
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    speeds = {}   # frame -> {"truck": v, "car": v}
    for k in range(1, frames + 1):
        boxes = [truck_box(k), car_box(k)]
        dets = np.array([det_from_box(b) for b in boxes], dtype=float)
        info = np.array([[k, 100 + k, -1], [k, 200 + k, -1]], dtype=np.int64)
        bundle = {"dets": dets, "info": info}
        if with_edges:
            bundle["edges"] = np.array(boxes, dtype=float)
        out, _ = trk.track(bundle, k)
        rows = out[0] if out and len(out[0]) else np.zeros((0, 16))
        for r in rows:
            v = math.hypot(float(r[10]), float(r[12])) / DT
            name = "truck" if float(r[5]) < 110 else "car"
            speeds.setdefault(k, {})[name] = v
    return speeds


def slower_edge_rule():
    s = run(with_edges=True)
    late = [k for k in s if k >= 14]
    assert late, "no output frames"
    for k in late:
        assert "truck" in s[k] and "car" in s[k], (k, s[k])
        assert s[k]["truck"] == 0.0, (k, s[k]["truck"])
        assert abs(s[k]["car"] - 13.0) < 1.5, (k, s[k]["car"])
    print("with edges: truck at rest (0.0 m/s) in frames 14..24, car at %.1f m/s" % s[24]["car"])


def fallback_without_edges():
    s = run(with_edges=False)
    v = s[20]["truck"]
    assert v > 2.0, v
    assert abs(s[20]["car"] - 13.0) < 1.5, s[20]["car"]
    print("without edges: the sliding centre of the truck is published at %.1f m/s (position-displacement path)" % v)




def _feed(trk, k, boxes):
    """Feed one frame of boxes (world xmin, xmax, ymin, ymax) with edges; return (tid, x, y, speed) rows."""
    dets = np.array([det_from_box(b) for b in boxes], dtype=float).reshape(len(boxes), 7)
    info = np.array([[k, 1000 * k + i, -1] for i in range(len(boxes))], dtype=np.int64).reshape(len(boxes), 3)
    bundle = {"dets": dets, "info": info, "edges": np.array(boxes, dtype=float).reshape(len(boxes), 4)}
    out, _ = trk.track(bundle, k)
    rows = out[0] if out and len(out[0]) else np.zeros((0, 16))
    return [(int(r[7]), float(r[3]), float(r[5]), math.hypot(float(r[10]), float(r[12])) / DT) for r in rows]


def walking_reports_do_not_move_a_resting_track():
    """A roadside sensor reports a parked truck at rest; during a 3-frame
    dropout a passing vehicle's reports of the column walk 1.8 m per frame.
    The resting track must not follow them, and no published track may carry
    the walk as velocity."""
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    rest = (-82.45, -79.55, 106.5, 114.9)          # truck centred (-81.0, 110.7)
    worst = 0.0
    for k in range(1, 31):
        boxes = []
        if not 13 <= k <= 15:
            boxes.append(rest)
        if k >= 13:
            y = 109.9 - 1.8 * (k - 13)
            boxes.append((-83.3, -81.3, y - 1.0, y + 1.0))
        rows = _feed(trk, k, boxes)
        for tid, x, y, v in rows:
            worst = max(worst, v)
            if abs(x + 81.0) < 1.5 and abs(y - 110.7) < 1.5:
                assert v == 0.0, (k, tid, v)
    assert worst == 0.0, worst
    print("walking reports beside a resting track: every published track at rest (max %.1f m/s)" % worst)


def fast_car_is_tracked_and_published():
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    ids, late = set(), []
    for k in range(1, 25):
        x = -37.0 - 1.2 * k                       # 24 m/s
        rows = _feed(trk, k, [(x - 2.25, x + 2.25, 126.7, 128.7)])
        for tid, _, _, v in rows:
            ids.add(tid)
            if k >= 14:
                late.append(v)
    assert len(ids) == 1, ids
    assert late and all(abs(v - 24.0) < 2.5 for v in late), late
    print("24 m/s car: one track, published %.1f m/s from frame 14" % late[-1])


def hop_restarts_the_estimate():
    """A moving track whose box hops 2.5 m beyond its expected motion in one
    frame (inside the association gate) restarts its velocity estimate."""
    w = AB3DMOTWrapper({"max_age": 10, "min_hits": 3, "anchoring": False})
    trk = w._tracker
    speeds = {}
    off = 0.0
    for k in range(1, 40):
        if k == 14:
            off = 2.5
        x = -37.0 - 0.5 * k - off                 # 10 m/s, then a 2.5 m hop
        rows = _feed(trk, k, [(x - 2.25, x + 2.25, 126.7, 128.7)])
        for _, _, _, v in rows:
            speeds[k] = v
    assert all(abs(speeds[k] - 10.0) < 1.5 for k in range(11, 14)), {k: speeds[k] for k in range(11, 14)}
    assert all(speeds[k] == 0.0 for k in range(15, 23)), {k: speeds[k] for k in range(15, 23)}
    assert all(abs(speeds[k] - 10.0) < 1.5 for k in range(28, 39)), {k: speeds[k] for k in range(28, 39)}
    print("hop: published %.1f m/s before, 0 for the next ten matched frames, %.1f m/s after" % (speeds[12], speeds[38]))


if __name__ == "__main__":
    slower_edge_rule()
    fallback_without_edges()
    walking_reports_do_not_move_a_resting_track()
    fast_car_is_tracked_and_published()
    hop_restarts_the_estimate()
