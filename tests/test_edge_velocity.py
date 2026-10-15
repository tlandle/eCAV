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


if __name__ == "__main__":
    slower_edge_rule()
    fallback_without_edges()
