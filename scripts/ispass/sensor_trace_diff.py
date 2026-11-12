#!/usr/bin/env python
"""Attribute a perception difference between two repeated runs (SENSOR_FRAME_TRACE=1).

    python scripts/ispass/sensor_trace_diff.py <stampA> <stampB> [--root experiment_results/<geometry>]

Reads the [SENSOR-TRACE] lines of run_1 under each stamp (one per detector call: the world
frame, the simulator frame and a digest of each camera image, of the lidar sweep, and of
the detector's boxes rounded to 0.01 px) and reports, call by call in order: how often a
sensor's data is from an earlier frame than the world frame, how often the camera images,
the lidar sweeps and the detector output differ between the runs, and how often the
detector output differs when every camera image is identical.
"""
import argparse, ast, glob, os, re, sys
from collections import Counter

PAT = re.compile(r"\[SENSOR-TRACE\] id=(\S+) world_frame=(\d+) cams=(\[.*?\]) lidar=(\(.*?\)) yolo=(\w+)")


def load(stamp, root):
    f = glob.glob(os.path.join(root, stamp, "**", "run_1", "runner_stdout.txt"), recursive=True)
    if not f:
        sys.exit(f"no run under {root}/{stamp}")
    rows = []
    for line in open(f[0], errors="ignore"):
        m = PAT.search(line)
        if m:
            rows.append((m.group(1), int(m.group(2)), ast.literal_eval(m.group(3)),
                         ast.literal_eval(m.group(4)), m.group(5)))
    if not rows:
        sys.exit(f"no [SENSOR-TRACE] lines under {stamp} (run with SENSOR_FRAME_TRACE=1)")
    f0 = rows[0][1]
    return [dict(frame=wf - f0,
                 cam_lag=tuple((cf - wf) if cf is not None else None for cf, _ in cams),
                 lidar_lag=(lid[0] - wf) if lid[0] is not None else None,
                 cam=tuple(h for _, h in cams), lidar=lid[1], det=det)
            for (_, wf, cams, lid, det) in rows]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--root", default="experiment_results/highway_merge_edge_late_fusion")
    a = ap.parse_args()
    A, B = load(a.a, a.root), load(a.b, a.root)
    for name, R in ((a.a, A), (a.b, B)):
        print(f"{name}: {len(R)} detector calls; camera frame lag {Counter(x for r in R for x in r['cam_lag']).most_common(4)}; "
              f"lidar frame lag {Counter(r['lidar_lag'] for r in R).most_common(4)}")
    n = min(len(A), len(B))
    c = Counter(); first = {}
    for k in range(n):
        x, y = A[k], B[k]
        def hit(key):
            c[key] += 1; first.setdefault(key, x["frame"])
        if x["frame"] != y["frame"]: hit("call order")
        if x["cam_lag"] != y["cam_lag"] or x["lidar_lag"] != y["lidar_lag"]: hit("sensor frame lag")
        if x["cam"] != y["cam"]: hit("camera image")
        elif x["det"] != y["det"]: hit("detector output on identical images")
        if x["lidar"] != y["lidar"]: hit("lidar sweep")
        if x["det"] != y["det"]: hit("detector output")
    print(f"calls compared: {n}")
    for key in ("call order", "sensor frame lag", "camera image", "lidar sweep", "detector output",
                "detector output on identical images"):
        print(f"  {key:<38}: differs on {c[key]:>5} calls ({100.0 * c[key] / n:5.1f} %), first at frame "
              f"{first.get(key, 'none')}")


if __name__ == "__main__":
    main()
