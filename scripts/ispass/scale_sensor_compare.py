#!/usr/bin/env python3
"""Compare the sensor data two launch_scale.sh perception runs fed to detection (clients run with SENSOR_FRAME_TRACE=1).

    python scripts/ispass/scale_sensor_compare.py <run dir A> <run dir B> [--json out.json]

Every detection call logs the world frame, the frame and an MD5 of each camera image, of the LiDAR sweep, and of
the detector's boxes. Per vehicle the calls are matched in order. Reported: how many simulator frames old the
camera and LiDAR data are at detection in each run, and in what share of calls the camera images, the LiDAR sweep
and the detector output are identical between the runs.
"""
import ast
import glob
import json
import os
import re
import sys
from collections import Counter

PAT = re.compile(r"\[SENSOR-TRACE\] id=(\S+) world_frame=(\d+) cams=(\[.*?\]) lidar=(\(.*?\)) yolo=(\w+)")


def load(d):
    out = {}
    for f in glob.glob(os.path.join(d, "*", "veh_*.log")):
        k = int(re.search(r"veh_(\d+)\.log$", f).group(1)); rows = []
        for line in open(f, errors="ignore"):
            m = PAT.search(line)
            if m:
                try:
                    rows.append((int(m.group(2)), ast.literal_eval(m.group(3)), ast.literal_eval(m.group(4)), m.group(5)))
                except (ValueError, SyntaxError):
                    pass
        out[k] = rows
    return out


def ages(runs):
    cam, lid = Counter(), Counter()
    for rows in runs.values():
        for wf, cams, lidar, _ in rows:
            for fr, _ in cams:
                cam[wf - fr] += 1
            lid[wf - lidar[0]] += 1
    pct = lambda c: {str(k): round(100.0 * v / sum(c.values()), 1) for k, v in sorted(c.items())} if c else None
    return {"camera_frame_age_pct": pct(cam), "lidar_frame_age_pct": pct(lid)}


def main():
    a, b = load(sys.argv[1]), load(sys.argv[2])
    n = same_cam = same_all_cams = same_lidar = same_yolo = cams = 0
    for k in sorted(set(a) & set(b)):
        for ra, rb in zip(a[k], b[k]):
            n += 1
            eq = [x[1] == y[1] for x, y in zip(ra[1], rb[1])]
            cams += len(eq); same_cam += sum(eq); same_all_cams += all(eq)
            same_lidar += ra[2][1] == rb[2][1]; same_yolo += ra[3] == rb[3]
    pc = lambda x, t: round(100.0 * x / t, 1) if t else None
    res = {"a": sys.argv[1], "b": sys.argv[2], "vehicles": len(set(a) & set(b)), "calls_compared": n,
           "a_ages": ages(a), "b_ages": ages(b),
           "camera_images_identical_pct": pc(same_cam, cams), "all_four_cameras_identical_pct": pc(same_all_cams, n),
           "lidar_identical_pct": pc(same_lidar, n), "detector_output_identical_pct": pc(same_yolo, n)}
    print(json.dumps(res, indent=1))
    if "--json" in sys.argv:
        json.dump(res, open(sys.argv[sys.argv.index("--json") + 1], "w"), indent=1)


if __name__ == "__main__":
    main()
