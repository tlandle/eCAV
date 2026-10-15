#!/usr/bin/env python3
"""Causal trace of one run's first brake: tick by tick, the consumed tracks
near the focal vehicle (position along the lane relative to the vehicle, age
at use, provenance, published speed) and the planner's decision, from the
per-tick age log and the vehicle's own pose lines.

    python scripts/sensys_analysis/self_ghost_trace.py RUN_DIR [--before 10] [--after 2] [--radius 12]

For reviewer B's question the same trace on the naive, VIPS, and VIPS-with-
contract runs of one cell shows what time rectification moves and what it
leaves: in the naive arm the vehicle's own state consumed 400 ms late sits
5 m behind it and is taken for another vehicle.
"""
import argparse
import csv
import math
import os
import re

LOC_RE = re.compile(r"\[LOC\] v=(\d+) t=(\d+) gnss=\(([-\d.]+),([-\d.]+)\) gt=\(([-\d.]+),([-\d.]+)\)")


BEACON_RE = re.compile(r"\[DET\] tick=(\d+) src=beacon x=([-\d.]+) y=([-\d.]+) conf=[\d.]+ cid=(-?\d+)")


def ego_poses(run_dir):
    """{vehicle_id: {tick: (x, y)}} from the [LOC] lines (ground truth, logged
    for the first 100 ticks only) extended by the vehicle's own beacon rows in
    the edge's [DET] log (GNSS position, every tick it beacons). With one
    managed vehicle every beacon is the focal vehicle's; with more, the
    beacon rows are keyed by their pseudonym and only [LOC] is used."""
    poses, beacons = {}, {}
    with open(os.path.join(run_dir, "runner_stdout.txt"), errors="replace") as fh:
        for line in fh:
            m = LOC_RE.search(line)
            if m:
                poses.setdefault(int(m.group(1)), {})[int(m.group(2))] = (float(m.group(5)), float(m.group(6)))
                continue
            b = BEACON_RE.search(line)
            if b:
                beacons.setdefault(int(b.group(1)), []).append((float(b.group(2)), float(b.group(3))))
    if len(poses) == 1 and all(len(v) == 1 for v in beacons.values()):
        vid = next(iter(poses))
        for t, v in beacons.items():
            poses[vid].setdefault(t, v[0])
    return poses


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--before", type=int, default=10)
    ap.add_argument("--after", type=int, default=2)
    ap.add_argument("--radius", type=float, default=12.0)
    a = ap.parse_args()
    csv_path = os.path.join(a.run, "evaluation_output", "per_tick_age.csv")
    if not os.path.exists(csv_path):
        csv_path = os.path.join(a.run, "per_tick_age.csv")
    rows = list(csv.DictReader(open(csv_path, newline="")))
    focal = min(int(r["focal_vehicle_id"]) for r in rows if r["focal_vehicle_id"])
    rows = [r for r in rows if int(r["focal_vehicle_id"]) == focal]
    poses = ego_poses(a.run).get(focal, {})
    by_tick = {}
    for r in rows:
        by_tick.setdefault(int(r["tick"]), []).append(r)
    brake = next((t for t in sorted(by_tick) if any(r.get("brake_event") == "1" for r in by_tick[t])), None)
    if brake is None:
        print("no brake event in this run")
        return
    cls = next((r.get("gt_brake_class") for r in by_tick[brake] if r.get("brake_event") == "1"), "")
    print(f"focal {focal}; first brake at tick {brake} ({cls}); ego pose from [LOC]; along = metres ahead of the ego along its heading")
    prev = None
    for t in range(brake - a.before, brake + a.after + 1):
        if t not in by_tick:
            continue
        ego = poses.get(t)
        heading = None
        if ego and prev and prev != ego:
            heading = math.atan2(ego[1] - prev[1], ego[0] - prev[0])
        prev = ego or prev
        dec = by_tick[t][0].get("decision", "")
        ev = " BRAKE" if any(r.get("brake_event") == "1" for r in by_tick[t]) else ""
        line = [f"t={t:4d} ego=({ego[0]:.1f},{ego[1]:.1f}) {dec}{ev}" if ego else f"t={t:4d} {dec}{ev}"]
        for r in by_tick[t]:
            if r.get("source") != "edge" or not r.get("obs_x"):
                continue
            x, y = float(r["obs_x"]), float(r["obs_y"])
            if ego is None:
                continue
            dx, dy = x - ego[0], y - ego[1]
            d = math.hypot(dx, dy)
            if d > a.radius:
                continue
            if heading is not None:
                along = dx * math.cos(heading) + dy * math.sin(heading)
                lat = -dx * math.sin(heading) + dy * math.cos(heading)
                rel = f"along {along:+.1f} lat {lat:+.1f}"
            else:
                rel = f"d {d:.1f}"
            line.append(f"  trk {r['track_id']} cid {r['carla_id'] or '-'} {r['provenance']} age {float(r['age_at_use_ms']):.0f} ms {rel} v {float(r['obs_speed_mps'] or 0):.1f}")
        print("\n".join(line))


if __name__ == "__main__":
    main()
