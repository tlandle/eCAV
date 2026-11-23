#!/usr/bin/env python
"""Launch leads for the left turn (ltp_column) from runs without the crossing car.

    python scripts/ispass/left_turn_calibrate.py <stem> [--speeds 40 50 60 70 80] [--car-speed 13.4]

The crossing car starts at (-35.0, 127.7) and drives west at a constant speed. The launch rule of
scp_edge_late_fusion.SyncCrossLaunch fires when the vehicle's straight-line distance to (-84.8, 127.9) over its
speed falls to the lead. The turning vehicle slows before the junction, so a lead equal to the car's travel time
does not put the two on a collision course. For each speed this script reads the vehicle's path from the per-tick
log of a run in which the car never launches (cell <stem><speed>_gt_vis_n1), finds the tick at which the vehicle
crosses the car's lane centre (y = 127.7) and the x at which it does, and prints the lead that launches the car so
that it reaches that point in the same tick.
"""
import argparse, csv, glob, math, os

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
CONFLICT = (-84.8, 127.9); START_X = -35.0; LANE_Y = 127.7; DT = 0.05


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("stem"); ap.add_argument("--speeds", type=int, nargs="+", default=[40, 50, 60, 70, 80])
    ap.add_argument("--car-speed", type=float, default=13.4); a = ap.parse_args(); out = []
    for spd in a.speeds:
        fs = sorted(glob.glob(os.path.join(ROOT, "experiment_results", "*", f"{a.stem}{spd}_gt_vis_n1", "**", "evaluation_output", "per_tick_age.csv"), recursive=True))
        if not fs: print(spd, "no run"); continue
        ego = {}
        for r in csv.DictReader(open(fs[0])):
            if r.get("ego_x") and r.get("ego_y"): ego[int(r["tick"])] = (float(r["ego_x"]), float(r["ego_y"]), float(r["ego_speed_mps"]))
        ts = sorted(ego); cross = next((t for t in ts if ego[t][1] >= LANE_Y), None)
        if cross is None: print(spd, "the vehicle does not reach the car's lane; last", ego[ts[-1]]); continue
        xc = ego[cross][0]; travel = (START_X - xc) / a.car_speed; t0 = cross - int(round(travel / DT))
        if t0 not in ego: print(spd, "launch tick outside the log"); continue
        x, y, v = ego[t0]; lead = math.hypot(x - CONFLICT[0], y - CONFLICT[1]) / v
        print(f"{spd} km/h: crosses y={LANE_Y} at tick {cross}, x {xc:.1f}, speed {ego[cross][2]:.1f} m/s; car travel {travel:.2f} s; launch tick {t0} "
              f"at ({x:.1f},{y:.1f}) speed {v:.1f} m/s; lead {lead:.2f} s; final ({ego[ts[-1]][0]:.0f},{ego[ts[-1]][1]:.0f}) at tick {ts[-1]}")
        out.append(f"{spd}:{lead:.2f}")
    print("LEADS " + " ".join(out))


if __name__ == "__main__":
    main()
