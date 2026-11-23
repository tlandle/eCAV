#!/usr/bin/env python
"""When the first vehicle of a column decides to brake for the crossing car (column_sweep.sh with COLUMN_TRACE=1).

    python scripts/ispass/column_lead_brake.py <stem> --speed 60 --ns 1 2 4 8 --runs-csv <runs.csv of column_trace_reduce.py> [--root <checkout>]

column_trace_reduce.py takes the brake start from the control signal (three ticks with brake >= 0.5), which also
catches a brake of the speed controller: a lone vehicle that overshoots its cruise speed brakes for three ticks about
0.45 s before the crossing car enters its view. This script takes the planner's decision from per_tick_age.csv: the
first tick after the launch of the crossing car with a brake event on the crossing car's track. It prints, per arm and
column length, the median, minimum and maximum over the runs, the first observation of the crossing car, and the
decision times of the runs in which the first vehicle collided and did not collide.
"""
import argparse, csv, glob, math, os, statistics as st

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DT = 0.05


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stem"); ap.add_argument("--speed", default="60"); ap.add_argument("--ns", type=int, nargs="+", default=[1, 2, 4, 8])
    ap.add_argument("--arms", nargs="+", default=["gt_all", "gt_vis", "onboard"]); ap.add_argument("--runs-csv", required=True)
    ap.add_argument("--scenario", default="scp_colu"); ap.add_argument("--csv")
    ap.add_argument("--root", default=ROOT, help="checkout that holds experiment_results (default: the one of the script)")
    a = ap.parse_args()
    coll = {(r["arm"], int(r["n"]), int(r["rep"])): int(r["lead"]) for r in csv.DictReader(open(a.runs_csv))}
    out = []
    print("arm N | decision s after launch: median [min, max] | first observation: median | collided / not collided")
    for arm in a.arms:
        for n in a.ns:
            cell = os.path.join(a.root, "experiment_results", a.scenario + a.speed, f"{a.stem}{a.speed}_{arm}_n{n}")
            be, fo, byc = [], [], {0: [], 1: []}
            for f in sorted(glob.glob(os.path.join(cell, "**", "run_*", "evaluation_output", "per_tick_age.csv"), recursive=True)):
                run = int(f.split("run_")[-1].split(os.sep)[0])
                rows = list(csv.DictReader(open(f)))
                gt = {}
                for r in rows:
                    if r["conflict_gt_x"]: gt.setdefault(int(r["tick"]), (float(r["conflict_gt_x"]), float(r["conflict_gt_y"])))
                launch = next((t for t in sorted(gt) if t - 1 in gt and math.hypot(gt[t][0] - gt[t - 1][0], gt[t][1] - gt[t - 1][1]) > 0.02), None)
                if launch is None: continue
                ys = {r["focal_vehicle_id"]: float(r["ego_y"]) for r in rows if int(r["tick"]) == launch}
                lead = max(ys, key=ys.get)
                rr = [r for r in rows if r["focal_vehicle_id"] == lead and int(r["tick"]) >= launch]
                b = next((int(r["tick"]) for r in rr if r["brake_event"] == "1" and r["is_conflict_actor"] == "1"), None)
                o = next((int(r["tick"]) for r in rr if r["is_conflict_actor"] == "1" and r["obs_x"]), None)
                if o is not None: fo.append((o - launch) * DT)
                if b is None: continue
                be.append((b - launch) * DT)
                c = coll.get((arm, n, run))
                if c is not None: byc[1 if c else 0].append(round((b - launch) * DT, 2))
                out.append(dict(arm=arm, n=n, rep=run, decision_s=round((b - launch) * DT, 2), lead_collided=c))
            if not be: continue
            print(f"{arm} {n} | {st.median(be):.2f} [{min(be):.2f}, {max(be):.2f}] n={len(be)} | {st.median(fo):.2f} | {sorted(byc[1])} / {sorted(byc[0])}")
    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["arm", "n", "rep", "decision_s", "lead_collided"]); w.writeheader(); w.writerows(out)
        print("wrote", a.csv)


if __name__ == "__main__":
    main()
