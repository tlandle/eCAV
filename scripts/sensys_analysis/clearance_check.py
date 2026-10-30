"""Footprint clearance and box timing per run (SenSys j6 diagnostic, Oct 30 2026).

Usage: clearance_check.py <root> <test> <mgr> <stamp or glob> <speeds,> <lats,> [reps,]
Axis-aligned footprints (ego heading +y, car heading -x): ego 2.1 x 4.9 m, car 4.7 x 1.9 m. Prints per run the
minimum footprint gap and when it occurs, the ego speed there, when the ego front reaches the box near edge
(y = 125.95) and when the car rear clears the ego lane, plus the simulator collision count. A run that completes
with a gap under 1 m after the car cleared the lane is the ego passing behind the car (the escape band)."""
import csv, glob, math, sys, os
root, test, mgr, stamp = sys.argv[1:5]
speeds = sys.argv[5].split(","); lats = sys.argv[6].split(","); reps = sys.argv[7].split(",") if len(sys.argv) > 7 else None
EL, EW, CL, CW = 4.9, 2.1, 4.7, 1.9
BOX_Y = 127.7 - 1.75
def gap(ex, ey, cx, cy):
    dx = max(abs(ex - cx) - (EW + CL) / 2, 0.0); dy = max(abs(ey - cy) - (EL + CW) / 2, 0.0)
    return math.hypot(dx, dy)
for s in speeds:
    for lat in lats:
        for d in sorted(glob.glob(f"{root}/{test}/{stamp}/speed_spd_{s}/baseline_default/mgr_{mgr}/anchoring_on/lat_{lat}_*/run_*"), key=lambda p: int(p.rsplit("_", 1)[1])):
            rep = d.rsplit("_", 1)[1]
            if reps and rep not in reps: continue
            f = d + "/per_tick_age.csv"
            if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
            if not os.path.exists(f): continue
            per = {}
            for r in csv.DictReader(open(f)):
                per.setdefault(int(r["tick"]), r)
            ticks = sorted(per); gx0 = gy0 = None; launch = None
            for t in ticks:
                r = per[t]
                try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
                except: continue
                if gx0 is None: gx0, gy0 = gx, gy
                if math.hypot(gx - gx0, gy - gy0) > 0.3: launch = t; break
            if launch is None: continue
            gmin, tmin, vmin = 1e9, None, None; t_front = None; t_clear = None
            for t in ticks:
                r = per[t]
                try: ex, ey, cx, cy, v = (float(r[k]) for k in ("ego_x", "ego_y", "conflict_gt_x", "conflict_gt_y", "ego_speed_mps"))
                except: continue
                g = gap(ex, ey, cx, cy)
                if g < gmin: gmin, tmin, vmin = g, t, v
                if t_front is None and ey + EL / 2 >= BOX_Y and t >= launch: t_front = t
                if t_clear is None and t >= launch and cx + CL / 2 < ex - EW / 2: t_clear = t
            import json
            m = json.load(open(d + "/simulation_metrics.json"))
            coll = sum(int(v.get("collisions", 0) or 0) for v in m.get("vehicles", {}).values())
            print(f"{s:>2} {lat:>4} r{rep:<2} gap_min {gmin:4.2f} m @+{(tmin-launch)*0.05:4.2f}s v={vmin:4.1f}  ego front at box +{(t_front-launch)*0.05 if t_front else -1:4.2f}s  car clears ego lane +{(t_clear-launch)*0.05 if t_clear else -1:4.2f}s  sim_collisions={coll}")
