"""Zero-age reaction margin of the reference per run (hard-encounter calibration, Oct 30 2026).
From the per-tick log: the launch tick (first motion of the conflict actor), the brake tick (first brake_event on the
conflict actor at or after the launch), the ego speed at the brake, and the ego's stopping point (first tick after the
brake with speed under 0.3 m/s). The margin is the distance left between the ego's front at the stop and the crossing
car's near flank (y 126.75 on both junctions), divided by the ego speed at the brake: the time the brake could have
come later and still stopped short. A run that never stops short (collision, or still moving when its front passes the
flank) reports a negative margin from the overlap. Prints one line per run and the mean; exit code 0.
Usage: zero_age_margin.py <experiment_results root> <test> <stamp> <speed> [mgr=oracle] [anch=on] [lat=0]"""
import csv, glob, math, os, sys
root, test, stamp, speed = sys.argv[1:5]
mgr = sys.argv[5] if len(sys.argv) > 5 else "oracle"; anch = sys.argv[6] if len(sys.argv) > 6 else "on"; lat = sys.argv[7] if len(sys.argv) > 7 else "0"
FLANK_Y = 127.7 - 0.95; HALF_LEN = 4.9 / 2
vals = []
for d in sorted(glob.glob(f"{root}/{test}/{stamp}/speed_spd_{speed}/baseline_default/mgr_{mgr}/anchoring_{anch}/lat_{lat}_*/run_*"), key=lambda p: int(p.rsplit("_", 1)[1])):
    f = d + "/per_tick_age.csv"
    if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
    if not os.path.exists(f): print(d.rsplit("/", 1)[1], "no per-tick log"); continue
    rows = list(csv.DictReader(open(f))); per = {}
    for r in rows: per.setdefault(int(r["tick"]), r)
    ticks = sorted(per); g0 = None; launch = None
    for t in ticks:
        r = per[t]
        try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
        except Exception: continue
        if g0 is None: g0 = (gx, gy)
        if math.hypot(gx - g0[0], gy - g0[1]) > 0.3: launch = t; break
    if launch is None: print(d.rsplit("/", 1)[1], "no launch"); continue
    brake = next((int(r["tick"]) for r in rows if int(r["tick"]) >= launch and r.get("brake_event") == "1" and r.get("is_conflict_actor") == "1"), None)
    if brake is None: print(d.rsplit("/", 1)[1], "no brake on the conflict actor"); vals.append(float("nan")); continue
    vb = float(per[brake]["ego_speed_mps"]); yb = float(per[brake]["ego_y"])
    stop = next((t for t in ticks if t > brake and float(per[t]["ego_speed_mps"]) < 0.3), None)
    cross = next((t for t in ticks if t > brake and float(per[t]["ego_y"]) + HALF_LEN >= FLANK_Y), None)
    if stop is not None and (cross is None or stop <= cross):
        ys = float(per[stop]["ego_y"]); margin_m = FLANK_Y - (ys + HALF_LEN); margin_s = margin_m / max(vb, 0.1); kind = "stopped"
    else:
        t_c = cross if cross is not None else ticks[-1]; vc = float(per[t_c]["ego_speed_mps"])
        margin_m = -(vc * vc / 16.0); margin_s = -(vc / 8.0); kind = "crossed at %.1f m/s" % vc   # overlap read as the stop it still needed at 8 m/s^2
    vals.append(margin_s)
    print(f"{d.rsplit('/', 1)[1]}: launch t{launch} brake +{(brake-launch)*0.05:.2f} s at y {yb:.1f} v {vb:.1f}; {kind}; margin {margin_m:+.1f} m = {margin_s:+.2f} s")
good = [v for v in vals if v == v]
print(f"MARGIN_MEAN {sum(good)/len(good) if good else float('nan'):.3f} n {len(good)}")
