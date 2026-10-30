"""Per j6s run: on the conflict actor's track at the planner, after the launch and before the ego reaches the box:
collapse = a forecast with speed 0 on the same track id after one with speed > 3 m/s; rebirth = track id change after the
first moving forecast; gen_gap = largest gap in gen_tick between consecutive conflict-track forecasts (ticks).
Usage: [STAMP=sensys_nov27_j6s] track_collapse_scan.py <out.csv> <root> [<root> ...]"""
import csv, glob, math, os, sys
out = sys.argv[1]; roots = sys.argv[2:]; STAMP = os.environ.get("STAMP", "sensys_nov27_j6s")
W = csv.writer(open(out, "w")); W.writerow(["host","geom","mgr","anch","speed","lat","rep","launch","collapse_t","collapse_ticks","rebirth_t","rebirth_gap_ticks","gen_gap","gen_gap_t","brake_t"])
for root in roots:
    host = "cetus" if "cetus" in root else ("azure" if "azure" in root else "atlas")
    for mf in glob.glob(f"{root}/*/{STAMP}/speed_spd_*/baseline_default/mgr_*/anchoring_*/lat_*/run_*/simulation_metrics.json"):
        if "/evaluation_output/" in mf: continue
        d = os.path.dirname(mf); parts = d.split("/")
        geom = parts[-8][:3]; speed = parts[-6].split("_")[-1]; mgr = parts[-4][4:]; anch = parts[-3].split("_")[-1]; lat = parts[-2].split("_")[1]; rep = parts[-1].split("_")[-1]
        f = d + "/per_tick_age.csv"
        if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
        if not os.path.exists(f): continue
        rows = list(csv.DictReader(open(f)))
        per = {}
        for r in rows: per.setdefault(int(r["tick"]), r)
        ticks = sorted(per); g0 = None; launch = None
        for t in ticks:
            r = per[t]
            try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
            except: continue
            if g0 is None: g0 = (gx, gy)
            if math.hypot(gx-g0[0], gy-g0[1]) > 0.3: launch = t; break
        if launch is None: continue
        seq = []  # (tick, track_id, gen_tick, obs_speed)
        brake_t = None
        for r in rows:
            t = int(r["tick"])
            if t < launch or r.get("is_conflict_actor") != "1": continue
            if float(r["ego_y"] or 0) > 126: break
            try: seq.append((t, r["track_id"], int(float(r["gen_tick"])), float(r["obs_speed_mps"])))
            except: continue
            if r.get("brake_event") == "1" and brake_t is None: brake_t = (t-launch)*0.05
        collapse_t = None; collapse_n = 0; moving = False; moving_id = None; rebirth_t = None; rebirth_gap = None
        last_gen = None; gen_gap = 0; gen_gap_t = None; prev_moving_tick = None
        for (t, tid, gen, ov) in seq:
            if last_gen is not None and gen != last_gen and gen - last_gen > gen_gap: gen_gap, gen_gap_t = gen - last_gen, (t-launch)*0.05
            if gen != last_gen: last_gen = gen
            if ov > 3.0:
                if not moving: moving, moving_id = True, tid
                elif tid != moving_id and rebirth_t is None:
                    rebirth_t = (t-launch)*0.05; rebirth_gap = t - prev_moving_tick if prev_moving_tick else None
                moving_id = tid; prev_moving_tick = t
            elif moving and ov == 0.0 and tid == moving_id:
                collapse_n += 1
                if collapse_t is None: collapse_t = (t-launch)*0.05
        W.writerow([host, geom, mgr, anch, speed, lat, rep, launch, "" if collapse_t is None else f"{collapse_t:.2f}", collapse_n, "" if rebirth_t is None else f"{rebirth_t:.2f}", "" if rebirth_gap is None else rebirth_gap, gen_gap, "" if gen_gap_t is None else f"{gen_gap_t:.2f}", "" if brake_t is None else f"{brake_t:.2f}"])
print("done", out)
