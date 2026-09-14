#!/usr/bin/env python3
"""Lander for the closed-loop state-depth block (frozen1r_state_depth_rows.csv).

One row per run log: maneuver, representation, rep, collided, completed, fde5_m,
route_time_s, hard_brakes, min_ttc_s, kb (+ provenance columns). Definitions:
  representation: from the [LAUNCHENV] line (MIGRATION_MODE=cold -> none; kf ->
    pos_vel; warm + MIGRATION_HIST=2|5 -> hist2|hist5; warm -> hist10; a stem
    containing 'nohandoff' -> no_handoff).
  maneuver: MANEUVER=<name> in [LAUNCHENV] (constant | ltap_od | scp | lane_change).
  collided / completed: khonsu_design_extract.parse_log (binary collided from the
    RUNROW episodes; completed = distance >= 90 m); INVALID logs are skipped.
  fde5_m: the FDE block's rule (khonsu_1l_land.run_fde): min of the two
    highest-scoring modes' 5 s endpoint error, mean over the first five [MODESROW]
    refreshes at or after the window anchor (destination commit; first local
    forecast in the destination for arms without a commit).
  route_time_s: time_alive_s from the planner metrics dict (after the 100-tick
    spawn buffer, as every landed block reports it).
  hard_brakes: count of separate deceleration episodes of the focal CAV at or
    below -4.0 m/s^2 (0.41 g), from the [EGO-DBG] speed samples (every 5 sim
    ticks = 0.25 s); an episode ends when the deceleration rises above the
    threshold. No runtime change: derived from markers every landed block has.
  min_ttc_s: khonsu_ablation_diag.min_ttc (the [EGO-DBG] ttc samples, 1000 = none).
  kb: bytes of the first [OBSTACLE_HANDOFF] for the gating oncoming / 1000 (0 for
    none and no_handoff).
Usage: khonsu_state_depth_land.py <logdir> <out.csv> [--machine atlas] [--tag khonsu-eval-freeze-1r]
"""
import argparse, csv, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import khonsu_1l_land as k1
import khonsu_ablation_diag as kad
import khonsu_design_extract as kde

COLS = ["maneuver", "representation", "rep", "collided", "completed", "fde5_m", "route_time_s", "hard_brakes", "min_ttc_s", "kb",
        "tag", "fde3_m", "window_anchor", "history_depth_env", "migration_mode", "dist_m", "eval_tag", "machine"]
LAUNCHENV_RE = re.compile(r"\[LAUNCHENV\](.*)")
HANDOFF_BYTES_RE = re.compile(r"\[OBSTACLE_HANDOFF\] carla_id=(\d+) \S+ tick=\d+ bytes=(\d+)")
HARD_BRAKE_MPS2 = -4.0

def launch_env(text):
    m = LAUNCHENV_RE.search(text); env = {}
    if m:
        for tok in m.group(1).split():
            if "=" in tok: k, v = tok.split("=", 1); env[k] = v
    return env

def representation(stem, env):
    if "nohandoff" in stem or env.get("MANEUVER_SCENARIO", "").endswith("single"): return "no_handoff"
    mode = env.get("MIGRATION_MODE", "warm").lower()
    if mode == "cold": return "none"
    if mode == "kf": return "pos_vel"
    if mode == "warm":
        h = env.get("MIGRATION_HIST", "")
        return f"hist{int(h)}" if h else "hist10"
    return mode

def hard_brakes(d):
    ego = d.get("ego", {}); ticks = sorted(ego); n = 0; inside = False; prev_t = None; prev_v = None
    for t in ticks:
        v = ego[t][2]
        if prev_t is not None and t > prev_t:
            dec = (v - prev_v) / ((t - prev_t) * 0.05)
            if dec <= HARD_BRAKE_MPS2:
                if not inside: n += 1; inside = True
            else: inside = False
        prev_t, prev_v = t, v
    return n

def fde5(text, d, gid):
    modes = k1._parse_modesrows(text).get(int(gid), {})
    if not modes: return None, None, "no_modesrow"
    hf = d["handoff"].get(int(gid), {})
    try: commit = int(hf.get("first_dst_track_tick"))
    except (TypeError, ValueError): commit = None
    ev, gt = kad.actual_timeline(d, gid)
    if commit is not None: anchor, kind = commit, "commit"
    else:
        anchor, kind = None, "first_local"
        for T in sorted(modes):
            ap = kad.lookup_actual(ev, gt, T)
            if ap is not None and ap[0] >= 240.0: anchor = T; break
    ticks = sorted(t for t, hd in modes.items() if 3.0 in hd and 5.0 in hd and (anchor is None or t >= anchor))[:5]
    f5, f3 = [], []
    for T in ticks:
        m5, _ = k1._min_top2_fde(modes[T][5.0], kad.lookup_actual(ev, gt, T + kad.horizon_ticks(5.0)))
        if m5 is not None: f5.append(m5)
        m3, _ = k1._min_top2_fde(modes[T][3.0], kad.lookup_actual(ev, gt, T + kad.horizon_ticks(3.0)))
        if m3 is not None: f3.append(m3)
    return (round(sum(f5) / len(f5), 3) if f5 else None), (round(sum(f3) / len(f3), 3) if f3 else None), kind

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("logdir"); ap.add_argument("out"); ap.add_argument("--machine", default="atlas"); ap.add_argument("--tag", default="khonsu-eval-freeze-1r")
    a = ap.parse_args(); rows = []
    for name in sorted(os.listdir(a.logdir)):
        if not name.endswith(".log"): continue
        stem = name[:-4]; path = os.path.join(a.logdir, name); text = k1._read(path)
        core = kde.parse_log(path)
        if core is None or core == "INVALID":
            print(f"skip ({'no RUNROW' if core is None else 'INVALID'}): {name}", file=sys.stderr); continue
        env = launch_env(text); rep = representation(stem, env); man = env.get("MANEUVER", "")
        d = kad.parse_log(path)
        gid = k1._gating_oncoming(d)   # migrated oncoming; for none/no_handoff the single identified oncoming (anchor = first local forecast in the destination)
        f5 = f3 = None; kind = ""
        if gid is not None: f5, f3, kind = fde5(text, d, gid)
        kb = 0.0
        if rep not in ("none", "no_handoff") and gid is not None:
            for m in HANDOFF_BYTES_RE.finditer(text):
                if int(m.group(1)) == int(gid): kb = int(m.group(2)) / 1000.0; break
        rows.append({"maneuver": man, "representation": rep, "rep": k1._seed_from(stem), "collided": core.get("collided", ""), "completed": core.get("completed", ""),
                     "fde5_m": f5 if f5 is not None else "", "route_time_s": core.get("time_s", ""), "hard_brakes": hard_brakes(d), "min_ttc_s": kad.min_ttc(d), "kb": round(kb, 3),
                     "tag": stem, "fde3_m": f3 if f3 is not None else "", "window_anchor": kind, "history_depth_env": env.get("MIGRATION_HIST", ""), "migration_mode": env.get("MIGRATION_MODE", ""),
                     "dist_m": core.get("dist_m", ""), "eval_tag": k1._eval_tag(text, a.tag), "machine": a.machine})
    k1._check_provenance(rows, a.logdir)
    with open(a.out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS); w.writeheader(); [w.writerow(r) for r in rows]
    print(f"{len(rows)} rows -> {a.out}")
    for r in rows: print(f"  {r['tag']:34s} {r['maneuver']:11s} {r['representation']:10s} rep {r['rep']:>2s} collided {r['collided']:3s} completed {r['completed']:3s} fde5 {r['fde5_m']} route {r['route_time_s']} hb {r['hard_brakes']} ttc {r['min_ttc_s']} kb {r['kb']}")

if __name__ == "__main__":
    main()
