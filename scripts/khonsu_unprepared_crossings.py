#!/usr/bin/env python3
"""Count Khonsu-arm crossings that had NO prepare for the entered locale (the
never-confident / never-fired case), from landed flow-family logs.

Per warm log: the identified oncoming actors (khonsu_ablation_diag.identify_oncoming);
a crossing = the first tick the actor's actual position enters the destination
locale (x >= boundary - 10 in the flow geometry, boundary 250 unless the launch
line carries boundary=); prepared = a handoff record for that actor into the
destination with prepare_tick <= entry tick ([HANDOFFROW]) or an
[OBSTACLE_HANDOFF] for it at or before entry. Reason for an unprepared crossing:
'trigger_never_fired' (predictive/computed/band triggers: the projected exit
never fell within the lookahead before entry), 'no_candidate_at_theta' when the
log carries the mtr trigger's [CONFROW]/[MTRTRIG] lines with every sample below
theta, else 'unknown'. Usage: khonsu_unprepared_crossings.py <logdir> [<logdir>...]
"""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import khonsu_ablation_diag as kad
import khonsu_t25_land as kt
RES_RE = re.compile(r"\[SCENB\] NPC carla_id=(\d+) resolved at tick=(\d+)")
OH_RE = re.compile(r"\[OBSTACLE_HANDOFF\] carla_id=(\d+) \S+ tick=(\d+)")
TRIG_RE = re.compile(r"TRIGGER_MODE=(\w+)")
CONF_RE = re.compile(r"\[(?:CONFROW|MTRTRIG)\][^\n]*?(?:p_max|pmax|prob|p)=([0-9.]+)")

def analyse(path):
    text = open(path, errors="ignore").read()
    env = text.split("\n", 1)[0] if text.startswith("[LAUNCHENV]") else ""
    if "MIGRATION_MODE=warm" not in env or "warm_nofinal" in env or "MIGRATION_ACCEL=0" in env: return None
    m = re.search(r"boundary=(\d+)", env); dest_x = (float(m.group(1)) - 10.0) if m else 240.0
    trig = (TRIG_RE.search(env).group(1) if TRIG_RE.search(env) else "predictive")
    d = kad.parse_log(path); onc, _ = kad.identify_oncoming(d)
    try: gid = kt.resolve_gating(d, text, onc, kad.launch_tick(d))
    except Exception: gid = None
    resolved = {int(a): int(t) for a, t in RES_RE.findall(text)}
    oh = {}
    for mm in OH_RE.finditer(text): oh.setdefault(int(mm.group(1)), []).append(int(mm.group(2)))
    rows = []
    for cid in onc:
        ev, gt = kad.actual_timeline(d, cid)
        ticks = sorted(ev) if ev else sorted(gt)
        entry = None; seen_west = False
        for t in ticks:
            p = ev.get(t) if ev else gt.get(t)
            if p is None: continue
            if p[0] < dest_x: seen_west = True
            elif seen_west: entry = t; break
        if entry is None: continue           # never crossed in this run
        hf = d["handoff"].get(cid, {})
        try: ptk = int(hf.get("prepare_tick"))
        except (TypeError, ValueError): ptk = None
        prepared = (ptk is not None and ptk <= entry) or any(t <= entry for t in oh.get(cid, []))
        reason = ""
        if not prepared:
            if trig == "mtr":
                ps = [float(x) for x in CONF_RE.findall(text)]
                reason = "no_candidate_at_theta" if ps else "unknown_mtr"
            else: reason = "trigger_never_fired"
        rows.append((cid, entry, prepared, reason, ptk, (str(cid) == str(gid)), entry - resolved.get(cid, entry)))
    return trig, rows

def main():
    tot = {"logs": 0, "crossings": 0, "prepared": 0, "unprepared": 0, "unprepared_gating": 0, "gating_crossings": 0}; by_reason = {}; per_block = {}; since_res = []
    for ld in sys.argv[1:]:
        b = os.path.basename(ld.rstrip("/")); pb = per_block.setdefault(b, {"logs": 0, "crossings": 0, "unprepared": 0, "runs_with_unprepared": []})
        for name in sorted(os.listdir(ld)):
            if not name.endswith(".log"): continue
            try: r = analyse(os.path.join(ld, name))
            except Exception as e: print(f"  error {name}: {e!r}", file=sys.stderr); continue
            if r is None: continue
            trig, rows = r; pb["logs"] += 1; tot["logs"] += 1
            for cid, entry, prepared, reason, ptk, is_gating, dt_res in rows:
                pb["crossings"] += 1; tot["crossings"] += 1
                if is_gating: tot["gating_crossings"] += 1; pb["gating_crossings"] = pb.get("gating_crossings", 0) + 1
                if prepared: tot["prepared"] += 1
                else:
                    tot["unprepared"] += 1; pb["unprepared"] += 1; by_reason[reason] = by_reason.get(reason, 0) + 1; since_res.append(dt_res)
                    if is_gating: tot["unprepared_gating"] += 1; pb["unprepared_gating"] = pb.get("unprepared_gating", 0) + 1
                    pb["runs_with_unprepared"].append(f"{name[:-4]}:cid{cid}@t{entry}{'G' if is_gating else ''}(+{dt_res}t)")
    for b, pb in per_block.items():
        print(f"{b:22s} warm logs {pb['logs']:4d} crossings {pb['crossings']:4d} (gating {pb.get('gating_crossings',0):3d}) unprepared {pb['unprepared']:4d} (gating {pb.get('unprepared_gating',0):3d})" + (f"  e.g. {', '.join(pb['runs_with_unprepared'][:3])}" if pb['runs_with_unprepared'] else ""))
    import numpy as np
    sr = np.array(since_res) if since_res else np.zeros(1)
    print(f"TOTAL warm logs {tot['logs']} crossings {tot['crossings']} (gating {tot['gating_crossings']}) prepared {tot['prepared']} unprepared {tot['unprepared']} (gating {tot['unprepared_gating']}) by reason {by_reason}; unprepared crossings happen {np.median(sr):.0f} ticks (median) after the actor was resolved, p90 {np.percentile(sr, 90):.0f}")

if __name__ == "__main__":
    main()
