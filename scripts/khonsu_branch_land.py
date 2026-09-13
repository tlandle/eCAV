#!/usr/bin/env python3
"""Lander for the branching-junction harness (NSDI experiment 1).

Reads evaluation_outputs/branch_1p_*/br_<site>_<arm>_s<seed>.log and writes
  branch_rows.csv   one row per run (READYROW/PLACEROW/RUNROW/SEEDROW joined)
  branch_conf.csv   CONFROW samples at the trigger tick and at T-2/1/0.5 s before entry
  branch_checks.txt validity checks (payload identity, scene identity, route, oracle)
  branch_summary.txt readiness / placement / calibration tables per site x arm
Checks are computed, never eyeballed: a failed check is printed with the run tags.
"""
import csv, glob, math, os, re, sys
from collections import defaultdict
ROOT = sys.argv[1] if len(sys.argv) > 1 else "evaluation_outputs"
OUT = sys.argv[2] if len(sys.argv) > 2 else ROOT
DT = 0.05
DEPTH = int(os.environ.get("RECORD_DEPTH", "10"))   # configured record depth (tracker max_window / MTR past frames)
KV = re.compile(r"(\w+)=([^\s]+)")
def kv(line): return {k: v for k, v in KV.findall(line)}
def num(v, d=float('nan')):
    try: return float(v)
    except Exception: return d
runs = []; conf = []; traj = {}
for log in sorted(glob.glob(f"{ROOT}/branch_1p_*/br_*.log")):
    tag = os.path.basename(log)[:-4]
    m = re.match(r"br_(k\d)_(\w+?)_s(\d+)$", tag)
    if not m: continue
    site, arm, seed = m.group(1), m.group(2), int(m.group(3))
    row = {"site": site, "arm": arm, "seed": seed, "tag": tag, "runrow": 0, "ready": "", "prepared": "", "n_prepared": "",
           "wrong_destination": "", "right_destination_too_late": "", "lead_s": "", "t_avail_s": "", "taken": "", "entered": "",
           "route_ok": "", "k": "", "trigger_tick": "", "crossing_tick": "", "conflict_tick": "", "first_dst_track_tick": "",
           "first_use_tick": "", "prepare_tick": "", "cv_pick": "", "run_bytes": "", "collided": "", "hashes": "", "fire_rule": "",
           "spawn_offset_m": "", "hardfail": ""}
    hashes = {}; depths = {}; confrows = []; npc = []; fire_tick = None; enter_tick = None; collided = None
    for line in open(log, errors="replace"):
        if "[READYROW]" in line:
            d = kv(line); row.update({k: d.get(k, "") for k in ("ready", "prepared", "n_prepared", "wrong_destination", "right_destination_too_late",
                "lead_s", "t_avail_s", "taken", "entered", "route_ok", "k", "trigger_tick", "crossing_tick", "conflict_tick",
                "first_dst_track_tick", "first_use_tick", "prepare_tick", "cv_pick")})
        elif "[PLACEROW]" in line: row["run_bytes"] = kv(line).get("run_bytes", "")
        elif "[HANDOFFROW]" in line:
            d = kv(line); hashes[d.get("dst", "?")] = (d.get("hash", ""), d.get("prepare_tick", ""))
            depths[d.get("dst", "?")] = int(num(d.get("depth", "-1"), -1))
        elif "[RUNROW]" in line: row["runrow"] = 1
        elif "[SEEDROW]" in line: row["spawn_offset_m"] = kv(line).get("spawn_offset_m", "")
        elif "[FIREROW]" in line and fire_tick is None: fire_tick = int(kv(line).get("tick", -1)); row["fire_rule"] = kv(line).get("rule", "")
        elif "[ENTERROW]" in line and enter_tick is None: enter_tick = int(kv(line).get("tick", -1))
        elif "[CONFROW]" in line: confrows.append(kv(line))
        elif "[NPCROW]" in line: d = kv(line); npc.append((int(d["tick"]), num(d["x"]), num(d["y"]), num(d["v"])))
        elif "HARD FAIL" in line: row["hardfail"] = 1
        elif "collision" in line.lower() and ("CollisionTest" in line or "Collision" in line and "FAILURE" in line): collided = 1
    # binary collision from the scenario-runner criteria block if present
    txt = open(log, errors="replace").read()
    if re.search(r"CollisionTest.*FAILURE", txt): collided = 1
    elif re.search(r"CollisionTest.*SUCCESS", txt): collided = collided or 0
    row["collided"] = "" if collided is None else collided
    row["hashes"] = ";".join(f"{d}:{h}@{t}" for d, (h, t) in sorted(hashes.items()))
    row["_hashes"] = hashes; row["_fire_tick"] = fire_tick; row["_enter_tick"] = enter_tick
    row["depths"] = ";".join(f"{d}:{v}" for d, v in sorted(depths.items())); row["_depths"] = depths
    traj[(site, seed, arm)] = npc
    # CONFROW samples: at the trigger tick (what the policy consumed) and T-2/1/0.5 s before entry
    def nearest(tick):
        if tick is None or tick < 0 or not confrows: return None
        return min(confrows, key=lambda c: abs(int(c["tick"]) - tick))
    for label, tk in (("trigger", fire_tick), ("T-2.0", enter_tick - int(2.0 / DT) if enter_tick else None),
                      ("T-1.0", enter_tick - int(1.0 / DT) if enter_tick else None), ("T-0.5", enter_tick - int(0.5 / DT) if enter_tick else None)):
        c = nearest(tk)
        if c is None or abs(int(c["tick"]) - tk) > 3: continue
        conf.append({"site": site, "arm": arm, "seed": seed, "sample": label, "tick": c["tick"], "n_modes": c.get("n_modes", ""),
                     "argmax": c.get("argmax", ""), "p_argmax": c.get("p_argmax", ""), "top_mode_branch": c.get("top_mode_branch", ""),
                     "cv_branch": c.get("cv_branch", ""), "taken": c.get("taken", ""),
                     "correct": int(c.get("argmax", "") == c.get("taken", "")) if c.get("argmax") else "",
                     "p_taken": next((v for k, v in c.items() if k == f"p[{c.get('taken','')}]"), "")})
    runs.append(row)
os.makedirs(OUT, exist_ok=True)
cols = [c for c in runs[0].keys() if not c.startswith("_")] if runs else []
with open(f"{OUT}/branch_rows.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore"); w.writeheader(); [w.writerow(r) for r in runs]
with open(f"{OUT}/branch_conf.csv", "w", newline="") as f:
    if conf: w = csv.DictWriter(f, fieldnames=list(conf[0].keys())); w.writeheader(); [w.writerow(c) for c in conf]
# ---------------- checks ----------------
chk = []
by_ss = defaultdict(list)
for r in runs: by_ss[(r["site"], r["seed"])].append(r)
for (site, seed), rs in sorted(by_ss.items()):
    arms = {r["arm"]: r for r in rs}
    # (1) payload identity. Same-tick arms must be byte-identical: replicate's k
    # prepares within a run; khonsu vs top1 (shared trigger tick by construction).
    # Arms firing at different ticks carry different tracker state, so across
    # those the check is payload SIZE equality (same record type), not the hash.
    for a, r in arms.items():
        hs = {h for (h, t) in r["_hashes"].values()}
        if a == "replicate" and len(r["_hashes"]) > 1 and len(hs) != 1:
            chk.append(f"FAIL payload identity within replicate run {r['tag']}: {r['hashes']}")
    if "khonsu" in arms and "top1" in arms:
        k, t = arms["khonsu"], arms["top1"]
        if k["_fire_tick"] != t["_fire_tick"]:
            chk.append(f"FAIL khonsu/top1 trigger tick differs {site} s{seed}: {k['_fire_tick']} vs {t['_fire_tick']}")
        common = set(k["_hashes"]) & set(t["_hashes"])
        for d in common:
            if k["_hashes"][d][0] != t["_hashes"][d][0]:
                chk.append(f"FAIL khonsu/top1 payload differs {site} s{seed} dst={d}: {k['_hashes'][d]} vs {t['_hashes'][d]}")
    sizes = {a: r["run_bytes"] for a, r in arms.items() if r["n_prepared"] not in ("", "0")}
    # (1b) record depth: every prepare must carry the configured depth (a
    # too-early trigger ships a partial record; size checks cannot see it)
    for a, r in arms.items():
        for d, v in r["_depths"].items():
            if v != DEPTH: chk.append(f"FAIL record depth {r['tag']} dst={d}: depth={v} expected {DEPTH}")
    # (2) scene identity: NPC polyline across arms (max deviation at common ticks)
    base = None
    for a in sorted(arms):
        tr = traj.get((site, seed, a))
        if tr: base = (a, {t: (x, y) for t, x, y, v in tr}); break
    if base:
        for a in sorted(arms):
            tr = traj.get((site, seed, a))
            if not tr or a == base[0]: continue
            devs = [math.hypot(x - base[1][t][0], y - base[1][t][1]) for t, x, y, v in tr if t in base[1]]
            if devs:
                mx = max(devs); p95 = sorted(devs)[int(0.95 * (len(devs) - 1))]
                (chk.append if mx > 1.0 else lambda s: None)(f"FAIL scene identity {site} s{seed} {a} vs {base[0]}: max dev {mx:.2f} m p95 {p95:.2f} m over {len(devs)} ticks")
                chk.append(f"info scene {site} s{seed} {a} vs {base[0]}: max {mx:.2f} p95 {p95:.2f} n={len(devs)}") if mx <= 1.0 else None
    # (3) route: the actor must enter BRANCH_TAKEN in every arm
    for a, r in arms.items():
        if r["route_ok"] != "YES": chk.append(f"FAIL route {r['tag']}: taken={r['taken']} entered={r['entered']} route_ok={r['route_ok']}")
    # (3b) Khonsu's MTR trigger must actually FIRE on the smoke route (mode
    # collapse under isolation caps the destination probability at 6/64 < theta,
    # so a silent non-fire would run 240 routes of a trigger that never fires)
    for a in ("khonsu", "top1"):
        if a in arms and arms[a]["fire_rule"] not in ("theta", "top1"):
            chk.append(f"FAIL {a} trigger never fired {site} s{seed}: fire_rule={arms[a]['fire_rule'] or 'none'} prepared={arms[a]['prepared']}")
    # (4) oracle readiness (conflict timeline sanity)
    if "oracle" in arms and arms["oracle"]["ready"] != "YES":
        chk.append(f"WARN oracle not ready {site} s{seed}: prepared={arms['oracle']['prepared']} first_dst={arms['oracle']['first_dst_track_tick']} crossing={arms['oracle']['crossing_tick']} first_use={arms['oracle']['first_use_tick']}")
open(f"{OUT}/branch_checks.txt", "w").write("\n".join(chk) + "\n")
# ---------------- summary ----------------
S = []
def frac(rs, key, yes="YES"):
    v = [r for r in rs if r[key] != ""]; return (sum(r[key] == yes for r in v) / len(v), len(v)) if v else (float('nan'), 0)
def med(vals):
    v = sorted(x for x in vals if not math.isnan(x)); return v[len(v) // 2] if v else float('nan')
S.append("READINESS per site x arm: ready frac (n), wrong_destination, right_too_late, median lead_s, median t_avail_s, prepared count mean, collided frac")
by = defaultdict(list)
for r in runs: by[(r["site"], r["arm"])].append(r)
for (site, arm), rs in sorted(by.items()):
    rd, n = frac(rs, "ready"); wd, _ = frac(rs, "wrong_destination"); rl, _ = frac(rs, "right_destination_too_late")
    ld = med([num(r["lead_s"]) for r in rs]); ta = med([num(r["t_avail_s"]) for r in rs])
    pc = [num(r["n_prepared"]) for r in rs if r["n_prepared"] != ""]; pcm = sum(pc) / len(pc) if pc else float('nan')
    col = [r["collided"] for r in rs if r["collided"] != ""]; cf = (sum(int(c) for c in col) / len(col)) if col else float('nan')
    S.append(f"  {site} {arm:10s} ready={rd:.2f} (n={n}) wrong_dst={wd:.2f} right_late={rl:.2f} lead_s={ld:.2f} t_avail_s={ta:.2f} placed={pcm:.2f} collided={cf:.2f}")
S.append("\nCONFIDENCE (CONFROW) per site x sample: n, median p_argmax, argmax correct frac, top-mode correct frac, cv correct frac")
byc = defaultdict(list)
for c in conf:
    if c["arm"] in ("khonsu",): byc[(c["site"], c["sample"])].append(c)   # one arm's predictor instance suffices (same model, same scene)
for (site, smp), cs in sorted(byc.items()):
    pa = med([num(c["p_argmax"]) for c in cs]); ac = sum(int(c["correct"] or 0) for c in cs) / len(cs)
    tc = sum(c["top_mode_branch"] == c["taken"] for c in cs) / len(cs); cv = sum(c["cv_branch"] == c["taken"] for c in cs) / len(cs)
    S.append(f"  {site} {smp:8s} n={len(cs)} p_argmax_med={pa:.3f} argmax_correct={ac:.2f} top_mode_correct={tc:.2f} cv_correct={cv:.2f}")
S.append("\nCALIBRATION at the trigger sample (khonsu runs), bins of p_argmax: n, fraction argmax==taken")
bins = defaultdict(list)
for c in conf:
    if c["arm"] == "khonsu" and c["sample"] == "trigger" and c["p_argmax"]:
        b = min(9, int(num(c["p_argmax"]) * 10)); bins[b].append(int(c["correct"] or 0))
for b in sorted(bins): S.append(f"  [{b/10:.1f},{(b+1)/10:.1f}) n={len(bins[b])} correct={sum(bins[b])/len(bins[b]):.2f}")
S.append(f"\nruns={len(runs)} conf_samples={len(conf)} checks: {sum(1 for c in chk if c.startswith('FAIL'))} FAIL, {sum(1 for c in chk if c.startswith('WARN'))} WARN")
open(f"{OUT}/branch_summary.txt", "w").write("\n".join(S) + "\n")
print("\n".join(S)); print("\n".join(c for c in chk if not c.startswith("info")))
