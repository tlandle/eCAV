"""Traffic column set: per-vehicle collision table and per-cell summary.
  python scripts/ispass/column_traffic_summary.py experiment_results/scp_colu60 ispass_fct1660 collisions.csv cells.csv --exclude 21
One row per collided column vehicle: first contact partner (crossing car, column vehicle, background vehicle), with the
step and place of the vehicle's largest one-step speed change. Cells count runs over the scenes that are not excluded.
column_trace_reduce.py labels every partner outside the column as the crossing car, so its with_crossing column is not
usable with background traffic."""
import json, glob, os, sys, re, csv, collections
def as_obj(x):
    if isinstance(x, str):
        try: return json.loads(x)
        except ValueError:
            import ast
            try: return ast.literal_eval(x)
            except Exception: return {}
    return x or {}
root, stem, out_c, out_cells = sys.argv[1:5]
excl = {int(x) for x in sys.argv[sys.argv.index("--exclude") + 1:]} if "--exclude" in sys.argv else set()
rows = []; scenes = collections.defaultdict(set)
for cell in sorted(glob.glob(os.path.join(root, stem + "_*"))):
    m = re.search(r"_(gt_all|gt_vis|onboard)_n(\d+)$", cell); arm, n = m.group(1), int(m.group(2))
    for mf in sorted(glob.glob(os.path.join(cell, "**", "run_*", "simulation_metrics.json"), recursive=True)):
        rd = os.path.dirname(mf)
        if os.path.basename(rd) == "evaluation_output": continue
        rep = int(os.path.basename(rd).split("_")[-1]); scenes[(arm, n)].add(rep)
        d = json.load(open(mf)); veh = d.get("vehicles") or {}
        log = open(os.path.join(rd, "runner_stdout.txt"), errors="replace").read()
        lm = re.search(r"\[SYNC LAUNCH\] step=(\d+)", log); launch = int(lm.group(1)) if lm else ""
        T = {}
        for r in csv.DictReader(open(os.path.join(rd, "evaluation_output", "column_trace.csv"))):
            T.setdefault(r["actor_id"], []).append((int(r["step"]), int(r["index"]), float(r["x"]), float(r["y"]), float(r["speed_mps"])))
        if len(veh) != n or len(T) != n or "error" in d: print("INCOMPLETE", arm, n, rep, len(veh), len(T))
        for vid, v in veh.items():
            s = as_obj(v.get("safety"))
            if not (isinstance(s, dict) and int(s.get("collision_count", 0) or 0) > 0): continue
            h = as_obj(s.get("collision_history")); f = h[0] if h else {}
            t = T.get(str(v.get("actor_id", vid)))
            k = max(range(1, len(t)), key=lambda i: abs(t[i][4] - t[i-1][4]) if t[i][0] > 3 else 0)
            typ = f.get("other_type_id", "?")
            kind = "crossing_car" if "tesla" in typ else ("column_vehicle" if str(f.get("other_actor_id")) in T else "background")
            rows.append(dict(arm=arm, n=n, scene=rep, position=t[k][1] + 1, partner=kind, partner_type=typ, step=t[k][0], x=t[k][2], y=t[k][3], launch_step=launch, scene_excluded=int(rep in excl)))
ARM = {"gt_all": "gt_all", "gt_vis": "gt_vis", "onboard": "onboard_640"}
for r in rows: r["arm"] = ARM[r["arm"]]
rows.sort(key=lambda r: (r["arm"], r["n"], r["scene"], r["position"]))
with open(out_c, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
cells = []
for (arm, n), sc in sorted(scenes.items()):
    a = ARM[arm]; ok = sorted(sc - excl); R = [r for r in rows if r["arm"] == a and r["n"] == n and r["scene"] in ok]
    by = lambda pred: len({r["scene"] for r in R if pred(r)})
    cells.append(dict(arm=a, n=n, scenes=len(ok), runs_with_collision=by(lambda r: True), collided_vehicles=len(R),
                      first_crossing_car=by(lambda r: r["position"] == 1 and r["partner"] == "crossing_car"),
                      first_background=by(lambda r: r["position"] == 1 and r["partner"] == "background"),
                      first_column=by(lambda r: r["position"] == 1 and r["partner"] == "column_vehicle"),
                      runs_follower_collides=by(lambda r: r["position"] > 1),
                      follower_crossing_car=by(lambda r: r["position"] > 1 and r["partner"] == "crossing_car"),
                      follower_background=by(lambda r: r["position"] > 1 and r["partner"] == "background"),
                      follower_column=by(lambda r: r["position"] > 1 and r["partner"] == "column_vehicle"),
                      follower_positions=" ".join(str(r["position"]) for r in R if r["position"] > 1)))
with open(out_cells, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(cells[0].keys())); w.writeheader(); w.writerows(cells)
print(f"{'arm':12s} n scenes any  veh | first: cross bg col | follower runs: any cross bg col  positions")
for c in cells:
    print(f"{c['arm']:12s} {c['n']} {c['scenes']:5d} {c['runs_with_collision']:4d} {c['collided_vehicles']:4d} | {c['first_crossing_car']:11d} {c['first_background']:3d} {c['first_column']:3d} | {c['runs_follower_collides']:17d} {c['follower_crossing_car']:5d} {c['follower_background']:3d} {c['follower_column']:3d}  {c['follower_positions']}")
