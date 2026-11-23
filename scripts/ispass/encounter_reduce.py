"""Per-run table of a single-vehicle encounter set (Section V): outcome, brake start, closest distance, launch,
first detection, first track.

    encounter_reduce.py <stem> <out.csv> [--cells 40 50 60 70 80] [--cell-dir "{stem}{cell}_{arm}_n1"]

The default cell directory is the speed set of column_sweep.sh. The cut-in set of cutin_sweep.sh uses
--cell-dir "{stem}_{arm}_cs{cell}" with the closing speeds as cells. From tmp/spd_reduce.py of the speed set."""
import sys, glob, os, csv, math, re, json
import numpy as np
sys.path.insert(0, "scripts/ispass"); import fidelity_reduce as fr
stem, out = sys.argv[1], sys.argv[2]
_a = sys.argv[3:]
CELLS = [int(x) for x in _a[_a.index("--cells") + 1:(_a.index("--cell-dir") if "--cell-dir" in _a and _a.index("--cell-dir") > _a.index("--cells") else len(_a))]] if "--cells" in _a else [40, 50, 60, 70, 80]
CELL_DIR = _a[_a.index("--cell-dir") + 1] if "--cell-dir" in _a else "{stem}{cell}_{arm}_n1"
rows_out = []
for spd in CELLS:
  for arm in ("gt_vis", "onboard"):
    for f in sorted(glob.glob("experiment_results/*/" + CELL_DIR.format(stem=stem, cell=spd, arm=arm) + "/**/evaluation_output/per_tick_age.csv", recursive=True)):
        rd = os.path.dirname(os.path.dirname(f)); rep = int(os.path.basename(rd).split("_")[-1])
        a = fr.attribution(f); d = json.load(open(os.path.join(rd, "simulation_metrics.json")))
        log = open(os.path.join(rd, "runner_stdout.txt"), errors="replace").read()
        m = re.search(r"\[SYNC LAUNCH\] lead ([\d.]+)", log); adv = re.search(r"advanced ([\d.]+) m", log)
        rows = {}
        for r in csv.DictReader(open(f)): rows.setdefault(int(r["tick"]), []).append(r)
        gt = {t: (float(v[0]["conflict_gt_x"]), float(v[0]["conflict_gt_y"])) for t, v in rows.items() if v[0].get("conflict_gt_x")}
        launch = next((t for t in sorted(gt) if t-1 in gt and math.hypot(gt[t][0]-gt[t-1][0], gt[t][1]-gt[t-1][1]) > 0.02), None)
        on = a["tp_onset"]; nan = float("nan")
        row = dict(speed=spd, arm=arm, rep=rep, collided=int(d["focal_collisions"] > 0), onset_s=on/1000, min_sep=a["min_sep"], lead=float(m.group(1)) if m else nan,
                   advanced_m=float(adv.group(1)) if adv else nan, launch_tick=launch, v_approach=float(rows[launch][0]["ego_speed_mps"]) if launch else nan,
                   track_s=nan, track_gap_ticks=nan, in_range_s=nan, det_s=nan, det_d=nan, bearing=nan, missed=nan, bad=int("Traceback" in log or launch is None))
        if launch is not None:
            t_on = launch + int(round(on/50)) if on == on else launch + 200
            trk = sorted(t for t, v in rows.items() if launch <= t <= t_on and any(r.get("is_conflict_actor") == "1" and r.get("obs_x") for r in v))
            if trk: row["track_s"] = (trk[0]-launch)*0.05; row["track_gap_ticks"] = t_on - trk[0] + 1 - len(trk)
            if arm == "onboard":
                calls = {}
                for mm in re.finditer(r"\[PERC-TRACE\] call=(\d+) ego=\d+ actor=(\d+) d=([\d.]+) vis=(\d+) det=(\d) .*? b=(-?\d+)", log):
                    if int(mm.group(2)) > 0: calls.setdefault(int(mm.group(1)), (float(mm.group(3)), int(mm.group(5)), int(mm.group(6))))
                cs = sorted(c for c in calls if launch <= c <= t_on); det = next((c for c in cs if calls[c][1]), None)
                if cs: row["in_range_s"] = (cs[0]-launch)*0.05
                if det is not None:
                    row.update(det_s=(det-launch)*0.05, det_d=calls[det][0], bearing=calls[det][2], missed=sum(1 for c in cs if c > det and not calls[c][1]))
        rows_out.append(row)
with open(out, "w", newline="") as fh:
    w = csv.DictWriter(fh, fieldnames=list(rows_out[0].keys())); w.writeheader(); w.writerows(rows_out)
for spd in CELLS:
    for arm in ("gt_vis", "onboard"):
        r = [x for x in rows_out if x["speed"] == spd and x["arm"] == arm and not x["bad"]]
        if not r: continue
        on = [x["onset_s"] for x in r if x["onset_s"] == x["onset_s"]]
        print("cell %d %-8s runs %2d distinct %2d collided %2d brake start median %.2f s (min %.2f, max %.2f) closest median %.1f m (min %.1f)" % (
            spd, arm, len(r), len({(x["launch_tick"], x["advanced_m"]) for x in r}), sum(x["collided"] for x in r), np.median(on) if on else float("nan"), min(on) if on else float("nan"), max(on) if on else float("nan"),
            np.median([x["min_sep"] for x in r]), min(x["min_sep"] for x in r)))
print("invalid runs:", sum(x["bad"] for x in rows_out), "of", len(rows_out))
