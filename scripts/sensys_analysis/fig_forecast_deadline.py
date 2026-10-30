"""Forecast-error deadline figures (Conductor method on the closed-loop logs, Oct 30 2026).
For every run of a stamp: at every tick between the conflict actor's launch and the ego reaching the box, the
planner's held observation of the actor is advanced at its observed speed for the age it carries (the planner's
re-indexing, actor heading -x) and compared with the ground truth at that tick. Per cell (geometry, speed, arm,
configured age) the p50 and p99 of that error over all ticks and runs are written to a CSV; the figures show p99
against the configured age at one speed with the tolerance lines, and the deadline (largest configured age with
p99 within the tolerance) against speed.
    python scripts/sensys_analysis/fig_forecast_deadline.py --roots <experiment_results dirs> --stamp sensys_nov27_j6s
        --speed 70 --tol 2.5 1.0 --out safety_envelope_paper/analysis/figs_j6/forecast_deadline
"""
import argparse, collections, csv, glob, math, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import figstyle
GEOMS = [("openscenario_3_edge_late_fusion", "Occluded left turn"), ("scp_edge_late_fusion", "Straight crossing")]
ARMS = [(("oracle", "on"), "reference"), (("late_fusion", "on"), "contract"), (("late_fusion", "off"), "naive")]
CONFLICT_X = -84.7   # the ego's lane centre; the crossing car heads -x along y 127.7 and crosses it there
HORIZON_S = 3.0      # the planner's forecast horizon (60 steps of 50 ms)
def run_errors(d):
    """Per tick of the decision window, the forecast's error at the conflict (metres along the car's path): the
    re-indexed observation advanced at its speed to the true arrival time, against the conflict point. The decision
    window starts when the forecast first reaches the conflict inside the planner's horizon and ends when the ego
    reaches the box, which is the span over which the brake decision is taken."""
    f = d + "/per_tick_age.csv"
    if not os.path.exists(f): f = d + "/evaluation_output/per_tick_age.csv"
    if not os.path.exists(f): return []
    rows = list(csv.DictReader(open(f)))
    per = {}
    for r in rows: per.setdefault(int(r["tick"]), r)
    ticks = sorted(per); g0 = None; launch = None; gt = {}
    for t in ticks:
        r = per[t]
        try: gx, gy = float(r["conflict_gt_x"]), float(r["conflict_gt_y"])
        except Exception: continue
        gt[t] = (gx, gy)
        if g0 is None: g0 = (gx, gy)
        if launch is None and math.hypot(gx - g0[0], gy - g0[1]) > 0.3: launch = t
    if launch is None: return []
    arr_true = next((t for t in ticks if t in gt and gt[t][0] <= CONFLICT_X), None)
    if arr_true is None: return []
    out = []; opened = False
    for r in rows:
        t = int(r["tick"])
        if t < launch or r.get("is_conflict_actor") != "1" or t not in gt: continue
        if float(r["ego_y"] or 0) > 126: break
        try: ox, ov, ot = float(r["obs_x"]), float(r["obs_speed_mps"]), int(float(r["obs_tick"]))
        except Exception: continue
        age_s = (t - ot) * 0.05
        fx = ox - ov * age_s                      # re-indexed estimate now
        if not opened:
            if ov > 0.5 and (fx - CONFLICT_X) / ov <= HORIZON_S: opened = True
            else: continue
        h = (arr_true - t) * 0.05                 # time until the car truly reaches the conflict
        if h <= 0: break
        out.append(abs((fx - ov * h) - CONFLICT_X))   # forecast position at the true arrival time vs the conflict point
    return out
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roots", nargs="+", required=True); ap.add_argument("--stamp", default="sensys_nov27_j6s")
    ap.add_argument("--speed", type=int, default=70); ap.add_argument("--tol", nargs="+", type=float, default=[2.5, 1.0])
    ap.add_argument("--speeds", nargs="*", type=int, default=[38, 50, 53, 63, 70]); ap.add_argument("--out", required=True)
    a = ap.parse_args(); os.makedirs(a.out, exist_ok=True)
    cells = collections.defaultdict(list); nruns = collections.Counter()
    for root in a.roots:
        for mf in glob.glob(f"{root}/*/{a.stamp}/speed_spd_*/baseline_default/mgr_*/anchoring_*/lat_*/run_*/simulation_metrics.json"):
            if "/evaluation_output/" in mf: continue
            d = os.path.dirname(mf); p = d.split("/")
            geom = p[-8]; speed = int(p[-6].split("_")[-1]); mgr = p[-4][4:]; anch = p[-3].split("_")[-1]; lat = int(p[-2].split("_")[1])
            key = next((k for (m, an), k in ARMS if m == mgr and an == anch), None)
            if key is None: continue
            e = run_errors(d)
            if e: cells[(geom, speed, key, lat)].extend(e); nruns[(geom, speed, key, lat)] += 1
    with open(os.path.join(a.out, "forecast_error_cells.csv"), "w") as fh:
        w = csv.writer(fh); w.writerow(["geom", "speed", "arm", "lat", "runs", "ticks", "p50_m", "p90_m", "p99_m"])
        for k in sorted(cells):
            v = np.array(cells[k]); w.writerow([k[0], k[1], k[2], k[3], nruns[k], len(v), f"{np.percentile(v,50):.3f}", f"{np.percentile(v,90):.3f}", f"{np.percentile(v,99):.3f}"])
    figstyle.rc()
    # (a) p99 error against configured age at one speed
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.TXT, 1.9), sharey=True)
    for ax, (geom, title) in zip(axes, GEOMS):
        for _, key in ARMS:
            label, color, marker = figstyle.ARMS[key]
            pts = sorted((lat, np.percentile(cells[(geom, a.speed, key, lat)], 99)) for (g, s, k, lat) in cells if g == geom and s == a.speed and k == key)
            if not pts: continue
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, marker=marker, ms=3, lw=1.2, label=label)
        for tol in a.tol: ax.axhline(tol, color="0.5", lw=0.8, ls="--")
        ax.set_title(f"({'ab'[GEOMS.index((geom, title))]}) {title}, {a.speed} km/h"); ax.set_xlabel("configured delay (ms)"); ax.set_xlim(-50, 2100); ax.set_ylim(0, 22); ax.grid(alpha=0.3)
    axes[0].set_ylabel("p99 error at the conflict (m)")
    axes[1].legend(loc="lower right", fontsize=6, frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "forecast_error_vs_age.pdf")); fig.savefig(os.path.join(a.out, "forecast_error_vs_age.png"), dpi=200); plt.close(fig)
    # (b) deadline against speed per arm and geometry, first tolerance
    tol = a.tol[0]; rows = []
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.TXT, 1.9), sharey=True)
    for ax, (geom, title) in zip(axes, GEOMS):
        for _, key in ARMS:
            label, color, marker = figstyle.ARMS[key]
            xs, ys, cens = [], [], []
            for s in a.speeds:
                lats = sorted(lat for (g, sp, k, lat) in cells if g == geom and sp == s and k == key)
                if not lats: continue
                ok = [lat for lat in lats if np.percentile(cells[(geom, s, key, lat)], 99) <= tol]
                dl = max(ok) if ok else 0; cen = bool(ok) and dl == lats[-1]
                xs.append(s); ys.append(dl); cens.append(cen); rows.append((geom, s, key, tol, dl, int(cen), ",".join(map(str, lats))))
            if xs:
                ax.plot(xs, ys, color=color, marker=marker, ms=3, lw=1.2, label=label)
                for x, y, c in zip(xs, ys, cens):
                    if c: ax.annotate("^", (x, y), textcoords="offset points", xytext=(0, 3), ha="center", fontsize=7, color=color)
        ax.set_title(f"({'ab'[GEOMS.index((geom, title))]}) {title}"); ax.set_xlabel("approach speed (km/h)"); ax.grid(alpha=0.3)
    axes[0].set_ylabel(f"deadline, p99 error within {tol:g} m (ms)"); axes[0].legend(loc="upper right", fontsize=6, frameon=False)
    fig.tight_layout(); fig.savefig(os.path.join(a.out, "deadline_vs_speed.pdf")); fig.savefig(os.path.join(a.out, "deadline_vs_speed.png"), dpi=200); plt.close(fig)
    with open(os.path.join(a.out, "deadline_vs_speed.csv"), "w") as fh:
        w = csv.writer(fh); w.writerow(["geom", "speed", "arm", "tol_m", "deadline_ms", "censored", "ages_landed"]); w.writerows(rows)
    for r in rows: print(r)
    print("wrote", a.out)
if __name__ == "__main__": main()
