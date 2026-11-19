#!/usr/bin/env python
"""Ground truth in sensor view against camera and LiDAR perception by approach speed, one vehicle, no edge.

    python scripts/ispass/plot_speed.py            # writes paper_ispass/fig_speed.pdf

(a) runs with a collision out of the ten runs of each cell against the measured approach speed. (b) median brake
start after the launch of the crossing car, with a bar from the earliest to the latest brake start of the cell.
Input: paper_ispass/speed_runs3.csv (the per-run table of tmp/spd_reduce.py for the set ispass_spdq: LiDAR at 2.4 m,
rear camera at x=-2.5 m, cameras of 100 degrees). Drawn at column width with 7 pt text, included at scale 1."""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
R = [r for r in csv.DictReader(open(os.path.join(ROOT, "paper_ispass", "speed_runs3.csv"))) if r["bad"] == "0"]
SPEEDS = [40, 50, 60, 70, 80]
ARMS = [("gt_vis", "ground truth in sensor view", "#003057", "o", "-"), ("onboard", "camera and LiDAR", "#eb6834", "s", "--")]
plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
                     "axes.titlesize": 7, "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": 0.5,
                     "grid.color": "#e3e3e3", "grid.linewidth": 0.4, "lines.linewidth": 1.0, "lines.markersize": 3,
                     "xtick.major.width": 0.5, "ytick.major.width": 0.5, "pdf.fonttype": 42})
fig, (a, b) = plt.subplots(1, 2, figsize=(3.45, 1.5))
for arm, lab, colr, mk, ls in ARMS:
    xs, col, md, lo, hi = [], [], [], [], []
    for s in SPEEDS:
        c = [r for r in R if r["arm"] == arm and int(r["speed"]) == s]
        xs.append(3.6 * np.mean([float(r["v_approach"]) for r in c])); col.append(sum(int(r["collided"]) for r in c))
        on = np.array([float(r["onset_s"]) for r in c if r["onset_s"] not in ("", "nan")])
        md.append(np.median(on)); lo.append(np.median(on) - on.min()); hi.append(on.max() - np.median(on))
        print(arm, s, "km/h measured %.0f" % xs[-1], "runs", len(c), "collided", col[-1], "brake start median %.2f min %.2f max %.2f" % (md[-1], on.min(), on.max()))
    a.plot(xs, col, color=colr, marker=mk, ls=ls, label=lab)
    b.errorbar(xs, md, yerr=[lo, hi], color=colr, marker=mk, ls=ls, capsize=1.5, elinewidth=0.6, capthick=0.6)
for ax in (a, b):
    ax.set_xlabel("approach speed (km/h)", labelpad=1); ax.set_xlim(33, 81); ax.set_xticks([40, 50, 60, 70, 80]); ax.grid(True); ax.set_axisbelow(True)
a.set_ylabel("runs with a collision (of 10)", labelpad=1); a.set_ylim(-0.5, 10.8); a.set_yticks([0, 2, 4, 6, 8, 10]); a.set_title("(a) collisions", pad=2)
b.set_ylabel("brake start (s)", labelpad=1); b.set_ylim(1.0, 3.2); b.set_title("(b) brake start", pad=2)
h, l = a.get_legend_handles_labels()
fig.legend(h, l, loc="upper center", ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.02), columnspacing=0.8, handlelength=1.6, handletextpad=0.3)
fig.subplots_adjust(left=0.105, right=0.99, bottom=0.235, top=0.76, wspace=0.36)
out = os.path.join(ROOT, "paper_ispass", "fig_speed.pdf")
fig.savefig(out); print("wrote", out)
