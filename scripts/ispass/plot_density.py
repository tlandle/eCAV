#!/usr/bin/env python
"""Ground truth in sensor view against camera and LiDAR perception on the in-view crossing, no edge.

    python scripts/ispass/plot_density.py            # writes paper_ispass/fig_density.pdf

(a) one point per run: when the vehicle starts to brake for the crossing car (seconds after the car
starts to move) against the closest distance between the two; collided runs marked. (b) median and
quartiles of the brake start against the number of background road users, with the collisions per
30 runs of the camera and LiDAR arm printed above its points. Input: paper_ispass/density_runs.csv
(scripts/ispass/density_reduce.py --runs-csv). Drawn at column width with 7 pt text, included at scale 1."""
import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
R = [r for r in csv.DictReader(open(os.path.join(ROOT, "paper_ispass", "density_runs.csv"))) if r["enc"] == "open" and r["valid"] == "1"]
MS = [4, 8, 16, 32, 64, 128]
ARMS = [("gt_vis", "ground truth in sensor view", "#003057", "o", "-"), ("onboard", "camera and LiDAR", "#eb6834", "s", "--")]
plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
                     "axes.titlesize": 7, "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": 0.5,
                     "grid.color": "#e3e3e3", "grid.linewidth": 0.4, "lines.linewidth": 1.0, "lines.markersize": 3,
                     "xtick.major.width": 0.5, "ytick.major.width": 0.5, "pdf.fonttype": 42})
fig, (a, b) = plt.subplots(1, 2, figsize=(3.45, 1.75))
has = lambda r: r["onset_s"] not in ("", "nan")
for arm, lab, colr, mk, ls in ARMS:
    c = [r for r in R if r["arm"] == arm and has(r)]
    ok = [r for r in c if r["collided"] == "0"]; hit = [r for r in c if r["collided"] == "1"]
    a.scatter([float(r["onset_s"]) for r in ok], [float(r["min_sep"]) for r in ok], s=5, marker=mk, color=colr, alpha=0.5, linewidths=0, label=lab)
    if hit: a.scatter([float(r["onset_s"]) for r in hit], [float(r["min_sep"]) for r in hit], s=14, marker="x", color="#c00000", linewidths=0.9, label="collision")
    md, lo, hi = [], [], []
    for m in MS:
        v = np.array([float(r["onset_s"]) for r in c if int(r["m"]) == m]); q1, q2, q3 = np.percentile(v, [25, 50, 75]); md.append(q2); lo.append(q2 - q1); hi.append(q3 - q2)
    b.errorbar(MS, md, yerr=[lo, hi], color=colr, marker=mk, ls=ls, capsize=1.5, elinewidth=0.6, capthick=0.6)
    if arm == "onboard":
        for m, y, h in zip(MS, md, hi):
            k = sum(int(r["collided"]) for r in R if r["arm"] == arm and int(r["m"]) == m)
            b.annotate(str(k), (m, y + h), textcoords="offset points", xytext=(0, 3), ha="center", fontsize=7, color="#c00000")
a.set_xlabel("brake start after car launch (s)", labelpad=1); a.set_ylabel("closest distance (m)", labelpad=1); a.set_xlim(1.6, 4.4); a.set_ylim(0, 16.5)
a.set_title("(a) each run", pad=2); a.grid(True); a.set_axisbelow(True)
b.set_xscale("log", base=2); b.set_xticks(MS); b.set_xlim(3, 170); b.set_ylim(0, 4.6)
b.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}")); b.xaxis.set_minor_formatter(NullFormatter())
b.set_xlabel("background road users", labelpad=1); b.set_ylabel("brake start (s)", labelpad=1); b.set_title("(b) by traffic density", pad=2); b.grid(True); b.set_axisbelow(True)
h, l = a.get_legend_handles_labels()
fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.5, 1.02), columnspacing=0.8, handlelength=1.0, handletextpad=0.2)
fig.subplots_adjust(left=0.105, right=0.99, bottom=0.2, top=0.79, wspace=0.36)
out = os.path.join(ROOT, "paper_ispass", "fig_density.pdf")
fig.savefig(out); print("wrote", out)
