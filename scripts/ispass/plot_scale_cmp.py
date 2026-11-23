#!/usr/bin/env python
"""Wall-clock time per tick against vehicle count, without and with camera and LiDAR perception:
OpenCDA 0.1.3 on one host, eCAV on one host, eCAV on several hosts (vehicle clients on two more
hosts and, with perception, the inference service on a fourth).

    python scripts/ispass/plot_scale_cmp.py            # writes paper_ispass/fig_scale_cmp.pdf

Values are mean tick times. eCAV: campaign mh3 (edge processes of at most 32 vehicles, sensor frame wait,
experiment_results/ispass_scale/mh3_*, reduced with scale_breakdown.py). With perception on several hosts: the row
with four inference replicas, mh3_mhpr4_n8 to n128. OpenCDA without perception:
paper_ispass/scale_competitor.md. OpenCDA with perception: mh3_opencda_matched (cameras at 640 by 480
pixels and 70 degrees, as in eCAV). Drawn at column width with 7 pt text, included at scale 1."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, NullFormatter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
N = [8, 16, 32, 64, 128, 256, 512]
OFF = {"OpenCDA": (N[:6], [162, 378, 918, 2155, 4913, 14038]),
       "one host": (N[:6], [63, 101, 201, 395, 834, 1704]),
       "several hosts": (N[3:], [307, 502, 817, 1516])}
ON = {"OpenCDA": (N[:3], [1290, 2726, 5695]),
      "one host": (N[:3], [435, 812, 1559]),
      "several hosts": (N[:5], [354, 657, 1190, 2131, 4749])}
STYLE = {"OpenCDA": dict(color="#6e6e6e", marker="x", ls="-"), "one host": dict(color="#2a78d6", marker="o", ls="-"),
         "several hosts": dict(color="#eb6834", marker="s", ls="--")}
plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
                     "axes.titlesize": 7, "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": 0.5,
                     "grid.color": "#e3e3e3", "grid.linewidth": 0.4, "lines.linewidth": 1.0, "lines.markersize": 3,
                     "xtick.major.width": 0.5, "ytick.major.width": 0.5, "pdf.fonttype": 42})
fig, axs = plt.subplots(1, 2, figsize=(3.45, 1.5), sharey=True)
for ax, data, ttl in zip(axs, (OFF, ON), ("without perception", "cameras and LiDAR")):
    for k, (x, y) in data.items():
        ax.plot(x, [v / 1000 for v in y], label=("OpenCDA" if k == "OpenCDA" else "ours, " + k), **STYLE[k])
    ax.set_xscale("log", base=2); ax.set_yscale("log"); ax.set_xticks(N); ax.set_xlim(6, 700); ax.set_ylim(0.04, 30)
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}")); ax.xaxis.set_minor_formatter(NullFormatter())
    ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}")); ax.yaxis.set_minor_formatter(NullFormatter())
    ax.tick_params(axis="x", labelrotation=60, pad=1); ax.grid(True); ax.set_axisbelow(True)
    ax.set_title(ttl, pad=2); ax.set_xlabel("vehicles", labelpad=1)
axs[0].set_ylabel("time per tick (s)", labelpad=1)
h, l = axs[0].get_legend_handles_labels()
fig.legend(h, l, loc="upper center", ncol=3, frameon=False, bbox_to_anchor=(0.54, 1.01), columnspacing=1.0, handlelength=1.8, handletextpad=0.4)
fig.subplots_adjust(left=0.115, right=0.995, bottom=0.28, top=0.76, wspace=0.08)
out = os.path.join(ROOT, "paper_ispass", "fig_scale_cmp.pdf")
fig.savefig(out); print("wrote", out)
