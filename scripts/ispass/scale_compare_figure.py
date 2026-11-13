#!/usr/bin/env python3
"""Tick time against the vehicle count on one pair of axes per layout: OpenCDA, eCAV on one host, eCAV on several hosts.

    python scripts/ispass/scale_compare_figure.py <multi-host stamp> [--single scale_sweep3] [--out paper_ispass]

Reads paper_ispass/scale_competitor.json (OpenCDA), paper_ispass/<single>.json (eCAV, every process on one host)
and paper_ispass/scale_<stamp>.json (eCAV, vehicle clients on other hosts; written by scale_report.py).
Writes fig_scale_<stamp>_compare.pdf (no perception), fig_scale_<stamp>_perception_compare.pdf and
scale_<stamp>_compare.md. Every curve is the median tick wall time. Four and eight replicas are in the table only
(their curves lie on the two-replica curve).
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def main():
    stamp = sys.argv[1]
    arg = lambda k, d: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else d
    out = arg("--out", os.path.join(ROOT, "paper_ispass"))
    single = json.load(open(os.path.join(out, arg("--single", "scale_sweep3") + ".json")))
    multi = json.load(open(os.path.join(out, f"scale_{stamp}.json")))
    comp = json.load(open(os.path.join(out, "scale_competitor.json")))
    series = {}   # layout -> label -> {n: median ms}
    for lay in ("dense", "spread"):
        s = series[lay] = {}
        s["OpenCDA"] = {r["n"]: r["opencda_default"]["tick_ms"]["p50"] for r in comp
                        if r["layout"] == lay and r["perception"] == "off" and r["opencda_default"].get("completed")}
        s["OpenCDA, map manager off"] = {r["n"]: r["opencda_map_off"]["tick_ms"]["p50"] for r in comp
                                         if r["layout"] == lay and r["perception"] == "off" and "opencda_map_off" in r}
        s["eCAV, one host"] = {r["n"]: r["wall_p50"] for r in single if r["config"] == lay}
        s["eCAV, three hosts"] = {r["n"]: r["wall_p50"] for r in multi if r["config"] == "mh" + lay}
    ns = sorted({n for lay in series.values() for v in lay.values() for n in v})
    L = ["| layout | system | " + " | ".join(str(n) for n in ns) + " |", "|---|---|" + "---|" * len(ns)]
    for lay, s in series.items():
        for lab, v in s.items():
            L.append(f"| {lay} | {lab} | " + " | ".join(f"{v[n]:.0f}" if n in v else "-" for n in ns) + " |")
    md = "Median tick wall time (ms) against the number of vehicles, no perception.\n\n" + "\n".join(L) + "\n"
    open(os.path.join(out, f"scale_{stamp}_compare.md"), "w").write(md)
    print(md)

    # perception rows: OpenCDA (detector in the vehicle loop), eCAV on one host (shared service beside the simulator),
    # eCAV on three hosts with R service replicas on a fourth
    # --perception <stamp>: the perception rows (one host and distributed) come from scale_<stamp>.json, e.g. the
    # rows measured with the sensor wait; without it they come from the single-host file and the multi-host stamp
    pst = arg("--perception", None)
    prow = json.load(open(os.path.join(out, f"scale_{pst}.json"))) if pst else None
    pser = {"OpenCDA": {r["n"]: r["opencda_default"]["tick_ms"]["p50"] for r in comp
                        if r["layout"] == "dense" and r["perception"] == "on" and r["opencda_default"].get("completed")},
            "eCAV, one host": {r["n"]: r["wall_p50"] for r in (prow or single) if r["config"] == "pdense"}}
    for rr in (1, 2, 4, 8):
        pser[f"eCAV, distributed, {rr} replica{'s' if rr > 1 else ''}"] = {r["n"]: r["wall_p50"] for r in (prow or multi) if r["config"] == f"mhpr{rr}"}
    pns = sorted({n for v in pser.values() for n in v})
    L = ["| system | " + " | ".join(str(n) for n in pns) + " |", "|---|" + "---|" * len(pns)]
    for lab, v in pser.items():
        L.append(f"| {lab} | " + " | ".join(f"{v[n]:.0f}" if n in v else "-" for n in pns) + " |")
    pmd = ("\nMedian tick wall time (ms) against the number of vehicles, perception on (four cameras and one LiDAR per vehicle, "
           "dense layout). OpenCDA at 64 vehicles: 40 measured ticks. OpenCDA at 128 vehicles does not start (the simulator "
           "ends with a segmentation fault at 22.1 GB of GPU memory).\n\n" + "\n".join(L) + "\n")
    open(os.path.join(out, f"scale_{stamp}_compare.md"), "a").write(pmd)
    print(pmd)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "legend.fontsize": 6.5, "axes.linewidth": 0.6, "pdf.fonttype": 42})
    style = {"OpenCDA": ("0.25", "x", "-"), "OpenCDA, map manager off": ("0.55", "x", ":"),
             "eCAV, one host": ("C0", "o", "-"), "eCAV, three hosts": ("C3", "s", "-")}
    fig, axes = plt.subplots(1, 2, figsize=(3.4, 1.75), sharey=True)
    for ax, (lay, s) in zip(axes, series.items()):
        for lab, v in s.items():
            if v:
                col, mk, ls = style[lab]
                x = sorted(v)
                ax.plot(x, [v[n] for n in x], color=col, marker=mk, ls=ls, ms=2.6, lw=0.9, label=lab)
        ax.set_xscale("log", base=2); ax.set_yscale("log")
        ax.set_xticks(ns); ax.set_xticklabels([str(n) for n in ns], rotation=90); ax.minorticks_off()
        ax.set_xlabel(f"vehicles N ({lay})"); ax.grid(True, which="major", lw=0.3, color="0.85")
    axes[0].set_ylabel("tick time, median (ms)")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, frameon=False, loc="upper center", ncol=2, handlelength=1.8, borderpad=0.1, labelspacing=0.15,
               columnspacing=1.0, bbox_to_anchor=(0.55, 1.01))
    fig.subplots_adjust(left=0.15, right=0.995, bottom=0.3, top=0.82, wspace=0.08)
    path = os.path.join(out, f"fig_scale_{stamp}_compare.pdf")
    fig.savefig(path)
    print(path)

    pstyle = {"OpenCDA": ("0.25", "x", "-"), "eCAV, one host": ("C0", "o", "-"),
              "eCAV, distributed, 1 replica": ("C3", "s", ":"), "eCAV, distributed, 2 replicas": ("C3", "s", "-")}
    fig, ax = plt.subplots(figsize=(3.4, 1.9))
    for lab, (col, mk, ls) in pstyle.items():
        v = pser.get(lab)
        if v:
            x = sorted(v)
            ax.plot(x, [v[n] for n in x], color=col, marker=mk, ls=ls, ms=2.6, lw=0.9, label=lab)
    ax.set_xscale("log", base=2); ax.set_yscale("log")
    ax.set_xticks(pns); ax.set_xticklabels([str(n) for n in pns]); ax.minorticks_off()
    ax.set_xlabel("vehicles N, perception on"); ax.set_ylabel("tick time, median (ms)")
    ax.grid(True, which="major", lw=0.3, color="0.85")
    ax.legend(frameon=False, loc="lower right", handlelength=1.8, borderpad=0.1, labelspacing=0.15)
    fig.subplots_adjust(left=0.16, right=0.99, bottom=0.2, top=0.97)
    path = os.path.join(out, f"fig_scale_{stamp}_perception_compare.pdf")
    fig.savefig(path)
    print(path)


if __name__ == "__main__":
    main()
