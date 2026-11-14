#!/usr/bin/env python
"""Figures and transcription table for the fidelity ablation delay sweep (ISPASS experiment 2).

    python scripts/ispass/plot_fidelity_sweep.py [--csv paper_ispass/fidelity.csv] [--out paper_ispass]

Reads the reduced table (scripts/ispass/fidelity_reduce.py) and writes, at printed size
(single column, 3.3 in wide, 7 pt text, included at scale 1):

  fig_fidelity.pdf      operational success against added delay, one panel per architecture,
                        the four evaluation conditions as four lines (colour, marker and dash).
                        Wilson 95 % intervals are drawn only where success is strictly between
                        0 and 1.
  fig_fidelity_age.pdf  intermediate fusion only: success against the mean forecast age at the
                        brake, to show which conditions fall on one curve.
  fidelity_table.md     success table, boundary table and delay terms.
"""
import argparse, csv, math, os

ARCHS = ["I2V object sharing", "V2X2V prediction", "intermediate fusion"]
SHORT = {"I2V object sharing": "I2V object\nsharing", "V2X2V prediction": "V2X2V\nprediction", "intermediate fusion": "intermediate\nfusion"}
CONDS = ["A", "B", "C", "D"]
NAME = {"A": "idealized", "B": "compute", "C": "network", "D": "full"}
STYLE = {"A": dict(color="#52514e", ls="-", marker="o", mfc="#52514e"),
         "B": dict(color="#2f6fb0", ls="--", marker="s", mfc="white"),
         "C": dict(color="#d0801f", ls="-.", marker="^", mfc="white"),
         "D": dict(color="#a8322d", ls=":", marker="D", mfc="#a8322d")}
HOST = {"I2V object sharing": "atlas, RTX 4080 Super", "V2X2V prediction": "atlas, RTX 4080 Super",
        "intermediate fusion": "cetus, RTX 3080 Ti"}


def fl(r, k):
    try:
        return float(r[k])
    except (ValueError, KeyError):
        return float("nan")


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    ph = k / n; d = 1 + z * z / n; c = ph + z * z / (2 * n); h = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n))
    return max(0.0, (c - h) / d), min(1.0, (c + h) / d)


def load(path, metric="nocollision"):
    """metric nocollision: the plotted rate is the fraction of runs completed without a collision
    (s_op, lo, hi are replaced; the success rate of the reducer is kept as s_all, and the fraction
    of runs that fail on a false brake without a collision as fb_only)."""
    rows = [r for r in csv.DictReader(open(path)) if r["arch"] in ARCHS]
    for r in rows:
        n = int(float(r["n"])); r["s_all"] = r["s_op"]
        r["fb_only"] = f"{max(0.0, 1.0 - float(r['collided']) - float(r['s_op'])):.2f}"
        if metric == "nocollision":
            k = round((1.0 - float(r["collided"])) * n); lo, hi = wilson(k, n)
            r["s_op"], r["lo"], r["hi"] = f"{k / n:.2f}", f"{lo:.4f}", f"{hi:.4f}"
    cell = {(r["arch"], r["cond"], int(r["lat"])): r for r in rows}
    lats = sorted({int(r["lat"]) for r in rows})
    return cell, lats


YLAB = "completed without collision"


def setup():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "legend.fontsize": 7, "axes.titlesize": 7, "axes.linewidth": 0.5, "pdf.fonttype": 42,
                         "font.family": "DejaVu Sans"})
    return plt


def draw(ax, xs, cell_of, cond):
    st = STYLE[cond]
    pts = [(x, cell_of(x)) for x in xs]
    pts = [(x, r) for x, r in pts if r is not None]
    X = [x for x, _ in pts]; Y = [fl(r, "s_op") for _, r in pts]
    ax.plot(X, Y, lw=1.1, ms=3.2, mew=0.8, mec=st["color"], zorder=3, **st)
    for x, r in pts:
        s = fl(r, "s_op")
        if 0.0 < s < 1.0:
            ax.plot([x, x], [fl(r, "lo"), fl(r, "hi")], color=st["color"], lw=0.6, alpha=0.7, zorder=2)


def fig_sweep(cell, lats, path):
    plt = setup()
    from matplotlib.lines import Line2D
    fig, axes = plt.subplots(1, 3, figsize=(3.3, 1.55), sharey=True)
    for ax, a in zip(axes, ARCHS):
        for c in CONDS:
            draw(ax, lats, lambda x: cell.get((a, c, x)), c)
        ax.set_title(SHORT[a], pad=2, linespacing=0.95)
        ax.set_xlim(-120, max(lats) + 120); ax.set_ylim(-0.05, 1.05)
        ax.set_xticks([0, 1000, 2000]); ax.set_xticks(lats, minor=True)   # labels clear of the panel edges
        ax.grid(True, lw=0.3, color="#d9d8d2"); ax.tick_params(width=0.5, length=2, pad=1.5)
        ax.tick_params(which="minor", width=0.4, length=1.2)
    axes[0].set_ylabel(YLAB); axes[0].set_yticks([0, 0.5, 1.0])
    axes[1].set_xlabel("added delay (ms)", labelpad=1.5)
    h = [Line2D([], [], lw=1.1, ms=3.2, mew=0.8, mec=STYLE[c]["color"], label=f"{c}: {NAME[c]}", **STYLE[c]) for c in CONDS]
    fig.legend(handles=h, loc="upper center", ncol=4, frameon=False, handlelength=2.3, columnspacing=0.9,
               handletextpad=0.4, borderaxespad=0.1, bbox_to_anchor=(0.5, 1.0))
    fig.subplots_adjust(left=0.115, right=0.985, top=0.70, bottom=0.215, wspace=0.12)
    fig.savefig(path); plt.close(fig); print("wrote", path)


def fig_age(cell, lats, path):
    plt = setup()
    a = "intermediate fusion"
    fig, ax = plt.subplots(figsize=(3.3, 1.45))
    for c in CONDS:
        st = STYLE[c]
        pts = sorted((fl(cell[(a, c, x)], "brake_age"), cell[(a, c, x)]) for x in lats if (a, c, x) in cell)
        ax.plot([p[0] for p in pts], [fl(p[1], "s_op") for p in pts], lw=1.1, ms=3.2, mew=0.8, mec=st["color"],
                label=f"{c}: {NAME[c]}", zorder=3, **st)
        for x, r in pts:
            if 0.0 < fl(r, "s_op") < 1.0:
                ax.plot([x, x], [fl(r, "lo"), fl(r, "hi")], color=st["color"], lw=0.6, alpha=0.7, zorder=2)
    ax.set_xlabel("forecast age at the brake (ms)", labelpad=1.5); ax.set_ylabel(YLAB)
    ax.set_ylim(-0.05, 1.05); ax.set_yticks([0, 0.5, 1.0]); ax.set_xlim(-100, 3000); ax.set_xticks(range(0, 3000, 500))
    ax.grid(True, lw=0.3, color="#d9d8d2"); ax.tick_params(width=0.5, length=2, pad=1.5)
    ax.legend(loc="lower left", frameon=False, handlelength=2.3, labelspacing=0.2, borderaxespad=0.3)
    fig.subplots_adjust(left=0.115, right=0.965, top=0.97, bottom=0.235)
    fig.savefig(path); plt.close(fig); print("wrote", path)


def table(cell, lats, path):
    L = ["# Fidelity ablation, straight crossing (stamp ispass_fid10)\n",
         "Ten seeds per cell, 70 km/h, one connected vehicle and two roadside units, table radio load 24 senders. "
         "Source: paper_ispass/fidelity.csv.\n",
         "## Fraction of runs completed without a collision, by added delay (ms)\n",
         "| architecture | condition | " + " | ".join(str(x) for x in lats) + " |", "|---|---|" + "---|" * len(lats)]
    for a in ARCHS:
        for c in CONDS:
            L.append(f"| {a} | {c} {NAME[c]} | " + " | ".join(f"{fl(cell[(a, c, x)], 's_op'):.2f}" if (a, c, x) in cell else "" for x in lats) + " |")
    L += ["\n## Fraction of runs that fail on a false brake without a collision\n",
          "A false brake is a sustained brake that starts on a track the labeler does not match to the crossing car, "
          "or a brake on the crossing car's forecast after the car has passed.\n",
          "| architecture | condition | " + " | ".join(str(x) for x in lats) + " |", "|---|---|" + "---|" * len(lats)]
    for a in ARCHS:
        for c in CONDS:
            L.append(f"| {a} | {c} {NAME[c]} | " + " | ".join(f"{fl(cell[(a, c, x)], 'fb_only'):.2f}" if (a, c, x) in cell else "" for x in lats) + " |")
    L += ["\n## Boundary\n",
          "Last cell with the rate at or above 0.90 and first cell at or below 0.40, as added delay and as the mean forecast age at the brake in that cell (ms).\n",
          "| architecture | condition | last >= 0.90: added delay | forecast age | success | first <= 0.40: added delay | forecast age | success |",
          "|---|---|---|---|---|---|---|---|"]
    for a in ARCHS:
        for c in CONDS:
            ok = [x for x in lats if (a, c, x) in cell and fl(cell[(a, c, x)], "s_op") >= 0.90]
            bad = [x for x in lats if (a, c, x) in cell and fl(cell[(a, c, x)], "s_op") <= 0.40]
            lo = max(ok) if ok else None; hi = min(bad) if bad else None
            f = lambda x, k, fmt: (format(fl(cell[(a, c, x)], k), fmt) if x is not None else "none")
            L.append(f"| {a} | {c} {NAME[c]} | {lo if lo is not None else 'none'} | {f(lo, 'brake_age', '.0f')} | {f(lo, 's_op', '.2f')} | "
                     f"{hi if hi is not None else 'none'} | {f(hi, 'brake_age', '.0f')} | {f(hi, 's_op', '.2f')} |")
    L += ["\n## Delay terms\n",
          "Mean over the cells of each condition (ms). Compute is the measured service time charged to each forecast; uplink and downlink are the table radio delays at 24 senders.\n",
          "| architecture | host and GPU | compute (B, D) | uplink (C, D) | downlink (C, D) |", "|---|---|---|---|---|"]
    for a in ARCHS:
        m = lambda conds, k: [fl(cell[(a, c, x)], k) for c in conds for x in lats if (a, c, x) in cell]
        comp = m("BD", "compute_ms"); ul = m("CD", "ul_ms"); dl = m("CD", "dl_ms")
        r = lambda v: f"{sum(v) / len(v):.0f} ({min(v):.0f} to {max(v):.0f})" if v else ""
        L.append(f"| {a} | {HOST[a]} | {r(comp)} | {r(ul)} | {r(dl)} |")
    L += ["\n## Notes\n",
          "- Code versions. Every cell with a composed delay (conditions B, C and D of all three architectures) ran at develop bcea7b2d or later "
          "(object-level arms at fd75bed5), which makes a recipient keep the newest forecast by source tick. Before it, an older forecast with a longer "
          "downlink sample could be delivered after a newer one and replace it (10.4 % of arrivals in intermediate fusion C). The earlier runs are in "
          "experiment_results/_prefix_reorder/ and _prefix_reorder_obj/. Condition A has no composed delay and is from the campaign before that commit; "
          "intermediate fusion B is also from before it.",
          "- Evidence that the fix does not act on intermediate fusion A and B (one run each at bcea7b2d against the campaign run of the same seed; "
          "sensor data is not repeatable across simulator sessions, so the comparison is on arrivals, control and onset): A at 1800 ms has the same "
          "forecast arrival sequence, the same control and ego state on all 346 ticks and the same onset (+3550 ms), with 0 forecasts superseded; "
          "B at 1500 ms has 27 arrivals in both, no arrival out of order, 0 superseded, onset +3400 against +3450 ms.",
          "- False brakes are listed separately from collisions. In intermediate fusion C at 1200 ms (3 of 10 runs) and 1500 ms (1 of 10) the brake "
          "is on a parked car's track beside the lane."]
    open(path, "w").write("\n".join(L) + "\n"); print("wrote", path)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__)); root = os.path.abspath(os.path.join(here, "..", ".."))
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(root, "paper_ispass", "fidelity.csv"))
    ap.add_argument("--out", default=os.path.join(root, "paper_ispass"))
    ap.add_argument("--metric", default="nocollision", choices=["nocollision", "success"])
    a = ap.parse_args()
    if a.metric == "success":
        YLAB = "operational success"
    cell, lats = load(a.csv, a.metric)
    fig_sweep(cell, lats, os.path.join(a.out, "fig_fidelity.pdf"))
    fig_age(cell, lats, os.path.join(a.out, "fig_fidelity_age.pdf"))
    table(cell, lats, os.path.join(a.out, "fidelity_table.md"))
