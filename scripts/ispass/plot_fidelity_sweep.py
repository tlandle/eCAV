#!/usr/bin/env python
"""Figures and transcription table for the fidelity ablation delay sweep (ISPASS experiment 2).

    python scripts/ispass/plot_fidelity_sweep.py [--csv paper_ispass/fidelity.csv] [--out paper_ispass]

Reads the reduced table (scripts/ispass/fidelity_reduce.py) and writes, at printed size
(single column, 3.3 in wide, 7 pt text, included at scale 1):

  fig_fidelity.pdf      operational success against added delay, one panel per architecture,
                        the four evaluation conditions as four lines (colour, marker and dash).
                        Wilson 95 % intervals are drawn only where success is strictly between
                        0 and 1. An architecture whose network and full conditions ran with the
                        live radio gets one panel per sender slot of the connected vehicle
                        (the idealized and compute conditions repeat in each), titled with the
                        fraction of downlink messages that slot received.
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
        "intermediate fusion": "atlas, RTX 4080 Super"}


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
    # panel = (architecture, sender slot of the live radio or None). Conditions without a radio
    # (A, B) belong to every panel of their architecture.
    slot_of = lambda r: int(float(r.get("slot") or 0)) if r.get("radio") == "live" else None
    panels = []
    for a in ARCHS:
        slots = sorted({slot_of(r) for r in rows if r["arch"] == a and slot_of(r) is not None})
        panels += [(a, k) for k in slots] or ([(a, None)] if any(r["arch"] == a for r in rows) else [])
    cell = {}
    for r in rows:
        for pn in panels:
            if pn[0] == r["arch"] and (r["cond"] in "AB" or slot_of(r) == pn[1]):
                cell[(pn, r["cond"], int(r["lat"]))] = r
    lats = sorted({int(r["lat"]) for r in rows})
    return cell, lats, panels


def delivered(cell, pn, lats):
    """Mean fraction of downlink messages delivered over the network and full cells of a panel."""
    v = [1.0 - fl(cell[(pn, c, x)], "dl_drop") for c in "CD" for x in lats if (pn, c, x) in cell]
    v = [x for x in v if x == x]
    return sum(v) / len(v) if v else float("nan")


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


def fig_sweep(cell, lats, panels, path):
    plt = setup()
    from matplotlib.lines import Line2D
    fig, axes = plt.subplots(1, len(panels), figsize=(3.3, 1.55), sharey=True, squeeze=False)
    axes = axes[0]
    for ax, pn in zip(axes, panels):
        for c in CONDS:
            draw(ax, [x / 1000.0 for x in lats], lambda x: cell.get((pn, c, int(round(x * 1000)))), c)
        if pn[1] is None:
            ax.set_title(SHORT[pn[0]], pad=2, linespacing=0.95)
        else:   # second title line of a live panel; the architecture name spans the group below
            d = delivered(cell, pn, lats)
            ax.set_title("all delivered" if d > 0.995 else f"{100 * d:.0f}% delivered", pad=2)
        ax.set_xlim(-0.14, max(lats) / 1000.0 + 0.14); ax.set_ylim(-0.05, 1.05)
        ax.set_xticks([0, 1, 2]); ax.set_xticks([x / 1000.0 for x in lats], minor=True)
        ax.grid(True, lw=0.3, color="#d9d8d2"); ax.tick_params(width=0.5, length=2, pad=1.5)
        ax.tick_params(which="minor", width=0.4, length=1.2)
    axes[0].set_ylabel(YLAB); axes[0].set_yticks([0, 0.5, 1.0])
    fig.supxlabel("configured delay (s)", fontsize=7, y=0.005)
    h = [Line2D([], [], lw=1.1, ms=3.2, mew=0.8, mec=STYLE[c]["color"], label=f"{c}: {NAME[c]}", **STYLE[c]) for c in CONDS]
    fig.legend(handles=h, loc="upper center", ncol=4, frameon=False, handlelength=2.3, columnspacing=0.9,
               handletextpad=0.4, borderaxespad=0.1, bbox_to_anchor=(0.5, 1.0))
    fig.subplots_adjust(left=0.115, right=0.985, top=0.70, bottom=0.215, wspace=0.12)
    # one architecture name over the panels of its sender slots
    for a in ARCHS:
        grp = [ax for ax, pn in zip(axes, panels) if pn[0] == a and pn[1] is not None]
        if grp:
            x0 = grp[0].get_position().x0; x1 = grp[-1].get_position().x1; y1 = grp[0].get_position().y1
            fig.text((x0 + x1) / 2, y1 + 0.105, a, ha="center", va="bottom", fontsize=7)
    fig.savefig(path); plt.close(fig); print("wrote", path)


def fig_age(cell, lats, panels, path):
    """Intermediate fusion: completion against the mean forecast age at the brake. With live sender
    slots the network and full conditions are drawn once per slot (the last slot with open markers)."""
    plt = setup()
    pns = [pn for pn in panels if pn[0] == "intermediate fusion"]
    fig, ax = plt.subplots(figsize=(3.3, 1.45))
    series = [(pns[0], c) for c in "AB"] + [(pn, c) for pn in pns for c in "CD"]
    for pn, c in series:
        st = dict(STYLE[c])
        lab = f"{c}: {NAME[c]}"
        if c in "CD" and pn[1] is not None:
            lab += f", {100 * delivered(cell, pn, lats):.0f}% delivered"
            if pn != pns[0]: st.update(mfc="white", ls=(0, (1, 2.5)))
        pts = sorted(((fl(cell[(pn, c, x)], "brake_age"), cell[(pn, c, x)]) for x in lats if (pn, c, x) in cell), key=lambda p: p[0])
        ax.plot([p[0] for p in pts], [fl(p[1], "s_op") for p in pts], lw=1.1, ms=3.2, mew=0.8, mec=st["color"],
                label=lab, zorder=3, **st)
        for x, r in pts:
            if 0.0 < fl(r, "s_op") < 1.0:
                ax.plot([x, x], [fl(r, "lo"), fl(r, "hi")], color=st["color"], lw=0.6, alpha=0.7, zorder=2)
    ax.set_xlabel("forecast age at the brake (ms)", labelpad=1.5); ax.set_ylabel(YLAB)
    ax.set_ylim(-0.05, 1.05); ax.set_yticks([0, 0.5, 1.0]); ax.set_xlim(-100, 3100); ax.set_xticks(range(0, 3100, 500))
    ax.grid(True, lw=0.3, color="#d9d8d2"); ax.tick_params(width=0.5, length=2, pad=1.5)
    ax.legend(loc="lower left", frameon=False, handlelength=2.3, labelspacing=0.2, borderaxespad=0.3)
    fig.subplots_adjust(left=0.115, right=0.965, top=0.97, bottom=0.235)
    fig.savefig(path); plt.close(fig); print("wrote", path)


def pname(pn, cell, lats, c):
    """Row name of a panel and condition: the sender slot is named only where the radio is on."""
    if pn[1] is None or c in "AB":
        return pn[0]
    return f"{pn[0]}, slot {pn[1]} ({100 * delivered(cell, pn, lats):.0f}% delivered)"


def table(cell, lats, panels, path, stamp):
    rows = []       # (panel, condition), the radio-free conditions of an architecture once
    for pn in panels:
        for c in CONDS:
            if c in "AB" and any(q[0] == pn[0] and q != pn for q in panels[:panels.index(pn)]):
                continue
            rows.append((pn, c))
    L = [f"# Fidelity ablation, straight crossing (stamp {stamp})\n",
         "Ten seeds per cell, 70 km/h, one connected vehicle and two roadside units, 24 senders on the radio. "
         "I2V object sharing and V2X2V prediction resolve the radio from the sampled tables. Intermediate fusion (16.9 KB "
         "messages) resolves it with the live ns-3 process, once with the connected vehicle in a sender slot that the "
         "scheduler serves in full and once in a slot that receives about half of its downlink messages. Its full "
         "condition replays the execution times recorded in the compute condition of the same delay and repetition. "
         "Source: paper_ispass/fidelity.csv.\n",
         "## Fraction of runs completed without a collision, by added delay (ms)\n",
         "| architecture | condition | " + " | ".join(str(x) for x in lats) + " |", "|---|---|" + "---|" * len(lats)]
    for pn, c in rows:
        L.append(f"| {pname(pn, cell, lats, c)} | {c} {NAME[c]} | " + " | ".join(f"{fl(cell[(pn, c, x)], 's_op'):.2f}" if (pn, c, x) in cell else "" for x in lats) + " |")
    L += ["\n## Fraction of runs that fail on a false brake without a collision\n",
          "A false brake is a sustained brake that starts on a track the labeler does not match to the crossing car, "
          "or a brake on the crossing car's forecast after the car has passed.\n",
          "| architecture | condition | " + " | ".join(str(x) for x in lats) + " |", "|---|---|" + "---|" * len(lats)]
    for pn, c in rows:
        L.append(f"| {pname(pn, cell, lats, c)} | {c} {NAME[c]} | " + " | ".join(f"{fl(cell[(pn, c, x)], 'fb_only'):.2f}" if (pn, c, x) in cell else "" for x in lats) + " |")
    L += ["\n## Mean forecast age at the brake (ms), by added delay (ms)\n",
          "| architecture | condition | " + " | ".join(str(x) for x in lats) + " |", "|---|---|" + "---|" * len(lats)]
    for pn, c in rows:
        L.append(f"| {pname(pn, cell, lats, c)} | {c} {NAME[c]} | " + " | ".join(f"{fl(cell[(pn, c, x)], 'brake_age'):.0f}" if (pn, c, x) in cell else "" for x in lats) + " |")
    L += ["\n## Boundary\n",
          "Last cell with the rate at or above 0.90 and first cell at or below 0.40, as added delay and as the mean forecast age at the brake in that cell (ms).\n",
          "| architecture | condition | last >= 0.90: added delay | forecast age | success | first <= 0.40: added delay | forecast age | success |",
          "|---|---|---|---|---|---|---|---|"]
    for pn, c in rows:
        ok = [x for x in lats if (pn, c, x) in cell and fl(cell[(pn, c, x)], "s_op") >= 0.90]
        bad = [x for x in lats if (pn, c, x) in cell and fl(cell[(pn, c, x)], "s_op") <= 0.40]
        lo = max(ok) if ok else None; hi = min(bad) if bad else None
        f = lambda x, k, fmt: (format(fl(cell[(pn, c, x)], k), fmt) if x is not None else "none")
        L.append(f"| {pname(pn, cell, lats, c)} | {c} {NAME[c]} | {lo if lo is not None else 'none'} | {f(lo, 'brake_age', '.0f')} | {f(lo, 's_op', '.2f')} | "
                 f"{hi if hi is not None else 'none'} | {f(hi, 'brake_age', '.0f')} | {f(hi, 's_op', '.2f')} |")
    L += ["\n## Delay terms\n",
          "Mean over the cells of each condition (ms), range over the cells in parentheses. Compute is the execution time charged to each forecast. "
          "Uplink and downlink are the radio delays of delivered messages at 24 senders, and dropped is the fraction of downlink messages not delivered.\n",
          "| architecture | host and GPU | compute (B, D) | uplink (C, D) | downlink mean (C, D) | downlink p95 (C, D) | dropped (C, D) |", "|---|---|---|---|---|---|---|"]
    for pn in panels:
        m = lambda conds, k: [v for v in (fl(cell[(pn, c, x)], k) for c in conds for x in lats if (pn, c, x) in cell) if v == v]
        r = lambda v, fmt=".0f": f"{sum(v) / len(v):{fmt}} ({min(v):{fmt}} to {max(v):{fmt}})" if v else ""
        L.append(f"| {pname(pn, cell, lats, 'C')} | {HOST[pn[0]]} | {r(m('BD', 'compute_ms'))} | {r(m('CD', 'ul_ms'))} | {r(m('CD', 'dl_ms'))} | "
                 f"{r(m('CD', 'dl_p95_ms'))} | {r(m('CD', 'dl_drop'), '.2f')} |")
    L += ["\n## Notes\n",
          "- One code version for every cell (develop cc8a5523, chain of Nov 20 to 22 on atlas).",
          "- The live ns-3 scheduler serves senders by slot and the pattern repeats across seeds. Each repetition uses its own ns-3 stream.",
          "- False brakes are listed separately from collisions."]
    open(path, "w").write("\n".join(L) + "\n"); print("wrote", path)


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__)); root = os.path.abspath(os.path.join(here, "..", ".."))
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=os.path.join(root, "paper_ispass", "fidelity.csv"))
    ap.add_argument("--out", default=os.path.join(root, "paper_ispass"))
    ap.add_argument("--metric", default="nocollision", choices=["nocollision", "success"])
    ap.add_argument("--stamp", default="ispass_fid11", help="stamp named in the table file")
    a = ap.parse_args()
    if a.metric == "success":
        YLAB = "operational success"
    cell, lats, panels = load(a.csv, a.metric)
    fig_sweep(cell, lats, panels, os.path.join(a.out, "fig_fidelity.pdf"))
    fig_age(cell, lats, panels, os.path.join(a.out, "fig_fidelity_age.pdf"))
    table(cell, lats, panels, os.path.join(a.out, "fidelity_table.md"), a.stamp)
