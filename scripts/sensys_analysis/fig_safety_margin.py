#!/usr/bin/env python3
"""Safety margin figure: the maneuver-specific useful-age budget (fig_situation_tau.csv, reference
rows) minus each architecture's landing p99(Delta_use) (landing_ci.md), with both intervals. Drawn at
text width (figstyle.TXT) for a figure* float, one panel per N.
    python scripts/sensys_analysis/fig_safety_margin.py --tau safety_envelope_paper/analysis/fig_situation_j3b/fig_situation_tau.csv \
        --landing safety_envelope_paper/analysis/landing_ci.md --out figures/safety_margin.pdf [--N 16 24] [--speeds 70 88]
Writes the PDF and a CSV beside it (one row per architecture, N, geometry, speed).
"""
import argparse, csv, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402

GEOM = {"openscenario_3_edge_late_fusion": "Left turn", "scp_edge_late_fusion": "Crossing"}
ARCH = [("oracle", "reference"), ("object_i2v", "i2v"), ("coop_pred", "v2x2v"), ("coop_planning", "cip"), ("intermediate", "intermediate")]


def read_landing(path):
    rows = {}
    for line in open(path):
        m = re.match(r"\|\s*(\w+)\s*\|\s*(\d+)\s*\|\s*\d+\s*\|\s*\d+\s*\|\s*(\d+)\s*\|\s*\[(\d+), (\d+)\]\s*\|\s*\[(\d+), (\d+)\]", line)
        if m:
            rows[(m.group(1), int(m.group(2)))] = (int(m.group(3)), int(m.group(6)), int(m.group(7)))  # p99, locale CI
    return rows


def read_tau(path):
    tau = {}
    for r in csv.DictReader(open(path)):
        if r["arm"] != "oracle":
            continue
        tau[(r["geom"], int(r["speed_kmh"]))] = (float(r["tau_ms"]), float(r["boot_lo_ms"]), float(r["boot_hi_ms"]))
    return tau


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tau", required=True); ap.add_argument("--landing", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--N", nargs="+", type=int, default=[16, 24])
    ap.add_argument("--speeds", nargs="+", type=int, default=[70, 88])
    a = ap.parse_args()
    land, tau = read_landing(a.landing), read_tau(a.tau)
    cells = [(g, s) for g in GEOM for s in a.speeds if (g, s) in tau]
    figstyle.rc()
    fig, axes = plt.subplots(1, len(a.N), figsize=(figstyle.TXT, 1.7), sharey=True)
    axes = [axes] if len(a.N) == 1 else list(axes)
    out_rows = []; ymin, ymax = 0.0, 0.0
    for ax, N in zip(axes, a.N):
        x = list(range(len(cells)))
        for i, (ak, key) in enumerate(ARCH):
            if (ak, N) not in land:
                continue
            aname, color, marker = figstyle.ARMS[key]
            p99, lo, hi = land[(ak, N)]
            ys = [tau[c][0] - p99 for c in cells]
            yerr = [[tau[c][0] - tau[c][1] + (hi - p99) for c in cells], [tau[c][2] - tau[c][0] + (p99 - lo) for c in cells]]
            ymin = min(ymin, min(y - e for y, e in zip(ys, yerr[0]))); ymax = max(ymax, max(y + e for y, e in zip(ys, yerr[1])))
            ax.errorbar([xx + (i - 2) * 0.15 for xx in x], ys, yerr=yerr, fmt=marker, color=color, ls="none", label=aname, **figstyle.EB)
            for c, y in zip(cells, ys):
                out_rows.append({"arch": ak, "N": N, "geom": c[0], "speed_kmh": c[1], "tau_ms": tau[c][0], "p99_ms": p99, "margin_ms": y})
        ax.axhline(0, color="k", lw=1.2)
        ax.set_xticks(x); ax.set_xticklabels([f"{GEOM[g]}\n{s} km/h" for g, s in cells]); ax.set_xlim(-0.5, len(cells) - 0.5)
        ax.set_title(f"$N = {N}$"); ax.grid(alpha=0.3)
    axes[0].set_ylabel("safety margin $m$ (ms)"); axes[0].set_ylim(min(-50.0, ymin - 20), ymax + 20)
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="outside upper center", ncol=3)
    figstyle.save(fig, a.out)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys())); w.writeheader(); w.writerows(out_rows)
    print("wrote", a.out, len(out_rows), "rows")


if __name__ == "__main__":
    main()
