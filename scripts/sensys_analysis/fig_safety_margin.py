#!/usr/bin/env python3
"""Safety margin figure: the maneuver-specific useful-age budget (fig_situation_tau.csv, reference
rows) minus each architecture's landing p99(Delta_use) (landing_ci.md), with both intervals.
    python scripts/sensys_analysis/fig_safety_margin.py --tau safety_envelope_paper/analysis/fig_situation_j3b/fig_situation_tau.csv \
        --landing safety_envelope_paper/analysis/landing_ci.md --out figures/safety_margin.pdf [--N 16 24] [--speeds 70 88]
Writes the PDF and a CSV beside it (one row per architecture, N, geometry, speed).
"""
import argparse, csv, os, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

GEOM = {"openscenario_3_edge_late_fusion": "Left turn", "scp_edge_late_fusion": "Straight crossing"}
ARCH = [("oracle", "Reference"), ("object_i2v", "I2V object sharing"), ("coop_pred", "V2X2V prediction"),
        ("coop_planning", "CIP planning"), ("intermediate", "Intermediate fusion")]


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
    fig, axes = plt.subplots(1, len(a.N), figsize=(3.4 * len(a.N), 2.6), sharey=True)
    axes = [axes] if len(a.N) == 1 else list(axes)
    out_rows = []
    for ax, N in zip(axes, a.N):
        x = list(range(len(cells)))
        for i, (ak, aname) in enumerate(ARCH):
            if (ak, N) not in land:
                continue
            p99, lo, hi = land[(ak, N)]
            ys = [tau[c][0] - p99 for c in cells]
            yerr = [[tau[c][0] - tau[c][1] + (hi - p99) for c in cells], [tau[c][2] - tau[c][0] + (p99 - lo) for c in cells]]
            ax.errorbar([xx + (i - 2) * 0.13 for xx in x], ys, yerr=yerr, fmt="o", ms=4, capsize=2, label=aname)
            for c, y in zip(cells, ys):
                out_rows.append({"arch": ak, "N": N, "geom": c[0], "speed_kmh": c[1], "tau_ms": tau[c][0], "p99_ms": p99, "margin_ms": y})
        ax.axhline(0, color="k", lw=0.8)
        ax.set_xticks(x); ax.set_xticklabels([f"{GEOM[g]}\n{s} km/h" for g, s in cells], fontsize=7)
        ax.set_title(f"N = {N}", fontsize=9); ax.grid(alpha=0.3)
    axes[0].set_ylabel("safety margin $m$ (ms)")
    axes[-1].legend(fontsize=6.5, loc="upper right")
    fig.tight_layout(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out); fig.savefig(os.path.splitext(a.out)[0] + ".png", dpi=150)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out_rows[0].keys())); w.writeheader(); w.writerows(out_rows)
    print("wrote", a.out, len(out_rows), "rows")


if __name__ == "__main__":
    main()
