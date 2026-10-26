#!/usr/bin/env python3
"""Shared style for the SenSys figures. Every figure is drawn at the width it is printed (acmart
sigconf: 3.35 in column, 7.0 in text width) and included at scale 1, so the point sizes below are the
printed sizes: 7 pt ticks and legends, 8 pt axis labels and panel titles, 1.5 pt lines. One colour
and marker per arm across every figure."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

COL, TXT = 3.35, 7.0

ARMS = {  # key: (label, colour, marker)
    "reference": ("Exact-state reference", "k", "o"),
    "contract": ("V2X2V prediction, +contract", "#1f77b4", "s"),
    "naive": ("V2X2V prediction, no contract", "#d62728", "^"),
    "v2x2v": ("V2X2V prediction", "#1f77b4", "s"),
    "i2v": ("I2V object sharing", "#2ca02c", "D"),
    "cip": ("Cooperative planning (CIP)", "#9467bd", "X"),
    "intermediate": ("Intermediate feature fusion", "#ff7f0e", "v"),
    "etsi": ("ETSI self-announcement", "#8c564b", "v"),
    "gate": ("Mahalanobis gate", "#e377c2", "x"),
    "gate+c": ("Mahalanobis gate, +contract", "#e377c2", "P"),
    "vips": ("VIPS rectification", "#17becf", "d"),
    "vips+c": ("VIPS rectification, +contract", "#17becf", "D"),
    "contract+valid": ("contract + validity checks", "#1f77b4", "s"),  # drawn dashed with hollow markers
}
GEOMS = {  # geometry: (label, colour)
    "openscenario_3_edge_late_fusion": ("Occluded left turn", "#0b3d91"),
    "scp_edge_late_fusion": ("Straight crossing", "#c75000"),
    "blind_overtake_edge_late_fusion": ("Blind overtake", "#2e7d32"),
}
EB = dict(capsize=2, elinewidth=0.8, capthick=0.8)  # error bar style


def rc():
    plt.rcParams.update({
        "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8, "xtick.labelsize": 7, "ytick.labelsize": 7,
        "legend.fontsize": 7, "legend.frameon": False, "legend.handlelength": 2.0, "legend.columnspacing": 1.2,
        "legend.handletextpad": 0.5, "lines.linewidth": 1.5, "lines.markersize": 4.5, "axes.linewidth": 0.6,
        "grid.linewidth": 0.4, "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.major.size": 2.5,
        "ytick.major.size": 2.5, "hatch.linewidth": 0.5, "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": 300,
        "figure.constrained_layout.use": True, "figure.constrained_layout.h_pad": 0.03,
        "figure.constrained_layout.w_pad": 0.03,
    })


def save(fig, out, png_dpi=200):
    os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
    fig.savefig(out)
    fig.savefig(os.path.splitext(out)[0] + ".png", dpi=png_dpi)
    print("wrote", out)
