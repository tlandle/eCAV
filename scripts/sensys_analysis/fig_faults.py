#!/usr/bin/env python3
"""Fault study figures. Two modes.
Per geometry (reads the N=4 beacon_fault cells of a stamp, writes a CSV and a one-row figure):
    python scripts/sensys_analysis/fig_faults.py --stamp sensys_nov27_j3 --roots experiment_results \
        --geom openscenario_3_edge_late_fusion --lat 400 --out figures/faults_lt.pdf
Composite (reads the per-geometry CSVs): the full grid, one row per geometry and one panel per fault
type, at text width for the appendix, and the summary at column width for the body: P(S_op) per fault
type and arm pooled over every severity of both geometries, Wilson bars.
    python scripts/sensys_analysis/fig_faults.py --csvs faults_lt.csv faults_scp.csv \
        --labels "left turn, 400 ms" "straight crossing, 200 ms" --grid figures/faults_grid.pdf --summary figures/faults_summary.pdf
CSV columns: fault, value, arm, n, k, p, lo, hi, collisions, ghost_episodes, fp_episodes.
"""
import argparse, collections, csv, glob, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts"))
from campaign_check import wilson  # noqa: E402
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402

FAULTS = [("beacon_fault_loss_rate", "beacon loss rate"), ("beacon_fault_delay_ticks", "beacon delay (control periods)"),
          ("beacon_fault_clock_offset_ms", "clock offset (ms)"), ("beacon_fault_pos_noise_m", "position noise (m)")]
SHORT = {"beacon_fault_loss_rate": "beacon loss", "beacon_fault_delay_ticks": "beacon delay",
         "beacon_fault_clock_offset_ms": "clock offset", "beacon_fault_pos_noise_m": "position noise"}
ARM_SHORT = {"naive": "no contract", "etsi": "ETSI self-announcement", "contract": "contract", "contract+valid": "contract + validity checks"}
ARMS = ["naive", "etsi", "contract", "contract+valid"]


def arm_of(mgr, anch, kv):
    if "etsi" in mgr:
        return "etsi"
    if not anch:
        return "naive"
    return "contract+valid" if kv.get("beacon_validity_checks") else "contract"


def style(key):
    """(label, colour, marker, linestyle, marker face): the validity-check arm is the contract dashed and hollow."""
    label, color, marker = figstyle.ARMS[key]
    return (label, color, marker, "--", "white") if key == "contract+valid" else (label, color, marker, "-", color)


def draw_curves(ax, rows, fk):
    for key in ARMS:
        label, color, marker, ls, mfc = style(key)
        pts = sorted((float(r["value"]), float(r["p"]), float(r["lo"]), float(r["hi"])) for r in rows if r["fault"] == fk and r["arm"] == key)
        if not pts:
            continue
        ax.errorbar(range(len(pts)), [p for _, p, _, _ in pts], yerr=[[p - l for _, p, l, _ in pts], [h - p for _, p, _, h in pts]],
                    fmt=marker, ls=ls, color=color, mfc=mfc, label=label, **figstyle.EB)
        ax.set_xticks(range(len(pts))); ax.set_xticklabels(["%g" % v for v, _, _, _ in pts])
    ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)


def per_geometry(a):
    cells = collections.defaultdict(list)
    for root in a.roots:
        pat = os.path.join(root, a.geom, a.stamp, "speed_spd_*/baseline_default/mgr_*/anchoring_*/lat_%d_*beacon_fault*/run_*/simulation_metrics.json" % a.lat)
        for f in sorted(glob.glob(pat)):
            if "/evaluation_output/" in f:
                continue
            d = json.load(open(f)); d["_run_dir"] = os.path.dirname(f)
            m = compute_run_metrics(d)
            if m is None:
                continue
            kv = d.get("config_edge_kv") or {}
            fault = [k for k in kv if k.startswith("beacon_fault_")]
            if len(fault) != 1:
                continue
            arm = arm_of(d["config_manager_type"], bool(d["config_anchoring"]), kv)
            cells[(fault[0], float(kv[fault[0]]), arm)].append(m)
    rows = []
    for fk, _ in FAULTS:
        for ak in ARMS:
            for v, ms in sorted((v, ms) for (f, v, arm), ms in cells.items() if f == fk and arm == ak):
                n = len(ms); k = sum(1 for m in ms if m["s_op"]); p = k / n; _w = wilson(k, n); l, h = _w[-2], _w[-1]
                rows.append(dict(fault=fk, value=v, arm=ak, n=n, k=k, p=round(p, 3), lo=round(l, 3), hi=round(h, 3),
                                 collisions=sum(1 for m in ms if m["focal_collisions"]),
                                 ghost_episodes=sum(m["focal_ghost_episodes"] for m in ms),
                                 fp_episodes=sum(m["focal_fp_episodes"] for m in ms)))
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    figstyle.rc()
    fig, axes = plt.subplots(1, 4, figsize=(figstyle.TXT, 1.7), sharey=True)
    srows = [{k: str(v) for k, v in r.items()} for r in rows]
    for ax, (fk, flabel) in zip(axes, FAULTS):
        draw_curves(ax, srows, fk); ax.set_xlabel(flabel)
    axes[0].set_ylabel("$\\Pr[S_{op}]$")
    h, l = axes[0].get_legend_handles_labels(); fig.legend(h, l, loc="outside upper center", ncol=4)
    figstyle.save(fig, a.out)
    print("wrote", a.out, len(rows), "cells")


def composite(a):
    sets = [list(csv.DictReader(open(p))) for p in a.csvs]
    figstyle.rc()
    if a.grid:
        fig, axes = plt.subplots(len(sets), 4, figsize=(figstyle.TXT, 1.3 * len(sets) + 0.5), sharey=True)
        for row, (rows, label) in enumerate(zip(sets, a.labels)):
            for col, (fk, flabel) in enumerate(FAULTS):
                ax = axes[row][col]; draw_curves(ax, rows, fk)
                if row == len(sets) - 1:
                    ax.set_xlabel(flabel)
                if col == 0:
                    ax.set_ylabel("(%s) %s\n$\\Pr[S_{op}]$" % ("ab"[row], label))
        h, l = axes[0][0].get_legend_handles_labels(); fig.legend(h, l, loc="outside upper center", ncol=4)
        figstyle.save(fig, a.grid)
    if a.summary:
        fig, ax = plt.subplots(figsize=(figstyle.COL, 1.7))
        width = 0.84 / len(ARMS); out = []
        for i, key in enumerate(ARMS):
            label, color, marker, ls, mfc = style(key)
            xs, ps, lo, hi = [], [], [], []
            for j, (fk, _) in enumerate(FAULTS):
                k = sum(int(r["k"]) for rows in sets for r in rows if r["fault"] == fk and r["arm"] == key)
                n = sum(int(r["n"]) for rows in sets for r in rows if r["fault"] == fk and r["arm"] == key)
                if not n:
                    continue
                p = k / n; w = wilson(k, n)
                xs.append(j + (i - (len(ARMS) - 1) / 2) * width); ps.append(p); lo.append(p - w[-2]); hi.append(w[-1] - p)
                out.append(dict(fault=fk, arm=key, n=n, k=k, p=round(p, 3), lo=round(w[-2], 3), hi=round(w[-1], 3)))
                print(f"{SHORT[fk]:15s} {key:15s} {k:3d}/{n}")
            ax.bar(xs, ps, width, color=color, edgecolor="k", linewidth=0.3, hatch="////" if key == "contract+valid" else "",
                   yerr=[lo, hi], error_kw=dict(elinewidth=0.8, capsize=2, capthick=0.8), label=ARM_SHORT[key])
        ax.set_xticks(range(len(FAULTS))); ax.set_xticklabels([SHORT[fk] for fk, _ in FAULTS])
        ax.set_ylim(0, 1.05); ax.set_ylabel("$\\Pr[S_{op}]$ over the sweep"); ax.grid(axis="y", alpha=0.3)
        h, l = ax.get_legend_handles_labels(); fig.legend(h, l, loc="outside upper center", ncol=2)
        figstyle.save(fig, a.summary)
        with open(os.path.splitext(a.summary)[0] + ".csv", "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(out[0].keys())); w.writeheader(); w.writerows(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp"); ap.add_argument("--roots", nargs="+", default=["experiment_results"])
    ap.add_argument("--geom"); ap.add_argument("--lat", type=int); ap.add_argument("--out")
    ap.add_argument("--csvs", nargs="+"); ap.add_argument("--labels", nargs="+", default=["left turn, 400 ms", "straight crossing, 200 ms"])
    ap.add_argument("--grid"); ap.add_argument("--summary")
    a = ap.parse_args()
    if a.csvs:
        composite(a)
    else:
        if not (a.stamp and a.geom and a.lat is not None and a.out):
            ap.error("per-geometry mode needs --stamp --geom --lat --out")
        global compute_run_metrics
        from recompute_metrics import compute_run_metrics
        per_geometry(a)


if __name__ == "__main__":
    main()
