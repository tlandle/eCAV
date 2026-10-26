#!/usr/bin/env python3
"""Fault study figure: P(S_op) against fault severity, one panel per fault type (beacon loss, beacon
delay, clock offset, position noise), one curve per arm (naive, ETSI, contract, contract with the
validity checks on), Wilson 95% bars. Cells are the N=4 beacon_fault cells of a stamp.
    python scripts/sensys_analysis/fig_faults.py --stamp sensys_nov27_j3 --root experiment_results \
        --geom openscenario_3_edge_late_fusion --lat 400 --out figures/faults_lt.pdf
Writes the PDF, a PNG and a CSV (fault, value, arm, n, k, p, lo, hi, collisions, ghost_episodes, fp_episodes).
"""
import argparse, collections, csv, glob, json, os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts"))
from campaign_check import wilson  # noqa: E402
from recompute_metrics import compute_run_metrics  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FAULTS = [("beacon_fault_loss_rate", "Beacon loss rate"), ("beacon_fault_delay_ticks", "Beacon delay (control periods)"),
          ("beacon_fault_clock_offset_ms", "Clock offset (ms)"), ("beacon_fault_pos_noise_m", "Position noise (m)")]
ARMS = [("naive", "no contract"), ("etsi", "ETSI self-announcement"), ("contract", "contract"), ("contract+valid", "contract + validity checks")]


def arm_of(mgr, anch, kv):
    if "etsi" in mgr:
        return "etsi"
    if not anch:
        return "naive"
    return "contract+valid" if kv.get("beacon_validity_checks") else "contract"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", required=True); ap.add_argument("--root", default="experiment_results")
    ap.add_argument("--geom", required=True); ap.add_argument("--lat", type=int, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    cells = collections.defaultdict(list)
    pat = os.path.join(a.root, a.geom, a.stamp, "speed_spd_*/baseline_default/mgr_*/anchoring_*/lat_%d_*beacon_fault*/run_*/simulation_metrics.json" % a.lat)
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
    fig, axes = plt.subplots(1, 4, figsize=(10.5, 2.5), sharey=True)
    for ax, (fk, flabel) in zip(axes, FAULTS):
        for ak, aname in ARMS:
            pts = sorted((v, ms) for (f, v, arm), ms in cells.items() if f == fk and arm == ak)
            if not pts:
                continue
            xs, ys, lo, hi = [], [], [], []
            for v, ms in pts:
                n = len(ms); k = sum(1 for m in ms if m["s_op"]); p = k / n; _w = wilson(k, n); l, h = _w[-2], _w[-1]
                xs.append(v); ys.append(p); lo.append(p - l); hi.append(h - p)
                rows.append(dict(fault=fk, value=v, arm=ak, n=n, k=k, p=round(p, 3), lo=round(l, 3), hi=round(h, 3),
                                 collisions=sum(1 for m in ms if m["focal_collisions"]),
                                 ghost_episodes=sum(m["focal_ghost_episodes"] for m in ms),
                                 fp_episodes=sum(m["focal_fp_episodes"] for m in ms)))
            ax.errorbar(range(len(xs)), ys, yerr=[lo, hi], fmt="o-", ms=4, capsize=2, label=aname)
            ax.set_xticks(range(len(xs))); ax.set_xticklabels([("%g" % v) for v in xs], fontsize=8)
        ax.set_xlabel(flabel, fontsize=8); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
    axes[0].set_ylabel("$\\Pr[S_{op}]$"); axes[-1].legend(fontsize=6.5, loc="lower left")
    fig.tight_layout(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out); fig.savefig(os.path.splitext(a.out)[0] + ".png", dpi=150)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote", a.out, len(rows), "cells")


if __name__ == "__main__":
    main()
