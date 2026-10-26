#!/usr/bin/env python3
"""Object sharing ablation figure (sba_ablation): (a) merge-point arms against the conflict actor's
measured p99 age at use on one geometry and speed (naive, ETSI self-announcement, Mahalanobis gate
with and without the contract, VIPS with and without the contract, contract), Wilson bars, pooled
over the hosts' envelope_measured.csv files; (b) the fault study panels from fig_faults.py CSVs, one
row per geometry (beacon loss, beacon delay, clock offset, position noise).
    python scripts/sensys_analysis/fig_ablation.py --csv <envelope csvs...> --faults <faults_lt.csv> <faults_scp.csv> \
        --geom openscenario_3_edge_late_fusion --speed 70 --out figures/sba_ablation.pdf
"""
import argparse, collections, csv, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from campaign_check import wilson  # noqa: E402
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import gridspec

ARMS = [("naive", "no contract", "#d62728", "^"), ("etsi", "ETSI self-announcement", "#9467bd", "v"),
        ("gate", "Mahalanobis gate", "#8c564b", "x"), ("gate+c", "Mahalanobis gate, +contract", "#8c564b", "P"),
        ("vips", "VIPS rectification", "#ff7f0e", "d"), ("vips+c", "VIPS rectification, +contract", "#ff7f0e", "D"),
        ("contract", "contract (SBA)", "#1f77b4", "s"), ("oracle", "exact-state reference", "k", "o")]
FAULTS = [("beacon_fault_loss_rate", "beacon loss rate"), ("beacon_fault_delay_ticks", "beacon delay (periods)"),
          ("beacon_fault_clock_offset_ms", "clock offset (ms)"), ("beacon_fault_pos_noise_m", "position noise (m)")]
FARMS = [("naive", "no contract", "#d62728"), ("etsi", "ETSI", "#9467bd"), ("contract", "contract", "#1f77b4"), ("contract+valid", "contract + checks", "#17becf")]


def arm_key(mgr, anch, kv):
    if mgr == "oracle":
        return "oracle"
    if "etsi" in mgr:
        return "etsi"
    gate = "assoc_metric" in kv
    if mgr == "vips":
        return "vips+c" if anch == "on" else "vips"
    if mgr == "late_fusion" and gate:
        return "gate+c" if anch == "on" else "gate"
    if mgr == "late_fusion":
        return "contract" if anch == "on" else "naive"
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", nargs="+", required=True); ap.add_argument("--faults", nargs=2, required=True)
    ap.add_argument("--geom", default="openscenario_3_edge_late_fusion"); ap.add_argument("--speed", type=int, default=70)
    ap.add_argument("--max-age", type=float, default=1100); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    acc = collections.defaultdict(list)   # (arm, lat) -> [(axis_ms, s_op)]
    for p in a.csv:
        for r in csv.DictReader(open(p)):
            if r["geom"] != a.geom or int(r["speed"]) != a.speed or int(r["N"]) != 1:
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("beacon_fault", "beacon_validity")) or k == "behavior.hazard_response" for k in kv):
                continue
            k = arm_key(r["mgr"], r["anch"], kv)
            if k is None:
                continue
            acc[(k, int(r["lat"]))].append((float(r["axis_ms"]), int(r["s_op"])))
    fig = plt.figure(figsize=(10.5, 6.2))
    gs = gridspec.GridSpec(3, 4, height_ratios=[1.35, 1, 1], figure=fig)
    ax = fig.add_subplot(gs[0, :])
    for key, label, color, marker in ARMS:
        pts = []
        for (kk, lat), v in sorted(acc.items(), key=lambda t: t[0][1]):
            if kk != key:
                continue
            x = sum(x for x, _ in v) / len(v)
            if x > a.max_age:
                continue
            pts.append((x, sum(o for _, o in v), len(v)))
        if not pts:
            continue
        xs = [x for x, _, _ in pts]; ps = [k / n for _, k, n in pts]
        lo = [max(0.0, p - wilson(k, n)[1]) for p, (_, k, n) in zip(ps, pts)]; hi = [max(0.0, wilson(k, n)[2] - p) for p, (_, k, n) in zip(ps, pts)]
        ax.errorbar(xs, ps, yerr=[lo, hi], fmt=marker + "-", color=color, ms=4, lw=1, capsize=1.5, label=label, alpha=0.9)
    ax.axhline(0.8, color="gray", lw=0.7, ls="--"); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
    ax.set_xlabel("conflict actor's measured p99 age at use (ms)", fontsize=8); ax.set_ylabel("$\\Pr[S_{op}]$")
    ax.set_title("(a) merge-point arms, %s at %d km/h, N=1" % ("left turn" if a.geom.startswith("open") else "straight crossing", a.speed), fontsize=9)
    ax.legend(fontsize=6.5, ncol=4, loc="lower left")
    for row, (fpath, gname) in enumerate(zip(a.faults, ("left turn, 400 ms", "straight crossing, 200 ms")), start=1):
        rows = list(csv.DictReader(open(fpath)))
        for col, (fk, flabel) in enumerate(FAULTS):
            axf = fig.add_subplot(gs[row, col])
            for ak, aname, color in FARMS:
                pts = sorted((float(r["value"]), float(r["p"]), float(r["lo"]), float(r["hi"])) for r in rows if r["fault"] == fk and r["arm"] == ak)
                if not pts:
                    continue
                axf.errorbar(range(len(pts)), [p for _, p, _, _ in pts], yerr=[[p - l for _, p, l, _ in pts], [h - p for _, p, _, h in pts]],
                             fmt="o-", color=color, ms=3.5, lw=1, capsize=1.5, label=aname)
                axf.set_xticks(range(len(pts))); axf.set_xticklabels(["%g" % v for v, _, _, _ in pts], fontsize=7)
            axf.set_ylim(-0.05, 1.05); axf.grid(alpha=0.3); axf.set_xlabel(flabel, fontsize=7.5)
            if col == 0:
                axf.set_ylabel("(b) %s\n$\\Pr[S_{op}]$" % gname, fontsize=8)
            else:
                axf.set_yticklabels([])
            if row == 1 and col == 3:
                axf.legend(fontsize=6, loc="lower left")
    fig.tight_layout(); os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    fig.savefig(a.out); fig.savefig(os.path.splitext(a.out)[0] + ".png", dpi=130)
    print("wrote", a.out, "arms:", sorted({k for (k, _) in acc}))


if __name__ == "__main__":
    main()
