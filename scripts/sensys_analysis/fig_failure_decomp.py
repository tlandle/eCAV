#!/usr/bin/env python3
"""Failure decomposition figure: per arm and configured delay, the fraction of runs that fail by
each class (collision; self-ghost brake; other false brake; late completion, S_prog) as stacked bars,
for the N=4 junction cells of a stamp and for the blind overtake stamps (N=1). Classes are colours,
arms are hatches (reference plain, no contract diagonal, ETSI crossed, contract dotted). Drawn at
text width (figstyle.TXT) for a figure* float.
    python scripts/sensys_analysis/fig_failure_decomp.py --stamp sensys_nov27_j3 --roots experiment_results \
        --overtake-stamps sensys_nov27_ov4 sensys_nov27_ov4_onc20 sensys_nov27_ov4_onc28 --out figures/arch_failure_decomp.pdf
Each failed run is assigned one class, in the order self-ghost brake (logic), collision (physics), other
false brake, late completion, so the bars of one arm sum to its failure fraction. Writes PDF, PNG, CSV.
"""
import argparse, collections, csv, glob, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), "scripts"))
from recompute_metrics import compute_run_metrics  # noqa: E402
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

PANELS = [("openscenario_3_edge_late_fusion", "(a) Left turn, $N{=}4$", "*_ego_4_*"), ("scp_edge_late_fusion", "(b) Straight crossing, $N{=}4$", "*_ego_4_*"),
          ("blind_overtake_edge_late_fusion", "(c) Blind overtake, $N{=}1$", "*")]
ARMS = [("oracle", "reference", ""), ("naive", "no contract", "//////"), ("etsi", "ETSI", "xxxxxx"), ("contract", "contract", "......")]
CLASSES = [("collision", "collision (physics)", "#b22222"), ("self_ghost", "self-ghost brake (logic)", "#ff8c00"),
           ("false_brake", "other false brake", "#daa520"), ("late", "late completion", "#4682b4")]


def arm_of(mgr, anch):
    if mgr == "oracle":
        return "oracle"
    if "etsi" in mgr:
        return "etsi"
    return "contract" if anch else "naive"


def collect(roots, stamp, geom, pattern):
    cells = collections.defaultdict(list)
    for root in roots:
        pat = os.path.join(root, geom, stamp, "speed_spd_*/baseline_default/mgr_*/anchoring_*/" + pattern + "/run_*/simulation_metrics.json")
        for f in sorted(glob.glob(pat)):
            if "/evaluation_output/" in f or "beacon_fault" in f or "v2x_regime" in f or "assoc_" in f or "ttc_graded" in f:
                continue
            d = json.load(open(f)); d["_run_dir"] = os.path.dirname(f)
            m = compute_run_metrics(d)
            if m is None:
                continue
            arm = arm_of(d["config_manager_type"], bool(d["config_anchoring"]))
            lat = int(round(float(d["config_latency_ms"])))
            flags = dict(self_ghost=not m["s_ghost"], collision=bool(m["focal_collisions"]),
                         false_brake=not m["s_fp"], late=not m["s_prog"])
            cls = next((k for k in ("self_ghost", "collision", "false_brake", "late") if flags[k]), None)
            cells[(arm, lat)].append({k: (k == cls) for k in flags})
    return cells


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", required=True); ap.add_argument("--roots", nargs="+", default=["experiment_results"])
    ap.add_argument("--overtake-stamps", nargs="*", default=[]); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    figstyle.rc()
    fig, axes = plt.subplots(1, 3, figsize=(figstyle.TXT, 1.8), sharey=True, gridspec_kw=dict(width_ratios=[3, 3, 8]))
    rows = []
    hatch_of = {ak: h for ak, _, h in ARMS}
    for ax, (geom, title, pattern) in zip(axes, PANELS):
        if geom.startswith("blind"):
            cells = collections.defaultdict(list)
            for st in a.overtake_stamps:
                for k, v in collect(a.roots, st, geom, pattern).items():
                    cells[k].extend(v)
        else:
            cells = collect(a.roots, a.stamp, geom, pattern)
        lats = sorted({lat for (_, lat) in cells})
        arms = [ak for ak, _, _ in ARMS if any((ak, lat) in cells for lat in lats)]
        width = 0.84 / max(1, len(arms))
        for i, ak in enumerate(arms):
            bottoms = [0.0] * len(lats)
            for ck, clabel, color in CLASSES:
                fr = []
                for lat in lats:
                    ms = cells.get((ak, lat), [])
                    fr.append(sum(1 for m in ms if m[ck]) / len(ms) if ms else 0.0)
                    if ms and ck == "collision":
                        rows.append(dict(panel=title, arm=ak, lat=lat, n=len(ms),
                                         **{c: sum(1 for m in ms if m[c]) for c, _, _ in CLASSES}))
                ax.bar([x + (i - (len(arms) - 1) / 2) * width for x in range(len(lats))], fr, width, bottom=bottoms,
                       color=color, edgecolor="k", linewidth=0.3, hatch=hatch_of[ak])
                bottoms = [b + f for b, f in zip(bottoms, fr)]
        ax.set_xticks(range(len(lats))); ax.set_xticklabels([str(l) for l in lats], rotation=45 if len(lats) > 6 else 0)
        ax.set_xlabel("configured delay (ms)"); ax.set_title(title)
        ax.set_ylim(0, 1.05); ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("fraction of runs failed, by class")
    cls_h = [Patch(facecolor=c, edgecolor="k", linewidth=0.3, label=l) for _, l, c in CLASSES]
    arm_h = [Patch(facecolor="white", edgecolor="k", linewidth=0.3, hatch=h, label=f"{l} (bars)") for _, l, h in ARMS]
    fig.legend(handles=[x for pair in zip(cls_h, arm_h) for x in pair], loc="outside upper center", ncol=4)
    figstyle.save(fig, a.out)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote", a.out, len(rows), "cells")


if __name__ == "__main__":
    main()
