#!/usr/bin/env python3
"""Failure decomposition figures: per arm, the fraction of runs that fail by each class (collision;
self-ghost brake; other false brake; late completion, S_prog) as stacked bars, for the N=4 junction
cells of a stamp and for the blind overtake stamps (N=1). --out (column width): one grouped plot, a
group per scenario at one configured delay, the largest at which every arm has runs and the reference
fails in no run, with the no-contract, ETSI and contract bars in each group (hatched as below), the
failed-run count over each bar, and only the classes that occur at those delays in the legend. The
reference has no bar there (it fails in no run by construction of the chosen delay). --sweep (text
width): every configured delay and every arm, arms as hatches (reference plain, no contract diagonal,
ETSI crossed, contract dotted).
    python scripts/sensys_analysis/fig_failure_decomp.py --stamp sensys_nov27_j3 --roots experiment_results \
        --overtake-stamps sensys_nov27_ov4 sensys_nov27_ov4_onc20 sensys_nov27_ov4_onc28 \
        --out figures/arch_failure_decomp.pdf --sweep figures/arch_failure_sweep.pdf
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
SHORT = {"openscenario_3_edge_late_fusion": "(a) Left turn", "scp_edge_late_fusion": "(b) Crossing", "blind_overtake_edge_late_fusion": "(c) Overtake"}
GROUP = {"openscenario_3_edge_late_fusion": "Left turn", "scp_edge_late_fusion": "Straight crossing", "blind_overtake_edge_late_fusion": "Blind overtake"}
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
    ap.add_argument("--sweep", help="also write the full delay sweep (figure* width) to this path")
    a = ap.parse_args()
    figstyle.rc()
    hatch_of = {ak: h for ak, _, h in ARMS}; name_of = {ak: n for ak, n, _ in ARMS}
    panels = []
    for geom, title, pattern in PANELS:
        if geom.startswith("blind"):
            cells = collections.defaultdict(list)
            for st in a.overtake_stamps:
                for k, v in collect(a.roots, st, geom, pattern).items():
                    cells[k].extend(v)
        else:
            cells = collect(a.roots, a.stamp, geom, pattern)
        panels.append((geom, title, cells))

    def fractions(ms):
        return {ck: (sum(1 for m in ms if m[ck]) / len(ms) if ms else 0.0) for ck, _, _ in CLASSES}

    rows = []
    for geom, title, cells in panels:
        for (ak, lat), ms in sorted(cells.items(), key=lambda t: (t[0][1], t[0][0])):
            rows.append(dict(panel=title, arm=ak, lat=lat, n=len(ms), **{c: sum(1 for m in ms if m[c]) for c, _, _ in CLASSES}))

    # main figure: one grouped plot, a group per scenario at one configured delay, the largest at which
    # every arm has runs and the reference fails in no run, so the logic failures show without the physics
    # ones. Bars are the no-contract, ETSI and contract arms (the reference has no failure there).
    fig, ax = plt.subplots(figsize=(figstyle.COL, 1.9))
    chosen = {}
    shown = [ak for ak, _, _ in ARMS if ak != "oracle"]
    width = 0.26
    present = set()
    for gi, (geom, title, cells) in enumerate(panels):
        lats = sorted({lat for (_, lat) in cells})
        arms = [ak for ak, _, _ in ARMS if any((ak, lat) in cells for lat in lats)]
        ok = [lat for lat in lats if all((ak, lat) in cells for ak in arms)
              and not any(any(m.values()) for m in cells.get(("oracle", lat), []))]
        lat = max(ok) if ok else max(lat for lat in lats if all((ak, lat) in cells for ak in arms))
        chosen[geom] = lat
        for i, ak in enumerate(shown):
            ms = cells.get((ak, lat), [])
            x = gi + (i - (len(shown) - 1) / 2) * (width + 0.02)
            bottom = 0.0
            for ck, clabel, color in CLASSES:
                fr = fractions(ms)[ck]
                if fr > 0:
                    present.add(ck)
                    ax.bar(x, fr, width, bottom=bottom, color=color, edgecolor="k", linewidth=0.3, hatch=hatch_of[ak])
                    bottom += fr
            k = sum(1 for m in ms if any(m.values()))
            ax.text(x, bottom + 0.03, f"{k}/{len(ms)}", ha="center", va="bottom", fontsize=7)
    ax.set_xticks(range(len(panels)))
    ax.set_xticklabels([f"{GROUP[geom]}\n{chosen[geom]} ms" for geom, _, _ in panels])
    ax.set_xlim(-0.5, len(panels) - 0.5); ax.set_ylim(0, 1.18); ax.grid(axis="y", alpha=0.3)
    ax.set_ylabel("fraction of runs failed")
    cls_h = [Patch(facecolor=c, edgecolor="k", linewidth=0.3, label=l) for ck, l, c in CLASSES if ck in present]
    arm_h = [Patch(facecolor="white", edgecolor="k", linewidth=0.3, hatch=hatch_of[ak], label=f"{name_of[ak]} (bars)") for ak in shown]
    fig.legend(handles=cls_h + arm_h, loc="outside upper center", ncol=2)
    figstyle.save(fig, a.out)
    print("representative delays:", chosen, "classes present:", sorted(present))

    # supplementary: the full delay sweep, arms by hatch
    if a.sweep:
        fig, axes = plt.subplots(1, 3, figsize=(figstyle.TXT, 1.8), sharey=True, gridspec_kw=dict(width_ratios=[3, 3, 8]))
        for ax, (geom, title, cells) in zip(axes, panels):
            lats = sorted({lat for (_, lat) in cells})
            arms = [ak for ak, _, _ in ARMS if any((ak, lat) in cells for lat in lats)]
            width = 0.84 / max(1, len(arms))
            for i, ak in enumerate(arms):
                bottoms = [0.0] * len(lats)
                for ck, clabel, color in CLASSES:
                    fr = [fractions(cells.get((ak, lat), []))[ck] for lat in lats]
                    ax.bar([x + (i - (len(arms) - 1) / 2) * width for x in range(len(lats))], fr, width, bottom=bottoms,
                           color=color, edgecolor="k", linewidth=0.3, hatch=hatch_of[ak])
                    bottoms = [b + f for b, f in zip(bottoms, fr)]
            ax.set_xticks(range(len(lats))); ax.set_xticklabels([str(l) for l in lats], rotation=45 if len(lats) > 6 else 0)
            ax.set_xlabel("configured delay (ms)"); ax.set_title(title)
            ax.set_ylim(0, 1.05); ax.grid(axis="y", alpha=0.3)
            if not any(fractions(cells.get((ak, lat), []))[ck] for ak in arms for lat in lats for ck, _, _ in CLASSES):
                ax.text(0.5, 0.5, "no run failed", ha="center", va="center", transform=ax.transAxes, fontsize=7, color="0.4")
        axes[0].set_ylabel("fraction of runs failed, by class")
        cls_h = [Patch(facecolor=c, edgecolor="k", linewidth=0.3, label=l) for _, l, c in CLASSES]
        arm_h = [Patch(facecolor="white", edgecolor="k", linewidth=0.3, hatch=h, label=f"{l} (bars)") for _, l, h in ARMS]
        fig.legend(handles=[x for pair in zip(cls_h, arm_h) for x in pair], loc="outside upper center", ncol=4)
        figstyle.save(fig, a.sweep)
    with open(os.path.splitext(a.out)[0] + ".csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("wrote", a.out, len(rows), "cells")


if __name__ == "__main__":
    main()
