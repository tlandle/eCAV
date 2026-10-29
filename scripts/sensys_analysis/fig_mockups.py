#!/usr/bin/env python3
"""Draft mock-ups of the figures whose measured cells have not landed (2026-10-28, Tyler: draw the expected
pattern from the v14 review so the text can be written; the measured build replaces every one of them).
Cells that have landed keep their measured
counts (envelope runs.csv files); cells that have not are filled with the review's expected pattern.
    python scripts/sensys_analysis/fig_mockups.py --runs <measured runs.csv ...> --landing <landing_ci.md> --out <dir>
Writes: arch_safety_scale.pdf (Fig. 6, via fig_envelope.py on a synthetic runs.csv), situation_sensitivity.pdf
(Fig. 7, two panels), situation_contract.pdf (Fig. 15), safety_margin.pdf (Fig. 9, via fig_safety_margin.py on a
mock budget CSV), arch_failure_decomp.pdf (Fig. 10), situation_control.pdf (Fig. 16).
"""
import argparse, collections, csv, json, os, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402
from campaign_check import wilson  # noqa: E402

PY = sys.executable
LT, SCP, OV = "openscenario_3_edge_late_fusion", "scp_edge_late_fusion", "blind_overtake_edge_late_fusion"
WM = ""  # no mark in the figures (Tyler, Oct 28); provenance lives in the change log and the commit
KV = '{"behavior.curv_lat_accel": 8, "behavior.self_exclusion_m": 1.0, "prediction_steps": 60}'
CELL = "mock_prediction_steps-60_curv_lat_accel-8_self_exclusion_m-1.0"

# ---- Fig. 6: expected success per configured delay where no cell has landed (k of 10) ----
EXPECT6 = {  # (geom, arm): {lat: (k, failure class)}
    (LT, "reference"): {1500: (9, "collision"), 2000: (5, "collision"), 3000: (1, "collision")},
    (LT, "contract"): {700: (10, ""), 1000: (10, ""), 1500: (8, "collision"), 2000: (4, "collision"), 3000: (0, "collision")},
    (LT, "naive"): {700: (7, "self_ghost"), 1000: (3, "self_ghost"), 1500: (1, "self_ghost"), 2000: (0, "self_ghost"), 3000: (0, "self_ghost")},
    (LT, "i2v"): {1500: (8, "collision"), 2000: (3, "collision"), 3000: (0, "collision")},
    (SCP, "reference"): {1500: (8, "collision"), 2000: (3, "collision"), 3000: (0, "collision")},
    (SCP, "contract"): {700: (10, ""), 1000: (9, "collision"), 1500: (7, "collision"), 2000: (2, "collision"), 3000: (0, "collision")},
    (SCP, "naive"): {700: (6, "self_ghost"), 1000: (2, "self_ghost"), 1500: (0, "self_ghost"), 2000: (0, "self_ghost"), 3000: (0, "self_ghost")},
    (SCP, "i2v"): {1500: (7, "collision"), 2000: (2, "collision"), 3000: (0, "collision")},
}
MGR = {"reference": ("oracle", "on"), "contract": ("late_fusion", "on"), "naive": ("late_fusion", "off"), "i2v": ("infra_only", "off")}
OFFSET = {LT: 176, SCP: 100}  # measured age at use above the configured delay for the reference at 70 km/h

# ---- Fig. 7 / 15: budgets (ms); measured where the boundary was observed, expected elsewhere ----
BUDGET = {  # geom: {speed: (reference, contract)}
    LT: {38: (3000, 2950), 50: (3000, 2950), 53: (2500, 2450), 63: (2000, 1950), 70: (1500, 1450), 88: (825, 775)},
    SCP: {38: (3000, 2950), 50: (2125, 2075), 53: (1900, 1850), 63: (1500, 1450), 70: (1200, 1150), 88: (825, 775)},
}
OVERTAKE = {12: (1625, 1625), 20: (1200, 1150), 28: (800, 750)}  # oncoming m/s: (reference, contract)
URGENCY = {7.0: (1500, 1450), 4.0: (900, 850), 3.5: (500, 450), 3.0: (250, 200)}  # launch lead s: (reference, contract) at LT 70
LANDING_N24 = [("Exact-state reference", 164, "k"), ("I2V object sharing", 200, figstyle.ARMS["i2v"][1]), ("V2X2V prediction", 202, figstyle.ARMS["v2x2v"][1]),
               ("Cooperative planning (CIP)", 203, figstyle.ARMS["cip"][1]), ("Intermediate feature fusion", 326, figstyle.ARMS["intermediate"][1])]
MARGIN_SITUATIONS = [("LT 70\n7.0 s lead", 1500), ("SCP 88", 825), ("LT 70\n3.0 s lead", 250)]
MEASURED_BUDGET = {(LT, 88), (SCP, 88), (SCP, 50), (OV, 12)}

# ---- Fig. 10: failures at the representative delay (N=4 junctions at 1000 ms, overtake N=1 at 500 ms) ----
DECOMP = [("(a) Left turn, $N{=}4$, 1000 ms", {"naive": (5, {"self_ghost": 4}), "etsi": (5, {"self_ghost": 4}), "contract": (5, {}), "reference": (3, {})}),
          ("(b) Straight crossing, $N{=}4$, 1000 ms", {"naive": (5, {"self_ghost": 5}), "etsi": (5, {"self_ghost": 4}), "contract": (5, {}), "reference": (3, {})}),
          ("(c) Blind overtake, $N{=}1$, 500 ms", {"naive": (10, {"self_ghost": 2}), "etsi": (10, {"self_ghost": 6}), "contract": (10, {}), "reference": (10, {})})]
CLASSES = [("collision", "collision (physics)", "#b22222"), ("self_ghost", "self-ghost brake (logic)", "#ff8c00"),
           ("false_brake", "false brake", "#808080"), ("late", "late completion", "#4682b4")]

# ---- Fig. 16: the second controller's boundary sits 300 ms below the reference's ----
CTRL_SHIFT = 300


def load_measured(paths):
    cells = collections.defaultdict(lambda: [0, 0])
    for p in paths:
        for r in csv.DictReader(open(p)):
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("beacon_", "assoc_")) or k == "behavior.hazard_response" for k in kv) or "v2x_regime" in r["cell"]: continue
            if int(r["N"]) != 1: continue
            arm = next((a for a, (m, an) in MGR.items() if r["mgr"] == m and r["anch"] == an), None)
            if arm is None: continue
            c = cells[(r["geom"], int(r["speed"]), arm, int(r["lat"]))]; c[0] += int(r["s_op"]); c[1] += 1
    return cells


def fig6(a, out):
    """Synthetic runs.csv: measured cells verbatim (from the hosts' runs.csv), expected cells filled in; then the
    production envelope builder draws it."""
    measured = load_measured(a.runs)
    rows = []
    for (geom, speed, arm, lat), (k, n) in measured.items():
        if speed != 70: continue
        for i in range(n):
            ok = i < k
            rows.append(dict(geom=geom, speed=70, mgr=MGR[arm][0], anch=MGR[arm][1], N=1, kv=KV, cell=CELL, lat=lat, rep=i + 1,
                             axis_ms=lat + OFFSET[geom] + (0 if arm == "reference" else 10 * (i % 3)), s_op=int(ok),
                             collided=0, self_ghost=0, false_brake=0, stalled=0))
    for (geom, arm), cells in EXPECT6.items():
        for lat, (k, cls) in cells.items():
            if (geom, 70, arm, lat) in measured: continue
            for i in range(10):
                ok = i < k
                rows.append(dict(geom=geom, speed=70, mgr=MGR[arm][0], anch=MGR[arm][1], N=1, kv=KV, cell=CELL, lat=lat, rep=i + 1,
                                 axis_ms=lat + OFFSET[geom] + (0 if arm == "reference" else 10 * (i % 3)), s_op=int(ok),
                                 collided=int(not ok and cls == "collision"), self_ghost=int(not ok and cls == "self_ghost"),
                                 false_brake=0, stalled=0))
    os.makedirs(os.path.join(out, "mock_runs"), exist_ok=True)
    p = os.path.join(out, "mock_runs", "runs.csv")
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    env = dict(os.environ, PYTHONPATH=os.environ.get("PYTHONPATH", "."))
    subprocess.run([PY, os.path.join(HERE, "fig_envelope.py"), "--csv", p, "--speed", "70", "--max-age", "3300", "--out", os.path.join(out, "arch_safety_scale.pdf")], check=True, env=env)


def budget_panels(a, out):
    figstyle.rc()
    # Fig. 7: (a) ego speed and maneuver, (b) oncoming speed on the overtake
    fig, (ax, axb, axc) = plt.subplots(1, 3, figsize=(figstyle.TXT, 2.0), gridspec_kw=dict(width_ratios=[3, 2, 2.4]))
    SHORT = {LT: "left turn", SCP: "crossing", OV: "overtake"}
    for geom in (LT, SCP):
        label, color = figstyle.GEOMS[geom]
        sp = sorted(BUDGET[geom]); ref = [BUDGET[geom][s][0] for s in sp]; con = [BUDGET[geom][s][1] for s in sp]
        ax.errorbar(sp, ref, yerr=[[60] * len(sp), [60] * len(sp)], fmt="o-", color=color, label=f"{SHORT[geom]}, reference", **figstyle.EB)
        ax.errorbar(sp, con, yerr=[[60] * len(sp), [60] * len(sp)], fmt="s--", color=color, mfc="white", label=f"{SHORT[geom]}, +contract", **figstyle.EB)
        for s in sp:
            if (geom, s) not in MEASURED_BUDGET and BUDGET[geom][s][0] >= 3000:
                ax.annotate("≥", (s, BUDGET[geom][s][0]), textcoords="offset points", xytext=(-9, -3), fontsize=7, color=color)
    ax.set_xlabel("approach speed (km/h)"); ax.set_ylabel(r"$\tau_{\max}$ at $\Pr[S_{op}] \geq 0.8$ (ms)"); ax.set_title("(a) two junctions, ego speed", fontsize=7)
    ax.set_ylim(0, 3300); ax.grid(alpha=0.3)
    label, color = figstyle.GEOMS[OV]
    onc = sorted(OVERTAKE); ref = [OVERTAKE[o][0] for o in onc]; con = [OVERTAKE[o][1] for o in onc]
    axb.errorbar(onc, ref, yerr=[[60] * 3, [60] * 3], fmt="o-", color=color, label=f"{SHORT[OV]}, reference", **figstyle.EB)
    axb.errorbar(onc, con, yerr=[[60] * 3, [60] * 3], fmt="s--", color=color, mfc="white", label=f"{SHORT[OV]}, +contract", **figstyle.EB)
    axb.set_xlabel("oncoming (m/s)"); axb.set_title("(b) overtake", fontsize=7); axb.set_ylim(0, 3300); axb.set_xticks(onc); axb.grid(alpha=0.3)
    # (c) urgency: the crossing car launches when the ego's time to arrival drops below the lead (left turn, 70 km/h)
    lead = sorted(URGENCY); color = figstyle.GEOMS[LT][1]
    axc.errorbar(lead, [URGENCY[t][0] for t in lead], yerr=[[60] * len(lead), [60] * len(lead)], fmt="o-", color=color, **figstyle.EB)
    axc.errorbar(lead, [URGENCY[t][1] for t in lead], yerr=[[60] * len(lead), [60] * len(lead)], fmt="s--", color=color, mfc="white", **figstyle.EB)
    axc.axhspan(150, 330, color="0.85", lw=0, zorder=0)
    axc.text(6.9, 240, "architecture\nages, N=24", fontsize=6, color="0.35", ha="right", va="center")
    axc.set_xlabel("time to conflict at launch (s)"); axc.set_title("(c) left turn, 70 km/h, urgency", fontsize=7); axc.set_ylim(0, 3300); axc.set_xticks(lead); axc.grid(alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels(); h2, l2 = axb.get_legend_handles_labels()
    fig.legend(h1 + h2, l1 + l2, loc="outside upper center", ncol=6, handlelength=1.2, columnspacing=0.6, fontsize=6.5)
    figstyle.save(fig, os.path.join(out, "situation_sensitivity.pdf"))
    # Fig. 15: reference vs contract budget per situation, with the difference
    fig, ax = plt.subplots(figsize=(figstyle.COL, 1.8))
    for geom in (LT, SCP):
        label, color = figstyle.GEOMS[geom]
        sp = sorted(BUDGET[geom])
        ax.plot(sp, [BUDGET[geom][s][0] for s in sp], "o-", color=color, label=f"{label}, reference")
        ax.plot(sp, [BUDGET[geom][s][1] for s in sp], "s--", color=color, mfc="white", label=f"{label}, +contract")
    ax.set_xlabel("approach speed (km/h)"); ax.set_ylabel(r"$\tau_{\max}$ (ms)"); ax.set_ylim(0, 3300); ax.grid(alpha=0.3)
    fig.legend(*ax.get_legend_handles_labels(), loc="outside upper center", ncol=2, handlelength=1.5, columnspacing=0.8)
    figstyle.save(fig, os.path.join(out, "situation_contract.pdf"))


def margin(a, out):
    p = os.path.join(out, "mock_tau.csv")
    with open(p, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["geom", "speed_kmh", "arm", "kv", "runs", "bins", "max_bin_ms", "tau_ms", "censored", "boot_lo_ms", "boot_hi_ms", "collided", "self_ghost", "false_brake", "stalled"])
        w.writeheader()
        for geom in (LT, SCP):
            for s, (ref, con) in BUDGET[geom].items():
                for arm, t in (("reference", ref), ("contract", con)):
                    w.writerow(dict(geom=geom, speed_kmh=s, arm="oracle" if arm == "reference" else arm, kv=KV, runs=100, bins=10, max_bin_ms=3125, tau_ms=t, censored=0, boot_lo_ms=t - 75, boot_hi_ms=t + 75, collided=0, self_ghost=0, false_brake=0, stalled=0))
        for o, (ref, con) in OVERTAKE.items():
            w.writerow(dict(geom=OV, speed_kmh=43, arm="oracle", kv=KV, runs=100, bins=10, max_bin_ms=3125, tau_ms=ref, censored=0, boot_lo_ms=ref - 75, boot_hi_ms=ref + 75, collided=0, self_ghost=0, false_brake=0, stalled=0))
    # compact dot-and-whisker: N=16 and N=24 side by side, three situations, five architectures (Fig. 8 colours),
    # margin = budget minus landing p99, interval from the budget's bootstrap (+-75) and the landing's locale CI
    LANDING = {16: [("Exact-state reference", 157, 142, 163, "k", "o"), ("I2V object sharing", 183, 171, 192, figstyle.ARMS["i2v"][1], "D"),
                    ("V2X2V prediction", 185, 172, 193, figstyle.ARMS["v2x2v"][1], "s"), ("Cooperative planning (CIP)", 186, 173, 196, figstyle.ARMS["cip"][1], "X"),
                    ("Intermediate feature fusion", 266, 253, 275, figstyle.ARMS["intermediate"][1], "v")],
               24: [("Exact-state reference", 164, 138, 170, "k", "o"), ("I2V object sharing", 200, 173, 205, figstyle.ARMS["i2v"][1], "D"),
                    ("V2X2V prediction", 202, 175, 207, figstyle.ARMS["v2x2v"][1], "s"), ("Cooperative planning (CIP)", 203, 176, 208, figstyle.ARMS["cip"][1], "X"),
                    ("Intermediate feature fusion", 326, 300, 333, figstyle.ARMS["intermediate"][1], "v")]}
    figstyle.rc()
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.COL, 2.0), sharey=True)
    for ax, N in zip(axes, (16, 24)):
        archs = LANDING[N]; n_arch = len(archs); width = 0.8 / n_arch
        for j, (label, age, lo, hi, color, marker) in enumerate(archs):
            xs = [i + (j - (n_arch - 1) / 2) * width for i in range(len(MARGIN_SITUATIONS))]
            ms = [tau - age for _, tau in MARGIN_SITUATIONS]
            err_lo = [m - ((tau - 75) - hi) for m, (_, tau) in zip(ms, MARGIN_SITUATIONS)]
            err_hi = [((tau + 75) - lo) - m for m, (_, tau) in zip(ms, MARGIN_SITUATIONS)]
            ax.errorbar(xs, ms, yerr=[err_lo, err_hi], fmt=marker, color=color, ms=4, label=label, **figstyle.EB)
        ax.axhline(0, color="k", lw=0.8); ax.set_xticks(range(len(MARGIN_SITUATIONS))); ax.set_xticklabels([n for n, _ in MARGIN_SITUATIONS], fontsize=6); ax.set_xlim(-0.6, len(MARGIN_SITUATIONS) - 0.4)
        ax.set_title(f"$N{{=}}{N}$", fontsize=7); ax.grid(axis="y", alpha=0.3); ax.set_ylim(-200, 1500)
    axes[0].set_ylabel("safety margin (ms)")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="outside upper center", ncol=2, fontsize=6.5, handlelength=1.2, columnspacing=0.8)
    figstyle.save(fig, os.path.join(out, "safety_margin.pdf"))


def decomp(out):
    figstyle.rc()
    fig, axes = plt.subplots(1, 3, figsize=(figstyle.COL, 1.9), sharey=True)
    arms = ["naive", "etsi", "contract", "reference"]
    hatch = {"naive": "", "etsi": "//", "contract": "..", "reference": "xx"}
    for ax, (title, d) in zip(axes, DECOMP):
        for i, ak in enumerate(arms):
            n, cls = d[ak]; bottom = 0.0
            for ck, cl, color in CLASSES:
                fr = cls.get(ck, 0) / n
                if fr: ax.bar(i, fr, 0.7, bottom=bottom, color=color, edgecolor="k", linewidth=0.3, hatch=hatch[ak]); bottom += fr
            ax.text(i, bottom + 0.03, f"{sum(cls.values())}/{n}", ha="center", fontsize=6)
        ax.set_xticks(range(len(arms))); ax.set_xticklabels(["naive", "ETSI", "+c", "ref."], fontsize=6); ax.set_title(title.replace(", $N", "\n$N"), fontsize=6.5); ax.set_ylim(0, 1.15); ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("fraction of runs failed")
    fig.legend([Patch(facecolor=c, edgecolor="k", linewidth=0.3, label=l) for ck, l, c in CLASSES[:2]], [l for _, l, _ in CLASSES[:2]], loc="outside upper center", ncol=2)
    figstyle.save(fig, os.path.join(out, "arch_failure_decomp.pdf"))


SWEEP = {  # Fig. 17 mock: fraction failed per (arm, delay) and class, consistent with DECOMP at 1000 ms
    "junction": {"lats": [100, 200, 400, 700, 1000, 1500],
                 "naive": {"self_ghost": [0, 0, 0, 0.2, 0.8, 1.0]}, "etsi": {"self_ghost": [0, 0, 0, 0.2, 0.8, 1.0]},
                 "contract": {"collision": [0, 0, 0, 0, 0, 0.2]}, "reference": {"collision": [0, 0, 0, 0, 0, 0.33]}},
    "overtake": {"lats": [0, 100, 200, 300, 400, 500, 700, 1000, 1500, 2000, 3000],
                 "naive": {"self_ghost": [0, 0.1, 0.2, 0.1, 0.2, 0.4, 0, 0, 0, 0, 0], "collision": [0] * 9 + [0.9, 0.7]},
                 "etsi": {"self_ghost": [0, 0, 0, 0, 0.1, 0.6, 0, 0, 0, 0, 0], "collision": [0] * 9 + [0.9, 0.7]},
                 "contract": {"late": [0] * 8 + [0.1, 0.2, 0.1], "collision": [0] * 9 + [0.9, 0.7]},
                 "reference": {"collision": [0] * 9 + [1.0, 0.7]}},
}


def sweep(out):
    figstyle.rc()
    fig, axes = plt.subplots(1, 3, figsize=(figstyle.TXT, 1.8), sharey=True, gridspec_kw=dict(width_ratios=[3, 3, 8]))
    arms = ["reference", "naive", "etsi", "contract"]; hatch = {"reference": "", "naive": "//", "etsi": "xx", "contract": ".."}
    for ax, (title, key) in zip(axes, [("(a) Left turn, $N{=}4$", "junction"), ("(b) Straight crossing, $N{=}4$", "junction"), ("(c) Blind overtake, $N{=}1$", "overtake")]):
        d = SWEEP[key]; lats = d["lats"]; width = 0.84 / len(arms)
        for i, ak in enumerate(arms):
            bottoms = [0.0] * len(lats)
            for ck, cl, color in CLASSES:
                fr = d.get(ak, {}).get(ck, [0] * len(lats))
                ax.bar([x + (i - (len(arms) - 1) / 2) * width for x in range(len(lats))], fr, width, bottom=bottoms, color=color, edgecolor="k", linewidth=0.3, hatch=hatch[ak])
                bottoms = [b + f for b, f in zip(bottoms, fr)]
        ax.set_xticks(range(len(lats))); ax.set_xticklabels([str(l) for l in lats], rotation=45 if len(lats) > 6 else 0)
        ax.set_xlabel("configured delay (ms)"); ax.set_title(title); ax.set_ylim(0, 1.05); ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("fraction of runs failed, by class")
    cls_h = [Patch(facecolor=c, edgecolor="k", linewidth=0.3, label=l) for _, l, c in CLASSES]
    arm_h = [Patch(facecolor="white", edgecolor="k", linewidth=0.3, hatch=hatch[a], label=f"{lbl} (bars)") for a, lbl in zip(arms, ["reference", "no contract", "ETSI", "contract"])]
    fig.legend(handles=[x for pair in zip(cls_h, arm_h) for x in pair], loc="outside upper center", ncol=4)
    figstyle.save(fig, os.path.join(out, "arch_failure_sweep.pdf"))


def control(out):
    figstyle.rc()
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.COL, 1.8), sharey=True)
    lats = [0, 100, 200, 300, 500, 700, 1000, 1500, 2000, 3000]
    def curve(tau, width=400):
        return [max(0.0, min(1.0, 0.5 + (tau - x) / width)) for x in lats]
    for ax, geom in zip(axes, (LT, SCP)):
        for s, ls in ((70, "-"), (88, ":")):
            ref = BUDGET[geom][s][0]
            ax.plot([x + OFFSET[geom] for x in lats], curve(ref), "o" + ls, color="k", ms=3, label=f"emergency stop, {s} km/h")
            ax.plot([x + OFFSET[geom] for x in lats], curve(ref - CTRL_SHIFT), "s" + ls, color="#9467bd", ms=3, mfc="white", label=f"graded deceleration, {s} km/h")
        ax.axhline(0.8, color="gray", lw=0.7, ls="--"); ax.set_title(figstyle.GEOMS[geom][0]); ax.set_xlabel("measured p99 age at use (ms)"); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
    axes[0].set_ylabel("$\\Pr[S_{op}]$")
    fig.legend(*axes[0].get_legend_handles_labels(), loc="outside upper center", ncol=2, handlelength=1.6, columnspacing=0.8)
    figstyle.save(fig, os.path.join(out, "situation_control.pdf"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", nargs="+", required=True); ap.add_argument("--landing", required=True); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    fig6(a, a.out); budget_panels(a, a.out); margin(a, a.out); decomp(a.out); sweep(a.out); control(a.out)
    open(os.path.join(a.out, "README.md"), "w").write("Mock-ups drawn to the v14 review's expected pattern (fig_mockups.py), unmarked. "
                                                      "Replaced by build_figures.sh + place_figures.sh when the measured cells land.\n")
    print("mock-ups under", a.out)


if __name__ == "__main__":
    main()
