#!/usr/bin/env python3
"""Situation figure for the SenSys resubmission (expanded Figure 7).

    python scripts/sensys_analysis/fig_situation.py --stamps sensys_nov27_j2 sensys_nov27_ov3 \
        --roots experiment_results experiment_results/_cetus_mirror/experiment_results \
        [--bin 50] [--threshold 0.8] [--boot 500] [--out safety_envelope_paper/analysis/fig_situation]

fig_situation.pdf: tau_max of the exact-state reference against approach speed
    per geometry with a 95% bootstrap interval over runs; a cell whose P(S_op)
    never falls below the threshold at any tested age is a lower bound (hollow
    triangle at the top). Supplementary: fig_situation_contract.pdf adds the
    contract arm (dashed, hollow); fig_situation_control.pdf is the reference
    under each hazard-response policy, one panel per speed with control-stage
    runs; fig_situation_ref.pdf is the reference envelope of each geometry at
    its reference speed. All at column width.

Runs from several roots (atlas results and the cetus mirror) are merged; a
design cell and seed present in more than one root keeps the first root's run.
Per-run axis values are cached in <out>/axis_cache.json (keyed by path and
mtime) because every value is a pass over a per-tick CSV.
"""
import argparse
import collections
import csv
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import load_runs, wilson, ROOT  # noqa: E402
from envelope_measured import run_axis, ARM_LABEL  # noqa: E402

ARMS = {"oracle": ("oracle", "on"), "contract": ("late_fusion", "on"), "naive": ("late_fusion", "off")}
EXCLUDE_KV = ("assoc_metric", "hazard_response")   # gate and control arms are not the physics/landing arms


def pav_rates(pts):
    """Monotone non-increasing fit of P over ascending bin centres by pooling
    adjacent violators; pts = [(x, successes, count)] sorted by x."""
    blocks = []
    for x, k, n in pts:
        blocks.append([x, float(k), float(n), [x]])
        while len(blocks) >= 2 and blocks[-2][1] / blocks[-2][2] < blocks[-1][1] / blocks[-1][2]:
            b = blocks.pop()
            a = blocks.pop()
            blocks.append([a[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]])
    rate = {}
    for x0, k, n, xs in blocks:
        for x in xs:
            rate[x] = k / n
    return rate


def tau_at(pts, thr):
    """(tau, censored): the largest bin centre at which the monotone fit is
    still at or above thr, walking up from the lowest bin and stopping at the
    first bin below it. censored is True when no bin falls below thr."""
    if not pts:
        return None, False
    rate = pav_rates(pts)
    tau = None
    for x, _, _ in pts:
        if rate[x] >= thr:
            tau = x
        else:
            return tau, False
    return tau, True


def bins_of(runs, width):
    b = collections.defaultdict(lambda: [0, 0])
    for r in runs:
        c = int(r["axis_ms"] // width) * width + width / 2
        b[c][0] += r["s_op"]
        b[c][1] += 1
    return sorted((x, k, n) for x, (k, n) in b.items())


def boot_tau(runs, width, thr, nboot, rng):
    taus = []
    for _ in range(nboot):
        smp = [rng.choice(runs) for _ in runs]
        t, _c = tau_at(bins_of(smp, width), thr)
        taus.append(t if t is not None else 0.0)
    taus.sort()
    return taus[int(0.025 * len(taus))], taus[max(int(0.975 * len(taus)) - 1, 0)]


def components(runs):
    return dict(collided=sum(1 for r in runs if r.get("focal_collisions", 0) > 0),
                self_ghost=sum(1 for r in runs if r.get("focal_ghost_episodes", 0) > 0),
                false_brake=sum(1 for r in runs if r.get("focal_fp_episodes", 0) > 0),
                stalled=sum(1 for r in runs if r.get("s_prog", 1) == 0))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stamps", nargs="+", default=["sensys_nov27_j2", "sensys_nov27_ov3"])
    ap.add_argument("--roots", nargs="+", default=[os.path.join(ROOT, "experiment_results")])
    ap.add_argument("--bin", type=float, default=50.0)
    ap.add_argument("--threshold", type=float, default=0.8)
    ap.add_argument("--boot", type=int, default=500)
    ap.add_argument("--axis", choices=("critical", "global", "brake"), default="critical")
    ap.add_argument("--measured-speeds", default=None,
                    help="JSON {set point km/h: measured cruise km/h}; the speed axis and the tau CSV then carry the measured speed "
                         "(the recalibrated sweep holds the ego at cruise before the launch; cruise sits under the set point)")
    ap.add_argument("--speeds", nargs="*", type=int, default=None,
                    help="set-point speeds (km/h) to plot; cells at other speeds are dropped")
    ap.add_argument("--out", default=os.path.join(ROOT, "safety_envelope_paper", "analysis", "fig_situation"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    rng = random.Random(0)
    measured = {str(int(float(k))): float(v) for k, v in json.load(open(a.measured_speeds)).items()} if a.measured_speeds else {}

    # ---- load and merge ---------------------------------------------------
    cache_path = os.path.join(a.out, "axis_cache.json")
    cache = json.load(open(cache_path)) if os.path.exists(cache_path) else {}
    seen = {}
    dup = 0
    for root in a.roots:
        for stamp in a.stamps:
            for r in load_runs(root, stamp):
                if r.get("bad") or r.get("speed") == "orig":
                    continue
                if a.speeds and int(r["speed"]) not in a.speeds:
                    continue
                key = (r["geom"], r["speed"], r["mgr"], r["anch"], r["N"], r["kv"], r["lat"], r["rep"])
                if key in seen:
                    dup += 1
                    continue
                ck = r["path"] + "@" + str(int(os.path.getmtime(r["path"])))
                if ck not in cache:
                    cache[ck] = run_axis(r, a.axis)
                r["axis_ms"] = cache[ck]
                r["speed_kmh"] = int(r["speed"])
                seen[key] = r
    json.dump(cache, open(cache_path, "w"))
    runs = [r for r in seen.values() if r["axis_ms"] == r["axis_ms"]]
    print(f"{len(seen)} runs merged ({dup} duplicates across roots dropped), {len(runs)} with a measured age")

    # ---- group by geometry, speed, arm; one knob set per group --------------
    groups = collections.defaultdict(list)
    for r in runs:
        for arm, (mgr, anch) in ARMS.items():
            if r["mgr"] == mgr and r["anch"] == anch and r["N"] == 1 and not any(k in r["kv"] for k in EXCLUDE_KV):
                groups[(r["geom"], r["speed_kmh"], arm, r["kv"])].append(r)
    chosen = {}
    for (geom, spd, arm, kv), v in groups.items():
        cur = chosen.get((geom, spd, arm))
        if cur is None or len(v) > len(cur[1]):
            chosen[(geom, spd, arm)] = (kv, v)
    for (geom, spd, arm), (kv, v) in sorted(chosen.items()):
        others = [(k, len(w)) for (g, s, ar, k), w in groups.items() if (g, s, ar) == (geom, spd, arm) and k != kv]
        if others:
            print(f"note: {geom} {spd} km/h {arm}: using knobs {kv} ({len(v)} runs); other knob sets present {others}")

    # ---- tau per group with bootstrap interval ---------------------------------
    rows = []
    for (geom, spd, arm), (kv, v) in sorted(chosen.items()):
        pts = bins_of(v, a.bin)
        tau, cens = tau_at(pts, a.threshold)
        lo, hi = boot_tau(v, a.bin, a.threshold, a.boot, rng)
        comp = components(v)
        rows.append(dict(geom=geom, speed_kmh=spd, measured_kmh=measured.get(str(spd), spd), arm=arm, kv=kv, runs=len(v), bins=len(pts),
                         max_bin_ms=pts[-1][0], tau_ms=tau if tau is not None else 0.0,
                         censored=int(cens), boot_lo_ms=lo, boot_hi_ms=hi, **comp))
    with open(os.path.join(a.out, "fig_situation_tau.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\n| geometry | km/h | arm | runs | tau {a.threshold} (ms) | bootstrap 95% | note |\n|---|---|---|---|---|---|---|")
    for r in rows:
        note = f"beyond the sweep (P >= {a.threshold} to {r['max_bin_ms']:.0f} ms)" if r["censored"] else ""
        print(f"| {r['geom']} | {r['speed_kmh']} | {r['arm']} | {r['runs']} | {r['tau_ms']:.0f} | [{r['boot_lo_ms']:.0f}, {r['boot_hi_ms']:.0f}] | {note} |")

    # ---- figures ---------------------------------------------------------------
    # Drawn at column width (figstyle.COL). fig_situation.pdf: (a) tau_max against approach speed,
    # (b) controller sensitivity, one panel per speed with control-stage runs. fig_situation_ref.pdf
    # (supplementary): the reference envelope of each geometry at its reference speed.
    try:
        import figstyle
        from figstyle import plt
    except ImportError:
        print("matplotlib missing, no figure")
        return 0
    figstyle.rc()
    EB = figstyle.EB
    geoms = sorted(set(g for g, _, _ in chosen))
    gname = {g: figstyle.GEOMS.get(g, (g, "k"))[0] for g in geoms}
    colors = {g: figstyle.GEOMS.get(g, (g, "k"))[1] for g in geoms}

    fig, ax = plt.subplots(figsize=(figstyle.COL, 2.0))
    for geom in geoms:
        speeds = sorted(s for (g, s, arm) in chosen if g == geom and arm == "oracle")
        if not speeds:
            continue
        ref = 70 if 70 in speeds else speeds[-1]
        v = chosen[(geom, ref, "oracle")][1]
        pts = bins_of(v, a.bin)
        xs = [x for x, _, _ in pts]
        ps, los, his = zip(*[wilson(k, n) for _, k, n in pts])
        ax.errorbar(xs, ps, yerr=[[p - l for p, l in zip(ps, los)], [h - p for p, h in zip(ps, his)]],
                    marker="o", color=colors[geom], label=f"{gname[geom]}, {ref} km/h", **EB)
    ax.axhline(a.threshold, color="gray", lw=0.8, ls="--")
    ax.set_xlabel("conflict actor's measured p99 age at use (ms)")
    ax.set_ylabel("$\\Pr[S_{op}]$")
    ax.set_ylim(-0.05, 1.05)
    ax.grid(alpha=0.3)
    ax.legend(loc="lower left")
    figstyle.save(fig, os.path.join(a.out, "fig_situation_ref.pdf"))
    plt.close(fig)

    from matplotlib.lines import Line2D
    ytop = 2300.0

    def budget_axes(ax, arms):
        """tau_max against approach speed per geometry: colour is the geometry, the reference solid with
        filled markers, the contract arm dashed with hollow markers, bootstrap intervals; a cell that never
        fell below the threshold at any tested age is a lower bound, drawn as a hollow triangle at the top.
        A geometry with one speed (the overtake) is one point, named in the legend. Returns True when a lower bound was drawn."""
        bound = False
        for geom in geoms:
            for arm, ls, hollow in arms:
                pts = sorted((r["measured_kmh"], r["tau_ms"], r["boot_lo_ms"], r["boot_hi_ms"], r["censored"], r["max_bin_ms"])
                             for r in rows if r["geom"] == geom and r["arm"] == arm)
                if not pts:
                    continue
                ax.plot([p[0] for p in pts], [min(p[1], ytop) for p in pts], ls=ls, lw=1.2, color=colors[geom])
                for s, t, lo, hi, cens, mx in pts:
                    if cens:
                        ax.plot([s], [ytop], marker="^", mfc="white", mec=colors[geom], ms=6, ls="none", clip_on=False)
                        bound = True
                    else:
                        ax.errorbar([s], [t], yerr=[[t - lo], [hi - t]], marker="o", mfc="white" if hollow else colors[geom],
                                    mec=colors[geom], color=colors[geom], ls="none", **EB)
        ax.set_xlabel("approach speed (km/h)")
        ax.set_ylabel(r"$\tau_{\max}$ at $\Pr[S_{op}] \geq %g$ (ms)" % a.threshold)
        ax.set_ylim(0, ytop)
        ax.grid(alpha=0.3)
        return bound

    # main figure: the exact-state reference alone
    fig, ax = plt.subplots(figsize=(figstyle.COL, 2.0))
    bound = budget_axes(ax, (("oracle", "-", False),))
    def geom_handles(lw):
        out = []
        for g in geoms:
            speeds = sorted({r["speed_kmh"] for r in rows if r["geom"] == g and r["arm"] == "oracle"})
            if len(speeds) > 1:
                out.append(Line2D([], [], color=colors[g], lw=lw, marker="o" if lw < 1.5 else None, label=gname[g]))
            elif speeds:
                out.append(Line2D([], [], color=colors[g], ls="none", marker="o", label=f"{gname[g]}, {speeds[0]} km/h only"))
        return out
    handles = geom_handles(1.2)
    if bound:
        handles.append(Line2D([], [], color="k", ls="none", marker="^", mfc="white", ms=6, label="lower bound"))
    ax.legend(handles=handles, loc="upper right")
    figstyle.save(fig, os.path.join(a.out, "fig_situation.pdf"))
    plt.close(fig)

    # supplementary: the reference against V2X2V prediction with the contract
    fig, ax = plt.subplots(figsize=(figstyle.COL, 2.0))
    bound = budget_axes(ax, (("oracle", "-", False), ("contract", "--", True)))
    handles = geom_handles(1.5)
    handles += [Line2D([], [], color="k", lw=1.2, ls="-", marker="o", label="reference"),
                Line2D([], [], color="k", lw=1.2, ls="--", marker="o", mfc="white", label="V2X2V prediction, +contract")]
    if bound:
        handles.append(Line2D([], [], color="k", ls="none", marker="^", mfc="white", ms=6, label="lower bound"))
    fig.legend(handles=handles, loc="outside upper center", ncol=2)
    figstyle.save(fig, os.path.join(a.out, "fig_situation_contract.pdf"))
    plt.close(fig)

    # supplementary: control response, the reference under the emergency stop (solid) and under deceleration
    # graded by the time to collision (dashed), per geometry colour, one panel per speed; Wilson bars.
    control = [r for r in runs if "hazard_response" in r["kv"] and r["mgr"] == "oracle" and r["N"] == 1]
    cspeeds = sorted({r["speed_kmh"] for r in control})
    if cspeeds:
        fig, axes_b = plt.subplots(1, len(cspeeds), figsize=(figstyle.COL, 1.7), sharey=True)
        axes_b = [axes_b] if len(cspeeds) == 1 else list(axes_b)
        for j, (axb, spd) in enumerate(zip(axes_b, cspeeds)):
            for geom in geoms:
                if (geom, spd, "oracle") not in chosen:
                    continue
                series = [("stop", chosen[(geom, spd, "oracle")][1], "-", colors[geom])]
                by_pol = collections.defaultdict(list)
                for r in control:
                    if r["geom"] == geom and r["speed_kmh"] == spd:
                        by_pol[json.loads(r["kv"]).get("behavior.hazard_response", "?")].append(r)
                for pol, v in sorted(by_pol.items()):
                    series.append((pol, v, "--", colors[geom]))
                for pol, v, ls, color in series:
                    pts = bins_of(v, a.bin)
                    xs = [x for x, _, _ in pts]
                    ps, los, his = zip(*[wilson(k, n) for _, k, n in pts])
                    axb.errorbar(xs, ps, yerr=[[p - l for p, l in zip(ps, los)], [h - p for p, h in zip(ps, his)]],
                                 marker="o", ms=3.5, ls=ls, lw=1.2, color=color, mfc=color if ls == "-" else "white",
                                 capsize=1.5, elinewidth=0.6, capthick=0.6)
                    print(f"(control) {geom} {spd} km/h {pol}: {len(v)} runs, components {components(v)}")
            axb.axhline(a.threshold, color="gray", lw=0.8, ls="--")
            axb.set_xlabel("measured p99 age (ms)")
            axb.set_ylim(-0.05, 1.05)
            axb.grid(alpha=0.3)
            axb.set_title(f"({'abcd'[j]}) {spd} km/h")
        axes_b[0].set_ylabel("$\\Pr[S_{op}]$")
        handles = [Line2D([], [], color=colors[g], lw=1.5, label=gname[g]) for g in geoms if any((g, s, "oracle") in chosen for s in cspeeds)]
        handles += [Line2D([], [], color="k", lw=1.2, ls="-", marker="o", ms=3.5, label="emergency stop"),
                    Line2D([], [], color="k", lw=1.2, ls="--", marker="o", ms=3.5, mfc="white", label="graded by time to collision")]
        fig.legend(handles=handles, loc="outside upper center", ncol=2)
        figstyle.save(fig, os.path.join(a.out, "fig_situation_control.pdf"))
        plt.close(fig)
    else:
        print("(control) no control-stage runs (behavior.hazard_response) in the merged set; figure omitted")
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
