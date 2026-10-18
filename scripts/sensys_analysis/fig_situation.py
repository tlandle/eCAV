#!/usr/bin/env python3
"""Situation figure for the SenSys resubmission (expanded Figure 7).

    python scripts/sensys_analysis/fig_situation.py --stamps sensys_nov27_j2 sensys_nov27_ov3 \
        --roots experiment_results experiment_results/_cetus_mirror/experiment_results \
        [--bin 50] [--threshold 0.8] [--boot 500] [--out safety_envelope_paper/analysis/fig_situation]

(a) P(S_op) against the measured age at use (the conflict actor's p99 age at
    use over the decision window, the paper's axis, as in envelope_measured.py)
    for the oracle arm at each geometry's reference speed (70 km/h at the
    junction, the overtake's own speed);
(b) tau_max against approach speed per geometry for the oracle and the
    contract arm, with a 95% bootstrap interval of tau_max over runs; a point
    whose P(S_op) never falls below the threshold inside the sweep is censored
    (hollow marker, interval open upwards) and reported as "beyond the sweep";
(c) controller sensitivity: the oracle at 70 km/h under each hazard-response
    policy, with the S_op components (collision, self-ghost, false brake, stall)
    per bin; drawn only when runs with behavior.hazard_response exist.

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
    ap.add_argument("--out", default=os.path.join(ROOT, "safety_envelope_paper", "analysis", "fig_situation"))
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    rng = random.Random(0)

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
        rows.append(dict(geom=geom, speed_kmh=spd, arm=arm, kv=kv, runs=len(v), bins=len(pts),
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

    # ---- figure --------------------------------------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib missing, no figure")
        return 0
    control = [r for r in runs if "hazard_response" in r["kv"] and r["mgr"] == "oracle"]
    npanel = 3 if control else 2
    fig, axes = plt.subplots(1, npanel, figsize=(3.4 * npanel, 2.9))
    geoms = sorted(set(g for g, _, _ in chosen))
    short = {"openscenario_3_edge_late_fusion": "left turn across path",
             "scp_edge_late_fusion": "straight crossing path",
             "blind_overtake_edge_late_fusion": "blind overtake"}
    colors = {g: c for g, c in zip(geoms, ("C0", "C1", "C2", "C3"))}

    # (a) oracle envelope at the reference speed per geometry
    ax = axes[0]
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
                    marker="o", ms=3, capsize=2, lw=1, color=colors[geom], label=f"{short.get(geom, geom)}, {ref} km/h")
    ax.axhline(a.threshold, color="gray", lw=0.8, ls="--")
    ax.set_xlabel("measured age at use, conflict actor p99 (ms)")
    ax.set_ylabel("P(S_op)")
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("(a) physics boundary per geometry (oracle)", fontsize=9)
    ax.legend(fontsize=6)

    # (b) tau_max against approach speed, oracle solid and contract dashed
    ax = axes[1]
    for geom in geoms:
        for arm, ls in (("oracle", "-"), ("contract", "--")):
            pts = sorted((r["speed_kmh"], r["tau_ms"], r["boot_lo_ms"], r["boot_hi_ms"], r["censored"], r["max_bin_ms"])
                         for r in rows if r["geom"] == geom and r["arm"] == arm)
            if not pts:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, ls=ls, lw=1, color=colors[geom], label=f"{short.get(geom, geom)}, {arm}")
            for s, t, lo, hi, cens, mx in pts:
                if cens:
                    ax.plot([s], [t], marker="^", mfc="white", mec=colors[geom], ms=6, ls="none")
                else:
                    ax.errorbar([s], [t], yerr=[[t - lo], [hi - t]], marker="o", ms=4, capsize=2, color=colors[geom], ls="none")
    ax.set_xlabel("approach speed (km/h)")
    ax.set_ylabel(f"tau_max at P(S_op) >= {a.threshold} (ms)")
    ax.set_title("(b) useful-age budget against speed", fontsize=9)
    ax.legend(fontsize=6)

    # (c) controller sensitivity, when the control stage exists
    if control:
        ax = axes[2]
        by_pol = collections.defaultdict(list)
        for r in control:
            pol = json.loads(r["kv"]).get("behavior.hazard_response", "?")
            by_pol[(r["geom"], pol)].append(r)
        base = [r for r in runs if r["mgr"] == "oracle" and r["speed_kmh"] == 70 and "hazard_response" not in r["kv"]
                and not any(k in r["kv"] for k in EXCLUDE_KV)]
        for r in base:
            by_pol[(r["geom"], "stop")].append(r)
        for (geom, pol), v in sorted(by_pol.items()):
            pts = bins_of(v, a.bin)
            xs = [x for x, _, _ in pts]
            ps = [k / n for _, k, n in pts]
            ax.plot(xs, ps, marker="o", ms=3, lw=1, color=colors.get(geom, "k"), ls="-" if pol == "stop" else "--",
                    label=f"{short.get(geom, geom)}, {pol}")
            print(f"(c) {geom} {pol}: {len(v)} runs, components {components(v)}")
        ax.axhline(a.threshold, color="gray", lw=0.8, ls="--")
        ax.set_xlabel("measured age at use, conflict actor p99 (ms)")
        ax.set_ylabel("P(S_op)")
        ax.set_ylim(-0.02, 1.02)
        ax.set_title("(c) controller sensitivity, oracle 70 km/h", fontsize=9)
        ax.legend(fontsize=6)
    else:
        print("(c) no control-stage runs (behavior.hazard_response) in the merged set; panel omitted")

    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(a.out, f"fig_situation.{ext}"), dpi=180)
    plt.close(fig)
    print("wrote", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
