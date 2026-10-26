#!/usr/bin/env python3
"""Object sharing ablation figure (sba_ablation): the merge-point arms against the conflict actor's
measured p99 age at use on one geometry and speed (naive, ETSI self-announcement, Mahalanobis gate
with and without the contract, VIPS with and without the contract, contract, reference), Wilson bars,
pooled over the hosts' runs.csv files: (a) the arms without the contract, (b) the same with the contract
and the reference. Points are the measured cells at their mean measured age, no connecting lines.
Drawn at text width (figstyle.TXT) for a figure* float. The fault study is fig_faults.py.
    python scripts/sensys_analysis/fig_ablation.py --csv <envelope csvs...> \
        --geom openscenario_3_edge_late_fusion --speed 70 --out figures/sba_ablation.pdf
"""
import argparse, collections, csv, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from campaign_check import wilson  # noqa: E402
import figstyle  # noqa: E402
from figstyle import plt  # noqa: E402

PANELS = [("(a) without the contract", ["naive", "etsi", "gate", "vips"]),
          ("(b) with the contract, and the reference", ["contract", "gate+c", "vips+c", "reference"])]
SHORT = {"naive": "no contract", "contract": "contract"}   # every arm but the reference is V2X2V prediction here


def arm_key(mgr, anch, kv):
    if mgr == "oracle":
        return "reference"
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
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--geom", default="openscenario_3_edge_late_fusion"); ap.add_argument("--speed", type=int, default=70)
    ap.add_argument("--max-age", type=float, default=1100); ap.add_argument("--out", required=True)
    a = ap.parse_args()
    acc = collections.defaultdict(list)   # (arm, lat) -> [(axis_ms, s_op)]
    for p in a.csv:
        for r in csv.DictReader(open(p)):
            if r["geom"] != a.geom or int(r["speed"]) != a.speed or int(r["N"]) != 1:
                continue
            if "v2x_regime" in r.get("cell", ""):
                continue
            kv = json.loads(r["kv"]) if r["kv"] else {}
            if any(k.startswith(("beacon_fault", "beacon_validity")) or k == "behavior.hazard_response" for k in kv):
                continue
            k = arm_key(r["mgr"], r["anch"], kv)
            if k is None:
                continue
            acc[(k, int(r["lat"]))].append((float(r["axis_ms"]), int(r["s_op"])))
    figstyle.rc()
    fig, axes = plt.subplots(1, 2, figsize=(figstyle.TXT, 1.9), sharey=True)
    handles, labels = [], []
    for ax, (title, keys) in zip(axes, PANELS):
        for key in keys:
            label, color, marker = figstyle.ARMS[key]; label = SHORT.get(key, label)
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
            h = ax.errorbar(xs, ps, yerr=[lo, hi], fmt=marker, color=color, ls="none", ms=5.5, label=label, **figstyle.EB)
            handles.append(h); labels.append(label)
        ax.axhline(0.8, color="gray", lw=0.8, ls="--"); ax.set_ylim(-0.05, 1.05); ax.grid(alpha=0.3)
        ax.set_xlabel("conflict actor's measured p99 age at use (ms)"); ax.set_title(title)
    axes[0].set_ylabel("$\\Pr[S_{op}]$")
    fig.legend(handles, labels, loc="outside upper center", ncol=4, columnspacing=1.0)
    figstyle.save(fig, a.out)
    print("wrote", a.out, "arms:", sorted({k for (k, _) in acc}))


if __name__ == "__main__":
    main()
