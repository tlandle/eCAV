#!/usr/bin/env python3
"""Table, JSON and figure for a scale sweep (launch_scale.sh results directories).

    python scripts/ispass/scale_report.py <stamp> [--out paper_ispass]

Reads experiment_results/ispass_scale/<stamp>_<config>_n<N> for the configurations
    dense, spread          distributed, one process per actor, no perception
    seqdense, seqspread    sequential single process, no perception
    pdense                 distributed, perception on (shared inference service)
and writes <out>/scale_<stamp>.json, <out>/scale_<stamp>.md and <out>/fig_scale_<stamp>.pdf.
Figure: (a) simulated seconds per wall-clock second against N for every configuration,
(b) the per-tick breakdown of the distributed runs, dense and spread, stacked.
Column width (3.4 in), 7 pt text.
"""
import glob
import json
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RES = os.path.join(ROOT, "experiment_results", "ispass_scale")
CONFIGS = ["dense", "spread", "seqdense", "seqspread", "pdense"]
LABEL = {"dense": "distributed, dense", "spread": "distributed, spread", "seqdense": "sequential, dense",
         "seqspread": "sequential, spread", "pdense": "distributed, dense, perception"}
GROUPS = [("clients", ["client compute", "client loop gap"]),
          ("edge step", ["edge step"]),
          ("transport", ["orchestrator -> edge", "edge fan-out", "client reply", "edge -> orchestrator",
                         "orchestrator unpack"]),
          ("world tick", ["world tick"])]


def load(stamp):
    runs = []
    for d in sorted(glob.glob(os.path.join(RES, f"{stamp}_*_n*"))):
        m = re.match(rf"{re.escape(stamp)}_(\w+?)_n(\d+)$", os.path.basename(d))
        if not m or m.group(1) not in CONFIGS:
            continue
        out = subprocess.run([sys.executable, os.path.join(ROOT, "scripts/ispass/scale_breakdown.py"), d, "--json"],
                             capture_output=True, text=True).stdout.strip()
        try:
            r = json.loads(out)
        except ValueError:
            print(f"no breakdown for {d}", file=sys.stderr)
            continue
        if not r.get("ticks"):
            print(f"no ticks in {d}", file=sys.stderr)
            continue
        r["config"] = m.group(1)
        gpu = os.path.join(d, "gpu.csv")
        if os.path.exists(gpu):
            rows = [l.split(",") for l in open(gpu).read().strip().splitlines() if l[:1].isdigit()]
            if rows:
                r["gpu_util_mean"] = sum(float(x[0]) for x in rows) / len(rows)
                r["gpu_mem_max_mib"] = max(float(x[1]) for x in rows)
        yl = os.path.join(d, "yolo_grpc.log")
        if os.path.exists(yl):
            tot = sorted(float(x) for x in re.findall(r"PerceptionServicer\].*? total=(\d+)ms", open(yl, errors="ignore").read()))
            if tot:
                q = lambda p: tot[min(len(tot) - 1, int(len(tot) * p))]
                r["service_ms"] = dict(p50=q(.5), p95=q(.95), p99=q(.99), requests=len(tot))
                # one request per client per tick: delivered detection updates per wall-clock second
                r["service_req_per_wall_s"] = len(tot) / (r["wall_mean"] / 1000.0 * max(r["ticks"] + 5, 1))
        runs.append(r)
    return runs


def f(v, fmt="{:.0f}"):
    return fmt.format(v) if v is not None and v == v else "-"


def table(runs):
    L = ["| configuration | N | RSUs | in sensing range | in forecast range | tick wall mean / p50 / p99 (ms) | sim s per wall s | "
         "clients | edge step | transport | world | client compute, all (ms) | first tick (s) | CPU busy | GPU util / mem | "
         "inference service p50 / p95 / p99 (ms), requests per wall s |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in CONFIGS:
        for r in sorted((x for x in runs if x["config"] == c), key=lambda x: x["n"]):
            g = {name: sum((r["rows"].get(k) or 0.0) for k in keys) for name, keys in GROUPS}
            busy = " ".join(f"{v * 100:.0f} %" for v in r["cpu_busy"].values()) or "-"
            gpu = f"{r['gpu_util_mean']:.0f} % / {r['gpu_mem_max_mib'] / 1024:.1f} GB" if "gpu_util_mean" in r else "-"
            seq = r["mode"] == "sequential"
            L.append(f"| {LABEL[c]} | {r['n']} | {r['rsus'] if not seq else '-'} | {f(r['neighbours_sense'], '{:.1f}')} | "
                     f"{f(r['neighbours_recipient'], '{:.1f}')} | {r['wall_mean']:.0f} / {r['wall_p50']:.0f} / {r['wall_p99']:.0f} | "
                     f"{r['sim_s_per_wall_s']:.3f} | {'-' if seq else f(g['clients'])} | {f(g['edge step'])}{' (with vehicles)' if seq else ''} | "
                     f"{'-' if seq else f(g['transport'])} | {f(g['world tick'])} | {f(r['client_compute_all_mean'])} | "
                     f"{f(r['first_tick_s'])} | {busy} | {gpu} | " +
                     (f"{r['service_ms']['p50']:.0f} / {r['service_ms']['p95']:.0f} / {r['service_ms']['p99']:.0f}, "
                      f"{r['service_req_per_wall_s']:.1f}" if "service_ms" in r else "-") + " |")
    return "\n".join(L)


def figure(runs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 7, "axes.labelsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
                         "legend.fontsize": 6.5, "axes.linewidth": 0.6, "pdf.fonttype": 42})
    fig, (a, b) = plt.subplots(1, 2, figsize=(3.4, 1.75), gridspec_kw={"width_ratios": [1.0, 1.15]})
    style = {"dense": ("C0", "o", "-"), "spread": ("C1", "s", "-"), "seqdense": ("C0", "o", ":"),
             "seqspread": ("C1", "s", ":"), "pdense": ("C3", "^", "-")}
    short = {"dense": "dist., dense", "spread": "dist., spread", "seqdense": "seq., dense",
             "seqspread": "seq., spread", "pdense": "dist., percep."}
    for c in CONFIGS:
        rs = sorted((x for x in runs if x["config"] == c), key=lambda x: x["n"])
        if rs:
            col, mk, ls = style[c]
            a.plot([r["n"] for r in rs], [r["sim_s_per_wall_s"] for r in rs], color=col, marker=mk, ls=ls, ms=2.5, lw=0.9,
                   label=short[c])
    a.axhline(1.0, color="0.6", lw=0.5, ls="--")
    a.set_xscale("log", base=2); a.set_yscale("log")
    a.set_xlabel("vehicles N"); a.set_ylabel("simulated s per wall-clock s")
    ns = sorted({r["n"] for r in runs})
    a.set_xticks(ns); a.set_xticklabels([str(n) for n in ns]); a.minorticks_off()
    a.legend(frameon=False, loc="lower left", handlelength=1.6, borderpad=0.1, labelspacing=0.2)
    a.set_title("(a) throughput", fontsize=7, pad=2)
    cols = ["C0", "C2", "0.55", "C4"]
    xs, lab = [], []
    x = 0
    for c in ("dense", "spread"):
        rs = sorted((r for r in runs if r["config"] == c), key=lambda r: r["n"])
        for r in rs:
            bottom = 0.0
            for (name, keys), col in zip(GROUPS, cols):
                v = sum((r["rows"].get(k) or 0.0) for k in keys) / 1000.0
                b.bar(x, v, 0.8, bottom=bottom, color=col, lw=0, label=name if x == 0 else None)
                bottom += v
            xs.append(x); lab.append(str(r["n"])); x += 1
        if rs:
            b.text(x - len(rs) / 2 - 0.5, 1.0, c, transform=b.get_xaxis_transform(), ha="center", va="top", fontsize=7)
        x += 0.8
    b.set_yscale("log"); b.set_xticks(xs); b.set_xticklabels(lab, rotation=90)
    b.set_xlabel("vehicles N"); b.set_ylabel("per-tick time (s)")
    b.legend(frameon=False, loc="upper left", bbox_to_anchor=(0.0, 0.9), handlelength=0.9, borderpad=0.1, labelspacing=0.2)
    b.set_title("(b) critical-path breakdown", fontsize=7, pad=2)
    fig.subplots_adjust(left=0.13, right=0.995, bottom=0.24, top=0.9, wspace=0.42)
    fig.savefig(path)


def main():
    stamp = sys.argv[1]
    out = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else os.path.join(ROOT, "paper_ispass")
    runs = load(stamp)
    json.dump(runs, open(os.path.join(out, f"scale_{stamp}.json"), "w"), indent=1)
    t = table(runs)
    open(os.path.join(out, f"scale_{stamp}.md"), "w").write(t + "\n")
    print(t)
    if runs:
        figure(runs, os.path.join(out, f"fig_scale_{stamp}.pdf"))
        print(os.path.join(out, f"fig_scale_{stamp}.pdf"))


if __name__ == "__main__":
    main()
