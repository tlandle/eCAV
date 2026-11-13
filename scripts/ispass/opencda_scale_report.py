#!/usr/bin/env python3
"""Tabulate the OpenCDA scale benchmark (opencda_scale_bench.py) beside the eCAV sweep.

    python scripts/ispass/opencda_scale_report.py experiment_results/ispass_scale/competitor_20261112 \
        --ecav paper_ispass/scale_sweep3.json --md paper_ispass/scale_competitor.md --json paper_ispass/scale_competitor.json
"""
import argparse
import glob
import json
import os


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    ap.add_argument("--ecav", default="paper_ispass/scale_sweep3.json")
    ap.add_argument("--md", required=True)
    ap.add_argument("--json", required=True)
    ap.add_argument("--notes", default=None, help="Markdown file appended to the table")
    a = ap.parse_args()

    ecav = {}
    for r in json.load(open(a.ecav)):
        ecav[(r["mode"], r.get("layout", "dense"), r["perception"], r["n"])] = r
    # the perception rows of the eCAV sweep are in the Markdown table only
    md = a.ecav[:-5] + ".md"
    if os.path.isfile(md):
        for line in open(md):
            c = [x.strip() for x in line.strip().strip("|").split("|")]
            if len(c) > 5 and c[0].endswith(", perception"):
                mode, layout = [x.strip() for x in c[0].split(",")[:2]]
                mean, p50, p99 = [float(x) for x in c[5].split("/")]
                ecav[(mode, layout, "on", int(c[1]))] = dict(wall_mean=mean, wall_p50=p50, wall_p99=p99)
    oc = {}
    for f in sorted(glob.glob(os.path.join(a.dir, "opencda_*.json"))):
        d = json.load(open(f))
        oc[(d["layout"], d["perception"], d["map_manager"], d["n"])] = d

    def cell(d):
        if d is None:
            return "-"
        if not d.get("completed"):
            return "failed"
        t = d["tick_ms"]
        return f'{t["mean"]:.0f} / {t["p50"]:.0f} / {t["p95"]:.0f}'

    def ecell(r):
        return "-" if r is None else f'{r["wall_mean"]:.0f} / {r["wall_p50"]:.0f} / {r["wall_p99"]:.0f}'

    out, rows = [], []
    out.append("OpenCDA: tick wall mean / p50 / p95 (ms). eCAV: tick wall mean / p50 / p99 (ms), from scale_sweep3.\n")
    for perc in ("off", "on"):
        for layout in ("dense", "spread"):
            ns = sorted({k[3] for k in oc if k[0] == layout and k[1] == perc})
            if not ns:
                continue
            out.append(f"\n### {layout} layout, perception {perc}\n")
            out.append("| N | OpenCDA, defaults | OpenCDA, map manager off | eCAV sequential | eCAV distributed | OpenCDA defaults / eCAV distributed | OpenCDA client RSS / CARLA RSS (GB) | GPU memory (MB) | planner exceptions |")
            out.append("|---|---|---|---|---|---|---|---|---|")
            for n in ns:
                on, off = oc.get((layout, perc, "on", n)), oc.get((layout, perc, "off", n))
                es, ed = ecav.get(("sequential", layout, perc, n)), ecav.get(("distributed", layout, perc, n))
                ratio = "-"
                if on and on.get("completed") and ed:
                    ratio = f'{on["tick_ms"]["mean"] / ed["wall_mean"]:.1f}'
                ref = on if on and on.get("completed") else off
                mem = f'{ref["peak_client_rss_gb"]} / {ref["peak_carla_rss_gb"]}' if ref and ref.get("completed") else "-"
                gpu = ref["peak_gpu_mb"] if ref and ref.get("completed") else "-"
                exc = " / ".join(str(x.get("planner_exceptions", "-")) if x and x.get("completed") else "-" for x in (on, off))
                out.append(f"| {n} | {cell(on)} | {cell(off)} | {ecell(es)} | {ecell(ed)} | {ratio} | {mem} | {gpu} | {exc} |")
                rows.append(dict(n=n, layout=layout, perception=perc, opencda_default=on, opencda_map_off=off,
                                 ecav_sequential_mean_ms=es and es["wall_mean"], ecav_distributed_mean_ms=ed and ed["wall_mean"]))
    meta = os.path.join(a.dir, "meta.txt")
    if os.path.isfile(meta):
        out.append("\nRun metadata:\n\n```\n" + open(meta).read().strip() + "\n```")
    if a.notes and os.path.isfile(a.notes):
        out.append("\n" + open(a.notes).read().strip())
    open(a.md, "w").write("\n".join(out) + "\n")
    json.dump(rows, open(a.json, "w"), indent=1)
    print("\n".join(out))


if __name__ == "__main__":
    main()
