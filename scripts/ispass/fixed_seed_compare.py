#!/usr/bin/env python3
"""Compare two fixed-seed runs tick by tick: ego position RMS and maximum difference,
decision agreement, and the edge publish sequence (order and ticks) of the consumed state.

    python scripts/ispass/fixed_seed_compare.py <stampA> <stampB> [--root experiment_results/highway_merge_edge_late_fusion]
"""
import argparse, csv, glob, json, math, os, sys


def load(stamp, root):
    d = glob.glob(os.path.join(root, stamp, "**", "run_1"), recursive=True)
    if not d:
        sys.exit(f"no run under {root}/{stamp}")
    d = d[0]
    p = (glob.glob(d + "/evaluation_output/per_tick_age.csv") + glob.glob(d + "/per_tick_age.csv"))[0]
    rows = list(csv.DictReader(open(p)))
    m = json.load(open(d + "/simulation_metrics.json"))
    return rows, m


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a"); ap.add_argument("b")
    ap.add_argument("--root", default="experiment_results/highway_merge_edge_late_fusion")
    a = ap.parse_args()
    ra, ma = load(a.a, a.root); rb, mb = load(a.b, a.root)
    ego = {}
    for rows, key in ((ra, "a"), (rb, "b")):
        for r in rows:
            if r["ego_x"]:
                ego.setdefault(int(r["tick"]), {})[key] = (float(r["ego_x"]), float(r["ego_y"]), r["decision"], float(r["ego_speed_mps"]))
    common = [t for t, v in sorted(ego.items()) if "a" in v and "b" in v]
    d = [math.hypot(ego[t]["a"][0] - ego[t]["b"][0], ego[t]["a"][1] - ego[t]["b"][1]) for t in common]
    rms = math.sqrt(sum(x * x for x in d) / len(d)) if d else float("nan")
    dec_agree = sum(1 for t in common if ego[t]["a"][2] == ego[t]["b"][2]) / len(common) if common else float("nan")
    first_div = next((t for t, x in zip(common, d) if x > 0.05), None)
    pa = sorted({(int(r["publish_tick"]), int(r["publish_seq"])) for r in ra if r["publish_seq"]})
    pb = sorted({(int(r["publish_tick"]), int(r["publish_seq"])) for r in rb if r["publish_seq"]})
    fa = list(ma["edges"].values())[0]["total_frames"]; fb = list(mb["edges"].values())[0]["total_frames"]
    print(f"frames {fa} / {fb}; common ticks {len(common)}; ego position RMS {rms:.3f} m, max {max(d) if d else float('nan'):.3f} m; "
          f"first divergence > 5 cm at tick {first_div}; decision agreement {dec_agree:.3f}; "
          f"publish sequence identical: {pa == pb} ({len(pa)} vs {len(pb)} publishes)")
    sa = [r for r in ra if r["is_conflict_actor"] == "1" and r["age_at_use_ms"]]
    sb = [r for r in rb if r["is_conflict_actor"] == "1" and r["age_at_use_ms"]]
    aa = sorted(float(r["age_at_use_ms"]) for r in sa); ab = sorted(float(r["age_at_use_ms"]) for r in sb)
    if aa and ab:
        print(f"conflict-actor age p50 {aa[len(aa)//2]:.0f} / {ab[len(ab)//2]:.0f} ms, p99 {aa[int(0.99*(len(aa)-1))]:.0f} / {ab[int(0.99*(len(ab)-1))]:.0f} ms")


if __name__ == "__main__":
    main()
