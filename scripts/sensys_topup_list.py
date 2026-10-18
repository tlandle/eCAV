#!/usr/bin/env python3
"""Emit the top-up commands for a junction stage from campaign_check output.

    python scripts/sensys_analysis/campaign_check.py <stamp> | \\
        python scripts/sensys_topup_list.py --stamp <stamp> --speeds 50 63 88

A V2X cell whose point estimate lies in [--lo, --hi] (default 0.2 to 0.9)
with fewer than --reps seeds (default 20) gets one test_runner command with
--repetitions <reps> under the same stamp (the runner's resume skips the
seeds already recorded). Oracle cells are deterministic and never topped
up. The crossing trigger map is the geometry's TTA map (logs/tta_maps.env
values are the defaults here); the planner and edge knobs are the campaign's.
"""
import argparse
import re
import sys

ROW = re.compile(r"^(?P<geom>\S+)\s+(?P<spd>\d+)\s+(?P<mgr>\S+)\s+(?P<anch>on|off|default)\s+N=(?P<N>\d+)\s+"
                 r"(?P<lat>\d+) ms\s+(?P<kv>\{[^}]*\})?\s*n=(?P<n>\d+)\s+P=(?P<p>[\d.]+)")
TTA = {"openscenario_3_edge_late_fusion": "38:5.25,50:5.0,53:5.0,63:5.5,70:7.0,88:9.0",
       "scp_edge_late_fusion": "38:5.55,50:5.4,53:5.3,63:6.5,70:8.0,88:10.17"}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stamp", required=True)
    ap.add_argument("--speeds", nargs="+", required=True)
    ap.add_argument("--lo", type=float, default=0.2)
    ap.add_argument("--hi", type=float, default=0.9)
    ap.add_argument("--reps", type=int, default=20)
    ap.add_argument("--port", default="2000")
    ap.add_argument("--py", default="$PY")
    a = ap.parse_args(argv)
    cells = []
    for line in sys.stdin:
        m = ROW.match(line.strip())
        if not m:
            continue
        d = m.groupdict()
        if d["geom"] not in TTA or d["spd"] not in a.speeds or d["mgr"] == "oracle":
            continue
        p, n = float(d["p"]), int(d["n"])
        if n >= a.reps or not (a.lo <= p <= a.hi):
            continue
        if d["N"] != "1" or (d["kv"] and "prediction_steps" not in d["kv"]):
            continue
        cells.append(d)
    for d in cells:
        anch = "" if d["anch"] == "default" else f" --anchoring {d['anch']}"
        print(f"{a.py} test_runner.py -t {d['geom']} --manager-types {d['mgr']}{anch} --latencies {d['lat']} "
              f"--speeds {d['spd']} --repetitions {a.reps} --behavior-kv curv_lat_accel=8 --edge-kv prediction_steps=60 "
              f"--latency-distribution hybrid --self-id-radius 1.0 --per-tick-age-log --carla-port {a.port} "
              f"--stamp {a.stamp} --scenario-env CROSS_TRIGGER_TTA={TTA[d['geom']]}")
    print(f"# {len(cells)} cells in [{a.lo}, {a.hi}] with n < {a.reps}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
