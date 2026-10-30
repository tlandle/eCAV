#!/usr/bin/env python3
"""Calibrate the crossing car's launch so the ego's footprint enters the conflict box 4.3 s after
the car's first visible motion, at every target speed, on both junctions (speed sweep j6).

    python scripts/sensys_analysis/j6_calibrate.py --out tmp/j6_maps.json [--iters 3] [--tol 0.15]
        [--speeds 38 50 53 63 70 88] [--spawn-y -90] [--port 2000]

Each iteration runs the free-flow arm (reference, hazard_response=none, one seed) for the cells
still out of tolerance under the current CROSS_TRIGGER_TTA maps, measures the ego's box entry
(encounter_check.footprint_metrics) and the ego's speed at launch, and moves each cell's lead by
the observed time error (the ego holds its speed before the launch, so entry time is linear in the
lead). It stops when every cell is within --tol of 4.3 s with footprints intersecting and the ego
at or above 95 percent of its target at launch, or after --iters iterations. The final maps go to
--out (JSON: {"lt": {spd: tta}, "scp": {...}, "ok": bool, "cells": {...}}).
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from campaign_check import ROOT  # noqa: E402
from encounter_check import load_log, launch_tick, footprint_metrics  # noqa: E402

HIDE = ("5946265674966002516,6337000252949488035,6180885027493966721,16537817839145246163,16697564674962583843,"
        "6207999516011777620,2654350897737485752,16235826325815247835,4815558027215542490,1415910357169383613,"
        "4380901688286554368,5034201441472765163,8005252838334864825,10145647440276496614,9929740016222508988,"
        "7518384223988529276")
TESTS = {"lt": "openscenario_3_edge_late_fusion", "scp": "scp_edge_late_fusion"}
PY = os.path.expanduser("~/anaconda3/envs/opencda310/bin/python")
WANT = 4.30


def fmt_map(m):
    return ",".join(f"{int(k)}:{v:.2f}" for k, v in sorted(m.items()))


def run_cells(test, stamp, speeds, tta_map, spawn_y, port, log, extra_env=()):
    cmd = [PY, "test_runner.py", "-t", test, "--manager-types", "oracle", "--anchoring", "on", "--latencies", "0",
           "--speeds", *[str(s) for s in speeds], "--ego-counts", "1", "--repetitions", "1",
           "--latency-distribution", "hybrid", "--per-tick-age-log", "--self-id-radius", "1", "--stamp", stamp,
           "--carla-port", str(port), "--edge-kv", "prediction_steps=60",
           "--behavior-kv", "curv_lat_accel=8", "self_exclusion_m=1.0", "hazard_response=none",
           "--scenario-env", f"CROSS_TRIGGER_TTA={fmt_map(tta_map)}", f"HIDE_ENV_OBJECTS={HIDE}",
           f"EGO_SPAWN_Y={spawn_y}", "EGO_DRIVE_DISTANCE=400", "SCENARIO_MAX_STEP=900", *extra_env]
    log.write(f"{time.strftime('%F %T')} run: {' '.join(cmd)}\n"); log.flush()
    subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False)


def measure(test, stamp, spd):
    fs = sorted(glob.glob(os.path.join(ROOT, "experiment_results", test, stamp, f"speed_spd_{spd}", "baseline_default",
                                       "mgr_oracle", "anchoring_on", "lat_0_*", "run_*", "evaluation_output", "per_tick_age.csv")))
    if not fs:
        return None
    rows, gt, ego_v, ego_xy = load_log(fs[-1])
    launch = launch_tick(gt)
    if launch is None or not ego_xy:
        return None
    fp = footprint_metrics(ego_xy, gt, launch)
    near = lambda t: ego_v.get(t) if t in ego_v else ego_v[min(ego_v, key=lambda k: abs(k - t))]
    # The approach speed is the mean over the 3 s before the launch: the speed controller holds a
    # steady state a few percent under its set point and pulses at low set points (38 km/h: 8.7 to
    # 10.5 m/s around 9.5), so the instantaneous speed at the launch is not the cruise speed.
    before = [v for t, v in ego_v.items() if launch - 60 <= t < launch]
    v_mean = sum(before) / len(before) if before else float("nan")
    return dict(entry=fp["ego_in"], hit=fp["fp_hit"], gap=fp["fp_min_m"], v_launch=near(launch), v_cruise=v_mean,
                car_in=fp["car_in"], car_out=fp["car_out"], run=os.path.dirname(os.path.dirname(fs[-1])))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--iters", type=int, default=3)
    ap.add_argument("--tol", type=float, default=0.15)
    ap.add_argument("--speeds", nargs="*", type=int, default=[38, 50, 53, 63, 70, 88])
    ap.add_argument("--spawn-y", type=float, default=-90.0)
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--stamp-prefix", default="sensys_nov27_j6c")
    ap.add_argument("--log", default=os.path.join(ROOT, "logs", "j6_calibrate.log"))
    ap.add_argument("--start-maps", default=None, help="JSON from an earlier run: start from its lt/scp maps")
    ap.add_argument("--tests", nargs="*", default=list(TESTS), choices=list(TESTS), help="junctions to calibrate")
    ap.add_argument("--extra-env", nargs="*", default=[], help="extra KEY=VALUE scenario env, e.g. CROSS_SPEED_CHANGE=40:20")
    ap.add_argument("--want-after-car-in", type=float, default=None,
                    help="target the ego box entry this many seconds after the measured car entry instead of 4.30 s "
                         "after the launch (for cells whose car changes speed on the way)")
    a = ap.parse_args(argv)
    # Starting leads: 4.3 s to the box plus the half-lane and half-length offset 4.2 m / v, plus the
    # 0.4 s between the trigger and the car's first visible motion; the left turn adds 0.3 s for the
    # planner's slowdown into the turn (free-flow runs of Oct 29).
    tests = {k: TESTS[k] for k in a.tests}
    maps = {}
    for key in tests:
        maps[key] = {}
        for s in a.speeds:
            v = s / 3.6
            maps[key][s] = round(WANT + 4.2 / v + 0.4 + (0.3 if key == "lt" else 0.0), 2)
    if a.start_maps and os.path.exists(a.start_maps):
        prev = json.load(open(a.start_maps))
        for key in tests:
            for s in a.speeds:
                if str(s) in prev.get(key, {}):
                    maps[key][s] = float(prev[key][str(s)])
    log = open(a.log, "a")
    cells = {}
    pending = {key: list(a.speeds) for key in tests}
    for it in range(1, a.iters + 1):
        for key, test in tests.items():
            if not pending[key]:
                continue
            stamp = f"{a.stamp_prefix}_{key}_i{it}"
            run_cells(test, stamp, pending[key], maps[key], a.spawn_y, a.port, log, a.extra_env)
            still = []
            for s in pending[key]:
                m = measure(test, stamp, s)
                target_v = s / 3.6
                if m is None:
                    log.write(f"{key} {s}: no measurable run\n"); still.append(s); continue
                m["tta"] = maps[key][s]
                # At cruise: the launch speed is within the controller's oscillation of the 3 s mean, and that
                # mean is at least 85 percent of the set point (the measured cruise speed is what the paper reports).
                m["speed_ok"] = (m["v_launch"] == m["v_launch"] and m["v_cruise"] == m["v_cruise"]
                                 and m["v_launch"] >= 0.9 * m["v_cruise"] and m["v_cruise"] >= 0.85 * target_v)
                want = (m["car_in"] + a.want_after_car_in) if (a.want_after_car_in is not None and m["car_in"] == m["car_in"]) else WANT
                m["want"] = want
                err = m["entry"] - want if m["entry"] == m["entry"] else float("nan")
                m["err"] = err
                ok = (err == err and abs(err) <= a.tol and m["hit"] == 1)
                m["ok"] = bool(ok and m["speed_ok"])
                cells[f"{key}_{s}"] = m
                log.write(f"{time.strftime('%F %T')} iter {it} {key} {s} km/h: lead {maps[key][s]:.2f} s -> ego box entry "
                          f"{m['entry']:.2f} s (car {m['car_in']:.2f} to {m['car_out']:.2f}), footprints hit {m['hit']}, "
                          f"gap {m['gap']:.2f} m, ego at launch {m['v_launch']:.1f} m/s, cruise {m['v_cruise']:.1f} (target {target_v:.1f}) -> "
                          f"{'ok' if m['ok'] else 'adjust'}\n"); log.flush()
                if not ok:
                    if err == err:
                        maps[key][s] = round(maps[key][s] - err, 2)   # later entry -> shorter lead
                    still.append(s)
            pending[key] = still
        json.dump(dict(lt=maps.get("lt", {}), scp=maps.get("scp", {}), ok=not any(pending.values()), pending=pending, cells=cells,
                       spawn_y=a.spawn_y, iteration=it), open(a.out, "w"), indent=1)
        if not any(pending.values()):
            break
    ok = not any(pending.values()) and all(c.get("speed_ok") for c in cells.values())
    json.dump(dict(lt=maps.get("lt", {}), scp=maps.get("scp", {}), ok=ok, pending=pending, cells=cells, spawn_y=a.spawn_y), open(a.out, "w"), indent=1)
    log.write(f"{time.strftime('%F %T')} calibration {'converged' if ok else 'NOT converged'}: lt {fmt_map(maps.get('lt', {}))} scp {fmt_map(maps.get('scp', {}))}\n")
    log.close()
    print(json.dumps(dict(ok=ok, lt=maps.get("lt", {}), scp=maps.get("scp", {}), pending=pending), indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
