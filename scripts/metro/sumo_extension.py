#!/usr/bin/env python3
"""Step 3 of the metropolitan replay: SUMO traffic on the real Athens network
cropped to the window that gives ~100 locales at 300 m, with demand scaled to
the pNEUMA-measured density, exported as 0.2 s floating car data for
metro_replay.py. Labelled calibrated extrapolation everywhere it is reported.

Usage: sumo_extension.py --osm athens_ext.osm --window_km 2.8 --target_active N
       [--sim_s 900] [--seed 1] [--out DIR]
 target_active = median instantaneous active tracks wanted (the figure's x);
 the departure period is set from a short calibration run so the steady-state
 active count lands on the target (reported, never assumed).
Env: metro_env with SUMO_HOME (netconvert, randomTrips.py, duarouter, sumo).
"""
import argparse, os, subprocess, sys, time, json, math
import xml.etree.ElementTree as ET
import numpy as np, sumolib
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pneuma_survey as ps
SUMO_HOME = os.environ["SUMO_HOME"]; TOOLS = os.path.join(SUMO_HOME, "tools")

def sh(cmd, log):
    with open(log, "a") as lg:
        lg.write("$ " + " ".join(cmd) + "\n"); lg.flush()
        subprocess.run(cmd, check=True, stdout=lg, stderr=subprocess.STDOUT)

def crop_net(osm, full_net, window_km, out_net, log):
    """Crop the full network to a square window (net coordinates) centred on
    its junction anchors' centroid; keeps the edges inside the boundary."""
    net = sumolib.net.readNet(full_net, withInternal=False); P = ps.partition(net)
    xs = np.array([v[0] for v in P["anchors"].values()]); ys = np.array([v[1] for v in P["anchors"].values()])
    cx, cy, h = xs.mean(), ys.mean(), window_km * 500.0
    bnd = f"{cx-h:.1f},{cy-h:.1f},{cx+h:.1f},{cy+h:.1f}"
    sh(["netconvert", "--osm-files", osm, "-o", out_net, "--geometry.remove", "--junctions.join", "--junctions.join-dist", "20",
        "--roundabouts.guess", "--ramps.guess", "--remove-edges.isolated", "--keep-edges.by-vclass", "passenger", "--no-turnarounds",
        "--proj.utm", "--keep-edges.in-boundary", bnd], log)
    return bnd

def active_count(fcd, cycle=0.2):
    per = []
    for ev, el in ET.iterparse(fcd, events=("end",)):
        if el.tag == "timestep": per.append(len(el.findall("vehicle"))); el.clear()
    a = np.array(per[len(per) // 3:]) if len(per) > 3 else np.array(per)   # after warm-up
    return float(np.median(a)) if len(a) else 0.0, float(np.percentile(a, 95)) if len(a) else 0.0

def run_sumo(net, trips_period, sim_s, seed, out_prefix, log, fcd_period=0.2):
    rou = out_prefix + ".rou.xml"; trips = out_prefix + ".trips.xml"; fcd = out_prefix + ".fcd.xml"
    sh([sys.executable, os.path.join(TOOLS, "randomTrips.py"), "-n", net, "-o", trips, "-r", rou, "-e", str(sim_s), "-p", f"{trips_period:.4f}",
        "--seed", str(seed), "--vehicle-class", "passenger", "--validate", "--fringe-factor", "5", "--min-distance", "300"], log)
    sh(["sumo", "-n", net, "-r", rou, "-b", "0", "-e", str(sim_s), "--step-length", "0.2", "--fcd-output", fcd, "--device.fcd.period", str(fcd_period),
        "--seed", str(seed), "--no-step-log", "--no-warnings", "--time-to-teleport", "120", "--ignore-junction-blocker", "30"], log)
    return fcd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--osm", required=True); ap.add_argument("--full_net", required=True)
    ap.add_argument("--window_km", type=float, default=2.8); ap.add_argument("--target_active", type=float, required=True)
    ap.add_argument("--sim_s", type=int, default=900); ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", default="ext"); ap.add_argument("--calib_s", type=int, default=300)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); log = os.path.join(a.out, "sumo_extension.log"); t0 = time.time()
    net_path = os.path.join(a.out, f"athens_w{a.window_km:g}.net.xml")
    if not os.path.exists(net_path): bnd = crop_net(a.osm, a.full_net, a.window_km, net_path, log)
    net = sumolib.net.readNet(net_path, withInternal=False); P = ps.partition(net)
    grp = ps.tile_anchors(P["anchors"], 300); L = len(set(grp.values()))
    print(f"cropped net: anchors {len(P['anchors'])} roads {len(P['roads'])} lane_km {P['lane_km']:.1f} L(300 m)={L} ({time.time()-t0:.0f}s)", flush=True)
    # calibration: one short run at a guessed period, then scale (active ~ 1/period)
    period0 = 1.0
    fcd0 = run_sumo(net_path, period0, a.calib_s, a.seed, os.path.join(a.out, f"calib_s{a.seed}"), log)
    med0, p95_0 = active_count(fcd0)
    period = period0 * med0 / max(a.target_active, 1.0)
    print(f"calibration: period {period0:.2f} s -> active median {med0:.0f}; period for target {a.target_active:.0f} = {period:.3f} s ({time.time()-t0:.0f}s)", flush=True)
    fcd = run_sumo(net_path, period, a.sim_s, a.seed, os.path.join(a.out, f"ext_L{L}_t{int(a.target_active)}_s{a.seed}"), log)
    med, p95 = active_count(fcd)
    meta = {"net": os.path.basename(net_path), "window_km": a.window_km, "L_300m": L, "lane_km": P["lane_km"], "target_active": a.target_active,
            "period_s": period, "active_median": med, "active_p95": p95, "vehicles_per_lane_km": med / P["lane_km"], "sim_s": a.sim_s, "seed": a.seed,
            "fcd": os.path.basename(fcd), "label": "calibrated extrapolation (SUMO demand on the real Athens network)"}
    json.dump(meta, open(fcd[:-8] + ".meta.json", "w"), indent=1)
    print(f"extension trace {os.path.basename(fcd)}: active median {med:.0f} p95 {p95:.0f} (target {a.target_active:.0f}), veh/lane-km {med/P['lane_km']:.2f} ({time.time()-t0:.0f}s)")

if __name__ == "__main__":
    main()
