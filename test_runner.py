#!/usr/bin/env python3
"""
sweep_runner.py
---------------
Sweeps edge-latency and/or ego-vehicle max-speed.
Automatically restarts CARLA whenever it’s not running **and** force-kills /
restarts the server after every 5 completed runs to avoid drift.

Example:
    python sweep_runner.py -t openscenario_3_edge \
           --latencies 0 100 200 --speeds 50 70 90 --repetitions 3
"""

import argparse, copy, json, shutil, subprocess, sys, time, yaml, socket
from pathlib import Path
from datetime import datetime

# ───────────────────────── CLI ────────────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--scenario", required=True, help="YAML without .yaml")
ap.add_argument("--latencies", nargs="*", type=float, default=[],
                help="edge latency values (ms)")
ap.add_argument("--speeds",    nargs="*", type=int,   default=[],
                help="ego max_speed values (km/h)")
ap.add_argument("--repetitions", type=int, default=1)
args = ap.parse_args()


CARLA_BIN = "CarlaUE4-Linux-Shipping"     # running binary name
LAT_LIST = args.latencies or [None]   # keep original if empty
SPD_LIST = args.speeds    or [None]
TOTAL_RUNS = len(LAT_LIST) * len(SPD_LIST) * args.repetitions

# ───────────────────────── paths ──────────────────────────────────────────
ROOT       = Path(__file__).resolve().parent
CFG_DIR    = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE   = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK    = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP      = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT   = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE   = ROOT / "opencda/scenario_testing/evaluation_outputs"


print(f"\n▶  Results under: {EXP_ROOT}\n", flush=True)

# ───────────────── CARLA watchdog helpers ────────────────────────────────
CARLA_SH   = Path.home() / "carla-0.9.15/CarlaUE4.sh"
CARLA_PORT = 2000


# ─────────── on-coming speed helper (same mapping as Scenario_3) ──────────
def oncoming_speed_for(ego_kmh: int) -> float:
    """Piece-wise linear mapping from ego→on-coming speed (km/h)."""
    x = [20, 25.0, 30.0, 40.0, 50.0, 70.0, 100.0]     # ego speed anchors
    y = [4.5, 5.0, 5.0, 10, 13.0, 25.0,  52.0]     # desired on-coming speed

    if ego_kmh <= x[0]:                               # below first anchor
        m = (y[1]-y[0]) / (x[1]-x[0]); return y[0] + m*(ego_kmh-x[0])
    if ego_kmh >= x[-1]:                              # above last anchor
        m = (y[-1]-y[-2]) / (x[-1]-x[-2]); return y[-1] + m*(ego_kmh-x[-1])
    for i in range(1, len(x)):                        # inside the range
        if ego_kmh <= x[i]:
            m = (y[i]-y[i-1]) / (x[i]-x[i-1])
            return y[i-1] + m*(ego_kmh-x[i-1])

def carla_running() -> bool:
    """True if server proc exists AND port answers."""
    try:
        ok_proc = subprocess.run(["pgrep", "-f", "CarlaUE4.sh"],
                                 stdout=subprocess.DEVNULL).returncode == 0
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            ok_port = s.connect_ex(("127.0.0.1", CARLA_PORT)) == 0
        return ok_proc and ok_port
    except Exception:
        return False

def start_carla():
    print("   ↻  (re)starting CARLA …", flush=True)
    subprocess.Popen([str(CARLA_SH)],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 30:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", CARLA_PORT)) == 0:
                print("      CARLA ready ✓", flush=True)
                return
        time.sleep(1)
    raise RuntimeError("CARLA failed to open port 2000 within 30 s")

def kill_carla():
    """Kill CarlaUE4.sh and wait for port 2000 to close."""
    print("   ✗  stopping CARLA …", flush=True)
    subprocess.run(["pkill", "-f", CARLA_BIN],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 15:
        subprocess.run(["pkill", "-f", "CarlaUE4-Linux-Shipping"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", CARLA_PORT)) != 0:
                print("      CARLA stopped ✓", flush=True)
                return
        time.sleep(1)
    print("      (timeout – continuing anyway)", flush=True)

# ───────────────────── YAML backup & load ────────────────────────────────
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

# ───── detect whether this YAML actually contains an edge section ─────
EDGE_PRESENT = bool(ORIGINAL_CFG.get("scenario", {})
                                   .get("edge_list", []))   # True ⇢ at least one edge


def patch_yaml(d, latency_ms, speed_kmh):
    global EDGE_PRESENT
    if EDGE_PRESENT and latency_ms is not None:
        print("   ↳  setting edge latency to", latency_ms/1000, "s")
        d["edge_base"]["latency"] = latency_ms / 1000.0
        for edge in d["scenario"]["edge_list"]:
            edge["latency"] = latency_ms / 1000.0


    # ---------------------- ego / on-coming speed ----------------------
    if speed_kmh is not None:
        # ---- OpenCDA ego vehicle ----
        if "edge_list" in d.get("scenario", {}):          # old “edge” YAMLs
            beh = (d["scenario"]["edge_list"][0]
                     .setdefault("vehicles", [{}])[0]
                     .setdefault("behavior", {}))
        else:                                             # single_cav_list YAMLs
            beh = (d["scenario"]["single_cav_list"][0]
                     .setdefault("behavior", {}))
        beh["max_speed"] = int(speed_kmh)

        # ---- ScenarioRunner parameters ----
        oc_speed = oncoming_speed_for(speed_kmh)
        d.setdefault("scenario_runner", {})["openscenarioparams"] = [
            f"ego_vehicle_max_speed={int(speed_kmh)}",
            f"oncoming_vehicle_speed={int(oc_speed)}"
        ]

# ───────────────────── single run helper ─────────────────────────────────
def run_once(lat_ms, spd_kmh, rep):
    run_dir = EXP_ROOT / f"speed_{spd_kmh if spd_kmh is not None else 'orig'}" \
                       / f"lat_{lat_ms if lat_ms is not None else 'orig'}" \
                       / f"run_{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)

    eval_dir = OUT_BASE / STAMP / f"spd_{spd_kmh}_lat_{lat_ms}_rep_{rep}"
    eval_dir.mkdir(parents=True, exist_ok=True)

    cmd = [sys.executable, "opencda.py",
           "-t", args.scenario,
           "--apply_ml",
           "--output_dir", str(eval_dir)]

    result = subprocess.run(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,
                                 text=True, encoding="utf-8", errors="replace")

    (run_dir / "runner_stdout.txt").write_text(
        f"CMD: {' '.join(cmd)}\nRET: {result.returncode}\n\n"
        f"=== STDOUT ===\n{result.stdout}\n=== STDERR ===\n{result.stderr}\n")

    try:
        shutil.copytree(eval_dir, run_dir / "evaluation_output")
        metrics = json.load((eval_dir / "simulation_metrics.json").open())
    except Exception as e:
        metrics = {"success_rate": 0.0, "error": str(e)}

    if lat_ms is not None: metrics["edge_latency_s"] = lat_ms / 1000.0
    if spd_kmh is not None:
        metrics["target_speed_kmh"]   = spd_kmh
        metrics["oncoming_speed_kmh"] = int(oncoming_speed_for(spd_kmh))
    (run_dir / "simulation_metrics.json").write_text(json.dumps(metrics, indent=2))

# ───────────────────── sweep loop ────────────────────────────────────────
try:
    run_idx = 0
    for lat in LAT_LIST:
        for spd in SPD_LIST:
            for rep in range(1, args.repetitions + 1):
                run_idx += 1
                print(f"→ Run {run_idx}/{TOTAL_RUNS}: lat={lat} spd={spd} rep={rep}", flush=True)

                if not carla_running():
                    start_carla()

                patched = copy.deepcopy(ORIGINAL_CFG)
                patch_yaml(patched, lat, spd)
                with CFG_FILE.open("w") as f: yaml.safe_dump(patched, f)

                run_once(lat, spd, rep)

                # restart if server crashed mid-run
                if not carla_running():
                    start_carla()

                # force restart every 5 runs (except after final run)
                if run_idx < TOTAL_RUNS:
                    kill_carla()
                    start_carla()

except Exception as e:
    print("\nSweep aborted:", e, flush=True)
finally:
    CFG_BAK.replace(CFG_FILE)
    print("\nYAML restored. Sweep completed.", flush=True)

