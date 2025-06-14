#!/usr/bin/env python3
"""
sweep_runner.py
---------------
One runner for both sweeps:
    • ego max_speed  (scenario.edge_list[0].vehicles[0].behavior.max_speed)
    • edge latency   (edge_base.latency)

CARLA is (re)started automatically whenever it is not running.
"""

import argparse, copy, json, shutil, subprocess, sys, time, yaml, socket
from pathlib import Path
from datetime import datetime

# ───────────────────────────── CLI ───────────────────────────────────────
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--scenario", required=True, help="YAML without .yaml")
ap.add_argument("--latencies", nargs="*", type=float, default=[],
                help="edge latency values (s)")
ap.add_argument("--speeds",    nargs="*", type=int,   default=[],
                help="ego max_speed values (km/h)")
ap.add_argument("--repetitions", type=int, default=1)
args = ap.parse_args()

LAT_LIST = args.latencies or [None]   # keep original if list empty
SPD_LIST = args.speeds    or [None]

# ───────────────────────────── paths ─────────────────────────────────────
ROOT       = Path(__file__).resolve().parent
CFG_DIR    = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE   = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK    = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP      = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT   = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE   = ROOT / "opencda/scenario_testing/evaluation_outputs"

print(f"\n▶  Results under: {EXP_ROOT}\n", flush=True)

# ─────────────────────── CARLA watchdog helpers ─────────────────────────
CARLA_SH   = Path.home() / "carla-0.9.15/CarlaUE4.sh"
CARLA_PORT = 2000

def carla_running() -> bool:
    """True if CarlaUE4.sh process exists *and* port 2000 answers."""
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
    print("   ↻  (re)starting CARLA server …", flush=True)
    subprocess.Popen([str(CARLA_SH)],
                     stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL)
    t0 = time.time()
    while time.time() - t0 < 30:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            if s.connect_ex(("127.0.0.1", CARLA_PORT)) == 0:
                print("      CARLA ready ✓", flush=True)
                return
        time.sleep(1)
    raise RuntimeError("CARLA failed to open port 2000 within 30 s")

# ─────────────────────── YAML backup & load ─────────────────────────────
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

def patch_yaml(d, latency, speed):
    if latency is not None:
        print("   ↳  setting edge latency to", (float(latency) / 1000.0), "s")
        print("   ↳  previous edge latency was", d["edge_base"]["latency"], "s")
        d["edge_base"]["latency"] = float(latency) / 1000.0   # ms → s
        for edge in d["scenario"]["edge_list"]:
            edge["latency"] = float(latency) / 1000.0  # ms → s 
    if speed is not None:
        # set ego behaviour
        d["scenario"]["edge_list"][0]["vehicles"][0]["behavior"]["max_speed"] = int(speed)

        # pass the same value to ScenarioRunner
        d["scenario_runner"]["openscenarioparams"] = [
             f"ego_vehicle_max_speed={int(speed)}"
        ]

# ─────────────────────── single run helper ──────────────────────────────
def run_once(lat, spd, rep):
    run_dir = EXP_ROOT / f"speed_{spd if spd is not None else 'orig'}" \
                       / f"lat_{lat if lat is not None else 'orig'}" \
                       / f"run_{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)

    eval_dir = OUT_BASE / STAMP / f"spd_{spd}_lat_{lat}_rep_{rep}"
    eval_dir.mkdir(parents=True, exist_ok=True)

    cmd = [sys.executable, "opencda.py",
           "-t", args.scenario,
           "--apply_ml",
           "--output_dir", str(eval_dir)]

    result = subprocess.run(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,
                                 text=True,
                                 encoding="utf-8",
                                 errors="replace")

    (run_dir / "runner_stdout.txt").write_text(
        f"CMD: {' '.join(cmd)}\n"
        f"RET: {result.returncode}\n\n"
        f"=== STDOUT ===\n{result.stdout}\n"
        f"=== STDERR ===\n{result.stderr}\n")

    try:
        shutil.copytree(eval_dir, run_dir / "evaluation_output")
        metrics = json.load((eval_dir / "simulation_metrics.json").open())
    except Exception as e:
        metrics = {"success_rate": 0.0, "error": str(e)}

    if lat is not None:
        metrics["edge_latency_s"] = float(lat)
    if spd is not None:
        metrics["target_speed_kmh"] = int(spd)

    (run_dir / "simulation_metrics.json").write_text(
        json.dumps(metrics, indent=2))

# ─────────────────────── sweep loop ─────────────────────────────────────
try:
    run_idx = 0
    for lat in LAT_LIST:
        for spd in SPD_LIST:
            for rep in range(1, args.repetitions + 1):
                run_idx += 1
                print(f"→ Run {run_idx}: lat={lat} spd={spd} rep={rep}", flush=True)

                # ensure CARLA alive
                if not carla_running():
                    start_carla()

                patched = copy.deepcopy(ORIGINAL_CFG)
                patch_yaml(patched, lat, spd)
                with CFG_FILE.open("w") as f:
                    yaml.safe_dump(patched, f)

                run_once(lat, spd, rep)

                # restart if server crashed during run
                if not carla_running():
                    start_carla()

except Exception as e:
    print("\nsweep aborted:", e, flush=True)
finally:
    CFG_BAK.replace(CFG_FILE)
    print("\nYAML restored. Sweep completed.", flush=True)


