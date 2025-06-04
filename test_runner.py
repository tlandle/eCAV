#!/usr/bin/env python3
"""
test_runner.py
---------------
Sweep edge latency, ego max-speed, or both.  Keeps one YAML backup/restore
cycle.  Each run captures stdout/err, copies the evaluator’s folder, and
writes a patched simulation_metrics.json that always contains:

    "edge_latency_s": <float>,
    "target_speed_kmh": <int>

Usage examples
--------------
# latency sweep only
python sweep_runner.py -t openscenario_3_edge --latencies 0 50 100 --repetitions 5

# speed sweep only
python sweep_runner.py -t openscenario_3_edge --speeds 20 30 40 --repetitions 3

# combined sweep
python sweep_runner.py -t openscenario_3_edge \
                       --latencies 50 100 150 \
                       --speeds 30 40 \
                       --repetitions 2
"""

import argparse, copy, json, shutil, subprocess, sys, yaml
from pathlib import Path
from datetime import datetime

# ────────────── CLI ────────────────────────────────────────────────────
cli = argparse.ArgumentParser()
cli.add_argument("-t", "--scenario", required=True,
                 help="config YAML name (without .yaml)")
cli.add_argument("--latencies", nargs="*", type=float, default=[],
                 help="edge latency values (s) to sweep")
cli.add_argument("--speeds",    nargs="*", type=int,   default=[],
                 help="ego max_speed values (km/h) to sweep")
cli.add_argument("--repetitions", type=int, default=1)
args = cli.parse_args()

LAT_LIST = args.latencies or [None]   # if empty keep original
SPD_LIST = args.speeds    or [None]

# ────────────── paths ──────────────────────────────────────────────────
ROOT      = Path(__file__).resolve().parent
CFG_DIR   = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE  = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK   = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP     = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT  = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE  = ROOT / "opencda/scenario_testing/evaluation_outputs"

print(f"\n▶ Results under: {EXP_ROOT}", flush=True)

# ────────────── backup original YAML ───────────────────────────────────
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

def patch_yaml(cfg, latency, speed):
    if latency is not None:
        cfg["edge_base"]["latency"] = float(latency)
    if speed is not None:
        cfg["scenario"]["edge_list"][0]["vehicles"][0] \
           ["behavior"]["max_speed"] = int(speed)

def run_once(latency, speed, rep):
    run_dir = EXP_ROOT / f"speed_{speed if speed is not None else 'orig'}" \
                       / f"lat_{latency if latency is not None else 'orig'}" \
                       / f"run_{rep}"
    run_dir.mkdir(parents=True, exist_ok=True)

    eval_dir = OUT_BASE / STAMP / f"spd_{speed}_lat_{latency}_rep_{rep}"
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

    # stdout / stderr
    (run_dir / "runner_stdout.txt").write_text(
        f"CMD: {' '.join(cmd)}\n"
        f"RET: {result.returncode}\n\n"
        f"=== STDOUT ===\n{result.stdout}\n"
        f"=== STDERR ===\n{result.stderr}\n")

    # copy evaluation folder + metrics
    try:
        shutil.copytree(eval_dir, run_dir / "evaluation_output")
        metrics = json.load((eval_dir / "simulation_metrics.json").open())
    except Exception as e:
        metrics = {"success_rate": 0.0,
                   "error": f"evaluation missing: {e}"}

    if latency is not None:
        metrics["edge_latency_s"] = float(latency)
    if speed is not None:
        metrics["target_speed_kmh"] = int(speed)

    (run_dir / "simulation_metrics.json").write_text(
        json.dumps(metrics, indent=2))

# ────────────── sweep ───────────────────────────────────────────────────
try:
    run_idx = 0
    for lat in LAT_LIST:
        for spd in SPD_LIST:
            for rep in range(1, args.repetitions + 1):
                run_idx += 1
                print(f"→ Run {run_idx}: lat={lat}  spd={spd}  rep={rep}", flush=True)

                patched = copy.deepcopy(ORIGINAL_CFG)
                patch_yaml(patched, lat, spd)
                with CFG_FILE.open("w") as f:
                    yaml.safe_dump(patched, f)

                run_once(lat, spd, rep)

except Exception as fatal:
    print("\n✖ Runner aborted:", fatal, flush=True)

finally:
    CFG_BAK.replace(CFG_FILE)
    print("\n✔ YAML restored — sweep completed.", flush=True)
