#!/usr/bin/env python3
"""
sweep_runner.py
---------------
Launches opencda.py multiple times, sweeping a latency parameter and capturing
all outputs in a fully deterministic folder hierarchy:

experiment_results/<scenario>/<timestamp>/latency_<v>/run_<rep>/

Each run directory contains:
    • runner_stdout.txt           (cmd, return-code, stdout, stderr)
    • evaluation_output/          (full folder from EvaluateManager)
    • simulation_metrics.json     (scenario-level metrics, always present)
"""

import argparse, subprocess, yaml, shutil, json, sys
import copy, os, time
from pathlib import Path
from datetime import datetime

# ────────────── CLI ─────────────────────────────────────────────────────
cli = argparse.ArgumentParser()
cli.add_argument("-t", "--scenario", required=True,
                 help="scenario name (YAML without extension)")
cli.add_argument("--values",   nargs="+", type=float, required=True,
                 help="latency values to sweep (s)")
cli.add_argument("--repetitions", type=int, default=1,
                 help="runs per latency value")
args = cli.parse_args()

# ────────────── paths & constants ───────────────────────────────────────
ROOT        = Path(__file__).resolve().parent
CFG_DIR     = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE    = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK     = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP       = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT    = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE    = ROOT / "opencda/scenario_testing/evaluation_outputs"

print(f"\n▶  Writing all runs under: {EXP_ROOT}", flush=True)

# ────────────── backup original YAML ────────────────────────────────────
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

try:
    run_idx = 0
    for latency in args.values:
        for rep in range(1, args.repetitions + 1):
            run_idx += 1
            print(f"\n→ Run {run_idx}: latency={latency}  rep={rep}", flush=True)

            # ---- 1. patch YAML in-place --------------------------------
            patched = copy.deepcopy(ORIGINAL_CFG)
            patched["edge_base"]["latency"] = float(latency)
            with CFG_FILE.open("w") as f:
                yaml.safe_dump(patched, f)

            # ---- 2. create runner output dir ---------------------------
            run_dir  = EXP_ROOT / f"latency_{latency}" / f"run_{rep}"
            run_dir.mkdir(parents=True, exist_ok=True)

            # dedicated evaluation dir for this run
            eval_dir = OUT_BASE / STAMP / f"lat_{latency}_rep_{rep}"
            eval_dir.mkdir(parents=True, exist_ok=True)

            # ---- 3. launch opencda.py ----------------------------------
            cmd = [sys.executable, "opencda.py",
                   "-t", args.scenario,
                   "--apply_ml",
                   "--output_dir", str(eval_dir)]

            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,              # keep text mode
                encoding="utf-8",
                errors="replace"        # <── any invalid byte → �
            )

            # ---- 4. save stdio -----------------------------------------
            (run_dir / "runner_stdout.txt").write_text(
                f"CMD: {' '.join(cmd)}\n"
                f"RET: {result.returncode}\n\n"
                f"=== STDOUT ===\n{result.stdout}\n"
                f"=== STDERR ===\n{result.stderr}\n")

            # ---- 5. copy evaluation output + metrics -------------------
            try:
                # copy full evaluation folder (may contain plots, logs…)
                shutil.copytree(eval_dir, run_dir / "evaluation_output")
                metrics_src = eval_dir / "simulation_metrics.json"
                metrics = json.load(metrics_src.open())
            except Exception as e:
                metrics = {"success_rate": 0.0,
                           "error": f"evaluation folder missing: {e}"}

            metrics["edge_latency_s"] = float(latency)
            (run_dir / "simulation_metrics.json").write_text(
                json.dumps(metrics, indent=2))

except Exception as fatal:
    print("\n FATAL error in runner:", fatal, flush=True)

finally:
    CFG_BAK.replace(CFG_FILE)                 # restore original YAML
    print("\n Runner finished. YAML restored.", flush=True)

