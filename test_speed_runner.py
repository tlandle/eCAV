#!/usr/bin/env python3
"""
sweep_speed_runner.py
---------------------
Sweep the ego vehicle's max_speed (km/h) in the OpenCDA YAML and execute
opencda.py for every speed value and repetition.

Run example
-----------
python sweep_speed_runner.py \
       -t openscenario_3_edge \
       --speeds 20 30 40 50 60 \
       --repetitions 3
"""

import argparse, copy, json, shutil, subprocess, sys, yaml
from pathlib import Path
from datetime import datetime

# ───────────────────────── CLI ───────────────────────────────────────────
cli = argparse.ArgumentParser()
cli.add_argument("-t", "--scenario", required=True,
                 help="scenario YAML name (without .yaml)")
cli.add_argument("--speeds", nargs="+", type=int, required=True,
                 help="max_speed values (km/h) to sweep")
cli.add_argument("--repetitions", type=int, default=1,
                 help="how many repetitions per speed")
args = cli.parse_args()

# ───────────────────────── paths ─────────────────────────────────────────
ROOT      = Path(__file__).resolve().parent
CFG_DIR   = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE  = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK   = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP     = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT  = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE  = ROOT / "opencda/scenario_testing/evaluation_outputs"

print(f"\n▶  Writing all runs under: {EXP_ROOT}", flush=True)

# ───────────────────────── backup original YAML ─────────────────────────
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

def patch_speed(cfg_dict, new_speed):
    """
    Mutate nested dict in-place:
      scenario.edge_list[0].vehicles[0].behavior.max_speed = new_speed
    """
    cfg_dict["scenario"]["edge_list"][0]["vehicles"][0]["behavior"]["max_speed"] = int(new_speed)

# ───────────────────────── main sweep loop ───────────────────────────────
try:
    run_idx = 0
    for spd in args.speeds:
        for rep in range(1, args.repetitions + 1):
            run_idx += 1
            print(f"\n→ Run {run_idx}: target_speed={spd} km/h  rep={rep}", flush=True)

            # 1) patch YAML ----------------------------------------------------
            patched = copy.deepcopy(ORIGINAL_CFG)
            patch_speed(patched, spd)
            with CFG_FILE.open("w") as f:
                yaml.safe_dump(patched, f)

            # 2) run & output dirs --------------------------------------------
            run_dir  = EXP_ROOT / f"speed_{spd}" / f"run_{rep}"
            run_dir.mkdir(parents=True, exist_ok=True)

            eval_dir = OUT_BASE / STAMP / f"spd_{spd}_rep_{rep}"
            eval_dir.mkdir(parents=True, exist_ok=True)

            cmd = [sys.executable, "opencda.py",
                   "-t", args.scenario,
                   "--apply_ml",
                   "--output_dir", str(eval_dir)]

            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding="utf-8",
                errors="replace"
            )

            # 3) save runner stdio --------------------------------------------
            (run_dir / "runner_stdout.txt").write_text(
                f"CMD: {' '.join(cmd)}\n"
                f"RET: {result.returncode}\n\n"
                f"=== STDOUT ===\n{result.stdout}\n"
                f"=== STDERR ===\n{result.stderr}\n"
            )

            # 4) copy evaluation folder & metrics -----------------------------
            try:
                shutil.copytree(eval_dir, run_dir / "evaluation_output")
                metrics = json.load((eval_dir / "simulation_metrics.json").open())
            except Exception as e:
                metrics = {"success_rate": 0.0,
                           "error": f"evaluation missing: {e}"}

            metrics["target_speed_kmh"] = int(spd)
            (run_dir / "simulation_metrics.json").write_text(
                json.dumps(metrics, indent=2))

except Exception as fatal:
    print("\n✖ FATAL error in runner:", fatal, flush=True)

finally:
    CFG_BAK.replace(CFG_FILE)            # restore pristine YAML
    print("\n✔ Runner finished. YAML restored.", flush=True)
