#!/usr/bin/env python3
"""
DEBUG version: captures stdout/stderr of each opencda.py run,
keeps going on every error, and logs everything into
experiment_results/<scenario>/<timestamp>/latency_<v>/run_<rep>/.
"""
import argparse, subprocess, yaml, shutil, json, time, copy, sys
from pathlib import Path
from datetime import datetime

# ---------------- CLI ---------------- #
cli = argparse.ArgumentParser()
cli.add_argument("--scenario", "-t", required=True)
cli.add_argument("--values",   nargs="+", type=float, required=True)
cli.add_argument("--repetitions", type=int, default=1)
args = cli.parse_args()

# ---------------- paths ---------------- #
ROOT       = Path(__file__).resolve().parent
CFG_DIR    = ROOT / "opencda/scenario_testing/config_yaml"
CFG_FILE   = CFG_DIR / f"{args.scenario}.yaml"
CFG_BAK    = CFG_DIR / f"{args.scenario}.yaml.bak"

STAMP      = datetime.now().strftime("%Y%m%d_%H%M%S")
EXP_ROOT   = ROOT / "experiment_results" / args.scenario / STAMP
OUT_BASE   = ROOT / "opencda/scenario_testing/evaluation_outputs"

print(f"⇒ Writing all runs beneath {EXP_ROOT}", flush=True)

# ---------------- backup yaml ---------------- #
shutil.copy(CFG_FILE, CFG_BAK)
with CFG_BAK.open() as f:
    ORIGINAL_CFG = yaml.safe_load(f)

try:
    run_count = 0
    for latency in args.values:
        for rep in range(1, args.repetitions + 1):
            run_count += 1
            print(f"\n▶ Run {run_count}: latency={latency}  rep={rep}", flush=True)

            # ---- patch yaml ---- #
            patched = copy.deepcopy(ORIGINAL_CFG)
            patched["edge_base"]["latency"] = latency
            with CFG_FILE.open("w") as f:
                yaml.safe_dump(patched, f)

            # ---- pre-create target dir ---- #
            run_dir = EXP_ROOT / f"latency_{latency}" / f"run_{rep}"
            run_dir.mkdir(parents=True, exist_ok=True)

            # ---- launch opencda ---- #
            cmd = [
                sys.executable, "opencda.py",
                "-t", args.scenario,
                "--apply_ml"
            ]
            before = set(OUT_BASE.iterdir()) if OUT_BASE.exists() else set()
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # write the raw stdio no matter what
            (run_dir / "runner_stdout.txt").write_text(
                f"CMD: {' '.join(cmd)}\n"
                f"RET: {result.returncode}\n\n"
                f"=== STDOUT ===\n{result.stdout}\n"
                f"=== STDERR ===\n{result.stderr}\n"
            )

            # ---- copy evaluation folder (if any) ---- #
            try:
                after  = set(OUT_BASE.iterdir())
                newdir = max(after - before, key=lambda p: p.stat().st_mtime)
                # copytree into run_dir/evaluation_output/
                shutil.copytree(newdir, run_dir / "evaluation_output")
                # tag latency inside metrics.json
                mfile = run_dir / "evaluation_output" / "simulation_metrics.json"
                if mfile.exists():
                    data = json.load(mfile.open())
                    data["edge_latency_s"] = latency
                    with mfile.open("w") as jf:
                        json.dump(data, jf, indent=2)
            except Exception as e:
                # still want a stub metrics.json so post-proc never fails
                stub = {"edge_latency_s": latency,
                        "success_rate": 0.0,
                        "error": f"no evaluation folder: {e}"}
                json.dump(stub, (run_dir / "simulation_metrics.json").open("w"), indent=2)

except Exception as fatal:
    print("\n✖ FATAL error in runner:", fatal, flush=True)
finally:
    CFG_BAK.replace(CFG_FILE)
    print("\n✔ Restored original YAML — runner finished.", flush=True)
