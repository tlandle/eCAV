
#!/usr/bin/env python3
"""Run a parameter‑sweep experiment and archive results hierarchically.

Directory layout created:

experiment_results/
└─ <scenario>/
   └─ <YYYYMMDD_HHMMSS>/            # timestamp when this runner was launched
      └─ latency_<value>/           # the swept parameter value
         └─ run_<rep>/              # repetition index (1‑based)
            └─ simulation_metrics.json (copied and tagged)

The script keeps the original YAML config backed up and restores it after the
sweep.  It never crashes on a failed child process.
"""

import argparse
import copy
import json
import shutil
import subprocess
import time
from pathlib import Path

import yaml


# ──────────────────────────────── CLI args ──────────────────────────────── #

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sweep edge‑latency values for a scenario")
    p.add_argument("--scenario", "-t", required=True,
                   help="Scenario name (matches both ‑t and <scenario>.yaml)")
    p.add_argument("--values", nargs="+", type=float, required=True,
                   help="Latency values (in seconds) to test, e.g. 0.05 0.1 0.2")
    p.add_argument("--repetitions", type=int, default=1,
                   help="How many times to repeat each latency value")
    return p.parse_args()


# ─────────────────────────────── main runner ────────────────────────────── #

def main() -> None:
    args = parse_args()

    repo_root   = Path(__file__).parent.resolve()
    cfg_dir     = repo_root / "opencda/scenario_testing/config_yaml"
    cfg_path    = cfg_dir / f"{args.scenario}.yaml"
    backup_path = cfg_dir / f"{args.scenario}.yaml.bak"

    out_base    = repo_root / "opencda/scenario_testing/evaluation_outputs"

    timestamp   = time.strftime("%Y%m%d_%H%M%S")  # runner start time
    exp_root    = repo_root / "experiment_results" / args.scenario / timestamp

    # ── backup original YAML ── #
    shutil.copy(cfg_path, backup_path)
    with open(backup_path, "r", encoding="utf-8") as bf:
        orig_cfg = yaml.safe_load(bf)

    try:
        for latency in args.values:
            for rep in range(1, args.repetitions + 1):
                # 1) patch YAML
                cfg = copy.deepcopy(orig_cfg)
                cfg["edge_base"]["latency"] = latency
                with open(cfg_path, "w", encoding="utf-8") as f:
                    yaml.safe_dump(cfg, f)

                # 2) snapshot existing output dirs
                out_base.mkdir(parents=True, exist_ok=True)
                before = set(out_base.iterdir())

                # 3) launch scenario
                cmd = ["python", "opencda.py", "-t", args.scenario, "--apply_ml"]
                print(f"→ [{time.strftime('%H:%M:%S')}] {args.scenario}  latency={latency}s  rep={rep}")
                result = subprocess.run(cmd, check=False)
                if result.returncode != 0:
                    print(f"⚠️  Child exited with code {result.returncode}")

                # 4) locate new evaluation folder
                after    = set(out_base.iterdir())
                new_dirs = after - before
                if not new_dirs:
                    print("⚠️  No new evaluation folder detected — skipping copy")
                    continue
                run_folder = max(new_dirs, key=lambda d: d.stat().st_mtime)

                # 5) tag metrics and copy to hierarchy
                metrics_file = run_folder / "simulation_metrics.json"
                if not metrics_file.exists():
                    print(f"⚠️  {metrics_file.name} missing in {run_folder.name}")
                    continue

                # inject latency value
                data = json.load(metrics_file.open())
                data["edge_latency_s"] = latency
                with open(metrics_file, "w", encoding="utf-8") as jf:
                    json.dump(data, jf, indent=2)

                # make hierarchical destination
                dest_dir = exp_root / f"latency_{latency}" / f"run_{rep}"
                dest_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy(metrics_file, dest_dir / metrics_file.name)
                print(f"   • saved → {dest_dir.relative_to(repo_root)}")

    finally:
        # ── restore YAML ── #
        backup_path.replace(cfg_path)
        print("\n✅ Restored original YAML and finished sweep.")


if __name__ == "__main__":
    main()
