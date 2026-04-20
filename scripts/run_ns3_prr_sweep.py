#!/usr/bin/env python3
"""
Run ns-3 C-V2X PRR sweep and invoke SB-SPS validation.

Executes Eckermann's v2x_communication_example for each (N, M) config,
parses delivery statistics from log_simtime_v2x.csv, and writes results
to data/ns3_prr_results.csv.  Then invokes validate_mac_vs_ns3.py.

Key ns-3 -> SB-SPS parameter mapping:
    numVeh           -> N  (number of senders)
    numSubchannel    -> M  (orthogonal resources per subframe)
    probResourceKeep -> p_keep (0.4)
    pRsvp=100        -> 100ms reservation period

Output format from ns-3 (log_simtime_v2x.csv):
    Simtime;TotalRx;TotalTx;PRR   (semicolon-separated)

Usage:
    python scripts/run_ns3_prr_sweep.py
    python scripts/run_ns3_prr_sweep.py --ns3-dir /path/to/ns-3_c-v2x
    python scripts/run_ns3_prr_sweep.py --dry-run
"""

import argparse
import csv
import math
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.join(SCRIPT_DIR, '..')
DEFAULT_NS3_DIR = '/home/atlas/ns3_cv2x/ns-3_c-v2x'
DEFAULT_OUTPUT = os.path.join(REPO_DIR, 'data', 'ns3_prr_results.csv')

N_VALUES = [2, 4, 8, 16, 32]
M_VALUES = [10, 20, 50]


def find_binary(ns3_dir):
    """Locate the v2x_communication_example binary and return (binary_path, lib_dir)."""
    # debug build
    for sub in ['build/scratch', 'build']:
        candidate = os.path.join(ns3_dir, sub, 'v2x_communication_example')
        if os.path.isfile(candidate):
            lib_dir = os.path.join(ns3_dir, 'build', 'lib')
            return candidate, lib_dir
    raise FileNotFoundError(
        f"v2x_communication_example not found in {ns3_dir}/build/. "
        f"Run: bash scripts/setup_ns3_cv2x.sh")


def run_ns3_simulation(binary, lib_dir, ns3_dir, N, M,
                       sim_time=60, p_keep=0.4):
    """Run one ns-3 simulation and return the final PRR."""
    cmd = [
        binary,
        f'--numVeh={N}',
        f'--numSubchannel={M}',
        f'--time={sim_time}',
        f'--probResourceKeep={p_keep}',
    ]

    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = lib_dir + ':' + env.get('LD_LIBRARY_PATH', '')

    print(f"  Running: N={N:>2}, M={M:>2} ... ", end='', flush=True)

    # ns-3 writes log_simtime_v2x.csv in CWD, so run from ns3_dir
    try:
        result = subprocess.run(
            cmd, cwd=ns3_dir, env=env,
            capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        print("TIMEOUT")
        return None
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        return None

    if result.returncode != 0:
        print(f"FAIL (exit {result.returncode})")
        stderr_lines = (result.stderr or '').strip().split('\n')
        for line in stderr_lines[-3:]:
            print(f"    {line}")
        return None

    # Parse output CSV written by the simulation
    prr = parse_ns3_output(ns3_dir)
    if prr is not None:
        print(f"PRR={prr:.4f}")
    else:
        print("PARSE_ERROR")
    return prr


def parse_ns3_output(ns3_dir):
    """Parse log_simtime_v2x.csv to extract the final steady-state PRR.

    Format: ``Simtime;TotalRx;TotalTx;PRR`` (semicolon-separated).
    Skips rows where PRR is -nan/nan (early seconds before any Tx).
    Returns the PRR from the last valid row.
    """
    csv_path = os.path.join(ns3_dir, 'log_simtime_v2x.csv')
    if not os.path.isfile(csv_path):
        return None

    try:
        with open(csv_path) as f:
            lines = f.readlines()
    except IOError:
        return None

    if len(lines) < 2:
        return None

    last_prr = None
    for line in lines[1:]:  # skip header
        parts = line.strip().split(';')
        if len(parts) < 4:
            continue
        try:
            prr_val = float(parts[3])
        except ValueError:
            continue
        if math.isnan(prr_val) or math.isinf(prr_val):
            continue
        last_prr = prr_val

    return last_prr


def main():
    ap = argparse.ArgumentParser(
        description="Run ns-3 C-V2X PRR sweep + SB-SPS validation")
    ap.add_argument('--ns3-dir', type=str, default=DEFAULT_NS3_DIR,
                    help=f"Path to ns-3_c-v2x build (default: {DEFAULT_NS3_DIR})")
    ap.add_argument('--output', type=str, default=DEFAULT_OUTPUT,
                    help="Output CSV path")
    ap.add_argument('--N-values', nargs='*', type=int, default=N_VALUES,
                    help="Sender counts")
    ap.add_argument('--M-values', nargs='*', type=int, default=M_VALUES,
                    help="Resource slot counts")
    ap.add_argument('--sim-time', type=int, default=60,
                    help="Simulation time in seconds")
    ap.add_argument('--p-keep', type=float, default=0.4,
                    help="SB-SPS resource keep probability")
    ap.add_argument('--dry-run', action='store_true',
                    help="Print commands without executing")
    ap.add_argument('--skip-validation', action='store_true',
                    help="Write CSV but don't run validate_mac_vs_ns3.py")
    args = ap.parse_args()

    ns3_dir = os.path.abspath(args.ns3_dir)

    if args.dry_run:
        print("DRY RUN — commands that would be executed:\n")
        for N in args.N_values:
            for M in args.M_values:
                print(f"  v2x_communication_example "
                      f"--numVeh={N} --numSubchannel={M} "
                      f"--time={args.sim_time} "
                      f"--probResourceKeep={args.p_keep}")
        print(f"\nTotal configs: {len(args.N_values) * len(args.M_values)}")
        return

    # Check ns-3 build
    print("ns-3 C-V2X PRR Sweep")
    print(f"  ns3-dir: {ns3_dir}")

    try:
        binary, lib_dir = find_binary(ns3_dir)
        print(f"  binary:  {binary}")
    except FileNotFoundError as e:
        print(f"\nERROR: {e}", file=sys.stderr)
        sys.exit(1)

    # Run sweep
    results = []
    total = len(args.N_values) * len(args.M_values)

    for M in args.M_values:
        print(f"\n  M={M}:")
        for N in args.N_values:
            prr = run_ns3_simulation(
                binary, lib_dir, ns3_dir,
                N, M, args.sim_time, args.p_keep)
            if prr is not None:
                results.append({'N': N, 'M': M, 'PRR': prr})

    # Write CSV
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['N', 'M', 'PRR'])
        writer.writeheader()
        writer.writerows(results)

    print(f"\n  Results: {len(results)}/{total} configs succeeded")
    print(f"  CSV written to: {args.output}")

    if not results:
        print("\nNo results — skipping validation.", file=sys.stderr)
        sys.exit(1)

    # Invoke validation
    if not args.skip_validation:
        print("\nRunning SB-SPS validation...")
        validate_script = os.path.join(SCRIPT_DIR, 'validate_mac_vs_ns3.py')
        cmd = [sys.executable, validate_script,
               '--ns3-csv', args.output]
        subprocess.run(cmd, check=False)


if __name__ == '__main__':
    main()
