#!/usr/bin/env python3
"""
Standalone validation of SB-SPS MAC model against analytical formula
and (optionally) ns-3 C-V2X simulation results.

Analytical baseline:
    Todisco et al. 2023 (arXiv:2309.16680) formula for C-V2X Mode 4 PRR
    under uniform random resource selection with reselection.

    PRR_analytical(N, M) = (1 - 1/M)^(N-1)

    This is the single-subframe collision probability for N UEs sharing
    M orthogonal resources.  The SB-SPS model adds persistence (p_keep)
    and fading (p_loss_base), so PRR should be slightly lower.

ns-3 baseline:
    If --ns3-csv is provided, loads PRR results from Eckermann's
    ns-3_c-v2x module and overlays them.

Usage:
    python scripts/validate_mac_vs_ns3.py
    python scripts/validate_mac_vs_ns3.py --ns3-csv ns3_prr_results.csv

    ns-3 CSV format: columns N,M,PRR (one row per config point)
"""

import argparse
import sys
import os

# Add parent dir to path so we can import the MAC model
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


def analytical_prr(N, M):
    """Todisco et al. analytical PRR for uniform random selection.

    PRR = (1 - 1/M)^(N-1) * (1 - p_loss_base)
    Without fading: PRR = (1 - 1/M)^(N-1)
    """
    return (1.0 - 1.0 / M) ** (N - 1)


def run_sbsps_sweep(N_values, M, num_ticks=10000, p_keep=0.4,
                    p_loss_base=0.02, seed=42):
    """Run SB-SPS model for each N and return PRR + burst stats."""
    from opencda.core.application.edge.latency.mac_model import SbSpsMac

    results = []
    for N in N_values:
        mac = SbSpsMac(M=M, RC_min=5, RC_max=15, p_keep=p_keep,
                       p_loss_base=p_loss_base, seed=seed)
        sender_ids = list(range(N))

        for tick in range(num_ticks):
            mac.attempt_tick(tick, sender_ids)

        mac.metrics.finalize()
        summary = mac.metrics.get_summary()
        results.append({
            'N': N,
            'M': M,
            'prr': summary['prr'],
            'collision_drops': summary['collision_drops'],
            'fading_drops': summary['fading_drops'],
            'burst_mean': summary['burst_length_mean'],
            'burst_p95': summary['burst_length_p95'],
            'burst_p99': summary['burst_length_p99'],
            'burstiness_factor': summary['mac_burstiness_factor'],
        })
    return results


def load_ns3_csv(path):
    """Load ns-3 PRR results from CSV (columns: N, M, PRR)."""
    import csv
    data = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            data.append({
                'N': int(row['N']),
                'M': int(row['M']),
                'prr': float(row['PRR']),
            })
    return data


def main():
    ap = argparse.ArgumentParser(description="Validate SB-SPS MAC model")
    ap.add_argument("--ns3-csv", type=str, default=None,
                    help="Path to ns-3 PRR results CSV (columns: N, M, PRR)")
    ap.add_argument("--M-values", nargs="*", type=int, default=[10, 20, 50],
                    help="Resource slot counts to test")
    ap.add_argument("--N-values", nargs="*", type=int,
                    default=[2, 4, 8, 16, 32],
                    help="Number of senders to test")
    ap.add_argument("--ticks", type=int, default=10000,
                    help="Number of ticks per simulation")
    ap.add_argument("--p-keep", type=float, default=0.4)
    ap.add_argument("--p-loss-base", type=float, default=0.02)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--output", type=str, default="mac_validation.png",
                    help="Output figure path")
    args = ap.parse_args()

    N_values = args.N_values
    M_values = args.M_values

    print(f"SB-SPS MAC Validation")
    print(f"  N values: {N_values}")
    print(f"  M values: {M_values}")
    print(f"  Ticks: {args.ticks}")
    print(f"  p_keep={args.p_keep}, p_loss_base={args.p_loss_base}")
    print()

    # Run sweeps
    all_results = {}
    for M in M_values:
        results = run_sbsps_sweep(N_values, M, num_ticks=args.ticks,
                                  p_keep=args.p_keep,
                                  p_loss_base=args.p_loss_base,
                                  seed=args.seed)
        all_results[M] = results

        print(f"  M={M}:")
        print(f"    {'N':>4} {'PRR_model':>10} {'PRR_analytical':>15} "
              f"{'delta':>8} {'burst_mean':>11} {'burst_p95':>10} "
              f"{'burstiness':>11}")
        for r in results:
            a_prr = analytical_prr(r['N'], M)
            delta = r['prr'] - a_prr
            print(f"    {r['N']:>4} {r['prr']:>10.4f} {a_prr:>15.4f} "
                  f"{delta:>+8.4f} {r['burst_mean']:>11.2f} "
                  f"{r['burst_p95']:>10.1f} {r['burstiness_factor']:>11.2f}")
        print()

    # Load ns-3 data if provided
    ns3_data = None
    if args.ns3_csv:
        ns3_data = load_ns3_csv(args.ns3_csv)
        print(f"  Loaded {len(ns3_data)} ns-3 data points")

    # Validate: PRR error vs analytical should be small
    max_abs_error = 0.0
    for M, results in all_results.items():
        for r in results:
            a_prr = analytical_prr(r['N'], M)
            # Model PRR should be <= analytical (fading + persistence lower it)
            error = abs(r['prr'] - a_prr)
            max_abs_error = max(max_abs_error, error)

    print(f"  Max absolute PRR error vs analytical: {max_abs_error:.4f}")
    # With p_loss_base=0.02 and p_keep=0.4, expect ~2-5% lower than analytical
    if max_abs_error < 0.10:
        print("  PASS: PRR within 10% of analytical (expected with fading + persistence)")
    else:
        print("  WARNING: PRR diverges >10% from analytical")

    # Check monotonicity: PRR should decrease with N for fixed M
    for M, results in all_results.items():
        prrs = [r['prr'] for r in results]
        monotone = all(prrs[i] >= prrs[i+1] for i in range(len(prrs)-1))
        print(f"  M={M} monotonicity (PRR decreases with N): "
              f"{'PASS' if monotone else 'FAIL'}")

    # Check collision dominance at high N
    for M, results in all_results.items():
        high_n = [r for r in results if r['N'] >= 16]
        for r in high_n:
            ratio = r['collision_drops'] / max(r['fading_drops'], 1)
            print(f"  M={M} N={r['N']}: collision/fading ratio = {ratio:.1f} "
                  f"({'PASS: collision dominant' if ratio > 2 else 'NOTE: fading significant'})")

    # Plot if matplotlib available
    if HAS_MPL:
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))

        # (a) PRR vs N
        ax = axes[0]
        colors = plt.cm.tab10(np.linspace(0, 0.5, len(M_values)))
        for i, M in enumerate(M_values):
            results = all_results[M]
            ns = [r['N'] for r in results]
            prrs = [r['prr'] for r in results]
            ax.plot(ns, prrs, 'o-', color=colors[i], label=f'SB-SPS M={M}',
                    linewidth=2)
            # Analytical overlay
            a_ns = np.linspace(min(ns), max(ns), 50)
            a_prrs = [analytical_prr(n, M) for n in a_ns]
            ax.plot(a_ns, a_prrs, '--', color=colors[i], alpha=0.5,
                    label=f'Analytical M={M}')

        # ns-3 overlay
        if ns3_data:
            for M in M_values:
                pts = [d for d in ns3_data if d['M'] == M]
                if pts:
                    ns = [p['N'] for p in pts]
                    prrs = [p['prr'] for p in pts]
                    ax.plot(ns, prrs, 'x', markersize=10, markeredgewidth=2,
                            label=f'ns-3 M={M}')

        ax.set_xlabel('Number of Senders (N)')
        ax.set_ylabel('Packet Reception Ratio')
        ax.set_title('(a) PRR vs N')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.2)
        ax.set_ylim(0, 1.05)

        # (b) Burst-length CDF for M=20
        ax = axes[1]
        M_ref = 20 if 20 in all_results else M_values[0]
        results = all_results[M_ref]
        for r in results:
            hist = r.get('burst_histogram', {})
            if not hist:
                # Re-run to get histogram (the summary didn't store raw)
                from opencda.core.application.edge.latency.mac_model import SbSpsMac
                mac = SbSpsMac(M=M_ref, p_keep=args.p_keep,
                               p_loss_base=args.p_loss_base, seed=args.seed)
                senders = list(range(r['N']))
                for t in range(args.ticks):
                    mac.attempt_tick(t, senders)
                mac.metrics.finalize()
                hist = mac.metrics.get_summary()['burst_histogram']

            if hist:
                sorted_lens = sorted(int(k) for k in hist.keys())
                total = sum(hist[str(k)] if str(k) in hist else hist.get(k, 0)
                            for k in sorted_lens)
                if total > 0:
                    cdf = []
                    cumul = 0
                    for bl in sorted_lens:
                        key = str(bl) if str(bl) in hist else bl
                        cumul += hist.get(key, 0)
                        cdf.append(cumul / total)
                    ax.step(sorted_lens, cdf, where='post',
                            label=f'N={r["N"]}', linewidth=2)

        ax.set_xlabel('Burst Length')
        ax.set_ylabel('CDF')
        ax.set_title(f'(b) Burst-Length CDF (M={M_ref})')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)

        # (c) Burstiness factor vs N
        ax = axes[2]
        for i, M in enumerate(M_values):
            results = all_results[M]
            ns = [r['N'] for r in results]
            bfs = [r['burstiness_factor'] for r in results]
            ax.plot(ns, bfs, 's-', color=colors[i], label=f'M={M}',
                    linewidth=2)
        ax.axhline(y=1.0, color='gray', linestyle=':', alpha=0.5,
                   label='Memoryless')
        ax.set_xlabel('Number of Senders (N)')
        ax.set_ylabel('Burstiness Factor ($p_{95}$ / mean)')
        ax.set_title('(c) Burstiness Factor')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.2)

        fig.suptitle('SB-SPS MAC Model Validation', fontsize=13, y=1.02)
        fig.tight_layout()
        fig.savefig(args.output, bbox_inches='tight', dpi=150)
        print(f"\n  Figure saved to {args.output}")
        plt.close(fig)

    print("\nDone.")


if __name__ == '__main__':
    main()
