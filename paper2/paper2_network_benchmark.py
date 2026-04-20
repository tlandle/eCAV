#!/usr/bin/env python3
"""
Network latency benchmark using ns-3 NR V2X co-simulation.

Measures uplink latency and PRR for the edge pipeline at varying N
using the 5G C-V2X Uu uplink model (NR Uu, not sidelink).

Also measures V2V sidelink PRR for comparison.

Payload sizes:
  Uplink (vehicle->edge):
    - Uncompressed intermediate fusion: 200,000 bytes (~200KB)
    - Post-backbone compressed (256x):   20,000 bytes (~20KB)
  Downlink (edge->vehicle):
    - SMART predictions: ~13,000 bytes per vehicle
      (20 tracks * 80 steps * 2 coords * 4 bytes)

Outputs:
  - paper2_figures/ns3_network_timing.csv
  - paper2_figures/fig_network_uplink.pdf
  - paper2_figures/fig_network_v2v_vs_edge.pdf
"""

import sys
import time
import csv
import math
import numpy as np
from pathlib import Path

sys.path.insert(0, '.')

OUT = Path("paper2_figures")
OUT.mkdir(exist_ok=True)

N_AGENTS = [1, 2, 4, 8, 16, 24, 32, 48, 64]
N_TICKS = 50   # ticks per N value to average over
TICK_DUR = 0.05  # 20 Hz

# Payload sizes (actual measured tensor sizes)
PAYLOADS = {
    'cobevt_256x':        16_896,   # CoBEVT 256x: [1,1,48,176] FP16
    'cobevt_uncompressed': 65535,   # CoBEVT raw: 4.2MB but SHM clamps to uint16 max
    'wf_uncompressed':     65535,   # WorldFusion raw: 9.8MB, clamped to uint16 max
    # Note: payloads > 65535 are clamped by SHM uint16 field.
    # The analytical model handles larger sizes correctly.
}

# Vehicle positions: distribute in a 200m x 200m area around an intersection
def generate_positions(n):
    """Place N vehicles around a 4-way intersection."""
    positions = []
    for i in range(n):
        angle = 2 * math.pi * i / n
        radius = 30 + 20 * (i % 3)
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = 1.5  # antenna height
        positions.append([x, y, z])
    return positions


def run_ns3_sweep(mode, payload_bytes, label):
    """Run ns-3 at each N and collect PRR + latency."""
    from ecav.core.networking.ns3_cosim.ns3_bridge import NS3Bridge

    print(f"\n=== ns-3 sweep: mode={mode}, payload={payload_bytes} bytes ({label}) ===")

    results = {}
    for N in N_AGENTS:
        positions = generate_positions(N)
        # Dummy occlusion (ns-3 has own propagation)
        from ecav.core.networking.channel_engine_py import OcclusionInfo
        occlusion = [[OcclusionInfo() for _ in range(N)] for _ in range(N)]

        bridge = NS3Bridge(
            mode=mode,
            max_vehicles=128,
            bandwidth_mhz=40.0,
            tx_power_dbm=23.0,
            numerology=0,
            tick_duration_s=TICK_DUR,
        )

        try:
            bridge.start()
        except FileNotFoundError as e:
            print(f"  ns-3 binary not found: {e}")
            return None
        except RuntimeError as e:
            print(f"  ns-3 failed to start: {e}")
            return None

        prr_list = []
        latency_list = []

        for tick in range(N_TICKS):
            link_results = bridge.compute_tick(
                positions, occlusion, tick, payload_bytes=payload_bytes)

            # Compute PRR and mean latency from link results
            if link_results:
                delivered = sum(1 for lr in link_results if lr.delivered)
                total = len(link_results)
                prr = delivered / total if total > 0 else 0.0
                delays = [lr.delay_ms for lr in link_results if lr.delivered]
                mean_delay = np.mean(delays) if delays else 0.0
            else:
                prr = 0.0
                mean_delay = 0.0

            prr_list.append(prr)
            latency_list.append(mean_delay)

        bridge.shutdown()

        mean_prr = np.mean(prr_list[5:])  # skip first 5 ticks (warmup)
        mean_lat = np.mean(latency_list[5:])
        p95_lat = np.percentile(latency_list[5:], 95)

        results[N] = {
            'prr': mean_prr,
            'latency_mean_ms': mean_lat,
            'latency_p95_ms': p95_lat,
        }
        print(f"  N={N:3d}: PRR={mean_prr:.3f}, latency={mean_lat:.1f}ms (p95={p95_lat:.1f}ms)")

    return results


def main():
    print("=" * 60)
    print("Network Benchmark via ns-3 NR V2X Co-Simulation")
    print("=" * 60)

    # Edge uplink (Uu) with compressed features (17KB CoBEVT 256x)
    uu_compressed = run_ns3_sweep("uu_uplink", PAYLOADS['cobevt_256x'], "Uu uplink 17KB")

    # V2V sidelink with compressed features (17KB CoBEVT 256x)
    v2v_compressed = run_ns3_sweep("sidelink", PAYLOADS['cobevt_256x'], "sidelink 17KB")

    # V2V sidelink with max representable payload (shows degradation)
    v2v_large = run_ns3_sweep("sidelink", PAYLOADS['cobevt_uncompressed'], "sidelink 65KB")

    # Save CSV
    csv_path = OUT / 'ns3_network_timing.csv'
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['N', 'mode', 'payload_label', 'payload_bytes',
                     'prr', 'latency_mean_ms', 'latency_p95_ms'])
        for label, mode, payload, data in [
            ('uu_17KB', 'uu_uplink', PAYLOADS['cobevt_256x'], uu_compressed),
            ('sidelink_17KB', 'sidelink', PAYLOADS['cobevt_256x'], v2v_compressed),
            ('sidelink_65KB', 'sidelink', PAYLOADS['cobevt_uncompressed'], v2v_large),
        ]:
            if data is None:
                continue
            for N in N_AGENTS:
                if N in data:
                    d = data[N]
                    w.writerow([N, mode, label, payload,
                               f"{d['prr']:.4f}",
                               f"{d['latency_mean_ms']:.2f}",
                               f"{d['latency_p95_ms']:.2f}"])
    print(f"\nSaved {csv_path}")

    # Plot
    if any(d is not None for d in [uu_compressed, v2v_compressed, v2v_large]):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MaxNLocator

        plt.rcParams.update({
            'font.size': 11, 'font.family': 'serif',
            'figure.figsize': (7, 4.5), 'axes.grid': True, 'grid.alpha': 0.3,
        })
        Ns = np.array(N_AGENTS)

        # PRR comparison: Uu uplink vs sidelink
        fig, ax = plt.subplots()
        if uu_compressed:
            ax.plot(Ns, [uu_compressed[n]['prr'] for n in Ns],
                    'o-', color='#4e79a7', label='Uu uplink (17KB, scheduled)', linewidth=2)
        if v2v_compressed:
            ax.plot(Ns, [v2v_compressed[n]['prr'] for n in Ns],
                    's-', color='#f28e2b', label='V2V sidelink (17KB, contention)', linewidth=2)
        if v2v_large:
            ax.plot(Ns, [v2v_large[n]['prr'] for n in Ns],
                    '^-', color='#e15759', label='V2V sidelink (65KB, contention)', linewidth=2)
        ax.axhline(y=0.9, color='gray', linestyle=':', label='90% reliability')
        ax.set_xlabel('Number of agents (N)')
        ax.set_ylabel('Packet Reception Ratio')
        ax.set_title('ns-3 5G-LENA NR V2X: Edge Uplink vs V2V Sidelink')
        ax.legend(fontsize=9)
        ax.set_xlim(Ns[0], Ns[-1])
        ax.set_ylim(0, 1.05)
        fig.tight_layout()
        fig.savefig(OUT / 'fig_ns3_prr_comparison.pdf', dpi=150)
        fig.savefig(OUT / 'fig_ns3_prr_comparison.png', dpi=150)
        plt.close(fig)
        print(f"Saved {OUT / 'fig_ns3_prr_comparison.png'}")

        # Uu uplink latency
        if uu_compressed:
            fig, ax = plt.subplots()
            ax.plot(Ns, [uu_compressed[n]['latency_mean_ms'] for n in Ns],
                    'o-', color='#4e79a7', label='Mean', linewidth=2)
            ax.plot(Ns, [uu_compressed[n]['latency_p95_ms'] for n in Ns],
                    'o--', color='#4e79a7', label='P95', linewidth=1, alpha=0.6)
            ax.set_xlabel('Number of agents (N)')
            ax.set_ylabel('Uplink latency (ms)')
            ax.set_title('ns-3 NR Uu Uplink Latency (17KB payload, 5G-LENA)')
            ax.legend(fontsize=9)
            ax.set_xlim(Ns[0], Ns[-1])
            ax.set_ylim(0)
            fig.tight_layout()
            fig.savefig(OUT / 'fig_ns3_uplink_latency.pdf', dpi=150)
            fig.savefig(OUT / 'fig_ns3_uplink_latency.png', dpi=150)
            plt.close(fig)
            print(f"Saved {OUT / 'fig_ns3_uplink_latency.png'}")

    print("\n" + "=" * 60)
    print("Network benchmark complete.")
    print("=" * 60)


if __name__ == '__main__':
    main()
