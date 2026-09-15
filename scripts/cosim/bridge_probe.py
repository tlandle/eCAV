#!/usr/bin/env python3
"""Standalone probe of the ns-3 co-simulation bridge with a given UE geometry.

Runs one uplink-mode bridge (gNB at the origin) for a few ticks with the
positions given on the command line, prints the rows per tick, and on a
server death prints its exit code and the tail of its stderr (the bridge
keeps the pipe). Use it to reproduce a closed-loop abort outside the
simulator, e.g.
  python scripts/cosim/bridge_probe.py --ue -7.1,-14.6 --ue -7.1,-14.6 --tick 0.2
"""
import argparse
import sys
import time

sys.path.insert(0, '.')
from ecav.core.networking.ns3_cosim.ns3_bridge import NS3Bridge  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ue', action='append', default=[], help='x,y relative to the gNB (m); repeatable')
    ap.add_argument('--tick', type=float, default=0.2)
    ap.add_argument('--ticks', type=int, default=4)
    ap.add_argument('--payload', type=int, default=16896)
    ap.add_argument('--max-vehicles', type=int, default=8)
    ap.add_argument('--tag', default='probe')
    a = ap.parse_args()
    pts = []
    for s in a.ue:
        x, y = (float(v) for v in s.split(','))
        pts.append([x, y, 1.5])
    if not pts:
        pts = [[10.0, 0.0, 1.5], [-10.0, 0.0, 1.5]]
    b = NS3Bridge(max_vehicles=a.max_vehicles, mode="uu_uplink", bandwidth_mhz=40.0,
                  tx_power_dbm=23.0, numerology=0, tick_duration_s=a.tick, shm_tag=a.tag)
    t0 = time.perf_counter()
    b.start()
    print('start_ms=%.0f shm=%s pid=%s' % ((time.perf_counter() - t0) * 1000, b._shm_name, b._process.pid))
    rc = 0
    try:
        for t in range(1, a.ticks + 1):
            t1 = time.perf_counter()
            res = b.compute_tick(pts, [], t, payload_bytes=a.payload)
            wall = (time.perf_counter() - t1) * 1000
            ul = [r.delay_ms for r in res if r.rx_id == 0xFFFF]
            dl = [r.delay_ms for r in res if r.tx_id == 0xFFFE]
            print('tick=%d n_ue=%d n_results=%d wall_ms=%.0f ul=%s dl=%s delivered=%d/%d' % (
                t, len(pts), len(res), wall, ['%.1f' % v for v in ul], ['%.1f' % v for v in dl],
                sum(1 for r in res if r.delivered), len(res)))
            if b._process.poll() is not None:
                rc = b._process.returncode
                print('SERVER DIED exit=%s' % rc)
                try:
                    err = b._process.stderr.read().decode(errors='replace')
                    out = b._process.stdout.read().decode(errors='replace')
                except Exception as e:  # noqa: BLE001
                    err, out = 'read failed: %r' % e, ''
                print('--- server stderr tail ---')
                print('\n'.join(err.splitlines()[-25:]))
                print('--- server stdout tail ---')
                print('\n'.join(out.splitlines()[-10:]))
                break
    finally:
        b.shutdown()
    return 1 if rc else 0


if __name__ == '__main__':
    sys.exit(main())
