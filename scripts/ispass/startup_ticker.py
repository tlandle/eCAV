#!/usr/bin/env python3
"""Advance the CARLA world at a fixed period while the vehicle clients of a scale run start.

    python scripts/ispass/startup_ticker.py <carla host> <port> <period s>

Used by launch_scale.sh with SCALE_TICKER=1. The clients then run with ECAV_STARTUP_TICKER=1 and wait for these
frames where ScenarioRunner would tick the world itself (ecav/scenario_testing/scale_town06.py). One ticker gives
frames a fixed spacing. Ticks from many starting clients arrive in bursts, and two frames in quick succession stop
the stream threads of CARLA 0.9.15 in synchronous mode. The launcher stops the ticker when every client has
started, before the orchestrator's first tick. The world is ticked only while it is in synchronous mode.
"""
import sys
import time

import carla


def main():
    host, port, period = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
    client = carla.Client(host, port)
    client.set_timeout(60.0)
    world = client.get_world()
    n = 0
    while True:
        t0 = time.time()
        try:
            if world.get_settings().synchronous_mode:
                frame = world.tick(60.0)
                n += 1
                if n % 60 == 1:
                    print(f"ticker: {n} ticks, frame {frame}", flush=True)
        except RuntimeError as e:
            print(f"ticker: {e}", flush=True)
        time.sleep(max(0.0, period - (time.time() - t0)))


if __name__ == "__main__":
    main()
