#!/usr/bin/env python3
"""Advance the CARLA world while the vehicle clients of a scale run start.

    python scripts/ispass/startup_ticker.py <carla host> <port> <gap s>

Used by launch_scale.sh with SCALE_TICKER=1. The clients then run with ECAV_STARTUP_TICKER=1 and wait for these
frames where ScenarioRunner would tick the world itself (ecav/scenario_testing/scale_town06.py). The launcher stops
the ticker when every client has started, before the orchestrator's first tick. The world is ticked only while it
is in synchronous mode.

Why one ticker: in synchronous mode CARLA 0.9.15 writes every frame to every stream session, and a session that
gets a frame while its previous write has not completed spins in its strand and takes a stream thread with it
(ServerSession::Write). Ticks from many starting clients arrive in bursts and stopped all stream threads at 716 of
1,024 clients. A fixed period between tick starts is not enough either: near 1,000 clients a tick takes about as
long as a 1 s period, the frames follow each other without a pause, and the stream threads stopped again. So a tick
is issued only when
  - the kernel holds no unsent data for the simulator's streaming sockets (ss Send-Q, when the simulator is local), and
  - at least <gap> seconds, and at least twice the duration of the last tick, have passed since the last frame
    returned.
"""
import subprocess
import sys
import time

import carla

FLUSH_WAIT_S = 30.0


def unsent_bytes(stream_port):
    """Bytes the kernel still holds for the simulator's streaming sockets, or None when ss is not usable."""
    try:
        out = subprocess.run(["ss", "-Htn", "sport", "=", f":{stream_port}"], capture_output=True, text=True,
                             timeout=20).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    total = 0
    for line in out.splitlines():
        f = line.split()
        if len(f) >= 3 and f[2].isdigit():
            total += int(f[2])
    return total


def main():
    host, port, gap = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
    local = host in ("localhost", "127.0.0.1")
    client = carla.Client(host, port)
    client.set_timeout(60.0)
    world = client.get_world()
    n = 0
    while True:
        try:
            if not world.get_settings().synchronous_mode:
                time.sleep(gap)
                continue
            w0 = time.time()
            while local and unsent_bytes(port + 1) and time.time() - w0 < FLUSH_WAIT_S:
                time.sleep(0.2)
            waited = time.time() - w0
            t0 = time.time()
            frame = world.tick(60.0)
            dt = time.time() - t0
            n += 1
            if n % 60 == 1 or waited > 2.0:
                print(f"{time.strftime('%T')} ticker: {n} ticks, frame {frame}, tick {dt * 1000:.0f} ms, "
                      f"waited {waited:.1f} s for the stream sockets", flush=True)
            time.sleep(max(gap, 2.0 * dt))
        except RuntimeError as e:
            print(f"{time.strftime('%T')} ticker: {e}", flush=True)
            time.sleep(gap)


if __name__ == "__main__":
    main()
