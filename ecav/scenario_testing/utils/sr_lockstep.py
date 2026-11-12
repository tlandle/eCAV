# -*- coding: utf-8 -*-
"""Lock step between the simulation loop and the scenario runner process.

In sequential runs the scenario runner (the process that drives the scripted
actors and their triggers) polls the world snapshot on wall-clock time and
ticks its behaviour tree when it sees a new frame. Nothing makes the main loop
wait for it, so the frame on which a scripted actor's control or trigger lands
depends on process scheduling. Two fixed-seed runs of the sensor-derived
highway cut-in differed in the scripted car's position by 0.9 m from the first
logged tick for that reason (2026-11-09 pair, diagnosed 2026-11-12).

With the environment variable SR_LOCKSTEP=1 (test_runner --scenario-env) the
scenario runner publishes the last frame it has processed and the main loop
waits for that frame before the next world tick. Default off: existing
campaigns ran without it and are not changed.

The acknowledgement is an 8-byte file in /dev/shm named after the main
process, which is the parent of the scenario runner process.
"""
import logging
import os
import struct
import time

logger = logging.getLogger(__name__)
_warned = False


def enabled() -> bool:
    return os.environ.get("SR_LOCKSTEP", "0") == "1"


def _path(main_pid: int) -> str:
    return f"/dev/shm/ecav_sr_ack_{int(main_pid)}"


def publish(frame: int) -> None:
    """Scenario runner side: frame is the snapshot frame just processed."""
    p = _path(os.getppid())
    tmp = p + ".tmp"
    with open(tmp, "wb") as f:
        f.write(struct.pack("<q", int(frame)))
    os.replace(tmp, p)


def _read(main_pid: int) -> int:
    try:
        with open(_path(main_pid), "rb") as f:
            b = f.read(8)
        return struct.unpack("<q", b)[0] if len(b) == 8 else -1
    except OSError:
        return -1


def wait(world, timeout_s: float = 5.0) -> bool:
    """Main loop side, before world.tick(): wait until the scenario runner has
    processed the current frame. Returns False on a timeout (the run goes on)."""
    global _warned
    if not enabled():
        return True
    target = world.get_snapshot().frame
    t0 = time.perf_counter()
    pid = os.getpid()
    while _read(pid) < target:
        if time.perf_counter() - t0 > timeout_s:
            if not _warned:
                logger.warning("scenario runner did not acknowledge frame %d within %.1f s; "
                               "continuing without lock step for this tick", target, timeout_s)
                _warned = True
            return False
        time.sleep(0.0005)
    return True


def cleanup() -> None:
    try:
        os.remove(_path(os.getpid()))
    except OSError:
        pass
