"""Stream a child's stdout with a watchdog.

The child's whole session is killed when it prints nothing for no_output_s
seconds or runs past hard_cap_s. test_runner.py wraps every run with it: the
CARLA server segfaulted in its Vulkan renderer mid-run (2026-10-15 01:50, crash
report under ~/.config/Epic/CarlaUE4/Saved/Crashes) and the run's main process
spun for 17 hours while the runner waited on its stdout. With the watchdog a
dead simulator costs one retried run, not the campaign.
"""
import os
import queue
import signal
import threading
import time


def kill_session(proc, grace_s=10.0):
    """SIGTERM, then SIGKILL, the process group of proc (started with
    start_new_session=True), so the scenario runner and the orchestration
    server the run spawned go with it."""
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        t0 = time.time()
        while time.time() - t0 < grace_s:
            if proc.poll() is not None:
                return
            time.sleep(0.2)


def stream_with_watchdog(proc, sink, no_output_s, hard_cap_s):
    """Pass every stdout line of proc to sink(line) until it exits.

    Returns None on a normal exit, else the reason the watchdog killed it
    (also written to the sink)."""
    q = queue.Queue()

    def _reader():
        try:
            for line in proc.stdout:
                q.put(line)
        finally:
            q.put(None)

    threading.Thread(target=_reader, daemon=True).start()
    t0 = last = time.time()
    reason = None
    while True:
        try:
            item = q.get(timeout=1.0)
        except queue.Empty:
            item = ""
        if item is None:
            break
        if item:
            sink(item)
            last = time.time()
        now = time.time()
        if now - last > no_output_s:
            reason = f"no output for {int(now - last)} s"
        elif now - t0 > hard_cap_s:
            reason = f"ran longer than the {int(hard_cap_s)} s cap"
        if reason:
            sink(f"\nWATCHDOG: {reason}; killing the run's process group\n")
            kill_session(proc)
            break
    proc.wait()
    return reason
