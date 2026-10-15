"""The run watchdog kills a silent or over-long run and its children."""
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from scripts.proc_watchdog import stream_with_watchdog  # noqa: E402


def _spawn(code):
    return subprocess.Popen([sys.executable, "-u", "-c", code], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, start_new_session=True)


def _gone(pid, wait_s=5.0):
    t0 = time.time()
    while time.time() - t0 < wait_s:
        try:
            with open(f"/proc/{pid}/stat") as fh:
                state = fh.read().rsplit(")", 1)[1].split()[0]
        except FileNotFoundError:
            return True
        if state in ("Z", "X"):
            return True
        time.sleep(0.2)
    return False


def test_normal_exit_streams_every_line():
    p = _spawn("print('a'); print('b')")
    got = []
    assert stream_with_watchdog(p, got.append, 5, 30) is None
    assert got == ["a\n", "b\n"]
    assert p.returncode == 0


def test_silent_run_and_its_child_are_killed():
    code = ("import subprocess, sys, time; print('started', flush=True); "
            "g = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)']); "
            "print('grandchild', g.pid, flush=True); time.sleep(60)")
    p = _spawn(code)
    got = []
    t0 = time.time()
    reason = stream_with_watchdog(p, got.append, 2, 60)
    assert reason is not None and reason.startswith("no output")
    assert time.time() - t0 < 20
    assert p.returncode is not None
    gpid = int([l for l in got if l.startswith("grandchild")][0].split()[1])
    assert _gone(gpid)
    assert any(l.startswith("\nWATCHDOG") for l in got)


def test_wall_cap_kills_a_run_that_keeps_printing():
    p = _spawn("import time\nwhile True:\n    print('tick', flush=True); time.sleep(0.1)")
    got = []
    reason = stream_with_watchdog(p, got.append, 5, 2)
    assert reason is not None and "cap" in reason
    assert p.returncode is not None


if __name__ == "__main__":
    for _n, _f in sorted(globals().items()):
        if _n.startswith("test_"):
            _t0 = time.time()
            _f()
            print(f"ok {_n} ({time.time() - _t0:.1f} s)")
