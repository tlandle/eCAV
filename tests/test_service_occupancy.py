"""ServiceOccupancy against the barrier arm of scripts/ispass/live_service_bench.py.

The synthetic cases always run. The recorded cases replay the durations of the
live-service sweep (experiment_results/ispass_timing_live/<stamp>/raw/*/barrier_*.json,
not tracked) through the gate and require the same admission tick, source frame and
delivery tick for every invocation.
    python -m pytest tests/test_service_occupancy.py -q
    python tests/test_service_occupancy.py            # prints the per-file table
"""
import glob
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from ecav.core.application.edge.service_occupancy import ServiceOccupancy, delivery_ticks  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
FRAME_MS = 50.0


def test_130ms_service_50ms_step():
    # the paper's example: a 130 ms invocation, 50 ms frames and steps
    out = delivery_ticks([130.0] * 6, 50.0, FRAME_MS, 20)
    assert [o[0] for o in out] == [0, 3, 6, 9, 12, 15]         # admitted every third step
    assert [o[1] for o in out] == [0, 3, 6, 9, 12, 15]         # newest frame each time
    assert [o[2] - o[0] for o in out] == [3] * 6               # visible three steps later


def test_service_within_one_step_admits_every_frame():
    out = delivery_ticks([30.0] * 10, 50.0, FRAME_MS, 10)
    assert [o[1] for o in out] == list(range(10))
    assert all(o[2] - o[0] == 1 for o in out)


def test_latest_frame_wins_and_counts():
    g = ServiceOccupancy(True, 50.0)
    assert g.admit(0, [(0, "a")]) == (0, "a")
    g.occupy(0, 120.0)
    assert g.admit(1, [(1, "b")]) is None
    assert g.admit(2, [(2, "c")]) is None
    assert g.admit(3, []) == (2, "c")          # the service frees at 120 ms; frame 2 is the newest waiting
    assert (g.offered, g.admitted, g.superseded) == (3, 2, 1)


def _recorded_files():
    return sorted(glob.glob(os.path.join(
        ROOT, "experiment_results", "ispass_timing_live", "*", "raw", "*", "barrier_*.json")))


def _compare(path):
    d = json.load(open(path))
    inv = d["invocations"]
    dt = float(d["dt_ms"])
    n_ticks = int(round((d["warm_s"] + d["duration_s"]) * 1000.0 / dt))
    out = delivery_ticks([i["total_ms"] for i in inv], dt, FRAME_MS, n_ticks)
    ref = [(int(i["tick"]), int(i["seq"]), int(i["deliver_tick"])) for i in inv]
    return len(ref), sum(1 for a, b in zip(out, ref) if a == b), len(out)


def test_recorded_barrier_traces():
    files = _recorded_files()
    if not files:
        import pytest
        pytest.skip("no recorded barrier traces on this host")
    for f in files:
        n_ref, n_same, n_out = _compare(f)
        assert n_out == n_ref and n_same == n_ref, (f, n_ref, n_same, n_out)


if __name__ == "__main__":
    files = _recorded_files()
    tot = same = 0
    bad = []
    for f in files:
        n_ref, n_same, n_out = _compare(f)
        tot += n_ref; same += n_same
        if not (n_out == n_ref == n_same):
            bad.append((f, n_ref, n_same, n_out))
    print(f"{len(files)} recorded traces, {tot} invocations, {same} identical (admit tick, frame, delivery tick)")
    for b in bad[:10]:
        print("MISMATCH", b)
