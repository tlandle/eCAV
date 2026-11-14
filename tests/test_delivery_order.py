"""A recipient keeps the newest forecast by source tick.

The intermediate-fusion manager queues each forecast with a delivery tick computed from a
sampled downlink delay. When an older forecast has the later delivery tick, it must not be
delivered over the newer one (it was before 2026-11-13: one arrival in ten in the network-only
condition). The same rule filters frames drained out of source order. Runs standalone."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ecav.core.application.edge.latency.delivery_order import select_ready, in_source_order  # noqa: E402


def main():
    # (deliver_tick, payload, source_tick, net_ms): the forecast of source tick 10 is delayed
    # to tick 17, the one of source tick 11 arrives at tick 15
    q = [(17, "f10", 10, 0.0), (15, "f11", 11, 0.0), (19, "f12", 12, 0.0)]
    newest, got, dropped = None, [], 0
    for tick in range(14, 21):
        e, q, sup = select_ready(q, tick, newest)
        dropped += sup
        if e is not None:
            newest = e[2]; got.append((tick, e[1]))
    assert got == [(15, "f11"), (19, "f12")], got       # f10 never replaces f11
    assert dropped == 1 and q == [], (dropped, q)
    # two forecasts ready on one tick: the newer source wins, the other is superseded
    e, q2, sup = select_ready([(5, "a", 3, 0.0), (5, "b", 4, 0.0)], 5, None)
    assert e[1] == "b" and sup == 1 and q2 == []
    # in order: nothing dropped, delivery unchanged
    q = [(11, "x", 10, 0.0), (12, "y", 11, 0.0)]
    e, q, sup = select_ready(q, 11, None)
    assert e[1] == "x" and sup == 0 and q == [(12, "y", 11, 0.0)]
    # nothing ready: the queue object is returned untouched
    q = [(20, "z", 15, 0.0)]
    e, q3, sup = select_ready(q, 11, 12)
    assert e is None and q3 is q and sup == 0
    # frames drained out of source order
    kept, newest, late = in_source_order([(7, "g7"), (8, "g8")], 8)
    assert kept == [] and newest == 8 and late == 2
    kept, newest, late = in_source_order([(6, "g6"), (9, "g9")], 8)
    assert kept == [(9, "g9")] and newest == 9 and late == 1
    kept, newest, late = in_source_order([(1, "a"), (2, "b")], None)
    assert kept == [(1, "a"), (2, "b")] and newest == 2 and late == 0
    print("PASS")


def test_delivery_order():
    main()


if __name__ == "__main__":
    main()
