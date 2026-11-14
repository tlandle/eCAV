"""Delivery in source order.

A delay that varies from one message to the next can deliver an older message after a newer
one. A receiver keeps the newest state by source tick: an older forecast that arrives later is
superseded in transit and must not replace the newer one (it did in the intermediate-fusion
manager's outbound queue, where the downlink delay is sampled per forecast: one arrival in
ten in the network-only condition, straight crossing, 2026-11-13).
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from typing import List, Optional, Tuple


def select_ready(queue: List[tuple], tick: int, newest_src: Optional[int]):
    """Pick the forecast to deliver at ``tick`` from an outbound queue.

    ``queue`` holds tuples whose first element is the delivery tick and whose third is the
    source tick, (deliver_tick, payload, source_tick, ...). Returns
    ``(entry or None, remaining queue, superseded)``: the ready entry with the newest source
    tick if it is newer than ``newest_src`` (the newest source tick already delivered), the
    entries still in flight, and the number of ready entries dropped because a newer forecast
    had been or is being delivered.
    """
    ready = [e for e in queue if e[0] <= tick]
    if not ready:
        return None, queue, 0
    remaining = [e for e in queue if e[0] > tick]
    best = max(ready, key=lambda e: (e[2], e[0]))
    if newest_src is not None and best[2] <= newest_src:
        return None, remaining, len(ready)
    return best, remaining, len(ready) - 1


def in_source_order(frames: List[Tuple[int, object]], newest_src: Optional[int]):
    """Filter frames drained in one tick, ``(source_tick, payload)`` ascending by source tick, to
    those newer than ``newest_src`` (the newest source tick already ingested). Returns
    ``(kept frames, newest source tick after them, dropped count)``."""
    kept, dropped = [], 0
    for src, payload in frames:
        if newest_src is not None and src <= newest_src:
            dropped += 1
            continue
        kept.append((src, payload)); newest_src = src
    return kept, newest_src, dropped
