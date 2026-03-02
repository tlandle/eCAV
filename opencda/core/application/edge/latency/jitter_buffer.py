# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Generic jitter buffer for the edge latency simulation.

Incoming data packets are stamped with (source_tick, arrival_tick).
Each simulation tick, the edge drains all packets whose arrival_tick
has been reached, sorted by source_tick so the downstream tracker
always sees temporally ordered data.

This is the standard approach used in Apollo, Autoware, and similar
production AV stacks to handle variable network latency without
replaying the full detection history every tick.
"""

from __future__ import annotations

from typing import Generic, List, Tuple, TypeVar

T = TypeVar("T")


class JitterBuffer(Generic[T]):
    """Capacity-bounded buffer that reorders packets by source timestamp."""

    def __init__(self, capacity: int = 100):
        self._buf: List[Tuple[int, int, T]] = []   # (source_tick, arrival_tick, payload)
        self._capacity = capacity

    # ------------------------------------------------------------------
    def push(self, source_tick: int, arrival_tick: int, payload: T) -> None:
        """Add a packet to the buffer.

        If the buffer is at capacity, the oldest entry (by source_tick)
        is silently dropped.
        """
        if len(self._buf) >= self._capacity:
            # Drop the entry with the smallest source_tick
            self._buf.sort(key=lambda x: x[0])
            self._buf.pop(0)
        self._buf.append((source_tick, arrival_tick, payload))

    # ------------------------------------------------------------------
    def drain(self, current_tick: int) -> List[Tuple[int, T]]:
        """Return all packets whose arrival_tick <= *current_tick*.

        Returns
        -------
        list of (source_tick, payload)
            Sorted ascending by source_tick so that the tracker
            processes data in temporal order.
        """
        ready = [(src, payload) for src, arr, payload in self._buf
                 if arr <= current_tick]
        self._buf = [(src, arr, payload) for src, arr, payload in self._buf
                     if arr > current_tick]
        ready.sort(key=lambda x: x[0])
        return ready

    # ------------------------------------------------------------------
    @property
    def pending(self) -> int:
        """Number of packets still waiting in the buffer."""
        return len(self._buf)
