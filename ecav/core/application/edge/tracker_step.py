"""Simulation time per tracker step, and the unit conversion of the tracker's velocity.

AB3DMOT publishes velocity in metres per tracker step (the least-squares slope of position
against its own call count over the last ten calls). A manager that feeds the tracker every
simulation tick has a step of one tick. A manager that feeds it only the frames its service
admits (service occupancy) has a step of several ticks, and the step varies from call to call.
The linear predictor extrapolates ``kf_vx`` once per simulation tick, so the manager must
publish metres per tick, whatever the step.
"""
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG Non-Commercial Non-Distributable License

from collections import deque
from typing import Tuple


class TrackerStepClock:
    """Seconds of simulation time per tracker step, measured from the source ticks of the frames
    the tracker was fed.

    The value is the least-squares slope of source tick against call count over the last
    ``WINDOW`` calls, the same window and the same regressor the tracker uses for its published
    velocity. For a target at constant velocity the tracker's slope (m per call) divided by this
    slope (ticks per call) is the velocity in m per tick for any sequence of strides. When every
    stride in the window is equal the value is that stride times ``dt``, computed as before, so a
    manager that tracks every tick is unchanged.
    """

    WINDOW = 10

    def __init__(self, dt: float, fallback_s: float):
        self.dt = float(dt)
        self.fallback_s = float(fallback_s)
        self._calls = 0
        self._prev_frame = None
        self._pts = deque(maxlen=self.WINDOW + 1)   # (call count, source tick)

    def note(self, frame_id: int) -> None:
        """Record one tracker call on the frame with source tick ``frame_id``."""
        self._calls += 1
        if self._prev_frame is None or frame_id > self._prev_frame:
            self._pts.append((self._calls, int(frame_id)))
        self._prev_frame = frame_id

    @property
    def measured(self) -> bool:
        """True once two tracker calls with increasing source ticks have been recorded."""
        return len(self._pts) >= 2

    def step_s(self) -> float:
        """Seconds per tracker step; the fallback before two calls have been recorded."""
        if not self.measured:
            return self.fallback_s
        pts = list(self._pts)
        strides = {(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])}
        if len(strides) == 1:
            d_tick, d_call = next(iter(strides))
            if d_call == 1:
                return d_tick * self.dt
        n = len(pts)
        mc = sum(p[0] for p in pts) / n
        mt = sum(p[1] for p in pts) / n
        den = sum((p[0] - mc) ** 2 for p in pts)
        slope = sum((p[0] - mc) * (p[1] - mt) for p in pts) / den
        return slope * self.dt


def velocity_per_tick(vx: float, vy: float, is_mps: bool, step_s: float, dt: float,
                      measured: bool) -> Tuple[float, float, float]:
    """Convert a tracker row's ground velocity to what the manager publishes.

    Returns ``(kf_vx, kf_vy, kf_speed_mps)`` with ``kf_vx`` and ``kf_vy`` in metres per simulation
    tick and the speed in m/s. ``is_mps`` marks rows that already carry m/s; other rows carry
    metres per tracker step of ``step_s`` seconds. Before the step has been measured the row is
    passed through as it was (the tracker publishes a young track at rest).
    """
    speed = (vx * vx + vy * vy) ** 0.5
    if is_mps:
        scale = dt if measured else step_s
        return vx * scale, vy * scale, speed
    scale = dt / step_s if measured else 1.0
    return vx * scale, vy * scale, speed / step_s
