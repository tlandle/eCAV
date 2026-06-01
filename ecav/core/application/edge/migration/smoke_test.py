# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""End-to-end smoke test for the locale handover primitives.

Drives a synthetic vehicle trajectory through a two-locale deployment and
prints a tick-by-tick trace of position, current binding, and any handoff
event. Used to sanity-check the binding logic in isolation, with no CARLA
and no edge_manager dependencies.

Run:
    python -m ecav.core.application.edge.migration.smoke_test
    python -m ecav.core.application.edge.migration.smoke_test --plot

Scenarios:
    straight     -- one vehicle drives x=0 -> x=30 in a straight line.
                    Crosses A->B once.
    bounce       -- vehicle hovers around the boundary at x=10. Tests
                    hysteresis: brief excursions must NOT cause handoff.
    gap          -- trajectory passes through a coverage hole (x in [22,28])
                    between two locales. Binding holds prior locale.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from .binding import HandoffEvent, VehicleLocaleTracker
from .locale import Locale
from .registry import LocaleRegistry, LocaleRouter


# ──────────────────────────────────────────────────────────────────────────────
#  Synthetic deployment
# ──────────────────────────────────────────────────────────────────────────────
def build_two_locale_deployment() -> Tuple[LocaleRegistry, LocaleRouter]:
    """Two 20x20 locales side by side with a 1m shared boundary at x=10."""
    a = Locale(
        locale_id="A",
        polygon=np.array([[0, 0], [10, 0], [10, 20], [0, 20]], dtype=float),
        edge_host_id="edge1",
    )
    b = Locale(
        locale_id="B",
        polygon=np.array([[10, 0], [20, 0], [20, 20], [10, 20]], dtype=float),
        edge_host_id="edge2",
    )
    reg = LocaleRegistry()
    reg.register(a)
    reg.register(b)
    return reg, LocaleRouter(reg)


def build_two_locale_with_gap() -> Tuple[LocaleRegistry, LocaleRouter]:
    """A in [0,20], gap in [20,28], B in [28,40]. Same y-range [0,20]."""
    a = Locale(
        locale_id="A",
        polygon=np.array([[0, 0], [20, 0], [20, 20], [0, 20]], dtype=float),
        edge_host_id="edge1",
    )
    b = Locale(
        locale_id="B",
        polygon=np.array([[28, 0], [40, 0], [40, 20], [28, 20]], dtype=float),
        edge_host_id="edge2",
    )
    reg = LocaleRegistry()
    reg.register(a)
    reg.register(b)
    return reg, LocaleRouter(reg)


# ──────────────────────────────────────────────────────────────────────────────
#  Synthetic trajectories
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class Trajectory:
    name: str
    positions: List[Tuple[float, float]]
    dt_s: float = 0.05  # 20 Hz


def traj_straight(n_ticks: int = 31) -> Trajectory:
    """Drive x from 0 to 30 in a straight line at constant 1 m/tick."""
    return Trajectory(
        name="straight",
        positions=[(float(i), 5.0) for i in range(n_ticks)],
    )


def traj_bounce() -> Trajectory:
    """Hover around the A/B boundary at x=10 to test hysteresis."""
    # Three single-tick excursions into B, then a sustained crossing.
    seq = [
        (5.0, 5.0), (8.0, 5.0), (9.0, 5.0),
        (11.0, 5.0),                       # tick 3: brief into B
        (9.0, 5.0), (8.0, 5.0),            # back to A
        (11.0, 5.0),                       # brief into B again
        (9.0, 5.0), (8.0, 5.0),
        (11.0, 5.0), (12.0, 5.0), (13.0, 5.0),  # sustained -> switch
        (14.0, 5.0), (15.0, 5.0),
    ]
    return Trajectory(name="bounce", positions=seq)


def traj_gap() -> Trajectory:
    """Pass through coverage gap between A ([0,20]) and B ([28,40])."""
    seq = [(float(x), 5.0) for x in range(0, 41, 1)]
    return Trajectory(name="gap", positions=seq)


# ──────────────────────────────────────────────────────────────────────────────
#  Driver
# ──────────────────────────────────────────────────────────────────────────────
def run_scenario(name: str, *, verbose: bool = True) -> List[HandoffEvent]:
    if name == "straight":
        _, router = build_two_locale_deployment()
        traj = traj_straight()
    elif name == "bounce":
        _, router = build_two_locale_deployment()
        traj = traj_bounce()
    elif name == "gap":
        _, router = build_two_locale_with_gap()
        traj = traj_gap()
    else:
        raise ValueError(f"unknown scenario {name!r}")

    events: List[HandoffEvent] = []
    tracker = VehicleLocaleTracker(router, min_dwell_ticks=3)
    tracker.subscribe(events.append)

    if verbose:
        print(f"\n=== scenario: {name} ===")
        print(f"{'tick':>4} {'x':>6} {'y':>6} {'bound':>6}  event")
    for tick, (x, y) in enumerate(traj.positions):
        ev = tracker.update(
            vehicle_id=42, position=(x, y), tick=tick, sim_time_s=tick * traj.dt_s,
        )
        if verbose:
            bound = tracker.locale_for(42) or "-"
            tag = ""
            if ev is not None:
                tag = f"HANDOFF {ev.source_locale_id} -> {ev.destination_locale_id}"
            print(f"{tick:>4} {x:>6.1f} {y:>6.1f} {bound:>6}  {tag}")
    return events


def assertions(events_by_scenario: dict) -> None:
    """Hard-coded oracle for the three scenarios."""
    s = events_by_scenario["straight"]
    assert len(s) == 2, f"straight: expected 2 events, got {len(s)}"
    assert s[0].source_locale_id is None and s[0].destination_locale_id == "A"
    assert s[1].source_locale_id == "A" and s[1].destination_locale_id == "B"

    b = events_by_scenario["bounce"]
    assert len(b) == 2, f"bounce: expected 2 events (initial + sustained), got {len(b)}"
    assert b[0].destination_locale_id == "A"
    assert b[1].source_locale_id == "A" and b[1].destination_locale_id == "B"
    # The switch must happen AFTER the third consecutive B observation, not
    # during the earlier single-tick excursions.
    assert b[1].tick >= 11, f"bounce: switch at tick {b[1].tick} is too early"

    g = events_by_scenario["gap"]
    # Initial bind to A, then handoff to B once the vehicle has dwelled in B
    # for ``min_dwell_ticks`` consecutive ticks after the gap.
    assert len(g) == 2, f"gap: expected 2 events, got {len(g)}"
    assert g[0].destination_locale_id == "A"
    assert g[1].source_locale_id == "A" and g[1].destination_locale_id == "B"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=["straight", "bounce", "gap", "all"],
                    default="all")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    scenarios = ["straight", "bounce", "gap"] if args.scenario == "all" else [args.scenario]
    events_by_scenario = {}
    for s in scenarios:
        events_by_scenario[s] = run_scenario(s, verbose=not args.quiet)

    if args.scenario == "all":
        assertions(events_by_scenario)
        print("\nAll assertions passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
