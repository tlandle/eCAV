# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""VehicleLocaleTracker: dynamic vehicle-to-locale binding with hysteresis.

The tracker holds the current locale assignment for each managed vehicle
and updates the assignment from periodic position observations. The
orchestrator calls :meth:`update` once per tick (or every ``cadence_ticks``
ticks for cheaper polling). When a vehicle's locale changes, the tracker
emits a :class:`HandoffEvent` that downstream subscribers (e.g., the
migration daemon) react to.

Two design points address boundary-flap and gap conditions:

* **Hysteresis.** A vehicle near a locale boundary may oscillate between
  locales due to detection noise. The tracker requires the vehicle to remain
  in the candidate destination locale for ``min_dwell_ticks`` consecutive
  observations before the binding switches. The default keeps the prior
  binding for short excursions into a neighbor's polygon.
* **Coverage gap.** When the router returns ``None`` (no locale contains the
  vehicle), the tracker holds the prior binding rather than detaching, on
  the assumption that the vehicle has briefly exited mapped coverage and
  will return.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

import numpy as np

from .locale import Locale
from .registry import LocaleRouter

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class HandoffEvent:
    """Emitted when a vehicle's locale binding changes.

    ``source_locale_id`` is ``None`` if the vehicle had no prior binding
    (first observation). Subscribers use this event to drive the migration
    protocol or to retarget the vehicle's uplink.
    """

    vehicle_id: int
    source_locale_id: Optional[str]
    destination_locale_id: str
    tick: int
    sim_time_s: float


@dataclass
class _Binding:
    locale_id: str
    dwell_ticks: int = 0  # consecutive observations confirming this locale
    candidate_locale_id: Optional[str] = None
    candidate_dwell: int = 0


class VehicleLocaleTracker:
    """Holds the current locale binding for each managed vehicle."""

    def __init__(
        self,
        router: LocaleRouter,
        *,
        min_dwell_ticks: int = 4,
        cadence_ticks: int = 1,
    ) -> None:
        self._router = router
        self._min_dwell = max(1, int(min_dwell_ticks))
        self._cadence = max(1, int(cadence_ticks))
        self._bindings: Dict[int, _Binding] = {}
        self._subscribers: List[Callable[[HandoffEvent], None]] = []

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def subscribe(self, cb: Callable[[HandoffEvent], None]) -> None:
        """Register a callback fired on every locale binding change."""
        self._subscribers.append(cb)

    # ------------------------------------------------------------------
    # Update path
    # ------------------------------------------------------------------
    def update(
        self,
        vehicle_id: int,
        position: Sequence[float],
        tick: int,
        sim_time_s: float,
    ) -> Optional[HandoffEvent]:
        """Refresh the binding for one vehicle. Returns a HandoffEvent if the
        binding changed this call, otherwise ``None``."""
        if tick % self._cadence != 0:
            return None

        observed = self._router.locale_for(position)
        prior = self._bindings.get(vehicle_id)

        # No coverage: hold prior binding
        if observed is None:
            if prior is None:
                return None
            prior.dwell_ticks += 1
            return None

        # First observation: bind immediately and emit
        if prior is None:
            self._bindings[vehicle_id] = _Binding(locale_id=observed.locale_id, dwell_ticks=1)
            return self._emit(vehicle_id, None, observed.locale_id, tick, sim_time_s)

        # Same locale: refresh dwell, clear any pending candidate
        if observed.locale_id == prior.locale_id:
            prior.dwell_ticks += 1
            prior.candidate_locale_id = None
            prior.candidate_dwell = 0
            return None

        # Different locale: enter or extend a candidate period
        if prior.candidate_locale_id == observed.locale_id:
            prior.candidate_dwell += 1
        else:
            prior.candidate_locale_id = observed.locale_id
            prior.candidate_dwell = 1

        # Switch when hysteresis condition met
        if prior.candidate_dwell >= self._min_dwell:
            src = prior.locale_id
            dst = observed.locale_id
            self._bindings[vehicle_id] = _Binding(locale_id=dst, dwell_ticks=1)
            return self._emit(vehicle_id, src, dst, tick, sim_time_s)

        return None

    # ------------------------------------------------------------------
    # Inspection
    # ------------------------------------------------------------------
    def locale_for(self, vehicle_id: int) -> Optional[str]:
        b = self._bindings.get(vehicle_id)
        return b.locale_id if b else None

    def all_bindings(self) -> Dict[int, str]:
        return {vid: b.locale_id for vid, b in self._bindings.items()}

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _emit(
        self,
        vehicle_id: int,
        src: Optional[str],
        dst: str,
        tick: int,
        sim_time_s: float,
    ) -> HandoffEvent:
        event = HandoffEvent(
            vehicle_id=vehicle_id,
            source_locale_id=src,
            destination_locale_id=dst,
            tick=tick,
            sim_time_s=sim_time_s,
        )
        for cb in self._subscribers:
            try:
                cb(event)
            except Exception:  # noqa: BLE001
                logger.exception("Handoff subscriber raised")
        logger.info(
            "Handoff vehicle=%d %s -> %s at tick=%d (%.2fs)",
            vehicle_id, src, dst, tick, sim_time_s,
        )
        return event
