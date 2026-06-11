# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Predictive latent migration for multi-locale cooperative prediction.

Module layout:

* :mod:`locale` -- :class:`Locale` (coverage region polygon, containment,
  signed distance, exit prediction).
* :mod:`registry` -- :class:`LocaleRegistry`, :class:`LocaleRouter` for
  area-based vehicle-to-edge assignment.
* :mod:`binding` -- :class:`VehicleLocaleTracker`, :class:`HandoffEvent`, and
  :class:`HandoffManager`: dynamic per-vehicle binding with hysteresis,
  subscriber callbacks, and the handoff decision interface.
* :mod:`payload` -- :class:`MigrationPayload`, :class:`TrackLatent`, and
  :class:`KFState` for the serialized latent state crossing the inter-locale
  link.
* :mod:`link` -- :class:`InterLocaleLink` and :class:`TransferCost`:
  simulated serialization + network cost model for edge-to-edge transfer.
* :mod:`daemon` -- :class:`SequentialMigrationDaemon`: single-process
  hand-off coordinator (mechanism + measurement wired together).
"""

from .binding import HandoffEvent, HandoffManager, VehicleLocaleTracker
from .daemon import SequentialMigrationDaemon
from .link import InterLocaleLink, TransferCost
from .locale import Locale
from .payload import KFState, MigrationPayload, TrackLatent
from .registry import LocaleRegistry, LocaleRouter

__all__ = [
    "HandoffEvent",
    "HandoffManager",
    "InterLocaleLink",
    "KFState",
    "Locale",
    "LocaleRegistry",
    "LocaleRouter",
    "MigrationPayload",
    "SequentialMigrationDaemon",
    "TrackLatent",
    "TransferCost",
    "VehicleLocaleTracker",
]
