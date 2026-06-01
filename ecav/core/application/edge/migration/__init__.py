# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Predictive latent migration for multi-locale cooperative prediction.

Module layout:

* :mod:`locale` -- :class:`Locale` (coverage region polygon, containment,
  signed distance, exit prediction).
* :mod:`registry` -- :class:`LocaleRegistry`, :class:`LocaleRouter` for
  area-based vehicle-to-edge assignment.
* :mod:`binding` -- :class:`VehicleLocaleTracker` and :class:`HandoffEvent`:
  dynamic per-vehicle binding with hysteresis, with subscriber callbacks.
* :mod:`payload` -- :class:`MigrationPayload`, :class:`TrackLatent` for the
  serialized latent state crossing the inter-locale link.

The trajectory trigger, inter-locale link model, and migration daemon are
forthcoming.
"""

from .binding import HandoffEvent, VehicleLocaleTracker
from .locale import Locale
from .payload import MigrationPayload, TrackLatent
from .registry import LocaleRegistry, LocaleRouter

__all__ = [
    "HandoffEvent",
    "Locale",
    "LocaleRegistry",
    "LocaleRouter",
    "MigrationPayload",
    "TrackLatent",
    "VehicleLocaleTracker",
]
