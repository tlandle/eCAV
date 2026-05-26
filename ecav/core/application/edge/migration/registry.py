# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""LocaleRegistry and LocaleRouter: area-based vehicle-to-edge assignment.

The registry holds the deployment's set of locales. The router answers, for
any position in the shared map frame, which locale a vehicle there belongs
to. This is the basis of dynamic vehicle-to-edge assignment: rather than
binding each vehicle to a specific edge at spawn time, the orchestrator
periodically asks the router which locale (and therefore which edge host)
currently serves each vehicle.

The router supports two ambiguity cases:

* The vehicle's position falls inside multiple locale polygons (overlapping
  coverage). The router returns the locale whose centroid is nearest, with a
  deterministic tiebreak on ``locale_id``.
* The vehicle's position falls outside every locale polygon (gap in
  coverage). The router returns ``None``; the caller's binding policy
  decides whether to hold the prior assignment or detach.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .locale import Locale

logger = logging.getLogger(__name__)


class LocaleRegistry:
    """In-memory catalog of locales for a deployment."""

    def __init__(self) -> None:
        self._locales: Dict[str, Locale] = {}

    def register(self, locale: Locale) -> None:
        if locale.locale_id in self._locales:
            raise ValueError(f"Locale {locale.locale_id!r} already registered")
        self._locales[locale.locale_id] = locale
        logger.info("Registered locale %s (edge_host=%s)", locale.locale_id, locale.edge_host_id)

    def get(self, locale_id: str) -> Locale:
        return self._locales[locale_id]

    def all(self) -> List[Locale]:
        return list(self._locales.values())

    def __len__(self) -> int:
        return len(self._locales)

    def __contains__(self, locale_id: str) -> bool:
        return locale_id in self._locales


class LocaleRouter:
    """Maps positions to locales using the deployment's registry."""

    def __init__(self, registry: LocaleRegistry) -> None:
        self._registry = registry
        self._centroids: Dict[str, np.ndarray] = {
            loc.locale_id: loc.polygon.mean(axis=0) for loc in registry.all()
        }

    def refresh_centroids(self) -> None:
        """Recompute polygon centroids; call after registering new locales."""
        self._centroids = {
            loc.locale_id: loc.polygon.mean(axis=0) for loc in self._registry.all()
        }

    def locale_for(self, position: Sequence[float]) -> Optional[Locale]:
        """Return the locale that currently serves a vehicle at ``position``.

        Overlapping locales are resolved by nearest centroid, with a
        deterministic tiebreak on ``locale_id``. No locale match returns
        ``None``.
        """
        x, y = float(position[0]), float(position[1])
        containing = [loc for loc in self._registry.all() if loc.contains((x, y))]
        if not containing:
            return None
        if len(containing) == 1:
            return containing[0]
        # Resolve overlap: nearest centroid, then deterministic id tiebreak
        def _rank(loc: Locale) -> Tuple[float, str]:
            cx, cy = self._centroids[loc.locale_id]
            return (float(np.hypot(x - cx, y - cy)), loc.locale_id)
        containing.sort(key=_rank)
        return containing[0]
