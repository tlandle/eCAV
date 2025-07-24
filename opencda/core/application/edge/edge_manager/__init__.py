
# -*- coding: utf-8 -*-
"""
Edge-manager package initialiser.

It makes the individual manager classes directly importable and provides a
simple registry so you can look them up by name (e.g. the “mode” field in your
YAML).
"""

# --------------------------------------------------------------------------- #
# >>> from opencda.core.application.edge.edge_manager import BM2CPEdge
# --------------------------------------------------------------------------- #
from .edge_manager_base import BaseEdgeManager
from .edge_manager_manuever import ManeuverEdge
from .edge_manager_perception import PerceptionEdge
from .edge_manager_prediction_bm2cp_ab3dmot_linear_predictor import (
    BM2CPEdge,
)
from .edge_manager_prediction_late_fusion_ab3dmot_linear_predictor import (
    LateFusionEdge,
)

__all__ = [
    "BaseEdgeManager",
    "ManeuverEdge",
    "PerceptionEdge",
    "BM2CPEdge",
    "LateFusionEdge",
]

# --------------------------------------------------------------------------- #
# optional: quick factory / registry
# --------------------------------------------------------------------------- #
_EDGE_REGISTRY = {
    # yaml “mode” / manager_type → class
    "MANEUVER": ManeuverEdge,
    "PERCEPTION": PerceptionEdge,
    "BM2CP_PRED": BM2CPEdge,
    "LATE_FUSION": LateFusionEdge,
}


def get_edge_class(name: str):
    """
    Return the edge-manager class that corresponds to *name*.

    Example
    -------
    >>> cls = get_edge_class("bm2cp_pred")
    >>> edge = cls(world, cfg, cav_world, client)
    """
    key = name.upper()
    if key not in _EDGE_REGISTRY:
        raise KeyError(f"Unknown edge-manager type: {name}")
    return _EDGE_REGISTRY[key]
