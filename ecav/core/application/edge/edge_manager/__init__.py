# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Edge-manager package initialiser.

Makes the individual manager classes directly importable and provides a
simple registry so you can look them up by name.
"""

# --------------------------------------------------------------------------- #
# >>> from ecav.core.application.edge.edge_manager import BM2CPEdge
# --------------------------------------------------------------------------- #
from .edge_manager_base import BaseEdgeManager
from .edge_manager_manuever import ManeuverEdge
from .edge_manager_perception import PerceptionEdge
from .edge_manager_prediction_late_fusion_ab3dmot_linear_predictor import (
    LateFusionEdge,
)
from .edge_manager_worldfusion_ab3dmot_linear_predictor import (
    WorldFusionEdge,
)
from .edge_manager_oracle_ab3dmot_linear_predictor import (
    OracleEdge,
)
from .edge_manager_vips_temporal_alignment import (
    VIPSTemporalEdge,
)
from .edge_manager_sota import SOTAEdge
from .edge_manager_adaptive import AdaptiveEdge
from .edge_manager_worldfusion_ab3dmot_mtr_adaptive import (
    WorldFusionAdaptiveEdge,
)
from .edge_manager_worldfusion_ab3dmot_mtr import WorldFusionMTREdge
from .edge_manager_worldfusion_mamba_mtr import WorldFusionMambaAdaptiveEdge
from .edge_manager_infra_only_ab3dmot_linear_predictor import InfraOnlyEdge
from .edge_manager_cip_ab3dmot_linear_predictor import CIPEdge

__all__ = [
    "BaseEdgeManager",
    "ManeuverEdge",
    "PerceptionEdge",
    "LateFusionEdge",
    "WorldFusionEdge",
    "OracleEdge",
    "VIPSTemporalEdge",
    "SOTAEdge",
    "AdaptiveEdge",
]

# --------------------------------------------------------------------------- #
# optional: quick factory / registry
# --------------------------------------------------------------------------- #
_EDGE_REGISTRY = {
    # yaml "mode" / manager_type → class
    "MANEUVER": ManeuverEdge,
    "PERCEPTION": PerceptionEdge,
    "LATE_FUSION": LateFusionEdge,
    "WORLDFUSION_PRED": WorldFusionEdge,
    "WORLDFUSION": WorldFusionEdge,  # alias
    "VIPS": VIPSTemporalEdge,
    "VIPS_PRED": VIPSTemporalEdge,  # alias
    "ORACLE": OracleEdge,
    "VIPS_TEMPORAL": VIPSTemporalEdge,
    "SOTA_EDGE": SOTAEdge,
    "SOTA": SOTAEdge,
    "ADAPTIVE_EDGE": AdaptiveEdge,
    "ADAPTIVE": AdaptiveEdge,
    "WORLDFUSION_ADAPTIVE": WorldFusionAdaptiveEdge,
    "WORLDFUSION_ADAPTIVE_EDGE": WorldFusionAdaptiveEdge,
    "WORLDFUSION_MTR": WorldFusionMTREdge,
    "WORLDFUSION_MAMBA_ADAPTIVE": WorldFusionMambaAdaptiveEdge,
    "WORLDFUSION_MAMBA_MTR": WorldFusionMambaAdaptiveEdge,  # alias
    "INFRA_ONLY": InfraOnlyEdge,
    "CIP": CIPEdge,
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
