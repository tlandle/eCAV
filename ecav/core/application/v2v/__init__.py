# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
V2V (vehicle-to-vehicle) manager package.

Contains cooperative perception managers that operate in V2V mode
(peer-to-peer between vehicles) rather than through an edge server.
"""

from .v2v_fusion import V2VFeatureExchange
from .v2v_metrics import V2VMetrics

__all__ = ["V2VCooperativeManager", "V2VFeatureExchange", "V2VMetrics"]

def __getattr__(name):
    if name == "V2VCooperativeManager":
        from .v2v_manager import V2VCooperativeManager
        return V2VCooperativeManager
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
