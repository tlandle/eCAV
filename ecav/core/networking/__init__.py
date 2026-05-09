# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Networking package for eCAV cooperative perception.

Shared by all communication paths: V2V sidelink, V2I uplink/downlink, edge.
Provides channel modeling, occlusion detection, and link-level simulation.
"""

try:
    from .occlusion_model import OcclusionResult, compute_occlusion_matrix
except ImportError:
    pass  # carla not available outside simulation

__all__ = ["OcclusionResult", "compute_occlusion_matrix"]
