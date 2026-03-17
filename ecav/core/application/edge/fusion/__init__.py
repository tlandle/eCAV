"""Edge fusion backend registry.

Usage:
    from ecav.core.application.edge.fusion import get_fusion
    backend = get_fusion('late_fusion', cfg)
    backend = get_fusion('worldfusion', cfg)
    backend = get_fusion('oracle', cfg, world=carla_world)
"""

from ecav.core.application.edge.fusion.base_fusion import BaseFusionBackend

_FUSION_REGISTRY = {}


def register_fusion(name: str, cls):
    _FUSION_REGISTRY[name.upper()] = cls


def get_fusion(name: str, cfg: dict, **kwargs) -> BaseFusionBackend:
    """Instantiate a fusion backend by name."""
    key = name.upper()
    if key not in _FUSION_REGISTRY:
        raise KeyError(
            f"Unknown fusion backend: {name!r}. "
            f"Available: {list(_FUSION_REGISTRY.keys())}")
    return _FUSION_REGISTRY[key](cfg, **kwargs)


def _register_builtins():
    from ecav.core.application.edge.fusion.late_fusion_backend import (
        LateFusionBackend)
    register_fusion('LATE_FUSION', LateFusionBackend)

    from ecav.core.application.edge.fusion.oracle_fusion_backend import (
        OracleFusionBackend)
    register_fusion('ORACLE', OracleFusionBackend)

    try:
        from ecav.core.application.edge.fusion.intermediate_fusion_backend import (
            IntermediateFusionBackend)
        register_fusion('WORLDFUSION', IntermediateFusionBackend)
        register_fusion('INTERMEDIATE', IntermediateFusionBackend)
    except ImportError:
        pass  # WorldFusion deps (opencood) not installed


_register_builtins()
