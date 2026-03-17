"""Multi-object tracker registry.

Usage:
    from ecav.core.tracking import get_tracker
    tracker = get_tracker('ab3dmot', {'min_hits': 3, 'max_age': 6})
"""

from ecav.core.tracking.base_tracker import BaseTracker

_TRACKER_REGISTRY = {}


def register_tracker(name: str, cls):
    _TRACKER_REGISTRY[name.upper()] = cls


def get_tracker(name: str, cfg: dict) -> BaseTracker:
    """Instantiate a tracker by name from config dict."""
    key = name.upper()
    if key not in _TRACKER_REGISTRY:
        raise KeyError(
            f"Unknown tracker: {name!r}. "
            f"Available: {list(_TRACKER_REGISTRY.keys())}")
    return _TRACKER_REGISTRY[key](cfg)


def _register_builtins():
    from ecav.core.tracking.ab3dmot_wrapper import AB3DMOTWrapper
    register_tracker('AB3DMOT', AB3DMOTWrapper)


_register_builtins()
