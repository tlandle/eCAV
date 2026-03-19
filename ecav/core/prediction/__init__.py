"""Trajectory predictor registry.

Usage:
    from ecav.core.prediction import get_predictor
    predictor = get_predictor('linear', {'num_future_steps': 25})
    predictor = get_predictor('smart', {'checkpoint': '...', 'device': 'cuda'})
"""

from ecav.core.prediction.base_predictor import BasePredictor

_PREDICTOR_REGISTRY = {}


def register_predictor(name: str, cls):
    _PREDICTOR_REGISTRY[name.upper()] = cls


def get_predictor(name: str, cfg: dict) -> BasePredictor:
    """Instantiate a predictor by name from config dict."""
    key = name.upper()
    if key not in _PREDICTOR_REGISTRY:
        raise KeyError(
            f"Unknown predictor: {name!r}. "
            f"Available: {list(_PREDICTOR_REGISTRY.keys())}")
    return _PREDICTOR_REGISTRY[key](cfg)


# Register built-in predictors (lazy imports to avoid heavy deps at import time)
def _register_builtins():
    from ecav.core.prediction.linear_predictor_manager import (
        LinearPredictorManager)
    register_predictor('LINEAR', LinearPredictorManager)

    try:
        from ecav.core.prediction.smart_predictor_manager import (
            SMARTPredictorManager3D)
        register_predictor('SMART', SMARTPredictorManager3D)
    except ImportError:
        pass  # SMART deps not installed


_register_builtins()
