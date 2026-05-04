# -*- coding: utf-8 -*-
# License: TDG-Attribution-NonCommercial-NoDistrib
"""General-purpose utility functions for the eCAV package."""

import pickle


def find_unpicklable(obj, path=""):
    """Recursively find the first unpicklable object in obj's attribute tree.

    Returns the unpicklable object, or None if obj is fully picklable.
    Callers are responsible for logging the result.
    """
    try:
        pickle.dumps(obj)
        return None
    except Exception:
        if hasattr(obj, '__dict__'):
            for key, value in obj.__dict__.items():
                result = find_unpicklable(value, f"{path}.{key}")
                if result is not None:
                    return result
        return obj
