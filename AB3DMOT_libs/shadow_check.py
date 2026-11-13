"""Per-call differential check of the tracker against an earlier version of model.py.

With AB3D_SHADOW_PREV=<path to the earlier model.py> every AB3DMOT.track() call is run twice
from the same state: live on the current code, and on a deep copy of the tracker rebound to the
earlier class. The published rows and the state after the call (filter state and covariance,
hit counters, identities, position and edge histories, held velocity) are compared for exact
equality, and one line per call goes to AB3D_SHADOW_LOG (default /tmp/ab3d_shadow_<pid>.log).
Because each call starts from the live state, equality on every call means the run is the one
the earlier code would have produced. Off unless the variable is set.
"""
import copy
import os
import types

import numpy as np

_prev_cls = None
_log = None


def _prev_class(path):
    global _prev_cls
    if _prev_cls is None:
        m = types.ModuleType("ab3dmot_model_prev")
        m.__file__ = path
        with open(path) as fh:
            exec(compile(fh.read(), path, "exec"), m.__dict__)
        _prev_cls = m.AB3DMOT
    return _prev_cls


def _rows(out):
    return np.asarray(out[0]) if out and len(out[0]) else np.zeros((0, 0))


def _state(trk):
    st = []
    for t in trk.trackers:
        st.append((int(t.id), int(t.hits), int(t.time_since_update), int(getattr(t, "carla_id", -1)),
                   np.array(t.kf.x, dtype=float).tobytes(), np.array(t.kf.P, dtype=float).tobytes(),
                   repr(getattr(t, "pos_hist", None)), repr(getattr(t, "edge_hist", None)),
                   None if getattr(t, "pub_vel", None) is None else np.array(t.pub_vel, dtype=float).tobytes(),
                   int(getattr(t, "rest_frames", 0))))
    return st, int(trk.frame_count), list(trk.ID_count)


def shadow_track(trk, dets_all, frame, prev_path):
    global _log
    if _log is None:
        _log = open(os.environ.get("AB3D_SHADOW_LOG", f"/tmp/ab3d_shadow_{os.getpid()}.log"), "a", buffering=1)
    try:
        shadow = copy.deepcopy(trk)
    except Exception:
        shadow = copy.copy(trk)
        for k, v in trk.__dict__.items():
            try:
                shadow.__dict__[k] = copy.deepcopy(v)
            except Exception:
                shadow.__dict__[k] = v
    shadow.__class__ = _prev_class(prev_path)
    d_prev = copy.deepcopy(dets_all)
    trk._shadow_busy = True
    try:
        out = trk.track(dets_all, frame)
    finally:
        trk._shadow_busy = False
    out_prev = shadow.track(d_prev, frame)
    a, b = _rows(out), _rows(out_prev)
    rows_eq = a.shape == b.shape and np.array_equal(a, b)
    state_eq = _state(trk) == _state(shadow)
    _log.write(f"frame={frame} stride={getattr(trk, '_stride', '?')} dets={len(dets_all['dets'])} "
               f"rows={a.shape[0] if a.ndim == 2 else 0} trackers={len(trk.trackers)} "
               f"rows_equal={int(rows_eq)} state_equal={int(state_eq)}\n")
    return out
