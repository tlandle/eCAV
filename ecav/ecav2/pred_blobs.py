"""Wire form of a recipient's forecast list in which every forecast is serialised once.

A recipient's buffer was pickle.dumps(list of forecasts), built per recipient, so an edge
with N recipients traversed N*(N-1) forecast objects per tick (48 % of the edge step at 64
vehicles). pack() takes forecasts that were pickled once each and concatenates the bytes
of the ones a recipient receives. unpack() reads both this form and the plain pickled list.
"""
import pickle

_TAG = "__pred_blobs__"


def pack(blobs):
    return pickle.dumps((_TAG, blobs), protocol=pickle.HIGHEST_PROTOCOL)


def unpack(buf):
    obj = pickle.loads(buf)
    if isinstance(obj, tuple) and len(obj) == 2 and obj[0] == _TAG:
        return [pickle.loads(b) for b in obj[1]]
    return obj
