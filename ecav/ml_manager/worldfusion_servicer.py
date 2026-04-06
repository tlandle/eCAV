# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
gRPC servicer for WorldFusion intermediate feature extraction.

Implements ExtractWfFeatures on PerceptionService. Runs in the same process
as the gRPC server — no IPC, no subprocess, no multiprocessing.Manager queues.
Inference runs directly in the handler thread.
"""
import time
import zlib

import msgpack
import msgpack_numpy as m_np

import perception_pb2
import perception_pb2_grpc

m_np.patch()


class WorldFusionServicer(perception_pb2_grpc.PerceptionServiceServicer):

    def __init__(self, extract_fn):
        """
        Parameters
        ----------
        extract_fn : callable
            (_extract_wf_features from litserve_models.py)
            Signature: (batch_numpy_dict) -> {'spatial_features': np.ndarray float16}
        """
        self._extract = extract_fn

    def ExtractWfFeatures(self, request, context):
        t0 = time.time()

        try:
            body = zlib.decompress(request.payload)
        except zlib.error:
            body = request.payload
        batch = msgpack.unpackb(body, raw=False)
        t1 = time.time()

        result = self._extract(batch)
        t2 = time.time()

        packed = msgpack.packb(result, use_bin_type=True)
        t3 = time.time()

        unpack_ms  = (t1 - t0) * 1000
        infer_ms   = (t2 - t1) * 1000
        pack_ms    = (t3 - t2) * 1000
        total_ms   = (t3 - t0) * 1000

        print(
            f"[WorldFusionServicer] actor={request.actor_id} "
            f"batch={result['spatial_features'].shape[0]} "
            f"unpack={unpack_ms:.0f}ms "
            f"inference={infer_ms:.0f}ms "
            f"pack={pack_ms:.0f}ms "
            f"total={total_ms:.0f}ms",
            flush=True,
        )

        return perception_pb2.WfResponse(
            payload=packed,
            unpack_ms=unpack_ms,
            inference_ms=infer_ms,
            pack_ms=pack_ms,
            total_ms=total_ms,
        )

    # DetectYolo is not implemented on this servicer (WF-only server, port 18002).
    # Inherited UNIMPLEMENTED behaviour from the base class is correct.
