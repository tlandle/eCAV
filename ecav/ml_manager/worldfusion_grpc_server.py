# -*- coding: utf-8 -*-
# Author: Jordan Rapp <jrapp@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Standalone gRPC server for WorldFusion intermediate feature extraction.

Loads the WorldFusion model once in-process (no subprocess fork, no IPC).
Serves ExtractWfFeatures on PerceptionService at WF_GRPC_PORT (default 18002).

Expected latency: grpc_ms ≈ server_total_ms ≈ 75-85ms
(vs ~185ms via LitServe due to multiprocessing.Manager IPC overhead).

Usage:
    python worldfusion_grpc_server.py [--port 18002] [--workers 4]

Profile mode (measure GPU throughput, same as litserve_models.py --profile):
    python worldfusion_grpc_server.py --profile
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))  # for litserve_models, worldfusion_servicer
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'ecav', 'protos'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from concurrent import futures

import grpc
import perception_pb2_grpc

from litserve_models import _extract_wf_features, _run_profile, load_wf_model
from worldfusion_servicer import WorldFusionServicer

_32MB = 32 * 1024 * 1024
_GRPC_OPTIONS = [
    ("grpc.max_send_message_length",    _32MB),
    ("grpc.max_receive_message_length", _32MB),
]


def serve(port: int, max_workers: int) -> None:
    load_wf_model()

    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=max_workers),
        options=_GRPC_OPTIONS,
    )
    perception_pb2_grpc.add_PerceptionServiceServicer_to_server(
        WorldFusionServicer(_extract_wf_features), server
    )
    server.add_insecure_port(f"[::]:{port}")
    server.start()
    print(f"[WorldFusionGrpcServer] Listening on port {port} (workers={max_workers})", flush=True)
    server.wait_for_termination()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WorldFusion gRPC server")
    parser.add_argument("--port",    type=int, default=int(os.environ.get("WF_GRPC_PORT", 18002)))
    parser.add_argument("--workers", type=int, default=int(os.environ.get("WF_GRPC_WORKERS", 4)))
    parser.add_argument("--profile", action="store_true", help="Run GPU throughput profile and exit")
    args = parser.parse_args()

    if args.profile:
        _run_profile()
    else:
        serve(args.port, args.workers)
