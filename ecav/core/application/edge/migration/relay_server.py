# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
"""MigrationRelay: second-process gRPC endpoint for real-bytes transfers.

Run: python -m ecav.core.application.edge.migration.relay_server [port]
The relay receives serialized MigrationPayload chunks (PREPARE/COMMIT),
acknowledges with the epoch, and keeps per-actor last-epoch state so
duplicate/stale sends are visible in its log. Timing is measured at the
CLIENT (daemon side); the relay stays minimal so loopback+netem dominates.
"""
import logging
import sys
import time
from concurrent import futures

import grpc

sys.path.insert(0, 'ecav/protos')
import migration_pb2          # noqa: E402
import migration_pb2_grpc     # noqa: E402

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("MigrationRelay")


class Relay(migration_pb2_grpc.MigrationRelayServicer):
    def __init__(self):
        self.last_epoch = {}

    def _handle(self, kind, request):
        prev = self.last_epoch.get(request.actor_id, -1)
        if request.epoch <= prev:
            logger.warning("%s stale epoch actor=%d %d<=%d",
                           kind, request.actor_id, request.epoch, prev)
        self.last_epoch[request.actor_id] = max(prev, request.epoch)
        return migration_pb2.Ack(ok=True, epoch=request.epoch)

    def Prepare(self, request, context):
        return self._handle("PREPARE", request)

    def Commit(self, request, context):
        return self._handle("COMMIT", request)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 50771
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    migration_pb2_grpc.add_MigrationRelayServicer_to_server(Relay(), server)
    server.add_insecure_port(f"127.0.0.1:{port}")
    server.start()
    logger.info("MigrationRelay listening on 127.0.0.1:%d", port)
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        server.stop(0)


if __name__ == "__main__":
    main()
