"""
EdgeFusionClient: gRPC client for Edge_PerformFusion in edge-only distributed mode.

ecav.py creates one client per registered edge container. The tick loop calls
fuse() once per tick as a blocking call; the edge container does fusion,
tracking, and prediction, then returns FusionResult.pickled_predictions.
"""

import time
import logging

import grpc

logger = logging.getLogger(__name__)


class EdgeFusionClient:
    """
    Wraps the Edge_PerformFusion and Edge_EndScenario RPCs for one edge container.

    Usage:
        client = EdgeFusionClient(edge_ip="127.0.0.1", edge_port=50054)
        client.connect(retry_timeout_s=60.0)
        result = client.fuse(tick_id=0, batch=my_batch)
        eval_result = client.end_scenario()
        client.close()
    """

    def __init__(self, edge_ip: str, edge_port: int, edge_index: int = 0):
        self.edge_ip = edge_ip
        self.edge_port = edge_port
        self.edge_index = edge_index
        self._channel = None
        self._stub = None

    def connect(self, retry_timeout_s: float = 60.0):
        """
        Open gRPC channel and retry until the edge server is reachable or timeout expires.
        Raises RuntimeError if the server is not up within retry_timeout_s.
        """
        import ecav.protos.ecloud_pb2 as ecloud
        import ecav.protos.ecloud_pb2_grpc as ecloud_rpc

        addr = f"{self.edge_ip}:{self.edge_port}"
        self._channel = grpc.insecure_channel(addr)
        self._stub = ecloud_rpc.EcloudStub(self._channel)

        logger.info("EdgeFusionClient[%d]: connecting to %s (timeout %.0fs)",
                    self.edge_index, addr, retry_timeout_s)

        deadline = time.monotonic() + retry_timeout_s
        while True:
            try:
                self._stub.Edge_PerformFusion(
                    ecloud.IntermediateFeaturesBatch(tick_id=-1),
                    timeout=2.0
                )
                # A valid response (even an error tick_id) means the server is up.
                break
            except grpc.RpcError as e:
                if e.code() in (grpc.StatusCode.UNAVAILABLE, grpc.StatusCode.DEADLINE_EXCEEDED):
                    if time.monotonic() > deadline:
                        raise RuntimeError(
                            f"EdgeFusionClient[{self.edge_index}]: "
                            f"edge at {addr} not reachable after {retry_timeout_s:.0f}s"
                        ) from e
                    logger.debug("EdgeFusionClient[%d]: waiting for edge at %s …",
                                 self.edge_index, addr)
                    time.sleep(1.0)
                else:
                    # Any other status code means the server IS up (just rejected the ping).
                    break

        logger.info("EdgeFusionClient[%d]: connected to %s", self.edge_index, addr)

    def fuse(self, tick_id: int, batch) -> object:
        """
        Call Edge_PerformFusion with the given IntermediateFeaturesBatch.
        Blocking. Returns FusionResult.
        """
        if self._stub is None:
            raise RuntimeError("EdgeFusionClient: call connect() before fuse()")
        return self._stub.Edge_PerformFusion(batch, timeout=30.0)

    def end_scenario(self) -> object:
        """
        Call Edge_EndScenario. Returns EdgeEvaluationResult.
        """
        import ecav.protos.ecloud_pb2 as ecloud
        if self._stub is None:
            return None
        try:
            return self._stub.Edge_EndScenario(ecloud.Empty(), timeout=30.0)
        except grpc.RpcError as e:
            logger.warning("EdgeFusionClient[%d]: Edge_EndScenario failed: %s",
                           self.edge_index, e.details())
            return None

    def close(self):
        if self._channel is not None:
            self._channel.close()
            self._channel = None
            self._stub = None
