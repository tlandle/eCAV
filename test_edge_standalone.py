"""
Phase 1 smoke test: start edge process in --standalone mode and call Edge_PerformFusion.

Usage (two terminals):

  Terminal 1 — start the edge:
    conda activate opencda
    python ecav/ecav2/edge_process.py \
        --standalone \
        --config ecav/scenario_testing/config_yaml/openscenario_3_edge_worldfusion.yaml \
        --edge_index 0 \
        --verbose

  Terminal 2 — run this script:
    conda activate opencda
    python test_edge_standalone.py

The edge process must print "standalone ready" before running this script.
"""

import sys
import grpc

sys.path.insert(0, ".")
import ecav.protos.ecloud_pb2 as ecloud
import ecav.protos.ecloud_pb2_grpc as ecloud_rpc

EDGE_ADDR = "localhost:50054"


def run():
    channel = grpc.insecure_channel(EDGE_ADDR)
    stub = ecloud_rpc.EcloudStub(channel)

    # Tick 0: empty feature batch (no RSU, no vehicle).
    # run_step() will see no features in the jitter buffer and return
    # _update_agents(0, None) with empty predictions. That is the expected path
    # when the edge is healthy but has nothing to fuse yet.
    req = ecloud.IntermediateFeaturesBatch(tick_id=0)
    print(f"Sending Edge_PerformFusion(tick_id=0, features=[]) to {EDGE_ADDR} ...")

    try:
        result = stub.Edge_PerformFusion(req, timeout=30.0)
    except grpc.RpcError as e:
        print(f"FAIL — gRPC error: {e.code()}: {e.details()}")
        sys.exit(1)

    assert result.tick_id == 0, f"Expected tick_id=0, got {result.tick_id}"
    print(f"OK  — FusionResult(tick_id={result.tick_id}, pickled_predictions={len(result.pickled_predictions)} bytes)")

    # Tick 1: duplicate of tick 0 — should return cached result (idempotency check).
    req_dup = ecloud.IntermediateFeaturesBatch(tick_id=0)
    print("Sending duplicate tick_id=0 (idempotency check) ...")
    try:
        result_dup = stub.Edge_PerformFusion(req_dup, timeout=10.0)
    except grpc.RpcError as e:
        print(f"FAIL — gRPC error on duplicate: {e.code()}: {e.details()}")
        sys.exit(1)

    assert result_dup.tick_id == 0, f"Expected cached tick_id=0, got {result_dup.tick_id}"
    print(f"OK  — cached result returned: tick_id={result_dup.tick_id}")

    # End scenario.
    print("Sending Edge_EndScenario ...")
    try:
        eval_result = stub.Edge_EndScenario(ecloud.Empty(), timeout=10.0)
    except grpc.RpcError as e:
        print(f"FAIL — gRPC error on Edge_EndScenario: {e.code()}: {e.details()}")
        sys.exit(1)

    print(f"OK  — EdgeEvaluationResult(edge_index={eval_result.edge_index}, "
          f"profiler={len(eval_result.pickled_edge_profiler)} bytes)")
    print("\nSmoke test PASSED.")


if __name__ == "__main__":
    run()
