# Shared perception service: standalone throughput per GPU replica (carla-ndm, 8 x V100)

Service: `ecav/ml_manager/yolo_grpc_server.py` (YOLOv5m, weights sha256 61d93336...), four worker threads per replica, one replica per GPU, ports 18001 to 18008.
Request: the call a `scale_town06` vehicle client makes each tick, four 640x480 JPEG frames (quality 85, 412 KB). Eight closed-loop clients per replica, 25 s after 5 s warm-up (`scripts/ispass/perception_service_bench.py`).

| replicas (GPUs) | total requests per s | per replica (min to max) | latency p50 / p99 at 8 clients per replica (ms) | errors |
|---|---|---|---|---|
| 1 | 36.6 | 36.6 to 36.6 | 218 / 227 | 0 |
| 2 | 73.8 | 36.6 to 37.1 | 218 / 225 | 0 |
| 4 | 146.9 | 36.3 to 37.0 | 220 / 226 | 0 |
| 8 | 291.1 | 36.0 to 37.1 | 222 / 231 | 0 |

One replica against the number of closed-loop clients: 26.1 requests per s at 1 client (p50 38 ms), 37.0 at 4 (109 ms), 36.6 at 8 (218 ms), 35.4 at 16 (451 ms). The replica saturates at four clients, the number of worker threads. GPU utilisation at saturation is 43 % with one replica and 55 to 77 % per GPU with eight.

Across the peering, from carla-client3: 36.6 requests per s on one replica and 290.7 on eight (64 clients), round-trip time 5.4 ms. The network path does not lower the ceiling.

A10 node, standalone, same request and load generator (one replica, four worker threads, GPU otherwise idle, measured 2026-11-13):

| closed-loop clients | requests per s | latency p50 / p99 (ms) | errors |
|---|---|---|---|
| 1 | 37.4 | 27 / 27 | 0 |
| 4 | 40.4 | 99 / 104 | 0 |
| 8 | 40.5 | 199 / 205 | 0 |
| 16 | 40.3 | 398 / 405 | 0 |

The A10 replica saturates at four clients at 40.4 requests per s (mean GPU utilisation 60 % over the four loads), against 37.0 requests per s for one V100 replica on carla-ndm. One A10 therefore serves about 1.1 times one V100 with this service. The 21.6 requests per s that the single-host sweep3 perception rows report is an in-run rate limited by the tick, about half of the standalone ceiling of the same GPU.
