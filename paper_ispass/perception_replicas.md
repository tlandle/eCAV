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

Reference on the A10 node: the single-host sweep3 perception rows report about 21.6 requests per wall-clock second inside a run (service p50 145 ms). That figure is an in-run rate and is limited by the tick, so it is not a standalone ceiling. The standalone A10 ceiling has not been measured with this load generator.
