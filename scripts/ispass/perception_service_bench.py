#!/usr/bin/env python
"""Standalone load test of the shared YOLO perception service (ecav/ml_manager/yolo_grpc_server.py).

Closed-loop clients, each sending the request a scale_town06 vehicle client sends every tick: four
640x480 camera frames as JPEG (quality 85) in one DetectYolo call. Client k uses endpoint k mod R,
the rule the scale launcher applies. Reports requests per second and latency per endpoint and in total.

    python scripts/ispass/perception_service_bench.py --endpoints 10.2.0.6:18001,10.2.0.6:18002 \
        --clients 16 --seconds 30 [--image yolov5/data/images/bus.jpg] [--json out.json]
"""
import argparse
import json
import os
import sys
import threading
import time

import cv2
import grpc
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
for p in (ROOT, os.path.join(ROOT, "ecav", "protos")):
    if p not in sys.path:
        sys.path.insert(0, p)
import perception_pb2  # noqa: E402
import perception_pb2_grpc  # noqa: E402

_16MB = 16 * 1024 * 1024


def frames(image, n=4, w=640, h=480):
    img = cv2.imread(image) if image and os.path.exists(image) else None
    if img is None:
        rng = np.random.default_rng(0)
        img = cv2.GaussianBlur(rng.integers(0, 255, (h, w, 3), dtype=np.uint8), (0, 0), 6)
    img = cv2.resize(img, (w, h))
    out = []
    for i in range(n):
        ok, buf = cv2.imencode(".jpg", np.roll(img, 40 * i, axis=1), [cv2.IMWRITE_JPEG_QUALITY, 85])
        assert ok
        out.append(buf.tobytes())
    return out


def worker(k, endpoint, payload, t_end, warm, lat):
    ch = grpc.insecure_channel(endpoint, options=[("grpc.max_send_message_length", _16MB),
                                                  ("grpc.max_receive_message_length", _16MB)])
    stub = perception_pb2_grpc.PerceptionServiceStub(ch)
    req = perception_pb2.YoloRequest(jpeg_frames=payload, actor_id=k)
    while time.time() < t_end:
        t0 = time.time()
        try:
            stub.DetectYolo(req, timeout=60)
        except grpc.RpcError as e:
            lat.append((time.time(), None, str(e.code())))
            time.sleep(0.2)
            continue
        if t0 >= warm:
            lat.append((time.time(), (time.time() - t0) * 1000.0, None))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True, help="comma-separated host:port list")
    ap.add_argument("--clients", type=int, required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--image", default=os.path.join(ROOT, "yolov5", "data", "images", "bus.jpg"))
    ap.add_argument("--json", default="")
    a = ap.parse_args()
    eps = [e.strip() for e in a.endpoints.split(",") if e.strip()]
    payload = frames(a.image)
    t0 = time.time()
    warm, t_end = t0 + a.warmup, t0 + a.warmup + a.seconds
    lats = [[] for _ in range(a.clients)]
    th = [threading.Thread(target=worker, args=(k, eps[k % len(eps)], payload, t_end, warm, lats[k]), daemon=True)
          for k in range(a.clients)]
    for t in th:
        t.start()
    for t in th:
        t.join()
    res = {"endpoints": eps, "clients": a.clients, "seconds": a.seconds,
           "request_bytes": sum(len(b) for b in payload), "per_endpoint": {}}
    allok, errors = [], 0
    for i, ep in enumerate(eps):
        ok = sorted(x[1] for k in range(a.clients) if k % len(eps) == i for x in lats[k] if x[1] is not None)
        errors += sum(1 for k in range(a.clients) if k % len(eps) == i for x in lats[k] if x[1] is None)
        allok += ok
        if ok:
            q = lambda p: ok[min(len(ok) - 1, int(len(ok) * p))]  # noqa: E731
            res["per_endpoint"][ep] = {"req_per_s": len(ok) / a.seconds, "p50_ms": q(.5), "p95_ms": q(.95), "p99_ms": q(.99)}
    allok.sort()
    q = lambda p: allok[min(len(allok) - 1, int(len(allok) * p))] if allok else float("nan")  # noqa: E731
    res.update(total_req_per_s=len(allok) / a.seconds, p50_ms=q(.5), p95_ms=q(.95), p99_ms=q(.99), errors=errors)
    print(json.dumps({k: (round(v, 1) if isinstance(v, float) else v) for k, v in res.items() if k != "per_endpoint"}))
    for ep, r in res["per_endpoint"].items():
        print(f"  {ep}: {r['req_per_s']:.1f} req/s p50 {r['p50_ms']:.0f} p95 {r['p95_ms']:.0f} p99 {r['p99_ms']:.0f} ms")
    if a.json:
        with open(a.json, "w") as fh:
            json.dump(res, fh, indent=1)


if __name__ == "__main__":
    main()
