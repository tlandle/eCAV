#!/usr/bin/env python3
"""Decomposition of the vehicle client's step in a perception cell run with ECAV_SENSOR_TIMING=1.

    python scripts/ispass/perception_step_breakdown.py experiment_results/ispass_scale/<tag> [--skip 5] [--json out.json]

Per client and tick, from the client logs ([CLIENT TIMING], [CLIENT VEHICLE TICK], [ML Manager], [SENSOR RX],
[SENSOR USE], [LOC]) and the service logs ([PerceptionServicer] with t0 and t_done):
  sensor frames   arrival of each frame after the client received the tick whose simulation time the frame carries,
                  bytes per tick, and how many simulator frames old the frames used for detection are
  preprocessing   colour conversion before the inference call
  inference call  JPEG encoding, round trip, the handler time in the replica (decode, model, encode) and the rest
                  (network and wait for a handler thread)
  rest of step    update_info outside detection, run_step, reply to the edge
"""
import glob
import json
import os
import re
import statistics as st
import sys

F = r"([-\d.]+)"
TIM = re.compile(rf"\[CLIENT TIMING VEHICLE (\d+) TICK (\d+)\] t_recv={F} t_start={F} t_built={F} t_sent={F}")
TICK = re.compile(rf"\[CLIENT VEHICLE TICK (\d+)\] update_info={F}ms \| run_step={F}ms")
ML = re.compile(rf"\[ML Manager\] serialize={F}ms rpc={F}ms parse={F}ms total={F}ms t_send={F} t_ret={F} req_bytes=(\d+)")
RX = re.compile(rf"\[SENSOR RX\] kind=(\w+) sensor=(\d+) frame=(\d+) sim_t={F} t={F} bytes=(\d+)")
USE = re.compile(rf"\[SENSOR USE\] t={F} prep_ms={F} cams=\[([\d, ]+)\] lidar=(\d+)")
LOC = re.compile(rf"\[LOC\].*? ts={F}")
UPD = re.compile(rf"\[VehicleManager update_info\] localize={F}ms \| detect={F}ms")
SND = re.compile(rf"\[CLIENT VEHICLE\] send_update={F}ms")
SYNC = re.compile(rf"\[SENSOR SYNC\] id=\S+ tick=\S+ frame=(\d+) wait_ms={F}")
SRV = re.compile(rf"\[PerceptionServicer\].*?decode={F}ms inference={F}ms encode={F}ms total={F}ms t0={F} t_done={F}")


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))] if v else float("nan")


def s3(v):
    return {"mean": round(st.mean(v), 1), "p50": round(q(v, .5), 1), "p95": round(q(v, .95), 1)} if v else None


def main():
    d = sys.argv[1]
    skip = int(sys.argv[sys.argv.index("--skip") + 1]) if "--skip" in sys.argv else 5
    acc = {k: [] for k in ("sync_wait", "sync_age", "update_info", "run_step", "detect", "prep", "serialize", "rpc", "parse", "reply", "step_total",
                           "cam_arrival", "lidar_arrival", "cam_lag_frames", "lidar_lag_frames", "bytes_per_tick",
                           "cam_bytes", "lidar_bytes", "rx_before_use")}
    nveh = 0
    for f in sorted(glob.glob(os.path.join(d, "*", "veh_*.log"))):
        nveh += 1
        cur = {}; ticks = {}; rx = []; frame_of_ts = {}
        for line in open(f, errors="replace"):
            m = RX.search(line)
            if m:
                rx.append((m.group(1), int(m.group(3)), float(m.group(4)), float(m.group(5)), int(m.group(6))))
                frame_of_ts[round(float(m.group(4)), 3)] = int(m.group(3)); continue
            m = SYNC.search(line)
            if m:
                cur["sync_wait"] = float(m.group(2)); cur["sync_frame"] = int(m.group(1)); continue
            m = LOC.search(line)
            if m:
                cur["ts"] = round(float(m.group(1)), 3); continue
            m = USE.search(line)
            if m:
                cur["use_t"] = float(m.group(1)); cur["prep"] = float(m.group(2))
                cur["cams"] = [int(x) for x in m.group(3).split(",")]; cur["lidar"] = int(m.group(4)); continue
            m = ML.search(line)
            if m:
                cur.update(serialize=float(m.group(1)), rpc=float(m.group(2)), parse=float(m.group(3))); continue
            m = UPD.search(line)
            if m:
                cur["detect"] = float(m.group(2)); continue
            m = TICK.search(line)
            if m:
                cur.update(n=int(m.group(1)), update_info=float(m.group(2)), run_step=float(m.group(3))); continue
            m = SND.search(line)
            if m:
                cur["reply"] = float(m.group(1)); continue
            m = TIM.search(line)
            if m:
                cur.update(t_recv=float(m.group(3)), t_start=float(m.group(4)), t_built=float(m.group(5)), t_sent=float(m.group(6)))
                if "n" in cur:
                    ticks[cur["n"]] = cur
                cur = {}
        recv_of_frame = {}   # simulator frame -> wall time at which this client received the tick that shows it
        for n, c in ticks.items():
            fr = frame_of_ts.get(c.get("ts"))
            if fr is not None:
                recv_of_frame[fr] = c["t_recv"]; c["frame"] = fr
        per_frame = {}
        for kind, fr, sim_t, t, b in rx:
            per_frame.setdefault(fr, []).append((kind, b))
            if fr in recv_of_frame:
                acc["cam_arrival" if kind == "camera" else "lidar_arrival"].append((t - recv_of_frame[fr]) * 1000)
            acc["cam_bytes" if kind == "camera" else "lidar_bytes"].append(b)
        for fr, v in per_frame.items():
            if fr in recv_of_frame:
                acc["bytes_per_tick"].append(sum(b for _, b in v))
        for n, c in ticks.items():
            if n <= skip or "rpc" not in c:
                continue
            if "sync_frame" in c and "cams" in c:
                acc["sync_age"].extend(c["sync_frame"] - x for x in c["cams"] + [c["lidar"]])
            for k in ("sync_wait", "update_info", "run_step", "detect", "prep", "serialize", "rpc", "parse", "reply"):
                if k in c:
                    acc[k].append(c[k])
            acc["step_total"].append((c["t_sent"] - c["t_start"]) * 1000)
            if "frame" in c and "cams" in c:
                acc["cam_lag_frames"].extend(c["frame"] - x for x in c["cams"])
                acc["lidar_lag_frames"].append(c["frame"] - c["lidar"])
    srv = {k: [] for k in ("decode", "inference", "encode", "total")}
    iv = {}
    for f in sorted(glob.glob(os.path.join(d, "yolo_grpc_r*.log"))):
        for line in open(f, errors="replace"):
            m = SRV.search(line)
            if m:
                for k, x in zip(("decode", "inference", "encode", "total"), m.groups()[:4]):
                    srv[k].append(float(x))
                iv.setdefault(f, []).append((float(m.group(5)), float(m.group(6))))
    # handlers in flight per replica, time-weighted
    inflight = []
    for f, v in iv.items():
        ev = sorted([(a, 1) for a, _ in v] + [(b, -1) for _, b in v]); k = 0; last = ev[0][0]; w = {}
        for t, e in ev:
            w[k] = w.get(k, 0.0) + t - last; last = t; k += e
        busy = sum(x for kk, x in w.items() if kk > 0)
        inflight.append(sum(kk * x for kk, x in w.items()) / busy if busy else 0.0)
    lag = lambda v: {str(k): round(100.0 * v.count(k) / len(v), 1) for k in sorted(set(v))} if v else None
    res = {"dir": d, "vehicles": nveh, "client_ticks": len(acc["rpc"]),
           "frame_age_at_detection_vs_tick_frame_pct": lag(acc["sync_age"]),
           "step_ms": {k: s3(acc[k]) for k in ("step_total", "sync_wait", "update_info", "detect", "prep", "serialize", "rpc", "parse", "run_step", "reply")},
           "service_ms": {k: s3(v) for k, v in srv.items()},
           "rpc_minus_handler_ms": round(st.mean(acc["rpc"]) - st.mean(srv["total"]), 1) if acc["rpc"] and srv["total"] else None,
           "handlers_in_flight_while_busy": [round(x, 2) for x in inflight],
           "requests_per_client_tick": 1,
           "sensor": {"camera_arrival_ms_after_tick_received": s3(acc["cam_arrival"]), "lidar_arrival_ms_after_tick_received": s3(acc["lidar_arrival"]),
                      "camera_frame_bytes": s3(acc["cam_bytes"]), "lidar_frame_bytes": s3(acc["lidar_bytes"]),
                      "bytes_per_tick_per_vehicle": s3(acc["bytes_per_tick"]),
                      "camera_frame_age_at_use_pct_by_frames": lag(acc["cam_lag_frames"]),
                      "lidar_frame_age_at_use_pct_by_frames": lag(acc["lidar_lag_frames"])}}
    print(json.dumps(res, indent=1))
    if "--json" in sys.argv:
        json.dump(res, open(sys.argv[sys.argv.index("--json") + 1], "w"), indent=1)


if __name__ == "__main__":
    main()
