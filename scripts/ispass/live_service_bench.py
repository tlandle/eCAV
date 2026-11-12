#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ISPASS Experiment 1: the platform's timing model against a continuously running GPU service.

Arm A, live service. The pipeline of live_service_pipeline.py runs in this
process behind a gRPC server. N client processes each send one contributor
frame message (16,896 B, the uplink payload of the platform's intermediate
fusion) every 50 ms of wall-clock time. A frame is complete when all N
messages of one capture have arrived. Whenever the GPU is free the service
admits the newest complete frame it has not yet processed (latest-frame
admission, no queue), runs the pipeline, and streams the result (2,400 B) to
a planner process, which records the receive time. Arrival, admission,
completion and delivery are wall-clock timestamps.

Arm B, barrier model. The same pipeline, inputs and GPU, driven in logical
time with a fixed step: a frame captured at step time t is available at t,
an invocation admitted at a step occupies the service for its measured
duration, frames captured while the service is occupied are superseded, and
the result is visible to the planner at the first step at or after admission
plus the measured duration. No wall-clock pacing.

Arm B0, develop as implemented (no occupancy): every captured frame is
processed and its delivery is delayed by its measured duration. This is what
the WorldFusion edge manager does on develop at the time of this bench.

Radio delay is zero in every arm. The live arm includes the loopback gRPC
transit, which is reported.

Usage:
  python scripts/ispass/live_service_bench.py --stamp gate --ns 2 16 --reps 1
  python scripts/ispass/live_service_bench.py --stamp full
"""
from __future__ import annotations

import argparse
import json
import math
import os
import platform
import queue
import struct
import subprocess
import sys
import threading
import time
from concurrent import futures
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
FRAME_MS = 50.0
UL_BYTES = 16896
DL_BYTES = 2400
HDR = struct.Struct("<iiq")          # client id, capture seq, capture time ns
RES = struct.Struct("<iqq")          # capture seq, admit ns, done ns
SVC = "ispass.LiveService"


# ----------------------------------------------------------------------------
# client and planner roles (separate processes)
# ----------------------------------------------------------------------------
def role_client(args):
    import grpc
    ch = grpc.insecure_channel(f"127.0.0.1:{args.port}",
                               options=[("grpc.max_send_message_length", 1 << 22)])
    submit = ch.unary_unary(f"/{SVC}/Submit", request_serializer=None, response_deserializer=None)
    pad = bytes(UL_BYTES - HDR.size)
    t0 = args.t0_ns
    n_frames = int(round(args.duration * 1000.0 / FRAME_MS))
    pending = []
    for seq in range(n_frames):
        t_cap = t0 + int(seq * FRAME_MS * 1e6)
        d = (t_cap - time.time_ns()) / 1e9
        if d > 0:
            time.sleep(d)
        pending.append(submit.future(HDR.pack(args.cid, seq, t_cap) + pad))
        if len(pending) > 64:
            pending = [f for f in pending if not f.done()]
    for f in pending:
        try:
            f.result(timeout=2.0)
        except Exception:
            pass


def role_planner(args):
    import grpc
    ch = grpc.insecure_channel(f"127.0.0.1:{args.port}")
    sub = ch.unary_stream(f"/{SVC}/Subscribe", request_serializer=None, response_deserializer=None)
    rows = []
    try:
        for msg in sub(b"", timeout=args.duration + 30.0):
            t = time.time_ns()
            seq, t_admit, t_done = RES.unpack_from(msg)
            if seq < 0:
                break
            rows.append((seq, t_admit, t_done, t))
    except Exception as exc:  # stream closed at the end of the run
        print("planner stream ended:", type(exc).__name__, flush=True)
    with open(args.out, "w") as f:
        json.dump(rows, f)


# ----------------------------------------------------------------------------
# live service (arm A)
# ----------------------------------------------------------------------------
class LiveService:
    def __init__(self, pipe, port):
        import grpc
        self.grpc = grpc
        self.pipe = pipe
        self.port = port
        self.cond = threading.Condition()
        self.n = 0
        self.count = {}       # seq -> messages arrived
        self.first = {}       # seq -> first arrival ns
        self.complete = {}    # seq -> arrival ns of the last message
        self.newest = -1
        self.subs = []
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=48),
                                  options=[("grpc.max_receive_message_length", 1 << 22)])
        h = grpc.method_handlers_generic_handler(SVC, {
            "Submit": grpc.unary_unary_rpc_method_handler(self._submit),
            "Subscribe": grpc.unary_stream_rpc_method_handler(self._subscribe),
        })
        self.server.add_generic_rpc_handlers((h,))
        self.server.add_insecure_port(f"127.0.0.1:{port}")
        self.server.start()

    def _submit(self, req, ctx):
        t = time.time_ns()
        cid, seq, t_cap = HDR.unpack_from(req)
        with self.cond:
            c = self.count.get(seq, 0) + 1
            self.count[seq] = c
            self.first.setdefault(seq, t)
            if c == self.n:
                self.complete[seq] = t
                if seq > self.newest:
                    self.newest = seq
                    self.cond.notify_all()
        return b""

    def _subscribe(self, req, ctx):
        q = queue.Queue()
        self.subs.append(q)
        while True:
            m = q.get()
            yield m
            if RES.unpack_from(m)[0] < 0:
                return

    def run_point(self, n, predict, duration_s, warm_s, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        with self.cond:
            self.n = n
            self.count.clear(); self.first.clear(); self.complete.clear()
            self.newest = -1
        self.subs.clear()
        self.pipe.reset()
        for s in range(8):                      # GPU warm-up at this N, then a clean tracker
            self.pipe.invoke(s, n, predict)
        self.pipe.reset()
        total = warm_s + duration_s
        py = sys.executable
        me = str(Path(__file__).resolve())
        plan_out = os.path.join(out_dir, "planner.json")
        procs = [subprocess.Popen([py, me, "--role", "planner", "--port", str(self.port),
                                   "--duration", str(total), "--out", plan_out])]
        t_wait = time.time() + 20.0
        while not self.subs and time.time() < t_wait:
            time.sleep(0.05)
        if not self.subs:
            raise RuntimeError("planner did not subscribe")
        t0 = time.time_ns() + int(4e9)
        for cid in range(n):
            procs.append(subprocess.Popen([py, me, "--role", "client", "--port", str(self.port),
                                           "--cid", str(cid), "--t0-ns", str(t0),
                                           "--duration", str(total)]))
        t_end = t0 + int(total * 1e9)
        pad = bytes(DL_BYTES - RES.size)
        last = -1
        inv = []
        while True:
            with self.cond:
                while self.newest <= last and time.time_ns() < t_end:
                    self.cond.wait(timeout=0.02)
                if self.newest <= last:
                    break
                seq = self.newest
            t_admit = time.time_ns()
            if t_admit >= t_end:
                break
            st = self.pipe.invoke(seq, n, predict)
            t_done = time.time_ns()
            for q in self.subs:
                q.put(RES.pack(seq, t_admit, t_done) + pad)
            st.update(seq=seq, t_admit=t_admit, t_done=t_done, t_arrive=self.complete[seq],
                      t_first=self.first[seq])
            inv.append(st)
            last = seq
        for q in self.subs:
            q.put(RES.pack(-1, 0, 0) + pad)
        for p in procs:
            try:
                p.wait(timeout=30)
            except subprocess.TimeoutExpired:
                p.kill()
        with self.cond:
            n_complete = len(self.complete)
            n_seen = len(self.count)
        deliveries = json.load(open(plan_out)) if os.path.exists(plan_out) else []
        raw = dict(t0_ns=t0, n=n, predict=predict, duration_s=duration_s, warm_s=warm_s,
                   invocations=inv, deliveries=deliveries, frames_complete=n_complete,
                   frames_seen=n_seen)
        with open(os.path.join(out_dir, "live_raw.json"), "w") as f:
            json.dump(raw, f)
        return summarize_live(raw)


def _pct(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def summarize_live(raw):
    t0, warm, dur = raw["t0_ns"], raw["warm_s"], raw["duration_s"]
    lo = t0 + int(warm * 1e9)
    hi = lo + int(dur * 1e9)
    dl = sorted((d[3], d[0]) for d in raw["deliveries"])          # (t_recv, seq)
    ages = []
    j, best = 0, None
    m0 = int(round(warm * 1000 / FRAME_MS))
    m1 = int(round((warm + dur) * 1000 / FRAME_MS))
    for m in range(m0, m1):
        t_plan = t0 + int(m * FRAME_MS * 1e6)
        while j < len(dl) and dl[j][0] <= t_plan:
            best = dl[j][1] if best is None else max(best, dl[j][1])
            j += 1
        if best is not None:
            ages.append((t_plan - (t0 + int(best * FRAME_MS * 1e6))) / 1e6)
    inv = [i for i in raw["invocations"] if lo <= i["t_admit"] < hi]
    comp = [(i["t_done"] - i["t_admit"]) / 1e6 for i in inv]
    n_del = sum(1 for t, _ in dl if lo <= t < hi)
    frames = dur * 1000.0 / FRAME_MS
    recv = {d[0]: d[3] for d in raw["deliveries"]}
    transit_dl = [(recv[i["seq"]] - i["t_done"]) / 1e6 for i in inv if i["seq"] in recv]
    transit_ul = [(i["t_arrive"] - (t0 + int(i["seq"] * FRAME_MS * 1e6))) / 1e6 for i in inv]
    return dict(
        age_mean=float(np.mean(ages)) if ages else float("nan"),
        age_p50=_pct(ages, 50), age_p95=_pct(ages, 95), age_p99=_pct(ages, 99),
        update_hz=n_del / dur, superseded=max(0.0, 1.0 - len(inv) / frames),
        svc_mean=float(np.mean(comp)) if comp else float("nan"),
        svc_p50=_pct(comp, 50), svc_p95=_pct(comp, 95), svc_p99=_pct(comp, 99),
        fusion_det_ms=float(np.mean([i["fusion_det_ms"] for i in inv])) if inv else float("nan"),
        track_ms=float(np.mean([i["track_ms"] for i in inv])) if inv else float("nan"),
        predict_ms=float(np.mean([i["predict_ms"] for i in inv])) if inv else float("nan"),
        n_trk=float(np.mean([i["n_trk"] for i in inv])) if inv else float("nan"),
        ul_transit_p50=_pct(transit_ul, 50), ul_transit_p99=_pct(transit_ul, 99),
        dl_transit_p50=_pct(transit_dl, 50), dl_transit_p99=_pct(transit_dl, 99),
        frames_complete_frac=raw["frames_complete"] / max(1, raw["frames_seen"]),
        n_invocations=len(inv), n_planner_samples=len(ages))


# ----------------------------------------------------------------------------
# barrier model (arm B) and develop-as-implemented (arm B0)
# ----------------------------------------------------------------------------
def run_barrier(pipe, n, predict, dt_ms, duration_s, warm_s, occupancy=True, out_path=None):
    pipe.reset()
    for s in range(8):
        pipe.invoke(s, n, predict)
    pipe.reset()
    total_ms = (warm_s + duration_s) * 1000.0
    ticks = int(round(total_ms / dt_ms))
    t_free = 0.0
    last = -1
    outbound = []            # (deliver_tick, seq)
    inv = []
    ages = []
    visible = None
    wall0 = time.perf_counter()
    for m in range(ticks):
        t = m * dt_ms
        newest = int(math.floor(t / FRAME_MS + 1e-9))
        if newest > last and (not occupancy or t_free <= t + 1e-9):
            st = pipe.invoke(newest, n, predict)
            c = st["total_ms"]
            t_free = t + c
            dtick = m + int(math.ceil(c / dt_ms))
            outbound.append((dtick, newest))
            st.update(seq=newest, tick=m, t_admit_ms=t, deliver_tick=dtick)
            inv.append(st)
            last = newest
        ready = [s for (d, s) in outbound if d <= m]
        if ready:
            visible = max(ready) if visible is None else max(visible, max(ready))
            outbound = [(d, s) for (d, s) in outbound if d > m]
        if abs(t / FRAME_MS - round(t / FRAME_MS)) < 1e-9 and t >= warm_s * 1000.0 and visible is not None:
            ages.append(t - visible * FRAME_MS)
    wall = time.perf_counter() - wall0
    lo = warm_s * 1000.0
    sel = [i for i in inv if i["t_admit_ms"] >= lo]
    comp = [i["total_ms"] for i in sel]
    n_del = sum(1 for i in inv if lo <= i["deliver_tick"] * dt_ms < total_ms)
    frames = duration_s * 1000.0 / FRAME_MS
    if out_path:
        with open(out_path, "w") as f:
            json.dump(dict(n=n, predict=predict, dt_ms=dt_ms, occupancy=occupancy,
                           duration_s=duration_s, warm_s=warm_s, invocations=inv), f)
    return dict(
        age_mean=float(np.mean(ages)) if ages else float("nan"),
        age_p50=_pct(ages, 50), age_p95=_pct(ages, 95), age_p99=_pct(ages, 99),
        update_hz=n_del / duration_s, superseded=max(0.0, 1.0 - len(sel) / frames),
        svc_mean=float(np.mean(comp)) if comp else float("nan"),
        svc_p50=_pct(comp, 50), svc_p95=_pct(comp, 95), svc_p99=_pct(comp, 99),
        wall_s=wall, wall_per_sim_s=wall / (warm_s + duration_s),
        n_invocations=len(sel), n_planner_samples=len(ages))


# ----------------------------------------------------------------------------
def hardware():
    import torch
    info = dict(host=platform.node(), python=platform.python_version(), torch=torch.__version__,
                cuda=torch.version.cuda, gpu=torch.cuda.get_device_name(0),
                kernel=platform.release())
    try:
        info["cpu"] = [l.split(":")[1].strip() for l in open("/proc/cpuinfo") if l.startswith("model name")][0]
        info["cores"] = os.cpu_count()
        info["nvidia_smi"] = subprocess.check_output(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"],
            text=True).strip()
        info["git"] = subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        pass
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--role", default="main")
    ap.add_argument("--port", type=int, default=50871)
    ap.add_argument("--cid", type=int, default=0)
    ap.add_argument("--t0-ns", type=int, default=0)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warm", type=float, default=5.0)
    ap.add_argument("--out", default="")
    ap.add_argument("--stamp", default="gate")
    ap.add_argument("--ns", type=int, nargs="+", default=[1, 2, 4, 8, 16, 24, 32])
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--steps", type=float, nargs="+", default=[50.0, 25.0])
    ap.add_argument("--workloads", nargs="+", default=["fusion", "full"])
    ap.add_argument("--b0-duration", type=float, default=20.0)
    ap.add_argument("--frames", type=int, default=20)
    a = ap.parse_args()
    if a.role == "client":
        return role_client(a)
    if a.role == "planner":
        return role_planner(a)

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from live_service_pipeline import Pipeline, SCENARIO, RSU, WF_DIR, WF_EPOCH, MTR
    root = REPO / "experiment_results" / "ispass_timing_live" / a.stamp
    root.mkdir(parents=True, exist_ok=True)
    pipe = Pipeline(n_frames=a.frames)
    ns = [n for n in a.ns if n <= pipe.max_n]
    cfg = dict(stamp=a.stamp, ns=ns, reps=a.reps, steps=a.steps, workloads=a.workloads,
               duration_s=a.duration, warm_s=a.warm, frame_ms=FRAME_MS, ul_bytes=UL_BYTES,
               dl_bytes=DL_BYTES, radio="zero in every arm (live arm adds loopback gRPC transit)",
               data=dict(scenario=SCENARIO, rsu=RSU, window=pipe.window, agents_min=pipe.max_n),
               checkpoints=dict(worldfusion=str(WF_DIR.relative_to(REPO)) + f"/net_epoch{WF_EPOCH}.pth",
                                mtr=os.path.relpath(MTR["mtr_checkpoint"], REPO)),
               hardware=hardware(), started=time.strftime("%F %T"))
    json.dump(cfg, open(root / "config.json", "w"), indent=1)
    svc = LiveService(pipe, a.port)
    results = []
    res_path = root / "results.json"
    for wl in a.workloads:
        predict = wl == "full"
        for n in ns:
            for rep in range(a.reps):
                d = root / "raw" / f"{wl}_N{n}_rep{rep}"
                d.mkdir(parents=True, exist_ok=True)
                live = svc.run_point(n, predict, a.duration, a.warm, str(d))
                row = dict(workload=wl, n=n, rep=rep, live=live, barrier={})
                for dt in a.steps:
                    row["barrier"][str(int(dt))] = run_barrier(
                        pipe, n, predict, dt, a.duration, a.warm, True,
                        str(d / f"barrier_{int(dt)}.json"))
                if rep == 0:
                    row["develop_no_occupancy"] = run_barrier(
                        pipe, n, predict, 50.0, a.b0_duration, a.warm, False,
                        str(d / "develop_no_occupancy.json"))
                results.append(row)
                json.dump(results, open(res_path, "w"), indent=1)
                b = row["barrier"][str(int(a.steps[0]))]
                print(f"{time.strftime('%T')} {wl:6s} N={n:2d} rep={rep} "
                      f"live age p50/p95/p99 {live['age_p50']:.0f}/{live['age_p95']:.0f}/{live['age_p99']:.0f} "
                      f"rate {live['update_hz']:.1f} svc {live['svc_mean']:.0f} | "
                      f"barrier{int(a.steps[0])} {b['age_p50']:.0f}/{b['age_p95']:.0f}/{b['age_p99']:.0f} "
                      f"rate {b['update_hz']:.1f} svc {b['svc_mean']:.0f}", flush=True)
    svc.server.stop(0)
    print("done", res_path)


if __name__ == "__main__":
    main()
