#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ISPASS Experiment 4: how much network-simulation fidelity is necessary.

Three radio models resolve the same workload, N contributors each sending one
uplink message and receiving one downlink message per 50 ms frame:

  live    the ns-3 5G-LENA process (NR Uu) resolves every message from the
          positions of the moving actors (NS3Bridge, mode uu_uplink).
  table   Ns3LutSampler: an independent draw per message from the stored
          (mean, p95, max, delivery ratio) of the nearest (N, payload) cell.
          This is what the edge managers use with use_ns3_lut.
  trace   the SEE-V2X field trace (LTE-V2X PC5, one sender to the RSU), in
          arrival order. A different deployment, reported separately.

Per (N, payload) the script records, per message, whether and when it was
delivered, then reports the delay distribution and delivery ratio per
direction, how bursty the loss is, the age of state at the planner when the
same recorded compute durations run through the platform's occupancy gate
(ecav/core/application/edge/service_occupancy.py), and the wall-clock cost.

  python scripts/ispass/radio_fidelity_bench.py --stamp rf1 --gate
  python scripts/ispass/radio_fidelity_bench.py --stamp rf1

Raw per-message traces: experiment_results/ispass_radio_fidelity/<stamp>/raw/.
"""
import argparse
import glob
import json
import math
import os
import random
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from ecav.core.application.edge.latency.ns3_lut_sampler import Ns3LutSampler  # noqa: E402
from ecav.core.application.edge.service_occupancy import ServiceOccupancy     # noqa: E402

DT_MS = 50.0
LUT_DIR = os.path.join(ROOT, "ecav/core/application/edge/latency/data")
LIVE_TIMING = os.path.join(ROOT, "experiment_results/ispass_timing_live/live_20261111/raw")


# --------------------------------------------------------------------------
# Actor positions: vehicles on the two roads of a four-way junction, the gNB
# at the junction centre (the server puts it at the origin). Each vehicle
# moves at its own speed and wraps at the end of the 240 m segment.
# --------------------------------------------------------------------------
class Layout:
    def __init__(self, n, seed, span=120.0):
        rng = random.Random(1000 + seed)
        self.span = span
        self.veh = []
        for i in range(n):
            road = i % 2                      # 0: east-west, 1: north-south
            lane = (-5.25, -1.75, 1.75, 5.25)[(i // 2) % 4]
            direction = 1.0 if lane > 0 else -1.0
            s0 = rng.uniform(-span, span)
            v = rng.uniform(8.0, 15.0) * direction
            self.veh.append((road, lane, s0, v))

    def positions(self, tick):
        t = tick * DT_MS / 1000.0
        out = []
        for road, lane, s0, v in self.veh:
            s = ((s0 + v * t + self.span) % (2 * self.span)) - self.span
            out.append([s, lane, 1.5] if road == 0 else [lane, s, 1.5])
        return out


# --------------------------------------------------------------------------
# Live ns-3
# --------------------------------------------------------------------------
def run_live(n, payload, ticks, seed, warm=20, progress=None, max_wall_s=None):
    """Returns per-tick completion records as the server reports them:
    ul_ok/dl_ok [ticks, n] and ul_ms/dl_ms [ticks, n], plus wall ms per tick."""
    from ecav.core.networking.ns3_cosim.ns3_bridge import NS3Bridge
    os.environ["NS_GLOBAL_VALUE"] = f"RngRun={seed + 1}"
    os.environ.pop("UU_DL_PAYLOAD_BYTES", None)
    lay = Layout(n, seed)
    br = NS3Bridge(max_vehicles=n, mode="uu_uplink", tick_duration_s=DT_MS / 1000.0,
                   shm_tag=f"rf{n}_{payload}_{seed}")
    br.first_tick_timeout_s = 3600.0; br.tick_timeout_s = 3600.0
    t_start = time.time()
    br.start()
    ul_ok = np.zeros((ticks, n), bool); dl_ok = np.zeros((ticks, n), bool)
    ul_ms = np.zeros((ticks, n), np.float32); dl_ms = np.zeros((ticks, n), np.float32)
    wall = np.zeros(ticks, np.float32)
    done = 0
    try:
        for k in range(-warm, ticks):
            t0 = time.perf_counter()
            res = br.compute_tick(lay.positions(k + warm), None, k + warm, payload_bytes=payload)
            w = (time.perf_counter() - t0) * 1000.0
            if k < 0:
                continue
            wall[k] = w
            if progress and (k < 3 or k % progress == 0):
                print(f"    tick {k}: {w:.0f} ms wall", flush=True)
            done = k + 1
            if max_wall_s and time.time() - t_start > max_wall_s:
                print(f"    wall budget reached after {done} recorded ticks", flush=True)
                break
            if not res:
                raise RuntimeError(f"ns-3 returned no results at tick {k} (N={n}, payload={payload})")
            for r in res:
                if r.rx_id == 0xFFFF and r.tx_id < n:          # uplink row
                    ul_ok[k, r.tx_id] = r.delivered; ul_ms[k, r.tx_id] = r.delay_ms
                elif r.tx_id == 0xFFFE and r.rx_id < n:        # downlink row
                    dl_ok[k, r.rx_id] = r.delivered; dl_ms[k, r.rx_id] = r.delay_ms
    finally:
        br.shutdown()
    ul_ok, ul_ms, dl_ok, dl_ms, wall = ul_ok[:done], ul_ms[:done], dl_ok[:done], dl_ms[:done], wall[:done]
    return dict(ul_ok=ul_ok, ul_ms=ul_ms, dl_ok=dl_ok, dl_ms=dl_ms, wall_ms=wall,
                startup_s=time.time() - t_start - float(wall.sum()) / 1000.0)


def by_send_tick(ok, ms):
    """The server reports, per tick, the message that completed in that tick and
    its delay. Re-index to the tick each message was sent in: a completion at
    tick k with delay d belongs to the message sent ceil(d / dt) - 1 ticks
    earlier. Messages with no completion are lost."""
    ticks, n = ok.shape
    s_ok = np.zeros_like(ok); s_ms = np.full(ok.shape, np.nan, np.float32)
    for k in range(ticks):
        for u in np.nonzero(ok[k])[0]:
            d = float(ms[k, u])
            s = k - (max(1, int(math.ceil(d / DT_MS - 1e-9))) - 1)
            if s >= 0 and not s_ok[s, u]:
                s_ok[s, u] = True; s_ms[s, u] = d
    return s_ok, s_ms


# --------------------------------------------------------------------------
# Table and trace arms: per-message outcome indexed by send tick
# --------------------------------------------------------------------------
def run_table(n, payload, ticks, seed):
    random.seed(7000 + seed)
    lut = Ns3LutSampler(os.path.join(LUT_DIR, "ns3_uplink_lut.csv"),
                        os.path.join(LUT_DIR, "ns3_downlink_lut.csv"))
    out = {}
    t0 = time.perf_counter()
    for d in ("ul", "dl"):
        ok = np.zeros((ticks, n), bool); ms = np.full((ticks, n), np.nan, np.float32)
        for k in range(ticks):
            for u in range(n):
                delay, delivered = lut.sample_with_loss(n, payload, d)
                ok[k, u] = delivered
                if delivered:
                    ms[k, u] = delay
        out[d + "_ok"] = ok; out[d + "_ms"] = ms
    out["wall_ms"] = np.full(ticks, (time.perf_counter() - t0) * 1000.0 / ticks, np.float32)
    return out


def run_trace(n, ticks, seed):
    """SEE-V2X in arrival order, one sender, by the harness's replay rule
    (latency_model.HybridModel): message period 20 ms, a receive gap longer
    than 1.5 periods is read as lost messages, at most 20 per gap. One trace
    message per frame. The same stream serves both directions with different
    offsets (the trace has one direction)."""
    from ecav.core.application.edge.latency.latency_model import load_trace_rows
    rows = load_trace_rows()
    if not rows:
        return None
    ts = np.array([r[0] for r in rows], np.int64); lat = np.array([r[1] for r in rows], np.float32)
    period = 20000.0
    ok_seq, ms_seq = [], []
    for i in range(len(ts)):
        if i > 0:
            gap = float(ts[i] - ts[i - 1])
            miss = int(round(gap / period)) - 1
            if gap > 1.5 * period and miss > 0:
                for _ in range(min(miss, 20)):
                    ok_seq.append(False); ms_seq.append(np.nan)
        ok_seq.append(True); ms_seq.append(lat[i])
    ok_seq = np.array(ok_seq, bool); ms_seq = np.array(ms_seq, np.float32)
    rng = random.Random(9000 + seed)
    out = {"trace_period_ms": period / 1000.0}
    for d in ("ul", "dl"):
        ok = np.zeros((ticks, n), bool); ms = np.full((ticks, n), np.nan, np.float32)
        for u in range(n):
            o = rng.randrange(0, len(ok_seq) - ticks)
            ok[:, u] = ok_seq[o:o + ticks]; ms[:, u] = ms_seq[o:o + ticks]
        out[d + "_ok"] = ok; out[d + "_ms"] = ms
    out["wall_ms"] = np.zeros(ticks, np.float32)
    return out


# --------------------------------------------------------------------------
# Metrics
# --------------------------------------------------------------------------
def pct(a, q):
    a = a[~np.isnan(a)]
    return float(np.percentile(a, q)) if a.size else float("nan")


def burstiness(ok):
    """Lag-1 autocorrelation of the loss indicator per UE (mean over UEs with
    any loss and any delivery), and the mean length of a run of lost messages."""
    acs, runs = [], []
    for u in range(ok.shape[1]):
        x = (~ok[:, u]).astype(float)
        if 0 < x.sum() < len(x):
            a, b = x[:-1] - x.mean(), x[1:] - x.mean()
            den = float((x - x.mean()).var() * (len(x) - 1))
            if den > 0:
                acs.append(float((a * b).sum() / den))
        run = 0
        for v in x:
            if v:
                run += 1
            elif run:
                runs.append(run); run = 0
        if run:
            runs.append(run)
    return (float(np.mean(acs)) if acs else float("nan"),
            float(np.mean(runs)) if runs else 0.0,
            int(max(runs)) if runs else 0)


def delay_autocorr(ms):
    acs = []
    for u in range(ms.shape[1]):
        x = ms[:, u]
        m = ~np.isnan(x[:-1]) & ~np.isnan(x[1:])
        if m.sum() > 20 and np.nanstd(x) > 1e-6:
            acs.append(float(np.corrcoef(x[:-1][m], x[1:][m])[0, 1]))
    return float(np.nanmean(acs)) if acs else float("nan")


def compute_durations(n):
    """Recorded service durations of the full pipeline for this contributor
    count (live-service experiment, 50 ms barrier arm). N = 31 uses the N = 30
    recording, the largest recorded frame."""
    nn = {31: 30}.get(n, n)
    out = []
    for p in sorted(glob.glob(os.path.join(LIVE_TIMING, f"full_N{nn}_rep*", "barrier_50.json"))):
        out += [float(i["total_ms"]) for i in json.load(open(p))["invocations"]]
    return out


def planner_age(arm, durs, ticks, warm=40):
    """Age of state at each planner step for each UE taken as contributor and
    recipient: frame k is captured at 50k, reaches the edge after the UE's
    uplink delay, the service runs under the occupancy gate with the recorded
    durations, the result is sent at the first step at or after completion and
    arrives after the UE's downlink delay for that step. A lost uplink skips the
    frame, a lost downlink skips the update."""
    n = arm["ul_ok"].shape[1]
    warm = min(warm, ticks // 4)
    ages, gaps, cover = [], [], []
    for u in range(n):
        gate = ServiceOccupancy(True, DT_MS)
        arrive = {}                                   # step -> [source frames arriving]
        for k in range(ticks):
            if arm["ul_ok"][k, u]:
                m = int(math.ceil((k * DT_MS + float(arm["ul_ms"][k, u])) / DT_MS - 1e-9))
                arrive.setdefault(m, []).append(k)
        deliver = {}                                  # step visible at planner -> newest source frame
        di = 0
        for m in range(ticks):
            f = gate.admit(m, [(s, None) for s in sorted(arrive.get(m, []))])
            if f is None:
                continue
            c = durs[di % len(durs)]; di += 1
            gate.occupy(m, c)
            md = m + int(math.ceil(c / DT_MS - 1e-9))  # step the result is sent
            if md >= ticks or not arm["dl_ok"][md, u]:
                continue
            mv = int(math.ceil((md * DT_MS + float(arm["dl_ms"][md, u])) / DT_MS - 1e-9))
            if mv < ticks:
                deliver[mv] = max(deliver.get(mv, -1), f[0])
        cur, last_upd, a_u, gap_u = -1, None, [], []
        for m in range(ticks):
            if m in deliver and deliver[m] > cur:
                cur = deliver[m]
                if last_upd is not None and m >= warm:
                    gap_u.append((m - last_upd) * DT_MS)
                last_upd = m
            if m >= warm:
                a_u.append((m - cur) * DT_MS if cur >= 0 else np.nan)
        a_u = np.array(a_u)
        cover.append(float((~np.isnan(a_u)).mean()))
        ages.append(a_u[~np.isnan(a_u)]); gaps += gap_u
    allages = np.concatenate(ages) if ages else np.array([])
    return dict(age_mean=float(allages.mean()) if allages.size else float("nan"),
                age_p50=pct(allages, 50), age_p95=pct(allages, 95), age_p99=pct(allages, 99),
                update_gap_p95=float(np.percentile(gaps, 95)) if gaps else float("nan"),
                update_gap_max=float(max(gaps)) if gaps else float("nan"),
                coverage=float(np.mean(cover)))


def summarize(arm, durs, ticks):
    row = {}
    for d in ("ul", "dl"):
        ok, ms = arm[d + "_ok"], arm[d + "_ms"]
        ac, run_mean, run_max = burstiness(ok)
        row.update({f"{d}_p50": pct(ms, 50), f"{d}_p95": pct(ms, 95), f"{d}_p99": pct(ms, 99),
                    f"{d}_mean": float(np.nanmean(ms)) if np.any(~np.isnan(ms)) else float("nan"),
                    f"{d}_delivery": float(ok.mean()),
                    f"{d}_loss_lag1": ac, f"{d}_loss_run_mean": run_mean, f"{d}_loss_run_max": run_max,
                    f"{d}_delay_lag1": delay_autocorr(ms)})
    row.update(planner_age(arm, durs, ticks))
    row["wall_ms_per_tick"] = float(np.mean(arm["wall_ms"]))
    row["wall_s_per_sim_s"] = row["wall_ms_per_tick"] / DT_MS
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", required=True)
    ap.add_argument("--gate", action="store_true", help="N = 4 and 24 at 16896 B, one seed, 200 ticks")
    ap.add_argument("--ns", type=int, nargs="*", default=[4, 8, 16, 24, 31])
    ap.add_argument("--payloads", type=int, nargs="*", default=[300, 1000, 16896])
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--ticks", type=int, default=600)
    ap.add_argument("--warm", type=int, default=20, help="ns-3 steps before recording")
    ap.add_argument("--progress", type=int, default=0, help="print wall time every k ticks")
    ap.add_argument("--seed-list", type=int, nargs="*", default=None, help="explicit seeds (parallel invocations)")
    ap.add_argument("--max-wall-s", type=float, default=None, help="stop a live cell after this wall time, keep its ticks")
    ap.add_argument("--append", action="store_true", help="keep the rows of earlier invocations under this stamp")
    ap.add_argument("--skip-live", action="store_true", help="re-reduce saved live traces")
    a = ap.parse_args()
    if a.gate:
        a.ns, a.payloads, a.seeds, a.ticks = [4, 24], [16896], 1, 200
    out = os.path.join(ROOT, "experiment_results/ispass_radio_fidelity", a.stamp + ("/gate" if a.gate else ""))
    os.makedirs(os.path.join(out, "raw"), exist_ok=True)
    json.dump({"ns": a.ns, "payloads": a.payloads, "seeds": a.seeds, "ticks": a.ticks, "dt_ms": DT_MS,
               "layout": "two roads of a four-way junction, 240 m each, gNB at the centre, 8 to 15 m/s",
               "radio": "NR Uu, 5.9 GHz, 40 MHz, numerology 0, 23 dBm (NS3Bridge defaults)",
               "dl_payload": "equal to the uplink payload", "host": os.uname().nodename,
               "ns3_binary": "/home/atlas/ns3_cv2x/ns-3-dev-5glena/build/scratch/ns3-dev-nr_v2x_cosim_server-default",
               "compute_durations": "experiment_results/ispass_timing_live/live_20261111 full pipeline, barrier_50"},
              open(os.path.join(out, "config.json"), "w"), indent=1)
    rows = []
    seeds = a.seed_list if a.seed_list is not None else list(range(a.seeds))
    rows_path = os.path.join(out, "rows.json")
    if a.append and os.path.exists(rows_path):
        cells = {(n, p, s) for n in a.ns for p in a.payloads for s in seeds}
        rows = [r for r in json.load(open(rows_path)) if (r["n"], r["payload"], r["seed"]) not in cells]
    for n in a.ns:
        durs = compute_durations(n)
        for p in a.payloads:
            for s in seeds:
                tag = f"N{n}_P{p}_s{s}"
                raw = os.path.join(out, "raw", tag + "_live.npz")
                if a.skip_live and os.path.exists(raw):
                    z = dict(np.load(raw))
                else:
                    t0 = time.time()
                    z = run_live(n, p, a.ticks, s, warm=a.warm, progress=a.progress, max_wall_s=a.max_wall_s)
                    np.savez_compressed(raw, **z)
                    print(f"[live] {tag} {time.time() - t0:.0f} s wall", flush=True)
                T = int(z["ul_ok"].shape[0])
                ul_ok, ul_ms = by_send_tick(z["ul_ok"], z["ul_ms"])
                dl_ok, dl_ms = by_send_tick(z["dl_ok"], z["dl_ms"])
                # drop the tail where a completion could still be pending
                arms = {"live": dict(ul_ok=ul_ok, ul_ms=ul_ms, dl_ok=dl_ok, dl_ms=dl_ms, wall_ms=z["wall_ms"]),
                        "table": run_table(n, p, T, s)}
                tr = run_trace(n, T, s)
                if tr is not None:
                    arms["trace"] = tr
                for name, arm in arms.items():
                    if name != "live":
                        np.savez_compressed(os.path.join(out, "raw", f"{tag}_{name}.npz"),
                                            **{k: v for k, v in arm.items() if isinstance(v, np.ndarray)})
                    r = summarize(arm, durs, T)
                    r.update(model=name, n=n, payload=p, seed=s, ticks=T)
                    if name == "live":
                        r["startup_s"] = float(z.get("startup_s", float("nan")))
                        r["tick_completion_ratio_ul"] = float(z["ul_ok"].mean())
                        r["tick_completion_ratio_dl"] = float(z["dl_ok"].mean())
                    rows.append(r)
                    print(f"  {name:5s} {tag}: UL p50/p95 {r['ul_p50']:.1f}/{r['ul_p95']:.1f} del {r['ul_delivery']:.2f} | "
                          f"DL p50/p95 {r['dl_p50']:.1f}/{r['dl_p95']:.1f} del {r['dl_delivery']:.2f} "
                          f"run {r['dl_loss_run_mean']:.1f} | age p50/p95 {r['age_p50']:.0f}/{r['age_p95']:.0f} "
                          f"cov {r['coverage']:.2f} | {r['wall_s_per_sim_s']:.2f} wall s per sim s", flush=True)
                json.dump(rows, open(os.path.join(out, "rows.json"), "w"), indent=1)
    print("rows:", os.path.join(out, "rows.json"))


if __name__ == "__main__":
    main()
