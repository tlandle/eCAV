#!/usr/bin/env python3
"""Per-tick breakdown of one launch_scale.sh results directory.

    python scripts/ispass/scale_breakdown.py experiment_results/ispass_scale/<tag> [--skip 5] [--row]

The orchestrator, the edge process and every client log absolute times per tick
(SERVER TICK, EDGE TICK, CLIENT TIMING lines). The tick wall time is partitioned
along the critical path, the client that reports last in that tick:

    orchestrator -> edge        Server_DoTick, C++ server, push to the edge
    edge fan-out                edge receives the tick until the last-reporting client has it
    client loop gap             tick in that client's queue until its tick() starts on it
                                (scenario tree tick and world snapshot of that process)
    client compute              update_info + run_step + serialization
    client reply                Edge_ActorSendUpdate: upload, the forecasts in the reply, until
                                the edge has counted every actor
    edge step                   edge manager run_step
    edge -> orchestrator        Edge_TickComplete, C++ server, push to the orchestrator
    orchestrator unpack         vehicle updates pulled and unpacked
    world tick                  CARLA world.tick()
    unattributed                tick wall minus the rows above

Clocks: all rows use time.time() of the logging process, so cross-process rows need
synchronised clocks (one host, or chrony between hosts). --row prints one table row.
"""
import glob
import os
import re
import statistics as st
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * q))] if v else float("nan")


def stats(v):
    return (st.mean(v), pct(v, .5), pct(v, .99)) if v else (float("nan"),) * 3


def cpu_busy(d):
    """Busy fraction per host between first tick and end, from /proc/stat snapshots."""
    out = {}
    for a in glob.glob(f"{d}/*/cpu_start.log"):
        b = a.replace("cpu_start", "cpu_end")
        if not os.path.exists(b):
            continue
        try:
            x = [int(t) for t in open(a).read().split()[1:9]]
            y = [int(t) for t in open(b).read().split()[1:9]]
        except (ValueError, IndexError):
            continue
        tot = sum(y) - sum(x)
        idle = (y[3] + y[4]) - (x[3] + x[4])
        if tot > 0:
            out[os.path.basename(os.path.dirname(a))] = 1.0 - idle / tot
    return out


def main():
    d = sys.argv[1].rstrip("/")
    skip = int(sys.argv[sys.argv.index("--skip") + 1]) if "--skip" in sys.argv else 5
    orch = "".join(open(f, errors="ignore").read() for f in glob.glob(f"{d}/*/orch.log"))
    edge = "".join(open(f, errors="ignore").read() for f in glob.glob(f"{d}/*/edge.log"))
    wall, world = {}, {}
    order = []
    for m in re.finditer(r"SCALE_TOWN06\] tick (\d+) wall=(\d+)ms clients=(\d+)ms world=(\d+)ms", orch):
        order.append(int(m.group(1)))
    # exact wall and world from the orchestrator line are integers in ms; keep them per step
    for m in re.finditer(r"SCALE_TOWN06\] tick (\d+) wall=(\d+)ms clients=(\d+)ms world=(\d+)ms", orch):
        wall[int(m.group(1))] = (int(m.group(2)), int(m.group(3)), int(m.group(4)))
    srv = []
    for m in re.finditer(r"SERVER TICK (\d+)\] send_cmd=(\d+)ms \| wait_clients=(\d+)ms \| unpack=(\d+)ms"
                         r"(?: \| t_start=([\d.]+) t_sent=([\d.]+) t_done=([\d.]+))?", orch):
        srv.append((int(m.group(1)), int(m.group(4)),
                    *(float(g) if g else None for g in m.groups()[4:7])))
    edg = {}
    for m in re.finditer(r"EDGE TICK (\d+)\].*?t_recv=([\d.]+) t_dequeued=([\d.]+) t_pushed=([\d.]+) "
                         r"t_first=([\d.]+) t_last=([\d.]+) t_step=([\d.]+) t_reported=([\d.]+)", edge):
        edg[int(m.group(1))] = [float(g) for g in m.groups()[1:]]
    cli = {}      # tick -> list of (t_recv, t_start, t_built, t_sent, label, index)
    nv = nr = 0
    comp = []
    for f in glob.glob(f"{d}/*/veh_*.log") + glob.glob(f"{d}/*/rsu_*.log"):
        is_v = "/veh_" in f
        nv += is_v
        nr += (not is_v)
        t = open(f, errors="ignore").read()
        for m in re.finditer(r"CLIENT TIMING (\w+) (-?\d+) TICK (\d+)\] t_recv=([\d.]+) t_start=([\d.]+) "
                             r"t_built=([\d.]+) t_sent=([\d.]+)", t):
            tk = int(m.group(3))
            r = [float(g) for g in m.groups()[3:]]
            cli.setdefault(tk, []).append((*r, m.group(1), int(m.group(2))))
            if is_v and tk >= skip:
                comp.append((r[2] - r[1]) * 1000)
    names = ["orchestrator -> edge", "edge fan-out", "client loop gap", "client compute",
             "client reply", "edge step", "edge -> orchestrator", "orchestrator unpack",
             "world tick", "unattributed"]
    rows = {n: [] for n in names}
    walls = []
    last_is_rsu = 0
    # the orchestrator's step counter and the server tick id differ by a constant; pair them in order
    srv = [s for s in srv if s[2] is not None]
    steps = sorted(wall)
    off = len(steps) - len(srv)
    for i, s in enumerate(srv):
        tk, unpack, t0, t_sent, t_done = s
        step = steps[i + off] if 0 <= i + off < len(steps) else None
        if step is None or step < skip or tk not in edg or tk not in cli:
            continue
        w, _, wd = wall[step]
        e_recv, e_deq, e_push, e_first, e_last, e_step, e_rep = edg[tk]
        c = max(cli[tk], key=lambda r: r[3])
        last_is_rsu += (c[4] == "RSU")
        seg = {
            "orchestrator -> edge": e_recv - t0,
            "edge fan-out": max(c[0] - e_recv, 0.0),
            "client loop gap": max(c[1] - max(c[0], e_recv), 0.0),
            "client compute": c[2] - c[1],
            "client reply": max(e_last - c[2], 0.0),
            "edge step": e_step - e_last,
            "edge -> orchestrator": t_done - e_step,
        }
        seg = {k: v * 1000 for k, v in seg.items()}
        seg["orchestrator unpack"] = unpack
        seg["world tick"] = wd
        seg["unattributed"] = w - sum(seg.values())
        for k, v in seg.items():
            rows[k].append(v)
        walls.append(w)
    busy = cpu_busy(d)
    wm, w50, w99 = stats(walls)
    launch = open(f"{d}/launch.log", errors="ignore").read() if os.path.exists(f"{d}/launch.log") else ""
    m = re.search(r"first tick after (\d+) s", launch)
    first = int(m.group(1)) if m else float("nan")
    if "--row" in sys.argv:
        cells = [f"{nv}", f"{wm:.0f} / {w50:.0f} / {w99:.0f}"] + [f"{st.mean(rows[n]):.0f}" if rows[n] else "-" for n in names]
        cells += [f"{st.mean(comp):.0f}" if comp else "-", f"{first}",
                  " ".join(f"{v * 100:.0f}%" for v in busy.values()) or "-"]
        print("| " + " | ".join(cells) + " |")
        return
    print(f"{d}: {nv} vehicle clients, {nr} RSU clients, {len(walls)} ticks after the first {skip}")
    print(f"  tick wall                mean {wm:7.1f}  p50 {w50:7.1f}  p99 {w99:7.1f} ms")
    for n in names:
        a, b, c = stats(rows[n])
        share = 100 * a / wm if walls else float("nan")
        print(f"    {n:<22} mean {a:7.1f}  p50 {b:7.1f}  p99 {c:7.1f} ms  {share:5.1f} %")
    if comp:
        print(f"  vehicle client compute, all clients: mean {st.mean(comp):.1f}  p50 {pct(comp, .5):.1f}  p99 {pct(comp, .99):.1f} ms")
    print(f"  last reporter was an RSU in {last_is_rsu} of {len(walls)} ticks")
    print(f"  time to first tick {first} s; CPU busy " + (", ".join(f"{h} {v * 100:.0f} %" for h, v in busy.items()) or "not sampled"))


if __name__ == "__main__":
    main()
