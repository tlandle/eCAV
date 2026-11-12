#!/usr/bin/env python3
"""Per-tick breakdown of one launch_scale.sh results directory, from the lines the
orchestrator, the server and the clients already log.

    python scripts/ispass/scale_breakdown.py experiment_results/ispass_scale/<tag> [--skip 5]
"""
import glob
import re
import statistics as st
import sys


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * q))] if v else float("nan")


def row(name, v):
    if not v:
        return f"  {name:<34} no samples"
    return f"  {name:<34} mean {st.mean(v):7.1f}  p50 {pct(v, .5):7.1f}  p99 {pct(v, .99):7.1f} ms  (n={len(v)})"


def main():
    d = sys.argv[1]
    skip = int(sys.argv[sys.argv.index("--skip") + 1]) if "--skip" in sys.argv else 5
    orch = "".join(open(f, errors="ignore").read() for f in glob.glob(f"{d}/*/orch.log"))
    wall, clients, world = [], [], []
    for m in re.finditer(r"SCALE_TOWN06\] tick (\d+) wall=(\d+)ms clients=(\d+)ms world=(\d+)ms", orch):
        if int(m.group(1)) >= skip:
            wall.append(int(m.group(2))); clients.append(int(m.group(3))); world.append(int(m.group(4)))
    send, wait, unpack = [], [], []
    for m in re.finditer(r"SERVER TICK (\d+)\] send_cmd=(\d+)ms \| wait_clients=(\d+)ms \| unpack=(\d+)ms", orch):
        if int(m.group(1)) >= skip:
            send.append(int(m.group(2))); wait.append(int(m.group(3))); unpack.append(int(m.group(4)))
    veh = {k: [] for k in ("update_info", "run_step", "pickle_obj", "pickle_feat", "total", "send_update", "wait_for_next_cmd")}
    rsu = {k: [] for k in ("total",)}
    nv = nr = 0
    for f in glob.glob(f"{d}/*/veh_*.log") + glob.glob(f"{d}/*/rsu_*.log"):
        is_v = "/veh_" in f
        nv += is_v; nr += (not is_v)
        t = open(f, errors="ignore").read()
        for m in re.finditer(r"CLIENT (?:VEHICLE|RSU) TICK (\d+)\] update_info=(\d+)ms \| run_step=(\d+)ms \| pickle_obj=(\d+)ms \| pickle_feat=(\d+)ms \| total=(\d+)ms", t):
            if int(m.group(1)) < skip:
                continue
            if is_v:
                for k, g in zip(("update_info", "run_step", "pickle_obj", "pickle_feat", "total"), m.groups()[1:]):
                    veh[k].append(int(g))
            else:
                rsu["total"].append(int(m.group(6)))
        if is_v:
            for k in ("send_update", "wait_for_next_cmd"):
                veh[k] += [int(x) for x in re.findall(rf"CLIENT VEHICLE\] {k}=(\d+)ms", t)][skip:]
    print(f"{d}: {nv} vehicle clients, {nr} RSU clients, {len(wall)} ticks after the first {skip}")
    print(row("orchestrator tick wall", wall))
    print(row("  barrier (broadcast to all done)", clients))
    print(row("  world tick", world))
    print(row("server send_cmd", send)); print(row("server wait_clients", wait)); print(row("server unpack", unpack))
    for k in ("total", "update_info", "run_step", "pickle_obj", "send_update", "wait_for_next_cmd"):
        print(row(f"vehicle client {k}", veh[k]))
    print(row("RSU client total", rsu["total"]))


if __name__ == "__main__":
    main()
