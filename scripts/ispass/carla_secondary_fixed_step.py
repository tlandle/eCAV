#!/usr/bin/env python
"""Give a CARLA secondary server (multi-GPU sensor rendering) the simulation's fixed time step.

    python scripts/ispass/carla_secondary_fixed_step.py <rpc port> [fixed_delta_seconds=0.05] [host=127.0.0.1]

A secondary renders one frame per frame the primary sends, but it starts with a variable time
step: its sensors integrate over the wall-clock time between frames (LiDAR sweep size and frame
time stamps then follow the wall clock). CARLA 0.9.15 resets the step at episode start, so
-benchmark -fps on the command line has no effect, and carla.Client.get_world() on the secondary's
port times out. The port does serve RPC calls, so this script calls set_episode_settings on it
directly (msgpack-rpc, with CARLA's call metadata as the first argument). Checked on one vehicle
with four cameras and one LiDAR: with the step set, frame time stamps advance by 0.05 s and the
LiDAR frame has the size of the one-server run.
"""
import socket, sys
import msgpack

port = int(sys.argv[1]); fds = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
host = sys.argv[3] if len(sys.argv) > 3 else "127.0.0.1"


def call(method, *params, timeout=30.0):
    s = socket.create_connection((host, port), timeout=timeout)
    s.sendall(msgpack.packb([0, 1, method, [[False]] + list(params)], use_bin_type=True))   # [False]: synchronous call
    u = msgpack.Unpacker(raw=False)
    while True:
        d = s.recv(65536)
        if not d: raise RuntimeError("connection closed by the server")
        u.feed(d)
        for m in u:
            s.close()
            if m[2] is not None: raise RuntimeError(str(m[2]))
            return m[3][0][1]            # carla::rpc::Response wraps the value


st = list(call("get_episode_settings"))   # [synchronous, no_rendering, optional fixed step, ...]; an optional is [False] or [True, value]
st[0] = True; st[2] = [True, fds]
call("set_episode_settings", st)
after = call("get_episode_settings")
assert after[2] == [True, fds] or (after[2][0] and abs(after[2][1] - fds) < 1e-6), after
print(f"secondary {host}:{port}: fixed_delta_seconds {after[2][1]}")
