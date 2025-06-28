#!/usr/bin/env python3
"""
size_probe.py  –  Runtime byte-counter for LSI uplink messages.

• Accepts the exact ObstacleVehicle objects created by your pipeline.
• Encodes each vehicle’s self-beacon (96 B) and its variable-length
  3-D bounding-box list (≤10 boxes) using the same binary layout that
  goes on the network.
• Maintains running {avg, p95} and worst-case bandwidth at 10 Hz / 20 Hz.
"""

import struct, statistics as st
from collections import defaultdict

# ------------- binary helpers (little-endian, 4-byte align) -------------
HDR  = "<HH"   # type, len
CRC4 = "<I"
I32  = "<I"
F32  = "<f"
U64  = "<Q"
PAD1 = "<3x"

def pack_beacon(v) -> bytes:
    """96-byte self-beacon from one ObstacleVehicle."""
    header  = struct.pack(HDR, 0x01, 96)
    payload = (
        struct.pack(I32,  v.carla_id)         # vehicle_id
      + struct.pack(U64,  0)                  # timestamp_ns (stub)
      + struct.pack(3*F32,
            v.location.x, v.location.y, v.location.z)
      + struct.pack(3*F32,
            v.velocity.x, v.velocity.y, v.velocity.z)
      + struct.pack(3*F32,
            v.bounding_box.extent.x,
            v.bounding_box.extent.y,
            v.bounding_box.extent.z)
      + struct.pack(F32, 0.0)                 # yaw (stub)
    )
    crc     = struct.pack(CRC4, 0)
    return header + payload + crc             # 96 B

def pack_box(v, track_id: int) -> bytes:
    """48-byte 3-D bounding-box record."""
    bb  = v.bounding_box
    loc = v.location
    return (
        struct.pack(I32, track_id)
      + struct.pack(3*F32, loc.x, loc.y, loc.z)
      + struct.pack(3*F32, bb.extent.x, bb.extent.y, bb.extent.z)
      + struct.pack(F32, 0.0)                 # yaw (stub)
      + struct.pack(3*F32,
            v.velocity.x, v.velocity.y, v.velocity.z)
      + struct.pack("<B", 1) + struct.pack(PAD1)
    )

def pack_box_list(v_list) -> bytes:
    """List message for exactly one vehicle (≤10 boxes)."""
    n = min(len(v_list), 10)
    header = struct.pack(HDR, 0x02, 0)        # len patched later
    count  = struct.pack(I32, n)
    boxes  = b"".join(pack_box(v_list[i], i) for i in range(n))
    crc    = struct.pack(CRC4, 0)
    payload = count + boxes + crc
    length  = len(header) + len(payload)
    header  = struct.pack(HDR, 0x02, length)
    return header + payload

# -------------------- statistics collector --------------------
_stats = defaultdict(list)   # key -> list of sizes

def measure_frame(vehicles):
    """
    Call once per simulation step.
    • vehicles: iterable of ObstacleVehicle seen in that frame.
    """
    for v in vehicles:
        b_beacon = pack_beacon(v)
        _stats["beacon"].append(len(b_beacon))

        # gather the vehicle’s own bbox plus at most 9 neighbours
        bbox_list = pack_box_list([v] + [o for o in vehicles if o is not v][:9])
        _stats["bbox_list"].append(len(bbox_list))

        _stats["total"].append(len(b_beacon) + len(bbox_list))

def report(hz: int = 20):
    """Print avg / p95 sizes and KB/s at the given publish rate."""
    for key in ("beacon", "bbox_list", "total"):
        s = _stats[key]
        avg = st.mean(s)
        p95 = st.quantiles(s, n=20)[18] if len(s) >= 20 else max(s)
        bw  = p95 * hz / 1024
        print(f"{key:10s}: avg {avg:6.0f} B   p95 {p95:6.0f} B"
              f"   worst-case {bw:5.2f} KB/s @ {hz} Hz")

# ---------------- Example use inside your sim loop --------------
# for frame in sim:
#     vehicles = get_obstacle_vehicle_list_for_frame(frame)
#     measure_frame(vehicles)
#
# # end of run
# report(hz=20)
