# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Shared memory layout for ns-3 co-simulation.

Fixed-size binary structures shared between Python (CARLA side) and
the ns-3 C++ process. No serialization overhead. Both sides read/write
directly to mapped memory.

Layout:
    Header (64 bytes):
        uint32  tick            offset 0
        uint16  n_vehicles      offset 4
        uint16  max_vehicles    offset 6
        uint8   state           offset 8   (0=idle, 1=python_ready, 2=ns3_done)
        uint8   mode            offset 9   (0=sidelink, 1=uu_uplink)
        uint16  payload_bytes   offset 10  (feature size per vehicle)
        float32 sim_time_s      offset 12
        uint8   padding[48]     offset 16

    Vehicle array (max_vehicles * 20 bytes each):
        float32 x               offset 0
        float32 y               offset 4
        float32 z               offset 8
        float32 heading_rad     offset 12
        uint8   tx_flag         offset 16  (1=transmitting this tick)
        uint8   padding[3]      offset 17

    Link results (max_vehicles * max_vehicles * 8 bytes each):
        uint16  tx_id           offset 0
        uint16  rx_id           offset 2
        uint8   delivered       offset 4
        uint8   padding         offset 5
        int16   sinr_db_x10     offset 6  (SINR * 10 for 0.1dB resolution)

    Result header (16 bytes, after link results):
        uint32  n_results       offset 0
        float32 prr             offset 4
        float32 mean_sinr_db    offset 8
        float32 channel_time_ms offset 12
"""

import ctypes
import struct

# Size constants
HEADER_SIZE = 64
VEHICLE_ENTRY_SIZE = 20
LINK_RESULT_SIZE = 8
RESULT_HEADER_SIZE = 16

MAX_VEHICLES_DEFAULT = 128

# State flags
STATE_IDLE = 0
STATE_PYTHON_READY = 1
STATE_NS3_DONE = 2
STATE_SHUTDOWN = 255


def compute_shm_size(max_vehicles: int = MAX_VEHICLES_DEFAULT) -> int:
    """Total shared memory size in bytes."""
    return (HEADER_SIZE
            + max_vehicles * VEHICLE_ENTRY_SIZE
            + max_vehicles * max_vehicles * LINK_RESULT_SIZE
            + RESULT_HEADER_SIZE)


def vehicle_array_offset() -> int:
    return HEADER_SIZE


def link_results_offset(max_vehicles: int) -> int:
    return HEADER_SIZE + max_vehicles * VEHICLE_ENTRY_SIZE


def result_header_offset(max_vehicles: int) -> int:
    return (link_results_offset(max_vehicles)
            + max_vehicles * max_vehicles * LINK_RESULT_SIZE)


# C struct definitions for documentation / ctypes interop
class Header(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("tick", ctypes.c_uint32),
        ("n_vehicles", ctypes.c_uint16),
        ("max_vehicles", ctypes.c_uint16),
        ("state", ctypes.c_uint8),
        ("mode", ctypes.c_uint8),
        ("payload_bytes", ctypes.c_uint16),
        ("sim_time_s", ctypes.c_float),
        ("_pad", ctypes.c_uint8 * 48),
    ]


class VehicleEntry(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("heading_rad", ctypes.c_float),
        ("tx_flag", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 3),
    ]


class LinkResult(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("tx_id", ctypes.c_uint16),
        ("rx_id", ctypes.c_uint16),
        ("delivered", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8),
        ("sinr_db_x10", ctypes.c_int16),
    ]


class ResultHeader(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("n_results", ctypes.c_uint32),
        ("prr", ctypes.c_float),
        ("mean_sinr_db", ctypes.c_float),
        ("channel_time_ms", ctypes.c_float),
    ]
