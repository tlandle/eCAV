# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""
Python-side bridge for ns-3 co-simulation via shared memory.

Spawns the ns-3 NR V2X process, maps shared memory, and provides
a compute_tick() interface compatible with the analytical channel engine.

Two operating modes:
    "sidelink" (mode=0): NR V2X Mode 2 sidelink for direct V2V
    "uu_uplink" (mode=1): NR Uu uplink for vehicle-to-edge

Usage:
    bridge = NS3Bridge(max_vehicles=128, mode="sidelink")
    bridge.start()

    # Each CARLA tick:
    results = bridge.compute_tick(positions, occlusion_info, tick,
                                  payload_bytes=5000)

    bridge.shutdown()
"""
from __future__ import annotations

import logging
import mmap
import os
import struct
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from .shm_layout import (
    HEADER_SIZE, VEHICLE_ENTRY_SIZE, LINK_RESULT_SIZE, RESULT_HEADER_SIZE,
    STATE_IDLE, STATE_PYTHON_READY, STATE_NS3_DONE, STATE_SHUTDOWN,
    MAX_VEHICLES_DEFAULT,
    compute_shm_size, vehicle_array_offset, link_results_offset,
    result_header_offset,
)

logger = logging.getLogger("NS3Bridge")

# Default ns-3 binary search paths
_NS3_SOURCE_DIR = Path(__file__).parent / "ns-3-dev-5glena"

_NS3_BUILD_PATHS = [
    # In-repo build output
    _NS3_SOURCE_DIR / "build" / "scratch" / "ns3-dev-nr_v2x_cosim_server-default",
    _NS3_SOURCE_DIR / "build" / "scratch" / "nr_v2x_cosim_server",
    # External build (if built outside the repo)
    Path("/home/atlas/ns3_cv2x/ns-3-dev-5glena/build/scratch/ns3-dev-nr_v2x_cosim_server-default"),
]


@dataclass
class LinkResult:
    """Per-link result matching the analytical engine interface."""
    tx_id: int
    rx_id: int
    delivered: bool
    sinr_db: float
    delay_ms: float = 1.0


class NS3Bridge:
    """
    Manages the ns-3 subprocess and shared memory for co-simulation.

    The ns-3 process runs continuously. Each tick:
      1. Python writes vehicle positions to shared memory
      2. Python sets state = PYTHON_READY
      3. ns-3 reads positions, runs NR V2X MAC for one tick
      4. ns-3 writes link results, sets state = NS3_DONE
      5. Python reads results
    """

    def __init__(
        self,
        ns3_binary: str = None,
        ns3_build_dir: str = None,
        max_vehicles: int = MAX_VEHICLES_DEFAULT,
        carrier_ghz: float = 5.9,
        bandwidth_mhz: float = 40.0,
        tx_power_dbm: float = 23.0,
        numerology: int = 0,
        mode: str = "sidelink",  # "sidelink" or "uu_uplink"
        tick_duration_s: float = 0.05,
    ):
        self.max_vehicles = max_vehicles
        self.carrier_ghz = carrier_ghz
        self.bandwidth_mhz = bandwidth_mhz
        self.tx_power_dbm = tx_power_dbm
        self.numerology = numerology
        self.mode = 0 if mode == "sidelink" else 1
        self.mode_name = mode
        self.tick_duration_s = tick_duration_s

        # Resolve ns-3 binary path
        self._ns3_binary = self._resolve_binary(ns3_binary, ns3_build_dir)

        self._shm_size = compute_shm_size(max_vehicles)
        self._shm_name = f"/ecav_ns3_cosim_{os.getpid()}"
        self._shm_fd = None
        self._shm_buf = None
        self._process: Optional[subprocess.Popen] = None
        self._started = False

        # Subchannel assignments (for metrics compatibility)
        self._subchannel_assignments: List[int] = []

        # Per-tick timing (for compute instrumentation)
        self._last_channel_ms: float = 0.0
        self._last_prr: float = 0.0

    @staticmethod
    def _resolve_binary(ns3_binary: Optional[str],
                        ns3_build_dir: Optional[str]) -> str:
        """Find the ns-3 co-sim server binary."""
        if ns3_binary:
            p = Path(ns3_binary)
            if p.exists():
                return str(p)
            logger.warning("Specified binary not found: %s", ns3_binary)

        if ns3_build_dir:
            p = Path(ns3_build_dir) / "nr_v2x_cosim_server"
            if p.exists():
                return str(p)

        for p in _NS3_BUILD_PATHS:
            if p.exists():
                return str(p)

        # Fallback to relative path (will fail at start() if not found)
        return str(Path(__file__).parent / "ns3_build" / "nr_v2x_cosim_server")

    def start(self):
        """Create shared memory and spawn ns-3 process."""
        shm_path = f"/dev/shm{self._shm_name}"

        # Create and size the shared memory file
        self._shm_fd = os.open(shm_path, os.O_CREAT | os.O_RDWR, 0o666)
        os.ftruncate(self._shm_fd, self._shm_size)

        # Memory-map it
        self._shm_buf = mmap.mmap(self._shm_fd, self._shm_size)

        # Zero out
        self._shm_buf[:] = b'\x00' * self._shm_size

        # Write max_vehicles to header
        struct.pack_into('<H', self._shm_buf, 6, self.max_vehicles)

        # Set state to a sentinel value so we can detect when ns-3 has
        # started and written STATE_IDLE. Without this, the zeroed SHM
        # already has state=0=STATE_IDLE, causing the wait loop to exit
        # before ns-3 has even launched.
        _STATE_INITIALIZING = 3
        self._shm_buf[8] = _STATE_INITIALIZING

        logger.info("Shared memory created: %s (%d bytes, max %d vehicles)",
                     self._shm_name, self._shm_size, self.max_vehicles)

        # Spawn ns-3 process
        cmd = [
            self._ns3_binary,
            f"--shm-path={shm_path}",
            f"--max-vehicles={self.max_vehicles}",
            f"--carrier-ghz={self.carrier_ghz}",
            f"--bandwidth-mhz={self.bandwidth_mhz}",
            f"--tx-power-dbm={self.tx_power_dbm}",
            f"--numerology={self.numerology}",
            f"--mode={self.mode}",
            f"--tick-duration={self.tick_duration_s}",
        ]
        logger.info("Starting ns-3 (%s): %s", self.mode_name, " ".join(cmd))

        if not Path(self._ns3_binary).exists():
            raise FileNotFoundError(
                f"ns-3 binary not found: {self._ns3_binary}. "
                f"Build it with: cd /home/atlas/ns3_cv2x/ns-3-dev-5glena && "
                f"./ns3 build nr_v2x_cosim_server")

        self._process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # Wait for ns-3 to signal ready
        timeout = 30.0  # 30s for full ns-3 init (topology setup)
        t0 = time.time()
        while time.time() - t0 < timeout:
            state = self._shm_buf[8]
            if state == STATE_IDLE:
                break
            time.sleep(0.01)

        if self._shm_buf[8] != STATE_IDLE:
            stderr = self._process.stderr.read().decode('utf-8', errors='replace')
            raise RuntimeError(
                f"ns-3 process failed to start within {timeout}s. "
                f"stderr: {stderr[:500]}")

        self._started = True
        logger.info("ns-3 process started (pid=%d, mode=%s)",
                     self._process.pid, self.mode_name)

    def compute_tick(
        self,
        positions: list,
        occlusion_info: list,  # unused by ns-3, kept for API compat
        tick: int,
        payload_bytes: int = 5000,
    ) -> List[LinkResult]:
        """
        Run one tick through ns-3 NR V2X MAC simulation.

        Same interface as the analytical ChannelEngine.compute_tick().

        Parameters
        ----------
        positions : list of [x, y, z] per vehicle
        occlusion_info : unused (ns-3 has its own propagation model)
        tick : CARLA simulation tick
        payload_bytes : feature size per vehicle in bytes
            300 = BSM, 1000 = compressed late fusion, 5000 = late fusion,
            50000 = early fusion, 200000 = intermediate fusion (fails on sidelink)
        """
        if not self._started:
            raise RuntimeError("NS3Bridge.start() must be called first")

        N = len(positions)
        if N > self.max_vehicles:
            logger.warning("N=%d exceeds max_vehicles=%d, clamping",
                           N, self.max_vehicles)
            N = self.max_vehicles

        # Clamp payload_bytes to uint16 range for SHM header
        shm_payload = min(payload_bytes, 65535)

        buf = self._shm_buf

        # Write header
        struct.pack_into('<I', buf, 0, tick)            # tick
        struct.pack_into('<H', buf, 4, N)               # n_vehicles
        struct.pack_into('<B', buf, 9, self.mode)       # mode
        struct.pack_into('<H', buf, 10, shm_payload)    # payload_bytes
        struct.pack_into('<f', buf, 12, tick * self.tick_duration_s)  # sim_time_s

        # Write vehicle positions
        veh_offset = vehicle_array_offset()
        for i in range(N):
            off = veh_offset + i * VEHICLE_ENTRY_SIZE
            struct.pack_into('<fff', buf, off,
                             positions[i][0], positions[i][1], positions[i][2])
            struct.pack_into('<f', buf, off + 12, 0.0)  # heading
            buf[off + 16] = 1  # tx_flag = transmitting

        # Signal ns-3
        buf[8] = STATE_PYTHON_READY

        # Wait for ns-3 to finish. First tick takes longer because it
        # triggers NR V2X topology setup (node creation, bearer activation).
        # With 128 max_vehicles, setup takes ~30s.
        if not hasattr(self, '_first_tick_done'):
            timeout = 60.0
        else:
            timeout = 5.0
        self._first_tick_done = True
        t0 = time.time()
        while time.time() - t0 < timeout:
            if buf[8] == STATE_NS3_DONE:
                break
            time.sleep(0.0001)

        if buf[8] != STATE_NS3_DONE:
            elapsed = (time.time() - t0) * 1000
            logger.warning("ns-3 tick %d timed out after %.0fms", tick, elapsed)
            # Check if process is still alive
            if self._process and self._process.poll() is not None:
                logger.error("ns-3 process died (exit code %d)",
                           self._process.returncode)
            return []

        # Read result header
        rh_offset = result_header_offset(self.max_vehicles)
        n_results, prr, mean_sinr, channel_ms = struct.unpack_from(
            '<Ifff', buf, rh_offset)
        self._last_channel_ms = channel_ms
        self._last_prr = prr

        # Read link results
        lr_offset = link_results_offset(self.max_vehicles)
        results = []
        for i in range(min(n_results, N * (N - 1))):
            off = lr_offset + i * LINK_RESULT_SIZE
            tx_id, rx_id, delivered, _, sinr_x10 = struct.unpack_from(
                '<HHBBh', buf, off)
            results.append(LinkResult(
                tx_id=tx_id,
                rx_id=rx_id,
                delivered=bool(delivered),
                sinr_db=sinr_x10 / 10.0,
            ))

        # Reset state
        buf[8] = STATE_IDLE

        # Update subchannel assignments for metrics compatibility
        self._subchannel_assignments = list(range(N))

        return results

    def get_subchannel_assignments(self) -> List[int]:
        return self._subchannel_assignments

    def get_last_timing(self) -> Dict[str, float]:
        """Return timing from the last compute_tick call."""
        return {
            "channel_ms": self._last_channel_ms,
            "prr": self._last_prr,
        }

    def reset(self):
        if self._shm_buf:
            self._shm_buf[8] = STATE_IDLE

    def shutdown(self):
        """Stop ns-3 and clean up shared memory."""
        if self._shm_buf:
            self._shm_buf[8] = STATE_SHUTDOWN
            time.sleep(0.1)

        if self._process:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait()
            logger.info("ns-3 process terminated")

        if self._shm_buf:
            self._shm_buf.close()
            self._shm_buf = None
        if self._shm_fd is not None:
            os.close(self._shm_fd)
            shm_path = f"/dev/shm{self._shm_name}"
            if os.path.exists(shm_path):
                os.unlink(shm_path)
            self._shm_fd = None
            logger.info("Shared memory cleaned up")

        self._started = False

    @staticmethod
    def backhaul_queue_delay_ms(n_vehicles, feature_bytes=175000,
                                 backhaul_bw_mbps=100.0):
        """Backhaul delay for edge comparison (same as analytical)."""
        total_bits = n_vehicles * feature_bytes * 8.0
        bw_bps = backhaul_bw_mbps * 1e6
        return (total_bits / bw_bps) * 1000.0 * 0.5
