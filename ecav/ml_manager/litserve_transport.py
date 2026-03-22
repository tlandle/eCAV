# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib
"""
Shared transport utilities for LitServe HTTP clients.

Used by both MLManager (YOLO gRPC path is separate, but WorldFusion HTTP path
uses these) and WorldFusionPerceptionManager.
"""
import csv
import os

import requests


def make_session() -> requests.Session:
    """Create a persistent requests.Session for LitServe HTTP calls.

    Reusing a Session keeps the underlying TCP connection alive across ticks,
    avoiding the per-request handshake overhead of bare requests.post().
    """
    s = requests.Session()
    adapter = requests.adapters.HTTPAdapter(max_retries=0)
    s.mount("http://", adapter)
    return s


class TimingRecorder:
    """Accumulate per-tick timing rows and dump to CSV at teardown.

    Parameters
    ----------
    fieldnames : list[str]
        Ordered column names for the CSV header.
    """

    def __init__(self, fieldnames: list):
        self.fieldnames = fieldnames
        self._rows: list = []

    def record(self, row: dict):
        """Append one timing row. Extra keys are silently ignored at dump time."""
        self._rows.append(row)

    def dump_csv(self, path: str):
        """Write accumulated rows to *path* as CSV. No-op if no rows recorded."""
        if not self._rows:
            return
        dir_ = os.path.dirname(path)
        if dir_:
            os.makedirs(dir_, exist_ok=True)
        with open(path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self._rows)

    def __len__(self):
        return len(self._rows)
