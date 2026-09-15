"""Duplicate-uploader counters for the landers.

The edge manager's vehicle collection loop was carried twice between
2026-04-30 (befaa080) and 2026-09-14 (2ee053c3 / 8a2a24e2), so each CAV
entered the fusion batch and the uplink charge twice. These counters make
a recurrence visible in every landed row:
  feature_dup_ticks  edge ticks whose "[WorldFusion Edge] Collected N
                     feature_dicts from M vehicles + R RSUs" print has
                     N > M + R (more entries than agents), all runs;
  radio_dup_ticks    sum of n_dup over [RADIOROW] rows (co-simulated runs);
  cosim_live_ticks   [RADIOROW] rows with n_results > 0 (co-simulated runs).
"""
import re

_COLLECTED = re.compile(r"Collected (\d+) feature_dicts from (\d+) vehicles \+ (\d+) RSUs")
_RADIOROW = re.compile(r"\[RADIOROW\][^\n]*?n_dup=(\d+)[^\n]*?n_results=(\d+)")


def counts(text):
    feature_dup = 0
    for m in _COLLECTED.finditer(text):
        n, mv, r = int(m.group(1)), int(m.group(2)), int(m.group(3))
        if n > mv + r:
            feature_dup += 1
    radio_dup = 0
    live = 0
    for m in _RADIOROW.finditer(text):
        radio_dup += int(m.group(1))
        if int(m.group(2)) > 0:
            live += 1
    return {"feature_dup_ticks": feature_dup, "radio_dup_ticks": radio_dup, "cosim_live_ticks": live}
