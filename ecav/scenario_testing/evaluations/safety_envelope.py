# -*- coding: utf-8 -*-
# Author: Tyler Landle <tlandle3@gatech.edu>
# License: TDG-Attribution-NonCommercial-NoDistrib

"""Analytical AoI safety envelope for the LTAP conflict.

Formalizes how ego speed, cross-traffic speed, and age-at-use (AoI) jointly
bound safe operation, and when cooperative perception (which trades earlier
sight for staleness) is worth its AoI. Pure analysis: no CARLA, no sim.

Time-reference discipline (avoid double-counting AoI)
-----------------------------------------------------
The AoI term must be charged in exactly ONE reference frame.

Form A -- generation time t_g (edge observation is generated). The ego must
still have room to absorb communication age (Delta_use) PLUS actuation delay
(rho), evaluated at the ego state at t_g:
    M_gen = d_e(t_g) - [ v_e(t_g)*(Delta_use + rho) + v_e(t_g)^2/(2 a_b) + d_buf ]
This is the form for SCENARIO TUNING: pick d_e(t_g), v_e so that
    Delta_max = (d_e(t_g) - d_buf - v_e(t_g)^2/(2 a_b))/v_e(t_g) - rho
lands at the target (e.g. 0.25-0.30 s).

Form B -- planner use time t_u (planner acts). The ego is already closer (it
has been travelling during Delta_use), so Delta_use is ALREADY PAID by the
smaller d_e(t_u). Do NOT add Delta_use again:
    M_use = d_e(t_u) - [ v_e(t_u)*rho + v_e(t_u)^2/(2 a_b) + d_buf ]
This is the form for LOG VALIDATION (the per-tick logger sees t_u state).

M_gen and M_use agree approximately when ego speed is stable across [t_g, t_u].

Cross-traffic
-------------
Cross-traffic speed does NOT enter the ego stopping distance. It enters (1) the
truth error of stale state and (2) the clear-vs-yield decision. A raw stale
observation generated Delta_use earlier believes cross-traffic is farther by
    eps_c = v_c * Delta_use   (or a prediction-residual eps_c(Delta) if the edge
                               extrapolates: 0.5*a_c_max*D^2 + sigma_v*D + sigma_x)
so the planner OVER-estimates time-to-conflict:
    d_c_believed = d_c_true + eps_c
    T_c_believed = T_c_true + Delta_use     (raw, constant v_c)

Clear condition (ego proceeds through the conflict before cross-traffic):
    T_e_clear + T_buf <= T_c_true            (truly safe)
With stale state the planner may believe T_e_clear + T_buf <= T_c_believed while
the true inequality is false -> the stale-true-obstacle failure.

Cooperative-perception value:
    raw:        d_coop - v_c*Delta_use > d_los   ->  Delta* = (d_coop - d_los)/v_c
    predicted:  d_coop - eps_c(Delta_use) > d_los
Above Delta*, the stale cooperative detection gives no better usable warning
than local line of sight. Delta* shrinks as v_c grows.
"""
from __future__ import annotations

import math
from typing import Optional


# ---------------------------------------------------------------- ego stopping
def stop_distance(v_e: float, response_s: float, a_b: float,
                  d_buf: float = 0.0) -> float:
    """d_stop = v*response + v^2/(2 a_b) + d_buf. response is rho (+Delta_use
    only in the generation-time form)."""
    if v_e <= 0.0:
        return d_buf
    return v_e * response_s + (v_e * v_e) / (2.0 * a_b) + d_buf


def margin_gen(d_e_gen: float, v_e_gen: float, a_b: float, rho: float,
               d_buf: float, d_use: float) -> float:
    """Form A: AoI charged via response = rho + Delta_use, at t_g state."""
    return d_e_gen - stop_distance(v_e_gen, rho + d_use, a_b, d_buf)


def margin_use(d_e_use: float, v_e_use: float, a_b: float, rho: float,
               d_buf: float) -> float:
    """Form B: AoI already paid by the smaller d_e(t_u); response = rho only."""
    return d_e_use - stop_distance(v_e_use, rho, a_b, d_buf)


def delta_max_yield(d_e_gen: float, v_e_gen: float, a_b: float, rho: float,
                    d_buf: float) -> float:
    """Max admissible AoI for the yield (stop-before-conflict) condition,
    generation-time form. Inf if essentially stopped."""
    if v_e_gen <= 0.1:
        return float("inf")
    return (d_e_gen - d_buf - (v_e_gen * v_e_gen) / (2.0 * a_b)) / v_e_gen - rho


def ego_speed_for_target_delta(d_e_gen: float, delta_target: float, a_b: float,
                               rho: float, d_buf: float) -> Optional[float]:
    """Ego speed putting delta_max_yield at delta_target for a given d_e(t_g).
    Solves v^2/(2a) + (delta_target+rho)*v - (d_e - d_buf) = 0 for v>0."""
    A = 1.0 / (2.0 * a_b)
    B = delta_target + rho
    C = -(d_e_gen - d_buf)
    disc = B * B - 4 * A * C
    if disc < 0:
        return None
    v = (-B + math.sqrt(disc)) / (2 * A)
    return v if v > 0 else None


# ----------------------------------------------------------- cross-traffic
def eps_cross_raw(v_c: float, d_use: float) -> float:
    """Stale raw-detection position error: v_c * Delta_use."""
    return v_c * d_use


def eps_cross_predicted(d_use: float, a_c_max: float = 2.0,
                        sigma_v: float = 0.5, sigma_x: float = 0.3) -> float:
    """Prediction-residual error when the edge extrapolates cross-traffic:
    0.5*a_c_max*D^2 + sigma_v*D + sigma_x. Smaller than raw for moderate D."""
    return 0.5 * a_c_max * d_use * d_use + sigma_v * d_use + sigma_x


def t_to_conflict(dist: float, speed: float) -> float:
    return dist / speed if speed > 0.1 else float("inf")


def delta_star_coop_vs_los(d_coop: float, d_los: float, v_c: float,
                           predicted: bool = False, **eps_kw) -> float:
    """AoI crossover where cooperative perception stops beating line of sight.
    raw: (d_coop - d_los)/v_c. predicted: solve d_coop - eps_c(D) = d_los."""
    if predicted:
        # solve 0.5 a D^2 + sigma_v D + sigma_x = (d_coop - d_los)
        a_c = eps_kw.get("a_c_max", 2.0)
        sv = eps_kw.get("sigma_v", 0.5)
        sx = eps_kw.get("sigma_x", 0.3)
        rhs = d_coop - d_los - sx
        A, B, C = 0.5 * a_c, sv, -rhs
        disc = B * B - 4 * A * C
        if disc < 0:
            return 0.0
        D = (-B + math.sqrt(disc)) / (2 * A)
        return max(0.0, D)
    if v_c <= 0.1:
        return float("inf")
    return max(0.0, (d_coop - d_los) / v_c)


# ----------------------------------------------------------- full report
def classify(d_e_gen, v_e_gen, d_e_use, v_e_use, d_c_use, v_c, d_use,
             a_b=6.0, rho=0.1, d_buf=1.0, t_buf=0.5,
             d_e_clear=8.0, d_coop=70.0, d_los=11.0, predicted=False):
    """Compute both margin forms, cross-traffic believed time, clear/yield
    conditions, and the classification flags. Returns a dict ready to print."""
    Mg = margin_gen(d_e_gen, v_e_gen, a_b, rho, d_buf, d_use)
    Mu = margin_use(d_e_use, v_e_use, a_b, rho, d_buf)
    dmax = delta_max_yield(d_e_gen, v_e_gen, a_b, rho, d_buf)

    eps = (eps_cross_predicted(d_use) if predicted else eps_cross_raw(v_c, d_use))
    d_c_believed = d_c_use + eps
    T_c_true = t_to_conflict(d_c_use, v_c)
    T_c_believed = t_to_conflict(d_c_believed, v_c)
    T_e_clear = t_to_conflict(d_e_clear, v_e_use)

    dstar = delta_star_coop_vs_los(d_coop, d_los, v_c, predicted=predicted)

    return {
        "ego_speed_gen": v_e_gen, "ego_speed_use": v_e_use, "cross_speed": v_c,
        "d_e_gen": d_e_gen, "d_e_use": d_e_use, "d_c_use": d_c_use,
        "d_c_believed": d_c_believed, "eps_c": eps, "d_use": d_use,
        "T_e_clear": T_e_clear, "T_c_true": T_c_true, "T_c_believed": T_c_believed,
        "M_gen": Mg, "M_use": Mu,
        "delta_max_yield": dmax, "delta_star_coop_vs_los": dstar,
        # classification flags
        "yield_safe": Mg >= 0,
        "clear_safe": (T_e_clear + t_buf) <= T_c_true,
        "stale_track_dangerous": ((T_e_clear + t_buf) <= T_c_believed)
                                 and not ((T_e_clear + t_buf) <= T_c_true),
        "coop_beats_los": d_use < dstar,
    }
