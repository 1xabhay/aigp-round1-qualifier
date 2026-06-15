"""
Ground truth for validating the vision pipeline.

Because the sim gives us the drone's true pose (telemetry) AND we know the gate
map, we can compute the TRUE bearing/range to a gate and compare it to what the
detector reported. That error series proves the detector works and calibrates
the camera FOV. Pure functions -> unit-tested.

Conventions match gate_detect.bearing_range:
  yaw_off  > 0  -> gate is to the drone's RIGHT
  pitch_off> 0  -> gate is ABOVE the horizontal flight direction
  range    = straight-line distance (m)
"""
from __future__ import annotations

import math

import numpy as np


def gate_truth(drone_pos, drone_yaw: float, gate_pos):
    """True (yaw_off, pitch_off, range) of a gate relative to the drone heading.
    All positions NED; drone_yaw is the heading (rad) from the ATTITUDE message."""
    rel = np.asarray(gate_pos, float) - np.asarray(drone_pos, float)
    rng = float(np.linalg.norm(rel))
    cy, sy = math.cos(drone_yaw), math.sin(drone_yaw)
    fwd = rel[0] * cy + rel[1] * sy           # along heading
    right = -rel[0] * sy + rel[1] * cy        # to the right of heading
    yaw_off = math.atan2(right, fwd)
    horiz = math.hypot(fwd, right)
    pitch_off = math.atan2(-rel[2], horiz)    # NED z down -> -rel[2] is up
    return yaw_off, pitch_off, rng


def active_gate_truth(drone_pos, drone_yaw, gates, active_index):
    """Truth for the gate the racer is currently targeting (clamped)."""
    i = max(0, min(active_index, len(gates) - 1))
    return gate_truth(drone_pos, drone_yaw, gates[i])


def interp_pose(times, positions, yaws, t):
    """Linear-interpolate (pos, yaw) at time t from monotonic samples.
    times: 1D increasing; positions: Nx3; yaws: N (rad, unwrapped is fine for
    the small ranges here). Clamps to the ends."""
    times = np.asarray(times, float)
    if t <= times[0]:
        return np.asarray(positions[0], float), float(yaws[0])
    if t >= times[-1]:
        return np.asarray(positions[-1], float), float(yaws[-1])
    j = int(np.searchsorted(times, t))
    t0, t1 = times[j - 1], times[j]
    a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
    p = (1 - a) * np.asarray(positions[j - 1], float) + a * np.asarray(positions[j], float)
    # interpolate yaw via shortest angular path
    y0, y1 = float(yaws[j - 1]), float(yaws[j])
    dy = math.atan2(math.sin(y1 - y0), math.cos(y1 - y0))
    return p, y0 + a * dy
