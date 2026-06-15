"""
Vision fusion: turn a gate detection into a CORRECTION of the flight target.

The map (hardcoded gate centres) is our prior; vision tells us where the gate
REALLY is. When a detection is fresh and trustworthy we nudge the target by a
fraction of the map error (seen_gate - map_gate); otherwise we trust the map.

`gate_from_observation` is the exact inverse of vision_truth.gate_truth, so the
correction is in metric world (NED) coordinates. Pure functions -> unit-tested.
"""
from __future__ import annotations

import math

import numpy as np


def gate_from_observation(drone_pos, drone_yaw: float, yaw_off: float,
                          pitch_off: float, range_m: float) -> np.ndarray:
    """Inverse of vision_truth.gate_truth: (bearing + range) -> world gate position
    (NED). yaw_off>0 = right of heading; pitch_off>0 = above; range = distance."""
    horiz = range_m * math.cos(pitch_off)
    fwd = horiz * math.cos(yaw_off)         # along heading
    right = horiz * math.sin(yaw_off)       # right of heading
    up = range_m * math.sin(pitch_off)
    cy, sy = math.cos(drone_yaw), math.sin(drone_yaw)
    dN = fwd * cy - right * sy              # rotate body->world by heading
    dE = fwd * sy + right * cy
    dD = -up                                # NED z is down
    return np.asarray(drone_pos, dtype=float) + np.array([dN, dE, dD])


def gated_offset(map_gate, drone_pos, drone_yaw: float, obs, age: float, *,
                 max_age: float = 0.3, min_range: float = 6.0, max_range: float = 35.0,
                 yaw_max: float = math.radians(25), range_tol: float = 6.0,
                 gain: float = 0.5):
    """(valid, offset). offset = gain*(seen_gate - map_gate); valid=True ONLY when
    the detection is confidently the ACTIVE gate ahead (data association — the
    missing piece the A/B test exposed). valid=False when:
      * no/old detection (age > max_age) or gain <= 0
      * range not trackable: not (min_range < range <= max_range)  (too close ->
        the gate fills the frame and bearing/range geometry breaks)
      * gate not ahead: |yaw_off| > yaw_max  (off to the side / being passed)
      * range disagrees with the map's active gate (a DIFFERENT gate)."""
    map_gate = np.asarray(map_gate, dtype=float)
    drone_pos = np.asarray(drone_pos, dtype=float)
    if obs is None or not getattr(obs, "detected", False) or age > max_age or gain <= 0.0:
        return False, np.zeros(3)
    if not (min_range < obs.range_m <= max_range) or abs(obs.yaw_off) > yaw_max:
        return False, np.zeros(3)
    if abs(obs.range_m - float(np.linalg.norm(map_gate - drone_pos))) > range_tol:
        return False, np.zeros(3)
    seen = gate_from_observation(drone_pos, drone_yaw, obs.yaw_off, obs.pitch_off, obs.range_m)
    rel = seen - map_gate
    # Correct only the LATERAL+vertical error (driven by the accurate BEARING,
    # ~0.3 deg); drop the along-heading component (driven by NOISY range, ~2.3 m)
    # so range jitter doesn't slosh the target forward/back.
    fwd = np.array([math.cos(drone_yaw), math.sin(drone_yaw), 0.0])
    lateral = rel - float(rel @ fwd) * fwd
    return True, gain * lateral


def vision_correction(map_gate, drone_pos, drone_yaw: float, obs, age: float, **kw) -> np.ndarray:
    """Instantaneous gated correction vector (0 if the detection isn't trustworthy)."""
    return gated_offset(map_gate, drone_pos, drone_yaw, obs, age, **kw)[1]


class MapErrorEstimator:
    """Recursive estimate of the map error, the fusion 'pooling' layer. A valid
    detection low-passes the estimate toward the observed offset; with no fresh
    info it HOLDS (slowly decays). So the correction PERSISTS between detections
    and through close range (where detection is gated off) — instead of a per-tick
    nudge that evaporates and leaves the drone on the wrong (map) line."""

    def __init__(self, alpha: float = 0.25, decay: float = 0.995, clamp: float = 2.5):
        self.offset = np.zeros(3)
        self.alpha = alpha
        self.decay = decay
        self.clamp = clamp

    def update(self, map_gate, drone_pos, drone_yaw, obs, age, **kw) -> np.ndarray:
        valid, raw = gated_offset(map_gate, drone_pos, drone_yaw, obs, age, **kw)
        if valid:
            self.offset = (1 - self.alpha) * self.offset + self.alpha * raw
        else:
            self.offset = self.offset * self.decay        # hold, slowly forget
        m = float(np.linalg.norm(self.offset))
        if m > self.clamp:
            self.offset = self.offset * (self.clamp / m)
        return self.offset.copy()


class GateMapEstimator:
    """Per-gate corrected map. Each gate keeps a low-passed lateral offset toward
    where vision sees it; corrected() returns map+offset per gate. We then REBUILD
    the spline through the corrected gates -> a correctly-SHAPED line (fixes the
    wedging from shifting a wrong-shaped path). In the limit (map fully wrong /
    absent) this becomes vision-only navigation. Hold (no decay) between sightings."""

    def __init__(self, map_gates, alpha: float = 0.25, clamp: float = 2.5):
        self.map = [np.asarray(g, dtype=float) for g in map_gates]
        self.offset = [np.zeros(3) for _ in self.map]
        self.alpha = alpha
        self.clamp = clamp

    def update(self, active: int, drone_pos, drone_yaw, obs, age, **kw) -> bool:
        i = max(0, min(active, len(self.map) - 1))
        valid, raw = gated_offset(self.map[i], drone_pos, drone_yaw, obs, age, **kw)
        if valid:
            self.offset[i] = (1 - self.alpha) * self.offset[i] + self.alpha * raw
            m = float(np.linalg.norm(self.offset[i]))
            if m > self.clamp:
                self.offset[i] = self.offset[i] * (self.clamp / m)
        return valid

    def corrected(self):
        return [self.map[i] + self.offset[i] for i in range(len(self.map))]

    def max_offset(self) -> float:
        return max((float(np.linalg.norm(o)) for o in self.offset), default=0.0)
