"""
Extract the drone's REAL motion limits from an instrumented run (telemetry
Samples). These measured numbers are what the speed profile / config SHOULD use
instead of guessed a_lat/a_lon/a_brk/v_max — the "physics-powered" half.

Driver-control reading:
  throttle authority -> peak forward accel   (a_lon)
  brake authority    -> peak deceleration    (a_brk)
  grip / steering    -> peak lateral accel    (a_lat)
  top speed          -> max sustained horizontal speed (v_top)
  climb authority    -> max climb rate

All pure functions over a list of telemetry.Sample.
"""
from __future__ import annotations

import numpy as np

from aigp_pilot.telemetry import Sample


def _arrays(samples: list[Sample]):
    t = np.array([s.t for s in samples], dtype=float)
    pos = np.array([[s.x, s.y, s.z] for s in samples], dtype=float)
    vel = np.array([[s.vx, s.vy, s.vz] for s in samples], dtype=float)
    return t, pos, vel


def top_speed(samples) -> float:
    _, _, vel = _arrays(samples)
    return float(np.max(np.linalg.norm(vel[:, :2], axis=1))) if len(vel) else 0.0


def max_climb_rate(samples) -> float:
    _, _, vel = _arrays(samples)
    return float(np.max(-vel[:, 2])) if len(vel) else 0.0   # NED: climb = -vz


# A multirotor at <=45 deg tilt can't exceed ~1 g horizontal; anything past this
# is a COLLISION velocity spike (or telemetry glitch), not thrust authority.
PHYSICAL_ACCEL_CAP = 12.0   # m/s^2


def _accel_samples(samples, cap=PHYSICAL_ACCEL_CAP):
    """Yield (a_long_signed, a_lat, dt) using finite differences of velocity,
    projecting accel onto the (horizontal) direction of travel. Samples whose
    magnitude exceeds `cap` are dropped as collision spikes (non-physical)."""
    t, _, vel = _arrays(samples)
    out = []
    for i in range(1, len(samples)):
        dt = t[i] - t[i - 1]
        if not (0.004 < dt < 0.5):
            continue
        a = (vel[i] - vel[i - 1]) / dt
        a_h = a[:2]
        vdir = vel[i, :2]
        sp = float(np.linalg.norm(vdir))
        if sp < 0.5:                      # direction undefined at low speed
            continue
        if float(np.linalg.norm(a_h)) > cap:   # collision/glitch spike -> reject
            continue
        vhat = vdir / sp
        a_long = float(a_h @ vhat)        # +accelerate, -brake
        a_lat = float(np.linalg.norm(a_h - a_long * vhat))
        out.append((a_long, a_lat, dt))
    return out


def _robust_max(values, q=0.97):
    """High percentile, not the raw max — rejects single-sample telemetry spikes."""
    if not values:
        return 0.0
    return float(np.quantile(np.asarray(values), q))


def measured_limits(samples) -> dict:
    acc = _accel_samples(samples)
    longs = [a for a, _, _ in acc]
    lats = [l for _, l, _ in acc]
    return {
        "v_top": round(top_speed(samples), 2),
        "a_lon": round(_robust_max([a for a in longs if a > 0]), 2),
        "a_brk": round(_robust_max([-a for a in longs if a < 0]), 2),
        "a_lat": round(_robust_max(lats), 2),
        "climb": round(max_climb_rate(samples), 2),
        "n": len(samples),
    }
