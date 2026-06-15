"""
Inner controller: desired NED acceleration -> attitude quaternion + throttle.

The sim's stabilised controller holds ATTITUDE only, so this is the loop we owe
it. Standard quadrotor acceleration-to-attitude mapping:

  thrust_vec = a_des - g          (specific force the props must produce, world NED)
                                  (g = [0,0,+9.81]; at hover a_des=0 -> [0,0,-9.81] = up)
  throttle   = hover_thrust * |thrust_vec| / g     (linear thrust model, clamped)
  attitude   = tilt the drone so its body-up (-Z_body) points along thrust_vec,
               with the chosen yaw.

Tilt is capped (max_tilt_deg) so aggressive demands can't flip the vehicle.
hover_thrust comes from probe_hover.py (~0.45 for this sim).
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation

G = 9.81
_G_VEC = np.array([0.0, 0.0, G])  # gravity accel, NED (down positive)


def acceleration_setpoint(v_current, v_target, *, kp: float = 2.0,
                          a_max: float = 10.0) -> np.ndarray:
    """Velocity P loop -> desired NED acceleration, magnitude-saturated."""
    v_current = np.asarray(v_current, dtype=float)
    v_target = np.asarray(v_target, dtype=float)
    a = kp * (v_target - v_current)
    mag = np.linalg.norm(a)
    if mag > a_max:
        a = a * (a_max / mag)
    return a


def _clamp_tilt(thrust_vec: np.ndarray, max_tilt_rad: float) -> np.ndarray:
    """Limit how far thrust_vec leans from straight-up, keeping its magnitude."""
    mag = np.linalg.norm(thrust_vec)
    if mag < 1e-9:
        return np.array([0.0, 0.0, -G])
    up = thrust_vec / mag
    # angle from vertical-up ([0,0,-1] in NED)
    cos_tilt = np.clip(-up[2], -1.0, 1.0)
    tilt = np.arccos(cos_tilt)
    if tilt <= max_tilt_rad:
        return thrust_vec
    # rebuild at the capped tilt: keep horizontal heading, limit horizontal share
    horiz = np.array([up[0], up[1], 0.0])
    hn = np.linalg.norm(horiz)
    if hn < 1e-9:
        return thrust_vec
    horiz /= hn
    capped = (np.array([0.0, 0.0, -1.0]) * np.cos(max_tilt_rad)
              + horiz * np.sin(max_tilt_rad))
    return capped * mag


def attitude_thrust_setpoint(a_des, yaw: float = 0.0, *, hover_thrust: float = 0.45,
                             max_tilt_deg: float = 35.0):
    """Map desired NED acceleration -> (quaternion wxyz, throttle 0..1)."""
    a_des = np.asarray(a_des, dtype=float)
    thrust_vec = a_des - _G_VEC
    thrust_vec = _clamp_tilt(thrust_vec, np.radians(max_tilt_deg))

    # throttle from specific-force magnitude (1 g -> hover_thrust)
    throttle = float(np.clip(hover_thrust * np.linalg.norm(thrust_vec) / G, 0.0, 1.0))

    # build the attitude: body-up (-Z_body) must point along thrust_vec
    up = thrust_vec / np.linalg.norm(thrust_vec)
    z_body = -up                                   # body Z (down) in world
    x_c = np.array([np.cos(yaw), np.sin(yaw), 0.0])  # desired heading
    y_body = np.cross(z_body, x_c)
    y_body /= np.linalg.norm(y_body)
    x_body = np.cross(y_body, z_body)
    rot = np.column_stack([x_body, y_body, z_body])  # body->world

    qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()  # scipy returns xyzw
    return np.array([qw, qx, qy, qz]), throttle


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle (rad) to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def accel_to_tilt(aN: float, aE: float, yaw: float, *, g: float = G,
                  tilt_max: float = math.radians(35)) -> tuple[float, float]:
    """Desired world-frame horizontal acceleration -> (roll, pitch) tilt, rotated
    into the BODY frame via heading `yaw` (heading-robust). Small-angle law:
    to produce horizontal accel a while holding gravity, the thrust vector tilts
    by ~a/g (tan(tilt) = a/g linearised). Each axis clamped to tilt_max.
    Returns (roll_des, pitch_des) in radians."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    body_fwd = aN * cy + aE * sy
    body_right = -aN * sy + aE * cy
    pitch = _clamp(body_fwd / g, -tilt_max, tilt_max)
    roll = _clamp(body_right / g, -tilt_max, tilt_max)
    return roll, pitch


def collective_thrust(az: float, roll: float, pitch: float, *,
                      hover: float = 0.29, g: float = G,
                      thr_min: float = 0.05, thr_max: float = 0.98) -> float:
    """Collective throttle for a desired NED-down acceleration `az`
    (negative = accelerate UP), TILT-COMPENSATED: only cos(tilt) of thrust holds
    altitude when pitched/rolled, so divide it back out. Physics invariant: the
    vertical force thr*cos(tilt) is independent of tilt for a given az (no sag at
    speed). Chains with vertical_accel(); matches the flown thr_hold law."""
    base = hover * (g - az) / g                      # az NED-down -> -az is upward
    tilt_factor = max(math.cos(roll) * math.cos(pitch), 0.5)
    return _clamp(base / tilt_factor, thr_min, thr_max)


def vertical_accel(target_z: float, pos_z: float, vel_z: float, *, vz_ff: float = 0.0,
                   kpz_pos: float = 1.0, kpz_vel: float = 3.0,
                   vz_max: float = 4.5, az_max: float = 10.0) -> float:
    """Cascaded altitude loop (NED, z down): position->velocity->accel, with a
    climb-rate feedforward vz_ff (= path_speed * slope) so altitude doesn't lag.
    Returns NED-down acceleration az (negative = accelerate upward)."""
    vz = _clamp(vz_ff + kpz_pos * (target_z - pos_z), -vz_max, vz_max)
    return _clamp(kpz_vel * (vz - vel_z), -az_max, az_max)


def attitude_rate_command(meas_rpy, des_rpy, *, kp: float = 2.0,
                          max_rate: float = 2.0) -> np.ndarray:
    """Body-rate setpoints (rad/s) to drive measured attitude -> desired.

    The sim is a clean rate controller. Sign convention measured on it: a
    POSITIVE commanded body rate DECREASES the reported angle, so the
    proportional law rate = kp * (measured - desired) drives measured to target.
    Yaw error wraps the short way around +-pi. All angles in radians.
    """
    rr = kp * (meas_rpy[0] - des_rpy[0])
    pr = kp * (meas_rpy[1] - des_rpy[1])
    yr = kp * wrap_to_pi(meas_rpy[2] - des_rpy[2])
    return np.clip(np.array([rr, pr, yr]), -max_rate, max_rate)
