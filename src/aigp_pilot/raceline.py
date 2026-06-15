"""
Race-line geometry + a physics-based time-optimal SPEED PROFILE.

The path is a smooth Catmull-Rom spline through the gate centres. The speed
profile is the classic F1 racing-line construction, derived from the drone's
*physical* motion limits (not hand-tuned rules):

  1. corner limit:  v(s) = sqrt(a_lat / curvature(s))   -- grip in the turns
  2. forward pass:  can't accelerate faster than a_lon   -- power out
  3. backward pass: must brake by a_brk BEFORE the corner -- brake in

So: brake into a turn, slow at the apex, accelerate out -- all from a handful
of measurable limits (a ~ g*tan(tilt); brake/accel from thrust authority).

All functions are pure (no I/O), so they're unit-tested in isolation.
"""
from __future__ import annotations

import math

import numpy as np


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def catmull_rom(ctrl, n: int = 18):
    """Dense polyline of a Catmull-Rom spline that passes THROUGH every control
    point (a smooth curve threading the gate centres)."""
    pts = [np.asarray(p, dtype=float) for p in ctrl]
    P = [pts[0]] + pts + [pts[-1]]          # pad the ends so every segment has neighbours
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for j in range(n):
            t = j / n
            t2, t3 = t * t, t * t * t
            out.append(0.5 * (2 * p1 + (-p0 + p2) * t
                              + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(pts[-1])
    return out


def arc_lengths(pts):
    """Cumulative arc length at each polyline point (len == len(pts), starts 0)."""
    cum = [0.0]
    for i in range(len(pts) - 1):
        cum.append(cum[-1] + float(np.linalg.norm(pts[i + 1] - pts[i])))
    return cum


def project_arc(pts, cum, pos) -> float:
    """Arc length along the path of the closest point to pos."""
    pos = np.asarray(pos, dtype=float)
    best_d, best_s = float("inf"), 0.0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-9 else clamp(float((pos - a) @ ab) / L2, 0.0, 1.0)
        proj = a + t * ab
        d = float(np.linalg.norm(pos - proj))
        if d < best_d:
            best_d, best_s = d, cum[i] + t * math.sqrt(L2)
    return best_s


def point_at_arc(pts, cum, s: float):
    """Point on the path at arc length s (clamped to the path)."""
    s = clamp(s, 0.0, cum[-1])
    for i in range(len(pts) - 1):
        if s <= cum[i + 1] or i == len(pts) - 2:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg < 1e-9 else (s - cum[i]) / seg
            return pts[i] + t * (pts[i + 1] - pts[i])
    return pts[-1]


def smooth_within_corridor(waypoints, radius, frac=1.0, iters=60, alpha=0.5):
    """Straighten a waypoint chain WITHOUT leaving the tube. Laplacian smoothing
    (pull each point toward the midpoint of its neighbours) clamped so no point
    moves more than radius*frac from its original gate centre — i.e. the line may
    cut the apex but still passes through every gate opening. Endpoints fixed.

    This is the 'wind tunnel' line: fastest smooth path inside the gate borders."""
    orig = [np.asarray(p, dtype=float) for p in waypoints]
    pts = [p.copy() for p in orig]
    n = len(pts)
    lim = radius * frac
    for _ in range(iters):
        new = [p.copy() for p in pts]
        for i in range(1, n - 1):
            mid = 0.5 * (pts[i - 1] + pts[i + 1])
            target = pts[i] + alpha * (mid - pts[i])
            d = target - orig[i]                 # keep within the aperture of gate i
            dn = float(np.linalg.norm(d))
            if dn > lim:
                target = orig[i] + d * (lim / dn)
            new[i] = target
        pts = new
    return pts


def menger_curvature(p0, p1, p2) -> float:
    """Curvature at p1 from three points = 4*Area / (|p0p1| |p1p2| |p2p0|).
    0 for collinear points; 1/R for points on a circle of radius R."""
    a = float(np.linalg.norm(p1 - p0))
    b = float(np.linalg.norm(p2 - p1))
    c = float(np.linalg.norm(p2 - p0))
    if a * b * c < 1e-12:
        return 0.0
    area = 0.5 * float(np.linalg.norm(np.cross(p1 - p0, p2 - p0)))
    return 4.0 * area / (a * b * c)


def speed_profile(pts, cum, a_lat: float, a_lon: float, a_brk: float,
                  v_max: float, v_min: float, v_start: float, climb_max: float = 0.0):
    """Time-optimal speed at each path point from the physics limits.
    Returns a list aligned with pts/cum. See module docstring for the method.

    climb_max (m/s, MEASURED): if set, also cap speed so the required climb rate
    v*|slope| never exceeds what the drone can actually climb -> the drone slows
    on steep sections instead of lagging low and scraping the climb gates."""
    n = len(pts)
    v = [v_max] * n
    # 1) cornering limit from local curvature  (+ optional climb-rate limit)
    for i in range(1, n - 1):
        kappa = menger_curvature(pts[i - 1], pts[i], pts[i + 1])
        if kappa > 1e-6:
            v[i] = min(v[i], math.sqrt(a_lat / kappa))
        if climb_max > 0.0:
            dz = abs(float(pts[i + 1][2] - pts[i - 1][2]))
            dh = float(np.linalg.norm((pts[i + 1] - pts[i - 1])[:2]))
            slope = dz / dh if dh > 1e-6 else 0.0
            if slope > 1e-3:
                v[i] = min(v[i], climb_max / slope)
    v[0] = min(v[0], v_start)
    # 2) forward pass: bound acceleration (can't speed up faster than a_lon)
    for i in range(1, n):
        ds = cum[i] - cum[i - 1]
        v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2.0 * a_lon * ds))
    # 3) backward pass: bound braking (must slow down BEFORE the corner)
    for i in range(n - 2, -1, -1):
        ds = cum[i + 1] - cum[i]
        v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2.0 * a_brk * ds))
    return [max(v_min, x) for x in v]


def path_curvature_vector(pts, cum, s: float, ds: float = 1.5):
    """Horizontal curvature vector dt/ds (= kappa * unit-normal-toward-centre) at
    arc length s, by finite-differencing the tangent. Units 1/m. Multiply by v^2
    to get the centripetal acceleration needed to FOLLOW the path at speed v
    (feedforward -> the controller pre-turns into a bend instead of lagging it)."""
    s0 = clamp(s - ds, 0.0, cum[-1])
    s1 = clamp(s + ds, 0.0, cum[-1])
    p_prev = point_at_arc(pts, cum, s0)
    p = point_at_arc(pts, cum, s)
    p_next = point_at_arc(pts, cum, s1)
    t1 = (p - p_prev)[:2]
    t2 = (p_next - p)[:2]
    n1 = float(np.linalg.norm(t1))
    n2 = float(np.linalg.norm(t2))
    if n1 < 1e-6 or n2 < 1e-6:
        return np.zeros(2)
    arc = 0.5 * (n1 + n2)
    dt = t2 / n2 - t1 / n1
    return dt / arc if arc > 1e-6 else np.zeros(2)


def speed_at(cum, vprof, s: float) -> float:
    """Interpolate the speed profile at arc length s."""
    s = clamp(s, 0.0, cum[-1])
    for i in range(len(cum) - 1):
        if s <= cum[i + 1] or i == len(cum) - 2:
            seg = cum[i + 1] - cum[i]
            t = 0.0 if seg < 1e-9 else (s - cum[i]) / seg
            return vprof[i] + t * (vprof[i + 1] - vprof[i])
    return vprof[-1]
