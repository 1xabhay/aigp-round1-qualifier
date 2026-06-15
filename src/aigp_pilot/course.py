"""
Round-1 course definition + path/profile builders, shared by the live runner
(racecourse_smooth.py) and the offline sweep (so they fly the SAME line).

Gate map is hard-coded NED (deterministic; the live broadcast is lossy).
"""
from __future__ import annotations

import numpy as np

from aigp_pilot import raceline

# Captured gate stats (capture_gates.py): all 6 gates are PARALLEL, size 2.72 m,
# normal along the course X axis -> the through-direction (angle of attack) is the
# same for every gate: cross moving along X. (Gate-map positions are in a different
# frame than flight telemetry, so we keep the flight-frame centres below.)
COURSE_NORMAL = np.array([1.0, 0.0, 0.0])
GATE_SIZE = 2.72

# Round-1 gate centres (NED metres). Course runs -X ~159 m and climbs ~26 m.
GATES = [
    np.array([-23.30, -0.40, -0.05]),
    np.array([-46.89, -2.50, 5.05]),
    np.array([-74.59, 1.20, 13.65]),
    np.array([-111.49, -5.10, 24.55]),
    np.array([-135.49, -0.80, 25.34]),
    np.array([-159.19, -4.40, 25.95]),
]


def build_path(start, cfg=None, gates=GATES):
    """Smooth spline through the gates, extended ~8 m past the last gate.
    If cfg.apex_frac > 0, cut the line inside the gate tube first (apex line)."""
    waypts = [g.copy() for g in gates]
    if cfg is not None and getattr(cfg, "apex_frac", 0.0) > 0.0:
        sm = raceline.smooth_within_corridor(waypts, cfg.gate_radius, cfg.apex_frac)
        # apex in the HORIZONTAL plane only — the vertical (gate) aperture is tight
        # on this climbing course, so keep every gate's altitude dead-on.
        waypts = [np.array([s[0], s[1], g[2]]) for s, g in zip(sm, gates)]

    cross_d = getattr(cfg, "cross_d", 0.0) if cfg is not None else 0.0
    if cross_d > 0.0:
        # Square crossing: a short segment ALONG the course normal (X) through each
        # gate, at the gate's exact Y,Z -> the drone crosses straight & level. The
        # course runs -X, so we approach from +X (pre) and exit to -X (post).
        ctrl = [np.asarray(start, float)]
        for w in waypts:
            ctrl.append(w + COURSE_NORMAL * cross_d)   # approach side (+X)
            ctrl.append(w.copy())                      # gate centre
            ctrl.append(w - COURSE_NORMAL * cross_d)   # exit side (-X)
        ctrl.append(waypts[-1] - COURSE_NORMAL * (cross_d + 8.0))
    else:
        d = waypts[-1] - waypts[-2]
        d = d / float(np.linalg.norm(d))
        ctrl = [np.asarray(start, float)] + waypts + [waypts[-1] + d * 8.0]
    pts = raceline.catmull_rom(ctrl, 18)
    return pts, raceline.arc_lengths(pts)


def build_profile(pts, cum, cfg):
    """Physics-derived time-optimal speed v(s) for this config."""
    return raceline.speed_profile(pts, cum, cfg.a_lat, cfg.a_lon, cfg.a_brk,
                                  cfg.v_max, cfg.v_min, cfg.v_start,
                                  climb_max=getattr(cfg, "climb_max", 0.0))
