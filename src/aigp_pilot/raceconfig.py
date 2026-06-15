"""
RaceConfig: every tunable in ONE versioned object, so each run is a named,
committed, reproducible config. Presets below are the configs we've flown /
want to try — add to them, never lose one.

The physics limits (a_lat/a_lon/a_brk, v_max) feed the speed profile (raceline);
the gains feed the cascade. The offline simulator (simulate.py) scores a config
without the real sim, so we can sweep hundreds in parallel and fly only the best.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class RaceConfig:
    name: str = "default"
    # vertical (altitude) loop
    kpz_pos: float = 1.0
    kpz_vel: float = 3.0
    # horizontal tracking
    kph_ct: float = 1.4          # cross-track correction gain (hold the centreline)
    kph_vel: float = 2.2         # velocity error -> accel
    vh_max: float = 13.0
    ah_max: float = 16.0
    # physics speed profile (the F1 / wind-tunnel limits)
    a_lat: float = 6.0           # cornering grip
    a_lon: float = 6.0           # accel out of turns
    a_brk: float = 7.0           # braking into turns
    v_max: float = 9.0           # straight-line top speed
    v_min: float = 2.5
    v_start: float = 1.0
    climb_max: float = 0.0       # MEASURED climb authority (m/s); caps v on steep climbs. 0=off
    # attitude (inner rate loop)
    tilt_deg: float = 35.0
    kp_att: float = 1.3
    max_rate: float = 2.0        # rad/s rate cap (also models how fast we can re-aim)
    # path
    curv_ff: float = 0.0         # curvature feedforward gain (1.0 = full v^2*kappa); pre-turn into bends
    look_ahead: float = 4.0      # tangent/slope lead
    alt_bias: float = 0.25       # fly slightly above the path (rides low otherwise)
    # racing line within the gate "tube": 0 = thread dead centres; >0 = let the
    # line cut apexes up to apex_frac*gate_radius (straighter -> faster, clean).
    gate_radius: float = 1.0     # usable half-aperture (gate is 2.72 m -> 1.36 half, leave margin)
    apex_frac: float = 0.0
    # square crossing: short X-aligned (course-normal) segment THROUGH each gate so
    # the drone crosses straight & level (right angle of attack); turn between gates.
    cross_d: float = 0.0         # half-length of that segment (m); 0 = off

    @property
    def tilt_rad(self) -> float:
        return math.radians(self.tilt_deg)

    def variant(self, name: str, **changes) -> "RaceConfig":
        return replace(self, name=name, **changes)


# --- named presets: every config we've run or want to try (committed) ---
# baseline6 = the proven clean lap (~32 s).  Then push V_MAX up.
PRESETS = {
    "baseline6": RaceConfig(name="baseline6", v_max=6.0),
    "v8":  RaceConfig(name="v8",  v_max=8.0),
    "v9":  RaceConfig(name="v9",  v_max=9.0),
    "v10": RaceConfig(name="v10", v_max=10.0),
    "v12": RaceConfig(name="v12", v_max=12.0),
    # PROFILE configs: high straight speed, but low a_lat brakes into the bends
    # (gates 3-4 zigzag) to the proven-clean ~7-8 m/s. The F1 line.
    "prof_a2": RaceConfig(name="prof_a2", v_max=12.0, a_lat=2.0),
    "prof_a3": RaceConfig(name="prof_a3", v_max=12.0, a_lat=3.0),
    # v10 cleared gate 2 (climb) but grazed the bends; brake the bends, keep 10 straights
    "prof10_a2": RaceConfig(name="prof10_a2", v_max=10.0, a_lat=2.0),
    "prof10_a3": RaceConfig(name="prof10_a3", v_max=10.0, a_lat=3.0),
    # APEX configs: cut the line inside the gate tube to straighten the zigzag.
    # alt_bias 0.6 -> fly the CENTRE of the vertical tube (was hugging the floor).
    "apex10": RaceConfig(name="apex10", v_max=10.0, a_lat=3.0, apex_frac=0.8, alt_bias=0.6),
    "apex12": RaceConfig(name="apex12", v_max=12.0, a_lat=4.0, apex_frac=0.8, alt_bias=0.6),
    # SQUARE crossing (gates are all parallel, normal = X): cross each gate straight
    # & level, turn between. Targets the oblique-crossing grazes at gates 2-4.
    "square10": RaceConfig(name="square10", v_max=10.0, a_lat=4.0, cross_d=3.0, alt_bias=0.6),
    "square12": RaceConfig(name="square12", v_max=12.0, a_lat=5.0, cross_d=3.0, alt_bias=0.6),
    # MEASURED config: every limit from real telemetry (capture + v9/square12 runs).
    # v_top~8-10, a_brk~5, a_lat~6, climb~2.3 (the binding limit on the steep climb).
    "measured": RaceConfig(name="measured", v_max=9.0, a_lat=5.5, a_lon=6.0, a_brk=5.0,
                           climb_max=2.3, alt_bias=0.6),
    # TRACKING: measured + curvature feedforward (pre-turn into bends). Single change
    # vs 'measured' to isolate the FF benefit; then we can raise speed.
    "track1": RaceConfig(name="track1", v_max=9.0, a_lat=5.5, a_lon=6.0, a_brk=5.0,
                         climb_max=2.3, alt_bias=0.6, curv_ff=1.0),
}


def get(name: str) -> RaceConfig:
    if name not in PRESETS:
        raise KeyError(f"unknown config {name!r}; have {sorted(PRESETS)}")
    return PRESETS[name]
