"""
Instrumentation: record full-rate telemetry for every run, so the pilot's
decisions rest on MEASURED data, not guesses. Pure data structures + a CSV
writer (no MAVLink here) -> unit-testable.

Pair with analyze.py to extract the drone's real motion limits from a run.
"""
from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, fields


@dataclass
class Sample:
    t: float          # seconds since GO
    x: float          # (wall-clock field `wall` is last, with a default, for sync)
    y: float
    z: float          # NED (down positive)
    vx: float
    vy: float
    vz: float
    roll: float       # rad
    pitch: float
    yaw: float
    thr: float        # collective sent [0..1]
    v_set: float      # speed the profile asked for
    s: float          # arc length along the path
    active: int       # active gate index (from RACE_STATUS)
    wall: float = 0.0  # wall-clock time (time.time()) — syncs telemetry to recorded frames


class RunLogger:
    """Collects Samples + discrete events (collisions, gate passes) for one run."""

    def __init__(self, config_name: str):
        self.config = config_name
        self.samples: list[Sample] = []
        self.collisions: list[tuple[float, int, int]] = []   # (t, id, threat_level)
        self.gate_events: list[tuple[float, int]] = []        # (t, new active index)

    def record(self, sample: Sample) -> None:
        self.samples.append(sample)

    def collision(self, t: float, cid: int, threat: int) -> None:
        self.collisions.append((t, cid, threat))

    def gate_event(self, t: float, active: int) -> None:
        self.gate_events.append((t, active))

    def write_csv(self, path: str) -> None:
        cols = [f.name for f in fields(Sample)]
        with open(path, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=cols)
            w.writeheader()
            for s in self.samples:
                w.writerow(asdict(s))


def load_csv(path: str) -> list[Sample]:
    """Read a run CSV back into Samples (for offline analysis)."""
    out: list[Sample] = []
    names = {f.name: f.type for f in fields(Sample)}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            kw = {}
            for k, v in row.items():
                if k not in names:
                    continue
                # field type may be the type OR its string name (PEP 563 annotations)
                kw[k] = int(float(v)) if names[k] in (int, "int") else float(v)
            out.append(Sample(**kw))
    return out
