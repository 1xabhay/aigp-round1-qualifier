"""
Navigator / waypoint sequencer for the AI-GP pilot.

Holds the mission (the ordered gate list) and maps the sim's broadcast
active_gate_index to the current target point. Stateless given the index: the
sim's RACE_STATUS is the authority on progress, so we never keep our own
counter that could drift out of sync.

Target is the gate CENTRE for now. A later refinement (Rung 2+) is to aim at a
point just beyond the gate along its orientation normal so we fly THROUGH it
rather than stop at it -- the orientation quaternion is carried on each Gate
for exactly that.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Gate:
    gate_id: int
    position: np.ndarray   # NED centre, metres
    q_wxyz: np.ndarray     # orientation quaternion (w,x,y,z); for fly-through later
    width: float           # metres (outer boundary ~2.7)
    height: float          # metres


class Mission:
    def __init__(self, gates: list[Gate]):
        self.gates = list(gates)

    @property
    def num_gates(self) -> int:
        return len(self.gates)

    def is_complete(self, active_gate_index: int) -> bool:
        """All gates passed once the active index reaches the gate count."""
        return active_gate_index >= self.num_gates

    def target_for(self, active_gate_index: int):
        """NED centre of the active gate, or None if the mission is complete."""
        if self.is_complete(active_gate_index):
            return None
        return self.gates[active_gate_index].position
