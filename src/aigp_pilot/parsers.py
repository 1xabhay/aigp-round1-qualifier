"""
Wire-format parsing for the AI-GP sim's custom payloads.

The sim packs two things into MAVLink ENCAPSULATED_DATA messages, discriminated
by the first byte (the "encapsulated message id"):

  id 1  RACE_STATUS  - fixed struct, carries the active gate index + timing.
  id 2  TRACK_INFO   - the gate map, sent CHUNKED across several packets and
                       reassembled into gate-count + per-gate records.

Layouts confirmed from a live capture of the Round-1 track. All little-endian.
Pure functions + one reassembly helper; no sockets here, so it is fully unit
tested with synthetic bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

from aigp_pilot.planner import Gate

RACE_STATUS_ID = 1
TRACK_INFO_ID = 2

_RACE_STATUS_FMT = "<BQqqIq"
_GATE_FMT = "<Hfffffffff"
_GATE_SZ = struct.calcsize(_GATE_FMT)  # 38 bytes


@dataclass
class RaceStatus:
    sim_boot_time_ms: int
    race_start_boot_time_ms: int
    race_finish_time_ns: int
    active_gate_index: int
    last_gate_race_time: int

    @property
    def started(self) -> bool:
        # start/finish are negative (or None) until they happen.
        return self.race_start_boot_time_ms >= 0

    @property
    def finished(self) -> bool:
        return self.race_finish_time_ns >= 0


def encapsulated_msg_id(raw: bytes) -> int:
    """First byte of an ENCAPSULATED_DATA payload tells us what it is."""
    return raw[0]


def parse_race_status(raw: bytes) -> RaceStatus:
    _id, boot, start, finish, active, last = struct.unpack_from(_RACE_STATUS_FMT, raw)
    return RaceStatus(
        sim_boot_time_ms=boot,
        race_start_boot_time_ms=start,
        race_finish_time_ns=finish,
        active_gate_index=active,
        last_gate_race_time=last,
    )


def split_track_chunk(raw: bytes):
    """A track chunk is '<BH' (data_type, transfer_id) + this chunk's bytes.

    Returns (transfer_id, chunk_content).
    """
    _id, transfer_id = struct.unpack_from("<BH", raw)
    return transfer_id, raw[3:]


def parse_gate_map(payload: bytes) -> list[Gate]:
    """Decode a fully-reassembled gate map into Gate objects."""
    (num_gates,) = struct.unpack_from("<H", payload)
    gates: list[Gate] = []
    offset = 2
    for _ in range(num_gates):
        gid, px, py, pz, qw, qx, qy, qz, w, h = struct.unpack_from(_GATE_FMT, payload, offset)
        gates.append(
            Gate(
                gate_id=gid,
                position=np.array([px, py, pz]),
                q_wxyz=np.array([qw, qx, qy, qz]),
                width=w,
                height=h,
            )
        )
        offset += _GATE_SZ
    return gates


class TrackDataReassembler:
    """Collects ENCAPSULATED_DATA track chunks into the full gate-map bytes.

    Usage mirrors the wire protocol:
      - on DATA_TRANSMISSION_HANDSHAKE: register(transfer_id, num_packets)
      - on each track chunk: add(transfer_id, seqnr, chunk_content)
        -> returns the full payload once the last chunk arrives, else None.
    Tolerates out-of-order chunks.
    """

    def __init__(self):
        self._chunks: dict[int, dict[int, bytes]] = {}
        self._expected: dict[int, int] = {}

    def register(self, transfer_id: int, num_packets: int) -> None:
        self._expected[transfer_id] = num_packets
        self._chunks[transfer_id] = {}

    def add(self, transfer_id: int, seqnr: int, chunk_content: bytes):
        if transfer_id not in self._expected:
            return None  # no handshake seen for this transfer; ignore
        self._chunks[transfer_id][seqnr] = chunk_content
        if len(self._chunks[transfer_id]) == self._expected[transfer_id]:
            ordered = self._chunks[transfer_id]
            full = b"".join(ordered[i] for i in range(self._expected[transfer_id]))
            del self._chunks[transfer_id]
            del self._expected[transfer_id]
            return full
        return None
