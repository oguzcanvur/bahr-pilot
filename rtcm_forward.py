"""Forwards GPS_RTCM_DATA fragments from the GCS link to the RTD100's
serial port, reassembling them with bahr_pilot.rtcm.RtcmReassembler.

Absorbs tools/rtcm_relay.py's role. ROADMAP.md section 4 notes that
rtcm_relay.py and echomap_bridge.py must not run at once against the same
GCS UDP port — both dial the same peer and the GCS answers whoever spoke
last, with no reliable winner. Folding RTCM forwarding into this one
process (alongside the GNSS reader in bahr_pilot.sensors) is the real fix:
one MAVLink link, one component.

Opens its own serial.Serial handle to the RTD100 port, separate from the
GNSS reader's read-side handle in bahr_pilot.sensors — two file descriptors to the
same tty, one effectively read-only and one write-only, don't contend with
each other on Linux.
"""
from __future__ import annotations

import serial

from bahr_pilot.rtcm import RtcmReassembler


class RtcmForwarder:
    def __init__(self, gnss_port: str, gnss_baud: int) -> None:
        self._serial = serial.Serial(gnss_port, gnss_baud, timeout=0)
        self._reassembler = RtcmReassembler()

    def handle_fragment(self, flags: int, length: int, data: bytes) -> None:
        payload = self._reassembler.add(flags, length, data)
        if payload:
            self._serial.write(payload)

    def close(self) -> None:
        self._serial.close()
