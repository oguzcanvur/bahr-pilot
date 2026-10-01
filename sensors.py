"""echoMAP (NMEA depth/temp/GPS) + SweGeo RTD100 (Bynav ASCII, dual-antenna
position/heading) readers.

Ported from tools/echomap_bridge.py — same regexes, CRC and field indices,
hardware-verified against the real units on 2026-09-15/16 (see that file's
docstring and bahr-own-autopilot-hardware memory for the measurement notes
behind the parsing choices). This module is the permanent home for that
logic; tools/echomap_bridge.py remains as a standalone manual-test utility.
"""
from __future__ import annotations

import re
import threading
import time

import serial

from bahr_pilot.state import VehicleState

_SENTENCE = re.compile(rb"\$([A-Z]{2}[A-Z]{3}),([^*\r\n]*)\*([0-9A-Fa-f]{2})")
_BYNAV_LOG = re.compile(rb"#([A-Z0-9_]{3,24}),[^;$#*\r\n]*;([^$#*\r\n]*)\*([0-9A-Fa-f]{8})")

# GGA fix quality -> MAVLink GPS_FIX_TYPE.
_FIX_TYPE = {0: 1, 1: 3, 2: 4, 4: 6, 5: 5, 6: 2}

# Bynav/NovAtel pos_type -> MAVLink GPS_FIX_TYPE. Not an exhaustive list —
# only the values seen/expected from the RTD100.
_BYNAV_FIX_TYPE = {
    "NONE": 1, "INSUFFICIENT_OBS": 1,
    "SINGLE": 3, "PSRDIFF": 4, "WAAS": 4, "SBAS": 4,
    "L1_FLOAT": 5, "NARROW_FLOAT": 5, "WIDE_FLOAT": 5,
    "L1_INT": 6, "NARROW_INT": 6, "WIDE_INT": 6,
}


def _crc32_novatel(data: bytes) -> int:
    """Bynav/NovAtel ASCII log CRC — reflected 0xEDB88320, init 0, no final
    XOR (different from zlib.crc32)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc


def _checksum_ok(body: bytes, checksum: bytes) -> bool:
    value = 0
    for byte in body:
        value ^= byte
    return value == int(checksum, 16)


def _coord(raw: str, hemisphere: str, degree_width: int) -> float | None:
    if not raw or not hemisphere:
        return None
    try:
        degrees = int(raw[:degree_width])
        minutes = float(raw[degree_width:])
    except ValueError:
        return None
    value = degrees + minutes / 60.0
    return -value if hemisphere in ("S", "W") else value


def _float(raw: str) -> float | None:
    try:
        return float(raw)
    except ValueError:
        return None


class GnssState:
    """Merges echoMAP's own (weak, single-antenna) GPS with the RTD100's
    dual-antenna position + heading when available — RTD100 wins whenever
    it has a lock."""

    def __init__(self) -> None:
        self.lat: float | None = None
        self.lon: float | None = None
        self.alt_m = 0.0
        self.fix_quality = 0
        self.satellites = 0
        self.heading_deg = 0.0
        self.depth_m: float | None = None
        self.water_temp_c: float | None = None

        self.rtd100_lat: float | None = None
        self.rtd100_lon: float | None = None
        self.rtd100_pos_type = "NONE"
        self.rtd100_satellites = 0
        self.rtd100_heading_deg: float | None = None

        self._lock = threading.Lock()

    def apply_echomap(self, kind: str, fields: list[str]) -> None:
        with self._lock:
            if kind == "GPGGA" and len(fields) >= 9:
                self.lat = _coord(fields[1], fields[2], 2)
                self.lon = _coord(fields[3], fields[4], 3)
                self.fix_quality = int(fields[5]) if fields[5].isdigit() else 0
                self.satellites = int(fields[6]) if fields[6].isdigit() else 0
                self.alt_m = _float(fields[8]) or 0.0
            elif kind == "HCHDM" and fields and fields[0]:
                heading = _float(fields[0])
                if heading is not None:
                    self.heading_deg = heading
            elif kind == "SDDPT" and fields and fields[0]:
                depth = _float(fields[0])
                if depth is not None:
                    self.depth_m = depth
            elif kind == "SDDBT" and len(fields) >= 3 and fields[2]:
                depth = _float(fields[2])  # field 3: depth in metres
                if depth is not None:
                    self.depth_m = depth
            elif kind == "SDMTW" and fields and fields[0]:
                temp = _float(fields[0])
                if temp is not None:
                    self.water_temp_c = temp

    def apply_rtd100(self, kind: str, fields: list[str]) -> None:
        """Bynav ASCII log fields (after ';', comma-separated).

        #BESTPOSA: sol_status,pos_type,lat,lon,height,... (NovAtel layout)
        #HEADINGA: sol_status,pos_type,length,heading,pitch,...
        """
        with self._lock:
            if kind == "BESTPOSA" and len(fields) >= 4:
                self.rtd100_pos_type = fields[1]
                lat = _float(fields[2])
                lon = _float(fields[3])
                if fields[1] not in ("NONE", "INSUFFICIENT_OBS") and lat and lon:
                    self.rtd100_lat = lat
                    self.rtd100_lon = lon
                else:
                    self.rtd100_lat = self.rtd100_lon = None
                if len(fields) >= 14:
                    try:
                        self.rtd100_satellites = int(float(fields[13]))
                    except ValueError:
                        pass
            elif kind == "HEADINGA" and len(fields) >= 4:
                if fields[1] not in ("NONE",):
                    self.rtd100_heading_deg = _float(fields[3])
                else:
                    self.rtd100_heading_deg = None

    def publish(self, vehicle: VehicleState) -> None:
        """Write the merged best-available values into the shared vehicle
        state (RTD100 preferred over echoMAP's own weaker GPS)."""
        with self._lock:
            if self.rtd100_lat is not None:
                lat, lon = self.rtd100_lat, self.rtd100_lon
                satellites = self.rtd100_satellites
                fix_type = _BYNAV_FIX_TYPE.get(self.rtd100_pos_type, 3)
            else:
                lat, lon = self.lat, self.lon
                satellites = self.satellites
                fix_type = _FIX_TYPE.get(self.fix_quality, 3 if lat is not None else 1)
            heading = (
                self.rtd100_heading_deg if self.rtd100_heading_deg is not None
                else self.heading_deg
            )
            depth_m = self.depth_m
            water_temp_c = self.water_temp_c
            alt_m = self.alt_m

        with vehicle.lock:
            vehicle.lat = lat
            vehicle.lon = lon
            vehicle.heading_deg = heading
            vehicle.satellites = satellites
            vehicle.fix_type = fix_type
            vehicle.depth_m = depth_m
            vehicle.water_temp_c = water_temp_c
            vehicle.alt_m = alt_m


def read_sentences(port: serial.Serial):
    buf = bytearray()
    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, _, buf[:] = buf.partition(b"\n")
            m = _SENTENCE.match(line.strip())
            if m and _checksum_ok(m.group(1) + b"," + m.group(2), m.group(3)):
                yield m.group(1).decode(), m.group(2).decode().split(",")


def read_bynav_logs(port: serial.Serial):
    buf = bytearray()
    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, _, buf[:] = buf.partition(b"\n")
            line = line.strip()
            m = _BYNAV_LOG.match(line)
            if not m:
                continue
            body = line[1:m.end(2)]  # between '#' and '*': name+header+';'+data
            if _crc32_novatel(body) == int(m.group(3), 16):
                yield m.group(1).decode(), m.group(2).decode().split(",")


def run_echomap_reader(serial_port: str, baud: int, gnss: GnssState, vehicle: VehicleState) -> None:
    while True:
        try:
            with serial.Serial(serial_port, baud, timeout=0.2) as port:
                for kind, fields in read_sentences(port):
                    gnss.apply_echomap(kind, fields)
                    gnss.publish(vehicle)
        except (serial.SerialException, OSError) as exc:
            print(f"echoMAP serial error: {exc} - retrying in 2s", flush=True)
            time.sleep(2)


def run_rtd100_reader(serial_port: str, baud: int, gnss: GnssState, vehicle: VehicleState) -> None:
    while True:
        try:
            with serial.Serial(serial_port, baud, timeout=0.2) as port:
                for kind, fields in read_bynav_logs(port):
                    gnss.apply_rtd100(kind, fields)
                    gnss.publish(vehicle)
        except (serial.SerialException, OSError) as exc:
            print(f"RTD100 serial error: {exc} - retrying in 2s", flush=True)
            time.sleep(2)
