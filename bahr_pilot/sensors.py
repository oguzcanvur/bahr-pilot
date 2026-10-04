"""echoMAP (NMEA depth/temp/GPS) + SweGeo RTD100 (Bynav ASCII, dual-antenna
position/heading) readers, and the single place their data is merged.

The wire parsing (regexes, CRC32, framing) was ported from
tools/echomap_bridge.py and hardware-verified against the real units on
2026-09-15/16; the field-level parsing and the validity rules live in
bahr_pilot/gnss.py (Phase 5). tools/echomap_bridge.py remains as a standalone
manual-test utility.

Merge rules (the reason GnssState exists as ONE shared object):
  * Position: the RTD100's fix when it is usable and fresh, else the
    echoMAP's own (single-antenna, metres-grade) GPS, else nothing. Never an
    old value presented as current.
  * Heading: ONLY the RTD100's dual-antenna heading, which is relative to true
    north. The echoMAP's HCHDM is a magnetic heading; without a declination
    correction it is several degrees off in Turkey, and mixing frames silently
    is exactly what Phase 5 forbids. It is kept (magnetic_heading_deg) but not
    published as the vehicle heading.
  * Depth / water temperature expire too (5 s).
"""
from __future__ import annotations

import collections
import dataclasses
import math
import re
import threading
import time

import serial

from bahr_pilot.gnss import (
    DEPTH_TIMEOUT_S, Dops, GnssFix, GnssHeading, GnssQuality, fix_is_usable, heading_is_usable,
    mav_fix_type, parse_bestposa, parse_gga, parse_gsa, parse_headinga,
)
from bahr_pilot.state import VehicleState

_SENTENCE = re.compile(rb"\$([A-Z]{2}[A-Z]{3}),([^*\r\n]*)\*([0-9A-Fa-f]{2})")
_BYNAV_LOG = re.compile(rb"#([A-Z0-9_]{3,24}),[^;$#*\r\n]*;([^$#*\r\n]*)\*([0-9A-Fa-f]{8})")

# The echoMAP reports each sounding twice (SDDPT and SDDBT): a reading this soon after the
# previous one is the duplicate. (A sounder faster than ~3 Hz would lose readings here.)
MIN_DEPTH_INTERVAL_S = 0.3

# How long a DOP / GSA mode reading stays attached to a position.
_DOP_TIMEOUT_S = 3.0


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


def _float(raw: str) -> float | None:
    try:
        return float(raw)
    except ValueError:
        return None


class GnssState:
    """Latest measurements from the RTD100 and the echoMAP, each stamped with
    its arrival time, and the rules for turning them into one position and
    one heading. One instance is shared by both reader threads."""

    def __init__(self, clock=time.monotonic) -> None:
        self._clock = clock
        self._lock = threading.Lock()

        self._rtd_fix: GnssFix | None = None
        self._rtd_heading: GnssHeading | None = None
        self._rtd_dops: tuple[float, Dops] | None = None   # (arrival time, dops)
        self._rtd_hdop: tuple[float, float] | None = None  # from GGA: (arrival time, hdop)
        self._echo_fix: GnssFix | None = None

        self.magnetic_heading_deg: float | None = None  # echoMAP HCHDM; NOT a vehicle heading
        self._depth: tuple[float, float] | None = None  # (arrival time, metres)
        # EVERY depth reading, for the sonar filter and the survey log: self._depth only keeps
        # the newest. The echoMAP sends SDDPT and SDDBT for the same sounding; anything arriving
        # within MIN_DEPTH_INTERVAL_S of the previous reading is that duplicate, not a new sounding.
        self._depth_queue: collections.deque[tuple[float, float]] = collections.deque(maxlen=64)
        self._last_queued_t = -math.inf
        self._water_temp: tuple[float, float] | None = None

        self.source = ""  # which source the last publish() used ("" = none)

    # -- input ---------------------------------------------------------------

    def apply_rtd100(self, kind: str, fields: list[str], now: float | None = None) -> None:
        """One log/sentence from the RTD100's port: #BESTPOSA, #HEADINGA, or
        NMEA GGA/GSA (it emits both formats at once)."""
        t = self._clock() if now is None else now
        with self._lock:
            if kind == "BESTPOSA":
                self._rtd_fix = parse_bestposa(fields, t, "rtd100")
            elif kind == "HEADINGA":
                self._rtd_heading = parse_headinga(fields, t)
            elif kind.endswith("GGA") and len(fields) >= 8:
                hdop = _float(fields[7])
                if hdop is not None:
                    self._rtd_hdop = (t, hdop)
            elif kind.endswith("GSA"):
                dops = parse_gsa(fields)
                if dops is not None:
                    self._rtd_dops = (t, dops)

    def apply_echomap(self, kind: str, fields: list[str], now: float | None = None) -> None:
        t = self._clock() if now is None else now
        with self._lock:
            if kind == "GPGGA":
                self._echo_fix = parse_gga(fields, t, "echomap")
            elif kind == "HCHDM" and fields and fields[0]:
                heading = _float(fields[0])
                if heading is not None:
                    self.magnetic_heading_deg = heading
            elif kind == "SDDPT" and fields and fields[0]:
                depth = _float(fields[0])
                if depth is not None:
                    self._depth = (t, depth)
                    self._queue_depth(t, depth)
            elif kind == "SDDBT" and len(fields) >= 3 and fields[2]:
                depth = _float(fields[2])  # field 3: depth in metres
                if depth is not None:
                    self._depth = (t, depth)
                    self._queue_depth(t, depth)
            elif kind == "SDMTW" and fields and fields[0]:
                temp = _float(fields[0])
                if temp is not None:
                    self._water_temp = (t, temp)

    def _queue_depth(self, t: float, depth: float) -> None:
        # called with self._lock held
        if t - self._last_queued_t < MIN_DEPTH_INTERVAL_S:
            return
        self._last_queued_t = t
        self._depth_queue.append((t, depth))

    def take_depth_readings(self) -> list[tuple[float, float]]:
        """Every (arrival time, metres) since the last call, oldest first."""
        with self._lock:
            readings = list(self._depth_queue)
            self._depth_queue.clear()
        return readings

    # -- output --------------------------------------------------------------

    def _best_fix_locked(self, now: float) -> GnssFix | None:
        fix = self._rtd_fix
        if fix_is_usable(fix, now):
            if self._rtd_hdop is not None and now - self._rtd_hdop[0] <= _DOP_TIMEOUT_S:
                fix = dataclasses.replace(fix, hdop=self._rtd_hdop[1])
            if self._rtd_dops is not None and now - self._rtd_dops[0] <= _DOP_TIMEOUT_S:
                dops = self._rtd_dops[1]
                fix = dataclasses.replace(
                    fix, hdop=dops.hdop if dops.hdop is not None else fix.hdop, vdop=dops.vdop)
                if dops.mode == 2 and fix.quality == GnssQuality.FIX_3D:
                    fix = dataclasses.replace(fix, quality=GnssQuality.FIX_2D)
            return fix
        if fix_is_usable(self._echo_fix, now):
            return self._echo_fix
        return None

    def best_fix(self, now: float | None = None) -> GnssFix | None:
        t = self._clock() if now is None else now
        with self._lock:
            return self._best_fix_locked(t)

    def best_heading(self, now: float | None = None) -> GnssHeading | None:
        t = self._clock() if now is None else now
        with self._lock:
            return self._rtd_heading if heading_is_usable(self._rtd_heading, t) else None

    def publish(self, vehicle: VehicleState, now: float | None = None) -> None:
        """Write the current snapshot into the shared vehicle state. Call it
        both when data arrives and on a timer: staleness has to be applied
        even when nothing arrives, which is when it matters most."""
        t = self._clock() if now is None else now
        with self._lock:
            fix = self._best_fix_locked(t)
            heading = self._rtd_heading if heading_is_usable(self._rtd_heading, t) else None
            depth = self._depth[1] if self._depth and t - self._depth[0] <= DEPTH_TIMEOUT_S else None
            water_temp = (self._water_temp[1]
                          if self._water_temp and t - self._water_temp[0] <= DEPTH_TIMEOUT_S else None)
            self.source = fix.source if fix is not None else ""

        with vehicle.lock:
            if fix is not None:
                vehicle.lat, vehicle.lon, vehicle.alt_m = fix.lat, fix.lon, fix.alt_m
                vehicle.satellites = fix.satellites
                vehicle.fix_type = mav_fix_type(fix.quality)
                vehicle.gnss_quality = int(fix.quality)
                vehicle.gnss_source = fix.source
                vehicle.fix_time = fix.t
                vehicle.h_acc_m, vehicle.v_acc_m = fix.h_acc_m, fix.v_acc_m
                vehicle.diff_age_s, vehicle.hdop = fix.diff_age_s, fix.hdop
            else:
                vehicle.lat = vehicle.lon = None
                vehicle.satellites = 0
                vehicle.fix_type = 1
                vehicle.gnss_quality = int(GnssQuality.NO_FIX)
                vehicle.gnss_source = ""
                vehicle.h_acc_m = vehicle.v_acc_m = vehicle.diff_age_s = vehicle.hdop = None
            if heading is not None:
                vehicle.heading_deg = heading.heading_deg
                vehicle.heading_valid = True
                vehicle.heading_acc_deg = heading.acc_deg
                vehicle.heading_time = heading.t
            else:
                # heading_deg keeps its last value for display only; consumers
                # must look at heading_valid before steering by it
                vehicle.heading_valid = False
                vehicle.heading_acc_deg = None
            vehicle.depth_m = depth
            vehicle.water_temp_c = water_temp


def _lines(port: serial.Serial):
    buf = bytearray()
    while True:
        chunk = port.read(256)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, _, buf[:] = buf.partition(b"\n")
            yield line.strip()


def read_sentences(port: serial.Serial):
    for line in _lines(port):
        m = _SENTENCE.match(line)
        if m and _checksum_ok(m.group(1) + b"," + m.group(2), m.group(3)):
            yield m.group(1).decode(), m.group(2).decode().split(",")


def read_bynav_logs(port: serial.Serial):
    for line in _lines(port):
        m = _BYNAV_LOG.match(line)
        if not m:
            continue
        body = line[1:m.end(2)]  # between '#' and '*': name+header+';'+data
        if _crc32_novatel(body) == int(m.group(3), 16):
            yield m.group(1).decode(), m.group(2).decode().split(",")


def read_rtd100(port: serial.Serial):
    """The RTD100 sends its own Bynav ASCII logs AND standard NMEA on the same
    port; yield both, each checksum/CRC verified."""
    for line in _lines(port):
        m = _BYNAV_LOG.match(line)
        if m:
            body = line[1:m.end(2)]
            if _crc32_novatel(body) == int(m.group(3), 16):
                yield m.group(1).decode(), m.group(2).decode().split(",")
            continue
        m = _SENTENCE.match(line)
        if m and _checksum_ok(m.group(1) + b"," + m.group(2), m.group(3)):
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
                for kind, fields in read_rtd100(port):
                    gnss.apply_rtd100(kind, fields)
                    gnss.publish(vehicle)
        except (serial.SerialException, OSError) as exc:
            print(f"RTD100 serial error: {exc} - retrying in 2s", flush=True)
            time.sleep(2)
