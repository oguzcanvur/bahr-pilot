"""GNSS measurements: parsing, quality classification and validity (Phase 5).

Pure functions and small immutable records; no serial ports, no threads, so
all of it is tested with synthetic sentences (tests/test_gnss.py).

Every measurement carries the rules from docs/SENSOR_INTERFACE.md: a time
(`t`, Pi monotonic seconds at arrival), a quality, an explicit validity, and a
source. Nothing downstream may use a position or heading that is not valid.

Field layouts are NovAtel OEM7's (which Bynav's receivers follow): BESTPOSA
and HEADINGA fields after the ';' of the log header, plus standard NMEA GGA /
GSA. They matched the real RTD100's captures when the reader was written
(2026-09-15/16, with empty values — it was indoors), but Bynav's own protocol
document (UG017) was never reachable, so treat the layout as best-effort, not
vendor-confirmed. The synthetic test sentences are built from the same layout:
they prove the code is consistent with it, not that the layout is right.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class GnssQuality(IntEnum):
    NO_FIX = 0
    FIX_2D = 1
    FIX_3D = 2
    DGPS = 3
    FLOAT = 4
    RTK_FIXED = 5


# MAVLink GPS_FIX_TYPE: 1 no fix, 2 2D, 3 3D, 4 DGPS, 5 RTK float, 6 RTK fixed.
_MAV_FIX_TYPE = {
    GnssQuality.NO_FIX: 1, GnssQuality.FIX_2D: 2, GnssQuality.FIX_3D: 3,
    GnssQuality.DGPS: 4, GnssQuality.FLOAT: 5, GnssQuality.RTK_FIXED: 6,
}


def mav_fix_type(quality: GnssQuality) -> int:
    return _MAV_FIX_TYPE[quality]


# Receiver position types that do NOT come from a measured fix.
_NOT_A_FIX = {"NONE", "INSUFFICIENT_OBS", "FIXEDPOS", "FIXEDHEIGHT", "FIXEDVEL",
              "DOPPLER_VELOCITY", "OUT_OF_BOUNDS", "WARNING"}
_DGPS_TYPES = {"PSRDIFF", "WAAS", "SBAS", "OMNISTAR", "PROPAGATED", "INS_SBAS", "INS_PSRDIFF"}


def classify_pos_type(pos_type: str) -> GnssQuality:
    """Receiver position type -> quality class. Unknown names fall back to a
    plain 3D fix: better to under-claim than to call an unrecognised type RTK."""
    name = pos_type.strip().upper()
    if name in _NOT_A_FIX:
        return GnssQuality.NO_FIX
    if name.endswith("_FLOAT") or name == "INS_RTKFLOAT":
        return GnssQuality.FLOAT
    if name.endswith("_INT") or name == "INS_RTKFIXED":
        return GnssQuality.RTK_FIXED
    if name in _DGPS_TYPES or name.startswith("PPP") or name.startswith("INS_PPP"):
        return GnssQuality.DGPS
    return GnssQuality.FIX_3D


# NMEA GGA fix-quality indicator.
_GGA_QUALITY = {
    0: GnssQuality.NO_FIX, 1: GnssQuality.FIX_3D, 2: GnssQuality.DGPS, 3: GnssQuality.FIX_3D,
    4: GnssQuality.RTK_FIXED, 5: GnssQuality.FLOAT, 6: GnssQuality.NO_FIX,  # 6 = dead reckoning
}


@dataclass(frozen=True)
class GnssFix:
    t: float                      # Pi monotonic seconds at arrival
    lat: float
    lon: float
    alt_m: float                  # above mean sea level
    quality: GnssQuality
    satellites: int
    source: str                   # "rtd100" or "echomap"
    pos_type: str = ""
    sol_status: str = ""
    h_acc_m: float | None = None  # 1-sigma horizontal, from the receiver's own estimate
    v_acc_m: float | None = None
    diff_age_s: float | None = None
    hdop: float | None = None
    vdop: float | None = None


@dataclass(frozen=True)
class GnssHeading:
    t: float
    heading_deg: float            # TRUE north, clockwise (dual-antenna)
    acc_deg: float | None         # 1-sigma
    baseline_m: float | None
    quality: GnssQuality
    pos_type: str
    source: str = "rtd100"


# -- validity policy -----------------------------------------------------------

# How old a measurement may be before it stops counting. RTD100 logs 5 Hz
# (BESTPOSA) and 1 Hz (HEADINGA); the echoMAP's NMEA is 1 Hz.
FIX_TIMEOUT_S = {"rtd100": 1.0, "echomap": 2.5}
HEADING_TIMEOUT_S = 5.0
DEPTH_TIMEOUT_S = 5.0

MAX_H_ACC_M = 10.0        # a receiver claiming worse than this is not navigating
MAX_HEADING_ACC_DEG = 10.0


def fix_is_usable(fix: GnssFix | None, now: float) -> bool:
    if fix is None or fix.quality == GnssQuality.NO_FIX:
        return False
    if now - fix.t > FIX_TIMEOUT_S.get(fix.source, 1.0):
        return False
    if fix.h_acc_m is not None and fix.h_acc_m > MAX_H_ACC_M:
        return False
    return -90.0 <= fix.lat <= 90.0 and -180.0 <= fix.lon <= 180.0 and not (fix.lat == 0.0 and fix.lon == 0.0)


def heading_is_usable(heading: GnssHeading | None, now: float) -> bool:
    if heading is None or heading.quality == GnssQuality.NO_FIX:
        return False
    if now - heading.t > HEADING_TIMEOUT_S:
        return False
    if heading.acc_deg is not None and heading.acc_deg > MAX_HEADING_ACC_DEG:
        return False
    return 0.0 <= heading.heading_deg < 360.0


# -- parsing --------------------------------------------------------------------

def _float(raw: str) -> float | None:
    try:
        value = float(raw)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


def _int(raw: str) -> int | None:
    try:
        return int(float(raw))
    except ValueError:
        return None


def _nmea_coord(raw: str, hemisphere: str, degree_width: int) -> float | None:
    if not raw or not hemisphere:
        return None
    try:
        degrees = int(raw[:degree_width])
        minutes = float(raw[degree_width:])
    except ValueError:
        return None
    value = degrees + minutes / 60.0
    return -value if hemisphere in ("S", "W") else value


def parse_bestposa(fields: list[str], t: float, source: str = "rtd100") -> GnssFix | None:
    """#BESTPOSA data fields: sol_status, pos_type, lat, lon, height,
    undulation, datum, lat_sigma, lon_sigma, hgt_sigma, stn_id, diff_age,
    sol_age, #SVs, #solnSVs, ...  Returns None if the log is unusable (no
    solution computed, or a field is missing/garbled) — never a fix with
    guessed numbers."""
    if len(fields) < 4:
        return None
    sol_status, pos_type = fields[0].strip(), fields[1].strip()
    lat, lon = _float(fields[2]), _float(fields[3])
    if sol_status != "SOL_COMPUTED" or lat is None or lon is None:
        return None
    quality = classify_pos_type(pos_type)
    if quality == GnssQuality.NO_FIX:
        return None

    alt = _float(fields[4]) if len(fields) > 4 else None
    lat_sigma = _float(fields[7]) if len(fields) > 7 else None
    lon_sigma = _float(fields[8]) if len(fields) > 8 else None
    hgt_sigma = _float(fields[9]) if len(fields) > 9 else None
    h_acc = math.hypot(lat_sigma, lon_sigma) if lat_sigma is not None and lon_sigma is not None else None
    diff_age = _float(fields[11]) if len(fields) > 11 else None
    satellites = _int(fields[13]) if len(fields) > 13 else None
    return GnssFix(
        t=t, lat=lat, lon=lon, alt_m=alt if alt is not None else 0.0, quality=quality,
        satellites=satellites if satellites is not None else 0, source=source,
        pos_type=pos_type, sol_status=sol_status, h_acc_m=h_acc, v_acc_m=hgt_sigma,
        diff_age_s=diff_age,
    )


def parse_headinga(fields: list[str], t: float) -> GnssHeading | None:
    """#HEADINGA data fields: sol_status, pos_type, baseline length, heading,
    pitch, reserved, heading_sigma, pitch_sigma, stn_id, #SVs, ..."""
    if len(fields) < 4:
        return None
    sol_status, pos_type = fields[0].strip(), fields[1].strip()
    heading = _float(fields[3])
    if sol_status != "SOL_COMPUTED" or heading is None:
        return None
    quality = classify_pos_type(pos_type)
    if quality == GnssQuality.NO_FIX:
        return None
    return GnssHeading(
        t=t, heading_deg=0.0 if heading == 360.0 else heading,
        acc_deg=_float(fields[6]) if len(fields) > 6 else None,
        baseline_m=_float(fields[2]), quality=quality, pos_type=pos_type,
    )


def parse_gga(fields: list[str], t: float, source: str) -> GnssFix | None:
    """NMEA GGA (fields after the sentence id): time, lat, N/S, lon, E/W,
    quality, #sats, HDOP, altitude, M, ..."""
    if len(fields) < 9:
        return None
    quality_code = _int(fields[5])
    quality = _GGA_QUALITY.get(quality_code if quality_code is not None else 0, GnssQuality.NO_FIX)
    lat = _nmea_coord(fields[1], fields[2], 2)
    lon = _nmea_coord(fields[3], fields[4], 3)
    if quality == GnssQuality.NO_FIX or lat is None or lon is None:
        return None
    satellites = _int(fields[6])
    return GnssFix(
        t=t, lat=lat, lon=lon, alt_m=_float(fields[8]) or 0.0, quality=quality,
        satellites=satellites if satellites is not None else 0, source=source,
        pos_type=f"GGA{quality_code}", sol_status="SOL_COMPUTED", hdop=_float(fields[7]),
        diff_age_s=_float(fields[12]) if len(fields) > 12 else None,
    )


@dataclass(frozen=True)
class Dops:
    mode: int                      # 1 no fix, 2 2D, 3 3D
    hdop: float | None
    vdop: float | None


def parse_gsa(fields: list[str]) -> Dops | None:
    """NMEA GSA (fields after the sentence id): mode M/A, fix mode 1/2/3,
    12 satellite ids, PDOP, HDOP, VDOP."""
    if len(fields) < 17:
        return None
    mode = _int(fields[1])
    if mode is None:
        return None
    return Dops(mode=mode, hdop=_float(fields[15]), vdop=_float(fields[16]))
