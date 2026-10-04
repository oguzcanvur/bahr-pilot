"""Shared vehicle state: sensor and link threads write it, the MAVLink loop
and navigation read it."""
from __future__ import annotations

import threading
from typing import TYPE_CHECKING

from bahr_pilot.modes import MODE_MANUAL

if TYPE_CHECKING:
    from bahr_pilot.estimator import Pose


class VehicleState:
    """`lock` must be held while reading or writing more than one field
    together (e.g. lat+lon) — individual attribute access is already atomic
    under the GIL, but a pair can still be read mid-update otherwise."""

    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.lat: float | None = None
        self.lon: float | None = None
        self.heading_deg: float = 0.0   # last valid TRUE heading; stale when not heading_valid
        self.heading_valid: bool = False
        self.heading_acc_deg: float | None = None
        self.heading_time: float = 0.0  # Pi monotonic seconds of the heading measurement
        self.satellites: int = 0
        self.fix_type: int = 1  # MAVLink GPS_FIX_TYPE, 1 = no fix
        self.alt_m: float = 0.0
        # Phase 5: what the fix is made of (bahr_pilot/gnss.py). lat/lon are
        # None whenever there is no usable, fresh fix.
        self.gnss_quality: int = 0      # GnssQuality: 0 none .. 5 RTK fixed
        self.gnss_source: str = ""      # "rtd100" / "echomap" / "" (none)
        self.fix_time: float = 0.0      # Pi monotonic seconds of the fix
        self.h_acc_m: float | None = None
        self.v_acc_m: float | None = None
        self.diff_age_s: float | None = None
        self.hdop: float | None = None

        # What navigation and the MAVLink position messages use: the
        # estimator's output (bahr_pilot/estimator.py), NOT the raw fields
        # above, which stay the receiver's own latest values (GPS_RAW_INT,
        # logging). None until the vehicle loop has run once.
        self.pose: Pose | None = None

        self.depth_m: float | None = None
        self.water_temp_c: float | None = None

        self.battery_voltage: float | None = None

        self.mode: int = MODE_MANUAL
        self.armed: bool = False

        self.home_lat: float | None = None
        self.home_lon: float | None = None

        self.gcs_last_seen: float = 0.0   # time.monotonic() of the last GCS message; 0.0 = never
