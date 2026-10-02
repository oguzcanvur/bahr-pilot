"""Shared vehicle state: sensor and link threads write it, the MAVLink loop
and navigation read it."""
from __future__ import annotations

import threading

from bahr_pilot.modes import MODE_MANUAL


class VehicleState:
    """`lock` must be held while reading or writing more than one field
    together (e.g. lat+lon) — individual attribute access is already atomic
    under the GIL, but a pair can still be read mid-update otherwise."""

    def __init__(self) -> None:
        self.lock = threading.Lock()

        self.lat: float | None = None
        self.lon: float | None = None
        self.heading_deg: float = 0.0
        self.satellites: int = 0
        self.fix_type: int = 1  # MAVLink GPS_FIX_TYPE, 1 = no fix
        self.alt_m: float = 0.0

        self.depth_m: float | None = None
        self.water_temp_c: float | None = None

        self.battery_voltage: float | None = None

        self.mode: int = MODE_MANUAL
        self.armed: bool = False

        self.home_lat: float | None = None
        self.home_lon: float | None = None

        self.gcs_last_seen: float = 0.0
