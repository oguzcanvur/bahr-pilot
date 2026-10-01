"""Timestamped raw-sensor logging (GNSS fix, depth, IMU, battery) to an
NDJSON file — one JSON object per line, easy to append to and easy to
parse later for post-processing/reports. Only active when vehicle.py is
started with --log-dir; nothing imports or touches this module otherwise,
so existing deployments without that flag are unaffected.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from bahr_pilot.nucleo_link import RcTelemetry
from bahr_pilot.state import VehicleState


class DataLogger:
    def __init__(self, log_dir: str) -> None:
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / time.strftime("bahr-%Y%m%dT%H%M%S.ndjson")
        self._file = open(self.path, "a", encoding="utf-8")

    def log(self, state: VehicleState, telemetry: RcTelemetry | None) -> None:
        record = {
            "t": time.time(),
            "lat": state.lat,
            "lon": state.lon,
            "alt_m": state.alt_m,
            "heading_deg": state.heading_deg,
            "fix_type": state.fix_type,
            "satellites": state.satellites,
            "depth_m": state.depth_m,
            "water_temp_c": state.water_temp_c,
        }
        if telemetry is not None and telemetry.imu_valid:
            record["roll_deg"] = telemetry.roll_deg
            record["pitch_deg"] = telemetry.pitch_deg
        if telemetry is not None and telemetry.battery_valid:
            record["battery_mv"] = telemetry.battery_mv
        self._file.write(json.dumps(record) + "\n")
        self._file.flush()  # small/infrequent writes — a boat losing power
                             # mid-survey shouldn't lose buffered log lines

    def close(self) -> None:
        self._file.close()
