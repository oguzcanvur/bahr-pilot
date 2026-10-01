"""DataLogger writes one valid JSON object per line and includes
battery/IMU fields only when the telemetry says they're valid."""
from __future__ import annotations

import json

from bahr_pilot.datalog import DataLogger
from bahr_pilot.nucleo_link import RcTelemetry
from bahr_pilot.state import VehicleState


def test_log_writes_one_json_line_per_call(tmp_path):
    logger = DataLogger(str(tmp_path))
    state = VehicleState()
    state.lat, state.lon = 41.0, 29.0

    logger.log(state, None)
    logger.log(state, None)
    logger.close()

    lines = logger.path.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    record = json.loads(lines[0])
    assert record["lat"] == 41.0
    assert record["lon"] == 29.0
    assert "roll_deg" not in record  # no telemetry given


def test_log_includes_imu_and_battery_only_when_valid(tmp_path):
    logger = DataLogger(str(tmp_path))
    state = VehicleState()
    telemetry = RcTelemetry()
    telemetry.imu_valid = True
    telemetry.roll_deg = 1.5
    telemetry.pitch_deg = -2.5
    telemetry.battery_valid = False

    logger.log(state, telemetry)
    logger.close()

    record = json.loads(logger.path.read_text(encoding="utf-8").splitlines()[0])
    assert record["roll_deg"] == 1.5
    assert record["pitch_deg"] == -2.5
    assert "battery_mv" not in record
