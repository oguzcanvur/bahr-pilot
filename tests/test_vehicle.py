"""Unit-level tests against the real Vehicle class — no Nucleo/GNSS/echoMAP
ports given, so nothing here touches hardware or even opens a serial port.
A real (but peer-less) UDP socket is still created for the MAVLink link,
same as sim/fake_vehicle.py and bahr_pilot/vehicle.py always do; sendto()
on a connectionless UDP socket with nobody listening doesn't raise.
"""
from __future__ import annotations

import argparse
import time

import pytest
from pymavlink import mavutil

from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_MANUAL
from bahr_pilot.nucleo_link import PULSE_NEUTRAL_US
from bahr_pilot.vehicle import DEFAULT_PARAMS, Vehicle


def make_vehicle() -> Vehicle:
    args = argparse.Namespace(
        gcs_host="127.0.0.1", gcs_port=14599,
        nucleo_port=None, nucleo_baud=115200,
        gnss_port=None, gnss_baud=115200,
        echomap_port=None, echomap_baud=38400,
        log_dir=None,
    )
    return Vehicle(args)


@pytest.fixture
def vehicle():
    return make_vehicle()


def test_default_params_include_new_battery_and_rc_params():
    assert "BATT_LOW_VOLT" in DEFAULT_PARAMS
    assert "BATT_FS_ENABLE" in DEFAULT_PARAMS
    assert DEFAULT_PARAMS["BATT_FS_ENABLE"] == 0.0  # off by default, see vehicle.py's comment
    for ch in range(1, 9):
        assert f"RC{ch}_MIN" in DEFAULT_PARAMS
        assert f"RC{ch}_REVERSED" in DEFAULT_PARAMS


def test_param_set_accepts_known_rejects_unknown(vehicle):
    class FakeParamSet:
        param_id = "BATT_LOW_VOLT"
        param_value = 11.4

    vehicle._on_param_set(FakeParamSet())
    assert vehicle.params["BATT_LOW_VOLT"] == pytest.approx(11.4)

    class FakeUnknown:
        param_id = "NOT_A_REAL_PARAM"
        param_value = 1.0

    vehicle._on_param_set(FakeUnknown())
    assert "NOT_A_REAL_PARAM" not in vehicle.params


def test_battery_failsafe_inactive_without_nucleo(vehicle):
    vehicle.params["BATT_FS_ENABLE"] = 1.0
    assert vehicle.nucleo is None
    assert vehicle._battery_failsafe_active() is False


def test_battery_status_unknown_without_nucleo(vehicle):
    assert vehicle._battery_status() == (65535, -1)


def test_manual_arm_does_not_require_gps_fix(vehicle):
    m = mavutil.mavlink
    vehicle.state.mode = MODE_MANUAL
    vehicle.state.fix_type = 1  # no fix
    vehicle._handle_command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0)
    assert vehicle.state.armed is True


def test_guided_arm_still_requires_gps_fix(vehicle):
    m = mavutil.mavlink
    vehicle.state.mode = MODE_GUIDED
    vehicle.state.fix_type = 1  # no fix
    vehicle._handle_command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0)
    assert vehicle.state.armed is False

    vehicle.state.fix_type = 3  # 3D fix
    vehicle._handle_command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0)
    assert vehicle.state.armed is True


def test_auto_arm_denied_without_mission(vehicle):
    m = mavutil.mavlink
    vehicle.state.mode = MODE_AUTO
    vehicle.state.fix_type = 3
    vehicle._handle_command(m.MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0)
    assert vehicle.state.armed is False


# -- BAHR-GCS compatibility: these mirror exactly what gcs/mavlink_worker.py
# sends, so a regression here means the real GCS stops working against us.

def test_do_change_speed_is_metres_per_second_not_percent(vehicle):
    """BAHR-GCS: set_speed()/upload_mission() send param1=1 (ground speed),
    param2=speed in m/s. An earlier version read param2 as a percentage and
    clamped 1.5 m/s down to 5% throttle."""
    m = mavutil.mavlink
    vehicle.params["CRUISE_SPEED"] = 1.5
    vehicle.params["CRUISE_THROTTLE"] = 60.0
    assert vehicle._cruise_fraction() == pytest.approx(0.60)  # default = CRUISE_SPEED

    vehicle._handle_command(m.MAV_CMD_DO_CHANGE_SPEED, 1.0, 0.75, -1.0, 0, 0, 0)
    assert vehicle.target_speed_mps == pytest.approx(0.75)
    assert vehicle._cruise_fraction() == pytest.approx(0.30)

    vehicle._handle_command(m.MAV_CMD_DO_CHANGE_SPEED, 1.0, -1.0, -1.0, 0, 0, 0)
    assert vehicle.target_speed_mps == pytest.approx(0.75)  # -1 = no change

    vehicle._handle_command(m.MAV_CMD_DO_CHANGE_SPEED, 1.0, 10.0, -1.0, 0, 0, 0)
    assert vehicle._cruise_fraction() == pytest.approx(1.0)  # saturates, never >100%


def test_wp_radius_answers_gcs_read_by_name(vehicle, monkeypatch):
    """BAHR-GCS requests WP_RADIUS by name (index -1) on connect to size its
    turn arcs; it must get an answer."""
    sent = []
    monkeypatch.setattr(vehicle, "_send_param", lambda name, index: sent.append(name))

    class ReadByName:
        param_id = b"WP_RADIUS"
        param_index = -1

    vehicle._on_param_request_read(ReadByName())
    assert sent == ["WP_RADIUS"]


def test_gcs_known_param_names_replace_custom_ones():
    for name in ("WP_RADIUS", "CRUISE_SPEED", "CRUISE_THROTTLE", "FS_TIMEOUT", "FS_GCS_ENABLE"):
        assert name in DEFAULT_PARAMS
    for old in ("WP_RADIUS_M", "CRUISE_FRAC", "GCS_FS_TIMEOUT_S"):
        assert old not in DEFAULT_PARAMS


def test_unimplemented_reboot_is_not_reported_as_done(vehicle, monkeypatch):
    acks = []
    monkeypatch.setattr(vehicle, "_ack", lambda command, result: acks.append(result))
    vehicle._handle_command(mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1, 0, 0, 0, 0, 0)
    assert acks == [mavutil.mavlink.MAV_RESULT_UNSUPPORTED]


def test_entering_auto_by_mode_button_starts_at_item_1(vehicle):
    vehicle.navigator.mission = [(0.0, 0.0), (41.0, 29.0), (41.001, 29.0)]
    vehicle.navigator.mission_seq = 0
    vehicle.set_mode(MODE_AUTO)
    assert vehicle.navigator.mission_seq == 1
    assert vehicle.navigator.active_target(MODE_AUTO, None) == (41.0, 29.0)


def _guided_vehicle_heading_to_target(vehicle):
    vehicle.state.lat, vehicle.state.lon = 41.0, 29.0
    vehicle.state.heading_deg = 0.0
    vehicle.state.fix_type = 3
    vehicle.state.armed = True
    vehicle.state.mode = MODE_GUIDED
    vehicle.navigator.guided_target = (41.001, 29.0)  # ~111 m due north


def test_gcs_loss_stops_motors_when_enabled(vehicle):
    _guided_vehicle_heading_to_target(vehicle)
    vehicle.params["FS_GCS_ENABLE"] = 1.0
    vehicle.state.gcs_last_seen = time.time() - 10.0
    assert vehicle._motor_command() == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)


def test_gcs_loss_ignored_when_disabled(vehicle):
    _guided_vehicle_heading_to_target(vehicle)
    vehicle.params["FS_GCS_ENABLE"] = 0.0
    vehicle.state.gcs_last_seen = time.time() - 10.0
    m1, m2 = vehicle._motor_command()
    assert m1 > PULSE_NEUTRAL_US and m2 > PULSE_NEUTRAL_US  # driving forward
