"""Unit-level tests against the real Vehicle class — no Nucleo/GNSS/echoMAP
ports given, so nothing here touches hardware or even opens a serial port.
A real (but peer-less) UDP socket is still created for the MAVLink link,
same as sim/fake_vehicle.py and bahr_pilot/vehicle.py always do; sendto()
on a connectionless UDP socket with nobody listening doesn't raise.
"""
from __future__ import annotations

import argparse

import pytest
from pymavlink import mavutil

from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_MANUAL
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
