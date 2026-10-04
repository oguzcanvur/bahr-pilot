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

from bahr_pilot.failsafe import Source as Source_
from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_HOLD, MODE_MANUAL, MODE_RTL
from bahr_pilot.nucleo_link import PULSE_NEUTRAL_US, RcTelemetry
from bahr_pilot.vehicle import DEFAULT_PARAMS, Vehicle
from tests.poses import make_pose


def make_vehicle(**overrides) -> Vehicle:
    fields = dict(
        gcs_host="127.0.0.1", gcs_port=14599,
        nucleo_port=None, nucleo_baud=115200,
        gnss_port=None, gnss_baud=115200,
        echomap_port=None, echomap_baud=38400,
        log_dir=None,
    )
    fields.update(overrides)
    return Vehicle(argparse.Namespace(**fields))


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
    vehicle.state.armed = True
    vehicle._update_failsafes(100.0)
    vehicle._update_failsafes(200.0)
    assert vehicle.failsafe.active == {}
    assert vehicle.nucleo is None


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


def test_gcs_joystick_right_speeds_up_the_left_motor(vehicle):
    """RC_CHANNELS_OVERRIDE stick right (chan1 > 1500) must turn the boat
    right: motor 1 (left) faster than motor 2 (right)."""
    vehicle.state.armed = True
    vehicle.state.mode = MODE_MANUAL
    vehicle.state.gcs_last_seen = time.monotonic()

    class Override:
        chan1_raw = 1600  # steering right (+0.25)
        chan3_raw = 1800  # throttle forward (+0.75)

    vehicle._on_rc_channels_override(Override())
    m1, m2 = vehicle._motor_command()
    assert m1 > m2 > PULSE_NEUTRAL_US


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
    vehicle.state.pose = make_pose(41.0, 29.0, heading_deg=0.0, speed_mps=1.5)   # already at cruise speed
    vehicle.state.fix_type = 3
    vehicle.state.armed = True
    vehicle.state.mode = MODE_GUIDED
    vehicle.navigator.guided_target = (41.001, 29.0)  # ~111 m due north


def test_gcs_loss_stops_motors_when_enabled(vehicle):
    _guided_vehicle_heading_to_target(vehicle)
    vehicle.params["FS_GCS_ENABLE"] = 1.0
    vehicle.state.gcs_last_seen = 50.0
    vehicle._update_failsafes(60.0)                       # 10 s of silence
    assert vehicle._motor_command() == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)
    assert vehicle.state.mode == MODE_HOLD


def test_gcs_loss_ignored_when_disabled(vehicle):
    _guided_vehicle_heading_to_target(vehicle)
    vehicle.params["FS_GCS_ENABLE"] = 0.0
    vehicle.state.gcs_last_seen = 50.0
    vehicle._update_failsafes(60.0)                       # 10 s of silence
    m1, m2 = vehicle._motor_command()
    assert m1 > PULSE_NEUTRAL_US and m2 > PULSE_NEUTRAL_US  # driving forward


# -- RC mode switch (ArduPilot MODE_CH / MODE1..6 scheme) -------------------

class FakeNucleo:
    """Stands in for NucleoLink: records what the vehicle sends, and lets a
    test play the part of the STM's telemetry."""

    def __init__(self):
        self.telemetry = RcTelemetry()
        self.imu = None  # like NucleoLink: no IMU frame seen yet
        self.imu_frames = 0
        self.clock = SimpleNamespace(resets=0)              # StmClock.resets: how often the STM restarted
        self.rc_maps = []
        self.motors = []

    def send_rc_map(self, **config):
        self.rc_maps.append(config)

    def send_motors(self, motor1_us, motor2_us):
        self.motors.append((motor1_us, motor2_us))

    def rc(self, slot, *, override=False, link=True, age=0.0):
        t = self.telemetry
        t.mode_slot = slot
        t.override_active = override
        t.rc_link_up = link
        t.last_update = time.monotonic() - age


@pytest.fixture
def rc_vehicle(monkeypatch):
    vehicle = make_vehicle()
    vehicle.nucleo = FakeNucleo()
    vehicle.acks = []
    vehicle.texts = []
    monkeypatch.setattr(vehicle, "_ack", lambda command, result: vehicle.acks.append(result))
    monkeypatch.setattr(vehicle, "_statustext",
                        lambda text, severity=6: vehicle.texts.append(text))
    return vehicle


def set_mode_from_gcs(vehicle, mode):
    vehicle._handle_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, mode, 0, 0, 0, 0)
    return vehicle.acks[-1]


ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED
DENIED = mavutil.mavlink.MAV_RESULT_DENIED


def test_mode_switch_param_defaults_suit_a_three_position_switch():
    assert DEFAULT_PARAMS["MODE_CH"] == 5.0
    assert [DEFAULT_PARAMS[f"MODE{n}"] for n in range(1, 7)] == [
        MODE_MANUAL, MODE_MANUAL, MODE_MANUAL, MODE_HOLD, MODE_HOLD, MODE_AUTO]
    assert "RCMAP_OVERRIDE" not in DEFAULT_PARAMS  # replaced by MODE_CH


def test_rc_map_carries_mode_channel_and_manual_mask(rc_vehicle):
    rc_vehicle._push_rc_map()
    config = rc_vehicle.nucleo.rc_maps[-1]
    assert config["mode_channel"] == 4  # MODE_CH 5, sent 0-based
    assert config["manual_slot_mask"] == 0b000111  # MODE1..3 are MANUAL
    assert (config["mode_min"], config["mode_max"]) == (172, 1811)


def test_changing_a_mode_param_resends_the_map(rc_vehicle):
    class SetParam:
        param_id = "MODE2"
        param_value = float(MODE_HOLD)

    sent_before = len(rc_vehicle.nucleo.rc_maps)
    rc_vehicle._on_param_set(SetParam())
    assert len(rc_vehicle.nucleo.rc_maps) == sent_before + 1
    assert rc_vehicle.nucleo.rc_maps[-1]["manual_slot_mask"] == 0b000101


def test_switch_edge_selects_the_mode_that_band_names(rc_vehicle):
    n = rc_vehicle.nucleo
    n.rc(6)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_AUTO
    n.rc(4)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_HOLD
    n.rc(1, override=True)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_MANUAL


def test_first_telemetry_applies_the_current_switch_position(rc_vehicle):
    assert rc_vehicle.state.mode == MODE_MANUAL  # boot default
    rc_vehicle.nucleo.rc(4)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_HOLD


def test_gcs_can_change_mode_between_switch_edges(rc_vehicle):
    n = rc_vehicle.nucleo
    n.rc(4)
    rc_vehicle._apply_rc_mode_switch()
    assert set_mode_from_gcs(rc_vehicle, MODE_GUIDED) == ACCEPTED
    for _ in range(3):  # same band again and again: no edge, no revert
        rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_GUIDED


def test_gcs_cannot_take_over_while_the_switch_is_in_manual(rc_vehicle):
    m = mavutil.mavlink
    rc_vehicle.nucleo.rc(1, override=True)
    rc_vehicle._apply_rc_mode_switch()
    rc_vehicle.state.fix_type = 3
    rc_vehicle.navigator.mission = [(0.0, 0.0), (41.0, 29.0)]

    assert set_mode_from_gcs(rc_vehicle, MODE_AUTO) == DENIED
    rc_vehicle._handle_command(m.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0)
    assert rc_vehicle.acks[-1] == DENIED
    rc_vehicle._handle_command(m.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0)
    assert rc_vehicle.acks[-1] == DENIED
    rc_vehicle._handle_command(m.MAV_CMD_DO_REPOSITION, -1, 1, 0, 0, 410000000, 290000000)
    assert rc_vehicle.acks[-1] == DENIED
    assert rc_vehicle.state.mode == MODE_MANUAL
    assert rc_vehicle.navigator.guided_target is None

    assert set_mode_from_gcs(rc_vehicle, MODE_MANUAL) == ACCEPTED  # no-op is fine


def test_gcs_regains_control_once_the_switch_leaves_manual(rc_vehicle):
    n = rc_vehicle.nucleo
    n.rc(1, override=True)
    rc_vehicle._apply_rc_mode_switch()
    assert set_mode_from_gcs(rc_vehicle, MODE_AUTO) == DENIED

    n.rc(4)  # switch to the HOLD band
    rc_vehicle._apply_rc_mode_switch()
    assert set_mode_from_gcs(rc_vehicle, MODE_GUIDED) == ACCEPTED


def test_hardware_manual_overrides_a_stale_software_mode(rc_vehicle):
    """The STM says the sticks are driving (e.g. MODE params were edited
    while the switch sat in a band that just became MANUAL): the reported
    mode must follow the hardware, not keep claiming AUTO."""
    n = rc_vehicle.nucleo
    n.rc(4)
    rc_vehicle._apply_rc_mode_switch()
    assert set_mode_from_gcs(rc_vehicle, MODE_GUIDED) == ACCEPTED
    n.rc(4, override=True)  # same band, but the STM now drives manually
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_MANUAL


def test_stale_telemetry_is_ignored(rc_vehicle):
    rc_vehicle.nucleo.rc(6, age=5.0)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_MANUAL
    assert rc_vehicle._rc_in_manual() is False


def test_rc_signal_loss_keeps_the_mode_and_reapplies_on_return(rc_vehicle):
    n = rc_vehicle.nucleo
    n.rc(6)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_AUTO

    n.rc(0, link=False)  # transmitter off / out of range
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_AUTO  # the STM stops the motors; mode is left alone

    n.rc(4)  # signal back, switch now in another band
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_HOLD


def test_unsupported_mode_in_a_band_is_reported_not_applied(rc_vehicle):
    rc_vehicle.params["MODE5"] = 3.0  # STEERING: BAHR-GCS lists it, this vehicle has no such mode
    rc_vehicle.nucleo.rc(5)
    rc_vehicle._apply_rc_mode_switch()
    assert rc_vehicle.state.mode == MODE_MANUAL
    assert any("not supported" in text for text in rc_vehicle.texts)


def test_no_nucleo_means_no_mode_switch(vehicle):
    vehicle._apply_rc_mode_switch()  # must not raise
    assert vehicle.state.mode == MODE_MANUAL
    assert vehicle._rc_in_manual() is False


# -- arm state reporting (Nucleo arming.h -> BAHR-GCS STATUSTEXT) -------------

from bahr_pilot.nucleo_link import (  # noqa: E402
    ARM_BLOCK_BATTERY, ARM_BLOCK_BOOT_GRACE, ARM_BLOCK_IMU, ARM_BLOCK_NO_RC, ARM_BLOCK_NONE,
    ARM_BLOCK_SWITCH_ON, ARM_BLOCK_THROTTLE, ARM_STATE_ARMED, ARM_STATE_BOOT, ARM_STATE_DISARMED,
    ARM_STATE_READY,
)


def arm_telemetry(vehicle, state, block):
    vehicle.nucleo.rc(4)
    vehicle.nucleo.telemetry.arm_state = state
    vehicle.nucleo.telemetry.arm_block = block
    vehicle._report_rc_arming()


def test_refused_arming_is_explained_once_per_reason(rc_vehicle):
    arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_THROTTLE)
    assert rc_vehicle.texts == ["PreArm: throttle stick not centred"]
    for _ in range(5):  # same reason every tick: no spam
        arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_THROTTLE)
    assert len(rc_vehicle.texts) == 1

    arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_SWITCH_ON)
    assert "arm switch is on" in rc_vehicle.texts[-1]
    arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_IMU)
    arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_BATTERY)
    assert [t for t in rc_vehicle.texts if t.startswith("PreArm")].__len__() == 4


def test_power_up_noise_is_not_reported(rc_vehicle):
    arm_telemetry(rc_vehicle, ARM_STATE_BOOT, ARM_BLOCK_NO_RC)
    arm_telemetry(rc_vehicle, ARM_STATE_BOOT, ARM_BLOCK_BOOT_GRACE)
    arm_telemetry(rc_vehicle, ARM_STATE_READY, ARM_BLOCK_NONE)
    assert rc_vehicle.texts == []


def test_arm_and_disarm_are_announced(rc_vehicle):
    arm_telemetry(rc_vehicle, ARM_STATE_READY, ARM_BLOCK_NONE)
    arm_telemetry(rc_vehicle, ARM_STATE_ARMED, ARM_BLOCK_NONE)
    assert rc_vehicle.texts == ["Armed (RC switch)"]
    arm_telemetry(rc_vehicle, ARM_STATE_DISARMED, ARM_BLOCK_NONE)
    assert rc_vehicle.texts == ["Armed (RC switch)", "Disarmed (RC)"]


def test_stale_telemetry_reports_nothing(rc_vehicle):
    rc_vehicle.nucleo.rc(4, age=5.0)
    rc_vehicle.nucleo.telemetry.arm_block = ARM_BLOCK_THROTTLE
    rc_vehicle._report_rc_arming()
    assert rc_vehicle.texts == []


def test_version_banner_fits_statustext():
    from bahr_pilot.versions import BAHR_LINK_VERSION, MISSION_FORMAT_VERSION, banner

    assert len(banner()) <= 50
    assert f"link{BAHR_LINK_VERSION}" in banner()
    assert f"msn{MISSION_FORMAT_VERSION}" in banner()


# -- IMU -> ATTITUDE (Phase 4) ------------------------------------------------

import math  # noqa: E402

from bahr_pilot.attitude import euler_to_quat  # noqa: E402
from bahr_pilot.nucleo_link import ImuData  # noqa: E402


def fake_imu(quat=(0.0, 0.0, 0.0, 1.0), gyro=(0.0, 0.0, 0.0), *, quat_valid=True, gyro_valid=True,
             age=0.0) -> ImuData:
    now = time.monotonic() - age
    return ImuData(t_stm_us=1, t_pi=now, rx_time=now, quat=quat, gyro=gyro, accel=(0.0, 0.0, 0.0),
                   quat_valid=quat_valid, gyro_valid=gyro_valid, accel_valid=False, quat_accuracy=3)


def test_attitude_comes_from_the_imu_quaternion(rc_vehicle):
    rc_vehicle.nucleo.imu = fake_imu(quat=euler_to_quat(math.radians(10), math.radians(-5), 1.0),
                                     gyro=(0.1, 0.2, 0.3))
    roll, pitch, p, q, r = rc_vehicle._attitude()
    assert math.degrees(roll) == pytest.approx(10.0, abs=1e-6)
    assert math.degrees(pitch) == pytest.approx(-5.0, abs=1e-6)
    assert (p, q, r) == pytest.approx((0.1, 0.2, 0.3))


def test_attitude_applies_the_board_orientation(rc_vehicle):
    rc_vehicle.params["AHRS_ORIENTATION"] = 4.0  # board yawed 180 deg
    rc_vehicle.nucleo.imu = fake_imu(gyro=(1.0, 0.0, 0.0))
    _, _, p, q, _ = rc_vehicle._attitude()
    assert p == pytest.approx(-1.0) and q == pytest.approx(0.0, abs=1e-9)


def test_attitude_is_zero_without_fresh_imu_data(rc_vehicle):
    assert rc_vehicle._attitude() == (0.0, 0.0, 0.0, 0.0, 0.0)  # no frame yet
    rc_vehicle.nucleo.imu = fake_imu(quat=euler_to_quat(0.3, 0.0, 0.0), age=5.0)
    assert rc_vehicle._attitude() == (0.0, 0.0, 0.0, 0.0, 0.0)  # stale


def test_invalid_fields_are_not_passed_on(rc_vehicle):
    rc_vehicle.nucleo.imu = fake_imu(quat=euler_to_quat(0.3, 0.0, 0.0), gyro=(5.0, 5.0, 5.0),
                                     quat_valid=False, gyro_valid=False)
    assert rc_vehicle._attitude() == (0.0, 0.0, 0.0, 0.0, 0.0)


def test_no_nucleo_means_no_attitude(vehicle):
    assert vehicle._attitude() == (0.0, 0.0, 0.0, 0.0, 0.0)
    assert "AHRS_ORIENTATION" in DEFAULT_PARAMS


# -- GNSS (Phase 5) -------------------------------------------------------------

def test_gnss_source_changes_are_announced_once(rc_vehicle):
    state = rc_vehicle.state
    for source in ("rtd100", "rtd100", "echomap", "echomap", "rtd100", ""):
        state.gnss_source = source
        rc_vehicle._report_gnss_source()
    assert rc_vehicle.texts == [
        "GNSS: RTD100 lost, using echoMAP GPS (low accuracy)",
        "GNSS: RTD100 fix restored",
        "GNSS: no usable fix",
    ]


def test_dop_is_reported_in_hundredths_with_65535_unknown():
    assert Vehicle._dop_cents(None) == 65535
    assert Vehicle._dop_cents(0.9) == 90
    assert Vehicle._dop_cents(2.5) == 250
    assert Vehicle._dop_cents(1e9) == 65534
    assert Vehicle._dop_cents(-1.0) == 0


def capture_position_messages(vehicle, monkeypatch):
    sent = {}
    monkeypatch.setattr(vehicle.link.mav, "global_position_int_send",
                        lambda *a, **k: sent.__setitem__("global", a))
    monkeypatch.setattr(vehicle.link.mav, "gps_raw_int_send",
                        lambda *a, **k: sent.__setitem__("raw", a))
    vehicle.send_telemetry()
    return sent


def test_unknown_heading_is_reported_as_65535(vehicle, monkeypatch):
    vehicle.state.pose = make_pose(41.0, 29.0, heading_deg=153.0, heading_valid=False)
    sent = capture_position_messages(vehicle, monkeypatch)
    assert sent["global"][-1] == 65535 and sent["raw"][8] == 65535


def test_valid_heading_goes_out_in_centidegrees_and_hdop_in_eph(vehicle, monkeypatch):
    vehicle.state.pose = make_pose(41.0, 29.0, heading_deg=153.2)
    vehicle.state.hdop = 0.9
    sent = capture_position_messages(vehicle, monkeypatch)
    assert sent["global"][-1] == 15320
    assert sent["raw"][5] == 90 and sent["raw"][6] == 65535   # eph, epv


# -- estimator integration ------------------------------------------------------------------

def test_update_estimate_turns_gnss_and_gyro_into_a_pose(vehicle, monkeypatch):
    from bahr_pilot.gnss import GnssFix, GnssHeading, GnssQuality
    now = time.monotonic()
    fix = GnssFix(t=now, lat=41.0, lon=29.0, alt_m=5.0, quality=GnssQuality.RTK_FIXED,
                  satellites=20, source="rtd100", h_acc_m=0.02)
    heading = GnssHeading(t=now, heading_deg=77.0, acc_deg=0.3, baseline_m=1.0,
                          quality=GnssQuality.RTK_FIXED, pos_type="NARROW_INT")
    monkeypatch.setattr(vehicle.gnss, "best_fix", lambda t=None: fix)
    monkeypatch.setattr(vehicle.gnss, "best_heading", lambda t=None: heading)
    assert vehicle.state.pose is None
    vehicle._update_estimate()
    pose = vehicle.state.pose
    assert pose.position_valid and pose.heading_valid
    assert (pose.lat, pose.lon) == pytest.approx((41.0, 29.0), abs=1e-7)
    assert pose.heading_deg == pytest.approx(77.0, abs=0.01)
    assert vehicle._display_heading_deg == pytest.approx(77.0, abs=0.01)


def test_without_gnss_the_pose_is_unknown_not_stale(vehicle):
    vehicle._update_estimate()
    pose = vehicle.state.pose
    assert not pose.position_valid and not pose.heading_valid and pose.lat is None


def test_global_position_is_the_estimate_and_gps_raw_is_the_receiver(vehicle, monkeypatch):
    vehicle.state.lat, vehicle.state.lon = 41.5, 29.5      # what the receiver said
    vehicle.state.fix_type = 6
    vehicle.state.pose = make_pose(41.0, 29.0, heading_deg=10.0, speed_mps=1.5, course_deg=90.0,
                                   velocity_en=(1.5, 0.0))
    sent = capture_position_messages(vehicle, monkeypatch)
    assert sent["global"][1:3] == (int(41.0 * 1e7), int(29.0 * 1e7))
    assert sent["raw"][2:4] == (int(41.5 * 1e7), int(29.5 * 1e7))
    # vx = north, vy = east, cm/s: due east at 1.5 m/s
    assert sent["global"][5:8] == (0, 150, 0)
    assert sent["raw"][7] == 150 and sent["raw"][8] == 9000      # vel cm/s, cog cdeg


def test_course_over_ground_is_unknown_when_the_boat_is_not_moving(vehicle, monkeypatch):
    vehicle.state.pose = make_pose(41.0, 29.0, speed_mps=0.05, course_deg=None)
    sent = capture_position_messages(vehicle, monkeypatch)
    assert sent["raw"][8] == 65535


def test_no_pose_sends_zeros_and_unknown_heading(vehicle, monkeypatch):
    vehicle.state.pose = make_pose(None, None, heading_valid=False)
    sent = capture_position_messages(vehicle, monkeypatch)
    assert sent["global"][1:3] == (0, 0) and sent["global"][-1] == 65535


def test_home_is_set_by_the_first_3d_fix_even_if_earlier_fixes_were_2d(vehicle):
    """The old code only looked at the very first loop iteration."""
    vehicle.state.lat, vehicle.state.lon, vehicle.state.fix_type = 41.0, 29.0, 2
    vehicle._update_home()
    assert vehicle.state.home_lat is None
    vehicle.state.lat, vehicle.state.lon, vehicle.state.fix_type = 41.1, 29.1, 3
    vehicle._update_home()
    assert (vehicle.state.home_lat, vehicle.state.home_lon) == (41.1, 29.1)
    vehicle.state.lat, vehicle.state.lon = 42.0, 30.0       # moving on must not move home
    vehicle._update_home()
    assert vehicle.state.home_lat == 41.1


def test_the_motor_command_follows_the_estimated_pose_not_the_raw_fix(vehicle):
    _guided_vehicle_heading_to_target(vehicle)
    vehicle.state.gcs_last_seen = time.monotonic()
    forward = vehicle._motor_command()
    assert forward[0] > PULSE_NEUTRAL_US
    vehicle.state.pose = None                  # estimator has nothing: hold still, whatever the raw fields say
    vehicle.state.lat, vehicle.state.lon = 41.0, 29.0
    assert vehicle._motor_command() == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)


def test_nav_controller_output_carries_the_followers_aim_and_cross_track(vehicle, monkeypatch):
    from bahr_pilot import geo as geo_
    frame = geo_.LocalFrame(41.0, 29.0)
    vehicle.navigator.mission = [frame.to_geodetic(0, 0), frame.to_geodetic(0, 100), frame.to_geodetic(100, 100)]
    vehicle.navigator.mission_seq = 2
    vehicle.state.mode = MODE_AUTO
    lat, lon = frame.to_geodetic(10.0, 105.0)             # 5 m left of the eastbound leg
    vehicle.state.pose = make_pose(lat, lon, heading_deg=90.0, speed_mps=1.5)
    vehicle.state.armed = True
    vehicle.state.gcs_last_seen = time.monotonic()
    vehicle._motor_command()                               # one navigation step fills the guidance
    sent = {}
    monkeypatch.setattr(vehicle.link.mav, "nav_controller_output_send", lambda *a, **k: sent.__setitem__("nav", a))
    vehicle.send_telemetry()
    nav_bearing, wp_dist, xtrack = sent["nav"][2], sent["nav"][4], sent["nav"][7]
    assert 90 < nav_bearing < 180                          # aiming back towards the line, south of east
    assert xtrack == pytest.approx(-5.0, abs=0.05)         # left of the line = negative
    assert wp_dist == int(geo_.distance_m(lat, lon, *vehicle.navigator.mission[2]))


def test_the_l1_parameters_reach_the_navigator(vehicle):
    vehicle.params["NAVL1_PERIOD"] = 6.0
    vehicle.params["NAVL1_DAMPING"] = 0.9
    vehicle.state.mode = MODE_AUTO
    vehicle.state.armed = True
    vehicle.state.gcs_last_seen = time.monotonic()
    vehicle._motor_command()
    assert (vehicle.navigator.l1_period_s, vehicle.navigator.l1_damping) == (6.0, 0.9)
    assert "NAVL1_PERIOD" in DEFAULT_PARAMS and DEFAULT_PARAMS["NAVL1_DAMPING"] == 0.75


# -- Phase 20: failsafe enforcement ---------------------------------------------------------------------

class _Statustexts:
    def __init__(self, vehicle, monkeypatch):
        self.texts = []
        monkeypatch.setattr(vehicle.link.mav, "statustext_send",
                            lambda severity, text: self.texts.append((severity, bytes(text).decode())))


T_HEARD = 100.0          # when the GCS was last heard (monotonic seconds)
T_SILENT = 200.0         # a later "now": 100 s of silence, far beyond FS_TIMEOUT


@pytest.fixture
def armed_auto(vehicle, monkeypatch):
    """Armed, AUTO, a good pose, a GCS that has been heard recently."""
    vehicle.navigator.mission = [(41.0, 29.0), (41.001, 29.0), (41.002, 29.0)]
    vehicle.navigator.mission_seq = 1
    vehicle.state.mode = MODE_AUTO
    vehicle.state.armed = True
    vehicle.state.pose = make_pose(41.0, 29.0, speed_mps=1.5)
    vehicle.state.home_lat, vehicle.state.home_lon = 41.0, 29.0
    vehicle.state.gcs_last_seen = T_HEARD
    vehicle.texts = _Statustexts(vehicle, monkeypatch)
    return vehicle


def test_gcs_loss_in_auto_switches_to_hold_and_says_so(armed_auto):
    armed_auto._update_failsafes(T_SILENT)
    assert armed_auto.state.mode == MODE_HOLD
    sent = [text for _, text in armed_auto.texts.texts]
    assert "Failsafe: GCS link lost (HOLD)" in sent and "Failsafe: mode forced to HOLD" in sent
    assert armed_auto._motor_command() == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)


def test_the_mission_does_not_resume_by_itself_when_the_link_returns(armed_auto):
    armed_auto._update_failsafes(T_SILENT)
    for k in range(40):                                    # 8 s of a healthy link: heard every step
        now = T_SILENT + 1.0 + 0.2 * k
        armed_auto.state.gcs_last_seen = now
        armed_auto._update_failsafes(now)
    assert armed_auto.failsafe.active == {}                # cleared...
    assert armed_auto.state.mode == MODE_HOLD              # ...but the operator has to re-engage
    assert armed_auto.set_mode(MODE_AUTO) and armed_auto.state.mode == MODE_AUTO


def test_selecting_auto_again_while_the_cause_persists_gives_hold_again(armed_auto):
    armed_auto._update_failsafes(T_SILENT)
    armed_auto.set_mode(MODE_AUTO)
    armed_auto._update_failsafes(T_SILENT + 0.2)
    assert armed_auto.state.mode == MODE_HOLD


def test_fs_action_rtl_goes_home_and_falls_back_to_hold_without_a_home(armed_auto):
    armed_auto.params["FS_ACTION"] = 1.0
    armed_auto._update_failsafes(T_SILENT)
    assert armed_auto.state.mode == MODE_RTL
    armed_auto.state.mode = MODE_AUTO
    armed_auto.state.home_lat = armed_auto.state.home_lon = None
    armed_auto._update_failsafes(T_SILENT + 0.2)
    assert armed_auto.state.mode == MODE_HOLD
    assert "Failsafe: RTL impossible, holding" in [text for _, text in armed_auto.texts.texts]


def test_fs_action_terminate_disarms(armed_auto):
    armed_auto.params["FS_ACTION"] = 5.0
    armed_auto._update_failsafes(T_SILENT)
    assert armed_auto.state.armed is False and armed_auto.state.mode == MODE_HOLD
    assert armed_auto._motor_command() == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)


def test_fs_action_report_only_keeps_flying(armed_auto):
    armed_auto.params["FS_ACTION"] = 0.0
    armed_auto._update_failsafes(T_SILENT)
    assert armed_auto.state.mode == MODE_AUTO
    assert "Failsafe: GCS link lost (report only)" in [text for _, text in armed_auto.texts.texts]
    m1, m2 = armed_auto._motor_command()
    assert m1 > PULSE_NEUTRAL_US and m2 > PULSE_NEUTRAL_US


def test_manual_mode_is_never_changed_by_a_failsafe(armed_auto):
    armed_auto.state.mode = MODE_MANUAL
    armed_auto._update_failsafes(T_SILENT)
    assert armed_auto.state.mode == MODE_MANUAL
    assert Source_.GCS_LOST in armed_auto.failsafe.active           # it is still reported


def test_losing_the_position_in_auto_holds_after_a_second(armed_auto):
    armed_auto.state.pose = make_pose(None, None)
    armed_auto._update_failsafes(10.0)
    assert armed_auto.state.mode == MODE_AUTO                 # not yet: hold time
    armed_auto._update_failsafes(11.5)
    assert armed_auto.state.mode == MODE_HOLD
    assert "Failsafe: position lost (HOLD)" in [text for _, text in armed_auto.texts.texts]


def test_a_never_connected_gcs_does_not_trigger_the_link_failsafe(armed_auto):
    armed_auto.state.gcs_last_seen = 0.0                      # armed from the RC without a GCS
    armed_auto._update_failsafes(1.0)
    armed_auto._update_failsafes(100.0)
    assert armed_auto.failsafe.active == {} and armed_auto.state.mode == MODE_AUTO


def test_low_battery_holds_when_enabled(armed_auto):
    from types import SimpleNamespace
    from bahr_pilot.nucleo_link import RcTelemetry
    telemetry = RcTelemetry()
    telemetry.battery_valid, telemetry.battery_mv = True, 9500
    armed_auto.nucleo = SimpleNamespace(telemetry=telemetry)
    armed_auto.params["BATT_FS_ENABLE"] = 1.0                 # 9.5 V < BATT_LOW_VOLT 10.5
    armed_auto._update_failsafes(0.0)
    assert armed_auto.state.mode == MODE_AUTO                 # a load sag is not enough
    armed_auto._update_failsafes(3.5)
    assert armed_auto.state.mode == MODE_HOLD
    assert "Failsafe: battery low (HOLD)" in [text for _, text in armed_auto.texts.texts]


def test_rtl_is_refused_without_a_home(vehicle, monkeypatch):
    texts = _Statustexts(vehicle, monkeypatch)
    vehicle.state.home_lat = None
    assert vehicle.set_mode(MODE_RTL) is False and vehicle.state.mode != MODE_RTL
    assert "RTL refused: no home position" in [text for _, text in texts.texts]
    vehicle.state.home_lat, vehicle.state.home_lon = 41.0, 29.0
    assert vehicle.set_mode(MODE_RTL) is True


def test_failsafe_defaults_are_ardurovers():
    for name in ("FS_ACTION", "FS_EKF_ACTION", "BATT_CRT_VOLT", "BATT_FS_LOW_ACT", "BATT_FS_CRT_ACT"):
        assert name in DEFAULT_PARAMS
    assert DEFAULT_PARAMS["FS_ACTION"] == 2.0 and DEFAULT_PARAMS["FS_EKF_ACTION"] == 1.0
    assert DEFAULT_PARAMS["BATT_CRT_VOLT"] == 0.0 and DEFAULT_PARAMS["BATT_FS_ENABLE"] == 0.0


# -- Phase 21: geofence over MAVLink --------------------------------------------------------------------

from types import SimpleNamespace

from bahr_pilot import geo as _geo
from bahr_pilot.geofence import CMD_POLYGON_EXCLUSION, CMD_POLYGON_INCLUSION

_F = _geo.LocalFrame(41.0, 29.0)
_SQUARE = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]
_MISSION, _FENCE, _RALLY = 0, 1, 2


def _calls(vehicle, monkeypatch):
    calls = []
    for name in ("mission_ack_send", "mission_request_int_send", "mission_count_send", "mission_item_int_send"):
        monkeypatch.setattr(vehicle.link.mav, name, lambda *a, _n=name, **k: calls.append((_n, a)))
    return calls


def _fence_item(seq, command, p1, east, north):
    lat, lon = _F.to_geodetic(east, north)
    return SimpleNamespace(mission_type=_FENCE, seq=seq, command=command, param1=p1,
                           x=int(round(lat * 1e7)), y=int(round(lon * 1e7)))


def _upload_square(vehicle, command=CMD_POLYGON_INCLUSION):
    vehicle._on_mission_count(SimpleNamespace(mission_type=_FENCE, count=4))
    for seq, (east, north) in enumerate(_SQUARE):
        vehicle._on_mission_item_int(_fence_item(seq, command, 4.0, east, north))


def test_a_fence_upload_follows_the_mavlink_sequence_and_is_acked(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    requests = [a for name, a in calls if name == "mission_request_int_send"]
    assert [(a[2], a[3]) for a in requests] == [(0, _FENCE), (1, _FENCE), (2, _FENCE), (3, _FENCE)]
    acks = [a for name, a in calls if name == "mission_ack_send"]
    assert acks == [(255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED, _FENCE)]
    assert vehicle.geofence.has_shapes and len(vehicle._fence_items) == 4


def test_a_fence_upload_does_not_touch_the_mission(vehicle, monkeypatch):
    """Until Phase 21 the mission_type was ignored: a fence would have replaced the mission."""
    _calls(vehicle, monkeypatch)
    vehicle.navigator.mission = [(41.0, 29.0), (41.001, 29.0), (41.002, 29.0)]
    vehicle.navigator.mission_seq = 2
    before = list(vehicle.navigator.mission)
    _upload_square(vehicle)
    assert vehicle.navigator.mission == before and vehicle.navigator.mission_seq == 2


def test_a_mission_upload_still_works_and_does_not_touch_the_fence(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    calls.clear()
    vehicle._on_mission_count(SimpleNamespace(mission_type=_MISSION, count=3))
    for seq in range(3):
        vehicle._on_mission_item_int(SimpleNamespace(mission_type=_MISSION, seq=seq, x=int((41.0 + seq * 1e-3) * 1e7),
                                                     y=int(29.0 * 1e7), command=16, param1=0.0))
    assert len(vehicle.navigator.mission) == 3
    assert vehicle.geofence.has_shapes and len(vehicle._fence_items) == 4
    requests = [a for name, a in calls if name == "mission_request_int_send"]
    assert [a[2] for a in requests] == [0, 1, 2]
    assert [name for name, _ in calls].count("mission_ack_send") == 1


def test_a_message_without_mission_type_is_a_normal_mission_message(vehicle, monkeypatch):
    """pymavlink 1.0 clients and BAHR-GCS may omit the field entirely."""
    _calls(vehicle, monkeypatch)
    vehicle._on_mission_count(SimpleNamespace(count=2))
    vehicle._on_mission_item_int(SimpleNamespace(seq=0, x=int(41.0 * 1e7), y=int(29.0 * 1e7)))
    vehicle._on_mission_item_int(SimpleNamespace(seq=1, x=int(41.001 * 1e7), y=int(29.0 * 1e7)))
    assert len(vehicle.navigator.mission) == 2


def test_an_unusable_fence_is_refused_and_the_old_one_is_kept(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    calls.clear()
    vehicle._on_mission_count(SimpleNamespace(mission_type=_FENCE, count=2))
    vehicle._on_mission_item_int(_fence_item(0, CMD_POLYGON_INCLUSION, 2.0, 0.0, 0.0))
    vehicle._on_mission_item_int(_fence_item(1, CMD_POLYGON_INCLUSION, 2.0, 10.0, 0.0))
    acks = [a for name, a in calls if name == "mission_ack_send"]
    assert acks == [(255, 0, mavutil.mavlink.MAV_MISSION_INVALID, _FENCE)]
    assert len(vehicle._fence_items) == 4 and vehicle.geofence.has_shapes


def test_an_out_of_sequence_fence_item_aborts_the_upload(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    vehicle._on_mission_count(SimpleNamespace(mission_type=_FENCE, count=4))
    vehicle._on_mission_item_int(_fence_item(0, CMD_POLYGON_INCLUSION, 4.0, -50.0, -50.0))
    vehicle._on_mission_item_int(_fence_item(2, CMD_POLYGON_INCLUSION, 4.0, 50.0, 50.0))      # 1 was skipped
    assert [a for name, a in calls if name == "mission_ack_send"] == [
        (255, 0, mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE, _FENCE)]
    assert vehicle._fence_upload is None and not vehicle.geofence.has_shapes


def test_the_stored_fence_can_be_read_back_item_for_item(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle, CMD_POLYGON_EXCLUSION)
    calls.clear()
    vehicle._on_mission_request_list(SimpleNamespace(mission_type=_FENCE))
    assert calls == [("mission_count_send", (255, 0, 4, _FENCE))]
    calls.clear()
    for seq in range(4):
        vehicle._on_mission_request_int(SimpleNamespace(mission_type=_FENCE, seq=seq))
    items = [a for name, a in calls if name == "mission_item_int_send"]
    assert len(items) == 4
    for (target_system, target_component, seq, frame, command, current, autocontinue, p1, p2, p3, p4, x, y, z, kind), \
            expected in zip(items, vehicle._fence_items):
        assert (command, p1, kind) == (CMD_POLYGON_EXCLUSION, 4.0, _FENCE)
        assert x == pytest.approx(expected[2] * 1e7, abs=1) and y == pytest.approx(expected[3] * 1e7, abs=1)


def test_mission_download_is_unchanged_and_ignores_the_fence(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    vehicle.navigator.mission = [(41.0, 29.0), (41.001, 29.0)]
    calls.clear()
    vehicle._on_mission_request_list(SimpleNamespace())                      # no mission_type at all
    assert calls == [("mission_count_send", (255, 0, 2))]


def test_rally_points_are_refused_not_misfiled(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    vehicle.navigator.mission = [(41.0, 29.0), (41.001, 29.0)]
    vehicle._on_mission_count(SimpleNamespace(mission_type=_RALLY, count=3))
    assert calls == [("mission_ack_send", (255, 0, mavutil.mavlink.MAV_MISSION_UNSUPPORTED, _RALLY))]
    assert len(vehicle.navigator.mission) == 2
    calls.clear()
    vehicle._on_mission_request_list(SimpleNamespace(mission_type=_RALLY))
    assert calls == [("mission_count_send", (255, 0, 0, _RALLY))]


def test_clear_all_is_typed(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    vehicle.navigator.mission = [(41.0, 29.0), (41.001, 29.0)]
    calls.clear()
    vehicle._on_mission_clear_all(SimpleNamespace(mission_type=_FENCE))
    assert not vehicle.geofence.has_shapes and vehicle._fence_items == [] and len(vehicle.navigator.mission) == 2
    assert calls == [("mission_ack_send", (255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED, _FENCE))]
    calls.clear()
    _upload_square(vehicle)
    calls.clear()
    vehicle._on_mission_clear_all(SimpleNamespace(mission_type=_MISSION))
    assert vehicle.navigator.mission == [] and vehicle.geofence.has_shapes
    assert calls == []                      # no reply for type 0: BAHR-GCS would count it as a MISSION_ACK
    vehicle._on_mission_clear_all(SimpleNamespace())                       # no field = type 0 as well
    assert calls == []
    vehicle.navigator.mission = [(41.0, 29.0)]
    vehicle._on_mission_clear_all(SimpleNamespace(mission_type=255))
    assert vehicle.navigator.mission == [] and not vehicle.geofence.has_shapes
    assert calls == [("mission_ack_send", (255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED, 255))]


def test_an_empty_fence_upload_clears_the_fence(vehicle, monkeypatch):
    calls = _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    calls.clear()
    vehicle._on_mission_count(SimpleNamespace(mission_type=_FENCE, count=0))
    assert not vehicle.geofence.has_shapes
    assert calls == [("mission_ack_send", (255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED, _FENCE))]


def test_waypoints_outside_the_fence_are_reported_at_upload(vehicle, monkeypatch):
    texts = _Statustexts(vehicle, monkeypatch)
    _calls(vehicle, monkeypatch)
    vehicle.params["FENCE_ENABLE"] = 1.0
    vehicle.params["FENCE_TYPE"] = 4.0
    vehicle.navigator.mission = [_F.to_geodetic(0, 0), _F.to_geodetic(0, 20), _F.to_geodetic(80, 0), _F.to_geodetic(0, -90)]
    _upload_square(vehicle)
    said = [text for _, text in texts.texts]
    assert "Fence: waypoint 2 is 30 m outside" in said and "Fence: waypoint 3 is 40 m outside" in said
    assert not any("waypoint 1" in text for text in said)


def test_a_stored_fence_with_the_fence_disabled_says_so(vehicle, monkeypatch):
    texts = _Statustexts(vehicle, monkeypatch)
    _calls(vehicle, monkeypatch)
    _upload_square(vehicle)
    assert "Fence stored but FENCE_ENABLE is 0" in [text for _, text in texts.texts]


def test_fence_defaults_are_off_and_ardurovers(vehicle):
    for name in ("FENCE_ENABLE", "FENCE_TYPE", "FENCE_ACTION", "FENCE_RADIUS", "FENCE_MARGIN"):
        assert name in DEFAULT_PARAMS
    assert DEFAULT_PARAMS["FENCE_ENABLE"] == 0.0               # a fence nobody drew must not stop a boat


def test_a_breach_in_auto_goes_home_through_the_failsafe(armed_auto):
    armed_auto.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 100.0, "FENCE_ACTION": 1.0})
    armed_auto.state.home_lat, armed_auto.state.home_lon = _F.to_geodetic(0.0, 0.0)
    armed_auto.state.pose = make_pose(*_F.to_geodetic(0.0, 150.0), speed_mps=1.5)           # 50 m outside
    armed_auto._update_failsafes(T_HEARD)
    assert armed_auto.state.mode == MODE_AUTO                    # hold time
    armed_auto._update_failsafes(T_HEARD + 1.0)
    assert armed_auto.state.mode == MODE_RTL
    assert "Failsafe: fence breached (RTL)" in [text for _, text in armed_auto.texts.texts]


def test_the_position_of_the_fence_check_is_the_estimate_not_the_raw_fix(armed_auto):
    armed_auto.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 100.0})
    armed_auto.state.home_lat, armed_auto.state.home_lon = _F.to_geodetic(0.0, 0.0)
    armed_auto.state.lat, armed_auto.state.lon = _F.to_geodetic(0.0, 500.0)               # a wild RAW fix
    armed_auto.state.pose = make_pose(*_F.to_geodetic(0.0, 10.0), speed_mps=1.5)           # the estimate is fine
    armed_auto._update_failsafes(T_HEARD)
    armed_auto._update_failsafes(T_HEARD + 5.0)
    assert Source_.GEOFENCE not in armed_auto.failsafe.active


def test_the_fence_check_looks_ahead_along_the_velocity(vehicle):
    """Circle r = 100 m around home; the boat is 20 m inside, north of home.
    Stop distance at 5 m/s: 5 x 1.0 s reaction + 25 / (2 x 0.3 m/s^2 glide) = 46.67 m."""
    vehicle.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 100.0})
    vehicle.state.home_lat, vehicle.state.home_lon = _F.to_geodetic(0.0, 0.0)
    here = _F.to_geodetic(0.0, 80.0)
    outward = make_pose(*here, speed_mps=5.0, velocity_en=(0.0, 5.0))
    inward = make_pose(*here, speed_mps=5.0, velocity_en=(0.0, -5.0))
    still = make_pose(*here, speed_mps=0.0)
    slow = make_pose(*here, speed_mps=0.1, velocity_en=(0.0, 0.1))
    assert vehicle._fence_margin(outward) == pytest.approx(20.0 - 46.667, abs=0.05)     # would stop outside: act now
    assert vehicle._fence_margin(inward) == pytest.approx(20.0, abs=0.01)               # moving away: no early trigger
    assert vehicle._fence_margin(still) == pytest.approx(20.0, abs=0.01)
    assert vehicle._fence_margin(slow) == pytest.approx(20.0, abs=0.01)                 # too slow to predict from


def test_the_fence_check_has_no_opinion_without_a_position_or_with_the_fence_off(vehicle):
    vehicle.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 100.0})
    vehicle.state.home_lat, vehicle.state.home_lon = _F.to_geodetic(0.0, 0.0)
    assert vehicle._fence_margin(None) is None
    assert vehicle._fence_margin(make_pose(None, None)) is None
    vehicle.params["FENCE_ENABLE"] = 0.0
    assert vehicle._fence_margin(make_pose(*_F.to_geodetic(0.0, 500.0))) is None


def test_an_already_outside_boat_stays_breached_even_when_heading_back_in(vehicle):
    """The smaller of (here, ahead) counts, so a boat still outside is not cleared by
    the fact that it is heading inwards."""
    vehicle.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 100.0})
    vehicle.state.home_lat, vehicle.state.home_lon = _F.to_geodetic(0.0, 0.0)
    pose = make_pose(*_F.to_geodetic(0.0, 105.0), speed_mps=2.0, velocity_en=(0.0, -2.0))
    assert vehicle._fence_margin(pose) == pytest.approx(-5.0, abs=0.01)


def test_rtl_is_refused_when_the_way_home_crosses_a_no_go_area(vehicle, monkeypatch):
    texts = _Statustexts(vehicle, monkeypatch)
    _calls(vehicle, monkeypatch)
    vehicle.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 4.0})
    vehicle.state.home_lat, vehicle.state.home_lon = _F.to_geodetic(0.0, 0.0)
    vehicle.state.pose = make_pose(*_F.to_geodetic(0.0, 100.0))
    vehicle._on_mission_count(SimpleNamespace(mission_type=_FENCE, count=4))
    for seq, (east, north) in enumerate([(-20.0, 40.0), (20.0, 40.0), (20.0, 60.0), (-20.0, 60.0)]):
        vehicle._on_mission_item_int(_fence_item(seq, CMD_POLYGON_EXCLUSION, 4.0, east, north))
    assert vehicle.set_mode(MODE_RTL) is False and vehicle.state.mode != MODE_RTL
    assert "RTL refused: path to home crosses the fence" in [text for _, text in texts.texts]
    vehicle.state.pose = make_pose(*_F.to_geodetic(60.0, 100.0))                 # beside the zone: a clear run
    assert vehicle.set_mode(MODE_RTL) is True


def test_a_failsafe_rtl_that_would_cross_a_no_go_area_holds_instead(armed_auto):
    armed_auto.params.update({"FENCE_ENABLE": 1.0, "FENCE_TYPE": 4.0, "FS_ACTION": 1.0})
    armed_auto.state.home_lat, armed_auto.state.home_lon = _F.to_geodetic(0.0, 0.0)
    armed_auto.state.pose = make_pose(*_F.to_geodetic(0.0, 100.0), speed_mps=1.5)
    armed_auto.geofence.load([(CMD_POLYGON_EXCLUSION, 4.0, *_F.to_geodetic(e, n))
                              for e, n in [(-20.0, 40.0), (20.0, 40.0), (20.0, 60.0), (-20.0, 60.0)]])
    armed_auto._update_failsafes(T_SILENT)                                       # GCS silent: FS_ACTION 1 = RTL
    assert armed_auto.state.mode == MODE_HOLD
    assert "Failsafe: RTL impossible, holding" in [text for _, text in armed_auto.texts.texts]


def test_rtl_is_not_second_guessed_without_a_fence_or_a_position(vehicle):
    vehicle.state.home_lat, vehicle.state.home_lon = _F.to_geodetic(0.0, 0.0)
    vehicle.state.pose = None
    assert vehicle.set_mode(MODE_RTL) is True                                    # no pose: nothing to check
    vehicle.state.mode = MODE_AUTO
    vehicle.state.pose = make_pose(*_F.to_geodetic(0.0, 100.0))
    assert vehicle.set_mode(MODE_RTL) is True                                    # no fence at all
