"""The Nucleo's failsafe priority table, run for real: tests/c/failsafe_harness.c
is firmware/reflex/Core/Src/failsafe.c compiled on the host together with the
real arming.c, motor.c and mode_switch.c; only the receiver, ESC outputs, Pi
link, battery and IMU are stubbed. Until this existed, the table (the most
safety-critical logic in the project) had only ever been compiled.

Units: SBUS raw values 172..1811, 992 = centre; outputs are ESC pulses in us
(motor 1 = LEFT, motor 2 = RIGHT).
"""
from __future__ import annotations

import subprocess

import pytest

SBUS_MIN, SBUS_MID, SBUS_MAX = 172, 992, 1811
CH_STEERING, CH_THROTTLE, CH_MODE, CH_ARM = 0, 2, 4, 5
BAND_MANUAL, BAND_HOLD, BAND_AUTO = SBUS_MIN, SBUS_MID, SBUS_MAX  # mode slots 1 / 4 / 6
ARM_BLOCK_SWITCH_ON, ARM_BLOCK_THROTTLE = 3, 4
STATE_READY, STATE_ARMED = 2, 3


class Failsafe:
    def __init__(self, exe):
        self.proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.out = (1500, 1500)
        self.armed = False
        self.link(True)
        self.sensors(imu=True, battery=True)

    def cmd(self, line: str) -> str:
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        return self.proc.stdout.readline().strip()

    def ch(self, idx: int, value: int) -> None:
        assert self.cmd(f"ch {idx} {value}") == "ok"

    def link(self, up: bool) -> None:
        assert self.cmd(f"link {int(up)}") == "ok"

    def sensors(self, imu: bool, battery: bool) -> None:
        assert self.cmd(f"sensors {int(imu)} {int(battery)}") == "ok"

    def pi(self, fresh: bool, m1: int = 1500, m2: int = 1500) -> None:
        assert self.cmd(f"pi {int(fresh)} {m1} {m2}") == "ok"

    def mask(self, value: int) -> None:
        assert self.cmd(f"mask {value}") == "ok"

    def update(self) -> tuple[int, int]:
        reply = self.cmd("update").split()
        assert reply[0] == "out"
        self.out = (int(reply[1]), int(reply[2]))
        self.armed = bool(int(reply[3]))
        return self.out

    def run(self, ms: int, step: int = 10) -> tuple[int, int]:
        for _ in range(ms // step):
            assert self.cmd(f"tick {step}") == "ok"
            self.update()
        return self.out

    def tele(self) -> dict:
        fields = self.cmd("tele").split()
        keys = ["armed", "rc", "override", "bat", "mv", "imu", "roll", "pitch", "slot", "arm_info",
                "calls"]
        values = dict(zip(keys, (int(v) for v in fields[1:])))
        values["arm_state"] = values["arm_info"] & 0x07
        values["arm_block"] = values["arm_info"] >> 4
        return values

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=10)

    # --- scenario helpers ---------------------------------------------------
    def sticks_centred(self):
        self.ch(CH_THROTTLE, SBUS_MID)
        self.ch(CH_STEERING, SBUS_MID)

    def arm(self, band: int = BAND_MANUAL):
        """Bring the boat up properly: sticks centred, arm switch off through
        the boot grace period, then switched on."""
        self.sticks_centred()
        self.ch(CH_MODE, band)
        self.ch(CH_ARM, SBUS_MIN)
        self.run(2500)
        self.ch(CH_ARM, SBUS_MAX)
        self.run(50)
        assert self.armed, "failed to arm"


@pytest.fixture
def fs(build):
    harness = Failsafe(build.failsafe)
    yield harness
    harness.close()


def test_power_up_with_the_arm_switch_already_on_never_moves_the_motors(fs):
    """The bug the arm state machine was written for: transmitter on, arm
    switch on, throttle forward, THEN the boat gets power."""
    fs.ch(CH_ARM, SBUS_MAX)
    fs.ch(CH_MODE, BAND_MANUAL)
    fs.ch(CH_THROTTLE, SBUS_MAX)
    for _ in range(2000):  # 20 s
        assert fs.run(10) == (1500, 1500)
        assert not fs.armed
    assert fs.tele()["arm_block"] == ARM_BLOCK_SWITCH_ON


def test_switch_on_at_power_up_with_sticks_centred_still_does_not_arm(fs):
    """Isolates the "switch must be seen off first" rule: with the sticks
    centred the throttle check cannot be what saves us, and nobody touches
    the switch, so the boat must stay disarmed for as long as it likes."""
    fs.sticks_centred()
    fs.ch(CH_ARM, SBUS_MAX)
    fs.ch(CH_MODE, BAND_MANUAL)
    for _ in range(1500):  # 15 s
        assert fs.run(10) == (1500, 1500)
        assert not fs.armed
    assert fs.tele()["arm_block"] == ARM_BLOCK_SWITCH_ON

    fs.ch(CH_THROTTLE, SBUS_MAX)         # pushing the stick must not wake it up either
    assert fs.run(1000) == (1500, 1500)

    fs.sticks_centred()
    fs.ch(CH_ARM, SBUS_MIN)              # cycling the switch is what arms it
    fs.run(100)
    fs.ch(CH_ARM, SBUS_MAX)
    fs.run(50)
    assert fs.armed


def test_boot_grace_period_holds_motors_even_with_everything_in_order(fs):
    fs.sticks_centred()
    fs.ch(CH_MODE, BAND_MANUAL)
    fs.ch(CH_ARM, SBUS_MIN)
    fs.run(500)
    fs.ch(CH_ARM, SBUS_MAX)
    fs.ch(CH_THROTTLE, SBUS_MAX)
    assert fs.run(200) == (1500, 1500)  # still inside the 2 s grace period
    assert not fs.armed


def test_manual_drive_after_a_proper_arm_ramps_up(fs):
    fs.arm()
    assert fs.armed
    fs.ch(CH_THROTTLE, SBUS_MAX)
    first = fs.run(20)
    assert 1500 <= first[0] <= 1520 and first[0] == first[1]   # not an instant jump
    after_100ms = fs.run(80)
    assert 1540 <= after_100ms[0] <= 1560                       # ~100 %/s slew
    assert fs.run(1500) == (2000, 2000)


def test_stick_right_speeds_up_the_left_motor(fs):
    fs.arm()
    fs.ch(CH_THROTTLE, 1400)   # about +50 % throttle
    fs.ch(CH_STEERING, SBUS_MAX)
    left, right = fs.run(2000)
    assert left > right
    assert left > 1500


def test_stick_deadband_prevents_creep(fs):
    fs.arm()
    fs.ch(CH_THROTTLE, SBUS_MID + 30)  # ~3.7 % of full travel, inside the 5 % band
    assert fs.run(500) == (1500, 1500)


def test_arm_switch_off_stops_instantly_from_full_speed(fs):
    fs.arm()
    fs.ch(CH_THROTTLE, SBUS_MAX)
    assert fs.run(1500) == (2000, 2000)
    fs.ch(CH_ARM, SBUS_MIN)
    assert fs.run(10) == (1500, 1500)   # one control tick, no ramp-down


def test_rc_loss_stops_instantly_and_a_short_dropout_keeps_the_boat_armed(fs):
    fs.arm()
    fs.ch(CH_THROTTLE, SBUS_MAX)
    assert fs.run(1500) == (2000, 2000)
    fs.link(False)
    assert fs.run(10) == (1500, 1500)
    fs.run(2000)                         # 2 s dropout
    fs.link(True)
    fs.run(20)
    assert fs.armed                      # still armed
    assert fs.run(1500) == (2000, 2000)  # and driving again, ramping up from zero


def test_long_rc_loss_requires_rearming(fs):
    fs.arm()
    fs.ch(CH_THROTTLE, SBUS_MAX)
    fs.run(1500)
    fs.link(False)
    fs.run(6000)
    fs.link(True)                        # transmitter back, switch still on
    for _ in range(300):
        assert fs.run(10) == (1500, 1500)
    assert not fs.armed
    fs.ch(CH_ARM, SBUS_MIN)
    fs.sticks_centred()
    fs.run(100)
    fs.ch(CH_ARM, SBUS_MAX)
    fs.run(50)
    assert fs.armed


def test_throttle_off_centre_at_arming_is_refused_and_reported(fs):
    fs.sticks_centred()
    fs.ch(CH_MODE, BAND_MANUAL)
    fs.ch(CH_ARM, SBUS_MIN)
    fs.run(2500)
    fs.ch(CH_THROTTLE, SBUS_MAX)
    fs.ch(CH_ARM, SBUS_MAX)
    assert fs.run(100) == (1500, 1500)
    assert not fs.armed
    assert fs.tele()["arm_block"] == ARM_BLOCK_THROTTLE

    fs.ch(CH_THROTTLE, SBUS_MID)         # recentring alone must not arm it
    assert fs.run(500) == (1500, 1500)
    assert not fs.armed


def test_pi_in_command_outside_the_manual_bands_but_ramped(fs):
    fs.arm(band=BAND_AUTO)
    fs.pi(True, 2000, 2000)
    first = fs.run(20)
    assert first[0] <= 1520              # a runaway Pi cannot slam the drive train
    assert fs.run(1500) == (2000, 2000)


def test_pi_pulses_keep_left_right_order(fs):
    fs.arm(band=BAND_AUTO)
    fs.pi(True, 1700, 1300)              # left ahead, right astern: pivot right
    left, right = fs.run(2000)
    assert left == 1700 and right == 1300


def test_pi_silence_stops_instantly(fs):
    fs.arm(band=BAND_AUTO)
    fs.pi(True, 2000, 2000)
    assert fs.run(1500) == (2000, 2000)
    fs.pi(False, 2000, 2000)
    assert fs.run(10) == (1500, 1500)


def test_pi_never_heard_from_means_stopped(fs):
    fs.arm(band=BAND_AUTO)
    assert fs.run(1000) == (1500, 1500)


def test_manual_band_ignores_the_pi_entirely(fs):
    fs.arm(band=BAND_MANUAL)
    fs.pi(True, 2000, 2000)              # sticks are centred, so the boat must sit still
    assert fs.run(1000) == (1500, 1500)


def test_manual_band_works_with_a_dead_pi(fs):
    fs.arm(band=BAND_MANUAL)
    fs.pi(False)
    fs.ch(CH_THROTTLE, SBUS_MAX)
    assert fs.run(1500) == (2000, 2000)


def test_hold_band_is_not_manual_and_does_not_drive(fs):
    fs.arm(band=BAND_HOLD)
    fs.ch(CH_THROTTLE, SBUS_MAX)         # sticks must not matter outside MANUAL
    assert fs.run(1000) == (1500, 1500)


def test_changing_the_manual_mask_changes_which_bands_drive(fs):
    fs.mask(0b100000)                    # only band 6 is manual
    fs.arm(band=BAND_AUTO)
    fs.ch(CH_THROTTLE, SBUS_MAX)
    assert fs.run(1500) == (2000, 2000)


def test_telemetry_reports_slot_armed_and_arm_state(fs):
    fs.arm(band=BAND_AUTO)
    fs.run(200)
    t = fs.tele()
    assert t["calls"] >= 2               # sent periodically
    assert t["slot"] == 6
    assert t["armed"] == 1 and t["rc"] == 1
    assert t["arm_state"] == STATE_ARMED
    assert t["override"] == 0


def test_imu_and_battery_checks_default_to_off(fs):
    fs.sensors(imu=False, battery=False)
    fs.arm()
    assert fs.armed
