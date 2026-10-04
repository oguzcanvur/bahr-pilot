"""Runs real firmware C code on the host and checks it against the real
Python side. The C is built by the `build` fixture (tests/conftest.py).

  * tests/c/test_mode_switch.c, test_arming.c, test_motor.c — pure logic
    modules (mode-switch bands, arm state machine, motor mixer/ramp).
  * tests/c/pi_link_harness.c — firmware/reflex/Core/Src/pi_link.c itself,
    with the HAL stubbed (tests/c/stubs), driven with frames built by the
    real Python NucleoLink and decoded by it again. This is what proves the
    two halves of the Pi<->Nucleo protocol agree byte for byte.
The failsafe priority table has its own scenarios in test_failsafe_c.py.
"""
from __future__ import annotations

import subprocess
import threading
from collections import deque
from types import SimpleNamespace

import pytest

from bahr_pilot.nucleo_link import NucleoLink, RcTelemetry


@pytest.mark.parametrize("name", ["mode_switch", "arming", "motor", "clock_core", "imu_reports"])
def test_pure_logic_modules(build, name):
    result = subprocess.run([str(getattr(build, name))], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout
    assert "all checks passed" in result.stdout


class Harness:
    def __init__(self, exe):
        self.proc = subprocess.Popen([str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)

    def cmd(self, line: str) -> str:
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        return self.proc.stdout.readline().strip()

    def rx(self, data: bytes) -> None:
        assert self.cmd(f"rx {data.hex()}") == "ok"

    def tick(self, ms: int) -> None:
        assert self.cmd(f"tick {ms}") == "ok"

    def apply_pending(self) -> None:
        """What the main loop does every tick: pick up a config frame the ISR
        stashed. (Disarmed, so it never triggers a flash write here — that
        needs the debounce to elapse too.)"""
        self.saves(armed=False)

    def saves(self, armed: bool) -> int:
        reply = self.cmd(f"process {int(armed)}")
        assert reply.startswith("saves ")
        return int(reply.split()[1])

    def rc_map(self) -> dict:
        fields = self.cmd("map").split()
        assert fields[0] == "map"
        names = ["throttle_channel", "steering_channel", "arm_channel", "mode_channel",
                 "throttle_min", "throttle_max", "throttle_trim",
                 "steering_min", "steering_max", "steering_trim",
                 "throttle_reversed", "steering_reversed",
                 "mode_min", "mode_max", "manual_slot_mask"]
        return dict(zip(names, (int(v) for v in fields[1:])))

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=10)


@pytest.fixture
def pi_link(build):
    harness = Harness(build.pi_link)
    yield harness
    harness.close()


CONFIG = dict(
    throttle_channel=3, steering_channel=1, arm_channel=6, mode_channel=7,
    throttle_min=200, throttle_max=1800, throttle_trim=1000,
    steering_min=210, steering_max=1790, steering_trim=990,
    mode_min=180, mode_max=1805, manual_slot_mask=0b001001,
    throttle_reversed=True, steering_reversed=False,
)


def bare_link() -> tuple[NucleoLink, list[bytes]]:
    """A NucleoLink with no serial port, capturing what it would write."""
    written: list[bytes] = []
    link = NucleoLink.__new__(NucleoLink)
    link._serial = SimpleNamespace(write=lambda data: written.append(bytes(data)))
    link._write_lock = threading.Lock()
    link.telemetry = RcTelemetry()
    link._imu_queue = deque(maxlen=256)
    return link, written


def test_defaults_before_any_config(pi_link):
    assert pi_link.rc_map() == dict(
        throttle_channel=2, steering_channel=0, arm_channel=5, mode_channel=4,
        throttle_min=172, throttle_max=1811, throttle_trim=992,
        steering_min=172, steering_max=1811, steering_trim=992,
        throttle_reversed=0, steering_reversed=0,
        mode_min=172, mode_max=1811, manual_slot_mask=0b000111,
    )


def test_python_config_frame_is_decoded_field_for_field_by_c(pi_link):
    frame = NucleoLink.build_rc_map_frame(**CONFIG)
    pi_link.rx(frame)
    pi_link.apply_pending()
    assert pi_link.rc_map() == {**CONFIG, "throttle_reversed": 1, "steering_reversed": 0}


def test_rx_isr_only_stashes_the_config_main_loop_applies_it(pi_link):
    """The control loop must never see a half-applied map: the ISR writes a
    pending copy, only PiLink_Process() (main loop) swaps it in."""
    before = pi_link.rc_map()
    pi_link.rx(NucleoLink.build_rc_map_frame(**CONFIG))
    assert pi_link.rc_map() == before
    pi_link.apply_pending()
    assert pi_link.rc_map() != before


def test_config_survives_garbage_and_other_frames_around_it(pi_link):
    link, written = bare_link()
    link.send_motors(1400, 1600)
    motor_frame = written[0]
    config = NucleoLink.build_rc_map_frame(**CONFIG)
    pi_link.rx(bytes([0x00, 0xFF, 0x13]) + motor_frame + bytes([0xC5, 0x00]) + config + motor_frame)
    pi_link.apply_pending()
    assert pi_link.rc_map()["mode_channel"] == CONFIG["mode_channel"]
    assert pi_link.cmd("motors") == "motors 1 1400 1600"


def test_corrupt_checksum_leaves_the_map_untouched(pi_link):
    frame = bytearray(NucleoLink.build_rc_map_frame(**CONFIG))
    frame[-1] ^= 0xFF
    before = pi_link.rc_map()
    pi_link.rx(bytes(frame))
    pi_link.apply_pending()
    assert pi_link.rc_map() == before


@pytest.mark.parametrize("field", ["throttle_channel", "steering_channel", "arm_channel",
                                    "mode_channel"])
def test_out_of_range_channel_index_is_rejected(pi_link, field):
    """An index past SBUS's 16 channels would read out of bounds in the
    control loop; the checksum can be perfectly valid and still be nonsense."""
    before = pi_link.rc_map()
    pi_link.rx(NucleoLink.build_rc_map_frame(**{**CONFIG, field: 16}))
    pi_link.apply_pending()
    assert pi_link.rc_map() == before


def test_manual_mask_is_limited_to_six_bands(pi_link):
    frame = bytearray(NucleoLink.build_rc_map_frame(**CONFIG))
    frame[23] = 0xFF
    frame[24] = 0
    for b in frame[:24]:
        frame[24] ^= b
    pi_link.rx(bytes(frame))
    pi_link.apply_pending()
    assert pi_link.rc_map()["manual_slot_mask"] == 0x3F


def test_config_is_saved_to_flash_only_when_safe(pi_link):
    pi_link.rx(NucleoLink.build_rc_map_frame(**CONFIG))
    assert pi_link.saves(armed=False) == 0  # debounce not elapsed yet

    pi_link.tick(1500)
    assert pi_link.saves(armed=True) == 0  # armed: a flash erase now could glitch the loop
    assert pi_link.saves(armed=False) == 1
    pi_link.tick(5000)
    assert pi_link.saves(armed=False) == 1  # saved once, not on every tick


def test_a_burst_of_config_frames_costs_one_flash_write(pi_link):
    """RadioPage calibration writes dozens of parameters back to back."""
    for i in range(10):
        pi_link.rx(NucleoLink.build_rc_map_frame(**{**CONFIG, "throttle_min": 100 + i}))
        pi_link.tick(50)
        assert pi_link.saves(armed=False) == 0  # applied, not yet persisted
    pi_link.tick(1200)
    assert pi_link.saves(armed=False) == 1
    assert pi_link.rc_map()["throttle_min"] == 109  # the last one won


def test_telemetry_frame_built_by_c_is_decoded_by_python(pi_link):
    for index, value in enumerate(range(172, 172 + 16)):
        assert pi_link.cmd(f"chan {index} {value}") == "ok"
    reply = pi_link.cmd("tx 1 1 1 1 12345 1 -1534 820 6 67")  # arm info 0x43: block 4, state 3
    assert reply.startswith("tx ")
    frame = bytes.fromhex(reply.split()[1])
    assert len(frame) == 44

    link, _ = bare_link()
    link._decode_telemetry(frame)
    t = link.telemetry
    assert t.last_update > 0  # checksum accepted
    assert t.channels == list(range(172, 172 + 16))
    assert (t.armed, t.rc_link_up, t.override_active) == (True, True, True)
    assert (t.battery_valid, t.imu_valid) == (True, True)
    assert t.battery_mv == 12345
    assert t.roll_deg == pytest.approx(-15.34)
    assert t.pitch_deg == pytest.approx(8.20)
    assert t.mode_slot == 6
    assert (t.arm_state, t.arm_block) == (3, 4)


def test_telemetry_validity_bits_and_negative_angles(pi_link):
    reply = pi_link.cmd("tx 0 0 0 0 0 0 -32768 32767 0")
    link, _ = bare_link()
    link._decode_telemetry(bytes.fromhex(reply.split()[1]))
    t = link.telemetry
    assert (t.armed, t.rc_link_up, t.override_active) == (False, False, False)
    assert (t.battery_valid, t.imu_valid) == (False, False)
    assert t.roll_deg == pytest.approx(-327.68)
    assert t.pitch_deg == pytest.approx(327.67)
    assert t.mode_slot == 0


def test_motor_frame_from_python_makes_the_pi_link_fresh(pi_link):
    assert pi_link.cmd("motors") == "motors 0 1500 1500"  # nothing heard yet
    link, written = bare_link()
    link.send_motors(1234, 1801)
    pi_link.rx(written[0])
    assert pi_link.cmd("motors") == "motors 1 1234 1801"
    pi_link.tick(501)  # PI_LINK_TIMEOUT_MS
    assert pi_link.cmd("motors").startswith("motors 0 ")


def test_a_sync_byte_right_before_a_real_frame_does_not_cost_the_frame(pi_link):
    """The receiver used to go back to hunting on a failed second sync byte,
    throwing away a first sync byte that was really the start of the next
    frame: A5 A5 5A ... lost the whole motor frame."""
    link, written = bare_link()
    link.send_motors(1700, 1300)
    pi_link.rx(bytes([0xA5]) + written[0])
    assert pi_link.cmd("motors") == "motors 1 1700 1300"

    pi_link.tick(600)                     # let it go stale again
    pi_link.rx(bytes([0xC5]) + NucleoLink.build_rc_map_frame(**CONFIG))
    pi_link.apply_pending()
    assert pi_link.rc_map()["mode_channel"] == CONFIG["mode_channel"]
