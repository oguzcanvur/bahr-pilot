"""Exercises the real NucleoLink class (not a reimplementation) against a
pyserial loop:// virtual port — no hardware needed.
"""
from __future__ import annotations

import struct
import time

import pytest
import serial

from bahr_pilot.nucleo_link import NucleoLink, _xor_checksum

TELEMETRY_LEN = 44


@pytest.fixture
def link(monkeypatch):
    # serial.Serial is the platform-specific class (serialwin32.Serial on
    # Windows) and doesn't understand "url://"-style ports on its own —
    # only serial.serial_for_url() dispatches those to the loopback/socket/
    # etc. handlers. NucleoLink itself rightly calls plain serial.Serial()
    # (real deployments always pass a real port path), so the swap happens
    # here, test-side only.
    monkeypatch.setattr(serial, "Serial", serial.serial_for_url)
    link = NucleoLink("loop://", baud=115200)
    yield link
    link.close()


def capture_writes(link, monkeypatch) -> list[bytes]:
    written: list[bytes] = []
    monkeypatch.setattr(link._serial, "write", lambda data: written.append(bytes(data)))
    return written


def telemetry_frame(*, channels=None, status=0, battery_mv=0, roll_cdeg=0,
                    pitch_cdeg=0, mode_slot=0, arm_info=0, checksum=None) -> bytes:
    frame = bytearray([0xE5, 0x5E])
    frame += struct.pack(">16H", *(channels or [992] * 16))
    frame.append(status)
    frame += struct.pack(">Hhh", battery_mv, roll_cdeg, pitch_cdeg)
    frame.append(mode_slot)
    frame.append(arm_info)
    frame.append(_xor_checksum(frame) if checksum is None else checksum)
    assert len(frame) == TELEMETRY_LEN
    return bytes(frame)


def wait_for_telemetry(link, timeout=1.0):
    deadline = time.time() + timeout
    while link.telemetry.last_update == 0.0 and time.time() < deadline:
        time.sleep(0.01)


RC_MAP = dict(
    throttle_channel=2, steering_channel=0, arm_channel=5, mode_channel=4,
    throttle_min=172, throttle_max=1811, throttle_trim=992,
    steering_min=172, steering_max=1811, steering_trim=992,
    mode_min=172, mode_max=1811, manual_slot_mask=0b000111,
)


def test_telemetry_is_stamped_on_the_monotonic_clock(link):
    """The vehicle compares this stamp with time.monotonic() to decide whether the STM is alive. On
    the wall clock a time sync (NTP, GNSS) would make the STM look dead, or alive, for a moment."""
    link._serial.write(telemetry_frame())
    wait_for_telemetry(link)
    stamp = link.telemetry.last_update
    assert stamp > 0.0 and abs(stamp - time.monotonic()) < 5.0
    assert abs(stamp - time.time()) > 1e6                      # not the wall clock


def test_send_motors_frame_bytes(link, monkeypatch):
    written = capture_writes(link, monkeypatch)
    link.send_motors(1234, 1800)
    (frame,) = written
    assert frame[:2] == bytes([0xA5, 0x5A])
    assert struct.unpack(">HH", frame[2:6]) == (1234, 1800)
    assert len(frame) == 7
    assert _xor_checksum(frame[:-1]) == frame[-1]


def test_send_motors_clamps_to_esc_range(link, monkeypatch):
    written = capture_writes(link, monkeypatch)
    link.send_motors(500, 9000)
    assert struct.unpack(">HH", written[0][2:6]) == (1000, 2000)


def test_rc_map_frame_layout_matches_pi_link_h(link, monkeypatch):
    """Byte offsets here are the ones pi_link.h documents and pi_link.c's
    DecodeConfigFrame reads; if either side changes, this must fail."""
    written = capture_writes(link, monkeypatch)
    link.send_rc_map(**RC_MAP, throttle_reversed=True, steering_reversed=False)
    (frame,) = written

    assert len(frame) == 25
    assert frame[0:2] == bytes([0xC5, 0x5C])
    assert list(frame[2:6]) == [2, 0, 5, 4]  # throttle, steering, arm, mode channels
    assert struct.unpack(">HHHHHH", frame[6:18]) == (172, 1811, 992, 172, 1811, 992)
    assert frame[18] == 0b01  # flags: throttle reversed only
    assert struct.unpack(">HH", frame[19:23]) == (172, 1811)  # mode_min, mode_max
    assert frame[23] == 0b000111  # manual_slot_mask
    assert _xor_checksum(frame[:-1]) == frame[-1]


def test_rc_map_manual_mask_limited_to_six_bands():
    frame = NucleoLink.build_rc_map_frame(**{**RC_MAP, "manual_slot_mask": 0xFF})
    assert frame[23] == 0x3F


def test_telemetry_round_trip_extended_frame(link):
    """Feeds a synthetic 44-byte telemetry frame in through the loopback
    (writing to the serial port is what the Nucleo would do) and checks
    NucleoLink._decode_telemetry (the real decoder, not a copy) parses it
    correctly — battery/IMU (2026-10-01), mode-switch slot and arm state
    (2026-10-02)."""
    channels = list(range(100, 116))
    status = 0x01 | 0x02 | 0x04 | 0x10 | 0x20  # armed, rc up, override, battery, imu
    frame = telemetry_frame(channels=channels, status=status, battery_mv=12400,
                            roll_cdeg=-1534, pitch_cdeg=820, mode_slot=4,
                            arm_info=(4 << 4) | 1)  # block THROTTLE, state DISARMED

    with link._write_lock:
        link._serial.write(frame)
    wait_for_telemetry(link)

    t = link.telemetry
    assert t.channels == channels
    assert t.armed is True
    assert t.rc_link_up is True
    assert t.override_active is True
    assert t.battery_valid is True
    assert t.imu_valid is True
    assert t.battery_mv == 12400
    assert t.roll_deg == pytest.approx(-15.34)
    assert t.pitch_deg == pytest.approx(8.20)
    assert t.mode_slot == 4
    assert t.arm_state == 1
    assert t.arm_block == 4


def test_telemetry_rejects_bad_checksum(link):
    frame = telemetry_frame(battery_mv=11000, mode_slot=6, checksum=0xFF)
    with link._write_lock:
        link._serial.write(frame)
    time.sleep(0.1)
    assert link.telemetry.last_update == 0.0  # corrupt frame must be dropped
    assert link.telemetry.mode_slot == 0


@pytest.mark.parametrize("with_mode_slot", [False, True])
def test_old_frame_layouts_are_not_mistaken_for_current(link, with_mode_slot):
    """Stale firmware still sending a previous layout (42 B without the mode
    slot, 43 B without arm info) must not be decoded as garbage — its
    checksum lands in the wrong slot and fails."""
    old = bytearray([0xE5, 0x5E])
    old += struct.pack(">16H", *([992] * 16))
    old.append(0x03)
    old += struct.pack(">Hhh", 12000, 0, 0)
    if with_mode_slot:
        old.append(4)
    old.append(_xor_checksum(old))
    assert len(old) == (43 if with_mode_slot else 42)
    with link._write_lock:
        link._serial.write(bytes(old))
        link._serial.write(bytes(old))  # the decoder needs 44 bytes to look
    time.sleep(0.1)
    assert link.telemetry.last_update == 0.0
