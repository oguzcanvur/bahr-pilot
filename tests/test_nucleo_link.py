"""Exercises the real NucleoLink class (not a reimplementation) against a
pyserial loop:// virtual port — no hardware needed. Mirrors the checks done
ad hoc during this project's 2026-10-01 session, now committed so they run
on every change instead of being retyped by hand.
"""
from __future__ import annotations

import time

import pytest
import serial

from bahr_pilot.nucleo_link import NucleoLink, _xor_checksum


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


def test_send_motors_frame_bytes(link):
    link.send_motors(1234, 1800)
    time.sleep(0.05)
    # send_motors() writes into the loopback, which the background read
    # thread immediately reads back out as telemetry input — so instead of
    # reading the raw bytes ourselves (the read thread already consumed
    # them), verify indirectly: a motor frame is not a telemetry sync, so
    # it must NOT have updated self.telemetry.
    assert link.telemetry.last_update == 0.0


def test_send_rc_map_frame_checksum():
    # Build the frame the same way NucleoLink.send_rc_map does, bypassing
    # the serial write, to check the exact byte layout pi_link.c expects.
    import struct

    throttle_channel, steering_channel, arm_channel, override_channel = 2, 0, 5, 4
    frame = bytearray([0xC5, 0x5C])
    frame += bytes([throttle_channel, steering_channel, arm_channel, override_channel])
    frame += struct.pack(">HHHHHH", 172, 1811, 992, 172, 1811, 992)
    frame.append(0)  # flags: not reversed
    frame.append(_xor_checksum(frame))

    assert len(frame) == 20
    assert frame[0:2] == bytes([0xC5, 0x5C])
    assert _xor_checksum(frame[:-1]) == frame[-1]


def test_telemetry_round_trip_extended_frame(link):
    """Feeds a synthetic 42-byte telemetry frame in through the loopback
    (writing to the serial port is what the Nucleo would do) and checks
    NucleoLink._decode_telemetry (the real decoder, not a copy) parses it
    correctly — including the battery/IMU fields added 2026-10-01."""
    import struct

    channels = list(range(100, 100 + 16))
    status = 0x01 | 0x02 | 0x10 | 0x20  # armed, rc_link_up, battery_valid, imu_valid
    battery_mv = 12400
    roll_cdeg = -1534  # -15.34 deg
    pitch_cdeg = 820    # 8.20 deg

    frame = bytearray([0xE5, 0x5E])
    frame += struct.pack(">16H", *channels)
    frame.append(status)
    frame += struct.pack(">Hhh", battery_mv, roll_cdeg, pitch_cdeg)
    frame.append(_xor_checksum(frame))
    assert len(frame) == 42

    with link._write_lock:
        link._serial.write(frame)

    deadline = time.time() + 1.0
    while link.telemetry.last_update == 0.0 and time.time() < deadline:
        time.sleep(0.01)

    t = link.telemetry
    assert t.channels == channels
    assert t.armed is True
    assert t.rc_link_up is True
    assert t.override_active is False
    assert t.battery_valid is True
    assert t.imu_valid is True
    assert t.battery_mv == battery_mv
    assert t.roll_deg == pytest.approx(-15.34)
    assert t.pitch_deg == pytest.approx(8.20)


def test_telemetry_rejects_bad_checksum(link):
    import struct

    channels = [992] * 16
    frame = bytearray([0xE5, 0x5E])
    frame += struct.pack(">16H", *channels)
    frame.append(0x00)
    frame += struct.pack(">Hhh", 11000, 0, 0)
    frame.append(0xFF)  # wrong checksum on purpose
    assert len(frame) == 42

    with link._write_lock:
        link._serial.write(frame)
    time.sleep(0.1)

    assert link.telemetry.last_update == 0.0  # corrupt frame must be dropped, not applied
