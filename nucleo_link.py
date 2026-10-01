"""Python side of bahr_pilot/firmware/reflex/Core/Src/pi_link.c's packet protocol —
now bidirectional (extended 2026-10-01 for configurable RC channel mapping,
see pi_link.h for the authoritative frame layout both sides must agree on).

Pi -> Nucleo, motor command, 7 bytes: [0xA5][0x5A][motor1_us hi/lo]
[motor2_us hi/lo][XOR checksum]. Absolute microseconds, big-endian. The
Nucleo treats a gap of more than 500 ms between frames as "Pi link down"
(bahr_pilot/firmware/reflex/Core/Src/failsafe.c) — send periodically, not just on
change.

Pi -> Nucleo, RC map config, 20 bytes: [0xC5][0x5C][throttle_ch][steering_ch]
[arm_ch][override_ch][throttle_min][throttle_max][throttle_trim]
[steering_min][steering_max][steering_trim][flags][XOR checksum]. Channel
indices are 0-based SBUS indices; min/max/trim are raw SBUS values (same
units BAHR-GCS's RCn_MIN/MAX/TRIM parameters already use). flags bit0 =
throttle reversed, bit1 = steering reversed. Sent once whenever the mapping
changes, not periodically.

Nucleo -> Pi, RC telemetry, 42 bytes (extended 2026-10-01 from the original
36 to add battery/IMU): [0xE5][0x5E][16 x channel uint16][status]
[battery_mv uint16][roll_cdeg int16][pitch_cdeg int16][XOR checksum].
status bit0=armed, bit1=rc_link_up, bit2=override_active,
bit3=pi_link_fresh, bit4=battery_valid, bit5=imu_valid. battery_mv/
roll_cdeg/pitch_cdeg are only meaningful when their validity bit is set —
see bahr_pilot/firmware/reflex/Core/Src/battery.c and imu.c for where
these actually come from (and their own hardware-unverified caveats).
"""
from __future__ import annotations

import struct
import threading
import time

import serial

PULSE_MIN_US = 1000
PULSE_NEUTRAL_US = 1500
PULSE_MAX_US = 2000

_MOTOR_SYNC = bytes([0xA5, 0x5A])
_CONFIG_SYNC = bytes([0xC5, 0x5C])
_TELEMETRY_SYNC = bytes([0xE5, 0x5E])
_TELEMETRY_FRAME_LEN = 42
_NUM_CHANNELS = 16


def _clamp(pulse_us: int) -> int:
    return max(PULSE_MIN_US, min(PULSE_MAX_US, int(pulse_us)))


def _xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


class RcTelemetry:
    def __init__(self) -> None:
        self.channels = [992] * _NUM_CHANNELS  # SBUS_CH_MID default
        self.armed = False
        self.rc_link_up = False
        self.override_active = False
        self.pi_link_fresh = False
        self.battery_valid = False
        self.battery_mv = 0
        self.imu_valid = False
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.last_update: float = 0.0


class NucleoLink:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._serial = serial.Serial(port, baud, timeout=0.1)
        self._write_lock = threading.Lock()
        self.telemetry = RcTelemetry()
        self._stop = False
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def send_motors(self, motor1_us: int, motor2_us: int) -> None:
        m1 = _clamp(motor1_us)
        m2 = _clamp(motor2_us)
        frame = bytearray(_MOTOR_SYNC)
        frame += struct.pack(">HH", m1, m2)
        frame.append(_xor_checksum(frame))
        with self._write_lock:
            self._serial.write(frame)

    def stop(self) -> None:
        self.send_motors(PULSE_NEUTRAL_US, PULSE_NEUTRAL_US)

    def send_rc_map(self, *, throttle_channel: int, steering_channel: int,
                     arm_channel: int, override_channel: int,
                     throttle_min: int, throttle_max: int, throttle_trim: int,
                     steering_min: int, steering_max: int, steering_trim: int,
                     throttle_reversed: bool = False, steering_reversed: bool = False) -> None:
        flags = (1 if throttle_reversed else 0) | (2 if steering_reversed else 0)
        frame = bytearray(_CONFIG_SYNC)
        frame += bytes([throttle_channel, steering_channel, arm_channel, override_channel])
        frame += struct.pack(">HHHHHH", throttle_min, throttle_max, throttle_trim,
                              steering_min, steering_max, steering_trim)
        frame.append(flags)
        frame.append(_xor_checksum(frame))
        with self._write_lock:
            self._serial.write(frame)

    def close(self) -> None:
        self._stop = True
        self._thread.join(timeout=1.0)
        self._serial.close()

    def _read_loop(self) -> None:
        buf = bytearray()
        while not self._stop:
            try:
                chunk = self._serial.read(64)
            except (serial.SerialException, OSError):
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while True:
                idx = buf.find(_TELEMETRY_SYNC)
                if idx < 0:
                    if len(buf) > 2:
                        del buf[:-1]  # keep only a possible partial sync byte
                    break
                del buf[:idx]
                if len(buf) < _TELEMETRY_FRAME_LEN:
                    break
                frame = bytes(buf[:_TELEMETRY_FRAME_LEN])
                del buf[:_TELEMETRY_FRAME_LEN]
                self._decode_telemetry(frame)

    def _decode_telemetry(self, frame: bytes) -> None:
        if _xor_checksum(frame[:-1]) != frame[-1]:
            return
        channels = list(struct.unpack(">16H", frame[2:34]))
        status = frame[34]
        battery_mv, roll_cdeg, pitch_cdeg = struct.unpack(">Hhh", frame[35:41])
        t = self.telemetry
        t.channels = channels
        t.armed = bool(status & 0x01)
        t.rc_link_up = bool(status & 0x02)
        t.override_active = bool(status & 0x04)
        t.pi_link_fresh = bool(status & 0x08)
        t.battery_valid = bool(status & 0x10)
        t.imu_valid = bool(status & 0x20)
        t.battery_mv = battery_mv
        t.roll_deg = roll_cdeg / 100.0
        t.pitch_deg = pitch_cdeg / 100.0
        t.last_update = time.time()
