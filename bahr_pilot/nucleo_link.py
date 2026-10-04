"""Python side of firmware/reflex/Core/Src/pi_link.c's packet protocol —
now bidirectional (extended 2026-10-01 for configurable RC channel mapping,
see pi_link.h for the authoritative frame layout both sides must agree on).

Pi -> Nucleo, motor command, 7 bytes: [0xA5][0x5A][motor1_us hi/lo]
[motor2_us hi/lo][XOR checksum]. Absolute microseconds, big-endian. The
Nucleo treats a gap of more than 500 ms between frames as "Pi link down"
(firmware/reflex/Core/Src/failsafe.c) — send periodically, not just on
change.

Pi -> Nucleo, RC map config, 25 bytes: [0xC5][0x5C][throttle_ch][steering_ch]
[arm_ch][mode_ch][throttle_min][throttle_max][throttle_trim]
[steering_min][steering_max][steering_trim][flags][mode_min][mode_max]
[manual_slot_mask][XOR checksum]. Channel indices are 0-based SBUS indices;
min/max/trim are raw SBUS values (same units BAHR-GCS's RCn_MIN/MAX/TRIM
parameters already use). flags bit0 = throttle reversed, bit1 = steering
reversed. manual_slot_mask bit n = mode-switch band n+1 is MANUAL (the
Pi's MODEn parameter equals MANUAL), i.e. the Nucleo drives the motors
from the sticks itself while the switch is in that band. Sent once
whenever the mapping changes, not periodically.

Nucleo -> Pi, RC telemetry, 44 bytes (36 originally; extended 2026-10-01 for
battery/IMU and 2026-10-02 for the mode switch and arm state): [0xE5][0x5E]
[16 x channel uint16][status][battery_mv uint16][roll_cdeg int16]
[pitch_cdeg int16][mode_slot][arm_info][XOR checksum]. status bit0=armed,
bit1=rc_link_up, bit2=override_active (mode switch in a MANUAL band),
bit3=pi_link_fresh, bit4=battery_valid, bit5=imu_valid. battery_mv/
roll_cdeg/pitch_cdeg are only meaningful when their validity bit is set —
see firmware/reflex/Core/Src/battery.c and imu.c for where these actually
come from (and their own hardware-unverified caveats). mode_slot is the
mode-switch band 1..6, or 0 when there is no valid RC signal. arm_info:
bits 0-2 = arm state (0 BOOT, 1 DISARMED, 2 READY, 3 ARMED), bits 4-7 =
what is blocking arming (ARM_BLOCK_*, firmware/reflex/Core/Inc/arming.h).

Nucleo -> Pi, IMU frame, 28 bytes (2026-10-02, ~50 Hz): [0xE6][0x6E]
[t_us uint32 STM clock][qx qy qz qw int16 Q14][gx gy gz int16 Q9 rad/s]
[ax ay az int16 Q8 m/s^2][flags][XOR checksum]. flags bit0/1/2 = quaternion/
gyro/accel valid, bits 4-5 = quaternion accuracy. t_us is mapped onto the
Pi's clock by timesync.StmClock.
"""
from __future__ import annotations

import struct
import threading
import time
from collections import deque
from dataclasses import dataclass

import serial

from bahr_pilot.timesync import StmClock

PULSE_MIN_US = 1000
PULSE_NEUTRAL_US = 1500
PULSE_MAX_US = 2000

_MOTOR_SYNC = bytes([0xA5, 0x5A])
_CONFIG_SYNC = bytes([0xC5, 0x5C])
_TELEMETRY_SYNC = bytes([0xE5, 0x5E])
_TELEMETRY_FRAME_LEN = 44
_IMU_SYNC = bytes([0xE6, 0x6E])
_IMU_FRAME_LEN = 28
_INCOMING = ((_TELEMETRY_SYNC, _TELEMETRY_FRAME_LEN), (_IMU_SYNC, _IMU_FRAME_LEN))

ARM_STATE_BOOT, ARM_STATE_DISARMED, ARM_STATE_READY, ARM_STATE_ARMED = 0, 1, 2, 3
(ARM_BLOCK_NONE, ARM_BLOCK_NO_RC, ARM_BLOCK_BOOT_GRACE, ARM_BLOCK_SWITCH_ON,
 ARM_BLOCK_THROTTLE, ARM_BLOCK_IMU, ARM_BLOCK_BATTERY) = range(7)
_NUM_CHANNELS = 16


def _clamp(pulse_us: int) -> int:
    return max(PULSE_MIN_US, min(PULSE_MAX_US, int(pulse_us)))


def _xor_checksum(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def extract_frames(buf: bytearray) -> list[bytes]:
    """Removes and returns every complete, checksum-valid incoming frame at the
    front of `buf`, leaving an incomplete trailing frame in place.

    Garbage and false syncs are skipped one byte at a time: a failed checksum
    must cost one byte, not a whole frame's worth — otherwise a stray sync
    pair inside a payload would swallow the start of the real frame behind
    it."""
    frames: list[bytes] = []
    while True:
        best = None
        for sync, length in _INCOMING:
            idx = buf.find(sync)
            if idx >= 0 and (best is None or idx < best[0]):
                best = (idx, length)
        if best is None:
            del buf[:-1]  # keep only a possible first sync byte
            return frames
        idx, length = best
        del buf[:idx]
        if len(buf) < length:
            return frames
        frame = bytes(buf[:length])
        if _xor_checksum(frame[:-1]) == frame[-1]:
            del buf[:length]
            frames.append(frame)
        else:
            del buf[:1]


@dataclass
class ImuData:
    """One IMU frame. Orientation and rates are in the SENSOR frame; how the
    BNO086 is mounted in the boat is unverified (no hardware yet)."""
    t_stm_us: int                 # raw 32-bit STM clock at the newest report
    t_pi: float                   # the same instant on the Pi's monotonic clock (estimated)
    rx_time: float                # when the frame arrived (Pi monotonic)
    quat: tuple[float, float, float, float]   # x, y, z, w
    gyro: tuple[float, float, float]          # rad/s
    accel: tuple[float, float, float]         # m/s^2, gravity removed
    quat_valid: bool
    gyro_valid: bool
    accel_valid: bool
    quat_accuracy: int            # 0 unreliable .. 3 high


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
        self.mode_slot = 0  # 1..6 mode-switch band, 0 = no valid RC signal
        self.arm_state = ARM_STATE_BOOT
        self.arm_block = ARM_BLOCK_NO_RC
        self.last_update: float = 0.0   # Pi MONOTONIC seconds of the last accepted frame; 0.0 = none yet


class NucleoLink:
    def __init__(self, port: str, baud: int = 115200) -> None:
        self._serial = serial.Serial(port, baud, timeout=0.1)
        self._write_lock = threading.Lock()
        self.telemetry = RcTelemetry()
        self.imu: ImuData | None = None
        # Every decoded IMU sample, for the estimator: self.imu only keeps the
        # newest, and a gyro integrator must see all of them. A deque's append
        # and popleft are thread-safe; maxlen drops the OLDEST if the main
        # loop stalls (256 samples = 5 s at 50 Hz).
        self._imu_queue: deque[ImuData] = deque(maxlen=256)
        self.clock = StmClock()
        self.imu_frames = 0
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

    @staticmethod
    def build_rc_map_frame(*, throttle_channel: int, steering_channel: int,
                            arm_channel: int, mode_channel: int,
                            throttle_min: int, throttle_max: int, throttle_trim: int,
                            steering_min: int, steering_max: int, steering_trim: int,
                            mode_min: int, mode_max: int, manual_slot_mask: int,
                            throttle_reversed: bool = False,
                            steering_reversed: bool = False) -> bytes:
        flags = (1 if throttle_reversed else 0) | (2 if steering_reversed else 0)
        frame = bytearray(_CONFIG_SYNC)
        frame += bytes([throttle_channel, steering_channel, arm_channel, mode_channel])
        frame += struct.pack(">HHHHHH", throttle_min, throttle_max, throttle_trim,
                              steering_min, steering_max, steering_trim)
        frame.append(flags)
        frame += struct.pack(">HH", mode_min, mode_max)
        frame.append(manual_slot_mask & 0x3F)
        frame.append(_xor_checksum(frame))
        return bytes(frame)

    def send_rc_map(self, **config) -> None:
        frame = self.build_rc_map_frame(**config)
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
                # Blocks for the first byte (up to the port timeout), then takes
                # whatever else has arrived. read(64) would instead sit waiting
                # for 64 bytes, delaying frames by tens of milliseconds and
                # lumping their arrival times together — ruinous for timesync.
                chunk = self._serial.read(max(1, self._serial.in_waiting))
            except (serial.SerialException, OSError):
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            rx_time = time.monotonic()
            buf += chunk
            for frame in extract_frames(buf):
                if frame[0] == _TELEMETRY_SYNC[0]:
                    self._decode_telemetry(frame)
                else:
                    self._decode_imu(frame, rx_time)

    def _decode_imu(self, frame: bytes, rx_time: float) -> None:
        if _xor_checksum(frame[:-1]) != frame[-1]:
            return
        t_us, qx, qy, qz, qw, gx, gy, gz, ax, ay, az, flags = struct.unpack(">I4h3h3hB", frame[2:27])
        self.imu = ImuData(
            t_stm_us=t_us,
            t_pi=self.clock.update(t_us, rx_time),
            rx_time=rx_time,
            quat=(qx / 16384.0, qy / 16384.0, qz / 16384.0, qw / 16384.0),
            gyro=(gx / 512.0, gy / 512.0, gz / 512.0),
            accel=(ax / 256.0, ay / 256.0, az / 256.0),
            quat_valid=bool(flags & 0x01),
            gyro_valid=bool(flags & 0x02),
            accel_valid=bool(flags & 0x04),
            quat_accuracy=(flags >> 4) & 0x03,
        )
        self._imu_queue.append(self.imu)
        self.imu_frames += 1

    def take_imu_samples(self) -> list[ImuData]:
        """Everything decoded since the last call, oldest first."""
        samples = []
        while True:
            try:
                samples.append(self._imu_queue.popleft())
            except IndexError:
                return samples

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
        t.mode_slot = frame[41]
        t.arm_state = frame[42] & 0x07
        t.arm_block = frame[42] >> 4
        t.roll_deg = roll_cdeg / 100.0
        t.pitch_deg = pitch_cdeg / 100.0
        t.last_update = time.monotonic()
