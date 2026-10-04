"""The IMU frame and the frame splitter, Python side and across the language
boundary (real pi_link.c on the host -> real Python decoder)."""
from __future__ import annotations

import struct
import time

import pytest

from bahr_pilot.nucleo_link import ImuData, NucleoLink, _xor_checksum, extract_frames

from tests.test_c_firmware_logic import Harness, bare_link  # noqa: F401  (fixtures reused below)


def imu_frame(t_us=1000, quat=(0.0, 0.0, 0.0, 1.0), gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 0.0),
              flags=0x07, accuracy=3, checksum=None) -> bytes:
    frame = bytearray([0xE6, 0x6E])
    frame += struct.pack(">I", t_us)
    frame += struct.pack(">4h", *(round(v * 16384) for v in quat))
    frame += struct.pack(">3h", *(round(v * 512) for v in gyro))
    frame += struct.pack(">3h", *(round(v * 256) for v in accel))
    frame.append(flags | (accuracy << 4))
    frame.append(_xor_checksum(frame) if checksum is None else checksum)
    assert len(frame) == 28
    return bytes(frame)


def telemetry_frame(**kwargs) -> bytes:
    frame = bytearray([0xE5, 0x5E])
    frame += struct.pack(">16H", *([992] * 16))
    frame.append(0x03)
    frame += struct.pack(">Hhh", 12000, 0, 0)
    frame.append(kwargs.get("slot", 4))
    frame.append(0)
    frame.append(_xor_checksum(frame))
    assert len(frame) == 44
    return bytes(frame)


# -- extract_frames ---------------------------------------------------------

def test_extracts_back_to_back_frames_of_both_kinds():
    buf = bytearray(telemetry_frame() + imu_frame(1) + imu_frame(2) + telemetry_frame())
    frames = extract_frames(buf)
    assert [f[0] for f in frames] == [0xE5, 0xE6, 0xE6, 0xE5]
    assert len(buf) <= 1


def test_incomplete_trailing_frame_is_kept_for_the_next_read():
    whole = imu_frame(5)
    buf = bytearray(imu_frame(4) + whole[:10])
    assert len(extract_frames(buf)) == 1
    assert bytes(buf) == whole[:10]
    buf += whole[10:]
    assert extract_frames(buf) == [whole]


def test_garbage_between_frames_is_skipped():
    buf = bytearray(b"\x00\x13\xff" + imu_frame(1) + b"\xe5\x00\x77" + telemetry_frame())
    frames = extract_frames(buf)
    assert [f[0] for f in frames] == [0xE6, 0xE5]


def test_false_sync_inside_a_payload_does_not_swallow_the_next_frame():
    """A frame whose payload happens to contain E5 5E: on a checksum failure
    only ONE byte may be dropped, or the real frame right behind would be
    eaten along with it."""
    poisoned = bytes([0xE5, 0x5E]) + bytes(10)             # a sync pair, then junk
    buf = bytearray(poisoned + imu_frame(7) + imu_frame(8))
    frames = extract_frames(buf)
    assert [struct.unpack(">I", f[2:6])[0] for f in frames] == [7, 8]


def test_corrupt_frame_is_dropped_the_next_one_survives():
    bad = bytearray(imu_frame(1))
    bad[10] ^= 0xFF
    frames = extract_frames(bytearray(bytes(bad) + imu_frame(2)))
    assert [struct.unpack(">I", f[2:6])[0] for f in frames] == [2]


def test_every_split_point_yields_the_same_frames():
    stream = telemetry_frame() + imu_frame(1) + imu_frame(2)
    for cut in range(1, len(stream)):
        buf = bytearray()
        got = []
        for chunk in (stream[:cut], stream[cut:]):
            buf += chunk
            got += extract_frames(buf)
        assert len(got) == 3, cut


# -- decoding ----------------------------------------------------------------

@pytest.fixture
def link(monkeypatch):
    import serial

    monkeypatch.setattr(serial, "Serial", serial.serial_for_url)
    link = NucleoLink("loop://", baud=115200)
    yield link
    link.close()


def wait_for_imu(link, count=1, timeout=1.0):
    deadline = time.time() + timeout
    while link.imu_frames < count and time.time() < deadline:
        time.sleep(0.005)


def test_imu_frame_is_decoded_through_the_serial_reader(link):
    frame = imu_frame(t_us=123_456, quat=(0.1, -0.2, 0.3, 0.9), gyro=(1.0, -2.0, 0.5),
                      accel=(0.25, -9.75, 1.0), flags=0x05, accuracy=2)
    with link._write_lock:
        link._serial.write(frame)
    wait_for_imu(link)
    imu = link.imu
    assert isinstance(imu, ImuData)
    assert imu.t_stm_us == 123_456
    assert imu.quat == pytest.approx((0.1, -0.2, 0.3, 0.9), abs=1e-3)
    assert imu.gyro == pytest.approx((1.0, -2.0, 0.5), abs=2e-3)
    assert imu.accel == pytest.approx((0.25, -9.75, 1.0), abs=4e-3)
    assert (imu.quat_valid, imu.gyro_valid, imu.accel_valid) == (True, False, True)
    assert imu.quat_accuracy == 2
    assert imu.rx_time > 0 and imu.t_pi > 0


def test_a_burst_of_mixed_frames_through_the_reader_loses_nothing(link):
    stream = b"".join(imu_frame(i) + (telemetry_frame() if i % 5 == 0 else b"") for i in range(100))
    with link._write_lock:
        link._serial.write(stream)
    wait_for_imu(link, 100, timeout=2.0)
    assert link.imu_frames == 100
    assert link.telemetry.last_update > 0


def test_imu_checksum_failure_is_ignored(link):
    with link._write_lock:
        link._serial.write(imu_frame(1, checksum=0x00))
    time.sleep(0.1)
    assert link.imu is None


def test_imu_times_map_onto_a_steady_pi_clock(link):
    """Feed frames at 50 Hz with a 1 % fast STM clock straight into the
    decoder and check the estimator ends up tracking."""
    for i in range(50 * 20):
        t = i / 50
        link._decode_imu(imu_frame(t_us=int(t * 1.01 * 1e6)), 100.0 + t + 0.004)
    assert link.clock.ready
    assert link.imu.t_pi == pytest.approx(100.0 + 19.98 + 0.004, abs=0.004)


# -- the sample queue the estimator drains ------------------------------------

def _decoder():
    from bahr_pilot.timesync import StmClock
    link, _ = bare_link()
    link.clock = StmClock()
    link.imu_frames = 0
    return link


def test_every_imu_sample_is_queued_in_order_and_drained_once():
    """link.imu only holds the newest sample; a gyro integrator needs them all."""
    link = _decoder()
    for k in range(5):
        link._decode_imu(imu_frame(t_us=1000 * (k + 1), gyro=(0.0, 0.0, 0.1 * k)), 10.0 + 0.02 * k)
    samples = link.take_imu_samples()
    assert [s.t_stm_us for s in samples] == [1000, 2000, 3000, 4000, 5000]
    assert link.take_imu_samples() == []
    assert link.imu.t_stm_us == 5000          # the "latest" view is unchanged


def test_a_stalled_consumer_loses_the_oldest_samples_not_the_newest():
    link = _decoder()
    for k in range(300):
        link._decode_imu(imu_frame(t_us=1000 * (k + 1)), 10.0 + 0.02 * k)
    samples = link.take_imu_samples()
    assert len(samples) == 256
    assert samples[-1].t_stm_us == 300_000 and samples[0].t_stm_us == 45_000


def test_a_corrupt_frame_is_not_queued():
    link = _decoder()
    link._decode_imu(imu_frame(1, checksum=0x00), 1.0)
    assert link.take_imu_samples() == []


# -- across the language boundary: real C pi_link.c -> real Python ------------

@pytest.fixture
def pi_link(build):
    harness = Harness(build.pi_link)
    yield harness
    harness.close()


def test_imu_frame_built_by_c_decodes_to_the_same_numbers(pi_link):
    reply = pi_link.cmd("imu 4000000000 0.1 -0.2 0.3 0.9 1.0 -2.0 0.5 0.25 -9.75 1.0 5 2")
    assert reply.startswith("imu ")
    frame = bytes.fromhex(reply.split()[1])
    assert len(frame) == 28
    link, _ = bare_link()
    link.clock = __import__("bahr_pilot.timesync", fromlist=["StmClock"]).StmClock()
    link.imu_frames = 0
    link._decode_imu(frame, 10.0)
    imu = link.imu
    assert imu.t_stm_us == 4_000_000_000
    assert imu.quat == pytest.approx((0.1, -0.2, 0.3, 0.9), abs=1e-3)
    assert imu.gyro == pytest.approx((1.0, -2.0, 0.5), abs=2e-3)
    assert imu.accel == pytest.approx((0.25, -9.75, 1.0), abs=4e-3)
    assert (imu.quat_valid, imu.gyro_valid, imu.accel_valid) == (True, False, True)
    assert imu.quat_accuracy == 2


def test_c_saturates_out_of_range_values_instead_of_wrapping(pi_link):
    """+-64 rad/s is the Q9 limit, +-128 m/s^2 the Q8 limit. A wrapped
    value would flip sign and read as the opposite rotation."""
    reply = pi_link.cmd("imu 1 0 0 0 1 100.0 -100.0 0 300.0 -300.0 0 7 3")
    frame = bytes.fromhex(reply.split()[1])
    link, _ = bare_link()
    link.clock = __import__("bahr_pilot.timesync", fromlist=["StmClock"]).StmClock()
    link.imu_frames = 0
    link._decode_imu(frame, 1.0)
    assert link.imu.gyro[0] == pytest.approx(32767 / 512, abs=1e-3) and link.imu.gyro[0] > 0
    assert link.imu.gyro[1] < 0
    assert link.imu.accel[0] > 0 and link.imu.accel[1] < 0


def test_ring_overflow_drops_whole_frames_never_half_frames(pi_link):
    """Queue far more than the 256-byte TX ring can hold without letting the
    UART drain: whatever does go out must still parse as complete frames."""
    reply = pi_link.cmd("burst 200")
    stream = bytes.fromhex(reply.split()[1])
    assert 0 < len(stream) < 200 * (44 + 28)          # something was dropped (ring is small) ...
    buf = bytearray(stream)
    frames = extract_frames(buf)
    assert sum(len(f) for f in frames) == len(stream)   # ... but every byte belongs to a whole frame
    assert not buf
    kinds = {f[0] for f in frames}
    assert kinds == {0xE5, 0xE6}


def test_light_traffic_is_never_dropped(pi_link):
    """At the real rates (IMU 50 Hz + telemetry 10 Hz, 1.84 KB/s against an
    11.5 KB/s link) nothing may be lost: frames drained between sends."""
    got = []
    for i in range(50):
        got.append(pi_link.cmd(f"imu {i} 0 0 0 1 0 0 0 0 0 0 7 3"))
        if i % 5 == 0:
            got.append(pi_link.cmd("tx 1 1 0 1 12000 1 0 0 4 0"))
    assert all(len(g.split()[1]) // 2 in (28, 44) for g in got)
    assert len(got) == 60
