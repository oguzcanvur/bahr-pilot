"""Phases 18 and 19: sampling by distance and quality classification
(bahr_pilot/bathymetry.py)."""
from __future__ import annotations

import math

import pytest

from bahr_pilot import geo
from bahr_pilot.bathymetry import (
    BathyConfig, BathymetryRecorder, SampleQuality, _tilt_deg, summarize,
)
from bahr_pilot.estimator import EstimateStatus
from bahr_pilot.sonar import DepthQuality, DepthReading
from tests.poses import make_pose

FRAME = geo.LocalFrame(41.0, 29.0)
VALID, LOW, INVALID = SampleQuality.VALID, SampleQuality.LOW_QUALITY, SampleQuality.INVALID


def reading(t=0.0, raw=8.0, quality=DepthQuality.GOOD, reason="", offset=0.0):
    depth = None if quality == DepthQuality.BAD else raw + offset
    return DepthReading(t=t, raw_m=raw, depth_m=depth, quality=quality, reason=reason)


def pose_at(east, north, speed=1.5, **kwargs):
    lat, lon = FRAME.to_geodetic(east, north)
    return make_pose(lat, lon, speed_mps=speed, velocity_en=kwargs.pop("velocity_en", (0.0, speed)), **kwargs)


def classify(read=None, pose="ok", cfg=None):
    recorder = BathymetryRecorder(cfg)
    return recorder.classify(read or reading(), pose_at(0.0, 0.0) if pose == "ok" else pose)


# -- quality (Phase 19) ------------------------------------------------------------------------------

def test_a_good_reading_with_a_good_position_is_valid():
    assert classify() == (VALID, ())


def test_a_suspect_reading_makes_a_low_quality_sample_and_says_why():
    quality, reasons = classify(reading(quality=DepthQuality.SUSPECT, reason="noisy: sigma 0.60 m"))
    assert quality == LOW and reasons == ("sonar suspect: noisy: sigma 0.60 m",)


def test_a_rejected_reading_is_invalid():
    quality, reasons = classify(reading(raw=0.0, quality=DepthQuality.BAD, reason="below minimum depth 0.5 m"))
    assert quality == INVALID and reasons == ("sonar: below minimum depth 0.5 m",)


@pytest.mark.parametrize("pose", [None, make_pose(None, None), make_pose(41.0, 29.0, status=EstimateStatus.NONE)])
def test_no_position_is_invalid(pose):
    quality, reasons = classify(pose=pose)
    assert quality == INVALID and "no position" in reasons


def test_a_dead_reckoned_position_is_low_quality():
    quality, reasons = classify(pose=pose_at(0, 0, status=EstimateStatus.DEAD_RECKONING))
    assert quality == LOW and reasons == ("position dead-reckoned",)


@pytest.mark.parametrize("sigma,expected", [(0.05, VALID), (0.5, VALID), (0.51, LOW), (2.5, LOW), (None, LOW)])
def test_the_position_accuracy_threshold(sigma, expected):
    assert classify(pose=pose_at(0, 0, sigma_m=sigma))[0] == expected


def test_the_accuracy_reason_names_the_numbers():
    assert classify(pose=pose_at(0, 0, sigma_m=1.2))[1] == ("position accuracy 1.20 m > 0.5 m",)
    assert classify(pose=pose_at(0, 0, sigma_m=None))[1] == ("position accuracy unknown",)


@pytest.mark.parametrize("speed,expected", [(1.5, VALID), (3.0, VALID), (3.1, LOW), (6.0, LOW)])
def test_the_speed_threshold(speed, expected):
    assert classify(pose=pose_at(0, 0, speed=speed))[0] == expected


@pytest.mark.parametrize("roll,pitch,expected", [
    (0.0, 0.0, VALID), (10.0, 5.0, VALID), (15.0, 0.0, VALID), (16.0, 0.0, LOW),
    (12.0, 10.0, LOW),                       # each below 15 but the combined tilt is 15.5 degrees
    (None, None, VALID),                     # attitude unknown: nothing to judge by
    (20.0, None, VALID),
])
def test_the_tilt_threshold(roll, pitch, expected):
    assert classify(pose=pose_at(0, 0, roll_deg=roll, pitch_deg=pitch))[0] == expected


def test_tilt_is_the_angle_from_the_vertical():
    assert _tilt_deg(make_pose(roll_deg=30.0, pitch_deg=0.0)) == pytest.approx(30.0)
    assert _tilt_deg(make_pose(roll_deg=0.0, pitch_deg=-20.0)) == pytest.approx(20.0)
    assert _tilt_deg(make_pose(roll_deg=12.0, pitch_deg=10.0)) == pytest.approx(
        math.degrees(math.acos(math.cos(math.radians(12)) * math.cos(math.radians(10)))))
    assert _tilt_deg(make_pose(roll_deg=0.0, pitch_deg=0.0)) == 0.0
    assert _tilt_deg(make_pose()) is None


def test_every_reason_is_recorded_not_just_the_first():
    quality, reasons = classify(reading(quality=DepthQuality.SUSPECT, reason="x"),
                                pose_at(0, 0, speed=4.0, sigma_m=1.0, roll_deg=25.0, pitch_deg=0.0,
                                        status=EstimateStatus.DEAD_RECKONING))
    assert quality == LOW and len(reasons) == 5


def test_invalid_wins_but_the_other_doubts_are_kept():
    quality, reasons = classify(reading(quality=DepthQuality.BAD, reason="spike"), pose_at(0, 0, speed=4.0))
    assert quality == INVALID and reasons[0] == "sonar: spike" and any("speed" in r for r in reasons)


def test_thresholds_are_configurable():
    strict = BathyConfig(max_h_acc_m=0.02, max_speed_mps=1.0, max_tilt_deg=5.0)
    assert classify(pose=pose_at(0, 0, sigma_m=0.05), cfg=strict)[0] == LOW
    assert classify(pose=pose_at(0, 0, speed=1.2, sigma_m=0.01), cfg=strict)[0] == LOW
    assert classify(pose=pose_at(0, 0, speed=0.8, sigma_m=0.01, roll_deg=6.0, pitch_deg=0.0), cfg=strict)[0] == LOW
    assert classify(pose=pose_at(0, 0, speed=0.8, sigma_m=0.01, roll_deg=4.0, pitch_deg=0.0), cfg=strict)[0] == VALID


# -- sampling (Phase 18) -----------------------------------------------------------------------------------

def run_track(cfg, seconds, speed=1.5, **kw):
    """A boat running due north at `speed`, one reading per second."""
    recorder = BathymetryRecorder(cfg)
    samples = []
    for k in range(seconds):
        s = recorder.update(reading(t=float(k), **kw), pose_at(0.0, speed * k, speed=speed), float(k) + 0.01, 1e9 + k)
        if s is not None:
            samples.append(s)
    return recorder, samples


def test_one_sample_per_reading_when_the_readings_are_further_apart_than_the_spacing():
    _, samples = run_track(BathyConfig(spacing_m=1.0), 10)             # readings every 1.5 m
    assert len(samples) == 10 and all(s.quality == VALID for s in samples)


def test_samples_are_emitted_per_distance_not_per_second():
    """Spacing 5 m at 1.5 m/s: positions 0, 6.0 (the 4th reading is the first >= 5 m on), 12.0, ..."""
    _, samples = run_track(BathyConfig(spacing_m=5.0), 20)
    assert [round(s.along_track_m, 1) for s in samples] == [0.0, 6.0, 6.0, 6.0, 6.0]
    assert [s.seq for s in samples] == [1, 2, 3, 4, 5]


def test_the_first_sample_is_emitted_whatever_the_spacing():
    _, samples = run_track(BathyConfig(spacing_m=100.0), 3)
    assert len(samples) == 1 and samples[0].along_track_m == 0.0


def test_a_stationary_boat_records_nothing_not_even_rejected_readings():
    recorder = BathymetryRecorder(BathyConfig())
    for k in range(10):
        assert recorder.update(reading(t=float(k)), pose_at(0.0, 0.0, speed=0.1), float(k), 1e9) is None
        assert recorder.update(reading(t=float(k), quality=DepthQuality.BAD, reason="spike"),
                               pose_at(0.0, 0.0, speed=0.1), float(k), 1e9) is None
    assert recorder.skipped_stationary == 20 and recorder.counts == {INVALID: 0, LOW: 0, VALID: 0}


def test_a_crawling_boat_does_not_pile_samples_on_one_spot():
    recorder = BathymetryRecorder(BathyConfig(spacing_m=1.0))
    emitted = 0
    for k in range(100):                                                # 0.3 m/s: one reading per 0.3 m
        if recorder.update(reading(t=float(k)), pose_at(0.0, 0.3 * k, speed=0.3), float(k), 1e9) is not None:
            emitted += 1
    assert 24 <= emitted <= 31                                          # about one per metre: 30 m / 1 m


def test_rejected_readings_are_logged_as_invalid_without_resetting_the_spacing():
    recorder = BathymetryRecorder(BathyConfig(spacing_m=1.0))
    first = recorder.update(reading(t=0.0), pose_at(0.0, 0.0), 0.0, 1e9)
    rejected = recorder.update(reading(t=1.0, raw=0.0, quality=DepthQuality.BAD, reason="no lock"),
                               pose_at(0.0, 1.5), 1.0, 1e9)
    third = recorder.update(reading(t=2.0), pose_at(0.0, 3.0), 2.0, 1e9)
    assert first.quality == VALID and rejected.quality == INVALID and third.quality == VALID
    assert rejected.depth_m is None and rejected.raw_depth_m == 0.0 and rejected.lat is not None
    assert third.along_track_m == pytest.approx(3.0, abs=0.01)           # measured from the first VALID, not the invalid


def test_a_sample_too_close_to_the_last_one_is_dropped():
    recorder = BathymetryRecorder(BathyConfig(spacing_m=2.0))
    assert recorder.update(reading(t=0.0), pose_at(0.0, 0.0), 0.0, 1e9) is not None
    assert recorder.update(reading(t=1.0), pose_at(0.0, 1.5), 1.0, 1e9) is None
    assert recorder.skipped_close == 1
    assert recorder.update(reading(t=2.0), pose_at(0.0, 3.0), 2.0, 1e9) is not None


def test_a_reading_without_a_position_is_still_recorded_as_invalid():
    recorder = BathymetryRecorder(BathyConfig())
    s = recorder.update(reading(), None, 0.0, 1e9)
    assert s.quality == INVALID and s.lat is None and s.lon is None and "no position" in s.reasons


# -- where the echo was taken --------------------------------------------------------------------------------------

def test_the_position_is_moved_back_by_the_age_of_the_reading_plus_the_sounder_latency():
    """Pose 100 m north, moving north at 2 m/s; the reading is 0.3 s old at processing time and
    the sounder itself is 0.2 s late: the echo was taken 0.5 s ago = 1.0 m further back."""
    recorder = BathymetryRecorder(BathyConfig(latency_s=0.2))
    s = recorder.update(reading(t=10.0), pose_at(0.0, 100.0, speed=2.0), now=10.3, utc=1e9)
    east, north = FRAME.to_enu(s.lat, s.lon)
    assert (east, north) == pytest.approx((0.0, 99.0), abs=0.005)


def test_no_age_and_no_latency_means_no_shift():
    recorder = BathymetryRecorder(BathyConfig(latency_s=0.0))
    s = recorder.update(reading(t=10.0), pose_at(0.0, 100.0, speed=2.0), now=10.0, utc=1e9)
    assert FRAME.to_enu(s.lat, s.lon) == pytest.approx((0.0, 100.0), abs=0.002)


def test_a_stale_reading_is_not_dragged_back_for_ever():
    recorder = BathymetryRecorder(BathyConfig())
    s = recorder.update(reading(t=0.0), pose_at(0.0, 100.0, speed=2.0), now=500.0, utc=1e9)
    assert FRAME.to_enu(s.lat, s.lon) == pytest.approx((0.0, 90.0), abs=0.005)          # capped at 5 s x 2 m/s


def test_the_shift_follows_the_velocity_direction_not_just_north():
    recorder = BathymetryRecorder(BathyConfig(latency_s=0.5))
    pose = pose_at(50.0, 20.0, speed=2.0, velocity_en=(2.0, 0.0))                        # due east
    s = recorder.update(reading(t=0.0), pose, now=0.5, utc=1e9)
    assert FRAME.to_enu(s.lat, s.lon) == pytest.approx((48.0, 20.0), abs=0.005)


# -- the record ----------------------------------------------------------------------------------------------------------

def test_the_sample_carries_what_a_later_processing_step_needs():
    recorder = BathymetryRecorder(BathyConfig())
    s = recorder.update(reading(t=7.0, raw=8.3, offset=0.4), pose_at(10.0, 20.0, speed=1.5, sigma_m=0.07,
                                                                     roll_deg=3.0, pitch_deg=4.0, heading_deg=33.0),
                        now=7.0, utc=1_700_000_000.5, gnss_quality=5)
    assert (s.seq, s.t, s.utc) == (1, 7.0, 1_700_000_000.5)
    assert s.raw_depth_m == 8.3 and s.depth_m == pytest.approx(8.7)                       # below the waterline
    assert (s.gnss_quality, s.h_acc_m, s.speed_mps, s.heading_deg) == (5, 0.07, 1.5, 33.0)
    assert s.tilt_deg == pytest.approx(math.degrees(math.acos(math.cos(math.radians(3)) * math.cos(math.radians(4)))))
    assert s.sonar_quality == DepthQuality.GOOD and s.quality == VALID and s.reasons == ()


def test_the_heading_is_left_out_when_it_is_not_valid():
    recorder = BathymetryRecorder(BathyConfig())
    s = recorder.update(reading(), pose_at(0.0, 0.0, heading_valid=False, heading_deg=77.0), 0.0, 1e9)
    assert s.heading_deg is None


def test_counts_summary_and_reset():
    recorder, samples = run_track(BathyConfig(spacing_m=1.0), 6)
    recorder.update(reading(t=6.0, raw=0.0, quality=DepthQuality.BAD, reason="x"), pose_at(0.0, 9.0), 6.0, 1e9)
    recorder.update(reading(t=7.0, quality=DepthQuality.SUSPECT, reason="y"), pose_at(0.0, 10.5), 7.0, 1e9)
    assert recorder.counts == {INVALID: 1, LOW: 1, VALID: 6}
    assert summarize(samples) == {"INVALID": 0, "LOW_QUALITY": 0, "VALID": 6}
    recorder.reset()
    assert recorder.counts == {INVALID: 0, LOW: 0, VALID: 0}
    assert recorder.update(reading(), pose_at(0.0, 0.0), 0.0, 1e9).seq == 1
