"""Phases 17-19 on the vehicle: every depth reading queued, filtered, sampled by
distance, classified, and sent to the GCS once, filtered. (The filter, the
sampler and the classifier have their own tests: test_sonar.py, test_bathymetry.py.)"""
from __future__ import annotations

import pytest

from bahr_pilot import geo
from bahr_pilot.bathymetry import SampleQuality
from bahr_pilot.sensors import MIN_DEPTH_INTERVAL_S, GnssState
from bahr_pilot.sonar import DepthQuality
from tests.poses import make_pose
from tests.test_vehicle import make_vehicle

FRAME = geo.LocalFrame(41.0, 29.0)


# -- GnssState: every reading, once ----------------------------------------------------------------------

def test_every_depth_reading_is_queued_in_order_and_taken_once():
    gnss = GnssState()
    for k, depth in enumerate((5.0, 5.1, 5.2)):
        gnss.apply_echomap("SDDPT", [str(depth), "0.0"], now=float(k))
    assert gnss.take_depth_readings() == [(0.0, 5.0), (1.0, 5.1), (2.0, 5.2)]
    assert gnss.take_depth_readings() == []


def test_the_echomap_reports_each_sounding_twice_and_only_one_counts():
    """SDDPT and SDDBT for the same sounding arrive within a few milliseconds."""
    gnss = GnssState()
    gnss.apply_echomap("SDDPT", ["7.5", "0.0"], now=10.00)
    gnss.apply_echomap("SDDBT", ["24.6", "f", "7.5", "M", "4.1", "F"], now=10.02)
    gnss.apply_echomap("SDDBT", ["24.9", "f", "7.6", "M", "4.1", "F"], now=11.00)      # a new sounding (other sentence first)
    gnss.apply_echomap("SDDPT", ["7.6", "0.0"], now=11.02)
    assert gnss.take_depth_readings() == [(10.0, 7.5), (11.0, 7.6)]


def test_a_sounder_that_only_sends_sddbt_still_works():
    gnss = GnssState()
    gnss.apply_echomap("SDDBT", ["24.6", "f", "7.5", "M", "4.1", "F"], now=3.0)
    assert gnss.take_depth_readings() == [(3.0, 7.5)]


def test_readings_closer_together_than_the_minimum_interval_are_one_sounding():
    gnss = GnssState()
    gnss.apply_echomap("SDDPT", ["5.0", "0.0"], now=0.0)
    gnss.apply_echomap("SDDPT", ["5.1", "0.0"], now=MIN_DEPTH_INTERVAL_S - 0.01)
    gnss.apply_echomap("SDDPT", ["5.2", "0.0"], now=MIN_DEPTH_INTERVAL_S + 0.01)
    assert [d for _, d in gnss.take_depth_readings()] == [5.0, 5.2]


def test_garbage_depth_fields_queue_nothing():
    gnss = GnssState()
    gnss.apply_echomap("SDDPT", ["abc", "0.0"], now=0.0)
    gnss.apply_echomap("SDDPT", [""], now=1.0)
    gnss.apply_echomap("SDDBT", ["1", "f", "", "M"], now=2.0)
    assert gnss.take_depth_readings() == []


def test_the_queue_is_bounded_and_keeps_the_newest():
    gnss = GnssState()
    for k in range(200):
        gnss.apply_echomap("SDDPT", [f"{5 + k * 0.01:.2f}", "0.0"], now=float(k))
    readings = gnss.take_depth_readings()
    assert len(readings) == 64 and readings[-1][0] == 199.0


def test_the_newest_depth_for_the_logs_is_unchanged():
    from bahr_pilot.state import VehicleState
    gnss, state = GnssState(), VehicleState()
    gnss.apply_echomap("SDDPT", ["7.5", "0.0"], now=100.0)
    gnss.publish(state, now=100.5)
    assert state.depth_m == 7.5


# -- the vehicle ----------------------------------------------------------------------------------------------

@pytest.fixture
def vehicle():
    v = make_vehicle()
    v.state.armed = True
    return v


def advance(vehicle, k, depth, *, speed=1.5, sigma=0.05, armed=True):
    """The loop's work for one second k: the boat has run 1.5 m north per second, one reading arrives."""
    lat, lon = FRAME.to_geodetic(0.0, speed * k)
    vehicle.state.armed = armed
    vehicle.state.pose = make_pose(lat, lon, speed_mps=speed, velocity_en=(0.0, speed), sigma_m=sigma)
    vehicle.gnss.apply_echomap("SDDPT", [f"{depth:.1f}", "0.0"], now=float(k))
    vehicle._update_bathymetry(float(k) + 0.02)


def run(vehicle, depths, **kw):
    for k, depth in enumerate(depths):
        advance(vehicle, k, depth, **kw)


def test_readings_become_samples_with_a_position_and_a_quality(vehicle):
    run(vehicle, [8.0, 8.1, 8.0, 8.1, 8.0, 8.1])
    samples = list(vehicle.bathy_samples)
    assert len(samples) == 6
    assert [s.quality for s in samples] == [SampleQuality.LOW_QUALITY] * 3 + [SampleQuality.VALID] * 3
    assert all("not enough history" in s.reasons[0] for s in samples[:3])
    assert FRAME.to_enu(samples[5].lat, samples[5].lon)[1] == pytest.approx(7.5, abs=0.1)     # 1.5 m/s x 5 s, minus the 0.02 s age


def test_a_spike_is_logged_as_invalid_and_never_reaches_the_gcs(vehicle):
    run(vehicle, [8.0, 8.1, 8.0, 8.1, 8.0])
    vehicle._depth_to_send = None
    advance(vehicle, 5, 12.0)
    last = vehicle.bathy_samples[-1]
    assert last.quality == SampleQuality.INVALID and last.depth_m is None and last.raw_depth_m == 12.0
    assert vehicle._depth_to_send is None


def test_a_rejected_reading_does_not_erase_a_good_one_still_waiting_to_be_sent(vehicle):
    """Two readings in one pass of the loop (the loop was slow): the good one is still sent."""
    run(vehicle, [8.0, 8.1, 8.0, 8.1])
    vehicle._depth_to_send = None
    lat, lon = FRAME.to_geodetic(0.0, 6.0)
    vehicle.state.pose = make_pose(lat, lon, speed_mps=1.5, velocity_en=(0.0, 1.5))
    vehicle.gnss.apply_echomap("SDDPT", ["8.2", "0.0"], now=4.0)
    vehicle.gnss.apply_echomap("SDDPT", ["14.0", "0.0"], now=5.0)               # a spike right after it
    vehicle._update_bathymetry(5.02)
    assert vehicle._depth_to_send == 8.2


def test_the_first_readings_are_not_sent_to_the_gcs(vehicle):
    sent = []
    for k, depth in enumerate([8.0, 8.1, 8.0, 8.1]):
        advance(vehicle, k, depth)
        sent.append(vehicle._depth_to_send)
        vehicle._depth_to_send = None
    assert sent == [None, None, None, 8.1]           # three warm-up readings, then the filter has a basis


def test_a_boat_on_the_bench_does_not_survey_but_the_depth_tile_still_works(vehicle):
    run(vehicle, [8.0, 8.1, 8.0, 8.1], armed=False)
    assert len(vehicle.bathy_samples) == 0 and vehicle._depth_to_send == 8.1


def test_a_stationary_boat_records_no_samples(vehicle):
    run(vehicle, [8.0] * 8, speed=0.05)
    assert len(vehicle.bathy_samples) == 0


def test_a_poor_position_makes_low_quality_samples(vehicle):
    run(vehicle, [8.0, 8.1, 8.0, 8.1, 8.0, 8.1, 8.0], sigma=1.2)
    later = list(vehicle.bathy_samples)[3:]
    assert later and all(s.quality == SampleQuality.LOW_QUALITY and any("accuracy" in r for r in s.reasons) for s in later)


def test_the_gnss_quality_label_comes_from_the_raw_fix(vehicle):
    vehicle.state.gnss_quality = 5
    run(vehicle, [8.0] * 5)
    assert all(s.gnss_quality == 5 for s in vehicle.bathy_samples)


# -- what the GCS receives ---------------------------------------------------------------------------------------

def capture_depth_messages(vehicle, monkeypatch):
    sent = []
    monkeypatch.setattr(vehicle.link.mav, "distance_sensor_send", lambda *a, **k: sent.append(a))
    return sent


def test_one_distance_sensor_message_per_accepted_sounding_not_five_copies(vehicle, monkeypatch):
    """BAHR-GCS plots a point for every DISTANCE_SENSOR it gets. Telemetry runs at 5 Hz and
    sounders at 1 Hz: the old code sent each raw reading five times."""
    sent = capture_depth_messages(vehicle, monkeypatch)
    run(vehicle, [8.0, 8.1, 8.0, 8.1])
    for _ in range(5):
        vehicle.send_telemetry()
    assert len(sent) == 1
    advance(vehicle, 4, 8.2)
    for _ in range(5):
        vehicle.send_telemetry()
    assert len(sent) == 2 and sent[1][3] == 820


def test_nothing_is_sent_when_no_new_sounding_has_arrived(vehicle, monkeypatch):
    sent = capture_depth_messages(vehicle, monkeypatch)
    vehicle.send_telemetry()
    assert sent == []


def test_the_message_carries_the_filtered_offset_depth_and_the_configured_range(vehicle, monkeypatch):
    sent = capture_depth_messages(vehicle, monkeypatch)
    vehicle.params.update({"RNGFND1_OFFSET": 0.30, "RNGFND1_MIN": 0.6, "RNGFND1_MAX": 40.0})
    run(vehicle, [8.0, 8.1, 8.0, 8.1])
    vehicle.send_telemetry()
    _, min_cm, max_cm, depth_cm, *_ = sent[0]
    assert (min_cm, max_cm, depth_cm) == (60, 4000, 840)                       # 8.1 + 0.30 below the waterline


def test_a_depth_beyond_the_message_range_is_clamped_not_wrapped(vehicle, monkeypatch):
    sent = capture_depth_messages(vehicle, monkeypatch)
    vehicle.params["RNGFND1_MAX"] = 1000.0
    run(vehicle, [900.0, 900.1, 900.0, 900.1])
    vehicle.send_telemetry()
    assert sent[0][3] == 65535 and sent[0][2] == 65535


# -- tuning --------------------------------------------------------------------------------------------------------------

def test_tuning_changes_reach_the_filter_and_the_sampler(vehicle):
    run(vehicle, [8.0] * 4)
    vehicle.params.update({"SONAR_SPIKE": 0.9, "RNGFND1_OFFSET": 0.2, "BATHY_SPACING": 3.0, "SONAR_LATENCY": 0.25})
    vehicle._update_bathymetry(100.0)
    assert vehicle.sonar.cfg.spike_abs_m == 0.9 and vehicle.sonar.cfg.offset_m == 0.2
    assert vehicle.bathymetry.cfg.spacing_m == 3.0 and vehicle.bathymetry.cfg.latency_s == 0.25
    assert vehicle.sonar.counts[DepthQuality.GOOD] == 0                       # a changed sonar setting restarts the filter


def test_an_unchanged_configuration_does_not_restart_the_filter(vehicle):
    run(vehicle, [8.0, 8.1, 8.0, 8.1, 8.0])
    good = vehicle.sonar.counts[DepthQuality.GOOD]
    vehicle._update_bathymetry(50.0)
    vehicle._update_bathymetry(51.0)
    assert vehicle.sonar.counts[DepthQuality.GOOD] == good and good > 0


def test_a_maximum_depth_below_the_minimum_cannot_reject_everything(vehicle):
    vehicle.params.update({"RNGFND1_MIN": 5.0, "RNGFND1_MAX": 2.0})
    config = vehicle._sonar_config()
    assert config.max_depth_m > config.min_depth_m


def test_the_defaults_are_the_documented_ones(vehicle):
    p = vehicle.params
    assert (p["RNGFND1_MIN"], p["RNGFND1_MAX"], p["RNGFND1_OFFSET"]) == (0.5, 100.0, 0.0)
    assert (p["SONAR_SPIKE"], p["SONAR_LATENCY"]) == (0.5, 0.0)
    assert (p["BATHY_SPACING"], p["BATHY_MAX_HACC"], p["BATHY_MAX_SPEED"], p["BATHY_MAX_TILT"]) == (1.0, 0.5, 3.0, 15.0)


def test_the_sample_buffer_is_bounded(vehicle):
    assert vehicle.bathy_samples.maxlen == 2000
