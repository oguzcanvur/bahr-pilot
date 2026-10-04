"""A survey line flown in the simulator over a known seabed, with the recorded
bathymetry compared with the TRUTH: sonar filter + sampler + classifier + the
messages the ground station gets, all on the real Vehicle.

The seabed is a 26 degree bank (0.5 m deeper per metre north, over 40 m) and
flat beyond. The boat runs 100 m north from a standing start, so it also
accelerates onto the bank - the case that locked an earlier filter out for 20 m."""
from __future__ import annotations

import math
import statistics

import pytest

from bahr_pilot import geo
from bahr_pilot.bathymetry import SampleQuality
from bahr_pilot.sitl.sensors import Fault
from tests.test_sitl_vehicle import Flight

FRAME = geo.LocalFrame(41.0, 29.0)
VALID, LOW, INVALID = SampleQuality.VALID, SampleQuality.LOW_QUALITY, SampleQuality.INVALID
LINE = ((0.0, 100.0),)


def bank(east: float, north: float) -> float:
    return 4.0 + 0.5 * min(max(north, 0.0), 40.0)


def survey(*, sensor=None, params=None, faults=(), seconds=110.0):
    return Flight(route=LINE, seabed=bank, seconds=seconds, telemetry=True, sensor=sensor or {}, params=params,
                  faults=faults)


def errors(flight, qualities=(VALID, LOW)):
    out = []
    for s in flight.vehicle.bathy_samples:
        if s.quality in qualities and s.depth_m is not None:
            east, north = FRAME.to_enu(s.lat, s.lon)
            out.append(s.depth_m - bank(east, north))
    return out


def rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


def by_quality(flight):
    return {q: sum(1 for s in flight.vehicle.bathy_samples if s.quality == q) for q in SampleQuality}


@pytest.fixture(scope="module")
def clean():
    return survey(params={"SONAR_LATENCY": 0.2})


# -- a clean sounder ----------------------------------------------------------------------------------------

def test_a_clean_run_over_a_steep_bank_loses_nothing_and_reaches_the_end(clean):
    """Measured: 0 INVALID, 5 LOW_QUALITY (the start-up readings and the knee of the bank), 63 VALID."""
    q = by_quality(clean)
    assert clean.completed
    assert q[INVALID] == 0 and q[VALID] >= 55 and q[LOW] <= 8


def test_the_recorded_depths_match_the_seabed_at_the_recorded_positions(clean):
    err = errors(clean)
    assert rms(err) < 0.1 and max(abs(e) for e in err) < 0.3                 # measured 0.060 m RMS, 0.15 m worst


def test_samples_follow_the_track_in_order_and_one_per_sounding(clean):
    samples = list(clean.vehicle.bathy_samples)
    norths = [FRAME.to_enu(s.lat, s.lon)[1] for s in samples]
    assert norths == sorted(norths)
    assert [s.seq for s in samples] == list(range(1, len(samples) + 1))
    gaps = [b - a for a, b in zip(norths, norths[1:])]
    assert 1.2 < statistics.median(gaps) < 1.8                               # 1.5 m/s, one sounding a second


def test_the_start_up_readings_are_flagged_not_trusted(clean):
    first = list(clean.vehicle.bathy_samples)[0]
    assert first.quality == LOW and "not enough history" in first.reasons[0]


def test_what_the_gcs_receives_matches_the_seabed(clean):
    """Every DISTANCE_SENSOR depth sent, against the truth where it was taken: none is a spike."""
    assert len(clean.depth_messages) >= 90
    trace = clean.trace
    worst = 0.0
    for t, cm in clean.depth_messages:
        i = trace.index_at(max(0.0, t - 0.4))                                # sounded ~0.2 s ago, sent within 0.2 s
        worst = max(worst, abs(cm / 100.0 - bank(trace.east[i], trace.north[i])))
    assert worst < 0.5


def test_the_gcs_gets_one_message_per_sounding_not_five(clean):
    seconds = clean.trace.t[-1]
    assert len(clean.depth_messages) <= seconds * 1.05                       # 1 Hz sounder over the whole run


# -- a dirty sounder -------------------------------------------------------------------------------------------

DIRTY = dict(sonar_spike_rate=0.08, sonar_zero_rate=0.03)


@pytest.fixture(scope="module")
def dirty():
    return survey(sensor=DIRTY, params={"SONAR_LATENCY": 0.2})


def test_spikes_and_dropouts_are_logged_as_invalid(dirty):
    """~11 % of the readings were corrupted; they appear as INVALID rows with the reason."""
    q = by_quality(dirty)
    assert 7 <= q[INVALID] <= 20
    reasons = {r for s in dirty.vehicle.bathy_samples if s.quality == INVALID for r in s.reasons}
    assert any("spike" in r for r in reasons) and any("below minimum" in r for r in reasons)


def test_no_spike_leaks_into_the_usable_samples(dirty):
    err = errors(dirty)
    assert max(abs(e) for e in err) < 0.5                                    # measured 0.12 m
    assert rms(err) < 0.1


def test_no_spike_reaches_the_gcs(dirty):
    trace = dirty.trace
    for t, cm in dirty.depth_messages:
        i = trace.index_at(max(0.0, t - 0.4))
        assert abs(cm / 100.0 - bank(trace.east[i], trace.north[i])) < 0.8


def test_the_dirty_run_still_reaches_the_end(dirty):
    assert dirty.completed


# -- position latency --------------------------------------------------------------------------------------------

def test_compensating_the_sounder_latency_removes_the_along_track_shift():
    """The echo is taken 0.2 s before it is reported: 0.3 m behind the boat at 1.5 m/s, which on a
    0.5 m/m bank is 0.15 m of depth error unless SONAR_LATENCY says so. Measured RMS 0.111 -> 0.060 m."""
    uncompensated = survey(params={"SONAR_LATENCY": 0.0})
    compensated = survey(params={"SONAR_LATENCY": 0.2})
    assert rms(errors(compensated)) < 0.7 * rms(errors(uncompensated))
    assert max(abs(e) for e in errors(uncompensated)) > 0.2


# -- degraded positioning, sounder failure ---------------------------------------------------------------------------

def test_a_poor_position_marks_every_sample_low_quality():
    flight = survey(sensor={}, faults=[Fault("gnss_degraded", 0.0, 1000.0)], params={"SONAR_LATENCY": 0.2},
                    seconds=120.0)
    samples = [s for s in flight.vehicle.bathy_samples if s.quality != INVALID]
    assert samples and all(s.quality == LOW for s in samples)
    assert all(any("position accuracy" in r for r in s.reasons) for s in samples[5:])


def test_a_sonar_outage_leaves_a_gap_and_no_failsafe():
    flight = survey(faults=[Fault("sonar_outage", 30.0, 45.0)], params={"SONAR_LATENCY": 0.2}, seconds=130.0)
    times = [s.t for s in flight.vehicle.bathy_samples]
    assert not [t for t in times if 31.0 <= t < 45.0]                       # nothing recorded while the sounder was silent
    assert [t for t in times if t >= 46.0]                                  # and it resumes
    assert not flight.said("Failsafe") and flight.completed


def test_after_an_outage_the_filter_starts_again_with_suspect_readings():
    flight = survey(faults=[Fault("sonar_outage", 30.0, 45.0)], params={"SONAR_LATENCY": 0.2}, seconds=130.0)
    after = [s for s in flight.vehicle.bathy_samples if s.t >= 45.0][:3]      # the sounder is back at t = 45
    assert all(s.quality == LOW and "not enough history" in s.reasons[0] for s in after)


def test_a_silent_sounder_still_lets_the_boat_complete_the_mission():
    flight = survey(faults=[Fault("sonar_outage", 0.0, 1000.0)])
    assert flight.completed and len(flight.vehicle.bathy_samples) == 0 and flight.depth_messages == []


# -- the mission folder of a whole survey ------------------------------------------------------------------------

def test_the_mission_folder_holds_the_whole_survey(tmp_path):
    import csv
    import json
    flight = Flight(route=LINE, seabed=bank, seconds=110.0, telemetry=True, params={"SONAR_LATENCY": 0.2},
                    log_dir=str(tmp_path))
    log = flight.vehicle.mission_log
    folder = log.stop("test")
    with open(folder / "bathymetry.csv", encoding="utf-8", newline="") as handle:
        logged = list(csv.DictReader(handle))
    samples = list(flight.vehicle.bathy_samples)
    assert [int(r["seq"]) for r in logged] == [s.seq for s in samples]            # nothing missing, in order
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert summary["bathymetry"]["VALID"] == sum(s.quality == VALID for s in samples)
    t = flight.trace                                                              # the simulator's own truth
    truth = sum(math.hypot(t.east[i] - t.east[i - 1], t.north[i] - t.north[i - 1]) for i in range(1, len(t.t)))
    assert truth == pytest.approx(102.0, abs=1.0)                                 # 100 m leg + the glide in HOLD
    assert summary["distance_m"] == pytest.approx(truth, abs=1.0)                 # measured 102.0 (104.7 before the step rule)
    usable = [s.depth_m for s in samples if s.quality != INVALID and s.depth_m is not None]
    assert summary["depth_min_m"] == min(usable) and summary["depth_max_m"] == max(usable)
    assert 3.7 < summary["depth_min_m"] < 5.0 and 23.7 < summary["depth_max_m"] < 24.3   # the bank is 4 m ... 24 m
    events = [r["text"] for r in csv.DictReader(open(folder / "events.csv", encoding="utf-8", newline=""))]
    assert "mode -> HOLD" in events and any(e.startswith("Logging to ") for e in events)
    track = list(csv.DictReader(open(folder / "track.csv", encoding="utf-8", newline="")))
    assert 150 <= len(track) <= 230                                               # ~2 Hz over ~100 s
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    assert meta["params_changed"] == {"SONAR_LATENCY": 0.2}
