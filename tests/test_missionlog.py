"""Phase 25: the mission folder (bahr_pilot/missionlog.py)."""
from __future__ import annotations

import csv
import json

import pytest

from bahr_pilot import geo
from bahr_pilot.bathymetry import BathySample, SampleQuality
from bahr_pilot.missionlog import (
    BATHY_COLUMNS, EVENT_COLUMNS, TRACK_COLUMNS, MissionLog, summarize_folder,
)
from bahr_pilot.sonar import DepthQuality
from tests.poses import make_pose

FRAME = geo.LocalFrame(41.0, 29.0)
VALID, LOW, INVALID = SampleQuality.VALID, SampleQuality.LOW_QUALITY, SampleQuality.INVALID


class Clock:
    def __init__(self, start=1_790_000_000.0):          # 2026-09-21T14:13:20Z
        self.now = start

    def __call__(self):
        return self.now


def make_log(tmp_path, clock=None, mono=None):
    clock = clock or Clock()
    return MissionLog(tmp_path, clock=clock, mono=mono or clock), clock


def sample(seq=1, quality=VALID, depth=8.0, reasons=(), lat=41.0, lon=29.0, **kw):
    return BathySample(
        seq=seq, t=100.0 + seq, utc=1_790_000_000.0 + seq, lat=lat, lon=lon, depth_m=depth, raw_depth_m=8.0,
        quality=quality, reasons=tuple(reasons), sonar_quality=DepthQuality.GOOD, gnss_quality=5, h_acc_m=0.07,
        speed_mps=1.5, heading_deg=33.0, tilt_deg=2.0, along_track_m=1.5, **kw)


def rows(path):
    with open(path, encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def started(tmp_path, **kw):
    log, clock = make_log(tmp_path, **kw)
    log.start({"software": "test"})
    return log, clock


# -- the folder -----------------------------------------------------------------------------------------

def test_a_mission_gets_its_own_timestamped_folder_with_the_standard_files(tmp_path):
    log, _ = started(tmp_path)
    assert log.folder.parent == tmp_path and log.folder.name == "mission-20260921T141320"
    assert sorted(p.name for p in log.folder.iterdir()) == ["bathymetry.csv", "events.csv", "meta.json", "track.csv"]
    assert log.active


def test_two_missions_in_the_same_second_do_not_collide(tmp_path):
    log, _ = make_log(tmp_path)
    first = log.start({})
    log.stop()
    second = log.start({})
    assert first != second and second.name == first.name + "-2"


def test_the_base_directory_is_created_if_missing(tmp_path):
    log, _ = make_log(tmp_path / "deep" / "er")
    assert log.start({}) is not None and log.folder.exists()


def test_meta_records_what_flew(tmp_path):
    log, _ = make_log(tmp_path)
    log.start({"software": {"pilot": "0.1.1"}, "params": {"WP_RADIUS": 3.0}, "args": {"gcs_host": "10.0.0.1"}})
    meta = json.loads((log.folder / "meta.json").read_text(encoding="utf-8"))
    assert meta["format"] == 1 and meta["started"] == "2026-09-21T14:13:20Z"
    assert meta["software"] == {"pilot": "0.1.1"} and meta["params"] == {"WP_RADIUS": 3.0}


def test_headers_are_the_documented_columns(tmp_path):
    log, _ = started(tmp_path)
    folder = log.folder
    log.stop()
    for name, columns in (("bathymetry", BATHY_COLUMNS), ("track", TRACK_COLUMNS), ("events", EVENT_COLUMNS)):
        assert (folder / f"{name}.csv").read_text(encoding="utf-8").splitlines()[0] == ",".join(columns)


# -- bathymetry.csv ---------------------------------------------------------------------------------------

def test_a_sample_becomes_a_row_with_every_field(tmp_path):
    log, _ = started(tmp_path)
    log.bathymetry(sample(seq=7, depth=8.126, reasons=("position accuracy 0.60 m > 0.5 m", "speed 3.4 m/s > 3 m/s"),
                          quality=LOW, lat=41.00001234567, lon=29.1))
    row = rows(log.folder / "bathymetry.csv")[0]
    assert row["seq"] == "7" and row["quality"] == "LOW_QUALITY" and row["sonar_quality"] == "GOOD"
    assert row["lat"] == "41.00001235" and row["lon"] == "29.10000000"
    assert row["depth_m"] == "8.13" and row["raw_depth_m"] == "8.00"
    assert row["reasons"] == "position accuracy 0.60 m > 0.5 m; speed 3.4 m/s > 3 m/s"
    assert (row["gnss_quality"], row["h_acc_m"], row["speed_mps"], row["heading_deg"]) == ("5", "0.070", "1.50", "33.0")


def test_a_rejected_sample_keeps_its_reason_and_has_empty_depth_and_position(tmp_path):
    log, _ = started(tmp_path)
    log.bathymetry(sample(quality=INVALID, depth=None, reasons=("sonar: spike: 6.90 m from the expected 5.10 m",
                                                                 "no position"), lat=None, lon=None))
    row = rows(log.folder / "bathymetry.csv")[0]
    assert row["quality"] == "INVALID" and row["depth_m"] == "" and row["lat"] == "" and row["lon"] == ""
    assert "spike" in row["reasons"] and "no position" in row["reasons"]
    assert row["raw_depth_m"] == "8.00"


def test_a_reason_with_a_comma_survives_the_csv_round_trip(tmp_path):
    log, _ = started(tmp_path)
    log.bathymetry(sample(reasons=('sonar suspect: noisy, "wavy" bottom',)))
    assert rows(log.folder / "bathymetry.csv")[0]["reasons"] == 'sonar suspect: noisy, "wavy" bottom'


def test_rows_are_on_disk_while_the_file_is_still_open(tmp_path):
    """A power cut must not lose what was already logged."""
    log, _ = started(tmp_path)
    for k in range(5):
        log.bathymetry(sample(seq=k + 1))
    assert len(rows(log.folder / "bathymetry.csv")) == 5                       # read back without closing


def test_missing_values_are_empty_cells_not_nan(tmp_path):
    log, _ = started(tmp_path)
    log.bathymetry(sample(depth=float("nan")))
    assert rows(log.folder / "bathymetry.csv")[0]["depth_m"] == ""


# -- track.csv ----------------------------------------------------------------------------------------------

def pose_at(north, **kw):
    lat, lon = FRAME.to_geodetic(0.0, north)
    return make_pose(lat, lon, speed_mps=1.5, course_deg=0.0, heading_deg=1.0, **kw)


def test_the_track_is_written_at_two_hertz_however_often_it_is_offered(tmp_path):
    log, clock = started(tmp_path)
    for k in range(200):                                # offered at 20 Hz for 10 s
        clock.now += 0.05
        log.track(100.0 + k * 0.05, pose_at(1.5 * k * 0.05), mode=10)
    assert len(rows(log.folder / "track.csv")) == 20    # 2 Hz


def test_track_rows_carry_the_estimate(tmp_path):
    log, _ = started(tmp_path)
    log.track(100.0, pose_at(10.0, roll_deg=3.0, pitch_deg=-2.0, sigma_m=0.08), mode=10)
    row = rows(log.folder / "track.csv")[0]
    assert row["position_status"] == "OK" and row["mode"] == "10" and row["speed_mps"] == "1.50"
    assert row["heading_valid"] == "1" and row["heading_deg"] == "1.0" and row["sigma_m"] == "0.080"
    assert (row["roll_deg"], row["pitch_deg"], row["course_deg"]) == ("3.0", "-2.0", "0.0")


def test_nothing_is_logged_while_the_position_is_unknown(tmp_path):
    log, _ = started(tmp_path)
    log.track(100.0, make_pose(None, None), mode=10)
    log.track(101.0, None, mode=10)
    assert rows(log.folder / "track.csv") == []


def test_the_heading_cell_is_empty_when_the_heading_is_not_valid(tmp_path):
    log, _ = started(tmp_path)
    log.track(100.0, pose_at(0.0, heading_valid=False), mode=10)
    row = rows(log.folder / "track.csv")[0]
    assert row["heading_deg"] == "" and row["heading_valid"] == "0"


# -- events.csv -----------------------------------------------------------------------------------------------

def test_events_carry_time_severity_source_and_text(tmp_path):
    log, clock = started(tmp_path)
    clock.now += 12.5
    log.event(2, "failsafe", "Failsafe: GCS link lost (HOLD)")
    log.event(6, "mode", "mode -> HOLD")
    got = rows(log.folder / "events.csv")
    assert [(r["level"], r["source"], r["text"]) for r in got] == [
        ("CRITICAL", "failsafe", "Failsafe: GCS link lost (HOLD)"), ("INFO", "mode", "mode -> HOLD")]
    assert float(got[0]["utc"]) == pytest.approx(1_790_000_012.5)


def test_estimator_jitter_while_stationary_is_not_distance_travelled(tmp_path):
    """Found in the simulator: a 102 m run reported 104.7 m because 44 s of standing still
    summed the position jitter at 2 Hz. 5 cm of wobble, 400 points, would be 20 m."""
    log, _ = started(tmp_path)
    for k in range(400):
        log.track(100.0 + k * 0.5, pose_at(50.0 + (0.05 if k % 2 else -0.05)), mode=0)
    folder = log.stop()
    s = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert s["track_points"] == 400 and s["distance_m"] == 0.0


def test_slow_motion_below_the_step_is_still_counted_once_it_adds_up(tmp_path):
    """0.3 m/s at 2 Hz is 0.15 m a point, under the 0.25 m step: the distance is measured from the
    last COUNTED point, so a crawling boat is not recorded as stopped."""
    log, _ = started(tmp_path)
    for k in range(201):                                        # 100 s at 0.3 m/s = 30 m
        log.track(100.0 + k * 0.5, pose_at(0.15 * k), mode=0)
    s = json.loads((log.stop() / "summary.json").read_text(encoding="utf-8"))
    assert s["distance_m"] == pytest.approx(30.0, abs=0.3)


def test_an_unknown_severity_is_logged_as_its_number(tmp_path):
    log, _ = started(tmp_path)
    log.event(42, "x", "y")
    assert rows(log.folder / "events.csv")[0]["level"] == "42"


# -- summary.json --------------------------------------------------------------------------------------------------

def test_the_summary_counts_distance_depth_and_events(tmp_path):
    log, clock = started(tmp_path)
    for k in range(11):                                          # 100 m north in 11 track points, 10 s apart
        clock.now += 10.0
        log.track(100.0 + k * 10.0, pose_at(10.0 * k), mode=10)
    log.bathymetry(sample(seq=1, depth=5.0))
    log.bathymetry(sample(seq=2, depth=24.1))
    log.bathymetry(sample(seq=3, quality=LOW, depth=3.9, reasons=("x",)))
    log.bathymetry(sample(seq=4, quality=INVALID, depth=None, reasons=("spike",)))
    # a good echo with no position is INVALID too, and still carries a depth: it is not "the seabed here"
    log.bathymetry(sample(seq=5, quality=INVALID, depth=99.0, reasons=("no position",)))
    log.event(2, "failsafe", "a")
    log.event(2, "failsafe", "b")
    log.event(6, "mode", "c")
    folder = log.stop("disarmed", {"note": "ok"})
    s = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert s["reason"] == "disarmed" and s["note"] == "ok" and s["duration_s"] == pytest.approx(110.0)
    assert s["distance_m"] == pytest.approx(100.0, abs=0.5) and s["track_points"] == 11
    assert s["bathymetry"] == {"INVALID": 2, "LOW_QUALITY": 1, "VALID": 2}
    assert (s["depth_min_m"], s["depth_max_m"]) == (3.9, 24.1)                # the 99 m INVALID one is left out
    assert s["events"] == {"CRITICAL": 2, "INFO": 1} and s["rows_dropped"] == 0 and s["log_error"] is None
    assert not log.active


def test_stop_is_safe_twice_and_before_start(tmp_path):
    log, _ = make_log(tmp_path)
    assert log.stop() is None
    log.start({})
    assert log.stop() is not None and log.stop() is None


def test_starting_while_a_mission_is_open_closes_it_first(tmp_path):
    log, _ = make_log(tmp_path)
    first = log.start({})
    second = log.start({})
    assert (first / "summary.json").exists()
    assert json.loads((first / "summary.json").read_text(encoding="utf-8"))["reason"] == "restarted"
    assert second != first


def test_a_folder_that_lost_its_summary_can_be_summarised_from_the_data(tmp_path):
    log, _ = started(tmp_path)
    log.bathymetry(sample(seq=1, depth=5.0))
    log.bathymetry(sample(seq=2, quality=INVALID, depth=None, reasons=("spike",)))
    log.bathymetry(sample(seq=3, quality=LOW, depth=7.5, reasons=("x",)))
    # the boat loses power here: no stop(), no summary.json
    assert not (log.folder / "summary.json").exists()
    assert summarize_folder(log.folder) == {"bathymetry": {"INVALID": 1, "LOW_QUALITY": 1, "VALID": 1},
                                            "depth_min_m": 5.0, "depth_max_m": 7.5}


# -- the disk misbehaves ------------------------------------------------------------------------------------------------

def test_a_write_error_stops_logging_without_raising(tmp_path, monkeypatch):
    log, _ = started(tmp_path)
    log.bathymetry(sample(seq=1))
    monkeypatch.setattr(log._files["bathymetry"]._file, "flush", lambda: (_ for _ in ()).throw(OSError("No space left on device")))
    log.bathymetry(sample(seq=2))                         # the vehicle loop must survive this
    log.bathymetry(sample(seq=3))
    log.track(100.0, pose_at(0.0))
    log.event(2, "x", "y")
    assert log.error == "OSError: No space left on device" and not log.active
    assert log.dropped >= 4


def test_after_a_failure_the_summary_still_gets_written_and_says_so(tmp_path, monkeypatch):
    log, _ = started(tmp_path)
    monkeypatch.setattr(log._files["events"]._file, "flush", lambda: (_ for _ in ()).throw(OSError("full")))
    log.event(2, "x", "y")
    folder = log.stop("disarmed")
    s = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert s["log_error"] == "OSError: full" and s["rows_dropped"] >= 1


def test_a_start_that_cannot_create_the_folder_returns_none_and_never_raises(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.write_text("a file where a directory should be", encoding="utf-8")
    log, _ = make_log(blocker / "missions")
    assert log.start({}) is None and log.error is not None and not log.active
    log.bathymetry(sample())                              # and everything after it is a harmless no-op
    log.track(1.0, pose_at(0.0))
    log.event(2, "x", "y")
    log.stop()                                            # must not raise either


def test_a_new_mission_after_a_failed_one_starts_clean(tmp_path, monkeypatch):
    log, _ = started(tmp_path)
    monkeypatch.setattr(log._files["events"]._file, "flush", lambda: (_ for _ in ()).throw(OSError("full")))
    log.event(2, "x", "y")
    log.stop()
    monkeypatch.undo()
    assert log.start({}) is not None and log.error is None and log.dropped == 0 and log.active
    log.event(2, "x", "again")
    assert len(rows(log.folder / "events.csv")) == 1
