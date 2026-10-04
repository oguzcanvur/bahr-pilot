"""Phase 25 on the vehicle: a mission folder per arming, fed by the loop."""
from __future__ import annotations

import csv
import json
from types import SimpleNamespace

import pytest

import bahr_pilot.vehicle as vehicle_module
from bahr_pilot import geo
from bahr_pilot.modes import MODE_AUTO, MODE_HOLD
from tests.poses import make_pose
from tests.test_vehicle import make_vehicle

FRAME = geo.LocalFrame(41.0, 29.0)


def rows(path):
    with open(path, encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def missions(tmp_path):
    base = tmp_path / "missions"
    return sorted(base.iterdir()) if base.exists() else []


@pytest.fixture
def vehicle(tmp_path):
    v = make_vehicle(log_dir=str(tmp_path))
    v.sent_texts = []
    original = v.link.mav.statustext_send
    v.link.mav.statustext_send = lambda severity, text: (v.sent_texts.append(bytes(text).decode()),
                                                          original(severity, text))[1]
    return v


def tick(vehicle, now, north=0.0, speed=1.5):
    lat, lon = FRAME.to_geodetic(0.0, north)
    vehicle.state.pose = make_pose(lat, lon, speed_mps=speed, velocity_en=(0.0, speed), course_deg=0.0)
    vehicle._update_mission_log(now)


def test_arming_opens_a_mission_folder_and_says_so(vehicle, tmp_path):
    assert missions(tmp_path) == []
    vehicle.state.armed = True
    tick(vehicle, 100.0)
    folders = missions(tmp_path)
    assert len(folders) == 1 and folders[0].name.startswith("mission-")
    assert f"Logging to {folders[0].name}" in vehicle.sent_texts


def test_disarming_closes_it_with_a_summary(vehicle, tmp_path):
    vehicle.state.armed = True
    tick(vehicle, 100.0)
    vehicle.state.armed = False
    tick(vehicle, 101.0)
    folder = missions(tmp_path)[0]
    summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
    assert summary["reason"] == "disarmed"
    assert any(r["text"] == "mission ended: disarmed" for r in rows(folder / "events.csv"))
    assert not vehicle.mission_log.active


def test_nothing_is_created_while_disarmed(vehicle, tmp_path):
    for k in range(5):
        tick(vehicle, 100.0 + k)
    assert missions(tmp_path) == []


def test_each_arming_is_its_own_mission(vehicle, tmp_path):
    for round_ in range(2):
        vehicle.state.armed = True
        tick(vehicle, 100.0 + round_ * 10)
        vehicle.state.armed = False
        tick(vehicle, 105.0 + round_ * 10)
    folders = missions(tmp_path)
    assert len(folders) == 2 and all((f / "summary.json").exists() for f in folders)


def test_the_track_is_written_while_armed_at_two_hertz(vehicle, tmp_path):
    vehicle.state.armed = True
    for k in range(100):                                       # 20 Hz for 5 s
        tick(vehicle, 100.0 + k * 0.05, north=k * 0.075)
    folder = missions(tmp_path)[0]
    track = rows(folder / "track.csv")
    assert 9 <= len(track) <= 11 and track[0]["mode"] == "0"          # MODE_MANUAL until told otherwise


def test_statustexts_and_mode_changes_become_events(vehicle, tmp_path):
    vehicle.state.armed = True
    tick(vehicle, 100.0)
    vehicle._statustext("Failsafe: GCS link lost (HOLD)", 2)
    vehicle.set_mode(MODE_AUTO)
    vehicle.set_mode(MODE_HOLD)
    events = rows(missions(tmp_path)[0] / "events.csv")
    got = {(r["level"], r["source"], r["text"]) for r in events}
    assert ("CRITICAL", "vehicle", "Failsafe: GCS link lost (HOLD)") in got
    assert ("INFO", "mode", "mode -> AUTO") in got and ("INFO", "mode", "mode -> HOLD") in got


def test_a_failsafe_reaches_the_event_log(vehicle, tmp_path):
    vehicle.state.armed = True
    vehicle.state.mode = MODE_AUTO
    vehicle.state.pose = make_pose(41.0, 29.0, speed_mps=1.5)
    vehicle.state.gcs_last_seen = 100.0
    tick(vehicle, 100.0)
    vehicle._update_failsafes(200.0)
    texts = [r["text"] for r in rows(missions(tmp_path)[0] / "events.csv")]
    assert "Failsafe: GCS link lost (HOLD)" in texts


def test_bathymetry_samples_are_written_as_they_are_recorded(vehicle, tmp_path):
    vehicle.state.armed = True
    for k in range(8):
        lat, lon = FRAME.to_geodetic(0.0, 1.5 * k)
        vehicle.state.pose = make_pose(lat, lon, speed_mps=1.5, velocity_en=(0.0, 1.5))
        vehicle.gnss.apply_echomap("SDDPT", [f"{8.0 + 0.1 * (k % 2):.1f}", "0.0"], now=float(k))
        vehicle._update_mission_log(float(k) + 0.02)        # the loop's order: the folder first
        vehicle._update_bathymetry(float(k) + 0.02)
    written = rows(missions(tmp_path)[0] / "bathymetry.csv")
    assert len(written) == len(vehicle.bathy_samples) == 8
    assert [r["seq"] for r in written] == [str(s.seq) for s in vehicle.bathy_samples]


def test_the_meta_file_records_versions_and_only_the_parameters_that_were_changed(vehicle, tmp_path):
    vehicle.params["WP_RADIUS"] = 6.5
    vehicle.state.armed = True
    tick(vehicle, 100.0)
    meta = json.loads((missions(tmp_path)[0] / "meta.json").read_text(encoding="utf-8"))
    assert meta["software"]["bahr_link"] == 4 and meta["software"]["autopilot"]
    assert meta["params_changed"] == {"WP_RADIUS": 6.5}
    assert meta["params"]["WP_RADIUS"] == 6.5 and len(meta["params"]) == len(vehicle.params)
    assert "log_dir" not in meta["args"] and meta["args"]["gcs_host"] == "127.0.0.1"


def test_a_failing_disk_is_reported_once_and_never_stops_the_loop(vehicle, tmp_path, monkeypatch):
    vehicle.state.armed = True
    tick(vehicle, 100.0)
    monkeypatch.setattr(vehicle.mission_log._files["track"]._file, "flush",
                        lambda: (_ for _ in ()).throw(OSError("No space left on device")))
    for k in range(10):
        tick(vehicle, 101.0 + k)                                # must not raise
    assert vehicle.sent_texts.count("Log: write failed, logging stopped") == 1        # once, not every tick
    assert vehicle.mission_log.error is not None and not vehicle.mission_log.active


def test_without_a_log_directory_there_is_no_mission_log(tmp_path):
    v = make_vehicle()
    v.state.armed = True
    v._update_mission_log(100.0)
    assert v.mission_log is None and list(tmp_path.iterdir()) == []


def test_shutdown_closes_the_open_mission(tmp_path, monkeypatch):
    stopped = []
    log = SimpleNamespace(stop=lambda reason: stopped.append(reason))

    class StubVehicle:
        def __init__(self, args):
            self.mission_log, self.data_logger = log, None

        def run(self):
            raise SystemExit(0)

        def _save_params(self, force=False):
            pass

    monkeypatch.setattr(vehicle_module, "Vehicle", StubVehicle)
    monkeypatch.setattr("sys.argv", ["bahr-pilot", "--gcs-host", "127.0.0.1"])
    import signal
    previous = signal.getsignal(signal.SIGTERM)
    try:
        assert vehicle_module.main() == 0
    finally:
        signal.signal(signal.SIGTERM, previous)
    assert stopped == ["shutdown"]
