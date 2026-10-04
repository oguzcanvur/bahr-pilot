"""Phase 24 on the vehicle: the health table is fed from the real sources, announced as STATUSTEXT
(and so into the mission log), and sent as the SYS_STATUS sensor bits."""
from __future__ import annotations

import csv
import dataclasses
import time

import pytest
from pymavlink import mavutil

import bahr_pilot.vehicle as vehicle_module
from bahr_pilot import diagnostics as d
from bahr_pilot.diagnostics import HealthMonitor, Level, Part
from bahr_pilot.failsafe import Source
from bahr_pilot.modes import MODE_AUTO
from tests.poses import make_pose
from tests.test_vehicle import FakeNucleo, fake_imu, make_vehicle

MAV = mavutil.mavlink
DT = 0.05
T0 = 1000.0                                                   # the fake clock starts here


class Live:
    """A vehicle whose sensors are all alive, driven on a fake clock. `faults` cuts or bends one
    source at a time; `step()` refreshes everything that is not faulted, then updates the health."""

    def __init__(self, vehicle, with_failsafes=False):
        self.v = vehicle
        self.t = T0
        self.faults: dict[str, object] = {}
        self.frames = 0.0
        self.with_failsafes = with_failsafes

    def step(self, seconds: float):
        end = self.t + seconds
        while self.t < end:
            self._sensors(self.t)
            if self.with_failsafes:
                self.v._update_failsafes(self.t)
            self.v._update_health(self.t)
            self.t += DT

    def _sensors(self, now):
        v, f, nucleo = self.v, self.faults, self.v.nucleo
        if "stm" not in f:
            tel = nucleo.telemetry
            tel.last_update = now
            tel.rc_link_up = "rc" not in f
            tel.battery_valid = f.get("battery", 12.0) is not None
            tel.battery_mv = int(1000 * (f.get("battery") or 12.0))
            if "imu" not in f:
                self.frames += 50.0 * DT
                nucleo.imu_frames = int(self.frames)
                nucleo.imu = dataclasses.replace(fake_imu(gyro_valid="gyro" not in f), rx_time=now, t_pi=now)
        nucleo.clock.resets = f.get("stm_resets", 0)
        if "gnss" not in f:
            v.state.pose = make_pose(41.0, 29.0, heading_valid="heading" not in f, speed_mps=0.0)
        else:
            v.state.pose = None
        v.state.gcs_last_seen = now - f.get("gcs_silence", 0.1) if "gcs_never" not in f else 0.0
        if "sonar" not in f and abs(now - round(now)) < DT / 2:                       # one sounding a second
            v.gnss.apply_echomap("SDDPT", ["8.0", "0.0"], now=now)
            v._update_bathymetry(now)


@pytest.fixture
def vehicle(tmp_path):
    v = make_vehicle(log_dir=str(tmp_path))
    v.nucleo = FakeNucleo()
    v.args.echomap_port = "fake"                              # the sonar part exists (its reader thread starts only in __init__)
    v.health = HealthMonitor(v._health_parts())
    v.sent = []
    original = v.link.mav.statustext_send
    v.link.mav.statustext_send = lambda severity, text: (v.sent.append((severity, bytes(text).decode())),
                                                          original(severity, text))[1]
    return v


@pytest.fixture
def live(vehicle):
    return Live(vehicle)


# -- which parts exist ----------------------------------------------------------------------------------

def test_a_bare_vehicle_has_no_stm_or_sonar_parts():
    assert make_vehicle()._health_parts() == {Part.GNSS, Part.HEADING, Part.GCS, Part.STORAGE, Part.LOOP}


def test_an_stm_and_a_sonar_add_their_parts(vehicle):
    assert vehicle._health_parts() == set(Part) - {Part.FAILSAFE}


# -- a healthy boat is silent -------------------------------------------------------------------------------

def test_a_healthy_boat_says_nothing_in_a_minute(live):
    live.step(60.0)
    assert live.v.sent == [] and live.v.health.worst() == Level.OK
    status = live.v.health.sys_status(live.v._health_snapshot(live.t))
    assert status.health == status.present                            # every present sensor is healthy


# -- every source is connected -------------------------------------------------------------------------------

FAULTS = [
    ({"gnss": True}, (3, "GNSS: no position")),
    ({"heading": True}, (3, "heading: no valid heading")),
    ({"stm": True}, (3, "STM link: silent")),
    ({"imu": True}, (3, "IMU: no data")),
    ({"gyro": True}, (4, "IMU: data invalid")),
    ({"rc": True}, (4, "RC: no signal")),
    ({"battery": 10.0}, (4, "battery: 10.0 V low")),
    ({"battery": None}, (4, "battery: no reading")),
    ({"sonar": True}, (4, "sonar: no data for")),
    ({"gcs_silence": 3.0}, (4, "GCS link: silent 3 s")),
]


@pytest.mark.parametrize("fault,expected", FAULTS, ids=[next(iter(f)) + str(f[next(iter(f))]) for f, _ in FAULTS])
def test_each_source_reaches_the_operator(live, fault, expected):
    live.step(5.0)
    live.faults = fault
    live.step(d.STARTUP_GRACE_S + 10.0)
    severity, prefix = expected
    mine = [(s, text) for s, text in live.v.sent if text.startswith(prefix)]
    assert len(mine) == 1 and mine[0][0] == severity, live.v.sent


def test_one_sounding_a_second_is_not_a_quiet_sonar(live):
    live.step(15.0)
    assert live.v.sent == [] and live.v.health.level(Part.SONAR) == Level.OK


def test_a_stored_parameter_error_and_a_log_failure_are_reported(live):
    live.step(5.0)
    live.v.store.write_error = "read-only file system"
    live.step(d.STARTUP_GRACE_S + 5.0)
    assert (4, "storage: params not saved") in live.v.sent
    live.v.mission_log.error = "OSError: disk full"
    live.step(d.EVENT_MIN_INTERVAL_S + d.ESCALATE_HOLD_S + 1.0)
    assert (3, "storage: mission log failed") in live.v.sent


def test_a_stalled_loop_is_reported(live):
    live.step(5.0)
    live.v.loop_timer.record(live.t, 0.2)
    live.step(d.STARTUP_GRACE_S + 3.0)
    assert (3, "main loop: stalled 200 ms") in live.v.sent


def test_an_stm_restart_is_reported(live):
    live.step(5.0)
    live.faults = {"stm_resets": 1}
    live.step(d.STARTUP_GRACE_S + 3.0)
    assert (4, "STM link: restarted (1)") in live.v.sent


def test_a_lone_spike_does_not_make_the_sonar_suspect_but_a_run_of_them_does(vehicle):
    v = vehicle

    def sound(t, depth):
        v.gnss.apply_echomap("SDDPT", [f"{depth}", "0.0"], now=t)
        v._update_bathymetry(t + 0.02)
        return v._health_snapshot(t + 0.02).sonar_suspect

    t = T0
    for _ in range(8):
        t += 1.0
        assert sound(t, 8.0) is False
    assert sound(t + 1.0, 40.0) is False                              # one spike
    assert sound(t + 2.0, 12.0) is False
    assert sound(t + 3.0, 70.0) is True                               # three unconfirmed readings of the last five
    for k in range(4, 12):
        sound(t + k, 8.0)
    assert v._health_snapshot(t + 12.0).sonar_suspect is False        # and it clears once they have left the window


def test_start_up_soundings_are_not_suspicion(vehicle):
    """The filter has nothing to compare the first readings with; that is not a fault."""
    v = vehicle
    for k in range(3):
        v.gnss.apply_echomap("SDDPT", ["8.0", "0.0"], now=T0 + k)
        v._update_bathymetry(T0 + k + 0.02)
    assert v._health_snapshot(T0 + 3.0).sonar_suspect is False


# -- the snapshot itself ----------------------------------------------------------------------------------------

def test_the_snapshot_reads_each_source_in_the_right_units(live):
    live.step(2.0)
    live.v.state.mode = MODE_AUTO
    live.v.params["FENCE_ENABLE"] = 1.0
    snap = live.v._health_snapshot(live.t)
    assert snap.autonomous and snap.fence_enabled and not snap.fence_breached
    assert snap.battery_v == pytest.approx(12.0) and snap.stm_age_s == pytest.approx(DT, abs=1e-6)
    assert snap.imu_age_s == pytest.approx(DT, abs=1e-6) and snap.rc_link_up is True
    assert snap.gcs_age_s == pytest.approx(0.1 + DT, abs=1e-6) and snap.log_configured and snap.log_error is None
    assert snap.sonar_age_s is not None and 0.0 <= snap.sonar_age_s < 1.1 and snap.failsafes == ()


def test_unknown_things_are_none_not_zero(vehicle):
    snap = vehicle._health_snapshot(T0)
    assert snap.stm_age_s is None and snap.imu_age_s is None and snap.imu_data_valid is None
    assert snap.rc_link_up is None and snap.battery_v is None and snap.sonar_age_s is None
    assert snap.gcs_age_s is None and snap.position is None and snap.heading_valid is False


def test_a_stale_stm_makes_the_rc_state_unknown_not_down(live):
    live.step(2.0)
    live.faults = {"stm": True}
    live.step(3.0)
    snap = live.v._health_snapshot(live.t)
    assert snap.stm_age_s > 1.0 and snap.rc_link_up is None


def test_active_failsafes_and_a_breached_fence_are_in_the_snapshot(vehicle):
    vehicle.failsafe.active = {Source.GCS_LOST: object(), Source.GEOFENCE: object()}
    snap = vehicle._health_snapshot(T0)
    assert set(snap.failsafes) == {"GCS link lost", "fence breached"} and snap.fence_breached


def test_a_lost_gcs_link_shows_in_the_table_as_an_error_and_the_failsafe_row(vehicle):
    live = Live(vehicle, with_failsafes=True)
    vehicle.state.armed = True
    vehicle.state.mode = MODE_AUTO
    live.step(5.0)
    live.faults = {"gcs_silence": 10.0}
    live.step(d.STARTUP_GRACE_S + 3.0)
    assert vehicle.health.level(Part.GCS) == Level.ERROR and vehicle.health.level(Part.FAILSAFE) == Level.ERROR
    assert (3, "GCS link: link lost") in vehicle.sent
    assert not any(text.startswith("failsafe:") for _, text in vehicle.sent)       # failsafe.py says its own


# -- announcements reach the mission log -----------------------------------------------------------------------------

def test_an_announcement_is_also_an_event_in_the_mission_log(live, tmp_path):
    v = live.v
    v.state.armed = True
    live.step(2.0)
    v._update_mission_log(live.t)
    live.faults = {"gnss": True}
    live.step(d.STARTUP_GRACE_S + 3.0)
    v.mission_log.stop("test")
    folder = next((tmp_path / "missions").iterdir())
    with open(folder / "events.csv", encoding="utf-8", newline="") as handle:
        events = list(csv.DictReader(handle))
    assert ("ERROR", "GNSS: no position") in {(e["level"], e["text"]) for e in events}


def test_the_health_table_at_arming_is_recorded_in_meta_json(live, tmp_path):
    import json
    v = live.v
    live.faults = {"battery": None}
    live.step(5.0)
    v.state.armed = True
    v._update_mission_log(live.t)
    v.mission_log.stop("test")
    meta = json.loads((next((tmp_path / "missions").iterdir()) / "meta.json").read_text(encoding="utf-8"))
    assert set(meta["health"]) == {p.value for p in d.Part if p in v.health.present}
    assert meta["health"]["battery"]["level"] == "WARNING" and meta["health"]["GNSS"]["level"] == "OK"


# -- SYS_STATUS ---------------------------------------------------------------------------------------------------------

def sys_status_sent(vehicle):
    sent = []
    vehicle.link.mav.sys_status_send = lambda *args: sent.append(args)
    vehicle.send_telemetry()
    return sent[-1]


def test_sys_status_carries_the_health_bits_and_the_loop_load(live):
    live.step(15.0)
    v = live.v
    for k in range(40):
        v.loop_timer.record(live.t - 4.0 + k * 0.1, 0.01)          # 10 ms of work every 100 ms: 10 % load
    present, enabled, health, load = sys_status_sent(v)[:4]
    assert present == health and present & MAV.MAV_SYS_STATUS_SENSOR_GPS and enabled & MAV.MAV_SYS_STATUS_SENSOR_3D_GYRO
    assert load == pytest.approx(100, abs=5)


def test_a_failed_part_clears_its_health_bit_in_the_message_the_ground_station_gets(live):
    live.step(15.0)
    live.faults = {"gnss": True}
    live.step(3.0)
    present, enabled, health, _ = sys_status_sent(live.v)[:4]
    gps = MAV.MAV_SYS_STATUS_SENSOR_GPS
    assert present & gps and enabled & gps and not health & gps
    assert health & MAV.MAV_SYS_STATUS_SENSOR_3D_GYRO                   # the rest is untouched


def test_the_voltage_fields_of_sys_status_are_unchanged(live):
    live.step(15.0)
    args = sys_status_sent(live.v)
    assert args[4] == 12000 and args[5] == -1 and args[6] == -1 and args[7:] == (0, 0, 0, 0, 0, 0)


# -- the loop ----------------------------------------------------------------------------------------------------------

class _StopLoop(Exception):
    pass


def test_the_main_loop_measures_its_own_work_and_not_the_sleep(monkeypatch):
    vehicle = make_vehicle()
    calls = []
    monkeypatch.setattr(vehicle.loop_timer, "record", lambda start, busy: calls.append((start, busy)))
    monkeypatch.setattr(vehicle.link, "recv_match", lambda blocking=False: None)
    slept = []

    def sleep(seconds):
        slept.append(seconds)
        raise _StopLoop

    monkeypatch.setattr(vehicle_module.time, "sleep", sleep)
    before = time.monotonic()
    with pytest.raises(_StopLoop):
        vehicle.run()
    assert len(calls) == 1 and before <= calls[0][0] <= time.monotonic()
    assert 0.0 <= calls[0][1] < 5.0 and slept == [1.0 / vehicle_module.TICK_HZ]    # the sleep is outside the measurement


# -- the STM clock ------------------------------------------------------------------------------------------------------

def test_stm_freshness_is_immune_to_the_wall_clock_jumping(vehicle, monkeypatch):
    """Pi has no RTC: NTP or a GNSS time sync steps time.time() by years. The STM stamp is monotonic."""
    vehicle.nucleo.rc(3, link=True, override=True, age=0.2)
    assert vehicle._rc_telemetry_fresh()
    monkeypatch.setattr(vehicle_module.time, "time", lambda: 5.0e9)
    assert vehicle._rc_telemetry_fresh()
    vehicle.nucleo.rc(3, link=True, override=True, age=2.0)
    assert not vehicle._rc_telemetry_fresh()
