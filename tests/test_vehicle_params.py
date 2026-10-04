"""Phase 22 on the vehicle: PARAM_SET validation, saving, restoring, the boot push
to the STM and MAV_CMD_PREFLIGHT_STORAGE. The store itself is tested in test_params.py."""
from __future__ import annotations

import argparse
import json
from types import SimpleNamespace

import pytest
from pymavlink import mavutil

import bahr_pilot.params as params_module
import bahr_pilot.vehicle as vehicle_module
from bahr_pilot.modes import MODE_AUTO, MODE_HOLD
from bahr_pilot.vehicle import DEFAULT_PARAMS, Vehicle
from tests.poses import make_pose
from tests.test_vehicle import FakeNucleo

ACCEPTED = mavutil.mavlink.MAV_RESULT_ACCEPTED
DENIED = mavutil.mavlink.MAV_RESULT_DENIED
FAILED = mavutil.mavlink.MAV_RESULT_FAILED
UNSUPPORTED = mavutil.mavlink.MAV_RESULT_UNSUPPORTED


def build(tmp_path=None, *, name="params.json", nucleo=False, monkeypatch=None):
    args = argparse.Namespace(
        gcs_host="127.0.0.1", gcs_port=14599, nucleo_port="/dev/null" if nucleo else None, nucleo_baud=115200,
        gnss_port=None, gnss_baud=115200, echomap_port=None, echomap_baud=38400, log_dir=None,
        param_file=None if tmp_path is None else str(tmp_path / name),
    )
    if nucleo:
        monkeypatch.setattr(vehicle_module, "NucleoLink", lambda *a, **k: FakeNucleo())
    vehicle = Vehicle(args)
    vehicle.sent = []
    vehicle.link.mav.param_value_send = lambda *a, **k: vehicle.sent.append((a[0].decode(), a[1]))
    vehicle.texts = []
    vehicle._statustext = lambda text, severity=6: vehicle.texts.append(text)
    vehicle.acks = []
    vehicle._ack = lambda command, result: vehicle.acks.append(result)
    return vehicle


def set_param(vehicle, name, value):
    vehicle._on_param_set(SimpleNamespace(param_id=name.encode(), param_value=value))


def storage(vehicle, action):
    vehicle._handle_command(mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, action, 0, 0, 0, 0, 0)
    return vehicle.acks[-1]


# -- PARAM_SET --------------------------------------------------------------------------------------------

def test_an_accepted_set_is_echoed_and_applied():
    vehicle = build()
    set_param(vehicle, "WP_RADIUS", 6.5)
    assert vehicle.params["WP_RADIUS"] == 6.5 and vehicle.sent[-1] == ("WP_RADIUS", 6.5)
    assert vehicle.texts == []


@pytest.mark.parametrize("name,bad", [("FS_TIMEOUT", float("nan")), ("FS_TIMEOUT", 0.0), ("WP_RADIUS", -3.0),
                                      ("FS_ACTION", 7.0), ("RCMAP_THROTTLE", 2.5), ("MODE1", 99.0)])
def test_a_rejected_set_echoes_the_old_value_and_says_why(name, bad):
    vehicle = build()
    before = vehicle.params[name]
    set_param(vehicle, name, bad)
    assert vehicle.params[name] == before
    assert vehicle.sent[-1] == (name, before)                       # the GCS is shown what the vehicle really holds
    assert len(vehicle.texts) == 1 and vehicle.texts[0].startswith(f"Param {name} rejected: ")


def test_a_nan_timeout_no_longer_defeats_the_link_failsafe():
    """End to end: the parameter write that used to disable the failsafe is refused,
    and the failsafe still fires when the ground station goes quiet."""
    vehicle = build()
    vehicle.state.armed = True
    vehicle.state.mode = MODE_AUTO
    vehicle.state.pose = make_pose(41.0, 29.0, speed_mps=1.5)
    vehicle.state.gcs_last_seen = 100.0
    set_param(vehicle, "FS_TIMEOUT", float("nan"))
    vehicle._update_failsafes(200.0)                                # 100 s of silence
    assert vehicle.state.mode == MODE_HOLD


def test_a_rejected_rc_param_does_not_touch_the_stm(monkeypatch):
    vehicle = build(nucleo=True, monkeypatch=monkeypatch)
    before = len(vehicle.nucleo.rc_maps)
    set_param(vehicle, "RCMAP_THROTTLE", 0.0)                       # would have sent channel -1
    assert len(vehicle.nucleo.rc_maps) == before
    set_param(vehicle, "RCMAP_THROTTLE", 4.0)
    assert len(vehicle.nucleo.rc_maps) == before + 1 and vehicle.nucleo.rc_maps[-1]["throttle_channel"] == 3


def test_unknown_parameters_are_ignored_silently():
    vehicle = build()
    set_param(vehicle, "NOT_A_PARAM", 1.0)
    assert "NOT_A_PARAM" not in vehicle.params and vehicle.sent == [] and vehicle.texts == []


# -- persistence --------------------------------------------------------------------------------------------

def test_tuning_survives_a_restart(tmp_path):
    first = build(tmp_path)
    set_param(first, "ATC_STR_RAT_P", 0.77)
    set_param(first, "FENCE_ENABLE", 1.0)
    first._save_params(force=True)
    second = build(tmp_path)
    assert second.params["ATC_STR_RAT_P"] == 0.77 and second.params["FENCE_ENABLE"] == 1.0
    assert second.params["WP_RADIUS"] == DEFAULT_PARAMS["WP_RADIUS"]
    assert second._boot_notes == []


def test_the_restored_values_are_the_ones_the_controllers_use(tmp_path):
    first = build(tmp_path)
    set_param(first, "ATC_STR_RAT_P", 0.77)
    first._save_params(force=True)
    second = build(tmp_path)
    second.navigator.configure(second.params)
    assert second.navigator.control_config.str_rat.p == 0.77


def test_saving_waits_for_a_quiet_moment_in_the_loop(tmp_path):
    vehicle = build(tmp_path)
    set_param(vehicle, "WP_RADIUS", 6.0)
    vehicle._save_params()                                           # too soon after the change
    assert not (tmp_path / "params.json").exists()
    vehicle.store._dirty_since -= 5.0                                # ...a few seconds later
    vehicle._save_params()
    assert json.loads((tmp_path / "params.json").read_text(encoding="utf-8"))["params"] == {"WP_RADIUS": 6.0}


def test_a_corrupt_file_is_reported_at_boot_and_the_defaults_are_used(tmp_path):
    (tmp_path / "params.json").write_text("{broken", encoding="utf-8")
    vehicle = build(tmp_path)
    assert vehicle.params == DEFAULT_PARAMS
    assert len(vehicle._boot_notes) == 1 and vehicle._boot_notes[0].startswith("Params: saved parameters unreadable")


def test_dropped_entries_are_reported_at_boot(tmp_path):
    (tmp_path / "params.json").write_text(json.dumps({"format": 1, "params": {"FS_TIMEOUT": 0.0, "WP_RADIUS": 6.0}}),
                                          encoding="utf-8")
    vehicle = build(tmp_path)
    assert vehicle.params["WP_RADIUS"] == 6.0 and vehicle.params["FS_TIMEOUT"] == DEFAULT_PARAMS["FS_TIMEOUT"]
    assert any("FS_TIMEOUT" in note for note in vehicle._boot_notes)


def test_a_disk_that_will_not_take_the_file_is_reported_once(tmp_path, monkeypatch):
    vehicle = build(tmp_path)
    set_param(vehicle, "WP_RADIUS", 6.0)
    monkeypatch.setattr(params_module.os, "replace", lambda *a: (_ for _ in ()).throw(OSError("read-only")))
    for _ in range(5):
        vehicle._save_params(force=True)
    assert vehicle.texts.count("Params: cannot save, changes will be lost on restart") == 1
    monkeypatch.undo()
    vehicle._save_params(force=True)                                 # the disk is back
    assert vehicle._write_error_reported is False
    assert (tmp_path / "params.json").exists()


def test_without_a_parameter_file_nothing_is_written_and_nothing_breaks(tmp_path):
    vehicle = build(None)
    set_param(vehicle, "WP_RADIUS", 6.0)
    vehicle._save_params(force=True)
    assert vehicle.params["WP_RADIUS"] == 6.0 and vehicle.texts == []


# -- the STM at boot ---------------------------------------------------------------------------------------------------

def test_a_fresh_pi_does_not_overwrite_the_calibration_saved_in_the_stm(monkeypatch):
    """No parameter file: the Pi only has DEFAULTS for the RC map, and pushing those
    at boot used to erase whatever the STM had stored."""
    vehicle = build(nucleo=True, monkeypatch=monkeypatch)
    assert vehicle.nucleo.rc_maps == []


def test_a_saved_rc_map_is_pushed_to_the_stm_at_boot(tmp_path, monkeypatch):
    first = build(tmp_path, nucleo=True, monkeypatch=monkeypatch)
    set_param(first, "RC3_MIN", 300.0)
    set_param(first, "RCMAP_THROTTLE", 4.0)
    first._save_params(force=True)
    second = build(tmp_path, nucleo=True, monkeypatch=monkeypatch)
    assert len(second.nucleo.rc_maps) == 1
    assert second.nucleo.rc_maps[0]["throttle_channel"] == 3


def test_saved_tuning_that_is_not_an_rc_value_does_not_push_the_rc_map(tmp_path, monkeypatch):
    first = build(tmp_path)
    set_param(first, "ATC_STR_RAT_P", 0.77)
    first._save_params(force=True)
    second = build(tmp_path, nucleo=True, monkeypatch=monkeypatch)
    assert second.nucleo.rc_maps == []


# -- MAV_CMD_PREFLIGHT_STORAGE ------------------------------------------------------------------------------------------

def test_storage_write_saves_now(tmp_path):
    vehicle = build(tmp_path)
    set_param(vehicle, "WP_RADIUS", 6.0)
    assert storage(vehicle, 1) == ACCEPTED
    assert (tmp_path / "params.json").exists()


def test_storage_write_without_a_file_is_denied_and_says_so():
    vehicle = build(None)
    assert storage(vehicle, 1) == DENIED
    assert "Params: no parameter file configured" in vehicle.texts


def test_storage_write_reports_a_failing_disk(tmp_path, monkeypatch):
    vehicle = build(tmp_path)
    set_param(vehicle, "WP_RADIUS", 6.0)
    monkeypatch.setattr(params_module.os, "replace", lambda *a: (_ for _ in ()).throw(OSError("full")))
    assert storage(vehicle, 1) == FAILED


def test_storage_reset_restores_defaults_deletes_the_file_and_resends_the_rc_map(tmp_path, monkeypatch):
    vehicle = build(tmp_path, nucleo=True, monkeypatch=monkeypatch)
    set_param(vehicle, "WP_RADIUS", 6.0)
    set_param(vehicle, "RCMAP_THROTTLE", 4.0)
    vehicle._save_params(force=True)
    sent_before = len(vehicle.nucleo.rc_maps)
    assert storage(vehicle, 2) == ACCEPTED
    assert vehicle.params == DEFAULT_PARAMS and not (tmp_path / "params.json").exists()
    assert len(vehicle.nucleo.rc_maps) == sent_before + 1
    assert vehicle.nucleo.rc_maps[-1]["throttle_channel"] == DEFAULT_PARAMS["RCMAP_THROTTLE"] - 1
    assert "Params: reset to defaults" in vehicle.texts


def test_storage_reload_discards_unsaved_changes(tmp_path):
    vehicle = build(tmp_path)
    set_param(vehicle, "WP_RADIUS", 6.0)
    vehicle._save_params(force=True)
    set_param(vehicle, "WP_RADIUS", 9.0)                              # not saved yet
    set_param(vehicle, "CRUISE_SPEED", 2.5)
    assert storage(vehicle, 0) == ACCEPTED
    assert vehicle.params["WP_RADIUS"] == 6.0 and vehicle.params["CRUISE_SPEED"] == DEFAULT_PARAMS["CRUISE_SPEED"]


def test_an_unknown_storage_action_is_unsupported(tmp_path):
    assert storage(build(tmp_path), 7) == UNSUPPORTED


def test_the_default_parameter_file_lives_in_the_home_directory(monkeypatch):
    import signal
    seen = {}

    class StopAfterParse(Exception):
        pass

    def fake_vehicle(args):
        seen["param_file"] = args.param_file
        raise StopAfterParse

    previous = signal.getsignal(signal.SIGTERM)                   # main() installs a SIGTERM handler
    monkeypatch.setattr(vehicle_module, "Vehicle", fake_vehicle)
    monkeypatch.setattr("sys.argv", ["bahr-pilot", "--gcs-host", "127.0.0.1"])
    try:
        with pytest.raises(StopAfterParse):
            vehicle_module.main()
    finally:
        signal.signal(signal.SIGTERM, previous)
    assert seen["param_file"].replace("\\", "/").endswith("/.bahr_pilot/params.json")


# -- the main loop: every phase's step is wired in, in the order that matters ---------------------------------

class _StopLoop(Exception):
    pass


def test_the_main_loop_runs_estimation_failsafe_and_saving_before_the_motors(monkeypatch):
    """Nothing else tests that run() calls these at all: a step dropped from the loop would
    leave every unit test green. Order: the estimate first (everything uses the pose), then
    home, the mission log (its folder must exist before the first sounding of a mission is
    recorded), the soundings (they need the pose), the failsafes (they may stop the motors),
    the health table (it shows which failsafes are active), the parameter save, and only then the
    motor command."""
    vehicle = build()
    order = []
    for name in ("_update_estimate", "_update_home", "_update_mission_log", "_update_bathymetry", "_update_failsafes",
                 "_update_health", "_save_params", "_apply_rc_mode_switch", "_report_rc_arming", "_motor_command"):
        original = getattr(vehicle, name)

        def wrapper(*args, _name=name, _original=original, **kwargs):
            order.append(_name)
            return _original(*args, **kwargs)
        monkeypatch.setattr(vehicle, name, wrapper)
    monkeypatch.setattr(vehicle.link, "recv_match", lambda blocking=False: None)
    monkeypatch.setattr(vehicle_module.time, "sleep", lambda s: (_ for _ in ()).throw(_StopLoop()))
    with pytest.raises(_StopLoop):
        vehicle.run()
    assert order == ["_update_estimate", "_update_home", "_update_mission_log", "_update_bathymetry", "_update_failsafes",
                     "_update_health", "_save_params", "_apply_rc_mode_switch", "_report_rc_arming", "_motor_command"]


def test_boot_notes_are_sent_with_the_banner(tmp_path, monkeypatch):
    (tmp_path / "params.json").write_text("{broken", encoding="utf-8")
    vehicle = build(tmp_path)
    monkeypatch.setattr(vehicle.link, "recv_match", lambda blocking=False: None)
    monkeypatch.setattr(vehicle_module.time, "sleep", lambda s: (_ for _ in ()).throw(_StopLoop()))
    with pytest.raises(_StopLoop):
        vehicle.run()
    assert any(text.startswith("Params: saved parameters unreadable") for text in vehicle.texts)
