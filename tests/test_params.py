"""Phase 22: parameter ranges and persistence (bahr_pilot/params.py)."""
from __future__ import annotations

import json
import math

import pytest

from bahr_pilot import params as params_module
from bahr_pilot.params import FILE_FORMAT, SPECS, ParamSpec, ParamStore, check_value
from bahr_pilot.vehicle import DEFAULT_PARAMS


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


def make_store(tmp_path, clock=None, name="params.json", defaults=None):
    return ParamStore(defaults or DEFAULT_PARAMS, path=tmp_path / name, clock=clock or Clock())


# -- the table itself -----------------------------------------------------------------------------

def test_every_parameter_has_a_range_and_every_range_a_parameter():
    assert set(SPECS) == set(DEFAULT_PARAMS)


@pytest.mark.parametrize("name", sorted(DEFAULT_PARAMS))
def test_every_default_is_valid_for_its_own_spec(name):
    """A default the validator would refuse could never be set back."""
    assert check_value(SPECS[name], DEFAULT_PARAMS[name]) is None, (name, DEFAULT_PARAMS[name])


def test_rc_map_and_mode_switch_are_owned_by_the_stm():
    stm = {n for n, s in SPECS.items() if s.owner == "stm"}
    assert {"RCMAP_ROLL", "RCMAP_THROTTLE", "RCMAP_ARM", "MODE_CH", "MODE1", "MODE6", "RC3_MIN", "RC8_REVERSED"} <= stm
    assert "FS_TIMEOUT" not in stm and "ATC_STR_RAT_P" not in stm


def test_a_store_refuses_to_exist_with_a_parameter_that_has_no_range():
    with pytest.raises(ValueError, match="without a range"):
        ParamStore({**DEFAULT_PARAMS, "BRAND_NEW": 1.0}, SPECS)


# -- what a value must be to be accepted ----------------------------------------------------------------------

@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf"), None, "12", True, [3.0], {}])
def test_garbage_is_never_a_value(value):
    assert check_value(SPECS["FS_TIMEOUT"], value) is not None


@pytest.mark.parametrize("name,value,ok", [
    ("FS_TIMEOUT", 3.0, True), ("FS_TIMEOUT", 1.0, True), ("FS_TIMEOUT", 0.0, False), ("FS_TIMEOUT", 120.01, False),
    ("RCMAP_THROTTLE", 3, True), ("RCMAP_THROTTLE", 0, False), ("RCMAP_THROTTLE", 17, False),
    ("RCMAP_THROTTLE", 2.5, False),                                    # a channel number is a whole number
    ("FS_ACTION", 2, True), ("FS_ACTION", 6, False), ("FS_ACTION", -1, False),
    ("FS_EKF_ACTION", 2, True), ("FS_EKF_ACTION", 3, False),
    ("AHRS_ORIENTATION", 4, True), ("AHRS_ORIENTATION", 3, False),     # in range but not an offered rotation
    ("MODE1", 10, True), ("MODE1", 99, False), ("MODE1", 7, False),
    ("FENCE_ENABLE", 1, True), ("FENCE_ENABLE", 2, False),
    ("FENCE_TYPE", 6, True), ("FENCE_TYPE", 16, False),
    ("ATC_STR_RAT_FILT", 0.0, False), ("ATC_STR_RAT_FILT", 0.5, True),
    ("RC1_MIN", 172, True), ("RC1_MIN", 2501, False), ("RC1_REVERSED", 0.4, False),
])
def test_ranges_integers_and_enumerations(name, value, ok):
    assert (check_value(SPECS[name], value) is None) is ok


def test_an_integer_that_arrives_as_a_float_is_fine():
    """MAVLink parameters are all REAL32 on the wire."""
    assert check_value(SPECS["RCMAP_THROTTLE"], 3.0) is None
    assert check_value(SPECS["RCMAP_THROTTLE"], 3.0000001) is None


# -- set() ------------------------------------------------------------------------------------------------------------

def test_an_accepted_set_changes_the_live_table(tmp_path):
    store = make_store(tmp_path)
    result = store.set("WP_RADIUS", 5.5)
    assert (result.accepted, result.value) == (True, 5.5) and store.values["WP_RADIUS"] == 5.5


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0, 1e9])
def test_a_rejected_set_keeps_the_old_value_and_reports_it(tmp_path, bad):
    store = make_store(tmp_path)
    result = store.set("FS_TIMEOUT", bad)
    assert result.accepted is False and result.value == DEFAULT_PARAMS["FS_TIMEOUT"] and result.reason
    assert store.values["FS_TIMEOUT"] == DEFAULT_PARAMS["FS_TIMEOUT"]


def test_a_nan_fs_timeout_can_no_longer_switch_the_link_failsafe_off(tmp_path):
    """The failure this phase exists to prevent: age > NaN is never true."""
    store = make_store(tmp_path)
    store.set("FS_TIMEOUT", float("nan"))
    assert math.isfinite(store.values["FS_TIMEOUT"]) and store.values["FS_TIMEOUT"] >= 1.0


def test_integers_are_rounded_to_whole_numbers(tmp_path):
    store = make_store(tmp_path)
    store.set("RCMAP_THROTTLE", 4.0000001)
    assert store.values["RCMAP_THROTTLE"] == 4.0 and isinstance(store.values["RCMAP_THROTTLE"], float)


def test_an_unknown_parameter_is_not_created(tmp_path):
    store = make_store(tmp_path)
    result = store.set("NOT_A_PARAM", 1.0)
    assert result.accepted is False and "NOT_A_PARAM" not in store.values


def test_the_live_table_is_one_dict_shared_with_the_vehicle(tmp_path):
    store = make_store(tmp_path)
    table = store.values
    store.set("WP_RADIUS", 7.0)
    assert table is store.values and table["WP_RADIUS"] == 7.0


# -- writing ------------------------------------------------------------------------------------------------------------

def test_the_file_is_written_only_after_a_quiet_moment(tmp_path):
    clock = Clock()
    store = make_store(tmp_path, clock)
    store.set("WP_RADIUS", 5.0)
    assert store.flush() is False and not store.path.exists()           # too soon: a burst of changes may follow
    clock.now += 0.5
    store.set("CRUISE_SPEED", 2.0)
    assert store.flush() is False
    clock.now += 1.0
    assert store.flush() is True and store.path.exists()
    assert store.flush() is False                                       # nothing left to write


def test_force_writes_immediately(tmp_path):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    assert store.flush(force=True) is True


def test_only_values_that_differ_from_the_defaults_are_saved(tmp_path):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    store.set("CRUISE_SPEED", DEFAULT_PARAMS["CRUISE_SPEED"])           # "changed" to what it already was
    store.flush(force=True)
    data = json.loads(store.path.read_text(encoding="utf-8"))
    assert data == {"format": FILE_FORMAT, "params": {"WP_RADIUS": 5.0}}


def test_setting_a_value_back_to_its_default_removes_it_from_the_file(tmp_path):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    store.flush(force=True)
    store.set("WP_RADIUS", DEFAULT_PARAMS["WP_RADIUS"])
    store.flush(force=True)
    assert json.loads(store.path.read_text(encoding="utf-8"))["params"] == {}


def test_a_store_without_a_path_never_writes(tmp_path):
    store = ParamStore(DEFAULT_PARAMS, path=None)
    store.set("WP_RADIUS", 5.0)
    assert store.flush(force=True) is False and list(tmp_path.iterdir()) == []


def test_missing_directories_are_created(tmp_path):
    store = ParamStore(DEFAULT_PARAMS, path=tmp_path / "a" / "b" / "params.json", clock=Clock())
    store.set("WP_RADIUS", 5.0)
    assert store.flush(force=True) and store.path.exists()


def test_no_temporary_file_is_left_behind(tmp_path):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    store.flush(force=True)
    assert sorted(p.name for p in tmp_path.iterdir()) == ["params.json"]


def test_a_failed_write_keeps_the_old_file_and_is_retried(tmp_path, monkeypatch):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    store.flush(force=True)
    before = store.path.read_text(encoding="utf-8")
    store.set("WP_RADIUS", 9.0)
    real_replace = params_module.os.replace
    monkeypatch.setattr(params_module.os, "replace", lambda *a: (_ for _ in ()).throw(OSError("disk full")))
    assert store.flush(force=True) is False and "disk full" in store.write_error
    assert store.path.read_text(encoding="utf-8") == before            # untouched: a half-written file is the worst case
    monkeypatch.setattr(params_module.os, "replace", real_replace)
    assert store.flush(force=True) is True and store.write_error is None
    assert json.loads(store.path.read_text(encoding="utf-8"))["params"]["WP_RADIUS"] == 9.0


# -- loading -------------------------------------------------------------------------------------------------------------

def test_values_survive_a_restart(tmp_path):
    first = make_store(tmp_path)
    first.set("ATC_STR_RAT_P", 0.77)
    first.set("FENCE_ENABLE", 1.0)
    first.set("RCMAP_THROTTLE", 4.0)
    first.flush(force=True)
    second = make_store(tmp_path)
    assert second.load() == []
    assert second.values["ATC_STR_RAT_P"] == 0.77 and second.values["FENCE_ENABLE"] == 1.0
    assert second.values["RCMAP_THROTTLE"] == 4.0
    assert second.values["WP_RADIUS"] == DEFAULT_PARAMS["WP_RADIUS"]    # untouched stay default
    assert second.loaded_names == {"ATC_STR_RAT_P", "FENCE_ENABLE", "RCMAP_THROTTLE"}


def test_a_new_software_default_reaches_vehicles_that_never_changed_that_parameter(tmp_path):
    first = make_store(tmp_path)
    first.set("WP_RADIUS", 5.0)
    first.flush(force=True)
    newer_defaults = {**DEFAULT_PARAMS, "CRUISE_SPEED": 2.2}              # the update changes a default
    second = ParamStore(newer_defaults, path=first.path)
    second.load()
    assert second.values["CRUISE_SPEED"] == 2.2 and second.values["WP_RADIUS"] == 5.0


def test_no_file_means_defaults_and_no_complaint(tmp_path):
    store = make_store(tmp_path)
    assert store.load() == [] and store.values == DEFAULT_PARAMS and not store.loaded_names


@pytest.mark.parametrize("content", ["{not json", "", "[1, 2, 3]", '{"format": 1}', '{"format": 1, "params": [1]}'])
def test_a_corrupt_file_is_set_aside_and_the_defaults_used(tmp_path, content):
    path = tmp_path / "params.json"
    path.write_text(content, encoding="utf-8")
    store = make_store(tmp_path)
    warnings = store.load()
    assert len(warnings) == 1 and "unreadable" in warnings[0] and "params.json.corrupt" in warnings[0]
    assert store.values == DEFAULT_PARAMS
    assert not path.exists() and (tmp_path / "params.json.corrupt").read_text(encoding="utf-8") == content


def test_bad_entries_are_dropped_one_by_one_and_the_rest_is_kept(tmp_path):
    path = tmp_path / "params.json"
    path.write_text(json.dumps({"format": FILE_FORMAT, "params": {
        "WP_RADIUS": 6.0,                       # good
        "FS_TIMEOUT": float("nan"),             # NaN
        "FS_ACTION": 9,                         # not an action
        "CRUISE_SPEED": 99.0,                   # out of range
        "RCMAP_THROTTLE": 2.5,                  # not whole
        "MODE1": "AUTO",                        # wrong type
        "GONE_PARAM": 1.0,                      # not a parameter any more
        "ATC_STR_RAT_P": 0.5,                   # good
    }}), encoding="utf-8")
    store = make_store(tmp_path)
    warnings = store.load()
    assert store.values["WP_RADIUS"] == 6.0 and store.values["ATC_STR_RAT_P"] == 0.5
    for name in ("FS_TIMEOUT", "FS_ACTION", "CRUISE_SPEED", "RCMAP_THROTTLE", "MODE1"):
        assert store.values[name] == DEFAULT_PARAMS[name], name
    assert len(warnings) == 6 and any("GONE_PARAM" in w and "unknown" in w for w in warnings)
    assert path.exists()                                                  # a partially good file is kept in place


def test_a_file_from_another_format_is_read_with_a_warning(tmp_path):
    (tmp_path / "params.json").write_text(json.dumps({"format": 99, "params": {"WP_RADIUS": 6.0}}), encoding="utf-8")
    store = make_store(tmp_path)
    warnings = store.load()
    assert store.values["WP_RADIUS"] == 6.0 and any("format 99" in w for w in warnings)


def test_has_saved_tells_which_owner_changed_things(tmp_path):
    first = make_store(tmp_path)
    first.set("ATC_STR_RAT_P", 0.77)
    first.flush(force=True)
    second = make_store(tmp_path)
    second.load()
    assert second.has_saved("pi") and not second.has_saved("stm")
    third = make_store(tmp_path, name="other.json")
    third.set("RCMAP_THROTTLE", 4.0)
    third.flush(force=True)
    fourth = make_store(tmp_path, name="other.json")
    fourth.load()
    assert fourth.has_saved("stm")


# -- reset -----------------------------------------------------------------------------------------------------------------

def test_reset_restores_the_defaults_and_deletes_the_file(tmp_path):
    store = make_store(tmp_path)
    store.set("WP_RADIUS", 5.0)
    store.flush(force=True)
    store.set("CRUISE_SPEED", 2.0)
    store.reset()
    assert store.values == DEFAULT_PARAMS and not store.path.exists() and not store.loaded_names
    assert store.flush(force=True) is False                              # nothing pending after a reset


def test_spec_is_immutable():
    with pytest.raises(Exception):
        SPECS["WP_RADIUS"].minimum = 0.0                                  # type: ignore[misc]
    assert isinstance(SPECS["WP_RADIUS"], ParamSpec)
