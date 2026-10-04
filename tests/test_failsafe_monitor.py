"""Phase 20 failsafe rules (bahr_pilot/failsafe.py): pure logic, injected clock.

Not to be confused with tests/test_failsafe_c.py, which tests the STM's own
failsafe.c. This module is the Pi-side policy: which problem, which action."""
from __future__ import annotations

import pytest

from bahr_pilot.failsafe import (
    AUTONOMOUS_MODES, Action, BATTERY_HOLD_S, BATTERY_HYSTERESIS_V, CLEAR_HOLD_S, FailsafeMonitor, Inputs,
    FENCE_HOLD_S, POSITION_HOLD_S, Source, action_from_param,
)
from bahr_pilot.modes import MODE_AUTO, MODE_HOLD, MODE_MANUAL

PARAMS = {
    "FS_GCS_ENABLE": 1.0, "FS_TIMEOUT": 3.0, "FS_ACTION": 2.0,
    "BATT_FS_ENABLE": 1.0, "BATT_LOW_VOLT": 10.5, "BATT_CRT_VOLT": 9.6,
    "BATT_FS_LOW_ACT": 2.0, "BATT_FS_CRT_ACT": 2.0, "FS_EKF_ACTION": 1.0,
}


def params(**overrides):
    return {**PARAMS, **overrides}


def inputs(now, *, armed=True, mode=MODE_AUTO, gcs_age=0.0, battery=12.0, position=True, heading=True, fence=None):
    return Inputs(now=now, armed=armed, mode=mode, gcs_age_s=gcs_age, battery_v=battery,
                  position_ok=position, heading_ok=heading, fence_margin_m=fence)


def run(monitor, seconds, step=0.05, p=None, start=0.0, **kw):
    """Feed constant inputs; return the events in order."""
    events = []
    steps = round(seconds / step)
    for k in range(steps + 1):
        events.extend(monitor.update(inputs(start + k * step, **kw), p or PARAMS))
    return events


# -- action mapping ------------------------------------------------------------------------------

@pytest.mark.parametrize("value,expected", [
    (0, Action.REPORT), (1, Action.RTL), (2, Action.HOLD), (3, Action.RTL), (4, Action.HOLD), (5, Action.TERMINATE),
    (7, Action.HOLD), (-1, Action.HOLD), (2.9, Action.HOLD), (float("nan"), Action.HOLD),
    (float("inf"), Action.HOLD), (None, Action.HOLD), ("x", Action.HOLD),
])
def test_arduPilot_action_values_map_to_what_this_vehicle_can_do(value, expected):
    """SmartRTL (3, 4) needs a recorded track: 3 acts as RTL, 4 as HOLD; garbage as HOLD."""
    assert action_from_param(value) == expected


# -- GCS link ---------------------------------------------------------------------------------------

def test_gcs_loss_fires_as_soon_as_the_age_exceeds_the_timeout():
    m = FailsafeMonitor()
    assert m.update(inputs(0.0, gcs_age=2.9), PARAMS) == []
    events = m.update(inputs(0.05, gcs_age=3.1), PARAMS)
    assert [(e.kind, e.source, e.action) for e in events] == [("triggered", Source.GCS_LOST, Action.HOLD)]
    assert events[0].text == "Failsafe: GCS link lost (HOLD)"
    assert m.required_action() == Action.HOLD and m.stops_motors()


def test_a_gcs_that_was_never_heard_cannot_be_lost():
    m = FailsafeMonitor()
    assert run(m, 20.0, gcs_age=None) == [] and m.required_action() is None


def test_gcs_failsafe_can_be_switched_off():
    m = FailsafeMonitor()
    assert run(m, 10.0, p=params(FS_GCS_ENABLE=0.0), gcs_age=99.0) == []


@pytest.mark.parametrize("value,action,stops", [(0, Action.REPORT, False), (1, Action.RTL, False),
                                                (2, Action.HOLD, True), (5, Action.TERMINATE, True)])
def test_fs_action_chooses_what_happens(value, action, stops):
    m = FailsafeMonitor()
    run(m, 1.0, p=params(FS_ACTION=float(value)), gcs_age=99.0)
    assert m.required_action() == action and m.stops_motors() == stops


def test_gcs_failsafe_clears_only_after_the_link_has_been_back_for_a_while():
    m = FailsafeMonitor()
    run(m, 0.5, gcs_age=99.0)
    assert m.required_action() == Action.HOLD
    assert run(m, CLEAR_HOLD_S - 0.3, gcs_age=0.1, start=1.0) == []           # back, but not for long enough
    assert m.required_action() == Action.HOLD
    events = run(m, 0.6, gcs_age=0.1, start=1.0 + CLEAR_HOLD_S - 0.3)
    assert [(e.kind, e.source) for e in events] == [("cleared", Source.GCS_LOST)]
    assert m.required_action() is None


def test_a_flickering_link_does_not_flood_the_operator():
    """Lost, back for 1 s, lost again: one 'triggered', no 'cleared' in between."""
    m = FailsafeMonitor()
    events = run(m, 0.5, gcs_age=99.0)
    events += run(m, 1.0, gcs_age=0.1, start=1.0)
    events += run(m, 1.0, gcs_age=99.0, start=2.1)
    assert [e.kind for e in events] == ["triggered"]


def test_it_can_trigger_again_after_it_cleared():
    m = FailsafeMonitor()
    run(m, 0.5, gcs_age=99.0)
    run(m, 3.0, gcs_age=0.0, start=1.0)
    assert m.required_action() is None
    events = run(m, 0.5, gcs_age=99.0, start=10.0)
    assert [e.kind for e in events] == ["triggered"]


# -- battery -------------------------------------------------------------------------------------------

def test_low_battery_must_persist_before_it_fires():
    """A load sag lasting less than BATTERY_HOLD_S is not a failsafe."""
    m = FailsafeMonitor()
    assert run(m, BATTERY_HOLD_S - 0.2, battery=10.0) == []
    events = run(m, 0.4, battery=10.0, start=BATTERY_HOLD_S - 0.2)
    assert [(e.kind, e.source) for e in events] == [("triggered", Source.BATTERY_LOW)]


def test_a_short_sag_resets_the_battery_timer():
    m = FailsafeMonitor()
    run(m, 2.5, battery=10.0)
    run(m, 0.2, battery=12.0, start=2.6)                  # recovered briefly
    assert run(m, 2.5, battery=10.0, start=3.0) == []     # the clock restarted, 2.5 s < 3 s


def test_battery_failsafe_is_off_unless_enabled_and_needs_a_known_voltage():
    assert run(FailsafeMonitor(), 10.0, p=params(BATT_FS_ENABLE=0.0), battery=5.0) == []
    assert run(FailsafeMonitor(), 10.0, battery=None) == []


def test_battery_hysteresis_stops_the_pack_clearing_its_own_failsafe():
    """With the motors stopped the voltage recovers; just above the threshold is not enough."""
    m = FailsafeMonitor()
    run(m, BATTERY_HOLD_S + 0.5, battery=10.0)
    assert m.required_action() == Action.HOLD
    run(m, 10.0, battery=10.5 + BATTERY_HYSTERESIS_V - 0.05, start=5.0)       # recovered, inside the band
    assert Source.BATTERY_LOW in m.active
    events = run(m, CLEAR_HOLD_S + 0.5, battery=10.5 + BATTERY_HYSTERESIS_V + 0.05, start=20.0)
    assert [(e.kind, e.source) for e in events] == [("cleared", Source.BATTERY_LOW)]


def test_critical_battery_has_its_own_threshold_and_action():
    m = FailsafeMonitor()
    run(m, BATTERY_HOLD_S + 0.5, p=params(BATT_FS_LOW_ACT=1.0, BATT_FS_CRT_ACT=5.0), battery=9.0)
    assert m.active == {Source.BATTERY_LOW: Action.RTL, Source.BATTERY_CRITICAL: Action.TERMINATE}
    assert m.required_action() == Action.TERMINATE                           # the strongest wins


def test_critical_threshold_zero_means_disabled_like_ardupilot():
    m = FailsafeMonitor()
    run(m, 10.0, p=params(BATT_CRT_VOLT=0.0), battery=1.0)
    assert Source.BATTERY_CRITICAL not in m.active and Source.BATTERY_LOW in m.active


# -- position and heading ------------------------------------------------------------------------------

def test_position_loss_fires_after_its_hold_time_in_an_autonomous_mode():
    m = FailsafeMonitor()
    assert run(m, POSITION_HOLD_S - 0.2, position=False) == []
    events = run(m, 0.4, position=False, start=POSITION_HOLD_S - 0.2)
    assert [(e.kind, e.source, e.action) for e in events] == [("triggered", Source.POSITION_LOST, Action.HOLD)]


def test_a_position_dropout_shorter_than_the_hold_time_is_ignored():
    m = FailsafeMonitor()
    assert run(m, 0.8, position=False) == []
    assert run(m, 0.2, position=True, start=1.0) == []
    assert run(m, 0.8, position=False, start=1.3) == []       # the timer restarted


@pytest.mark.parametrize("mode", [MODE_MANUAL, MODE_HOLD])
def test_position_and_heading_are_not_watched_outside_autonomous_modes(mode):
    m = FailsafeMonitor()
    assert run(m, 10.0, mode=mode, position=False, heading=False) == []


@pytest.mark.parametrize("mode", sorted(AUTONOMOUS_MODES))
def test_position_is_watched_in_every_autonomous_mode(mode):
    m = FailsafeMonitor()
    run(m, 2.0, mode=mode, position=False)
    assert Source.POSITION_LOST in m.active


def test_fs_ekf_action_report_only_does_not_stop_the_boat():
    m = FailsafeMonitor()
    run(m, 2.0, p=params(FS_EKF_ACTION=2.0), position=False)
    assert m.active[Source.POSITION_LOST] == Action.REPORT and not m.stops_motors()


def test_fs_ekf_action_zero_disables_both():
    m = FailsafeMonitor()
    assert run(m, 10.0, p=params(FS_EKF_ACTION=0.0), position=False, heading=False) == []


def test_heading_is_judged_by_availability_not_by_an_unbroken_run():
    """Without a gyro the heading is valid ~0.15 s per second: never invalid for 2 s
    straight, but unusable. 20 Hz ticks, valid on 3 of every 20."""
    m = FailsafeMonitor()
    events = []
    for k in range(200):                                      # 10 s
        events += m.update(inputs(k * 0.05, heading=(k % 20) < 3), PARAMS)
    assert [(e.kind, e.source) for e in events] == [("triggered", Source.HEADING_LOST)]


def test_a_mostly_valid_heading_is_fine():
    m = FailsafeMonitor()
    events = []
    for k in range(200):
        events += m.update(inputs(k * 0.05, heading=(k % 20) < 12), PARAMS)     # 60 % available
    assert events == []


def test_heading_is_not_judged_on_less_than_five_seconds_of_data():
    """Waiting for the first GNSS heading after engaging AUTO is not a failure."""
    m = FailsafeMonitor()
    assert run(m, 4.8, heading=False) == []
    events = run(m, 0.5, heading=False, start=4.85)
    assert [e.source for e in events] == [Source.HEADING_LOST]


def test_a_heading_that_appears_after_two_seconds_at_start_up_is_not_a_loss():
    m = FailsafeMonitor()
    events = []
    for k in range(400):                                      # 20 s; the heading arrives at 2 s
        events += m.update(inputs(k * 0.05, heading=k * 0.05 >= 2.0), PARAMS)
    assert events == []


def test_leaving_autonomy_drops_position_and_heading_failsafes_without_claiming_they_cleared():
    """The cause is still there (no position, no heading): saying "cleared" would be false. This is what
    happens right after the failsafe forces HOLD itself: the simulator showed "Failsafe cleared: position
    lost" 0.05 s after "position lost" with the GNSS still down."""
    m = FailsafeMonitor()
    run(m, 6.5, position=False, heading=False)
    assert {Source.POSITION_LOST, Source.HEADING_LOST} <= set(m.active)
    events = m.update(inputs(7.0, mode=MODE_MANUAL, position=False, heading=False), PARAMS)
    assert events == [] and m.active == {} and m.required_action() is None


def test_a_failsafe_whose_rule_is_switched_off_is_dropped_silently_too():
    m = FailsafeMonitor()
    run(m, 1.0, gcs_age=99.0)
    assert Source.GCS_LOST in m.active
    events = m.update(inputs(1.5, gcs_age=99.0), {**PARAMS, "FS_GCS_ENABLE": 0.0})
    assert events == [] and m.active == {}


def test_a_cause_that_really_went_away_is_still_announced_as_cleared():
    m = FailsafeMonitor()
    run(m, 1.0, gcs_age=99.0)
    events = run(m, 3.0, gcs_age=0.1, start=1.05)
    assert [(e.kind, e.text) for e in events] == [("cleared", "Failsafe cleared: GCS link lost")]


# -- general -----------------------------------------------------------------------------------------------

def test_disarming_forgets_everything_silently():
    m = FailsafeMonitor()
    run(m, 1.0, gcs_age=99.0)
    assert m.active
    assert m.update(inputs(2.0, armed=False, gcs_age=99.0), PARAMS) == []
    assert m.active == {} and m.required_action() is None
    events = m.update(inputs(2.05, armed=True, gcs_age=99.0), PARAMS)           # armed again: fresh
    assert [e.kind for e in events] == ["triggered"]


def test_nothing_fires_when_everything_is_fine():
    m = FailsafeMonitor()
    assert run(m, 30.0) == [] and m.required_action() is None and not m.stops_motors()


def test_the_strongest_action_wins():
    m = FailsafeMonitor()
    run(m, 5.0, p=params(FS_ACTION=1.0, FS_EKF_ACTION=1.0), gcs_age=99.0, position=False)
    assert m.active[Source.GCS_LOST] == Action.RTL and m.active[Source.POSITION_LOST] == Action.HOLD
    assert m.required_action() == Action.HOLD and m.stops_motors()


def test_a_changed_parameter_updates_an_active_action():
    m = FailsafeMonitor()
    run(m, 1.0, gcs_age=99.0)
    assert m.required_action() == Action.HOLD
    m.update(inputs(2.0, gcs_age=99.0), params(FS_ACTION=1.0))
    assert m.required_action() == Action.RTL


# -- geofence ------------------------------------------------------------------------------------------

FENCE = {"FENCE_ENABLE": 1.0, "FENCE_ACTION": 1.0, "FENCE_MARGIN": 2.0}


def test_a_breach_fires_after_its_short_hold_with_the_fence_action():
    m = FailsafeMonitor()
    assert run(m, FENCE_HOLD_S - 0.2, p=params(**FENCE), fence=-1.0) == []
    events = run(m, 0.4, p=params(**FENCE), fence=-1.0, start=FENCE_HOLD_S - 0.2)
    assert [(e.kind, e.source, e.action) for e in events] == [("triggered", Source.GEOFENCE, Action.RTL)]
    assert events[0].text == "Failsafe: fence breached (RTL)"


def test_a_graze_of_the_boundary_shorter_than_the_hold_is_ignored():
    m = FailsafeMonitor()
    assert run(m, 0.3, p=params(**FENCE), fence=-0.5) == []
    assert run(m, 0.2, p=params(**FENCE), fence=3.0, start=0.4) == []
    assert run(m, 0.3, p=params(**FENCE), fence=-0.5, start=0.7) == []          # the timer restarted


def test_the_fence_margin_is_the_hysteresis_for_clearing():
    """Just back inside is not enough: the boat has to be FENCE_MARGIN (2 m) inside."""
    m = FailsafeMonitor()
    run(m, 1.0, p=params(**FENCE), fence=-1.0)
    assert Source.GEOFENCE in m.active
    run(m, 10.0, p=params(**FENCE), fence=1.5, start=2.0)                        # inside, but within the margin
    assert Source.GEOFENCE in m.active
    events = run(m, CLEAR_HOLD_S + 0.5, p=params(**FENCE), fence=2.5, start=20.0)
    assert [(e.kind, e.source) for e in events] == [("cleared", Source.GEOFENCE)]


@pytest.mark.parametrize("value,action", [(0, Action.REPORT), (1, Action.RTL), (2, Action.HOLD), (5, Action.TERMINATE)])
def test_fence_action_chooses_what_happens(value, action):
    m = FailsafeMonitor()
    run(m, 1.0, p=params(**{**FENCE, "FENCE_ACTION": float(value)}), fence=-5.0)
    assert m.active[Source.GEOFENCE] == action


def test_the_fence_is_ignored_when_disabled_or_when_no_fence_applies():
    assert run(FailsafeMonitor(), 5.0, p=params(**{**FENCE, "FENCE_ENABLE": 0.0}), fence=-50.0) == []
    assert run(FailsafeMonitor(), 5.0, p=params(**FENCE), fence=None) == []


def test_the_fence_is_watched_in_every_mode_so_manual_driving_is_reported():
    m = FailsafeMonitor()
    run(m, 1.0, p=params(**FENCE), mode=MODE_MANUAL, fence=-5.0)
    assert Source.GEOFENCE in m.active


def test_a_breach_beats_a_milder_failsafe():
    m = FailsafeMonitor()
    run(m, 2.0, p=params(**{**FENCE, "FENCE_ACTION": 2.0, "FS_ACTION": 1.0}), gcs_age=99.0, fence=-5.0)
    assert m.active == {Source.GCS_LOST: Action.RTL, Source.GEOFENCE: Action.HOLD}
    assert m.required_action() == Action.HOLD
