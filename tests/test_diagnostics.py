"""The health table: levels, debounce, announcements, SYS_STATUS bits (Phase 24).

The monitor is pure, so everything runs on a made-up clock. `healthy()` is a snapshot in
which nothing is wrong; each test breaks one thing."""
from __future__ import annotations

import dataclasses

import pytest
from pymavlink import mavutil

from bahr_pilot import diagnostics as d
from bahr_pilot.diagnostics import HealthMonitor, Level, LoopTimer, Part, Snapshot
from bahr_pilot.estimator import EstimateStatus, EstimatorConfig
from bahr_pilot.failsafe import BATTERY_HYSTERESIS_V
from bahr_pilot.sonar import SonarConfig
from bahr_pilot.vehicle import DEFAULT_PARAMS, RC_TELEMETRY_FRESH_S, TICK_HZ

ALL_PARTS = set(Part)
PARAMS = {**DEFAULT_PARAMS, "BATT_CRT_VOLT": 9.0}          # the shipped default has no critical level
MAV = mavutil.mavlink
DT = 0.05


def healthy(now: float = 0.0, **changes) -> Snapshot:
    base = Snapshot(now=now, autonomous=False, position=EstimateStatus.OK, heading_valid=True, imu_age_s=0.02,
                    imu_frames=None, imu_data_valid=True, stm_age_s=0.05, stm_resets=0, rc_link_up=True,
                    battery_v=12.0, sonar_age_s=0.5, sonar_suspect=False, gcs_age_s=0.2, log_configured=True)
    return dataclasses.replace(base, **changes)


def monitor(parts=ALL_PARTS, warm: bool = True) -> HealthMonitor:
    """A monitor that has already been running healthy for longer than the start-up grace (so
    announcements are live), or, with warm=False, one that has just been switched on."""
    mon = HealthMonitor(parts)
    mon.test_t = 0.0                                          # where the next run() continues
    if warm:
        run(mon, d.STARTUP_GRACE_S + 1.0)
    return mon


def run(mon: HealthMonitor, seconds: float, start: float | None = None, params=PARAMS, **changes):
    """Feed the same snapshot every DT for `seconds`; returns (transitions, end time). Continues
    where the monitor's previous run() ended unless `start` is given."""
    out, t = [], mon.test_t if start is None else start
    steps = round(seconds / DT)
    for _ in range(steps + 1):
        out += mon.update(healthy(t, **changes), params)
        t += DT
    mon.test_t = t
    return out, t - DT


# -- nothing wrong ------------------------------------------------------------------------------------

def test_a_healthy_vehicle_says_nothing_and_every_row_is_ok():
    mon = monitor()
    transitions, _ = run(mon, 30.0)
    assert transitions == [] and mon.worst() == Level.OK
    assert {row.part for row in mon.rows()} == ALL_PARTS and all(row.level == Level.OK for row in mon.rows())


# -- one fault per part: the level and the words ----------------------------------------------------------

FAULTS = [
    (dict(position=None), Part.GNSS, Level.ERROR, "no position"),
    (dict(position=EstimateStatus.NONE), Part.GNSS, Level.ERROR, "no position"),
    (dict(position=EstimateStatus.DEAD_RECKONING), Part.GNSS, Level.WARNING, "dead reckoning"),
    (dict(heading_valid=False), Part.HEADING, Level.ERROR, "no valid heading"),
    (dict(imu_age_s=None), Part.IMU, Level.ERROR, "no data"),
    (dict(imu_age_s=0.6), Part.IMU, Level.ERROR, "no data"),
    (dict(imu_data_valid=False), Part.IMU, Level.WARNING, "data invalid"),
    (dict(stm_age_s=None), Part.STM, Level.ERROR, "no telemetry"),
    (dict(stm_age_s=2.0), Part.STM, Level.ERROR, "silent 2 s"),
    (dict(rc_link_up=False), Part.RC, Level.WARNING, "no signal"),
    (dict(battery_v=None), Part.BATTERY, Level.WARNING, "no reading"),
    (dict(battery_v=10.0), Part.BATTERY, Level.WARNING, "10.0 V low"),
    (dict(battery_v=8.7), Part.BATTERY, Level.ERROR, "8.7 V critical"),
    (dict(sonar_age_s=None), Part.SONAR, Level.WARNING, "no depth data"),
    (dict(sonar_age_s=6.0), Part.SONAR, Level.WARNING, "no data for 6 s"),
    (dict(sonar_suspect=True), Part.SONAR, Level.WARNING, "readings suspect"),
    (dict(gcs_age_s=3.0), Part.GCS, Level.WARNING, "silent 3 s"),
    (dict(failsafes=("GCS link lost",)), Part.GCS, Level.ERROR, "link lost"),
    (dict(log_error="OSError: disk full"), Part.STORAGE, Level.ERROR, "mission log failed"),
    (dict(params_write_error="read-only file system"), Part.STORAGE, Level.WARNING, "params not saved"),
    (dict(loop_busy_max_s=0.06), Part.LOOP, Level.WARNING, "slow cycle 60 ms"),
    (dict(loop_busy_max_s=0.15), Part.LOOP, Level.ERROR, "stalled 150 ms"),
]


@pytest.mark.parametrize("fault,part,level,words", FAULTS, ids=[f"{p.name}-{w}" for _, p, _, w in FAULTS])
def test_each_fault_is_reported_on_its_own_part_with_the_right_level(fault, part, level, words):
    mon = monitor()
    transitions, _ = run(mon, 2.0, **fault)
    assert len(transitions) == 1 and transitions[0].part == part              # nothing else lit up
    assert transitions[0].new == level and words in transitions[0].reason
    assert mon.level(part) == level


def test_a_failsafe_is_shown_in_the_table_but_not_announced_again():
    """failsafe.py already says "Failsafe: GCS link lost (HOLD)"; a second message would only be noise."""
    mon = monitor()
    transitions, _ = run(mon, 3.0, failsafes=("battery low", "position lost"))
    assert mon.level(Part.FAILSAFE) == Level.ERROR
    assert [r.reason for r in mon.rows() if r.part == Part.FAILSAFE] == ["battery low, position lost"]
    assert all(t.part != Part.FAILSAFE for t in transitions)


@pytest.mark.parametrize("edge", [
    dict(imu_age_s=d.IMU_STALE_S), dict(stm_age_s=d.STM_STALE_S), dict(sonar_age_s=d.SONAR_STALE_S),
    dict(gcs_age_s=d.GCS_STALE_S), dict(loop_busy_max_s=d.LOOP_WARN_BUSY_S), dict(battery_v=PARAMS["BATT_LOW_VOLT"]),
])
def test_a_value_exactly_on_its_limit_is_still_fine(edge):
    mon = monitor()
    transitions, _ = run(mon, 3.0, **edge)
    assert transitions == [] and mon.worst() == Level.OK


def test_a_part_the_vehicle_does_not_have_is_not_in_the_table_and_never_complains():
    mon = monitor({Part.GNSS, Part.HEADING, Part.GCS, Part.STORAGE, Part.LOOP})
    transitions, _ = run(mon, 5.0, stm_age_s=None, imu_age_s=None, battery_v=None, sonar_age_s=None, rc_link_up=False)
    assert transitions == [] and {r.part for r in mon.rows()} == {Part.GNSS, Part.HEADING, Part.GCS, Part.STORAGE,
                                                                   Part.LOOP, Part.FAILSAFE}


def test_a_gcs_that_was_never_heard_is_not_a_lost_link():
    """Same rule as the failsafe: a boat on RC with no ground station is not in trouble."""
    mon = monitor()
    transitions, _ = run(mon, 10.0, gcs_age_s=None)
    assert transitions == [] and mon.level(Part.GCS) == Level.OK


# -- debounce and rate limit -----------------------------------------------------------------------------------

def test_a_fault_shorter_than_the_hold_is_ignored_and_a_longer_one_is_reported():
    mon = monitor()
    short, t = run(mon, d.ESCALATE_HOLD_S - 0.2, position=None)
    assert short == []
    back, t = run(mon, 5.0, start=t + DT)                                  # healed before the hold ran out
    assert back == [] and mon.level(Part.GNSS) == Level.OK
    longer, _ = run(mon, d.ESCALATE_HOLD_S + 0.2, start=t + DT, position=None)
    assert len(longer) == 1 and mon.level(Part.GNSS) == Level.ERROR


def test_a_worse_level_is_shown_after_one_second_and_not_before():
    """Literal seconds on purpose: a test written in terms of ESCALATE_HOLD_S moves with the constant
    and would keep passing if someone set it to 0 (a mutation check found exactly that)."""
    mon = monitor()
    short, t = run(mon, 0.8, position=None)
    assert short == [] and mon.level(Part.GNSS) == Level.OK
    run(mon, 5.0, start=t + DT)
    mon = monitor()
    seen, _ = run(mon, 1.3, position=None)
    assert len(seen) == 1 and mon.level(Part.GNSS) == Level.ERROR


def test_a_better_level_is_shown_after_three_seconds_and_not_before():
    mon = monitor()
    run(mon, 1.3, position=None)
    run(mon, 2.5)
    assert mon.level(Part.GNSS) == Level.ERROR                               # 2.5 s of calm is not enough
    run(mon, 1.0)
    assert mon.level(Part.GNSS) == Level.OK                                  # 3.5 s is


def test_a_part_speaks_at_most_every_five_seconds_in_literal_seconds():
    mon = monitor()
    first, _ = run(mon, 1.3, position=None)                                  # announced at about 1.0 s
    assert len(first) == 1
    early, _ = run(mon, 3.6)                                                 # the table has recovered by now ...
    assert early == [] and mon.level(Part.GNSS) == Level.OK                  # ... but it is only ~5 s after the first
    later, _ = run(mon, 1.5)
    assert [(x.text, x.severity) for x in later] == [("GNSS: recovered", 6)]


def test_the_startup_grace_is_ten_seconds_in_literal_seconds():
    mon = monitor(warm=False)
    early, _ = run(mon, 9.0, position=None)
    assert early == []
    later, _ = run(mon, 2.0, position=None)
    assert [x.text for x in later] == ["GNSS: no position"]


def test_the_report_arrives_after_the_hold_not_before():
    mon = monitor()
    first, t0 = None, mon.test_t
    t = t0
    while t < t0 + 3.0:
        for _ in mon.update(healthy(t, position=None), PARAMS):
            first = t - t0 if first is None else first
        t += DT
    assert first is not None and d.ESCALATE_HOLD_S <= first < d.ESCALATE_HOLD_S + 2 * DT


def test_recovery_needs_a_longer_calm_than_a_fault_and_a_blip_of_ok_does_not_count():
    mon = monitor()
    _, t = run(mon, 2.0, position=None)
    assert mon.level(Part.GNSS) == Level.ERROR
    blip, t = run(mon, d.RECOVER_HOLD_S - 1.0, start=t + DT)                # OK for less than the recovery hold ...
    _, t = run(mon, 0.5, start=t + DT, position=None)                       # ... and then bad again
    assert mon.level(Part.GNSS) == Level.ERROR and blip == []
    # a recovery also waits for the part's last announcement to be EVENT_MIN_INTERVAL_S old
    transitions, _ = run(mon, d.RECOVER_HOLD_S + d.EVENT_MIN_INTERVAL_S, start=t + DT)
    assert [(x.part, x.new) for x in transitions] == [(Part.GNSS, Level.OK)] and mon.level(Part.GNSS) == Level.OK


def test_a_flapping_sensor_announces_at_most_once_per_interval_and_settles_on_the_truth():
    mon = monitor()
    announced, t0 = [], mon.test_t
    t = t0
    while t < t0 + 120.0:
        bad = int((t - t0) / 1.5) % 2 == 0                                   # 1.5 s bad, 1.5 s good, for two minutes
        announced += mon.update(healthy(t, position=EstimateStatus.NONE if bad else EstimateStatus.OK), PARAMS)
        t += DT
    assert announced and all(tr.part == Part.GNSS for tr in announced)
    assert len(announced) <= 120.0 / d.EVENT_MIN_INTERVAL_S + 1
    run(mon, 20.0, start=t)                                                  # then it stays good
    assert mon.level(Part.GNSS) == Level.OK


def test_consecutive_announcements_of_one_part_are_never_closer_than_the_interval():
    mon = monitor()
    stamps, t0 = [], mon.test_t
    t = t0
    while t < t0 + 200.0:
        bad = (int((t - t0) / 3.2) % 2 == 0)
        for _ in mon.update(healthy(t, heading_valid=not bad), PARAMS):
            stamps.append(t)
        t += DT
    assert len(stamps) > 4
    assert all(b - a >= d.EVENT_MIN_INTERVAL_S - 1e-9 for a, b in zip(stamps, stamps[1:]))


def test_the_announcement_describes_the_table_not_every_flap_on_the_way():
    """A part that went bad and good again while it was not allowed to speak is not mentioned at all."""
    mon = monitor()
    first, _ = run(mon, 1.2, position=None)
    assert [x.new for x in first] == [Level.ERROR]                           # announced; the part is now muted for 5 s
    calm, _ = run(mon, 3.2)                                                  # the table recovers ...
    assert calm == [] and mon.level(Part.GNSS) == Level.OK                   # ... but the part may not speak yet
    bad, _ = run(mon, 6.0, position=None)                                    # and it is bad again before its turn
    assert bad == [] and mon.level(Part.GNSS) == Level.ERROR                 # nothing to add to what the operator knows


# -- start-up ---------------------------------------------------------------------------------------------

def test_nothing_is_announced_in_the_startup_grace_but_the_table_is_already_current():
    mon = monitor(warm=False)
    early, _ = run(mon, d.STARTUP_GRACE_S - 1.0, position=None)
    assert early == [] and mon.level(Part.GNSS) == Level.ERROR               # SYS_STATUS must not claim "healthy" at boot
    assert mon.sys_status(healthy(mon.test_t, position=None)).health & GPS == 0
    later, _ = run(mon, 3.0, position=None)
    assert [(t.part, t.text) for t in later] == [(Part.GNSS, "GNSS: no position")]


def test_a_fault_that_heals_within_the_grace_is_never_mentioned():
    mon = monitor(warm=False)
    run(mon, 4.0, position=None)
    quiet, _ = run(mon, 40.0)
    assert quiet == [] and mon.level(Part.GNSS) == Level.OK


def test_a_fault_that_persists_past_the_grace_is_announced_once_right_after_it():
    mon = monitor(warm=False)
    stamps, t = [], 0.0
    while t < 40.0:
        stamps += [t for _ in mon.update(healthy(t, heading_valid=False), PARAMS)]
        t += DT
    assert len(stamps) == 1 and d.STARTUP_GRACE_S <= stamps[0] < d.STARTUP_GRACE_S + 2 * DT


def test_a_change_of_reason_at_the_same_level_is_updated_silently():
    mon = monitor()
    run(mon, 2.0, sonar_age_s=None)
    assert [r.reason for r in mon.rows() if r.part == Part.SONAR] == ["no depth data"]
    transitions, _ = run(mon, 2.0, start=2.1, sonar_suspect=True)
    assert transitions == [] and [r.reason for r in mon.rows() if r.part == Part.SONAR] == ["readings suspect"]


def test_the_announcement_text_and_severity_follow_the_level():
    mon = monitor()
    warn, t = run(mon, 2.0, position=EstimateStatus.DEAD_RECKONING)
    err, t = run(mon, 2.0, start=t + DT + d.EVENT_MIN_INTERVAL_S, position=None)
    ok, _ = run(mon, d.RECOVER_HOLD_S + d.EVENT_MIN_INTERVAL_S + 1.0, start=t + DT + d.EVENT_MIN_INTERVAL_S)
    assert [(x.text, x.severity) for x in warn] == [("GNSS: dead reckoning", 4)]
    assert [(x.text, x.severity, x.old, x.new) for x in err] == [("GNSS: no position", 3, Level.WARNING, Level.ERROR)]
    assert [(x.text, x.severity, x.old, x.new) for x in ok] == [("GNSS: recovered", 6, Level.ERROR, Level.OK)]


def test_every_announcement_fits_a_statustext_even_with_ridiculous_numbers():
    extremes = [dict(battery_v=59.9), dict(stm_age_s=99999.0), dict(sonar_age_s=99999.0),
                dict(gcs_age_s=99999.0), dict(loop_busy_max_s=9999.0), dict(imu_age_s=99999.0),
                dict(failsafes=("GCS link lost", "battery low", "battery critical", "position lost", "heading lost",
                                "fence breached"))]
    params = {**PARAMS, "BATT_LOW_VOLT": 60.0, "BATT_CRT_VOLT": 60.0}
    for fault in extremes:
        mon = monitor()
        transitions, _ = run(mon, 3.0, params=params, **fault)
        assert transitions
        for tr in transitions:
            assert len(tr.text.encode("ascii")) <= 50, tr.text
    # and the table keeps the whole failsafe list, which is not announced and so not cut
    assert len(", ".join(extremes[-1]["failsafes"])) > 50


# -- battery ------------------------------------------------------------------------------------------------

def test_the_battery_recovers_only_above_the_threshold_by_the_failsafe_hysteresis():
    low = PARAMS["BATT_LOW_VOLT"]
    mon = monitor()
    _, t = run(mon, 2.0, battery_v=low - 0.1)
    assert mon.level(Part.BATTERY) == Level.WARNING
    _, t = run(mon, 6.0, start=t + DT, battery_v=low + BATTERY_HYSTERESIS_V - 0.05)     # above low, inside the band
    assert mon.level(Part.BATTERY) == Level.WARNING
    _, t = run(mon, 6.0, start=t + DT, battery_v=low + BATTERY_HYSTERESIS_V + 0.05)
    assert mon.level(Part.BATTERY) == Level.OK


def test_the_critical_level_needs_the_hysteresis_to_clear_too():
    crit = PARAMS["BATT_CRT_VOLT"]
    mon = monitor()
    _, t = run(mon, 2.0, battery_v=crit - 0.5)
    assert mon.level(Part.BATTERY) == Level.ERROR
    _, t = run(mon, 6.0, start=t + DT, battery_v=crit + BATTERY_HYSTERESIS_V - 0.05)    # above it, inside the band
    assert mon.level(Part.BATTERY) == Level.ERROR
    _, t = run(mon, 6.0, start=t + DT, battery_v=crit + BATTERY_HYSTERESIS_V + 0.05)    # out of the band, still under "low"
    assert mon.level(Part.BATTERY) == Level.WARNING


def test_critical_wins_over_low_and_a_zero_critical_level_means_none():
    mon = monitor()
    run(mon, 2.0, battery_v=8.5)
    assert mon.level(Part.BATTERY) == Level.ERROR
    mon = monitor()
    run(mon, 2.0, params={**PARAMS, "BATT_CRT_VOLT": 0.0}, battery_v=1.0)
    assert mon.level(Part.BATTERY) == Level.WARNING                                       # low, never critical


def test_losing_the_reading_forgets_the_hysteresis_band():
    """After "no reading" a voltage just above the threshold is simply fine again."""
    low = PARAMS["BATT_LOW_VOLT"]
    mon = monitor()
    _, t = run(mon, 2.0, battery_v=low - 0.5)
    _, t = run(mon, 2.0, start=t + DT, battery_v=None)
    _, t = run(mon, 12.0, start=t + DT, battery_v=low + 0.1)
    assert mon.level(Part.BATTERY) == Level.OK


# -- IMU frame rate ---------------------------------------------------------------------------------------

def feed_imu(mon, hz: float, seconds: float, start: float | None = None, frames0: int = 0, age: float = 0.01):
    t = mon.test_t if start is None else start
    out, frames, end = [], float(frames0), t + seconds
    while t <= end:
        out += mon.update(healthy(t, imu_frames=int(frames), imu_age_s=age), PARAMS)
        frames += hz * DT
        t += DT
    mon.test_t = t
    return out, t, int(frames)


def test_a_full_rate_imu_is_fine():
    mon = monitor()
    out, _, _ = feed_imu(mon, 50.0, 10.0)
    assert out == [] and mon.level(Part.IMU) == Level.OK


def test_lost_frames_are_noticed_even_though_the_latest_one_is_fresh():
    """UART errors drop frames silently; the age alone looks fine."""
    mon = monitor()
    out, _, _ = feed_imu(mon, 30.0, 10.0)
    assert [(t.part, t.new) for t in out] == [(Part.IMU, Level.WARNING)] and "rate 30 Hz" in out[0].reason


def test_no_rate_is_judged_before_a_full_window_has_been_seen():
    mon = monitor()
    out, _, _ = feed_imu(mon, 10.0, d.IMU_RATE_WINDOW_S - 0.3)
    assert out == [] and mon.level(Part.IMU) == Level.OK


def test_a_restarted_frame_counter_is_not_a_negative_rate():
    mon = monitor()
    _, t, n = feed_imu(mon, 50.0, 5.0)
    out, _, _ = feed_imu(mon, 50.0, 6.0, start=t, frames0=0)                # the STM link object was recreated
    assert out == [] and mon.level(Part.IMU) == Level.OK


def test_no_frame_counter_means_no_rate_check():
    mon = monitor()
    out, _ = run(mon, 5.0, imu_frames=None)
    assert out == []


# -- STM restart ------------------------------------------------------------------------------------------

def test_an_stm_restart_is_visible_for_a_while_and_then_forgotten():
    mon = monitor()
    _, t = run(mon, 3.0)
    during, t = run(mon, 3.0, start=t + DT, stm_resets=1)
    assert [(x.part, x.new, x.reason) for x in during] == [(Part.STM, Level.WARNING, "restarted (1)")]
    after, _ = run(mon, d.STM_RESTART_HOLD_S + d.RECOVER_HOLD_S + 2.0, start=t + DT, stm_resets=1)
    assert [(x.part, x.new) for x in after] == [(Part.STM, Level.OK)]


def test_a_second_restart_is_reported_again():
    mon = monitor()
    _, t = run(mon, 3.0)
    _, t = run(mon, 3.0, start=t + DT, stm_resets=1)
    _, t = run(mon, d.STM_RESTART_HOLD_S + d.RECOVER_HOLD_S + 2.0, start=t + DT, stm_resets=1)
    again, _ = run(mon, 8.0, start=t + DT, stm_resets=2)
    assert [(x.new, x.reason) for x in again] == [(Level.WARNING, "restarted (2)")]


def test_restarts_that_happened_before_the_monitor_started_are_not_news():
    mon = monitor(warm=False)
    transitions, _ = run(mon, 2 * d.STARTUP_GRACE_S, stm_resets=7)           # long enough that an alarm would be announced
    assert transitions == [] and mon.level(Part.STM) == Level.OK


# -- SYS_STATUS -----------------------------------------------------------------------------------------------

def settled(mon, **changes):
    _, t = run(mon, 3.0, **changes)
    return healthy(t, **changes)


G, A = MAV.MAV_SYS_STATUS_SENSOR_3D_GYRO, MAV.MAV_SYS_STATUS_SENSOR_3D_ACCEL
GPS, LASER = MAV.MAV_SYS_STATUS_SENSOR_GPS, MAV.MAV_SYS_STATUS_SENSOR_LASER_POSITION
RC, MOTOR, BATT = (MAV.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, MAV.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS,
                   MAV.MAV_SYS_STATUS_SENSOR_BATTERY)
AHRS, XY, LOG, FENCE = (MAV.MAV_SYS_STATUS_AHRS, MAV.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL,
                        MAV.MAV_SYS_STATUS_LOGGING, MAV.MAV_SYS_STATUS_GEOFENCE)
EVERYTHING = G | A | GPS | LASER | RC | MOTOR | BATT | AHRS | XY | LOG | FENCE


def test_a_healthy_vehicle_reports_every_bit_present_and_healthy():
    mon = monitor()
    status = mon.sys_status(settled(mon))
    assert status.present == EVERYTHING and status.health == EVERYTHING
    assert status.enabled == EVERYTHING & ~A & ~XY & ~FENCE      # accel unused, nothing autonomous, no fence


def test_enabled_depends_on_what_the_vehicle_is_doing():
    mon = monitor()
    snap = dataclasses.replace(settled(mon), autonomous=True, fence_enabled=True)
    status = mon.sys_status(snap)
    assert status.enabled & XY and status.enabled & FENCE and not status.enabled & A


@pytest.mark.parametrize("fault,sick", [
    (dict(position=None), GPS | XY),
    (dict(position=EstimateStatus.DEAD_RECKONING), GPS | XY),
    (dict(heading_valid=False), AHRS | XY),
    (dict(imu_age_s=None), G | A),
    (dict(stm_age_s=None), MOTOR),
    (dict(rc_link_up=False), RC),
    (dict(battery_v=None), BATT),
    (dict(battery_v=10.0), BATT),
    (dict(sonar_age_s=None), LASER),
    (dict(sonar_suspect=True), LASER),
])
def test_an_unhealthy_part_clears_exactly_its_own_health_bits(fault, sick):
    mon = monitor()
    status = mon.sys_status(dataclasses.replace(settled(mon, **fault), autonomous=True))
    assert status.health == EVERYTHING & ~sick
    assert status.present == EVERYTHING                          # unhealthy is not absent


def test_a_geofence_breach_clears_its_bit_and_a_log_error_clears_the_logging_bit():
    mon = monitor()
    status = mon.sys_status(dataclasses.replace(settled(mon), fence_enabled=True, fence_breached=True))
    assert status.health == EVERYTHING & ~FENCE
    mon = monitor()
    status = mon.sys_status(settled(mon, log_error="OSError"))
    assert status.health == EVERYTHING & ~LOG


def test_a_failed_parameter_save_does_not_make_the_logging_bit_unhealthy():
    mon = monitor()
    status = mon.sys_status(settled(mon, params_write_error="read-only"))
    assert status.health & LOG


def test_logging_is_only_present_when_a_log_directory_was_given():
    mon = monitor()
    status = mon.sys_status(settled(mon, log_configured=False))
    assert not status.present & LOG and not status.enabled & LOG and not status.health & LOG


def test_a_vehicle_without_an_stm_or_sonar_sets_none_of_their_bits():
    mon = monitor({Part.GNSS, Part.HEADING, Part.GCS, Part.STORAGE, Part.LOOP})
    status = mon.sys_status(settled(mon))
    for bit in (G, A, RC, MOTOR, BATT, LASER):
        assert not status.present & bit and not status.enabled & bit and not status.health & bit
    assert status.present & GPS and status.present & AHRS


@pytest.mark.parametrize("fault", [dict(), dict(position=None), dict(stm_age_s=None, imu_age_s=None, battery_v=None),
                                   dict(log_error="x", failsafes=("battery low",))])
def test_the_bit_fields_are_consistent_and_fit_the_message(fault):
    mon = monitor()
    status = mon.sys_status(dataclasses.replace(settled(mon, **fault), autonomous=True, fence_enabled=True), 400)
    assert status.enabled & ~status.present == 0 and status.health & ~status.present == 0
    for field in (status.present, status.enabled, status.health):
        assert 0 <= field < 2 ** 32
    link = mavutil.mavlink_connection("udpout:127.0.0.1:1", source_system=1)
    message = link.mav.sys_status_encode(status.present, status.enabled, status.health, status.load,
                                         12000, -1, -1, 0, 0, 0, 0, 0, 0)
    decoded = link.mav.decode(bytearray(message.pack(link.mav)))
    assert (decoded.onboard_control_sensors_present, decoded.onboard_control_sensors_enabled,
            decoded.onboard_control_sensors_health, decoded.load) == (status.present, status.enabled,
                                                                      status.health, 400)


def test_the_load_is_clamped_to_the_field():
    mon = monitor()
    assert mon.sys_status(settled(mon), -5).load == 0 and mon.sys_status(settled(mon), 5000).load == 1000


# -- the table as a document -----------------------------------------------------------------------------------

def test_the_table_for_meta_json_lists_every_part_with_level_and_reason():
    mon = monitor()
    run(mon, 2.0, battery_v=10.0, position=None)
    table = mon.table()
    assert table["GNSS"] == {"level": "ERROR", "reason": "no position"}
    assert table["battery"] == {"level": "WARNING", "reason": "10.0 V low"}
    assert table["IMU"] == {"level": "OK", "reason": ""} and set(table) == {p.value for p in ALL_PARTS}


# -- the loop timer -----------------------------------------------------------------------------------------------

def test_loop_load_is_busy_time_over_elapsed_time():
    timer = LoopTimer()
    for k in range(60):                                          # 10 ms of work every 60 ms
        timer.record(k * 0.06, 0.01)
    assert timer.load_permille() == pytest.approx(167, abs=3) and timer.max_busy_s() == pytest.approx(0.01)


def test_loop_load_needs_two_records_and_is_clamped():
    timer = LoopTimer()
    assert timer.load_permille() == 0 and timer.max_busy_s() == 0.0
    timer.record(0.0, 0.5)
    assert timer.load_permille() == 0
    timer.record(0.1, 0.5)
    timer.record(0.2, 0.5)
    assert timer.load_permille() == 1000                          # busy for longer than the period: capped


def test_a_long_iteration_leaves_the_window_after_the_window():
    timer = LoopTimer(window_s=5.0)
    timer.record(0.0, 0.3)
    for k in range(1, 100):
        timer.record(k * 0.1, 0.01)
    assert timer.max_busy_s() == pytest.approx(0.01)              # the 0.3 s stall was 9.9 s ago
    timer.record(9.95, 0.3)
    assert timer.max_busy_s() == pytest.approx(0.3)


def test_a_negative_busy_time_counts_as_zero_and_cannot_hide_real_work():
    timer = LoopTimer()
    timer.record(0.0, -1.0)
    assert timer.max_busy_s() == 0.0
    timer.record(0.1, 0.05)
    timer.record(0.2, 0.05)
    assert timer.load_permille() == 250                          # 0 + 0.05 busy over 0.2 s, not clamped up from -0.95


def test_two_records_at_the_same_instant_are_not_a_division_by_zero():
    timer = LoopTimer()
    timer.record(5.0, 0.1)
    timer.record(5.0, 0.1)
    assert timer.load_permille() == 0


# -- the thresholds are not independent guesses -------------------------------------------------------------

def test_the_limits_are_tied_to_the_constants_they_come_from():
    assert d.STM_STALE_S == RC_TELEMETRY_FRESH_S                  # the vehicle's own "STM telemetry is fresh"
    assert d.SONAR_STALE_S == SonarConfig().gap_s                 # when the sonar filter forgets its history
    assert d.LOOP_ERROR_BUSY_S == EstimatorConfig().imu_timeout_s
    assert d.LOOP_WARN_BUSY_S == pytest.approx(1.0 / TICK_HZ)
    assert d.GCS_STALE_S < DEFAULT_PARAMS["FS_TIMEOUT"]           # the warning comes before the failsafe
    assert d.IMU_STALE_S > 1.0 / d.IMU_NOMINAL_HZ * 5             # several frames, not one
