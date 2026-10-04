"""Navigator geometry and acceptance-radius behaviour."""
from __future__ import annotations

import math

import pytest

from bahr_pilot import geo
from bahr_pilot.control import ControlConfig
from bahr_pilot.modes import MODE_GUIDED
from bahr_pilot.navigation import Navigator
from bahr_pilot.nucleo_link import PULSE_NEUTRAL_US
from bahr_pilot.state import VehicleState
from tests.poses import make_pose


def test_ranges_are_the_ellipsoidal_ones():
    # 0.001 deg of latitude at 41 N is 111.054 m on the WGS84 ellipsoid (the
    # old spherical formula said 111.195); full reference set in test_geo.py.
    assert geo.distance_m(41.0, 29.0, 41.001, 29.0) == pytest.approx(111.054, abs=0.001)
    assert geo.wrap_180(geo.bearing_deg(41.0, 29.0, 41.001, 29.0)) == pytest.approx(0.0, abs=1e-6)
    assert geo.bearing_deg(41.0, 29.0, 41.0, 29.001) == pytest.approx(90.0, abs=0.01)


def _state_near_target(distance_north_m: float) -> tuple[VehicleState, Navigator]:
    state = VehicleState()
    state.pose = make_pose(41.0, 29.0, heading_deg=0.0)
    state.mode = MODE_GUIDED
    nav = Navigator()
    nav.guided_target = geo.LocalFrame(41.0, 29.0).to_geodetic(0.0, distance_north_m)
    return state, nav


def test_step_uses_the_given_wp_radius():
    """WP_RADIUS used to be a hardcoded 3.0 in navigation.py; changing the
    parameter from BAHR-GCS had no effect."""
    state, nav = _state_near_target(2.0)
    assert nav.step(state, 0.5, wp_radius_m=3.0)[2] is True
    assert nav.step(state, 0.5, wp_radius_m=1.0)[2] is False


def _pulses_towards(east_m: float, north_m: float, heading_deg: float = 0.0):
    """One navigator step for a boat at the origin facing `heading_deg`, with a
    GUIDED target east/north of it."""
    state = VehicleState()
    state.pose = make_pose(41.0, 29.0, heading_deg=heading_deg, speed_mps=1.5)
    state.mode = MODE_GUIDED
    nav = Navigator()
    nav.guided_target = geo.LocalFrame(41.0, 29.0).to_geodetic(east_m, north_m)
    m1, m2, reached = nav.step(state, 0.5, 3.0)
    assert not reached
    return m1, m2


def test_target_on_the_right_speeds_up_the_left_motor():
    """Motor 1 = LEFT. Heading north with the target due east, the boat must
    turn right: left motor faster than right. The sides used to be swapped,
    which steers the boat away from its target."""
    left, right = _pulses_towards(100.0, 0.0)
    assert left > right
    left, right = _pulses_towards(-100.0, 0.0)
    assert right > left
    left, right = _pulses_towards(0.0, 100.0)
    assert left == right  # dead ahead


def test_both_motors_drive_forward_when_the_target_is_ahead():
    left, right = _pulses_towards(0.0, 100.0)
    assert left == right and left > PULSE_NEUTRAL_US


def test_a_target_behind_slows_the_boat_down_while_it_turns():
    """With the steering gains zeroed the throttle alone shows the cornering
    cut: 0.5 ahead, x 0.35 (the floor) with the target behind. (With steering on,
    the mix de-saturation shrinks both motors anyway, which would hide it.)"""
    def throttle_pulses(east, north):
        state = VehicleState()
        state.pose = make_pose(41.0, 29.0, heading_deg=0.0, speed_mps=1.5)
        state.mode = MODE_GUIDED
        nav = Navigator()
        nav.configure(_params(ATC_STR_RAT_P=0.0, ATC_STR_RAT_I=0.0, ATC_STR_RAT_FF=0.0))
        nav.guided_target = geo.LocalFrame(41.0, 29.0).to_geodetic(east, north)
        m1, m2, _ = nav.step(state, 0.5, 3.0)
        assert m1 == m2                                          # no steering at all
        return m1 - PULSE_NEUTRAL_US
    assert throttle_pulses(0.0, 100.0) == pytest.approx(250, abs=1)             # 500 us x 0.5
    assert throttle_pulses(0.0, -100.0) == pytest.approx(250 * 0.35, abs=1)     # cut to the floor
    assert throttle_pulses(100.0, 0.0) == pytest.approx(250 * 0.35, abs=1)      # 90 deg off: also the floor
    mid = throttle_pulses(100.0, 100.0)                                         # 45 deg off: 1 - 45/90 = 0.5
    assert mid == pytest.approx(250 * 0.5, abs=1)


def test_step_holds_neutral_without_position():
    state, nav = _state_near_target(50.0)
    state.pose = make_pose(None, None)
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)
    state.pose = None                       # the estimator has not run yet
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)


def test_dead_reckoned_position_still_navigates_but_a_lost_one_does_not():
    from bahr_pilot.estimator import EstimateStatus
    state, nav = _state_near_target(50.0)
    state.pose = make_pose(41.0, 29.0, status=EstimateStatus.DEAD_RECKONING)
    assert nav.step(state, 0.5, 3.0)[0] != PULSE_NEUTRAL_US
    state.pose = make_pose(41.0, 29.0, status=EstimateStatus.NONE)
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)


def test_raw_receiver_fields_are_never_used_for_steering():
    """state.lat/lon are the receiver's raw values; only the estimated pose counts."""
    state, nav = _state_near_target(50.0)
    state.pose = None
    state.lat, state.lon, state.heading_valid = 41.0, 29.0, True
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)


def test_no_valid_heading_means_hold_still_but_arrival_still_counts():
    state, nav = _state_near_target(50.0)
    state.pose = make_pose(41.0, 29.0, heading_valid=False)
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)

    state, nav = _state_near_target(2.0)    # already inside the acceptance radius
    state.pose = make_pose(41.0, 29.0, heading_valid=False)
    assert nav.step(state, 0.5, 3.0)[2] is True   # reaching a point needs no heading


# -- legs and path following (Phase 11) -----------------------------------------------------------

FRAME = geo.LocalFrame(41.0, 29.0)


def at(east, north):
    return FRAME.to_geodetic(east, north)


def auto_navigator(*waypoints, seq=1):
    """Mission slot 0 = home at the origin, then the given (east, north) waypoints."""
    nav = Navigator()
    nav.mission = [at(0.0, 0.0)] + [at(*w) for w in waypoints]
    nav.mission_seq = seq
    state = VehicleState()
    state.mode = 10  # MODE_AUTO
    return nav, state


def place(state, east, north, heading=0.0, speed=1.5):
    lat, lon = at(east, north)
    state.pose = make_pose(lat, lon, heading_deg=heading, speed_mps=speed)


def test_the_first_leg_starts_where_the_boat_is_not_at_home():
    nav, state = auto_navigator((0.0, 100.0), (100.0, 100.0))
    place(state, 30.0, 0.0)                         # 30 m east of home when AUTO begins
    nav.step(state, 0.5, 3.0)
    g = nav.last_guidance
    assert g.cross_track_m == pytest.approx(0.0, abs=1e-6)      # the leg begins at the boat
    assert g.along_track_m == pytest.approx(0.0, abs=1e-6)
    assert g.leg_length_m == pytest.approx(math.hypot(30.0, 100.0), abs=0.05)


def test_later_legs_follow_the_planned_line_from_the_previous_waypoint():
    nav, state = auto_navigator((0.0, 100.0), (100.0, 100.0), seq=2)
    place(state, 10.0, 105.0, heading=90.0)         # 5 m NORTH of the eastbound leg (left of it)
    nav.step(state, 0.5, 3.0)
    g = nav.last_guidance
    assert g.leg_length_m == pytest.approx(100.0, abs=0.05)
    assert g.along_track_m == pytest.approx(10.0, abs=0.05)
    assert g.cross_track_m == pytest.approx(-5.0, abs=0.05)      # left of travel = negative
    # steering back towards the line means turning right (towards south of east)
    assert 90.0 < g.desired_heading_deg < 180.0


def test_the_leg_is_kept_while_the_target_stays_and_rebuilt_when_it_changes():
    nav, state = auto_navigator((0.0, 100.0), (100.0, 100.0))
    place(state, 0.0, 0.0)
    nav.step(state, 0.5, 3.0)
    first = nav._leg
    place(state, 0.0, 20.0)                         # moved on: same leg, same origin
    nav.step(state, 0.5, 3.0)
    assert nav._leg is first and nav.last_guidance.along_track_m == pytest.approx(20.0, abs=0.05)
    nav.mission_seq = 2                             # next waypoint
    nav.step(state, 0.5, 3.0)
    assert nav._leg is not first


def test_guided_and_rtl_legs_start_at_the_boat():
    nav = Navigator()
    state = VehicleState()
    state.mode = MODE_GUIDED
    nav.guided_target = at(0.0, 100.0)
    place(state, 20.0, 10.0)
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance.cross_track_m == pytest.approx(0.0, abs=1e-6)
    state.mode, state.home_lat, state.home_lon = 11, *at(-50.0, 0.0)          # RTL
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance.leg_length_m == pytest.approx(math.hypot(70.0, 10.0), abs=0.05)


def test_a_mission_waypoint_reached_gives_the_next_leg_the_waypoint_as_its_start():
    nav, state = auto_navigator((0.0, 100.0), (100.0, 100.0))
    place(state, 1.0, 99.0)                         # inside the acceptance radius of waypoint 1
    assert nav.step(state, 0.5, 3.0)[2] is True
    nav.mission_seq = 2                             # what the vehicle does on 'reached'
    nav.step(state, 0.5, 3.0)
    g = nav.last_guidance
    assert g.along_track_m == pytest.approx(0.0, abs=1.5) and g.leg_length_m == pytest.approx(100.0, abs=0.05)


def test_guidance_is_cleared_when_nothing_is_being_followed():
    nav, state = auto_navigator((0.0, 100.0))
    place(state, 0.0, 0.0)
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance is not None
    nav.mission_paused = True
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance is None and nav._leg is None
    state.pose = None
    nav.mission_paused = False
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance is None


def test_steering_follows_the_desired_heading_not_the_bearing_to_the_target():
    """Boat 20 m east of a northbound line, pointing north: the target is dead
    ahead (bearing ~0) but pure pursuit already turns it back towards the line."""
    nav, state = auto_navigator((0.0, 200.0), (0.0, 300.0), seq=2)
    nav.mission = [at(0.0, 0.0), at(0.0, 100.0), at(0.0, 200.0)]
    nav.mission_seq = 2
    place(state, 20.0, 150.0, heading=0.0)
    m1, m2, _ = nav.step(state, 0.5, 3.0)
    assert m2 > m1                                   # right motor faster = turn LEFT, back to the line (west)
    assert nav.last_guidance.cross_track_m == pytest.approx(20.0, abs=0.05)


def test_distance_accuracy_over_a_long_leg():
    """Leg geometry lives in a tangent plane anchored at the leg start: a 5 km
    leg must still give the geodesic length to millimetres."""
    nav = Navigator()
    state = VehicleState()
    state.mode = MODE_GUIDED
    nav.guided_target = (41.045, 29.06)
    state.pose = make_pose(41.0, 29.0, heading_deg=45.0, speed_mps=1.5)
    nav.step(state, 0.5, 3.0)
    assert nav.last_guidance.leg_length_m == pytest.approx(geo.distance_m(41.0, 29.0, 41.045, 29.06), abs=0.01)


# -- parameter sanitising (a bad value must not reach the PID) -----------------------------------------

def _params(**overrides):
    from bahr_pilot.vehicle import DEFAULT_PARAMS
    params = dict(DEFAULT_PARAMS)
    params.update(overrides)
    return params


def test_configure_takes_the_vehicle_parameters():
    nav = Navigator()
    nav.configure(_params(ATC_STR_RAT_P=0.5, ATC_STR_ANG_P=2.5, ATC_SPEED_I=0.2, NAVL1_PERIOD=8.0))
    cfg = nav.control_config
    assert (cfg.str_rat.p, cfg.str_ang_p, cfg.speed.i, nav.l1_period_s) == (0.5, 2.5, 0.2, 8.0)


@pytest.mark.parametrize("name,bad,expected_attr", [
    ("ATC_STR_RAT_FILT", 0.0, ("str_rat", "filter_hz", 0.5)),            # would divide by zero
    ("ATC_STR_RAT_FILT", -5.0, ("str_rat", "filter_hz", 0.5)),
    ("ATC_STR_RAT_P", float("nan"), ("str_rat", "p", ControlConfig().str_rat.p)),   # NaN -> the default
    ("ATC_STR_RAT_P", -1.0, ("str_rat", "p", 0.0)),
    ("ATC_STR_RAT_P", 1e9, ("str_rat", "p", 3.0)),
    ("ATC_SPEED_I", float("inf"), ("speed", "i", 0.10)),
    ("ATC_ACCEL_MAX", 0.0, (None, "accel_max_mps2", 0.1)),
    ("ATC_STR_RAT_MAX", 0.0, (None, "str_rat_max_dps", 5.0)),
])
def test_bad_parameter_values_are_forced_into_a_safe_range(name, bad, expected_attr):
    nav = Navigator()
    nav.configure(_params(**{name: bad}))
    group, attribute, expected = expected_attr
    target = nav.control_config if group is None else getattr(nav.control_config, group)
    assert getattr(target, attribute) == pytest.approx(expected)


def test_a_zero_filter_frequency_does_not_crash_a_step():
    nav, state = auto_navigator((0.0, 100.0))
    nav.configure(_params(ATC_STR_RAT_FILT=0.0, NAVL1_PERIOD=0.0, NAVL1_DAMPING=float("nan")))
    place(state, 0.0, 0.0)
    for k in range(5):
        state.pose = make_pose(*at(0.0, 0.0), heading_deg=10.0, speed_mps=1.0, t=0.05 * k)
        m1, m2, _ = nav.step(state, 0.5, 3.0, 1.5)
        assert 1000 <= m1 <= 2000 and 1000 <= m2 <= 2000


def test_reconfiguring_with_unchanged_values_keeps_the_controller_state():
    nav, state = auto_navigator((0.0, 100.0))
    nav.configure(_params())
    controller = nav.heading_control
    nav.configure(_params())
    assert nav.heading_control is controller                       # not rebuilt, so its integral survives
    nav.configure(_params(ATC_STR_RAT_I=0.2))
    assert nav.heading_control is not controller


def test_the_vehicle_defaults_and_the_controller_defaults_agree():
    """DEFAULT_PARAMS is what flies; ControlConfig() is what the unit tests
    build. They must not drift apart."""
    nav = Navigator()
    nav.configure(_params())
    assert nav.control_config == ControlConfig()
