"""Pure-pursuit geometry. Every expected number is worked out by hand from the
picture in the comment (leg due north unless stated, vehicle position given as
(east, north) metres), not taken from the code's output."""
from __future__ import annotations

import math

import pytest

from bahr_pilot import geo
from bahr_pilot.pathfollow import Guidance, PathConfig, PurePursuit, lookahead_distance

NORTH_LEG = ((0.0, 0.0), (0.0, 100.0))      # 100 m due north
LOOKAHEAD = 10.0


def guide(position, leg=NORTH_LEG, *, lookahead=LOOKAHEAD, radius=3.0, speed=1.5, config=None):
    return PurePursuit(config).guide(position, leg[0], leg[1], speed, radius, lookahead)


# -- the aim point ------------------------------------------------------------------------------

def test_on_the_line_the_boat_aims_along_it():
    g = guide((0.0, 0.0))
    assert g.desired_heading_deg == pytest.approx(0.0, abs=1e-9) or g.desired_heading_deg == pytest.approx(360.0)
    assert (g.along_track_m, g.cross_track_m, g.leg_length_m) == pytest.approx((0.0, 0.0, 100.0))


@pytest.mark.parametrize("position,expected", [
    ((5.0, 0.0), math.degrees(math.atan2(-5.0, 10.0)) % 360.0),     # 5 m east: aim (0,10) -> (-5,10): 333.43
    ((-5.0, 0.0), math.degrees(math.atan2(5.0, 10.0))),             # 5 m west: (5,10): 26.57
    ((50.0, 0.0), math.degrees(math.atan2(-50.0, 10.0)) % 360.0),   # far east: 281.31, a shallow approach
    ((0.0, 40.0), 0.0),                                             # on the line mid-way
])
def test_aim_point_is_lookahead_metres_further_along_the_line(position, expected):
    g = guide(position)
    assert geo.wrap_180(g.desired_heading_deg - expected) == pytest.approx(0.0, abs=1e-9)


def test_far_from_the_line_the_approach_is_shallow_not_perpendicular():
    """Aim 10 m ahead of the projection from 50 m out: nearly west, not straight at the line."""
    heading = guide((50.0, 0.0)).desired_heading_deg
    assert 270.0 < heading < 285.0


def test_cross_track_sign_positive_means_right_of_the_direction_of_travel():
    assert guide((5.0, 30.0)).cross_track_m == pytest.approx(5.0)       # east of a northbound line = right
    assert guide((-5.0, 30.0)).cross_track_m == pytest.approx(-5.0)
    east_leg = ((0.0, 0.0), (100.0, 0.0))                               # eastbound: right is SOUTH
    assert guide((30.0, -4.0), east_leg).cross_track_m == pytest.approx(4.0)
    assert guide((30.0, 4.0), east_leg).cross_track_m == pytest.approx(-4.0)


def test_a_diagonal_leg():
    leg = ((0.0, 0.0), (100.0, 100.0))                                  # north-east
    assert geo.wrap_180(guide((0.0, 0.0), leg).desired_heading_deg - 45.0) == pytest.approx(0.0, abs=1e-9)
    g = guide((10.0, 0.0), leg)                                         # south-east of the line = right
    assert g.cross_track_m == pytest.approx(10.0 * math.sin(math.radians(45.0)))
    assert g.along_track_m == pytest.approx(10.0 * math.cos(math.radians(45.0)))


def test_the_crosstrack_gain_weights_the_offset():
    # 5 m east of the line, aim point (0,10).  gain 1: (-5,10)  gain 2: (-10,10) = 315  gain 0.5: (-2.5,10)
    cases = {1.0: math.degrees(math.atan2(-5.0, 10.0)) % 360.0,
             2.0: 315.0,
             0.5: math.degrees(math.atan2(-2.5, 10.0)) % 360.0}
    for gain, expected in cases.items():
        g = guide((5.0, 0.0), config=PathConfig(crosstrack_gain=gain))
        assert geo.wrap_180(g.desired_heading_deg - expected) == pytest.approx(0.0, abs=1e-9), gain
    assert (guide((5.0, 0.0), config=PathConfig(crosstrack_gain=2.0)).cross_track_m
            == pytest.approx(5.0))                                      # the reported error is the real one


# -- before the start, past the end, degenerate legs ------------------------------------------------

def test_before_the_start_the_boat_goes_to_the_start_of_the_leg():
    g = guide((0.0, -20.0))                                             # 20 m behind the start
    assert g.along_track_m == pytest.approx(-20.0)
    assert g.desired_heading_deg == pytest.approx(0.0, abs=1e-9) or g.desired_heading_deg == pytest.approx(360.0)


def test_past_the_end_the_aim_point_is_clamped_to_the_end_not_beyond():
    g = guide((0.0, 120.0))
    assert geo.wrap_180(g.desired_heading_deg - 180.0) == pytest.approx(0.0, abs=1e-9)   # back towards (0,100)


def test_near_the_end_it_heads_for_the_waypoint_itself():
    g = guide((2.0, 96.0))                      # the 10 m lookahead would overshoot: aim = the end (0,100)
    expected = math.degrees(math.atan2(-2.0, 4.0)) % 360.0
    assert geo.wrap_180(g.desired_heading_deg - expected) == pytest.approx(0.0, abs=1e-9)


def test_a_zero_length_leg_means_go_to_the_point():
    leg = ((10.0, 10.0), (10.0, 10.0))
    g = PurePursuit().guide((0.0, 10.0), leg[0], leg[1], 1.0, 3.0, 5.0)
    assert geo.wrap_180(g.desired_heading_deg - 90.0) == pytest.approx(0.0, abs=1e-9)
    assert g.leg_length_m == 0.0 and not g.reached
    assert PurePursuit().guide((9.0, 10.0), leg[0], leg[1], 1.0, 3.0, 5.0).reached


def test_standing_exactly_on_the_aim_point_keeps_the_leg_direction():
    g = guide((0.0, 10.0), lookahead=0.0)       # aim point == position
    assert math.isfinite(g.desired_heading_deg)
    assert geo.wrap_180(g.desired_heading_deg - 0.0) == pytest.approx(0.0, abs=1e-9)


# -- arrival ---------------------------------------------------------------------------------------------

def test_reached_inside_the_acceptance_radius():
    assert guide((0.0, 98.0), radius=3.0).reached
    assert not guide((0.0, 90.0), radius=3.0).reached


def test_reached_when_the_end_plane_is_crossed_within_two_radii():
    """A current that keeps the boat just outside the radius must not make it orbit."""
    assert guide((5.0, 100.5), radius=3.0).reached           # beyond the plane, 5 m off: inside 2 x 3
    assert not guide((7.0, 100.5), radius=3.0).reached       # beyond the plane but 7 m off
    assert not guide((5.0, 90.0), radius=3.0).reached        # 5 m off but short of the plane
    assert not guide((5.0, 100.5), radius=3.0, config=PathConfig(pass_factor=1.0)).reached


def test_guidance_reports_the_distance_to_the_target():
    g = guide((3.0, 60.0))
    assert g.distance_to_target_m == pytest.approx(math.hypot(3.0, 40.0))
    assert isinstance(g, Guidance) and g.lookahead_m == LOOKAHEAD


# -- the lookahead --------------------------------------------------------------------------------------

def test_lookahead_follows_the_l1_formula():
    cfg = PathConfig()
    assert lookahead_distance(1.5, 10.0, 0.75, cfg) == pytest.approx(0.75 * 10.0 * 1.5 / math.pi)
    assert lookahead_distance(10.0, 10.0, 0.75, cfg) == pytest.approx(0.75 * 10.0 * 10.0 / math.pi)


def test_lookahead_is_clamped_and_survives_bad_speeds():
    cfg = PathConfig()
    assert lookahead_distance(0.0, 10.0, 0.75, cfg) == cfg.lookahead_min_m
    assert lookahead_distance(1000.0, 10.0, 0.75, cfg) == cfg.lookahead_max_m
    for bad in (-3.0, float("nan"), float("inf") * 0 + float("nan")):
        assert lookahead_distance(bad, 10.0, 0.75, cfg) == cfg.lookahead_min_m


def test_a_shorter_l1_period_aims_closer_and_turns_harder():
    cfg = PathConfig(lookahead_min_m=0.1)
    tight = lookahead_distance(2.0, 5.0, 0.75, cfg)
    loose = lookahead_distance(2.0, 20.0, 0.75, cfg)
    assert tight < loose
    sharp = guide((5.0, 0.0), lookahead=tight).desired_heading_deg
    gentle = guide((5.0, 0.0), lookahead=loose).desired_heading_deg
    assert abs(geo.wrap_180(sharp)) > abs(geo.wrap_180(gentle))       # further from north = steeper return


# -- convergence: following the guidance actually reaches the line --------------------------------------

def test_flying_the_guidance_converges_onto_the_line():
    """Kinematic check, no dynamics: move 0.2 m per step along the desired
    heading from 20 m off the line; the cross-track error must shrink
    monotonically to ~0."""
    position, previous = [20.0, 5.0], math.inf
    pursuit = PurePursuit()
    for _ in range(400):
        g = pursuit.guide(tuple(position), NORTH_LEG[0], NORTH_LEG[1], 1.5, 3.0, LOOKAHEAD)
        if g.reached:
            break
        assert abs(g.cross_track_m) <= previous + 1e-9
        previous = abs(g.cross_track_m)
        heading = math.radians(g.desired_heading_deg)
        position[0] += 0.2 * math.sin(heading)
        position[1] += 0.2 * math.cos(heading)
    assert previous < 0.05 or g.reached
