"""Geofence geometry (bahr_pilot/geofence.py). Shapes are built in local
metres around (41 N, 29 E) and converted to coordinates, so every expected
margin is a distance worked out by hand in the plane."""
from __future__ import annotations

import math

import pytest

from bahr_pilot import geo
from bahr_pilot.geofence import (
    CMD_CIRCLE_EXCLUSION, CMD_CIRCLE_INCLUSION, CMD_POLYGON_EXCLUSION, CMD_POLYGON_INCLUSION, FenceError,
    Geofence, circle_margin, polygon_margin,
)

FRAME = geo.LocalFrame(41.0, 29.0)
HOME = FRAME.to_geodetic(0.0, 0.0)
PARAMS = {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 6.0, "FENCE_RADIUS": 100.0, "FENCE_MARGIN": 2.0}


def params(**overrides):
    return {**PARAMS, **overrides}


def ll(east, north):
    return FRAME.to_geodetic(east, north)


def polygon_items(command, vertices):
    """MAVLink fence items for one polygon given (east, north) vertices."""
    return [(command, float(len(vertices)), *ll(*v)) for v in vertices]


SQUARE = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]


def fence_with(*item_lists):
    fence = Geofence()
    fence.load([item for items in item_lists for item in items])
    return fence


def margin(fence, east, north, p=None):
    status = fence.evaluate(*ll(east, north), HOME, p or params(FENCE_TYPE=4.0))
    return None if status is None else status.margin_m


# -- plane geometry --------------------------------------------------------------------------------

def test_polygon_margin_is_the_signed_distance_to_the_nearest_edge():
    assert polygon_margin((0.0, 0.0), SQUARE) == pytest.approx(50.0)
    assert polygon_margin((40.0, 0.0), SQUARE) == pytest.approx(10.0)
    assert polygon_margin((60.0, 0.0), SQUARE) == pytest.approx(-10.0)
    assert polygon_margin((60.0, 60.0), SQUARE) == pytest.approx(-math.hypot(10.0, 10.0))   # nearest point is a corner
    assert polygon_margin((50.0, 0.0), SQUARE) == pytest.approx(0.0, abs=1e-9)               # on an edge


def test_polygon_orientation_does_not_matter():
    for point in ((0.0, 0.0), (45.0, 10.0), (70.0, -20.0)):
        assert polygon_margin(point, SQUARE) == pytest.approx(polygon_margin(point, SQUARE[::-1]))


def test_a_concave_polygon_notch_is_outside():
    ell = [(0.0, 0.0), (100.0, 0.0), (100.0, 40.0), (40.0, 40.0), (40.0, 100.0), (0.0, 100.0)]    # an L
    assert polygon_margin((20.0, 80.0), ell) == pytest.approx(20.0)           # in the tall arm, 20 m from either side
    assert polygon_margin((70.0, 70.0), ell) == pytest.approx(-30.0)          # in the notch: 30 m to the arm (x = 40)
    assert polygon_margin((80.0, 20.0), ell) == pytest.approx(20.0)           # in the long arm, 20 m from the top edge


def test_circle_margin():
    assert circle_margin((30.0, 0.0), (0.0, 0.0), 100.0) == pytest.approx(70.0)
    assert circle_margin((0.0, 130.0), (0.0, 0.0), 100.0) == pytest.approx(-30.0)


# -- the circle around home -------------------------------------------------------------------------

def test_the_circle_around_home_uses_ellipsoidal_distance():
    fence = Geofence()
    p = params(FENCE_TYPE=2.0)
    status = fence.evaluate(*ll(0.0, 60.0), HOME, p)
    assert status.margin_m == pytest.approx(40.0, abs=0.001) and not status.breached and status.which == "circle"
    status = fence.evaluate(*ll(130.0, 0.0), HOME, p)
    assert status.margin_m == pytest.approx(-30.0, abs=0.001) and status.breached


# -- uploaded polygons and circles --------------------------------------------------------------------------

def test_inside_and_outside_an_inclusion_polygon():
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE))
    assert margin(fence, 0.0, 0.0) == pytest.approx(50.0, abs=0.001)
    assert margin(fence, 60.0, 0.0) == pytest.approx(-10.0, abs=0.001)
    assert fence.evaluate(*ll(60.0, 0.0), HOME, params(FENCE_TYPE=4.0)).breached


def test_the_vehicle_may_be_in_any_one_of_several_inclusion_zones():
    other = [(200.0, -50.0), (300.0, -50.0), (300.0, 50.0), (200.0, 50.0)]
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE), polygon_items(CMD_POLYGON_INCLUSION, other))
    assert margin(fence, 0.0, 0.0) == pytest.approx(50.0, abs=0.001)
    assert margin(fence, 250.0, 0.0) == pytest.approx(50.0, abs=0.001)
    assert margin(fence, 125.0, 0.0) == pytest.approx(-75.0, abs=0.001)      # in the gap: nearest edge is 75 m away


def test_exclusion_polygon_is_a_no_go_area():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, SQUARE))
    assert margin(fence, 0.0, 0.0) == pytest.approx(-50.0, abs=0.001)         # inside it = breach
    assert margin(fence, 80.0, 0.0) == pytest.approx(30.0, abs=0.001)         # outside it, 30 m clear


def test_the_vehicle_must_stay_out_of_all_exclusion_zones():
    other = [(200.0, -50.0), (300.0, -50.0), (300.0, 50.0), (200.0, 50.0)]
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, SQUARE), polygon_items(CMD_POLYGON_EXCLUSION, other))
    assert margin(fence, 125.0, 0.0) == pytest.approx(75.0, abs=0.001)
    assert margin(fence, 250.0, 0.0) == pytest.approx(-50.0, abs=0.001)


def test_uploaded_circles():
    fence = fence_with([(CMD_CIRCLE_INCLUSION, 80.0, *ll(0.0, 0.0))])
    assert margin(fence, 30.0, 0.0) == pytest.approx(50.0, abs=0.001)
    assert margin(fence, 100.0, 0.0) == pytest.approx(-20.0, abs=0.001)
    ex = fence_with([(CMD_CIRCLE_EXCLUSION, 25.0, *ll(100.0, 0.0))])
    assert margin(ex, 100.0, 0.0) == pytest.approx(-25.0, abs=0.001)
    assert margin(ex, 160.0, 0.0) == pytest.approx(35.0, abs=0.001)


def test_the_smallest_margin_over_all_limits_wins_and_says_which():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, [(20.0, -10.0), (40.0, -10.0), (40.0, 10.0), (20.0, 10.0)]))
    status = fence.evaluate(*ll(10.0, 0.0), HOME, params(FENCE_TYPE=6.0))         # circle margin 90, exclusion margin 10
    assert status.margin_m == pytest.approx(10.0, abs=0.001) and status.which == "exclusion zone"
    status = fence.evaluate(*ll(0.0, 95.0), HOME, params(FENCE_TYPE=6.0))         # circle margin 5 now the smallest
    assert status.which == "circle"


def test_inclusion_and_exclusion_together():
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE),
                       polygon_items(CMD_POLYGON_EXCLUSION, [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]))
    assert margin(fence, 0.0, 0.0) == pytest.approx(-10.0, abs=0.001)        # inside the no-go square
    assert margin(fence, 30.0, 0.0) == pytest.approx(min(20.0, 20.0), abs=0.001)


# -- when no limit applies ---------------------------------------------------------------------------------------

@pytest.mark.parametrize("overrides", [
    {"FENCE_ENABLE": 0.0},
    {"FENCE_TYPE": 0.0},
    {"FENCE_TYPE": 1.0},                                       # altitude bits only: not applicable to a boat
])
def test_nothing_is_checked_when_the_fence_is_off_or_of_another_kind(overrides):
    assert Geofence().evaluate(*ll(500.0, 500.0), HOME, params(**overrides)) is None


def test_the_circle_needs_a_home_and_a_radius():
    assert Geofence().evaluate(*ll(0.0, 0.0), None, params(FENCE_TYPE=2.0)) is None
    assert Geofence().evaluate(*ll(0.0, 0.0), HOME, params(FENCE_TYPE=2.0, FENCE_RADIUS=0.0)) is None


def test_the_polygon_type_without_a_polygon_checks_nothing():
    assert Geofence().evaluate(*ll(0.0, 0.0), HOME, params(FENCE_TYPE=4.0)) is None


def test_uploaded_shapes_are_ignored_unless_the_polygon_bit_is_set():
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE))
    assert fence.evaluate(*ll(500.0, 0.0), HOME, params(FENCE_TYPE=2.0)).which == "circle"


# -- loading and validating ------------------------------------------------------------------------------------------

def test_load_replaces_the_previous_fence_and_clear_empties_it():
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE))
    assert fence.has_shapes and fence.items == 4
    fence.load(polygon_items(CMD_POLYGON_EXCLUSION, SQUARE))
    assert margin(fence, 0.0, 0.0) == pytest.approx(-50.0, abs=0.001)
    fence.clear()
    assert not fence.has_shapes and fence.items == 0 and margin(fence, 0.0, 0.0) is None


@pytest.mark.parametrize("items,reason", [
    ([(CMD_POLYGON_INCLUSION, 2.0, 41.0, 29.0), (CMD_POLYGON_INCLUSION, 2.0, 41.1, 29.0)], "at least 3 vertices"),
    ([(CMD_POLYGON_INCLUSION, 4.0, 41.0, 29.0), (CMD_POLYGON_INCLUSION, 4.0, 41.1, 29.0)], "past the end"),
    ([(CMD_POLYGON_INCLUSION, 3.0, 41.0, 29.0), (CMD_POLYGON_INCLUSION, 3.0, 41.1, 29.0),
      (CMD_POLYGON_EXCLUSION, 3.0, 41.1, 29.1)], "does not belong"),
    ([(CMD_POLYGON_INCLUSION, 3.0, 41.0, 29.0), (CMD_POLYGON_INCLUSION, 3.0, 41.1, 29.0),
      (CMD_POLYGON_INCLUSION, 4.0, 41.1, 29.1)], "does not belong"),
    ([(CMD_CIRCLE_INCLUSION, 0.0, 41.0, 29.0)], "radius must be positive"),
    ([(CMD_CIRCLE_INCLUSION, -5.0, 41.0, 29.0)], "radius must be positive"),
    ([(CMD_CIRCLE_INCLUSION, float("nan"), 41.0, 29.0)], "radius must be positive"),
    ([(16, 0.0, 41.0, 29.0)], "unsupported fence command"),
    ([(CMD_CIRCLE_INCLUSION, 10.0, float("nan"), 29.0)], "bad coordinate"),
    ([(CMD_CIRCLE_INCLUSION, 10.0, 95.0, 29.0)], "bad coordinate"),
    ([(CMD_POLYGON_INCLUSION, float("nan"), 41.0, 29.0)], "at least 3 vertices"),
])
def test_unusable_fences_are_refused_and_leave_the_old_one_alone(items, reason):
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE))
    with pytest.raises(FenceError, match=reason):
        fence.load(items)
    assert margin(fence, 0.0, 0.0) == pytest.approx(50.0, abs=0.001)       # still the square


def test_waypoints_outside_the_fence_are_listed_but_slot_zero_is_not():
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, SQUARE))
    mission = [ll(500.0, 500.0), ll(0.0, 0.0), ll(60.0, 0.0), ll(0.0, -80.0)]       # slot 0 (home) is far outside
    outside = fence.waypoints_outside(mission, HOME, params(FENCE_TYPE=4.0))
    assert [i for i, _ in outside] == [2, 3]
    assert outside[0][1] == pytest.approx(-10.0, abs=0.001) and outside[1][1] == pytest.approx(-30.0, abs=0.001)
    assert fence.waypoints_outside(mission, HOME, params(FENCE_ENABLE=0.0)) == []


# -- is the straight run home clear? (RTL) ------------------------------------------------------------------

NO_GO = [(-20.0, 40.0), (20.0, 40.0), (20.0, 60.0), (-20.0, 60.0)]          # a square across the line x = 0


def clear(fence, start, end, p=None):
    return fence.path_is_clear(ll(*start), ll(*end), p or params(FENCE_TYPE=4.0))


def test_a_path_around_a_no_go_area_is_clear_and_through_it_is_not():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO))
    assert clear(fence, (0.0, 100.0), (0.0, 0.0)) is False                   # straight through the square
    assert clear(fence, (40.0, 100.0), (40.0, 0.0)) is True                  # 20 m clear of its side
    assert clear(fence, (0.0, 30.0), (0.0, 0.0)) is True                     # stops short of it


def test_a_boat_inside_a_no_go_area_may_drive_out_of_it_but_not_back_in():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO))
    assert clear(fence, (0.0, 50.0), (0.0, 0.0)) is True                     # starts inside, leaves, ends outside
    two = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO),
                     polygon_items(CMD_POLYGON_EXCLUSION, [(-20.0, 10.0), (20.0, 10.0), (20.0, 25.0), (-20.0, 25.0)]))
    assert clear(two, (0.0, 50.0), (0.0, 0.0)) is False                      # leaves the first, enters the second


def test_a_path_that_never_gets_back_to_allowed_water_is_not_clear():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO))
    assert clear(fence, (0.0, 45.0), (0.0, 55.0)) is False                   # ends inside the zone (home in a no-go area)


def test_a_path_must_stay_inside_a_concave_inclusion_zone():
    ell = [(0.0, 0.0), (100.0, 0.0), (100.0, 40.0), (40.0, 40.0), (40.0, 100.0), (0.0, 100.0)]
    fence = fence_with(polygon_items(CMD_POLYGON_INCLUSION, ell))
    assert clear(fence, (20.0, 80.0), (20.0, 10.0)) is True                  # down the tall arm
    assert clear(fence, (20.0, 90.0), (90.0, 10.0)) is False                 # cuts the corner through the notch
    assert clear(fence, (150.0, 20.0), (20.0, 20.0)) is True                 # from outside straight into the long arm
    assert clear(fence, (110.0, 20.0), (60.0, 80.0)) is False                # enters the long arm, then leaves through the notch


def test_a_path_with_no_applicable_fence_is_clear():
    assert Geofence().path_is_clear(ll(0.0, 100.0), ll(0.0, 0.0), params(FENCE_TYPE=4.0)) is True
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO))
    assert clear(fence, (0.0, 100.0), (0.0, 0.0), params(FENCE_TYPE=2.0)) is True        # polygon bit off
    assert clear(fence, (0.0, 100.0), (0.0, 0.0), params(FENCE_TYPE=4.0, FENCE_ENABLE=0.0)) is True


def test_a_zero_length_path_is_judged_at_its_point():
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, NO_GO))
    assert clear(fence, (0.0, 0.0), (0.0, 0.0)) is True
    assert clear(fence, (0.0, 50.0), (0.0, 50.0)) is False


def test_a_narrow_no_go_strip_is_not_stepped_over():
    """A 3 m wide cable corridor across a 100 m run, sampled every metre: it must be seen."""
    strip = [(-50.0, 49.0), (50.0, 49.0), (50.0, 52.0), (-50.0, 52.0)]
    fence = fence_with(polygon_items(CMD_POLYGON_EXCLUSION, strip))
    assert clear(fence, (0.0, 100.0), (0.0, 0.0)) is False
    assert clear(fence, (0.0, 45.0), (0.0, 0.0)) is True
