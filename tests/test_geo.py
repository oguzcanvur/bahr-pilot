"""Geodesy: reference values were produced once by pyproj 3.8.0 (an
independent PROJ-based implementation of the same WGS84 maths), pasted in as
constants so the suite needs no extra package. A 2000-point random cross-check
against pyproj (not part of the suite) gave: ECEF identical, round trip
< 4 nm, distance error 30 nm at 300 m and 0.13 mm at 5 km.
"""
from __future__ import annotations

import math
import random

import pytest

from bahr_pilot import geo

# (lat, lon, h) -> ECEF metres, from pyproj
ECEF_REFERENCE = [
    ((0.0, 0.0, 0.0), (6378137.0, 0.0, 0.0)),
    ((90.0, 0.0, 0.0), (0.0, 0.0, 6356752.314245)),
    ((41.0, 29.0, 0.0), (4216183.902293, 2337068.89963, 4162423.200686)),
    ((41.0123456, 29.0654321, 12.5), (4212734.154011, 2341449.835598, 4163466.03496)),
    ((-33.8688, 151.2093, 58.0), (-4646093.477288, 2553229.535817, -3534404.71091)),
]

# (point 1, point 2) -> (geodesic distance m, forward azimuth deg), from pyproj
GEODESIC_REFERENCE = [
    ((41.0, 29.0), (41.001, 29.0), 111.053918, 0.0),
    ((41.0, 29.0), (41.0, 29.001), 84.135185, 89.999672),
    ((41.0, 29.0), (41.003, 29.004), 473.55182, 45.287117),
    ((41.0, 29.0), (41.05, 29.07), 8092.727959, 46.651948),
]


@pytest.mark.parametrize("geodetic,ecef", ECEF_REFERENCE)
def test_geodetic_to_ecef_matches_pyproj(geodetic, ecef):
    assert geo.geodetic_to_ecef(*geodetic) == pytest.approx(ecef, abs=2e-6)


def test_wgs84_constants():
    assert geo.A == 6378137.0
    assert geo.B == pytest.approx(6356752.314245, abs=1e-6)
    assert geo.E2 == pytest.approx(0.00669437999014, rel=1e-11)


@pytest.mark.parametrize("geodetic,ecef", ECEF_REFERENCE)
def test_ecef_round_trip(geodetic, ecef):
    lat, lon, h = geo.ecef_to_geodetic(*ecef)
    # the reference ECEF values are printed to 1e-6 m, which bounds what a round trip can match
    assert lat == pytest.approx(geodetic[0], abs=1e-10)
    assert h == pytest.approx(geodetic[2], abs=1e-6)
    if abs(geodetic[0]) < 90:
        assert lon == pytest.approx(geodetic[1], abs=1e-9)


def test_random_round_trips():
    rng = random.Random(3)
    for _ in range(500):
        lat, lon, h = rng.uniform(-89, 89), rng.uniform(-180, 180), rng.uniform(-50, 2000)
        got = geo.ecef_to_geodetic(*geo.geodetic_to_ecef(lat, lon, h))
        assert abs(got[0] - lat) < 1e-9 and abs(got[2] - h) < 1e-6
        assert abs((got[1] - lon + 180) % 360 - 180) < 1e-9


def test_poles_and_the_antimeridian_do_not_blow_up():
    lat, lon, h = geo.ecef_to_geodetic(*geo.geodetic_to_ecef(90.0, 0.0, 100.0))
    assert lat == pytest.approx(90.0) and h == pytest.approx(100.0, abs=1e-6)
    lat, lon, h = geo.ecef_to_geodetic(*geo.geodetic_to_ecef(-90.0, 0.0, 0.0))
    assert lat == pytest.approx(-90.0)
    assert geo.distance_m(0.0, 179.9999, 0.0, -179.9999) == pytest.approx(22.264, abs=0.01)


@pytest.mark.parametrize("p1,p2,distance,azimuth", GEODESIC_REFERENCE)
def test_distance_and_bearing_match_the_geodesic(p1, p2, distance, azimuth):
    tolerance = 3e-4 if distance < 1000 else 1e-3
    assert geo.distance_m(*p1, *p2) == pytest.approx(distance, abs=tolerance)
    assert geo.wrap_180(geo.bearing_deg(*p1, *p2) - azimuth) == pytest.approx(0.0, abs=0.05)


def test_distance_is_symmetric_and_zero_for_the_same_point():
    a, b = (41.0, 29.0), (41.0021, 29.0033)
    assert geo.distance_m(*a, *b) == geo.distance_m(*b, *a)
    assert geo.distance_m(*a, *a) == 0.0


def test_old_spherical_formula_was_materially_wrong():
    """Why this module exists. The haversine on a 6371 km sphere that
    navigation.py used before 2026-10-02, against the real ellipsoid, on a
    1.1 km north-south line at 41 N."""
    r = 6371000.0
    phi1, phi2 = math.radians(41.0), math.radians(41.01)
    spherical = 2 * r * math.asin(math.sin((phi2 - phi1) / 2))
    ellipsoidal = geo.distance_m(41.0, 29.0, 41.01, 29.0)
    assert abs(spherical - ellipsoidal) > 1.0          # over a metre on a 1.1 km line


def test_bearing_is_clockwise_from_north():
    # due north may come back as 359.9999999998; compare through the wrap
    assert geo.wrap_180(geo.bearing_deg(41.0, 29.0, 41.001, 29.0)) == pytest.approx(0.0, abs=1e-6)
    assert geo.bearing_deg(41.0, 29.0, 41.0, 29.001) == pytest.approx(90.0, abs=0.01)
    assert geo.bearing_deg(41.0, 29.0, 40.999, 29.0) == pytest.approx(180.0, abs=1e-6)
    assert geo.bearing_deg(41.0, 29.0, 41.0, 28.999) == pytest.approx(270.0, abs=0.01)
    assert 0.0 <= geo.bearing_deg(41.0, 29.0, 41.001, 29.0) < 360.0


def test_wrap_180():
    assert geo.wrap_180(190) == -170 and geo.wrap_180(-190) == 170
    assert geo.wrap_180(180) == 180 and geo.wrap_180(-180) == 180
    assert geo.wrap_180(720) == 0 and geo.wrap_180(0) == 0


def test_compass_to_math_angle():
    assert geo.heading_to_math_angle(0.0) == pytest.approx(math.pi / 2)    # north = +y
    assert geo.heading_to_math_angle(90.0) == pytest.approx(0.0)           # east = +x
    assert geo.heading_to_math_angle(180.0) == pytest.approx(-math.pi / 2)


# -- LocalFrame -------------------------------------------------------------------

def test_local_frame_axes():
    frame = geo.LocalFrame(41.0, 29.0)
    assert frame.to_enu(41.0, 29.0) == pytest.approx((0.0, 0.0), abs=1e-9)
    e, n = frame.to_enu(41.001, 29.0)
    assert e == pytest.approx(0.0, abs=1e-3) and n == pytest.approx(111.053918, abs=2e-3)
    e, n = frame.to_enu(41.0, 29.001)
    assert e == pytest.approx(84.135185, abs=2e-3) and n == pytest.approx(0.0, abs=2e-3)


def test_local_frame_round_trip_over_a_survey_area():
    frame = geo.LocalFrame(41.0, 29.0)
    rng = random.Random(9)
    for _ in range(300):
        e, n = rng.uniform(-2000, 2000), rng.uniform(-2000, 2000)
        lat, lon = frame.to_geodetic(e, n)
        assert frame.to_enu(lat, lon) == pytest.approx((e, n), abs=1e-7)


def test_local_frame_agrees_with_the_geodesic_distance():
    frame = geo.LocalFrame(41.0, 29.0)
    for p in ((41.003, 29.004), (41.05, 29.07)):
        e, n = frame.to_enu(*p)
        assert math.hypot(e, n) == pytest.approx(geo.distance_m(41.0, 29.0, *p), abs=0.002)


def test_ned_and_enu_are_each_others_inverse():
    assert geo.enu_to_ned(1.0, 2.0, 3.0) == (2.0, 1.0, -3.0)
    assert geo.ned_to_enu(*geo.enu_to_ned(1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_enu_rotation_matches_the_inverse():
    d = (123.0, -45.0, 6.0)
    back = geo.enu_to_ecef(*geo.ecef_to_enu(*d, 41.0, 29.0), 41.0, 29.0)
    assert back == pytest.approx(d, abs=1e-9)


# -- path geometry ------------------------------------------------------------------

def test_segment_errors_along_and_cross_track():
    a, b = (0.0, 0.0), (0.0, 100.0)               # travelling due north
    along, cross, length = geo.segment_errors((3.0, 40.0), a, b)
    assert (along, length) == (40.0, 100.0)
    assert cross == pytest.approx(3.0)             # 3 m EAST of a northbound track = to the RIGHT -> +
    along, cross, _ = geo.segment_errors((-2.0, 10.0), a, b)
    assert cross == pytest.approx(-2.0)            # west = left = negative


def test_segment_errors_for_any_direction():
    a, b = (0.0, 0.0), (100.0, 0.0)               # travelling due east: right is SOUTH
    _, cross, _ = geo.segment_errors((50.0, -3.0), a, b)
    assert cross == pytest.approx(3.0)
    a, b = (10.0, 10.0), (-90.0, 10.0)            # due west: right is NORTH
    _, cross, _ = geo.segment_errors((-40.0, 14.0), a, b)
    assert cross == pytest.approx(4.0)


def test_before_and_beyond_the_segment():
    a, b = (0.0, 0.0), (0.0, 100.0)
    assert geo.segment_errors((0.0, -5.0), a, b)[0] == -5.0
    assert geo.segment_errors((0.0, 130.0), a, b)[0] == 130.0


def test_degenerate_segment():
    along, cross, length = geo.segment_errors((3.0, 4.0), (0.0, 0.0), (0.0, 0.0))
    assert (along, cross, length) == (5.0, 0.0, 0.0)
