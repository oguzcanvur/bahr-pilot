"""Coordinate frames and geodesy: the one place conversions happen (Phase 10).

Frames (docs/SENSOR_INTERFACE.md, docs/NAVIGATION.md):
  WGS84  geodetic latitude/longitude (degrees), height above the ellipsoid (m)
  ECEF   earth-centred earth-fixed, metres
  ENU    local tangent plane: east, north, up, metres, origin = a LocalFrame
  NED    north, east, down — only at the MAVLink boundary (enu_to_ned)
  BODY   x forward, y starboard, z down (bahr_pilot/attitude.py)

Everything is exact on the WGS84 ellipsoid (not a sphere): the haversine /
spherical formulas this replaces were measured 1.3 m per km out north-south
and 2.6 m per km east-west at 41 N (up to 5.6 m per km at the equator), which
swamps an RTK fix good to 2 cm. The tangent-plane approximation used for
ranges (distance_m, bearing_deg, LocalFrame.to_enu) was checked against
pyproj over 2000 random points: ECEF identical, geodetic round trip < 4 nm,
distance error 30 nm at 300 m and 0.13 mm at 5 km (tests/test_geo.py keeps
reference values from that comparison).

Pure Python (no numpy) so the Pi needs nothing extra.
"""
from __future__ import annotations

import math

# WGS84 defining constants
A = 6378137.0                    # semi-major axis, m
F = 1.0 / 298.257223563          # flattening
B = A * (1.0 - F)                # semi-minor axis
E2 = F * (2.0 - F)               # first eccentricity squared
EP2 = E2 / (1.0 - E2)            # second eccentricity squared


def geodetic_to_ecef(lat_deg: float, lon_deg: float, h_m: float = 0.0) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    n = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)  # prime vertical radius of curvature
    return (
        (n + h_m) * cos_lat * math.cos(lon),
        (n + h_m) * cos_lat * math.sin(lon),
        (n * (1.0 - E2) + h_m) * sin_lat,
    )


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Bowring's method with Newton refinement; sub-micrometre after 3 steps
    for any point from the surface up to well above aircraft altitude."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    if p < 1e-9:  # on the polar axis
        lat = math.copysign(math.pi / 2.0, z)
        return math.degrees(lat), 0.0, abs(z) - B
    theta = math.atan2(z * A, p * B)
    lat = math.atan2(z + EP2 * B * math.sin(theta) ** 3, p - E2 * A * math.cos(theta) ** 3)
    for _ in range(3):
        sin_lat = math.sin(lat)
        n = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
        h = p / math.cos(lat) - n if abs(lat) < math.radians(89.0) else z / sin_lat - n * (1.0 - E2)
        lat = math.atan2(z, p * (1.0 - E2 * n / (n + h)))
    sin_lat = math.sin(lat)
    n = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
    h = p / math.cos(lat) - n if abs(lat) < math.radians(89.0) else z / sin_lat - n * (1.0 - E2)
    return math.degrees(lat), math.degrees(lon), h


def ecef_to_enu(dx: float, dy: float, dz: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float, float]:
    """Rotate an ECEF difference vector into the ENU axes at (lat0, lon0)."""
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    return (
        -so * dx + co * dy,
        -sl * co * dx - sl * so * dy + cl * dz,
        cl * co * dx + cl * so * dy + sl * dz,
    )


def enu_to_ecef(e: float, n: float, u: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float, float]:
    lat0, lon0 = math.radians(lat0_deg), math.radians(lon0_deg)
    sl, cl = math.sin(lat0), math.cos(lat0)
    so, co = math.sin(lon0), math.cos(lon0)
    return (
        -so * e - sl * co * n + cl * co * u,
        co * e - sl * so * n + cl * so * u,
        cl * n + sl * u,
    )


def enu_to_ned(e: float, n: float, u: float) -> tuple[float, float, float]:
    return n, e, -u


def ned_to_enu(n: float, e: float, d: float) -> tuple[float, float, float]:
    return e, n, -d


class LocalFrame:
    """A tangent plane at a fixed origin. Points are (east, north) metres.

    Pick the origin once per mission/session (the first usable fix or HOME)
    and keep it: moving the origin mid-mission re-labels every stored point.
    Heights are ignored — a surface vessel navigates in the horizontal.
    """

    def __init__(self, lat0_deg: float, lon0_deg: float, h0_m: float = 0.0) -> None:
        self.lat0, self.lon0, self.h0 = lat0_deg, lon0_deg, h0_m
        self._origin_ecef = geodetic_to_ecef(lat0_deg, lon0_deg, h0_m)

    def to_enu(self, lat_deg: float, lon_deg: float, h_m: float | None = None) -> tuple[float, float]:
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, self.h0 if h_m is None else h_m)
        ox, oy, oz = self._origin_ecef
        e, n, _ = ecef_to_enu(x - ox, y - oy, z - oz, self.lat0, self.lon0)
        return e, n

    def _plane_point(self, e: float, n: float) -> tuple[float, float]:
        ox, oy, oz = self._origin_ecef
        dx, dy, dz = enu_to_ecef(e, n, 0.0, self.lat0, self.lon0)
        lat, lon, _ = ecef_to_geodetic(ox + dx, oy + dy, oz + dz)
        return lat, lon

    def to_geodetic(self, e: float, n: float) -> tuple[float, float]:
        """Exact inverse of to_enu. (e, n, 0) is a point on the tangent PLANE,
        which rises above the ellipsoid (0.3 m at 2 km, 1.9 m at 5 km), while
        to_enu places points at height h0; one residual correction removes
        that offset (without it the round trip is out by 0.2 mm at 2 km)."""
        lat, lon = self._plane_point(e, n)
        e_back, n_back = self.to_enu(lat, lon)
        return self._plane_point(e + (e - e_back), n + (n - n_back))


# -- ranges between two geodetic points -----------------------------------------

def _enu_between(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """East/north of point 2 seen from point 1, in the tangent plane at their
    midpoint — symmetric, so distance(a, b) == distance(b, a) exactly."""
    mid_lat = (lat1 + lat2) / 2.0
    lon_diff = (lon2 - lon1 + 180.0) % 360.0 - 180.0
    mid_lon = lon1 + lon_diff / 2.0
    x1, y1, z1 = geodetic_to_ecef(lat1, lon1)
    x2, y2, z2 = geodetic_to_ecef(lat2, lon2)
    e, n, _ = ecef_to_enu(x2 - x1, y2 - y1, z2 - z1, mid_lat, mid_lon)
    return e, n


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    e, n = _enu_between(lat1, lon1, lat2, lon2)
    return math.hypot(e, n)


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial true bearing from point 1 to point 2, degrees clockwise from
    north in [0, 360)."""
    e, n = _enu_between(lat1, lon1, lat2, lon2)
    return math.degrees(math.atan2(e, n)) % 360.0


def wrap_180(angle_deg: float) -> float:
    """Wrap to (-180, 180]."""
    wrapped = (angle_deg + 180.0) % 360.0 - 180.0
    return 180.0 if wrapped == -180.0 else wrapped


def heading_to_math_angle(heading_deg: float) -> float:
    """Compass heading (clockwise from north) -> maths angle (counter-
    clockwise from east), radians. Keeps the two conventions from being mixed
    by hand at every call site."""
    return math.radians(90.0 - heading_deg)


# -- path geometry in the local plane (east, north) ---------------------------------

def segment_errors(point: tuple[float, float], a: tuple[float, float],
                   b: tuple[float, float]) -> tuple[float, float, float]:
    """(along_track, cross_track, segment_length) of `point` relative to the
    directed segment a -> b, all in metres.

    along_track: distance from `a` projected onto the line (negative before a,
        greater than the length past b).
    cross_track: signed perpendicular distance; POSITIVE when the vehicle is to
        the RIGHT of the direction of travel (so it must steer left).
    A zero-length segment returns (distance to a, 0, 0)."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(ux, uy)
    px, py = point[0] - a[0], point[1] - a[1]
    if length < 1e-9:
        return math.hypot(px, py), 0.0, 0.0
    ux, uy = ux / length, uy / length
    along = px * ux + py * uy
    cross = px * uy - py * ux        # cross product of the travel direction with the offset
    return along, cross, length
