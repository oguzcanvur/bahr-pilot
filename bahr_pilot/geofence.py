"""Geofence (Phase 21): where the boat may be.

Three kinds of limit, each reduced to ONE number, the signed distance in metres
from the position to the nearest boundary (positive = on the allowed side):

  circle      FENCE_RADIUS around home (FENCE_TYPE bit 2)
  inclusion   the boat must be inside AT LEAST ONE of the inclusion polygons /
              circles uploaded over MAVLink (FENCE_TYPE bit 4)
  exclusion   it must be outside ALL exclusion polygons / circles (no-go areas:
              a shipping channel, a cable crossing)

The fence margin is the smallest of the three; a negative value is a breach.
Margin as a number (not a yes/no) lets the failsafe add hysteresis and lets
tests check the geometry exactly.

Uploaded fences use the standard MAVLink fence items (mission type 1):
  MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION (5001) / _EXCLUSION (5002), param1
  = number of vertices of that polygon, consecutive items form one polygon;
  MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION (5003) / _EXCLUSION (5004), param1 = radius.

Geometry is done on a local tangent plane (bahr_pilot/geo.py) anchored at the
first vertex, so it is exact ellipsoidal geometry to millimetres within the
20 km the tangent plane is validated for.

Not covered: the RTL path itself may cross an exclusion zone (the straight line
home is not checked), and a fence is held in memory only (Phase 22).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Sequence

from bahr_pilot import geo

Point = tuple[float, float]               # (east, north) metres in the fence frame

FENCE_TYPE_CIRCLE = 2
FENCE_TYPE_POLYGON = 4

CMD_POLYGON_INCLUSION = 5001
CMD_POLYGON_EXCLUSION = 5002
CMD_CIRCLE_INCLUSION = 5003
CMD_CIRCLE_EXCLUSION = 5004
FENCE_COMMANDS = frozenset({CMD_POLYGON_INCLUSION, CMD_POLYGON_EXCLUSION,
                            CMD_CIRCLE_INCLUSION, CMD_CIRCLE_EXCLUSION})

MIN_POLYGON_VERTICES = 3


class FenceError(ValueError):
    """An uploaded fence that cannot be used."""


# -- plane geometry ---------------------------------------------------------------------------

def _inside_polygon(p: Point, poly: Sequence[Point]) -> bool:
    """Ray casting; points exactly on an edge count as inside."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if _distance_to_segment(p, poly[i], poly[(i + 1) % n]) < 1e-9:
            return True
        if (y1 > y) != (y2 > y):
            if x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
                inside = not inside
    return inside


def _distance_to_segment(p: Point, a: Point, b: Point) -> float:
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / length_sq))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def polygon_margin(p: Point, poly: Sequence[Point]) -> float:
    """Signed distance to the polygon boundary: positive inside, negative outside."""
    distance = min(_distance_to_segment(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))
    return distance if _inside_polygon(p, poly) else -distance


def circle_margin(p: Point, center: Point, radius: float) -> float:
    return radius - math.hypot(p[0] - center[0], p[1] - center[1])


# -- the fence ------------------------------------------------------------------------------------

@dataclass(frozen=True)
class FenceStatus:
    margin_m: float                    # smallest signed distance to any boundary
    breached: bool
    which: str                         # what the smallest margin belongs to


@dataclass
class _Shapes:
    polygons: list[list[Point]] = field(default_factory=list)
    circles: list[tuple[Point, float]] = field(default_factory=list)


class Geofence:
    def __init__(self) -> None:
        self._frame: geo.LocalFrame | None = None
        self._inclusion = _Shapes()
        self._exclusion = _Shapes()
        self.items = 0                              # how many MAVLink items made this fence

    @property
    def has_shapes(self) -> bool:
        return bool(self._inclusion.polygons or self._inclusion.circles
                    or self._exclusion.polygons or self._exclusion.circles)

    def clear(self) -> None:
        self._frame = None
        self._inclusion, self._exclusion = _Shapes(), _Shapes()
        self.items = 0

    # -- loading from MAVLink fence items ----------------------------------------------------------

    def load(self, items: Sequence[tuple[int, float, float, float]]) -> None:
        """Replace the fence with `items`: (command, param1, lat, lon) in
        order. Raises FenceError (leaving the previous fence untouched) for a
        polygon with the wrong vertex count or fewer than 3 vertices, a
        non-positive circle radius, an unknown command or a bad coordinate."""
        inclusion, exclusion = _Shapes(), _Shapes()
        raw_polygons: list[tuple[bool, list[tuple[float, float]]]] = []   # (inclusion?, [(lat, lon)])
        raw_circles: list[tuple[bool, float, float, float]] = []
        index = 0
        while index < len(items):
            command, p1, lat, lon = items[index]
            if command not in FENCE_COMMANDS:
                raise FenceError(f"item {index}: unsupported fence command {command}")
            if not (math.isfinite(lat) and math.isfinite(lon) and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                raise FenceError(f"item {index}: bad coordinate")
            if command in (CMD_CIRCLE_INCLUSION, CMD_CIRCLE_EXCLUSION):
                if not (math.isfinite(p1) and p1 > 0.0):
                    raise FenceError(f"item {index}: circle radius must be positive")
                raw_circles.append((command == CMD_CIRCLE_INCLUSION, lat, lon, p1))
                index += 1
                continue
            count = int(p1) if math.isfinite(p1) else 0
            if count < MIN_POLYGON_VERTICES:
                raise FenceError(f"item {index}: a polygon needs at least {MIN_POLYGON_VERTICES} vertices")
            if index + count > len(items):
                raise FenceError(f"item {index}: polygon of {count} vertices runs past the end")
            vertices = []
            for k in range(count):
                c, p, la, lo = items[index + k]
                if c != command or int(p) != count:
                    raise FenceError(f"item {index + k}: vertex does not belong to the polygon of {count}")
                if not (math.isfinite(la) and math.isfinite(lo) and -90.0 <= la <= 90.0 and -180.0 <= lo <= 180.0):
                    raise FenceError(f"item {index + k}: bad coordinate")
                vertices.append((la, lo))
            raw_polygons.append((command == CMD_POLYGON_INCLUSION, vertices))
            index += count

        anchor = (raw_polygons[0][1][0] if raw_polygons else
                  (raw_circles[0][1], raw_circles[0][2]) if raw_circles else None)
        frame = geo.LocalFrame(*anchor) if anchor is not None else None
        for is_inclusion, vertices in raw_polygons:
            (inclusion if is_inclusion else exclusion).polygons.append([frame.to_enu(la, lo) for la, lo in vertices])
        for is_inclusion, lat, lon, radius in raw_circles:
            (inclusion if is_inclusion else exclusion).circles.append((frame.to_enu(lat, lon), radius))
        self._frame, self._inclusion, self._exclusion = frame, inclusion, exclusion
        self.items = len(items)

    # -- evaluation ---------------------------------------------------------------------------------------

    def evaluate(self, lat: float, lon: float, home: tuple[float, float] | None,
                 params: Mapping[str, float]) -> FenceStatus | None:
        """The fence status at a position, or None when no limit applies (fence
        disabled, or nothing to check against: a circle with no home yet, a
        polygon type with no polygon uploaded)."""
        if params["FENCE_ENABLE"] < 0.5:
            return None
        fence_type = int(params["FENCE_TYPE"])
        margins: list[tuple[float, str]] = []

        radius = params["FENCE_RADIUS"]
        if fence_type & FENCE_TYPE_CIRCLE and home is not None and radius > 0.0:
            margins.append((radius - geo.distance_m(home[0], home[1], lat, lon), "circle"))

        if fence_type & FENCE_TYPE_POLYGON and self._frame is not None:
            margins.extend(self._shape_margins(self._frame.to_enu(lat, lon)))
        if not margins:
            return None
        margin, which = min(margins)
        return FenceStatus(margin_m=margin, breached=margin < 0.0, which=which)

    def _shape_margins(self, p: Point) -> list[tuple[float, str]]:
        """Margins of the uploaded shapes at a point of the fence frame."""
        margins = []
        inclusion = [polygon_margin(p, poly) for poly in self._inclusion.polygons]
        inclusion += [circle_margin(p, c, r) for c, r in self._inclusion.circles]
        if inclusion:
            margins.append((max(inclusion), "inclusion zone"))        # inside ANY one is enough
        exclusion = [-polygon_margin(p, poly) for poly in self._exclusion.polygons]
        exclusion += [-circle_margin(p, c, r) for c, r in self._exclusion.circles]
        if exclusion:
            margins.append((min(exclusion), "exclusion zone"))        # outside ALL of them
        return margins

    def path_is_clear(self, start: tuple[float, float], end: tuple[float, float],
                      params: Mapping[str, float], step_m: float = 1.0) -> bool:
        """Can a straight run from `start` to `end` be made without violating the
        uploaded shapes? (RTL drives straight home; it must not cross a no-go area.)

        The path is sampled every `step_m`. A boat that STARTS in a violation
        (inside a no-go zone) may leave it, but once the path has reached allowed
        water it must stay there, and it must end there. The home circle is not
        checked: it is convex and home is its centre, so a run towards home never
        leaves it. Returns True when no uploaded shape applies."""
        if (params["FENCE_ENABLE"] < 0.5 or not int(params["FENCE_TYPE"]) & FENCE_TYPE_POLYGON
                or self._frame is None or not self.has_shapes):
            return True
        a, b = self._frame.to_enu(*start), self._frame.to_enu(*end)
        samples = max(2, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step_m)) + 1)
        entered_allowed, margin = False, 0.0
        for k in range(samples):
            f = k / (samples - 1)
            margin = min(m for m, _ in self._shape_margins((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))))
            if margin >= 0.0:
                entered_allowed = True
            elif entered_allowed:
                return False
        return margin >= 0.0

    def waypoints_outside(self, waypoints: Sequence[tuple[float, float]], home: tuple[float, float] | None,
                          params: Mapping[str, float]) -> list[tuple[int, float]]:
        """(index, margin) of every waypoint that lies outside the fence."""
        outside = []
        for index, (lat, lon) in enumerate(waypoints):
            if index == 0:                      # slot 0 is home, never a target
                continue
            status = self.evaluate(lat, lon, home, params)
            if status is not None and status.breached:
                outside.append((index, status.margin_m))
        return outside
