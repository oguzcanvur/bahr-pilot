"""Path following (Phase 11): turn "the leg a -> b" and the vehicle's position
into a desired heading.

The old navigation steered at the TARGET: a boat pushed sideways by a current
or wind then pointed at the waypoint while drifting off the planned line (0.5
m/s of cross current gave 7 m average, 13.5 m worst cross-track error in the
simulator; see docs/PHASE_REPORTS.md). A survey boat has to stay ON the line, so
this module steers at a point on the line a little way ahead instead.

Pure pursuit: project the vehicle onto the line (that gives the along-track
and cross-track distances), pick the aim point `lookahead` metres further
along, and head for it. Far from the line the aim point is ahead of the
projection so the boat approaches at an angle; close to it the boat runs
parallel. The lookahead follows ArduPilot's L1 distance,

    L = damping * period * ground_speed / pi      (clamped to [min, max])

so the real ArduRover parameters NAVL1_PERIOD and NAVL1_DAMPING (already in
BAHR-GCS) tune it the way an ArduPilot user expects.

Everything here is in a local east/north plane in metres; compass headings
(clockwise from north) only appear at the boundary. Frames: bahr_pilot/geo.py.
Cross-track is positive when the vehicle is to the RIGHT of the direction of
travel.

`PathFollower` is the interface: another algorithm (Stanley, L1) can replace
PurePursuit without touching the navigator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

from bahr_pilot import geo

Point = tuple[float, float]       # (east, north), metres


@dataclass(frozen=True)
class PathConfig:
    lookahead_min_m: float = 3.0      # never aim closer than this (a stopped boat has speed 0)
    lookahead_max_m: float = 30.0
    # Scales the cross-track offset when aiming: 1 = textbook pure pursuit,
    # > 1 returns to the line harder, < 1 softer. (Prompt's NAV_CROSSTRACK_GAIN.)
    crosstrack_gain: float = 1.0
    # A waypoint also counts as reached when the boat has crossed the plane
    # through it (perpendicular to the leg) within this many acceptance radii,
    # so a current that keeps it just outside the radius cannot make it orbit.
    pass_factor: float = 2.0


def lookahead_distance(speed_mps: float, period_s: float, damping: float, cfg: PathConfig) -> float:
    """ArduPilot's L1 distance, clamped."""
    speed = speed_mps if math.isfinite(speed_mps) and speed_mps > 0.0 else 0.0
    l1 = damping * period_s * speed / math.pi
    return max(cfg.lookahead_min_m, min(cfg.lookahead_max_m, l1))


@dataclass(frozen=True)
class Guidance:
    desired_heading_deg: float        # compass, [0, 360)
    along_track_m: float              # from the leg start, along the leg
    cross_track_m: float              # + = vehicle is right of the line
    leg_length_m: float
    distance_to_target_m: float
    lookahead_m: float
    reached: bool


class PathFollower(Protocol):
    def guide(self, position: Point, start: Point, end: Point, speed_mps: float,
              wp_radius_m: float, lookahead_m: float) -> Guidance: ...


def _compass_deg(east: float, north: float) -> float:
    return math.degrees(math.atan2(east, north)) % 360.0


class PurePursuit:
    def __init__(self, config: PathConfig | None = None) -> None:
        self.cfg = config or PathConfig()

    def guide(self, position: Point, start: Point, end: Point, speed_mps: float,
              wp_radius_m: float, lookahead_m: float) -> Guidance:
        along, cross, length = geo.segment_errors(position, start, end)
        to_end = math.hypot(end[0] - position[0], end[1] - position[1])
        reached = to_end < wp_radius_m or (
            length > 0.0 and along >= length and to_end < self.cfg.pass_factor * wp_radius_m)

        if length < 1e-6:                          # the leg has no direction: just go to the point
            aim_e, aim_n = end[0] - position[0], end[1] - position[1]
        else:
            ux, uy = (end[0] - start[0]) / length, (end[1] - start[1]) / length
            projection = (start[0] + along * ux, start[1] + along * uy)
            aim_along = max(0.0, min(length, along + lookahead_m))
            aim = (start[0] + aim_along * ux, start[1] + aim_along * uy)
            # the vehicle's offset from the line, weighted by the cross-track gain
            g = self.cfg.crosstrack_gain
            weighted = (projection[0] + g * (position[0] - projection[0]),
                        projection[1] + g * (position[1] - projection[1]))
            aim_e, aim_n = aim[0] - weighted[0], aim[1] - weighted[1]
            if aim_e == 0.0 and aim_n == 0.0:      # standing exactly on the aim point
                aim_e, aim_n = ux, uy
        return Guidance(
            desired_heading_deg=_compass_deg(aim_e, aim_n), along_track_m=along, cross_track_m=cross,
            leg_length_m=length, distance_to_target_m=to_end, lookahead_m=lookahead_m, reached=reached)
