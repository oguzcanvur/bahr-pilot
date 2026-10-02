"""Waypoint following: turns the active target (mission waypoint, guided
goto, or RTL-to-home) plus the current position/heading from bahr_pilot.state into
differential-drive motor commands.

Geometry helpers are the same formulas sim/fake_vehicle.py uses for its
simulated kinematics — not re-derived here, just reused, since they're
already exercised by the GCS's own test suite.
"""
from __future__ import annotations

import math

from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_RTL
from bahr_pilot.nucleo_link import PULSE_MAX_US, PULSE_MIN_US, PULSE_NEUTRAL_US

EARTH_R = 6371000.0

# Steering tuning — not yet verified against the real boat's turning
# behaviour or which motor ends up physically left/right.
TURN_GAIN = 0.6
SPEED_SPAN_US = 500  # pulse swing from neutral at full commanded speed


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = phi2 - phi1
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)
    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    return math.degrees(math.atan2(y, x)) % 360.0


def _clamp_pulse(pulse_us: float) -> int:
    return max(PULSE_MIN_US, min(PULSE_MAX_US, int(pulse_us)))


def steer_towards(lat: float, lon: float, heading_deg: float,
                   target_lat: float, target_lon: float,
                   cruise_frac: float) -> tuple[int, int]:
    """Motor1/motor2 pulses (us) to drive from (lat, lon)/heading_deg
    towards the target. cruise_frac in [0, 1] scales top speed."""
    want = bearing_deg(lat, lon, target_lat, target_lon)
    error = ((want - heading_deg + 180.0) % 360.0) - 180.0

    # Slow down for sharp corrections instead of pirouetting at full speed.
    cornering = max(0.35, 1.0 - abs(error) / 90.0)
    forward = cruise_frac * cornering
    turn = max(-1.0, min(1.0, error / 90.0)) * TURN_GAIN

    m1 = PULSE_NEUTRAL_US + SPEED_SPAN_US * (forward - turn)
    m2 = PULSE_NEUTRAL_US + SPEED_SPAN_US * (forward + turn)
    return _clamp_pulse(m1), _clamp_pulse(m2)


class Navigator:
    """Holds the uploaded mission (slot 0 = home, matching
    sim/fake_vehicle.py's convention) and the guided-mode target, and
    produces motor commands for whichever mode is active."""

    def __init__(self) -> None:
        self.mission: list[tuple[float, float]] = []
        self.mission_seq = 0
        self.mission_paused = False
        self.guided_target: tuple[float, float] | None = None

    def active_target(self, mode: int, home: tuple[float, float] | None):
        if mode == MODE_GUIDED:
            return self.guided_target
        if mode == MODE_RTL:
            return home
        if mode == MODE_AUTO and not self.mission_paused:
            if 1 <= self.mission_seq < len(self.mission):
                return self.mission[self.mission_seq]
        return None

    def step(self, state, cruise_frac: float, wp_radius_m: float) -> tuple[int, int, bool]:
        """Returns (motor1_us, motor2_us, reached_current_target)."""
        if state.lat is None or state.lon is None:
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False

        home = (state.home_lat, state.home_lon) if state.home_lat is not None else None
        target = self.active_target(state.mode, home)
        if target is None:
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False

        remaining = distance_m(state.lat, state.lon, *target)
        if remaining < wp_radius_m:
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, True

        m1, m2 = steer_towards(state.lat, state.lon, state.heading_deg, *target, cruise_frac)
        return m1, m2, False
