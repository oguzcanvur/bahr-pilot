"""Navigator geometry and acceptance-radius behaviour."""
from __future__ import annotations

import pytest

from bahr_pilot.modes import MODE_GUIDED
from bahr_pilot.navigation import Navigator, bearing_deg, distance_m
from bahr_pilot.nucleo_link import PULSE_NEUTRAL_US
from bahr_pilot.state import VehicleState


def test_distance_and_bearing_due_north():
    # 0.001 deg of latitude is ~111.2 m everywhere.
    assert distance_m(41.0, 29.0, 41.001, 29.0) == pytest.approx(111.2, abs=0.5)
    assert bearing_deg(41.0, 29.0, 41.001, 29.0) == pytest.approx(0.0, abs=1e-6)
    assert bearing_deg(41.0, 29.0, 41.0, 29.001) == pytest.approx(90.0, abs=0.01)


def _state_near_target(distance_north_m: float) -> tuple[VehicleState, Navigator]:
    state = VehicleState()
    state.lat, state.lon = 41.0, 29.0
    state.mode = MODE_GUIDED
    nav = Navigator()
    nav.guided_target = (41.0 + distance_north_m / 111_195.0, 29.0)
    return state, nav


def test_step_uses_the_given_wp_radius():
    """WP_RADIUS used to be a hardcoded 3.0 in navigation.py; changing the
    parameter from BAHR-GCS had no effect."""
    state, nav = _state_near_target(2.0)
    assert nav.step(state, 0.5, wp_radius_m=3.0)[2] is True
    assert nav.step(state, 0.5, wp_radius_m=1.0)[2] is False


def test_step_holds_neutral_without_position():
    state, nav = _state_near_target(50.0)
    state.lat = state.lon = None
    assert nav.step(state, 0.5, 3.0) == (PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False)
