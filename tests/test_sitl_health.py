"""The health table on the whole vehicle in the simulator: a clean flight says nothing, and
faults injected into the sensors arrive as the operator's messages, in order."""
from __future__ import annotations

import pytest

from bahr_pilot.diagnostics import Level, Part
from bahr_pilot.modes import MODE_AUTO
from bahr_pilot.sitl.sensors import Fault
from tests.test_sitl_vehicle import Flight

PREFIXES = tuple(f"{part.value}:" for part in Part)


def health_messages(flight, prefix: str) -> list[tuple[float, str]]:
    return [(t, text) for t, text in flight.messages if text.startswith(prefix)]


@pytest.fixture(scope="module")
def clean():
    return Flight()


def test_a_clean_flight_raises_no_health_message(clean):
    assert clean.completed
    said = [text for _, text in clean.messages if text.startswith(PREFIXES)]
    assert said == []
    assert all(row.level == Level.OK for row in clean.vehicle.health.rows())


def test_a_gnss_outage_is_announced_as_it_gets_worse_and_again_when_it_is_over():
    """60-80 s without GNSS: first coasting on dead reckoning, then no position at all, then back."""
    flight = Flight(faults=[Fault("gnss_outage", 60.0, 80.0)], seconds=130.0)
    said = health_messages(flight, "GNSS:")
    assert [text for _, text in said] == ["GNSS: dead reckoning", "GNSS: no position", "GNSS: recovered"]
    t_dr, t_none, t_back = (t for t, _ in said)
    assert 61.0 < t_dr < 65.0                                       # the fix times out after 1 s; the table waits 1 s more
    assert 65.0 < t_none < 76.0 and t_none - t_dr >= 5.0 - 1e-6     # dead reckoning gives up at 5 s; one message per 5 s
    assert 80.0 < t_back < 95.0                                     # 3 s calm after the fix returns


def test_a_lost_ground_station_is_announced_and_so_is_its_return():
    """With the default 3 s FS_TIMEOUT the failsafe fires the moment the 2 s "silent" warning would have
    cleared its 1 s debounce, so the table goes straight to ERROR: the warning stage is skipped, correctly."""
    flight = Flight(gcs_loss=(50.0, 70.0), seconds=110.0)
    said = health_messages(flight, "GCS link:")
    assert [text for _, text in said] == ["GCS link: link lost", "GCS link: recovered"]
    assert flight.said("Failsafe: GCS link lost")                   # failsafe.py still says its own
    assert not any(text.startswith("failsafe:") for _, text in flight.messages)    # and the table does not repeat it
    assert 50.0 < said[0][0] < 60.0 and said[-1][0] > 70.0


def test_a_quiet_ground_station_is_a_warning_when_the_failsafe_is_not_yet_due():
    flight = Flight(gcs_loss=(50.0, 58.0), seconds=110.0, params={"FS_TIMEOUT": 10.0})
    said = health_messages(flight, "GCS link:")
    assert [text for _, text in said][1:] == ["GCS link: recovered"] and said[0][1].startswith("GCS link: silent")
    assert not flight.said("Failsafe:")                             # 8 s of silence under a 10 s timeout: no failsafe
    assert flight.mode_between(50.0, 100.0) == {MODE_AUTO}          # and the boat never stopped
