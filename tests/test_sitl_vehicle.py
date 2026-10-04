"""The REAL Vehicle (its own _motor_command, navigator and estimator wiring)
flown through missions in the simulator, with faults injected.

What is asserted here are SAFETY INVARIANTS and completion, not accuracy
numbers: the simulated boat's parameters are placeholders (docs/SITL.md), so
how far it strays from the line says nothing about the real hull. The
invariants are what must hold for any boat.

The measured baseline of the current waypoint-to-waypoint steering is recorded
in docs/PHASE_REPORTS.md so Phases 11/12 (line following, PID) can show they
improved it.
"""
from __future__ import annotations

import math
import statistics

import pytest

from bahr_pilot import geo
from bahr_pilot.estimator import EstimateStatus
from bahr_pilot.modes import MODE_AUTO, MODE_HOLD
from bahr_pilot.pathfollow import Guidance
from bahr_pilot.sitl.boat import BoatParams, Environment
from bahr_pilot.sitl.harness import Sitl, Trace, cross_track_errors, vehicle_controller
from bahr_pilot.sitl.sensors import Fault, SensorConfig
from tests.test_vehicle import make_vehicle

ROUTE = ((0.0, 100.0), (100.0, 100.0), (100.0, 0.0))      # (east, north) m: north, east, south
NEUTRAL = 1500


class DirectToTarget:
    """The pre-Phase-11 behaviour as a PathFollower: steer at the target, ignore
    the line. Kept here as the yardstick pure pursuit is measured against, and
    to show the follower interface is genuinely swappable."""

    def guide(self, position, start, end, speed_mps, wp_radius_m, lookahead_m):
        along, cross, length = geo.segment_errors(position, start, end)
        east, north = end[0] - position[0], end[1] - position[1]
        distance = math.hypot(east, north)
        return Guidance(math.degrees(math.atan2(east, north)) % 360.0, along, cross, length, distance,
                        lookahead_m, distance < wp_radius_m)


class Flight:
    """One simulated mission; everything the assertions need.

    `events` are operator actions: (sim time s, function(vehicle)), e.g. putting
    the vehicle back into AUTO after a failsafe held it. `gcs_loss` is a
    (start, end) window in which the simulated ground station is silent.
    "Completed" means the LAST WAYPOINT WAS REACHED, not that the mode became
    HOLD: a failsafe also ends in HOLD (an earlier version of these tests counted
    that as completion and kept passing while the boat sat stopped on leg 0)."""

    def __init__(self, *, faults=(), env=None, boat=None, seconds=320.0, route=ROUTE, follower=None,
                 params=None, events=(), gcs_loss=None, fence=None, seabed=None, sensor=None,
                 telemetry=False, log_dir=None):
        self.route = route
        self.sitl = Sitl(sensors=SensorConfig(faults=tuple(faults), **(sensor or {})), env=env,
                         boat_params=boat, seabed=seabed)
        self.vehicle = make_vehicle(log_dir=log_dir)
        v = self.vehicle
        if params:
            v.params.update(params)
        if follower is not None:
            v.navigator.follower = follower
        if fence:
            v.geofence.load(fence)
        v.navigator.mission = [self.sitl.geodetic(0.0, 0.0)] + [self.sitl.geodetic(*p) for p in route]
        v.navigator.mission_seq = 1
        v.state.mode = MODE_AUTO
        v.state.armed = True
        v.state.home_lat, v.state.home_lon = self.sitl.geodetic(0.0, 0.0)
        self.legs, previous = [], (0.0, 0.0)
        for point in route:
            self.legs.append((previous, point))
            previous = point

        self.reached: list[tuple[float, int]] = []
        self.depth_messages: list[tuple[float, int]] = []
        self.messages: list[tuple[float, str]] = []
        v.link.mav.mission_item_reached_send = lambda seq, *a, **k: self.reached.append((self.sitl.t, seq))
        v.link.mav.statustext_send = lambda severity, text: self.messages.append(
            (self.sitl.t, bytes(text).decode()))

        v.link.mav.distance_sensor_send = lambda now_ms, lo, hi, cm, *a, **k: self.depth_messages.append(
            (self.sitl.t, cm))
        pending = sorted(events, key=lambda event: event[0])
        base = vehicle_controller(
            v, self.sitl,
            gcs_connected=lambda: not (gcs_loss and gcs_loss[0] <= self.sitl.t < gcs_loss[1]), telemetry=telemetry)

        def controller(pose, now):
            while pending and pending[0][0] <= now:
                pending.pop(0)[1](v)
            return base(pose, now)

        self.trace: Trace = self.sitl.run(
            controller, seconds,
            probes={"leg": lambda: v.navigator.mission_seq - 1, "mode": lambda: v.state.mode})
        self.mission_end = (self.trace.index_at(self.reached[-1][0])
                            if len(self.reached) >= len(route) else None)

    @property
    def completed(self) -> bool:
        return self.mission_end is not None

    def said(self, fragment: str) -> bool:
        return any(fragment in text for _, text in self.messages)

    def mode_between(self, start_s: float, end_s: float) -> set[int]:
        t = self.trace
        return {m for tt, m in zip(t.t, t.probes["mode"]) if start_s <= tt < end_s}

    def xte(self) -> list[float]:
        """Signed cross-track error of the TRUE position, while the mission lasted."""
        errors = cross_track_errors(self.trace, self.legs, "leg")
        return errors[:self.mission_end] if self.completed else errors

    def distance_to_last_waypoint(self) -> float:
        last = self.route[-1]
        return math.hypot(self.trace.east[-1] - last[0], self.trace.north[-1] - last[1])

    def transitions(self) -> list[int]:
        legs = self.trace.probes["leg"]
        return [legs[i] for i in range(1, len(legs)) if legs[i] != legs[i - 1]]


def reengage(seconds: float):
    """The operator selects AUTO again."""
    return (seconds, lambda v: v.set_mode(MODE_AUTO))


@pytest.fixture(scope="module")
def calm():
    return Flight()


# -- the baseline mission ---------------------------------------------------------------------------

def test_calm_water_mission_completes_and_the_vehicle_holds(calm):
    assert calm.completed
    assert calm.transitions() == [1, 2]            # legs 0 -> 1 -> 2, in order, none skipped
    assert calm.trace.probes["mode"][-1] == MODE_HOLD
    assert calm.distance_to_last_waypoint() < 6.0   # WP_RADIUS 3 m plus the coast after arrival


def test_calm_water_stays_close_to_the_line(calm):
    errors = calm.xte()
    assert statistics.fmean(abs(e) for e in errors) < 1.0
    assert max(abs(e) for e in errors) < 5.0        # corner cutting inside the 3 m acceptance radius


def test_the_estimator_is_accurate_throughout_the_mission(calm):
    t = calm.trace
    start = t.index_at(20.0)
    pos = [math.hypot(t.est_east[i] - t.east[i], t.est_north[i] - t.north[i])
           for i in range(start, calm.mission_end) if t.est_east[i] is not None]
    assert statistics.fmean(pos) < 0.2 and max(pos) < 0.6


def test_commands_are_always_legal_integers_in_range(calm):
    for m in calm.trace.m1 + calm.trace.m2:
        assert isinstance(m, int) and 1000 <= m <= 2000


def test_no_motion_is_commanded_without_a_valid_position_and_heading(calm):
    """The rule the whole estimator exists to enforce: never drive on data the
    estimator has withdrawn."""
    t = calm.trace
    for i in range(len(t)):
        if t.status[i] == EstimateStatus.NONE or not t.heading_valid[i]:
            assert (t.m1[i], t.m2[i]) == (NEUTRAL, NEUTRAL), f"tick {i}, t={t.t[i]:.2f}"


def test_the_boat_waits_for_the_estimator_before_moving():
    """At the start nothing is known (first fix at 0.2 s, first heading at 1 s):
    the first ticks must be neutral even though the mission is armed and ready."""
    flight = Flight(seconds=3.0)
    assert flight.trace.m1[0] == NEUTRAL
    assert all(m == NEUTRAL for m in flight.trace.m1[:15])        # first 0.75 s
    assert max(flight.trace.m1) > NEUTRAL                          # and then it goes


# -- faults ---------------------------------------------------------------------------------------------

def test_a_gnss_outage_longer_than_dead_reckoning_holds_the_boat_until_the_operator_resumes():
    """GNSS gone 60-80 s. The position is dead-reckoned for 5 s, then withdrawn, the
    failsafe holds the boat, and it STAYS held after the fix returns: the mission
    resumes only when the operator selects AUTO again (here at 95 s)."""
    flight = Flight(faults=[Fault("gnss_outage", 60.0, 80.0)], seconds=400.0, events=[reengage(95.0)])
    t = flight.trace
    assert t.status[t.index_at(61.5)] == EstimateStatus.DEAD_RECKONING          # it coasts first
    assert flight.said("Failsafe: position lost (HOLD)")
    assert not flight.said("Failsafe cleared")          # found in this flight: it said "cleared" 0.05 s after the
    #                                                     trigger, with the GNSS still down (the HOLD ended autonomy)
    assert flight.mode_between(75.0, 94.0) == {MODE_HOLD}                       # fix back since 80 s, still held
    gave_up = [i for i in range(len(t)) if 60.0 <= t.t[i] < 80.0 and t.status[i] == EstimateStatus.NONE]
    assert gave_up and all((t.m1[i], t.m2[i]) == (NEUTRAL, NEUTRAL) for i in gave_up)
    assert flight.completed and flight.trace.t[flight.mission_end] > 95.0       # finished after the re-engage
    assert max(abs(e) for e in flight.xte()) < 8.0


def test_without_the_operator_a_held_mission_stays_held():
    flight = Flight(faults=[Fault("gnss_outage", 60.0, 80.0)], seconds=300.0)
    assert not flight.completed
    assert flight.mode_between(100.0, 300.0) == {MODE_HOLD}
    assert flight.trace.speed[-1] < 0.3                                        # drifting at most, not driving


def test_a_short_gnss_outage_is_bridged_by_dead_reckoning():
    flight = Flight(faults=[Fault("gnss_outage", 40.0, 43.0)])
    t = flight.trace
    window = [i for i in range(len(t)) if 41.5 <= t.t[i] < 42.9]     # past the 1 s fix timeout, before the return
    assert all(t.status[i] == EstimateStatus.DEAD_RECKONING for i in window)
    assert flight.completed and max(abs(e) for e in flight.xte()) < 5.0


def test_a_gnss_position_jump_is_rejected(calm):
    flight = Flight(faults=[Fault("gnss_jump", 50.0, 52.0, (30.0, -20.0))])
    assert flight.sitl.estimator.diag.position_rejected >= 5
    assert flight.completed
    # the wild fixes never pulled the boat off course noticeably
    assert max(abs(e) for e in flight.xte()) < max(abs(e) for e in calm.xte()) + 2.0


def test_a_gnss_heading_outage_is_carried_by_the_gyro_then_holds_the_boat():
    flight = Flight(faults=[Fault("heading_outage", 40.0, 100.0)], seconds=400.0, events=[reengage(110.0)])
    t = flight.trace
    during = [i for i in range(t.index_at(41.0), t.index_at(100.0))]
    valid = [i for i in during if t.heading_valid[i]]
    assert valid and t.t[valid[-1]] < 52.0           # last accepted ~40 s + the 10 s limit
    assert all((t.m1[i], t.m2[i]) == (NEUTRAL, NEUTRAL) for i in during if not t.heading_valid[i])
    assert flight.said("Failsafe: heading lost (HOLD)")
    assert flight.mode_between(70.0, 109.0) == {MODE_HOLD}
    assert flight.completed                           # after the operator resumed at 110 s


# The heading-validity contract is tested with the failsafe OFF so the boat keeps being
# driven (and turning) under the fault, which is the stressful case. What the failsafe does
# about the same faults is tested separately below.
NO_EKF_FAILSAFE = {"FS_EKF_ACTION": 0.0}


def heading_errors_while_valid(flight, after_s=0.0):
    tr = flight.trace
    return [abs(geo.wrap_180(tr.est_heading[i] - tr.heading[i]))
            for i in range(tr.index_at(after_s), len(tr)) if tr.est_heading[i] is not None]


def test_a_dead_gyro_withdraws_the_heading_instead_of_letting_it_go_stale():
    """Found in the simulator: with the IMU's gyro flagged invalid the heading
    used to stay 'valid' for a second at a time while the boat was turning at
    ~38 deg/s - up to 180 degrees wrong, and the boat spun. Without a gyro the
    doubt now grows at the boat's maximum yaw rate, so the heading is valid
    only for the ~0.15 s after each GNSS heading."""
    flight = Flight(faults=[Fault("gyro_invalid", 30.0, 120.0)], params=NO_EKF_FAILSAFE)
    errors = heading_errors_while_valid(flight, after_s=35.0)
    assert errors and max(errors) < 30.0
    assert sum(e > 10.0 for e in errors) / len(errors) < 0.02
    tr = flight.trace
    window = range(tr.index_at(35.0), tr.index_at(120.0))
    valid_share = sum(tr.heading_valid[i] for i in window) / len(window)
    assert valid_share < 0.5           # mostly withdrawn: the honest answer without a gyro


def test_a_frozen_gyro_is_overruled_by_the_receiver_and_the_heading_withdrawn():
    """A gyro stuck at a non-zero rate disagrees with GNSS; the filter used to
    reject the receiver 25 times while reporting its own frozen heading as
    valid (up to 167 degrees wrong). Now the disagreement withdraws the
    estimate; the next accepted measurement restores it. The remaining error
    (worst ~21 degrees) is the time until the next 1 Hz GNSS heading can reveal
    the fault: a frozen gyro cannot be detected from the gyro alone."""
    flight = Flight(faults=[Fault("gyro_stuck", 60.0, 90.0)], params=NO_EKF_FAILSAFE)
    errors = heading_errors_while_valid(flight, after_s=62.0)
    assert errors and max(errors) < 30.0
    assert sum(e > 10.0 for e in errors) / len(errors) < 0.02
    assert flight.sitl.estimator.diag.heading_rejected > 0


@pytest.mark.parametrize("faults", [
    [],
    [Fault("gnss_outage", 50.0, 70.0)],
    [Fault("heading_outage", 40.0, 90.0)],
    [Fault("gyro_invalid", 30.0, 120.0)],
    [Fault("gyro_stuck", 60.0, 90.0)],
    [Fault("gyro_bias_step", 40.0, 100.0, 3.0)],
    [Fault("gnss_jump", 50.0, 52.0, (30.0, -20.0))],
], ids=["none", "gnss_outage", "heading_outage", "gyro_invalid", "gyro_stuck", "gyro_bias_step", "gnss_jump"])
def test_a_heading_reported_valid_is_nearly_always_right(faults):
    """The estimator's whole contract: valid means usable. Across the fault
    scenarios, a heading flagged valid is wrong by more than 10 degrees on
    fewer than 2 % of ticks and never by 30 degrees."""
    errors = heading_errors_while_valid(Flight(faults=faults, params=NO_EKF_FAILSAFE), after_s=5.0)
    assert errors
    assert max(errors) < 30.0
    assert sum(e > 10.0 for e in errors) / len(errors) < 0.02


def test_a_cross_current_does_not_make_the_mission_fail():
    flight = Flight(env=Environment(current_east_mps=0.3))
    assert flight.completed
    assert flight.transitions() == [1, 2]


def test_disarming_mid_mission_stops_the_boat():
    flight = Flight(seconds=40.0)
    flight.vehicle.state.armed = False
    flight.sitl.armed = False
    flight.sitl.run(vehicle_controller(flight.vehicle, flight.sitl), 10.0)
    assert flight.sitl.boat.motors.slewed == [0.0, 0.0]


def test_swapped_motor_wiring_is_not_detected_by_the_software_today():
    """KNOWN LIMITATION, pinned so it is not forgotten: with the ESC leads
    exchanged the boat steers away from every waypoint and nothing notices.
    Phase 23 (calibration: a commanded test turn that must match the IMU) is
    what has to catch this before the first mission."""
    flight = Flight(boat=BoatParams(swapped_motors=True), seconds=200.0)
    assert not flight.completed
    assert flight.transitions() == []


# -- Phase 11: pure pursuit against the old steer-at-the-target behaviour ---------------------------

def mean_abs(values):
    return statistics.fmean(abs(v) for v in values)


@pytest.mark.parametrize("label,env,max_ratio", [
    ("0.5 m/s cross current", Environment(current_east_mps=0.5), 0.3),
    ("0.2 m/s cross current", Environment(current_east_mps=0.2), 0.3),
    ("15 N cross wind", Environment(wind_force_east_n=15.0), 0.3),
])
def test_pure_pursuit_holds_the_line_far_better_than_steering_at_the_target(label, env, max_ratio):
    """Same boat, same disturbance, same mission; only the follower differs.
    Measured: 7.2 m -> 1.1 m mean error at 0.5 m/s current."""
    direct = Flight(env=env, follower=DirectToTarget())
    pursuit = Flight(env=env)
    assert direct.completed and pursuit.completed
    assert mean_abs(pursuit.xte()) < max_ratio * mean_abs(direct.xte()), label
    assert mean_abs(pursuit.xte()) < 2.0 and max(abs(e) for e in pursuit.xte()) < 5.5


def test_pure_pursuit_is_no_worse_in_calm_water(calm):
    direct = Flight(follower=DirectToTarget())
    assert mean_abs(calm.xte()) <= mean_abs(direct.xte()) + 0.1
    assert max(abs(e) for e in calm.xte()) < 3.5


def test_a_shorter_l1_period_holds_the_line_tighter_in_a_current():
    env = Environment(current_east_mps=0.5)
    loose = Flight(env=env)
    tight = Flight(env=env, params={"NAVL1_PERIOD": 5.0})
    assert tight.completed and mean_abs(tight.xte()) < mean_abs(loose.xte())


def test_the_boat_never_orbits_a_waypoint_in_a_strong_current():
    """A 0.5 m/s current can hold the boat just outside a 3 m radius; the
    end-plane rule must still release it. The mission has to finish and not take
    wildly longer than in calm water."""
    windy = Flight(env=Environment(current_east_mps=0.5))
    calm_flight = Flight()
    assert windy.completed
    assert windy.trace.t[windy.mission_end] < 1.5 * calm_flight.trace.t[calm_flight.mission_end]


# -- Phase 20: failsafes in the simulator ---------------------------------------------------------------------

def test_a_dead_gyro_ends_in_hold_with_the_boat_stopped_not_spinning():
    """Before Phases 20 and 26 this fault sent the boat into uncontrolled circles."""
    flight = Flight(faults=[Fault("gyro_invalid", 30.0, 400.0)], seconds=200.0)
    t = flight.trace
    assert flight.said("Failsafe: heading lost (HOLD)")
    assert flight.mode_between(60.0, 200.0) == {MODE_HOLD}
    late = range(t.index_at(100.0), len(t))
    assert max(abs(geo.wrap_180(t.heading[i] - t.heading[i - 20])) for i in late) < 5.0      # no spinning
    assert t.speed[-1] < 0.3


def test_the_gcs_going_silent_holds_the_boat_and_it_waits_for_the_operator():
    flight = Flight(gcs_loss=(50.0, 70.0), seconds=300.0, events=[reengage(90.0)])
    assert flight.said("Failsafe: GCS link lost (HOLD)")
    held = flight.mode_between(54.0, 89.0)
    assert held == {MODE_HOLD}                                          # also after the link is back at 70 s
    assert flight.completed and flight.trace.t[flight.mission_end] > 90.0


def test_fs_action_rtl_brings_the_boat_home_when_the_gcs_goes_silent():
    from bahr_pilot.modes import MODE_RTL
    flight = Flight(gcs_loss=(50.0, 1000.0), params={"FS_ACTION": 1.0}, seconds=400.0)
    assert flight.said("Failsafe: GCS link lost (RTL)")
    assert MODE_RTL in flight.mode_between(51.0, 400.0)
    home_distance = math.hypot(flight.trace.east[-1], flight.trace.north[-1])
    assert home_distance < 8.0                                          # back at home (0, 0), within WP_RADIUS and the coast
    assert flight.mode_between(399.0, 400.0) == {MODE_HOLD}            # arrival ends in HOLD


def test_fs_action_report_only_leaves_the_mission_running():
    flight = Flight(gcs_loss=(50.0, 1000.0), params={"FS_ACTION": 0.0}, seconds=320.0)
    assert flight.said("Failsafe: GCS link lost (report only)")
    assert flight.completed


def test_a_short_dead_reckoned_dropout_triggers_no_failsafe():
    flight = Flight(faults=[Fault("gnss_outage", 40.0, 43.0)])
    assert not flight.said("Failsafe") and flight.completed


# -- Phase 21: geofence in the simulator ------------------------------------------------------------------------

from bahr_pilot.geofence import CMD_POLYGON_EXCLUSION, CMD_POLYGON_INCLUSION, Geofence
from bahr_pilot.modes import MODE_RTL

_FRAME = geo.LocalFrame(41.0, 29.0)                   # the simulator's local frame (sitl.harness.ORIGIN)
_HOME = _FRAME.to_geodetic(0.0, 0.0)
CIRCLE_120 = {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 2.0, "FENCE_RADIUS": 120.0, "FENCE_MARGIN": 2.0, "FENCE_ACTION": 1.0}
POLYGONS = {"FENCE_ENABLE": 1.0, "FENCE_TYPE": 4.0, "FENCE_RADIUS": 300.0, "FENCE_MARGIN": 2.0, "FENCE_ACTION": 2.0}


def fence_items(command, vertices):
    return [(command, float(len(vertices)), *_FRAME.to_geodetic(*v)) for v in vertices]


def worst_margin(flight, fence_params, items=()):
    """The smallest signed distance to the fence the TRUE position ever had (negative = outside)."""
    fence = Geofence()
    if items:
        fence.load(items)
    return min(fence.evaluate(*_FRAME.to_geodetic(e, n), _HOME, fence_params).margin_m
               for e, n in zip(flight.trace.east, flight.trace.north))


def test_without_a_fence_the_boat_goes_wherever_the_mission_says(calm):
    far = max(math.hypot(e, n) for e, n in zip(calm.trace.east, calm.trace.north))
    assert far > 135.0                                           # the route reaches 141 m from home


def test_a_fence_switched_on_with_nothing_to_check_changes_nothing():
    flight = Flight(params={"FENCE_ENABLE": 1.0, "FENCE_TYPE": 4.0})        # polygon type, no polygon uploaded
    assert flight.completed and not flight.said("fence")


def test_the_circle_around_home_holds_the_boat_at_the_boundary():
    """Measured: the boat stops 0.7 m outside the 120 m circle."""
    flight = Flight(params={**CIRCLE_120, "FENCE_ACTION": 2.0}, seconds=300.0)
    assert flight.said("Failsafe: fence breached (HOLD)")
    assert not flight.completed
    assert worst_margin(flight, CIRCLE_120) > -2.0
    assert flight.mode_between(250.0, 300.0) == {MODE_HOLD} and flight.trace.speed[-1] < 0.1


def test_fence_action_rtl_turns_the_boat_back_and_it_never_leaves():
    """Measured: it never crosses the boundary (worst margin +0.7 m) and ends at home in HOLD."""
    flight = Flight(params=CIRCLE_120, seconds=450.0)
    assert flight.said("Failsafe: fence breached (RTL)")
    assert MODE_RTL in flight.mode_between(0.0, 450.0)
    assert worst_margin(flight, CIRCLE_120) > -1.5
    assert math.hypot(flight.trace.east[-1], flight.trace.north[-1]) < 6.0           # home
    assert flight.mode_between(449.0, 450.0) == {MODE_HOLD}


def test_fence_action_report_only_lets_the_boat_cross():
    flight = Flight(params={**CIRCLE_120, "FENCE_ACTION": 0.0}, seconds=300.0)
    assert flight.said("Failsafe: fence breached (report only)")
    assert flight.completed and worst_margin(flight, CIRCLE_120) < -15.0


def test_an_exclusion_zone_across_the_route_stops_the_boat_before_it():
    """Measured: it stops 0.7 m inside the 40 m-edge of a no-go square."""
    zone = fence_items(CMD_POLYGON_EXCLUSION, [(-20.0, 40.0), (20.0, 40.0), (20.0, 60.0), (-20.0, 60.0)])
    flight = Flight(params=POLYGONS, fence=zone, seconds=250.0)
    assert flight.said("Failsafe: fence breached (HOLD)")
    assert worst_margin(flight, POLYGONS, zone) > -2.0
    assert max(flight.trace.north) < 43.0 and not flight.completed


def test_leaving_an_inclusion_zone_is_stopped_at_its_edge():
    box = fence_items(CMD_POLYGON_INCLUSION, [(-30.0, -30.0), (130.0, -30.0), (130.0, 80.0), (-30.0, 80.0)])
    flight = Flight(params=POLYGONS, fence=box, seconds=250.0)
    assert flight.said("Failsafe: fence breached (HOLD)")
    assert worst_margin(flight, POLYGONS, box) > -2.0 and max(flight.trace.north) < 82.0


def test_looking_ahead_is_what_keeps_the_overshoot_small():
    """The same circle with the glide prediction switched off (a coast deceleration so
    large the stopping distance collapses to the reaction time) overshoots well over twice as far."""
    zone = fence_items(CMD_POLYGON_EXCLUSION, [(-20.0, 40.0), (20.0, 40.0), (20.0, 60.0), (-20.0, 60.0)])
    with_prediction = Flight(params=POLYGONS, fence=zone, seconds=250.0)
    without = Flight(params={**POLYGONS, "FENCE_COAST_DECEL": 1000.0}, fence=zone, seconds=250.0)
    assert -worst_margin(without, POLYGONS, zone) > 2.0 * max(0.1, -worst_margin(with_prediction, POLYGONS, zone))


def test_the_operator_cannot_resume_automatic_flight_outside_the_fence():
    flight = Flight(params={**CIRCLE_120, "FENCE_ACTION": 2.0}, seconds=300.0, events=[reengage(200.0)])
    assert flight.mode_between(201.0, 300.0) == {MODE_HOLD}                    # AUTO was refused again, still held
    assert not flight.completed
