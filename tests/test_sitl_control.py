"""Phase 12 closed loop: the heading and speed controllers on the simulated
boat - step responses, robustness to the boat being wrong, unequal motors.

The gains were tuned on the placeholder boat, so the point of the sweep is
that they do not need the boat to be right: mass 0.5-2x, yaw inertia 0.3-3x,
thrust 0.6-1.4x and drag 0.5-2x all still fly the mission. Thresholds below
sit a margin above what was measured (quoted in each docstring)."""
from __future__ import annotations

import dataclasses
import statistics

import pytest

from bahr_pilot import geo
from bahr_pilot.control import ControlConfig, HeadingController, SpeedController, mix_to_commands
from bahr_pilot.navigation import commands_to_pulses
from bahr_pilot.sitl.boat import BoatParams, Environment
from bahr_pilot.sitl.harness import Sitl
from tests.test_sitl_vehicle import Flight, mean_abs

BASE = BoatParams()


def scaled(mass=1.0, inertia=1.0, thrust=1.0, drag=1.0, **extra) -> BoatParams:
    return dataclasses.replace(
        BASE, mass_kg=BASE.mass_kg * mass, inertia_kgm2=BASE.inertia_kgm2 * inertia,
        max_thrust_n=BASE.max_thrust_n * thrust,
        surge_drag_lin=BASE.surge_drag_lin * drag, surge_drag_quad=BASE.surge_drag_quad * drag,
        yaw_drag_lin=BASE.yaw_drag_lin * drag, yaw_drag_quad=BASE.yaw_drag_quad * drag,
        sway_drag_lin=BASE.sway_drag_lin * drag, sway_drag_quad=BASE.sway_drag_quad * drag, **extra)


BOATS = {
    "nominal": BASE,
    "heavy_weak": scaled(mass=1.5, inertia=1.6, thrust=0.7),
    "light_strong": scaled(mass=0.7, inertia=0.6, thrust=1.3),
}


def drive(desired_heading, desired_speed, boat, seconds, env=None, cfg=None):
    """The two controllers on their own (no path following), driving the boat."""
    cfg = cfg or ControlConfig()
    heading_ctl, speed_ctl = HeadingController(cfg), SpeedController(cfg)
    last = {"t": None}

    def controller(pose, now):
        if not pose.heading_valid or not pose.position_valid:
            heading_ctl.reset(), speed_ctl.reset()
            last["t"] = None
            return 1500, 1500
        dt = 0.0 if last["t"] is None else pose.t - last["t"]
        last["t"] = pose.t
        speed = desired_speed(now)
        steering = heading_ctl.update(desired_heading(now), pose.heading_deg, pose.yaw_rate_dps, dt)
        throttle = speed_ctl.update(speed, pose.speed_mps, dt, 0.4 * speed)       # CRUISE: 60 % at 1.5 m/s
        return commands_to_pulses(*mix_to_commands(throttle, steering))

    sitl = Sitl(boat_params=boat, env=env, start_heading_deg=0.0)
    return sitl.run(controller, seconds)


def heading_step_metrics(boat, degrees, t_step=15.0):
    trace = drive(lambda t: 0.0 if t < t_step else degrees, lambda t: 1.5, boat, 50.0)
    h = [geo.wrap_180(x) for x in trace.heading[trace.index_at(t_step):]]
    overshoot = max(0.0, (max(h) - degrees) / degrees * 100.0)
    settle = next(((k + 1) / 20.0 for k in range(len(h) - 1, -1, -1) if abs(h[k] - degrees) > 2.0), 0.0)
    return overshoot, settle, h[-1]


# -- heading step response -----------------------------------------------------------------------------

@pytest.mark.parametrize("boat_name", list(BOATS))
@pytest.mark.parametrize("degrees", [20.0, 60.0, 120.0])
def test_heading_step_settles_with_little_overshoot(boat_name, degrees):
    """Measured worst case over these 9: 10.3 % overshoot, 10.5 s to within 2 deg
    (the heavy, weak boat on a 120 deg turn). The first guess (ATC_STR_ANG_P 1.5)
    overshot 24-53 % and took 10-16 s."""
    overshoot, settle, final = heading_step_metrics(BOATS[boat_name], degrees)
    assert overshoot < 18.0
    assert settle < 14.0
    assert abs(final - degrees) < 1.0


def test_a_steady_heading_does_not_make_the_steering_jitter():
    trace = drive(lambda t: 0.0, lambda t: 1.5, BASE, 60.0)
    steering = [(a - b) / 1000.0 for a, b in zip(trace.m1[-400:], trace.m2[-400:])]
    assert statistics.pstdev(steering) < 0.03             # measured ~0.012: gyro noise passing through


# -- speed -----------------------------------------------------------------------------------------------

def speed_step_metrics(boat, start, end, env=None, t_step=25.0):
    trace = drive(lambda t: 0.0, lambda t: start if t < t_step else end, boat, 80.0, env=env)
    v = trace.speed[trace.index_at(t_step):]
    overshoot = max(0.0, (max(v) - end) / (end - start) * 100.0) if end > start else 0.0
    settle = next(((k + 1) / 20.0 for k in range(len(v) - 1, -1, -1) if abs(v[k] - end) > 0.08), 0.0)
    return overshoot, settle, v[-1]


@pytest.mark.parametrize("boat_name", list(BOATS))
def test_speed_steps_settle_on_the_target(boat_name):
    """Measured worst: 18 % overshoot and 8.7 s settle on the light, strong boat
    (its throttle feed-forward is too large, the integrator has to take it back)."""
    up_overshoot, up_settle, up_final = speed_step_metrics(BOATS[boat_name], 0.0, 1.5)
    _, down_settle, down_final = speed_step_metrics(BOATS[boat_name], 1.5, 0.8)
    assert up_final == pytest.approx(1.5, abs=0.05) and down_final == pytest.approx(0.8, abs=0.05)
    assert up_overshoot < 30.0 and up_settle < 12.0 and down_settle < 12.0


def test_ground_speed_is_held_against_a_head_current():
    """The controller regulates speed over the GROUND (what sample spacing
    depends on): a 0.5 m/s current against the boat makes it push harder."""
    _, _, final = speed_step_metrics(BASE, 1.5, 1.5, env=Environment(current_north_mps=-0.5))
    assert final == pytest.approx(1.5, abs=0.03)


def test_a_boat_too_weak_for_the_speed_saturates_without_winding_up():
    """Head current 0.5 m/s on the heavy, weak boat needs more thrust than it
    has: the speed falls short, but the integrator must not wind up - after the
    current stops the speed returns to target promptly."""
    trace = drive(lambda t: 0.0, lambda t: 1.5, BOATS["heavy_weak"], 120.0, env=None)
    assert trace.speed[-1] == pytest.approx(1.5, abs=0.05)
    sitl = Sitl(boat_params=BOATS["heavy_weak"], env=Environment(current_north_mps=-0.5))
    cfg = ControlConfig()
    speed_ctl, heading_ctl = SpeedController(cfg), HeadingController(cfg)
    last = {"t": None}

    def controller(pose, now):
        if not pose.heading_valid:
            return 1500, 1500
        dt = 0.0 if last["t"] is None else pose.t - last["t"]
        last["t"] = pose.t
        throttle = speed_ctl.update(1.5, pose.speed_mps, dt, 0.6)
        steering = heading_ctl.update(0.0, pose.heading_deg, pose.yaw_rate_dps, dt)
        return commands_to_pulses(*mix_to_commands(throttle, steering))

    sitl.run(controller, 90.0)
    assert speed_ctl.pid.integral <= cfg.speed.i_max + 1e-9          # clamped, never beyond
    sitl.boat.env.current_north_mps = 0.0
    trace = sitl.run(controller, 40.0)
    assert trace.speed[-1] == pytest.approx(1.5, abs=0.1)


# -- the whole vehicle, with the boat wrong ---------------------------------------------------------

SWEEP = [  # mass, inertia, thrust, drag  (multipliers)
    (1.0, 1.0, 1.0, 1.0), (1.6, 1.6, 0.8, 1.0), (0.6, 0.6, 1.2, 1.0), (1.0, 1.0, 0.7, 1.5),
    (1.3, 2.0, 1.0, 0.6), (0.8, 0.5, 1.4, 2.0), (1.6, 2.0, 0.6, 2.0), (0.6, 0.5, 1.4, 0.5),
    (2.0, 3.0, 1.0, 1.0), (0.5, 0.3, 1.0, 1.0),
]


@pytest.mark.parametrize("mass,inertia,thrust,drag", SWEEP)
def test_the_mission_flies_whatever_the_boat_turns_out_to_be(mass, inertia, thrust, drag):
    """Measured over the sweep: every mission completes; mean line error
    <= 0.34 m in calm water (worst: the 2x mass, 3x inertia boat)."""
    flight = Flight(boat=scaled(mass, inertia, thrust, drag), seconds=600.0)
    assert flight.completed
    assert flight.transitions() == [1, 2]
    assert mean_abs(flight.xte()) < 0.7
    assert max(abs(e) for e in flight.xte()) < 4.5


@pytest.mark.parametrize("mass,inertia,thrust,drag", [SWEEP[0], SWEEP[3], SWEEP[6]])
def test_the_mission_flies_in_a_cross_current_with_a_wrong_boat(mass, inertia, thrust, drag):
    """Measured: mean line error <= 1.05 m at 0.4 m/s cross current (worst: heavy,
    weak, high-drag boat)."""
    flight = Flight(boat=scaled(mass, inertia, thrust, drag), env=Environment(current_east_mps=0.4), seconds=600.0)
    assert flight.completed
    assert mean_abs(flight.xte()) < 1.8


@pytest.mark.parametrize("gains", [(1.0, 0.85), (0.85, 1.0), (1.0, 0.70), (0.70, 1.0)])
def test_unequal_motors_are_trimmed_out_by_the_steering_integral(gains):
    """One motor up to 30 % weaker is a constant turning moment. Measured mean
    line error 0.12-0.29 m with the default integral against 0.22-0.42 m without."""
    flight = Flight(boat=dataclasses.replace(BASE, motor_gain=gains), seconds=400.0)
    assert flight.completed
    assert mean_abs(flight.xte()) < 0.45
    flight_without = Flight(boat=dataclasses.replace(BASE, motor_gain=gains), seconds=400.0,
                            params={"ATC_STR_RAT_I": 0.0})
    assert mean_abs(flight.xte()) <= mean_abs(flight_without.xte()) + 0.02


# -- bookkeeping --------------------------------------------------------------------------------------------

def test_the_controllers_reset_when_the_boat_stops_being_driven():
    """No integral may survive a stretch in which the heading was withdrawn."""
    flight = Flight(seconds=60.0)
    nav = flight.vehicle.navigator
    nav.heading_control.rate_pid.integral = 0.1
    nav.speed_control.pid.integral = 0.2
    from tests.poses import make_pose
    flight.vehicle.state.pose = make_pose(*flight.sitl.geodetic(0.0, 50.0), heading_valid=False, speed_mps=1.0)
    flight.vehicle._motor_command()
    assert nav.heading_control.rate_pid.integral == 0.0 and nav.speed_control.pid.integral == 0.0
    assert nav._last_time is None
