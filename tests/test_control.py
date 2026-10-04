"""Phase 12: the PID, the heading and speed controllers, the motor mix.

Expected values are worked out by hand (each comment shows the arithmetic).
Closed-loop behaviour on the simulated boat is in test_sitl_control.py."""
from __future__ import annotations

import math

import pytest

from bahr_pilot.control import (
    ControlConfig, HeadingController, MAX_DT_S, PID, PidGains, SpeedController, mix_to_commands,
)

DT = 0.05


# -- PID ------------------------------------------------------------------------------------------

def test_proportional_term_and_output_clamp():
    pid = PID(PidGains(p=0.5))
    assert pid.update(1.0, 0.0, DT) == pytest.approx(0.5)            # 0.5 x error 1
    assert pid.update(0.5, 0.0, DT) == pytest.approx(0.25)
    assert pid.update(10.0, 0.0, DT) == 1.0                          # clamped to out_max
    assert pid.update(-10.0, 0.0, DT) == -1.0


def test_feedforward_acts_on_the_target_not_the_error():
    pid = PID(PidGains(ff=0.5))
    assert pid.update(0.4, 0.4, DT) == pytest.approx(0.2)            # zero error, FF 0.5 x 0.4


def test_integral_accumulates_error_times_time():
    pid = PID(PidGains(i=1.0))
    out = 0.0
    for _ in range(5):
        out = pid.update(0.1, 0.0, 0.1)                              # 1.0 x 0.1 x 0.1 per step
    assert out == pytest.approx(0.05)
    assert pid.integral == pytest.approx(0.05)


def test_integral_is_clamped_to_i_max():
    pid = PID(PidGains(i=1.0, i_max=0.3))
    for _ in range(1000):
        pid.update(1.0, 0.0, 0.1)
    assert pid.integral == pytest.approx(0.3)


def test_anti_windup_freezes_the_integral_while_saturated():
    pid = PID(PidGains(p=10.0, i=1.0, i_max=0.3))
    for _ in range(100):
        pid.update(1.0, 0.0, 0.1)                                    # P alone = 10 >> 1: saturated
    assert pid.integral == 0.0
    out = pid.update(-0.5, 0.0, 0.1)                                 # error reverses: unsaturated
    assert out == pytest.approx(-1.0)                                # P = -5 -> clamped; integral may move now
    for _ in range(50):
        pid.update(0.02, 0.0, 0.1)                                   # small positive error, no saturation
    assert pid.integral > 0.0


def test_the_derivative_acts_on_the_measurement_so_a_target_jump_causes_no_kick():
    pid = PID(PidGains(p=0.0, d=1.0, filter_hz=1000.0))
    pid.update(0.0, 0.0, DT)
    assert pid.update(5.0, 0.0, DT) == pytest.approx(0.0)            # target jumped, measurement did not


@pytest.mark.parametrize("filter_hz", [0.0, -3.0])
def test_a_nonpositive_filter_frequency_means_unfiltered_not_a_crash(filter_hz):
    pid = PID(PidGains(p=0.1, d=0.1, filter_hz=filter_hz))
    for k in range(5):
        assert math.isfinite(pid.update(1.0, 0.1 * k, DT))


def test_the_derivative_opposes_a_rising_measurement():
    pid = PID(PidGains(p=0.0, d=0.1, filter_hz=1000.0), out_min=-10.0, out_max=10.0)
    out = 0.0
    for k in range(40):
        out = pid.update(0.0, 1.0 * k * DT, DT)                      # measurement rising at 1 per second
    assert out == pytest.approx(-0.1, rel=0.02)                      # D x (-1)


def test_a_bad_timestep_never_makes_a_big_integral_or_derivative():
    pid = PID(PidGains(p=0.1, i=1.0, d=1.0))
    pid.update(1.0, 0.0, DT)
    before = pid.integral
    for bad_dt in (0.0, -1.0, 10.0, MAX_DT_S + 0.01, float("nan")):
        out = pid.update(1.0, 0.5, bad_dt)
        assert math.isfinite(out) and -1.0 <= out <= 1.0
        assert pid.integral == pytest.approx(before)


def test_non_finite_input_resets_and_outputs_zero():
    pid = PID(PidGains(p=1.0, i=1.0))
    pid.update(0.5, 0.0, DT)
    pid.update(0.5, 0.0, DT)
    assert pid.integral > 0
    assert pid.update(float("nan"), 0.0, DT) == 0.0
    assert pid.update(0.5, float("inf"), DT) == 0.0
    assert pid.integral == 0.0


def test_reset_clears_everything():
    pid = PID(PidGains(p=1.0, i=1.0, d=1.0))
    for _ in range(10):
        pid.update(1.0, 0.0, DT)
    pid.reset()
    assert pid.integral == 0.0 and pid.last_output == 0.0


# -- heading controller ----------------------------------------------------------------------------

CFG = ControlConfig()


def heading(desired, actual, rate_dps=0.0, controller=None):
    return (controller or HeadingController(CFG)).update(desired, actual, rate_dps, DT)


def test_steering_sign_right_is_positive():
    assert heading(100.0, 90.0) > 0                        # target clockwise of the bow
    assert heading(80.0, 90.0) < 0
    assert heading(90.0, 90.0) == pytest.approx(0.0, abs=1e-12)


def test_steering_wraps_through_north():
    assert heading(10.0, 350.0) > 0                        # +20 deg, not -340
    assert heading(350.0, 10.0) < 0
    assert heading(10.0, 350.0) == pytest.approx(heading(30.0, 10.0))     # same 20 deg error


PURE_PF = ControlConfig(str_rat=PidGains(p=0.30, i=0.0, d=0.0, ff=0.70))     # no integral: hand-checkable


def test_steering_value_for_a_small_error():
    """error 10 deg = 0.17453 rad -> desired rate = ANG_P 0.75 x 0.17453 = 0.1309 rad/s;
    rate PID (P 0.30 + FF 0.70, I off) on a stationary boat: 0.30 x 0.1309 + 0.70 x 0.1309 = 0.1309."""
    out = HeadingController(PURE_PF).update(100.0, 90.0, 0.0, DT)
    assert out == pytest.approx(0.75 * math.radians(10.0) * (0.30 + 0.70), rel=1e-9)


def test_the_integral_adds_a_little_on_the_very_first_step():
    """The default controller on the same case: desired rate 0.1309 rad/s, P 0.9 + FF 0.3 on a
    stationary boat = 1.2 x 0.1309 = 0.15708, plus I 0.03 x 0.1309 x dt (0.05) = 0.000196."""
    assert heading(100.0, 90.0) == pytest.approx(1.2 * 0.130900 + 0.03 * 0.130900 * DT, abs=2e-5)


def test_the_measured_turn_rate_damps_the_steering():
    """Same 10 deg error but already turning right at 10 deg/s (0.17453 rad/s): the
    desired rate is only 0.1309 rad/s, so the rate error is NEGATIVE (-0.0436) and the
    P term pulls the steering back: 0.30 x (-0.0436) + 0.70 x 0.1309 = 0.0785."""
    out = HeadingController(PURE_PF).update(100.0, 90.0, 10.0, DT)
    assert out == pytest.approx(0.30 * (0.75 * math.radians(10.0) - math.radians(10.0))
                                + 0.70 * 0.75 * math.radians(10.0), rel=1e-9)
    assert out < HeadingController(PURE_PF).update(100.0, 90.0, 0.0, DT)


def test_the_requested_turn_rate_is_limited():
    """A huge error asks for ATC_STR_RAT_MAX (90 deg/s = 1.5708 rad/s), not more."""
    cfg = ControlConfig(str_rat=PidGains(p=0.0, ff=0.5), str_rat_max_dps=90.0)
    out = HeadingController(cfg).update(170.0, 0.0, 0.0, DT)
    assert out == pytest.approx(0.5 * math.radians(90.0))           # FF x the limited rate


def test_steering_output_is_within_full_scale():
    assert heading(180.0, 0.0) == 1.0 or heading(180.0, 0.0) == -1.0 or abs(heading(180.0, 0.0)) <= 1.0
    for desired, actual in ((0, 170), (170, 0), (359, 1), (1, 359)):
        assert -1.0 <= heading(desired, actual) <= 1.0


def test_heading_reset_clears_the_rate_integrator():
    controller = HeadingController(CFG)
    for _ in range(100):
        controller.update(120.0, 90.0, 0.0, DT)
    assert controller.rate_pid.integral != 0.0
    controller.reset()
    assert controller.rate_pid.integral == 0.0


# -- speed controller -------------------------------------------------------------------------------

def test_a_boat_already_at_speed_gets_just_the_feedforward():
    controller = SpeedController(CFG)
    assert controller.update(1.5, 1.5, DT, 0.6) == pytest.approx(0.6)      # error 0: FF x (1.5 / 1.5)


def test_the_desired_speed_is_ramped_at_the_acceleration_limit():
    """From rest to 1.5 m/s with ATC_ACCEL_MAX = 1 m/s^2 and dt = 0.05: the
    ramped target rises 0.05 m/s per call, starting from the MEASURED speed."""
    controller = SpeedController(CFG)
    assert controller.update(1.5, 0.0, DT, 0.6) == pytest.approx(0.0)             # first call: ramp starts at 0
    out = controller.update(1.5, 0.0, DT, 0.6)
    # ramped = 0.05 ; FF = 0.6 x 0.05 / 1.5 = 0.02 ; P = 0.25 x 0.05 = 0.0125 ; I = 0.10 x 0.05 x 0.05 = 0.00025
    assert out == pytest.approx(0.02 + 0.0125 + 0.00025, rel=1e-6)
    for _ in range(100):
        out = controller.update(1.5, 1.0, DT, 0.6)
    assert controller._ramped == pytest.approx(1.5)                          # the ramp has arrived


def test_throttle_is_never_negative_and_never_above_one():
    controller = SpeedController(CFG)
    # target 0 while doing 2 m/s: the ramp falls 0.05 per call, so from the second
    # call the PID error is NEGATIVE; braking must not reverse the motors
    outputs = [controller.update(0.0, 2.0, DT, 0.0) for _ in range(30)]
    assert min(outputs) >= 0.0
    assert controller.pid.last_output < 0.0                                  # the PID did ask to brake
    controller = SpeedController(CFG)
    for _ in range(300):
        out = controller.update(5.0, 0.0, DT, 5.0)
    assert out <= 1.0


def test_zero_desired_speed_has_no_feedforward_and_does_not_divide_by_zero():
    controller = SpeedController(CFG)
    controller.update(0.0, 0.0, DT, 0.0)
    assert controller.update(0.0, 0.0, DT, 0.0) == pytest.approx(0.0)


def test_an_invalid_timestep_restarts_the_ramp_at_the_measured_speed():
    controller = SpeedController(CFG)
    for _ in range(40):
        controller.update(1.5, 0.0, DT, 0.6)
    assert controller._ramped > 1.0
    controller.update(1.5, 0.2, 5.0, 0.6)                                   # a 5 s gap
    assert controller._ramped == pytest.approx(0.2)


def test_speed_reset():
    controller = SpeedController(CFG)
    for _ in range(40):
        controller.update(1.5, 0.5, DT, 0.6)
    controller.reset()
    assert controller._ramped is None and controller.pid.integral == 0.0


def test_the_speed_integrator_removes_a_steady_shortfall():
    """A boat that is 0.1 m/s slow forever drives the integral up; it stays
    inside I_MAX."""
    controller = SpeedController(CFG)
    out = 0.0
    for _ in range(2000):
        out = controller.update(1.5, 1.4, DT, 0.5)
    assert controller.pid.integral == pytest.approx(CFG.speed.i_max)
    assert out == pytest.approx(0.5 + 0.25 * 0.1 + CFG.speed.i_max)


# -- the motor mix: same cases as firmware tests/c/test_motor.c --------------------------------------

def test_mix_directions_like_the_firmware():
    assert mix_to_commands(0.5, 0.0) == pytest.approx((0.5, 0.5))           # straight
    assert mix_to_commands(0.5, 0.2) == pytest.approx((0.7, 0.3))           # right: left faster
    left, right = mix_to_commands(0.5, -0.2)
    assert right > left                                                      # left: right faster
    assert mix_to_commands(0.0, 0.6) == pytest.approx((0.6, -0.6))          # pivot right
    assert mix_to_commands(-0.5, 0.0) == pytest.approx((-0.5, -0.5))        # reverse


def test_mix_desaturation_keeps_the_turn_ratio_like_the_firmware():
    left, right = mix_to_commands(1.0, 0.5)                                  # raw 1.5 / 0.5
    assert abs(left) <= 1.0 and abs(right) <= 1.0
    assert left / right == pytest.approx(3.0) and left > right
    assert mix_to_commands(1.0, 1.0) == pytest.approx((1.0, 0.0))            # raw 2.0 / 0.0
    assert mix_to_commands(-1.0, -1.0) == pytest.approx((-1.0, 0.0))         # raw -2.0 / 0.0
    assert mix_to_commands(0.0, 0.0) == (0.0, 0.0)
