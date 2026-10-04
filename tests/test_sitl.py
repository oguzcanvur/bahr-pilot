"""Phases 26-27: the boat model, the sensor models and the closed-loop harness.

The physics is checked against closed-form steady states (roots of the
drag-balance quadratics), not against the simulator's own output; the
sensors against their configured statistics; the harness against a case with
a known answer (a constant-thrust circle the estimator must track).

Nothing here says the simulated boat resembles the real one: the parameters
are placeholders (docs/SITL.md). What is established is that the simulator is
internally correct and has the SIGNS the control software assumes.
"""
from __future__ import annotations

import math
import statistics

import pytest

from bahr_pilot import geo
from bahr_pilot.estimator import EstimateStatus, Estimator
from bahr_pilot.gnss import GnssQuality
from bahr_pilot.sitl.boat import Boat, BoatParams, BoatState, Environment, MotorChain, pulse_to_command
from bahr_pilot.sitl.harness import ORIGIN, PHYSICS_DT, Sitl
from bahr_pilot.sitl.sensors import Fault, SensorConfig, SensorSuite

P = BoatParams()


def run_boat(pulses, seconds, *, params=None, env=None, state=None, dt=PHYSICS_DT):
    boat = Boat(params or P, env or Environment(), state or BoatState())
    for _ in range(round(seconds / dt)):
        boat.step(pulses, dt)
    return boat


def quad_root(a, b, c):
    """Positive root of a x^2 + b x - c = 0."""
    return (-b + math.sqrt(b * b + 4.0 * a * c)) / (2.0 * a)


# -- boat physics -------------------------------------------------------------------------

def test_terminal_speed_matches_the_drag_balance():
    """Both motors at command 0.6: thrust 24 N = X_u u + X_uu u^2."""
    boat = run_boat((1800, 1800), 120.0)
    expected = quad_root(P.surge_drag_quad, P.surge_drag_lin, 2 * P.max_thrust_n * 0.6)
    assert boat.state.surge == pytest.approx(expected, rel=0.005)
    assert boat.state.sway == pytest.approx(0.0, abs=1e-9)
    assert boat.state.yaw_rate == pytest.approx(0.0, abs=1e-9)
    assert boat.state.heading_deg == pytest.approx(0.0, abs=1e-9)   # straight north, no drift


def test_default_throttle_gives_roughly_the_default_cruise_speed():
    """The placeholders were picked so CRUISE_THROTTLE 60 % ~ CRUISE_SPEED 1.5 m/s."""
    assert run_boat((1800, 1800), 120.0).state.surge == pytest.approx(1.5, abs=0.15)


def test_pure_yaw_rate_matches_the_torque_balance():
    """Left ahead at 0.3, right astern at -0.5 gives T_L = -T_R = 6 N: no net
    thrust, so u = 0 exactly and  a (T_L - T_R) = N_r r + N_rr r^2."""
    boat = run_boat((1650, 1250), 60.0)
    torque = P.lever_arm_m * 12.0
    expected = quad_root(P.yaw_drag_quad, P.yaw_drag_lin, torque)
    assert boat.state.yaw_rate == pytest.approx(expected, rel=0.005)
    assert boat.state.surge == pytest.approx(0.0, abs=1e-6)
    assert expected > 0                                     # clockwise


def test_left_motor_faster_turns_the_boat_to_the_right():
    """The sign convention navigation.py and motor.h are built on."""
    boat = run_boat((1600, 1400), 5.0)
    assert boat.state.yaw_rate > 0
    assert 0.0 < boat.state.heading_deg < 180.0


def test_swapped_motor_wiring_turns_the_boat_the_other_way():
    swapped = run_boat((1600, 1400), 5.0, params=BoatParams(swapped_motors=True))
    assert swapped.state.yaw_rate < 0


def test_a_turning_boat_slips_sideways_outward():
    """Centrifugal sway: -m u r pushes the boat to the OUTSIDE of the turn
    (port side for a clockwise turn, so v < 0)."""
    boat = run_boat((1700, 1500), 30.0)
    assert boat.state.yaw_rate > 0 and boat.state.surge > 0
    assert boat.state.sway < 0


def test_ground_velocity_axes():
    """heading 0 = north; sway is towards STARBOARD (east when facing north)."""
    def velocity(heading, surge=0.0, sway=0.0):
        return Boat(P, Environment(), BoatState(heading_deg=heading, surge=surge, sway=sway)).ground_velocity()
    assert velocity(0, surge=1) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert velocity(90, surge=1) == pytest.approx((1.0, 0.0), abs=1e-12)
    assert velocity(180, surge=1) == pytest.approx((0.0, -1.0), abs=1e-12)
    assert velocity(0, sway=1) == pytest.approx((1.0, 0.0), abs=1e-12)
    assert velocity(90, sway=1) == pytest.approx((0.0, -1.0), abs=1e-12)


def test_position_follows_the_heading():
    boat = Boat(P, Environment(), BoatState(heading_deg=90.0, surge=1.5))
    boat.motors.slewed = boat.motors.response = [0.0, 0.0]
    for _ in range(100):
        boat.step((1500, 1500), PHYSICS_DT)          # coasting, drag only
    assert boat.state.east > 0.5 and abs(boat.state.north) < 1e-6


def test_without_thrust_the_boat_only_slows_down():
    boat = Boat(P, Environment(), BoatState(surge=2.0))
    previous = 2.0
    for _ in range(3000):
        boat.step((1500, 1500), PHYSICS_DT)
        assert 0.0 <= boat.state.surge <= previous + 1e-12
        previous = boat.state.surge


def test_a_boat_at_rest_drifts_with_the_current():
    boat = run_boat((1500, 1500), 30.0, env=Environment(current_east_mps=0.5))
    assert boat.state.east == pytest.approx(15.0, abs=0.01)
    assert boat.ground_velocity() == pytest.approx((0.5, 0.0), abs=1e-9)
    assert boat.state.surge == pytest.approx(0.0, abs=1e-9)        # still, relative to the water


def test_wind_pushes_the_hull_until_sway_drag_balances_it():
    """Heading north, a 10 N wind from the west is a pure sway force:
    Y_v v + Y_vv v^2 = 10 N."""
    boat = run_boat((1500, 1500), 60.0, env=Environment(wind_force_east_n=10.0))
    expected = quad_root(P.sway_drag_quad, P.sway_drag_lin, 10.0)
    assert boat.state.sway == pytest.approx(expected, rel=0.005)
    assert boat.ground_velocity()[0] == pytest.approx(expected, rel=0.005)
    assert boat.state.surge == pytest.approx(0.0, abs=1e-9)


def test_integration_is_converged():
    """Halving the step changes the answer by far less than a sensor's noise."""
    coarse = run_boat((1700, 1500), 20.0, dt=0.01)
    fine = run_boat((1700, 1500), 20.0, dt=0.005)
    assert coarse.state.east == pytest.approx(fine.state.east, abs=1e-3)
    assert coarse.state.north == pytest.approx(fine.state.north, abs=1e-3)
    assert geo.wrap_180(coarse.state.heading_deg - fine.state.heading_deg) == pytest.approx(0.0, abs=1e-3)


def test_integration_is_second_order():
    """Each halving of the step should cut the change to about a quarter. (The
    first version drove the motor lag with the END of the slew ramp and was only
    first-order: a factor of two per halving.)"""
    a, b, c = (run_boat((1700, 1500), 20.0, dt=dt).state.north for dt in (0.02, 0.01, 0.005))
    assert abs(b - a) / abs(c - b) > 3.0


def test_heading_stays_in_range_and_continuous_through_north():
    boat = Boat(P, Environment(), BoatState(heading_deg=355.0))
    seen = []
    for _ in range(600):
        boat.step((1650, 1250), PHYSICS_DT)
        seen.append(boat.state.heading_deg)
    assert all(0.0 <= h < 360.0 for h in seen)
    assert max(abs(geo.wrap_180(b - a)) for a, b in zip(seen, seen[1:])) < 1.0


# -- motor chain (mirrors firmware motor.c) --------------------------------------------

def test_pulse_to_command_and_deadband():
    assert pulse_to_command(1500) == 0.0
    assert pulse_to_command(2000) == 1.0 and pulse_to_command(1000) == -1.0
    assert pulse_to_command(1510, deadband_us=20) == 0.0
    assert pulse_to_command(1600, deadband_us=20) == pytest.approx(0.2)
    assert pulse_to_command(5000) == 1.0                    # clamped, never beyond full scale


def test_slew_limit_takes_a_second_for_full_scale():
    chain = MotorChain(P)
    for k in range(50):                                     # 0.5 s
        chain.step((2000, 2000), PHYSICS_DT * 1.0)
    # 50 steps of 10 ms at 1.0/s = 0.5
    assert chain.slewed[0] == pytest.approx(0.5, abs=1e-9)
    for k in range(50):
        chain.step((2000, 2000), PHYSICS_DT)
    assert chain.slewed[0] == pytest.approx(1.0, abs=1e-9)


def test_full_astern_to_full_ahead_takes_two_seconds():
    chain = MotorChain(P)
    chain.slewed = [-1.0, -1.0]
    for _ in range(150):
        chain.step((2000, 2000), PHYSICS_DT)
    assert chain.slewed[0] == pytest.approx(0.5, abs=1e-9)
    for _ in range(50):
        chain.step((2000, 2000), PHYSICS_DT)
    assert chain.slewed[0] == pytest.approx(1.0, abs=1e-9)


def test_halt_is_immediate_not_rate_limited():
    chain = MotorChain(P)
    for _ in range(300):
        chain.step((2000, 2000), PHYSICS_DT)
    chain.halt()
    assert chain.slewed == [0.0, 0.0] and chain.response == [0.0, 0.0]


def test_astern_thrust_is_weaker_than_ahead():
    ahead, astern = MotorChain(P), MotorChain(P)
    for _ in range(1000):
        t_ahead = ahead.step((2000, 2000), PHYSICS_DT)
        t_astern = astern.step((1000, 1000), PHYSICS_DT)
    assert t_ahead[0] == pytest.approx(P.max_thrust_n, rel=1e-3)
    assert t_astern[0] == pytest.approx(-P.reverse_factor * P.max_thrust_n, rel=1e-3)


def test_motor_lag_is_first_order():
    chain = MotorChain(BoatParams(slew_per_s=1000.0))        # slew out of the way
    thrust = None
    for _ in range(round(P.motor_tau_s / PHYSICS_DT)):       # one time constant
        thrust = chain.step((2000, 2000), PHYSICS_DT)
    assert chain.response[0] == pytest.approx(1.0 - math.exp(-1.0), abs=0.01)
    assert thrust[0] == pytest.approx(P.max_thrust_n * (1.0 - math.exp(-1.0)), abs=0.2)


# -- sensors ---------------------------------------------------------------------------------

FRAME = geo.LocalFrame(*ORIGIN)


def collect(cfg, seconds, *, track=lambda t: (0.0, 0.0, 0.0, 0.0)):
    """Feed the sensor suite a prescribed truth (east, north, heading, yaw rate)."""
    suite = SensorSuite(cfg, FRAME)
    imu, fixes, headings = [], [], []
    for k in range(round(seconds / PHYSICS_DT)):
        t = (k + 1) * PHYSICS_DT
        out = suite.update(t, *track(t))
        imu.extend(out.imu)
        if out.fix:
            fixes.append(out.fix)
        if out.heading:
            headings.append(out.heading)
    return imu, fixes, headings


def local(fix):
    return FRAME.to_enu(fix.lat, fix.lon)


def test_sensor_rates():
    imu, fixes, headings = collect(SensorConfig(), 10.0)
    assert len(imu) == 500 and len(fixes) == 50 and len(headings) == 10


def test_gnss_noise_matches_its_configured_sigma():
    _, fixes, _ = collect(SensorConfig(gnss_sigma_m=0.05), 400.0)
    east = [local(f)[0] for f in fixes]
    north = [local(f)[1] for f in fixes]
    assert statistics.fmean(east) == pytest.approx(0.0, abs=0.005)
    assert statistics.pstdev(east) == pytest.approx(0.05, rel=0.08)
    assert statistics.pstdev(north) == pytest.approx(0.05, rel=0.08)
    assert fixes[0].h_acc_m == 0.05 and fixes[0].quality == GnssQuality.RTK_FIXED


def test_gnss_fix_is_late_by_its_latency():
    """A boat moving east at 2 m/s: a fix delivered at t describes t - latency."""
    _, fixes, _ = collect(SensorConfig(gnss_sigma_m=0.0001, gnss_latency_s=0.1), 20.0,
                          track=lambda t: (2.0 * t, 0.0, 90.0, 0.0))
    late = [local(f)[0] - 2.0 * f.t for f in fixes if f.t > 1.0]
    assert statistics.fmean(late) == pytest.approx(-0.2, abs=0.002)


def test_heading_noise_latency_and_wrap_through_north():
    cfg = SensorConfig(heading_sigma_deg=0.0001, heading_latency_s=0.1)
    # turning at 10 deg/s through north: truth heading = 340 + 10 t
    _, _, headings = collect(cfg, 20.0, track=lambda t: (0.0, 0.0, (340.0 + 10.0 * t) % 360.0,
                                                         math.radians(10.0)))
    for h in headings:
        if h.t > 1.0:
            truth_then = 340.0 + 10.0 * (h.t - 0.1)
            assert geo.wrap_180(h.heading_deg - truth_then) == pytest.approx(0.0, abs=0.02)


def test_gyro_reports_the_yaw_rate_plus_bias_plus_noise():
    imu, _, _ = collect(SensorConfig(gyro_bias_dps=0.5, gyro_bias_walk_dps_per_rts=0.0), 200.0,
                        track=lambda t: (0.0, 0.0, 0.0, math.radians(7.0)))
    rates = [math.degrees(s.gyro[2]) for s in imu]
    assert statistics.fmean(rates) == pytest.approx(7.5, abs=0.05)
    assert statistics.pstdev(rates) == pytest.approx(0.1 / math.sqrt(0.02), rel=0.05)
    assert all(s.gyro_valid and s.quat_valid for s in imu)
    assert imu[10].rx_time == pytest.approx(imu[10].t_pi + 0.003)


def test_same_seed_same_data_different_seed_different_data():
    a = collect(SensorConfig(seed=5), 3.0)
    b = collect(SensorConfig(seed=5), 3.0)
    c = collect(SensorConfig(seed=6), 3.0)
    assert [f.lat for f in a[1]] == [f.lat for f in b[1]]
    assert [f.lat for f in a[1]] != [f.lat for f in c[1]]


def test_unknown_fault_names_are_rejected():
    with pytest.raises(ValueError):
        Fault("gnss_outtage", 0.0)


def test_gnss_outage_window():
    cfg = SensorConfig(faults=(Fault("gnss_outage", 5.0, 8.0),))
    _, fixes, _ = collect(cfg, 12.0)
    assert not [f for f in fixes if 5.0 <= f.t < 8.0]
    assert [f for f in fixes if f.t < 5.0] and [f for f in fixes if f.t >= 8.0]


def test_heading_and_imu_outages():
    cfg = SensorConfig(faults=(Fault("heading_outage", 3.0, 6.0), Fault("imu_outage", 4.0, 5.0)))
    imu, _, headings = collect(cfg, 8.0)
    assert not [h for h in headings if 3.0 <= h.t < 6.0]
    assert not [s for s in imu if 4.0 <= s.t_pi < 5.0]


def test_gnss_jump_displaces_fixes():
    cfg = SensorConfig(gnss_sigma_m=0.0001, faults=(Fault("gnss_jump", 2.0, 4.0, (30.0, -10.0)),))
    _, fixes, _ = collect(cfg, 6.0)
    jumped = [local(f) for f in fixes if 2.0 <= f.t < 4.0]
    normal = [local(f) for f in fixes if f.t >= 4.5]
    assert jumped[0] == pytest.approx((30.0, -10.0), abs=0.01)
    assert normal[0] == pytest.approx((0.0, 0.0), abs=0.01)


def test_heading_offset_gyro_invalid_degraded_and_bias_step():
    cfg = SensorConfig(heading_sigma_deg=0.0001, gnss_sigma_m=0.0001, gyro_bias_dps=0.0,
                       gyro_noise_dps_per_rthz=0.0, gyro_bias_walk_dps_per_rts=0.0,
                       faults=(Fault("heading_offset", 2.0, 4.0, 15.0), Fault("gyro_invalid", 2.0, 4.0),
                               Fault("gnss_degraded", 2.0, 4.0), Fault("gyro_bias_step", 5.0, 7.0, 2.0)))
    imu, fixes, headings = collect(cfg, 8.0)
    inside = [h for h in headings if 2.0 <= h.t < 4.0]
    assert inside and inside[0].heading_deg == pytest.approx(15.0, abs=0.1)
    assert headings[0].heading_deg == pytest.approx(0.0, abs=0.1) or headings[0].heading_deg > 359.9
    assert [s for s in imu if 2.0 <= s.t_pi < 4.0 and s.gyro_valid] == []
    assert all(s.gyro_valid for s in imu if s.t_pi < 2.0 or s.t_pi >= 4.0)
    degraded = [f for f in fixes if 2.0 <= f.t < 4.0]
    assert degraded[0].quality == GnssQuality.FIX_3D and degraded[0].h_acc_m == 2.5
    stepped = [math.degrees(s.gyro[2]) for s in imu if 5.0 <= s.t_pi < 7.0]
    assert statistics.fmean(stepped) == pytest.approx(2.0, abs=0.01)


def test_stuck_gyro_freezes_the_reading():
    cfg = SensorConfig(gyro_noise_dps_per_rthz=0.0, gyro_bias_dps=0.0, gyro_bias_walk_dps_per_rts=0.0,
                       faults=(Fault("gyro_stuck", 3.0, 6.0),))
    imu, _, _ = collect(cfg, 8.0, track=lambda t: (0.0, 0.0, 0.0, 0.1 * t))
    stuck = {s.gyro[2] for s in imu if 3.0 <= s.t_pi < 6.0}
    assert len(stuck) == 1
    assert len({s.gyro[2] for s in imu if s.t_pi >= 6.5}) > 1       # moving again afterwards


# -- the closed loop ----------------------------------------------------------------------------

def circle_controller(pose, now):
    return 1650, 1450


def est_errors(trace, sitl, after_s):
    start = trace.index_at(after_s)
    pos, head = [], []
    for i in range(start, len(trace)):
        if trace.est_east[i] is not None:
            pos.append(math.hypot(trace.est_east[i] - trace.east[i], trace.est_north[i] - trace.north[i]))
        if trace.est_heading[i] is not None:
            head.append(abs(geo.wrap_180(trace.est_heading[i] - trace.heading[i])))
    return pos, head


def test_the_estimator_tracks_a_boat_driving_in_a_circle():
    sitl = Sitl(start_heading_deg=30.0)
    trace = sitl.run(circle_controller, 120.0)
    pos, head = est_errors(trace, sitl, 20.0)
    assert len(trace) == 120 * 20
    assert statistics.fmean(trace.speed[400:]) > 0.5          # it really is moving
    assert statistics.fmean(pos) < 0.12 and max(pos) < 0.4
    assert statistics.fmean(head) < 1.0 and max(head) < 3.0


def test_the_estimator_tolerates_twice_the_assumed_gnss_latency():
    """The estimator assumes 0.1 s; the simulated receivers are 0.2 s late."""
    sensors = SensorConfig(gnss_latency_s=0.2, heading_latency_s=0.2)
    trace_sitl = Sitl(sensors=sensors, start_heading_deg=30.0)
    trace = trace_sitl.run(circle_controller, 120.0)
    pos, head = est_errors(trace, trace_sitl, 20.0)
    assert statistics.fmean(pos) < 0.25
    assert statistics.fmean(head) < 2.0


def test_disarming_halts_the_boat_motors_immediately():
    sitl = Sitl()
    sitl.run(lambda pose, now: (1800, 1800), 20.0)
    assert sitl.boat.motors.slewed[0] > 0.5
    sitl.armed = False
    sitl.run(lambda pose, now: (1800, 1800), 0.1)
    assert sitl.boat.motors.slewed == [0.0, 0.0]


def test_the_harness_passes_batched_imu_samples_at_the_control_rate():
    seen = []

    class Spy(Estimator):
        def update(self, now, imu_samples=(), *args, **kwargs):
            samples = list(imu_samples)
            seen.append(len(samples))
            return super().update(now, samples, *args, **kwargs)

    Sitl(estimator=Spy()).run(lambda pose, now: (1500, 1500), 10.0)
    assert len(seen) == 200
    assert sum(seen) in range(498, 501)                     # 50 Hz IMU, nothing lost or duplicated
    assert set(seen[5:]) <= {2, 3}                          # 50 Hz / 20 Hz = 2.5 per tick


def test_trace_records_what_the_boat_did_and_what_the_estimator_believed():
    sitl = Sitl()
    trace = sitl.run(circle_controller, 30.0, probes={"mode": lambda: "AUTO"})
    assert len(trace.t) == len(trace.east) == len(trace.est_east) == len(trace.m1) == 600
    assert trace.probes["mode"][0] == "AUTO"
    assert trace.t[-1] == pytest.approx(30.0, abs=0.01)
    assert trace.status[-1] == EstimateStatus.OK and trace.m1[-1] == 1650 and trace.m2[-1] == 1450
    assert trace.est_east[0] is None                          # nothing is known at the first tick
