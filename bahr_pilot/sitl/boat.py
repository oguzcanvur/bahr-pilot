"""Differential-thrust boat dynamics for software-in-the-loop testing (Phase 26).

A 3-degree-of-freedom surface-vessel model: surge u (forward), sway v
(towards starboard), yaw rate r (clockwise positive, so heading increases).
Velocities are relative to the WATER; a constant current is added to get the
motion over the ground, and drag acts on the water-relative velocity.

    m (du/dt - v r) = T_L + T_R - X_u u - X_uu |u| u + F_surge
    m (dv/dt + u r) =            - Y_v v - Y_vv |v| v + F_sway
    I (dr/dt)       = a (T_L - T_R) - N_r r - N_rr |r| r

with a = the lever arm of each motor. T_L > T_R turns the boat to the RIGHT,
matching bahr_pilot/navigation.py ("target to the right -> left motor
faster") and firmware motor.h.

THIS IS NOT THE REAL BOAT. Every default below is a placeholder chosen so a
60 % throttle pair gives roughly the vehicle's default CRUISE_SPEED of
1.5 m/s; nothing is measured. The simulator exists to test the control
software's LOGIC (signs, limits, failsafe behaviour, how the estimator and
the navigator interact), not to tune gains: gains need data from the real
hull. See docs/SITL.md.

Motor chain, mirroring the firmware: pulse -> ESC dead band -> slew limit (the
STM's MOT_SLEWRATE, 100 %/s by default; full astern to full ahead takes 2 s)
-> first-order lag of ESC + propeller -> thrust (reverse is weaker).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

PULSE_NEUTRAL_US = 1500
PULSE_SPAN_US = 500


@dataclass(frozen=True)
class BoatParams:
    mass_kg: float = 25.0                 # including added mass
    inertia_kgm2: float = 6.0
    lever_arm_m: float = 0.25             # each motor's distance from the centreline
    max_thrust_n: float = 20.0            # per motor, ahead
    reverse_factor: float = 0.6           # astern thrust relative to ahead
    surge_drag_lin: float = 5.0           # N per m/s
    surge_drag_quad: float = 6.4          # N per (m/s)^2
    sway_drag_lin: float = 40.0
    sway_drag_quad: float = 60.0
    yaw_drag_lin: float = 4.0             # N m per rad/s
    yaw_drag_quad: float = 6.0
    esc_deadband_us: int = 20
    motor_tau_s: float = 0.3              # ESC + propeller response
    slew_per_s: float = 1.0               # firmware MOT_SLEWRATE 100 % = 1.0 command units per second
    swapped_motors: bool = False          # wiring fault: left and right ESC leads exchanged
    motor_gain: tuple[float, float] = (1.0, 1.0)   # (left, right) thrust scale: unequal motors/propellers


@dataclass
class Environment:
    current_east_mps: float = 0.0         # water moves over the ground at this velocity
    current_north_mps: float = 0.0
    wind_force_east_n: float = 0.0        # constant force on the hull
    wind_force_north_n: float = 0.0


@dataclass
class BoatState:
    east: float = 0.0                     # metres, local frame
    north: float = 0.0
    heading_deg: float = 0.0              # compass, clockwise from north, [0, 360)
    surge: float = 0.0                    # m/s, forward, relative to water
    sway: float = 0.0                     # m/s, towards starboard, relative to water
    yaw_rate: float = 0.0                 # rad/s, clockwise positive


def pulse_to_command(pulse_us: float, deadband_us: float = 0.0) -> float:
    """Pulse width -> command in [-1, 1]; zero inside the ESC dead band."""
    offset = float(pulse_us) - PULSE_NEUTRAL_US
    if abs(offset) <= deadband_us:
        return 0.0
    return max(-1.0, min(1.0, offset / PULSE_SPAN_US))


class MotorChain:
    """Slew limiter and lag for the two motors. Commands are -1..1."""

    def __init__(self, params: BoatParams) -> None:
        self.p = params
        self.slewed = [0.0, 0.0]      # what the STM would be outputting
        self.response = [0.0, 0.0]    # what the propeller is actually doing

    def halt(self) -> None:
        """Immediate stop, as the STM does on any failsafe (never rate-limited)."""
        self.slewed = [0.0, 0.0]
        self.response = [0.0, 0.0]

    def step(self, pulses_us: tuple[float, float], dt: float) -> tuple[float, float]:
        """Advance dt seconds towards the commanded pulses; return the two
        thrusts in newtons (left, right)."""
        left_pulse, right_pulse = pulses_us
        if self.p.swapped_motors:
            left_pulse, right_pulse = right_pulse, left_pulse
        targets = (pulse_to_command(left_pulse, self.p.esc_deadband_us),
                   pulse_to_command(right_pulse, self.p.esc_deadband_us))
        max_step = self.p.slew_per_s * dt
        alpha = 1.0 - math.exp(-dt / self.p.motor_tau_s)
        thrusts = []
        for i in (0, 1):
            before = self.slewed[i]
            delta = max(-max_step, min(max_step, targets[i] - before))
            self.slewed[i] = max(-1.0, min(1.0, before + delta))
            # the lag is driven by the MEAN of the ramp over the step; driving
            # it with the end value alone made the whole simulation first-order
            self.response[i] += alpha * (0.5 * (before + self.slewed[i]) - self.response[i])
            c = self.response[i]
            thrusts.append(self.p.motor_gain[i] * self.p.max_thrust_n * (c if c >= 0.0 else self.p.reverse_factor * c))
        return thrusts[0], thrusts[1]


def _derivatives(s: tuple[float, ...], t_left: float, t_right: float, p: BoatParams,
                 env: Environment) -> tuple[float, ...]:
    east, north, psi, u, v, r = s
    sin_psi, cos_psi = math.sin(psi), math.cos(psi)
    force_surge = env.wind_force_east_n * sin_psi + env.wind_force_north_n * cos_psi
    force_sway = env.wind_force_east_n * cos_psi - env.wind_force_north_n * sin_psi
    du = ((t_left + t_right - p.surge_drag_lin * u - p.surge_drag_quad * abs(u) * u + force_surge) / p.mass_kg
          + v * r)
    dv = ((-p.sway_drag_lin * v - p.sway_drag_quad * abs(v) * v + force_sway) / p.mass_kg - u * r)
    dr = (p.lever_arm_m * (t_left - t_right) - p.yaw_drag_lin * r - p.yaw_drag_quad * abs(r) * r) / p.inertia_kgm2
    d_east = u * sin_psi + v * cos_psi + env.current_east_mps
    d_north = u * cos_psi - v * sin_psi + env.current_north_mps
    return d_east, d_north, r, du, dv, dr


@dataclass
class Boat:
    params: BoatParams = field(default_factory=BoatParams)
    env: Environment = field(default_factory=Environment)
    state: BoatState = field(default_factory=BoatState)

    def __post_init__(self) -> None:
        self.motors = MotorChain(self.params)
        self.time = 0.0
        self.last_thrust = (0.0, 0.0)

    def step(self, pulses_us: tuple[float, float], dt: float) -> None:
        """Advance by dt (use <= 0.02 s) with RK4 under the given pulses."""
        end_left, end_right = self.motors.step(pulses_us, dt)
        # Thrust is held constant over the step; the mean of its values at the
        # start and the end keeps the integration second-order (the end value
        # alone made it first-order: 3.5 mm out after 20 s at dt = 10 ms).
        t_left = 0.5 * (self.last_thrust[0] + end_left)
        t_right = 0.5 * (self.last_thrust[1] + end_right)
        self.last_thrust = (end_left, end_right)
        st = self.state
        y = (st.east, st.north, math.radians(st.heading_deg), st.surge, st.sway, st.yaw_rate)

        def f(state):
            return _derivatives(state, t_left, t_right, self.params, self.env)

        k1 = f(y)
        k2 = f(tuple(a + 0.5 * dt * b for a, b in zip(y, k1)))
        k3 = f(tuple(a + 0.5 * dt * b for a, b in zip(y, k2)))
        k4 = f(tuple(a + dt * b for a, b in zip(y, k3)))
        y = tuple(a + dt / 6.0 * (b + 2.0 * c + 2.0 * d + e) for a, b, c, d, e in zip(y, k1, k2, k3, k4))
        st.east, st.north = y[0], y[1]
        st.heading_deg = math.degrees(y[2]) % 360.0
        st.surge, st.sway, st.yaw_rate = y[3], y[4], y[5]
        self.time += dt

    # -- derived quantities the sensors need ------------------------------------------

    def ground_velocity(self) -> tuple[float, float]:
        """(east, north) velocity over the ground, m/s."""
        psi = math.radians(self.state.heading_deg)
        s = self.state
        return (s.surge * math.sin(psi) + s.sway * math.cos(psi) + self.env.current_east_mps,
                s.surge * math.cos(psi) - s.sway * math.sin(psi) + self.env.current_north_mps)

    def halt_motors(self) -> None:
        self.motors.halt()
