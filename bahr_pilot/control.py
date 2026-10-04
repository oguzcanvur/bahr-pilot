"""Heading and speed control (Phase 12).

Structure and parameter names follow ArduRover's, so the controls BAHR-GCS
already shows ("Direksiyon P", "Hız I", ...) mean what they say:

    steering   heading error --(ATC_STR_ANG_P)--> desired turn rate
               (limited to ATC_STR_RAT_MAX) --PID + feed-forward on the
               measured yaw rate--> steering, -1 (left) .. +1 (right)
    speed      throttle = feed-forward (CRUISE_THROTTLE at CRUISE_SPEED)
               + PID on the ground-speed error, with the desired speed
               ramped at ATC_ACCEL_MAX

Units are ArduPilot's: angles in the PID in radians, rates rad/s.

PID details that matter for a boat:
  * the derivative acts on the MEASUREMENT (no kick when the target jumps);
  * the integrator is clamped (I_MAX) and frozen while the output is
    saturated and the error would push it further (anti-windup);
  * a bad timestep (zero, backwards, or longer than MAX_DT) never produces a
    huge integral or derivative: the step is treated as a restart;
  * reset() clears everything, called whenever the boat stops being driven.

The defaults were tuned on the PLACEHOLDER simulator boat (docs/SITL.md), scored
on the WORST of three boats (nominal, heavy and weak, light and strong) rather
than on one, so they are a safe starting point; they are not the real boat's
tuning. Two findings from that search worth keeping: the steering integral must
stay small (I >= 0.04 made big turns creep for 18 s because it winds up while
the error is large), yet it is needed (with one motor 30 % weak, I = 0 left a
0.42 m mean line error against 0.29 m); and a high angle gain (ATC_STR_ANG_P
1.5) overshot 24-53 %, hence 0.75.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from bahr_pilot import geo

MAX_DT_S = 0.5


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass(frozen=True)
class PidGains:
    p: float = 0.0
    i: float = 0.0
    d: float = 0.0
    ff: float = 0.0
    i_max: float = 1.0          # |integral contribution| limit, in output units
    filter_hz: float = 10.0     # low-pass on the derivative term


class PID:
    def __init__(self, gains: PidGains, out_min: float = -1.0, out_max: float = 1.0) -> None:
        self.gains = gains
        self.out_min, self.out_max = out_min, out_max
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0                 # in output units (already multiplied by I)
        self._last_measurement: float | None = None
        self._d_filtered = 0.0
        self.last_output = 0.0

    def update(self, target: float, measurement: float, dt: float) -> float:
        g = self.gains
        if not (math.isfinite(target) and math.isfinite(measurement)):
            self.reset()
            return 0.0
        error = target - measurement
        valid_dt = 0.0 < dt <= MAX_DT_S
        derivative = 0.0
        if valid_dt and self._last_measurement is not None:
            raw = -(measurement - self._last_measurement) / dt        # derivative on the measurement
            alpha = dt / (dt + 1.0 / (2.0 * math.pi * g.filter_hz)) if g.filter_hz > 0.0 else 1.0   # 0 Hz = unfiltered
            self._d_filtered += alpha * (raw - self._d_filtered)
            derivative = self._d_filtered
        elif not valid_dt:
            self._d_filtered = 0.0
        self._last_measurement = measurement

        unsaturated = g.p * error + self.integral + g.d * derivative + g.ff * target
        if valid_dt and g.i > 0.0:
            # anti-windup: freeze while saturated and the error pushes further out
            pushing_out = (unsaturated >= self.out_max and error > 0.0) or \
                          (unsaturated <= self.out_min and error < 0.0)
            if not pushing_out:
                self.integral = _clamp(self.integral + g.i * error * dt, -g.i_max, g.i_max)
        output = _clamp(g.p * error + self.integral + g.d * derivative + g.ff * target,
                        self.out_min, self.out_max)
        self.last_output = output
        return output


@dataclass(frozen=True)
class ControlConfig:
    # steering (ArduRover ATC_STR_*)
    str_ang_p: float = 0.75                 # 1/s: desired turn rate per radian of heading error
    str_rat_max_dps: float = 90.0           # largest turn rate the autopilot may ask for
    str_rat: PidGains = PidGains(p=0.9, i=0.03, d=0.0, ff=0.3, i_max=0.2, filter_hz=10.0)
    # speed (ArduRover ATC_SPEED_*, ATC_ACCEL_MAX)
    speed: PidGains = PidGains(p=0.25, i=0.10, d=0.0, ff=0.0, i_max=0.3, filter_hz=10.0)
    accel_max_mps2: float = 1.0
    # how much the speed is cut for a large heading error (not pirouetting at full speed)
    min_cornering_factor: float = 0.35
    cornering_error_deg: float = 90.0


class HeadingController:
    def __init__(self, config: ControlConfig) -> None:
        self.cfg = config
        self.rate_pid = PID(config.str_rat)

    def reset(self) -> None:
        self.rate_pid.reset()

    def update(self, desired_heading_deg: float, heading_deg: float, yaw_rate_dps: float, dt: float) -> float:
        """Steering command, -1 .. +1, positive = turn right (clockwise)."""
        error = math.radians(geo.wrap_180(desired_heading_deg - heading_deg))
        max_rate = math.radians(self.cfg.str_rat_max_dps)
        desired_rate = _clamp(self.cfg.str_ang_p * error, -max_rate, max_rate)
        return self.rate_pid.update(desired_rate, math.radians(yaw_rate_dps), dt)


class SpeedController:
    def __init__(self, config: ControlConfig) -> None:
        self.cfg = config
        self.pid = PID(config.speed, out_min=-1.0, out_max=1.0)
        self._ramped: float | None = None

    def reset(self) -> None:
        self.pid.reset()
        self._ramped = None

    def update(self, desired_speed: float, speed: float, dt: float, feedforward: float) -> float:
        """Throttle 0 .. 1: the feed-forward for the desired speed plus a PID
        correction on the speed error. The desired speed is ramped at
        ATC_ACCEL_MAX so a step in the target does not wind the integrator up."""
        if self._ramped is None or not (0.0 < dt <= MAX_DT_S):
            self._ramped = speed if math.isfinite(speed) else 0.0
        else:
            step = self.cfg.accel_max_mps2 * dt
            self._ramped += _clamp(desired_speed - self._ramped, -step, step)
        ramped_ff = feedforward * (self._ramped / desired_speed) if desired_speed > 1e-6 else 0.0
        correction = self.pid.update(self._ramped, speed, dt)
        return _clamp(ramped_ff + correction, 0.0, 1.0)


def mix_to_commands(throttle: float, steering: float) -> tuple[float, float]:
    """Skid-steer mix to (left, right) commands in -1..1. If either side would
    exceed full scale both are scaled down together, so the turn RATIO is kept
    (the boat still turns the way it was told, just slower) - the same rule as
    firmware motor.c Motor_Mix. Steering > 0 = turn right = left faster."""
    left, right = throttle + steering, throttle - steering
    peak = max(abs(left), abs(right))
    if peak > 1.0:
        left, right = left / peak, right / peak
    return left, right
