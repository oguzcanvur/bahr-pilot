"""Waypoint following: turns the active target (mission waypoint, guided
goto, or RTL-to-home) plus the estimated pose into differential-drive motor
commands.

The boat follows the LEG from where the previous waypoint was (or from where
it was when the leg began) to the active target, with pure pursuit
(bahr_pilot/pathfollow.py), instead of steering at the target. The leg lives
in a local tangent plane anchored at its start; all geometry (distance,
bearing, angle wrapping) comes from bahr_pilot/geo.py, which works on the WGS84
ellipsoid.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from bahr_pilot import geo
from bahr_pilot.control import (
    ControlConfig, HeadingController, PidGains, SpeedController, mix_to_commands,
)
from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_RTL
from bahr_pilot.nucleo_link import PULSE_MAX_US, PULSE_MIN_US, PULSE_NEUTRAL_US
from bahr_pilot.pathfollow import (
    Guidance, PathConfig, PathFollower, Point, PurePursuit, lookahead_distance,
)

PULSE_SPAN_US = 500  # pulse swing from neutral at a full-scale (+-1) motor command


def _clamp_pulse(pulse_us: float) -> int:
    # round, not int(): truncating turns float noise (1749.9999999) into a
    # 1 us left/right asymmetry and biases every pulse low by half a us
    return max(PULSE_MIN_US, min(PULSE_MAX_US, round(pulse_us)))


def _sane(value: float, default: float, low: float, high: float) -> float:
    """`value` limited to [low, high]; the default if it is not a finite number."""
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return default
    return max(low, min(high, float(value)))


def commands_to_pulses(left: float, right: float) -> tuple[int, int]:
    """Motor 1 = LEFT, motor 2 = RIGHT (firmware/reflex/Core/Inc/motor.h);
    commands -1..1, pulse = 1500 us + 500 us x command."""
    return (_clamp_pulse(PULSE_NEUTRAL_US + PULSE_SPAN_US * left),
            _clamp_pulse(PULSE_NEUTRAL_US + PULSE_SPAN_US * right))


@dataclass
class _Leg:
    key: tuple
    frame: geo.LocalFrame
    start: Point
    end: Point


class Navigator:
    """Holds the uploaded mission (slot 0 = home, matching
    sim/fake_vehicle.py's convention) and the guided-mode target, and
    produces motor commands for whichever mode is active."""

    def __init__(self, follower: PathFollower | None = None, path_config: PathConfig | None = None) -> None:
        self.mission: list[tuple[float, float]] = []
        self.mission_seq = 0
        self.mission_paused = False
        self.guided_target: tuple[float, float] | None = None

        self.path_config = path_config or PathConfig()
        self.follower: PathFollower = follower or PurePursuit(self.path_config)
        # ArduRover's L1 parameters (NAVL1_PERIOD / NAVL1_DAMPING); the vehicle
        # copies the live parameter values here before each step.
        self.l1_period_s = 10.0
        self.l1_damping = 0.75

        self.control_config = ControlConfig()
        self.heading_control = HeadingController(self.control_config)
        self.speed_control = SpeedController(self.control_config)
        self._last_time: float | None = None

        self._leg: _Leg | None = None
        # What the follower decided on the latest step, for telemetry
        # (NAV_CONTROLLER_OUTPUT); None whenever the boat is not following a leg.
        self.last_guidance: Guidance | None = None

    def configure(self, params) -> None:
        """Take the tuning from the vehicle's parameter table (ArduRover's
        names). The controllers are rebuilt - and so reset - only when a value
        actually changed. Values come from the ground station (or a corrupt
        message), so each is forced into a range the controllers can survive:
        a NaN or out-of-range value falls back to the default / the nearest
        limit instead of reaching the PID (a filter frequency of 0 would
        divide by zero)."""
        d = ControlConfig()
        self.l1_period_s = _sane(params["NAVL1_PERIOD"], 10.0, 1.0, 60.0)
        self.l1_damping = _sane(params["NAVL1_DAMPING"], 0.75, 0.1, 2.0)
        config = ControlConfig(
            str_ang_p=_sane(params["ATC_STR_ANG_P"], d.str_ang_p, 0.0, 10.0),
            str_rat_max_dps=_sane(params["ATC_STR_RAT_MAX"], d.str_rat_max_dps, 5.0, 360.0),
            str_rat=PidGains(
                p=_sane(params["ATC_STR_RAT_P"], d.str_rat.p, 0.0, 3.0),
                i=_sane(params["ATC_STR_RAT_I"], d.str_rat.i, 0.0, 3.0),
                d=_sane(params["ATC_STR_RAT_D"], d.str_rat.d, 0.0, 1.0),
                ff=_sane(params["ATC_STR_RAT_FF"], d.str_rat.ff, 0.0, 3.0),
                i_max=d.str_rat.i_max,
                filter_hz=_sane(params["ATC_STR_RAT_FILT"], d.str_rat.filter_hz, 0.5, 50.0)),
            speed=PidGains(
                p=_sane(params["ATC_SPEED_P"], d.speed.p, 0.0, 3.0),
                i=_sane(params["ATC_SPEED_I"], d.speed.i, 0.0, 3.0),
                d=_sane(params["ATC_SPEED_D"], d.speed.d, 0.0, 1.0),
                i_max=d.speed.i_max, filter_hz=d.speed.filter_hz),
            accel_max_mps2=_sane(params["ATC_ACCEL_MAX"], d.accel_max_mps2, 0.1, 5.0),
        )
        if config != self.control_config:
            self.control_config = config
            self.heading_control = HeadingController(config)
            self.speed_control = SpeedController(config)

    def _stop_controllers(self) -> None:
        """Called whenever the boat stops being driven, so no integral or ramp
        carries over into the next time it is."""
        self.heading_control.reset()
        self.speed_control.reset()
        self._last_time = None

    def active_target(self, mode: int, home: tuple[float, float] | None):
        if mode == MODE_GUIDED:
            return self.guided_target
        if mode == MODE_RTL:
            return home
        if mode == MODE_AUTO and not self.mission_paused:
            if 1 <= self.mission_seq < len(self.mission):
                return self.mission[self.mission_seq]
        return None

    def _leg_to(self, mode: int, target: tuple[float, float], pose) -> _Leg:
        """The leg towards `target`, created when the target (or the mission
        item) changes. From the second mission item on it starts at the
        previous waypoint, so the boat follows the PLANNED line whatever it
        did at the corner; otherwise (first item, GUIDED, RTL) it starts where
        the boat is when the leg begins."""
        key = (mode, target, self.mission_seq if mode == MODE_AUTO else None)
        if self._leg is not None and self._leg.key == key:
            return self._leg
        origin = (pose.lat, pose.lon)
        if mode == MODE_AUTO and 2 <= self.mission_seq <= len(self.mission) - 1:
            origin = self.mission[self.mission_seq - 1]
        frame = geo.LocalFrame(*origin)
        self._leg = _Leg(key, frame, (0.0, 0.0), frame.to_enu(*target))
        return self._leg

    def step(self, state, cruise_frac: float, wp_radius_m: float,
             target_speed_mps: float | None = None) -> tuple[int, int, bool]:
        """Returns (motor1_us, motor2_us, reached_current_target). Works from
        state.pose (the estimator's output). A dead-reckoned position still
        counts - the estimator withdraws it after its own time limit - but
        raw receiver values are never used here.

        `cruise_frac` is the open-loop throttle for the wanted speed (the
        CRUISE_THROTTLE feed-forward); with `target_speed_mps` the speed
        controller adds a PID correction on the ground speed, without it the
        throttle is the feed-forward alone."""
        self.last_guidance = None
        pose = state.pose
        if pose is None or not pose.position_valid:
            self._stop_controllers()
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False

        home = (state.home_lat, state.home_lon) if state.home_lat is not None else None
        target = self.active_target(state.mode, home)
        if target is None:
            self._leg = None
            self._stop_controllers()
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False

        leg = self._leg_to(state.mode, target, pose)
        position = leg.frame.to_enu(pose.lat, pose.lon)
        lookahead = lookahead_distance(pose.speed_mps, self.l1_period_s, self.l1_damping, self.path_config)
        guidance = self.follower.guide(position, leg.start, leg.end, pose.speed_mps, wp_radius_m, lookahead)
        self.last_guidance = guidance
        if guidance.reached:
            self._stop_controllers()
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, True
        if not pose.heading_valid:
            # Steering needs a heading. Driving on with the last one would
            # point the boat wherever it happened to face when the heading
            # was lost, so hold still until a real one is back. (Arriving
            # needs no heading, hence the check above comes first.)
            self._stop_controllers()
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US, False

        dt = 0.0 if self._last_time is None else pose.t - self._last_time
        self._last_time = pose.t
        cfg = self.control_config
        steering = self.heading_control.update(guidance.desired_heading_deg, pose.heading_deg,
                                               pose.yaw_rate_dps, dt)
        # Slow down for sharp corrections instead of pirouetting at full speed.
        error = abs(geo.wrap_180(guidance.desired_heading_deg - pose.heading_deg))
        cornering = max(cfg.min_cornering_factor, 1.0 - error / cfg.cornering_error_deg)
        feedforward = cruise_frac * cornering
        if target_speed_mps is None:
            throttle = feedforward
        else:
            throttle = self.speed_control.update(target_speed_mps * cornering, pose.speed_mps, dt, feedforward)
        left, right = mix_to_commands(throttle, steering)
        m1, m2 = commands_to_pulses(left, right)
        return m1, m2, False
