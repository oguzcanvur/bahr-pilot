"""Closed-loop software-in-the-loop harness (Phases 26-27).

    boat truth -> SensorSuite -> Estimator -> controller -> motor pulses -> boat

The boat is integrated at 100 Hz; the controller and the estimator run at the
vehicle's own loop rate (TICK_HZ = 20 Hz), with the IMU samples that arrived
since the last tick handed to the estimator in a batch, exactly as
Vehicle._update_estimate does. Time is simulated: the harness passes its own
clock to everything that takes one, so a 5-minute mission runs in about a
second.

`controller(pose, now) -> (motor1_us, motor2_us)` is the only coupling. To
test the real control path, wrap a Vehicle with `vehicle_controller()`; to
test a controller on its own, pass any function.

Not modelled: the MAVLink link, the RC, the STM failsafe (the harness's `armed`
flag stands in for it: disarming halts the motors immediately, as the STM
does), the battery.
"""
from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Callable

from bahr_pilot import geo
from bahr_pilot.estimator import EstimateStatus, Estimator, Pose
from bahr_pilot.sitl.boat import Boat, BoatParams, Environment
from bahr_pilot.sitl.sensors import SensorConfig, SensorSuite

ORIGIN = (41.0, 29.0)           # where the simulated local frame is anchored
PHYSICS_DT = 0.01
CONTROL_HZ = 20.0               # vehicle.TICK_HZ

Controller = Callable[[Pose, float], tuple[int, int]]


@dataclass
class Trace:
    """One row per control tick."""
    t: list[float] = field(default_factory=list)
    east: list[float] = field(default_factory=list)          # truth
    north: list[float] = field(default_factory=list)
    heading: list[float] = field(default_factory=list)
    speed: list[float] = field(default_factory=list)         # over the ground
    est_east: list[float | None] = field(default_factory=list)
    est_north: list[float | None] = field(default_factory=list)
    est_heading: list[float | None] = field(default_factory=list)
    status: list[EstimateStatus] = field(default_factory=list)
    heading_valid: list[bool] = field(default_factory=list)
    m1: list[int] = field(default_factory=list)
    m2: list[int] = field(default_factory=list)
    probes: dict[str, list] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.t)

    def index_at(self, seconds: float) -> int:
        return min(len(self.t) - 1, max(0, round(seconds * CONTROL_HZ) - 1))


class Sitl:
    def __init__(self, *, boat_params: BoatParams | None = None, env: Environment | None = None,
                 sensors: SensorConfig | None = None, estimator: Estimator | None = None,
                 start_east: float = 0.0, start_north: float = 0.0, start_heading_deg: float = 0.0,
                 seabed: Callable[[float, float], float] | None = None) -> None:
        self.frame = geo.LocalFrame(*ORIGIN)
        self.boat = Boat(boat_params or BoatParams(), env or Environment())
        self.boat.state.east, self.boat.state.north = start_east, start_north
        self.boat.state.heading_deg = start_heading_deg % 360.0
        sensor_config = sensors or SensorConfig()
        if seabed is not None:
            sensor_config = dataclasses.replace(sensor_config, seabed=seabed)
        self.sensors = SensorSuite(sensor_config, self.frame)
        self._depth_pending: list[tuple[float, float]] = []
        self.estimator = estimator or Estimator()
        self.t = 0.0
        self.armed = True
        self._pulses = (1500, 1500)
        self._imu_batch: list = []
        self._latest_fix = None
        self._latest_heading = None
        self.pose: Pose | None = None

    def geodetic(self, east: float, north: float) -> tuple[float, float]:
        return self.frame.to_geodetic(east, north)

    def local(self, lat: float, lon: float) -> tuple[float, float]:
        return self.frame.to_enu(lat, lon)

    def run(self, controller: Controller, seconds: float,
            probes: dict[str, Callable[[], object]] | None = None) -> Trace:
        trace = Trace(probes={name: [] for name in (probes or {})})
        physics_per_control = round(1.0 / (CONTROL_HZ * PHYSICS_DT))
        for _ in range(round(seconds * CONTROL_HZ)):
            for _ in range(physics_per_control):
                self._physics_step()
            self.pose = self.estimator.update(
                self.t, self._imu_batch, self._latest_fix, self._latest_heading, 0)
            self._imu_batch = []
            self._pulses = tuple(controller(self.pose, self.t))
            self._record(trace, probes)
        return trace

    # -- internals ------------------------------------------------------------------------

    def _physics_step(self) -> None:
        pulses = self._pulses
        if not self.armed:
            # the STM ignores the Pi's pulses while disarmed and stops at once
            self.boat.halt_motors()
            pulses = (1500, 1500)
        self.boat.step(pulses, PHYSICS_DT)
        self.t += PHYSICS_DT
        s = self.boat.state
        out = self.sensors.update(self.t, s.east, s.north, s.heading_deg, s.yaw_rate)
        self._imu_batch.extend(out.imu)
        if out.fix is not None:
            self._latest_fix = out.fix
        if out.heading is not None:
            self._latest_heading = out.heading
        if out.depth_m is not None:
            self._depth_pending.append((self.t, out.depth_m))

    def take_depth_readings(self) -> list[tuple[float, float]]:
        """Echo-sounder readings (time, metres) produced since the last call."""
        readings, self._depth_pending = self._depth_pending, []
        return readings

    def _record(self, trace: Trace, probes) -> None:
        s = self.boat.state
        ve, vn = self.boat.ground_velocity()
        trace.t.append(self.t)
        trace.east.append(s.east)
        trace.north.append(s.north)
        trace.heading.append(s.heading_deg)
        trace.speed.append(math.hypot(ve, vn))
        pose = self.pose
        if pose is not None and pose.position_valid:
            e, n = self.local(pose.lat, pose.lon)
            trace.est_east.append(e)
            trace.est_north.append(n)
        else:
            trace.est_east.append(None)
            trace.est_north.append(None)
        trace.est_heading.append(pose.heading_deg if pose is not None and pose.heading_valid else None)
        trace.status.append(pose.status if pose is not None else EstimateStatus.NONE)
        trace.heading_valid.append(bool(pose is not None and pose.heading_valid))
        trace.m1.append(int(self._pulses[0]))
        trace.m2.append(int(self._pulses[1]))
        for name, probe in (probes or {}).items():
            trace.probes[name].append(probe())


def vehicle_controller(vehicle, sitl: Sitl, gcs_connected: Callable[[], bool] = lambda: True,
                       telemetry: bool = False) -> Controller:
    """Drive a real bahr_pilot.vehicle.Vehicle: publish the estimator's pose
    into its state, run its failsafe rules and take whatever its own
    _motor_command() decides. The simulated GCS is connected (it refreshes
    gcs_last_seen every tick) unless `gcs_connected()` says otherwise.
    Echo-sounder readings go in through GnssState.apply_echomap, the same call the NMEA reader
    makes. With `telemetry=True` send_telemetry() runs at the vehicle's 5 Hz.
    Arming and mode are the caller's to set on vehicle.state before running."""
    ticks = {"n": 0}

    def controller(pose: Pose, now: float) -> tuple[int, int]:
        vehicle.state.pose = pose
        if gcs_connected():
            vehicle.state.gcs_last_seen = now
        for reading_time, depth in sitl.take_depth_readings():
            vehicle.gnss.apply_echomap("SDDPT", [f"{depth:.1f}", "0.0"], now=reading_time)
        vehicle._update_mission_log(now)
        vehicle._update_bathymetry(now)
        vehicle._update_failsafes(now)
        vehicle._update_health(now)
        ticks["n"] += 1
        if telemetry and ticks["n"] % 4 == 0:
            vehicle.send_telemetry()
        sitl.armed = vehicle.state.armed
        return vehicle._motor_command()
    return controller


# -- metrics ----------------------------------------------------------------------------

def cross_track_errors(trace: Trace, legs: list[tuple[tuple[float, float], tuple[float, float]]],
                       leg_probe: str) -> list[float]:
    """Signed distance of the TRUE position from the active leg (positive = to
    the right of travel), one value per tick. `leg_probe` names a probe holding
    the index into `legs` that was active at that tick."""
    errors = []
    for e, n, leg_index in zip(trace.east, trace.north, trace.probes[leg_probe]):
        a, b = legs[min(max(leg_index, 0), len(legs) - 1)]
        errors.append(geo.segment_errors((e, n), a, b)[1])
    return errors
