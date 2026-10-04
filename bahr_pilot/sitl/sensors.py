"""Sensor models and fault injection for software-in-the-loop testing (Phase 27).

Produces, from the simulated boat's TRUE state, exactly the objects the
estimator consumes on the real vehicle: ImuData samples (50 Hz gyro with bias
and noise), GnssFix (5 Hz, delayed, noisy) and GnssHeading (1 Hz, delayed,
noisy). The estimator and everything above it therefore run unmodified.

The noise figures describe a PLAUSIBLE RTK receiver and MEMS gyro, not the
units on the boat (nothing has been measured; see docs/SITL.md). They are
independent of the estimator's own configured figures on purpose: a test that
feeds the filter exactly the noise it assumes proves little.

Faults are time windows, so a scenario reads like a test report: "GNSS lost
from t=30 to t=40".
"""
from __future__ import annotations

import math
import random
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from bahr_pilot import geo
from bahr_pilot.attitude import euler_to_quat
from bahr_pilot.gnss import GnssFix, GnssHeading, GnssQuality
from bahr_pilot.nucleo_link import ImuData

_TICK_EPS = 1e-9      # simulated time is a running float sum; do not lose a sample to rounding

FAULT_KINDS = {
    "gnss_outage",       # no position fixes
    "heading_outage",    # no GNSS heading
    "imu_outage",        # no IMU frames at all
    "gyro_invalid",      # IMU frames arrive with the gyro-valid bit clear
    "gnss_jump",         # every fix is displaced by value = (east_m, north_m)
    "heading_offset",    # every GNSS heading is off by value degrees (antenna misalignment)
    "gyro_stuck",        # the gyro keeps reporting the rate it had when the fault began
    "gyro_bias_step",    # the gyro bias changes by value deg/s (temperature, shock)
    "gnss_degraded",     # fixes drop to a standalone 3D solution with a 2.5 m sigma
    "sonar_outage",      # no depth readings
}


@dataclass(frozen=True)
class Fault:
    kind: str
    start_s: float
    end_s: float = math.inf
    value: float | tuple[float, float] = 0.0

    def __post_init__(self) -> None:
        if self.kind not in FAULT_KINDS:
            raise ValueError(f"unknown fault {self.kind!r}; known: {sorted(FAULT_KINDS)}")

    def active(self, t: float) -> bool:
        return self.start_s <= t < self.end_s


@dataclass(frozen=True)
class SensorConfig:
    # GNSS position (RTK fixed)
    gnss_rate_hz: float = 5.0
    gnss_sigma_m: float = 0.02
    gnss_latency_s: float = 0.1
    # GNSS dual-antenna heading
    heading_rate_hz: float = 1.0
    heading_sigma_deg: float = 0.3
    heading_latency_s: float = 0.1
    # gyro
    imu_rate_hz: float = 50.0
    gyro_noise_dps_per_rthz: float = 0.1
    gyro_bias_dps: float = 0.5
    gyro_bias_walk_dps_per_rts: float = 0.01
    uart_latency_s: float = 0.003          # frame arrival after the sample instant
    wave_roll_deg: float = 0.0             # amplitude of a sinusoidal roll, 0 = flat water
    # echo sounder (a depth is produced only when a `seabed` function is given)
    seabed: Callable[[float, float], float] | None = None    # (east, north) m -> true depth m
    sonar_rate_hz: float = 1.0
    sonar_sigma_m: float = 0.05
    sonar_resolution_m: float = 0.1                          # the echoMAP reports tenths of a metre
    sonar_latency_s: float = 0.2                             # the echo describes where the boat WAS
    sonar_spike_rate: float = 0.0                            # fraction of readings replaced by a spike
    sonar_zero_rate: float = 0.0                             # fraction with no bottom lock (reports 0)
    seed: int = 1
    faults: tuple[Fault, ...] = ()


@dataclass
class SensorOutput:
    imu: list[ImuData] = field(default_factory=list)
    fix: GnssFix | None = None
    heading: GnssHeading | None = None
    depth_m: float | None = None          # a raw echo-sounder reading, as the unit would report it


class SensorSuite:
    """Call update() once per simulation step with the boat's current truth."""

    def __init__(self, config: SensorConfig, frame: geo.LocalFrame) -> None:
        self.cfg = config
        self.frame = frame
        self.rng = random.Random(config.seed)
        self._history: deque[tuple[float, float, float, float]] = deque(maxlen=2000)  # t, e, n, heading_unwrapped
        # the first sample of each sensor is one period in, so a run of N
        # seconds yields exactly N x rate samples
        self._next_imu = 1.0 / config.imu_rate_hz
        self._next_gnss = 1.0 / config.gnss_rate_hz
        self._next_heading = 1.0 / config.heading_rate_hz
        self._next_sonar = 1.0 / config.sonar_rate_hz
        self._bias_dps = config.gyro_bias_dps
        self._stuck_rate: float | None = None
        self._last_t = 0.0
        self._heading_unwrapped = 0.0

    # -- faults ------------------------------------------------------------------------

    def _fault(self, kind: str, t: float) -> Fault | None:
        for fault in self.cfg.faults:
            if fault.kind == kind and fault.active(t):
                return fault
        return None

    # -- truth history, for latency ----------------------------------------------------

    def _truth_at(self, t: float) -> tuple[float, float, float]:
        """Linearly interpolated (east, north, unwrapped heading deg) at time t."""
        history = self._history
        if t <= history[0][0]:
            return history[0][1:]
        for i in range(len(history) - 1, 0, -1):
            t0, e0, n0, h0 = history[i - 1]
            t1, e1, n1, h1 = history[i]
            if t0 <= t <= t1:
                w = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return e0 + w * (e1 - e0), n0 + w * (n1 - n0), h0 + w * (h1 - h0)
        return history[-1][1:]

    # -- the step ------------------------------------------------------------------------

    def update(self, t: float, east: float, north: float, heading_deg: float,
               yaw_rate_rad_s: float) -> SensorOutput:
        cfg = self.cfg
        # unwrap the heading so interpolation across north is smooth
        if self._history:
            previous = self._heading_unwrapped
            self._heading_unwrapped = previous + geo.wrap_180(heading_deg - previous)
        else:
            self._heading_unwrapped = heading_deg
        self._history.append((t, east, north, self._heading_unwrapped))
        dt = t - self._last_t if t > self._last_t else 0.0
        self._last_t = t
        out = SensorOutput()

        # bias random walk, plus any commanded step
        self._bias_dps += self.rng.gauss(0.0, cfg.gyro_bias_walk_dps_per_rts * math.sqrt(dt)) if dt > 0 else 0.0
        step = self._fault("gyro_bias_step", t)
        bias = self._bias_dps + (float(step.value) if step else 0.0)

        if t >= self._next_imu - _TICK_EPS:
            self._next_imu += 1.0 / cfg.imu_rate_hz
            if not self._fault("imu_outage", t):
                out.imu.append(self._imu_sample(t, yaw_rate_rad_s, bias))
        if t >= self._next_gnss - _TICK_EPS:
            self._next_gnss += 1.0 / cfg.gnss_rate_hz
            if not self._fault("gnss_outage", t):
                out.fix = self._gnss_fix(t)
        if t >= self._next_heading - _TICK_EPS:
            self._next_heading += 1.0 / cfg.heading_rate_hz
            if not self._fault("heading_outage", t):
                out.heading = self._gnss_heading(t)
        if cfg.seabed is not None and t >= self._next_sonar - _TICK_EPS:
            self._next_sonar += 1.0 / cfg.sonar_rate_hz
            if not self._fault("sonar_outage", t):
                out.depth_m = self._sonar_reading(t)
        return out

    def _sonar_reading(self, t: float) -> float:
        cfg = self.cfg
        east, north, _ = self._truth_at(t - cfg.sonar_latency_s)
        true_depth = cfg.seabed(east, north)
        roll = self.rng.random()
        if roll < cfg.sonar_zero_rate:
            return 0.0                                        # no bottom lock
        if roll < cfg.sonar_zero_rate + cfg.sonar_spike_rate:
            true_depth += self.rng.choice((-1.0, 1.0)) * self.rng.uniform(1.5, 6.0)      # fish, weed, bubbles
        reading = true_depth + self.rng.gauss(0.0, cfg.sonar_sigma_m)
        return max(0.0, round(reading / cfg.sonar_resolution_m) * cfg.sonar_resolution_m)

    # -- individual sensors -----------------------------------------------------------------

    def _imu_sample(self, t: float, yaw_rate_rad_s: float, bias_dps: float) -> ImuData:
        cfg = self.cfg
        sample_dt = 1.0 / cfg.imu_rate_hz
        noise_dps = self.rng.gauss(0.0, cfg.gyro_noise_dps_per_rthz / math.sqrt(sample_dt))
        rate = yaw_rate_rad_s + math.radians(bias_dps + noise_dps)
        if self._fault("gyro_stuck", t):
            if self._stuck_rate is None:
                self._stuck_rate = rate
            rate = self._stuck_rate
        else:
            self._stuck_rate = None
        roll = math.radians(cfg.wave_roll_deg) * math.sin(2.0 * math.pi * 0.4 * t)
        return ImuData(
            t_stm_us=int(t * 1e6) & 0xFFFFFFFF, t_pi=t, rx_time=t + cfg.uart_latency_s,
            quat=euler_to_quat(roll, 0.0, 0.0), gyro=(0.0, 0.0, rate), accel=(0.0, 0.0, 0.0),
            quat_valid=True, gyro_valid=not self._fault("gyro_invalid", t), accel_valid=True,
            quat_accuracy=3,
        )

    def _gnss_fix(self, t: float) -> GnssFix:
        cfg = self.cfg
        east, north, _ = self._truth_at(t - cfg.gnss_latency_s)
        sigma, quality = cfg.gnss_sigma_m, GnssQuality.RTK_FIXED
        if self._fault("gnss_degraded", t):
            sigma, quality = 2.5, GnssQuality.FIX_3D
        east += self.rng.gauss(0.0, sigma)
        north += self.rng.gauss(0.0, sigma)
        jump = self._fault("gnss_jump", t)
        if jump:
            east += jump.value[0]
            north += jump.value[1]
        lat, lon = self.frame.to_geodetic(east, north)
        return GnssFix(t=t, lat=lat, lon=lon, alt_m=0.0, quality=quality, satellites=20,
                       source="rtd100", h_acc_m=sigma)

    def _gnss_heading(self, t: float) -> GnssHeading:
        cfg = self.cfg
        heading = self._truth_at(t - cfg.heading_latency_s)[2] + self.rng.gauss(0.0, cfg.heading_sigma_deg)
        offset = self._fault("heading_offset", t)
        if offset:
            heading += float(offset.value)
        return GnssHeading(t=t, heading_deg=heading % 360.0, acc_deg=cfg.heading_sigma_deg, baseline_m=1.0,
                           quality=GnssQuality.RTK_FIXED, pos_type="NARROW_INT")
