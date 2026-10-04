"""State estimation on the Pi (Phase 6): heading, position, velocity.

Two small Kalman filters, written out in scalar form so every line can be
checked by hand, and so the Pi needs no numpy.

HeadingFilter   2 states: heading psi and gyro bias b.
                  predict: psi' = psi + (rate - b) dt        (IMU gyro, ~50 Hz)
                  update:  psi measured by the RTD100 dual antenna (~1 Hz)
                Why: the GNSS heading arrives once a second, a boat can turn
                tens of degrees in that time, and the gyro is what carries the
                heading between measurements (and through a short dropout).

PositionFilter  per axis (east, north in a LocalFrame): position + velocity,
                constant-velocity model with white acceleration noise. Both
                axes share one 2x2 covariance because the model and the
                noise are identical on both. Gives a smooth position at the
                control rate between 5 Hz fixes, a velocity (the receiver's
                Doppler velocity is not parsed yet), and outlier rejection.

Both filters
  * gate each measurement on its normalised innovation, so one wild fix or
    heading never reaches the controller; a run of consecutive rejections
    re-initialises the filter instead (otherwise a filter that has drifted
    off the truth would reject the truth forever);
  * never produce numbers from nothing: until the first valid measurement they
    report invalid, and after a measurement gap they go DEAD_RECKONING and then
    invalid on a deadline;
  * ignore NaN/inf, and timestamps that go backwards.

Frames and units: internally radians / metres / seconds, local ENU. Heading
is a compass bearing (clockwise from true north), so a positive gyro rate
about the body z axis (down) INCREASES it. Public values are degrees, like
the rest of the project.

Not verified on hardware: every noise figure below is an initial guess
(config docstrings say so). They need real BNO086 / RTD100 data to tune, and
the receivers' output latency has not been measured.
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterable

from bahr_pilot import geo
from bahr_pilot.attitude import quat_to_euler, vehicle_orientation, vehicle_vector
from bahr_pilot.gnss import FIX_TIMEOUT_S, GnssFix, GnssHeading, GnssQuality

if TYPE_CHECKING:
    from bahr_pilot.nucleo_link import ImuData

_TWO_PI = 2.0 * math.pi
ATTITUDE_FRESH_S = 1.0     # roll/pitch older than this are reported as unknown
_SYNC_JITTER_S = 0.005   # how far past its arrival time a synced IMU timestamp may sit


def _wrap_pi(angle: float) -> float:
    """Wrap radians to [-pi, pi)."""
    return (angle + math.pi) % _TWO_PI - math.pi


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


# Receiver-claimed horizontal 1-sigma is never trusted below these floors: the
# figure is the receiver's own optimism, and the boat adds its own motion
# error. Standalone fixes in particular are routinely better-claimed than real.
_POSITION_SIGMA_FLOOR_M = {
    GnssQuality.RTK_FIXED: 0.03,
    GnssQuality.FLOAT: 0.25,
    GnssQuality.DGPS: 1.0,
    GnssQuality.FIX_3D: 2.5,
    GnssQuality.FIX_2D: 5.0,
}


@dataclass(frozen=True)
class EstimatorConfig:
    # -- heading filter -----------------------------------------------------
    # Gyro white-noise density and bias random walk (initial guesses; get the
    # real values from an Allan-variance run of the mounted BNO086).
    gyro_noise_dps_per_rthz: float = 0.1
    gyro_bias_walk_dps_per_rts: float = 0.01
    initial_bias_sigma_dps: float = 1.0
    max_bias_dps: float = 3.0               # beyond this the "bias" is a fault, not a bias
    heading_sigma_floor_deg: float = 0.3    # never believe the receiver below this
    # The HEADINGA output is late by an UNMEASURED amount. The nominal value
    # is compensated (the measurement is moved forward by rate * latency) and
    # the doubt about it is added to the noise (rate * sigma). Measure both
    # on the real receiver; the defaults are guesses.
    heading_latency_s: float = 0.1
    heading_latency_sigma_s: float = 0.05
    heading_gate_sigma: float = 4.0
    heading_reject_reset: int = 5           # consecutive rejections -> re-initialise
    # How fast the boat can turn, used as the doubt about its heading rate
    # while the gyro is missing. Found the hard way in the simulator: with 10
    # deg/s here the heading stayed "valid" for a second after the gyro died
    # while a boat under full differential thrust was turning at ~38 deg/s,
    # so the reported accuracy was wrong by a factor of four. Deliberately a
    # high bound: without a gyro the heading is good for ~0.15 s after each
    # GNSS heading and then withdrawn, which is the honest answer.
    max_yaw_rate_dps: float = 60.0
    max_heading_sigma_deg: float = 10.0     # matches gnss.MAX_HEADING_ACC_DEG
    heading_dead_reckoning_s: float = 10.0  # longest the heading is carried on the gyro alone
    imu_timeout_s: float = 0.1              # no gyro sample for this long = gyro missing (samples come every 20 ms)
    min_cos_pitch: float = 0.2              # beyond ~78 deg pitch the yaw-rate formula is meaningless

    # -- position / velocity filter ------------------------------------------
    accel_noise_mps2: float = 0.5           # 1-sigma manoeuvre acceleration of the boat (guess)
    initial_speed_sigma_mps: float = 3.0
    gnss_latency_s: float = 0.1             # unmeasured, see heading_latency_s
    gnss_latency_sigma_s: float = 0.05
    position_gate_sigma: float = 4.0
    position_reject_reset: int = 5
    max_dead_reckoning_s: float = 5.0       # longest the position is carried without a fix
    max_position_sigma_m: float = 5.0
    min_course_speed_mps: float = 0.3       # below this the course over ground is noise
    max_frame_radius_m: float = 20_000.0    # the local tangent plane is mm-accurate to here
    max_step_s: float = 10.0                # a longer silence than this restarts the filter


class EstimateStatus(enum.IntEnum):
    NONE = 0             # no usable estimate
    OK = 1               # a fix was accepted within its timeout
    DEAD_RECKONING = 2   # coasting on the motion model; accuracy degrades with time


# -- heading --------------------------------------------------------------------

class HeadingFilter:
    def __init__(self, config: EstimatorConfig) -> None:
        self.cfg = config
        self._s_gyro = math.radians(config.gyro_noise_dps_per_rthz) ** 2    # rad^2/s
        self._s_bias = math.radians(config.gyro_bias_walk_dps_per_rts) ** 2  # rad^2/s^3
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.psi = 0.0
        self.bias = 0.0
        self.p00 = self.p01 = self.p11 = 0.0
        self.t: float | None = None           # time the state refers to
        self.last_accept_t: float | None = None
        self.last_rate = 0.0                  # last yaw rate fed in, rad/s
        self.rate_unknown_s = 0.0             # how long the heading has run without a known rate
        self.reject_streak = 0
        self.last_nis = 0.0
        self.accepted = 0
        self.rejected = 0
        self.resets = 0
        self.bias_clamped = 0

    # -- helpers ---------------------------------------------------------------

    @property
    def sigma_deg(self) -> float:
        return math.degrees(math.sqrt(max(self.p00, 0.0)))

    @property
    def heading_deg(self) -> float:
        return math.degrees(self.psi) % 360.0

    @property
    def bias_dps(self) -> float:
        return math.degrees(self.bias)

    def valid(self, now: float) -> bool:
        """Usable for steering. Not while the receiver is contradicting the
        filter (a rejected measurement means one of the two is wrong, and
        a frozen gyro makes it the filter): the estimate is withdrawn until a
        measurement is accepted again, instead of being trusted for the
        several seconds a reset takes."""
        return (self.initialized and self.last_accept_t is not None
                and self.reject_streak == 0
                and now - self.last_accept_t <= self.cfg.heading_dead_reckoning_s
                and self.sigma_deg <= self.cfg.max_heading_sigma_deg)

    def _init_covariance(self, sigma_rad: float) -> None:
        self.p00 = sigma_rad ** 2
        self.p01 = 0.0
        self.p11 = math.radians(self.cfg.initial_bias_sigma_dps) ** 2

    # -- predict ----------------------------------------------------------------

    def predict(self, t: float, rate_rad_s: float | None) -> None:
        """Advance to time `t`. `rate_rad_s` is the heading rate (clockwise
        positive) from the gyro, or None when there is no usable gyro, in
        which case the heading is held and its uncertainty grows by how fast
        the boat might be turning."""
        if not self.initialized or self.t is None or not _finite(t):
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        if dt > self.cfg.max_step_s:
            self.reset()                      # too long a silence to trust anything
            return
        if rate_rad_s is None or not math.isfinite(rate_rad_s):
            # An unknown but PERSISTENT turn rate makes the heading error grow
            # like (rate x time), so the variance grows with the square of the
            # time since the rate was last known. (Adding (rate x dt)^2 each
            # step instead treats the rate as fresh noise every step: it grows
            # only linearly and depends on how often this is called, which
            # kept the heading "valid" about three times too long.)
            age = self.rate_unknown_s
            self.p00 += math.radians(self.cfg.max_yaw_rate_dps) ** 2 * ((age + dt) ** 2 - age ** 2)
            self.rate_unknown_s = age + dt
            self.last_rate = 0.0
        else:
            self.rate_unknown_s = 0.0
            self.psi = _wrap_pi(self.psi + (rate_rad_s - self.bias) * dt)
            self.last_rate = rate_rad_s
            sg, sb = self._s_gyro, self._s_bias
            q00 = sg * dt + sb * dt ** 3 / 3.0
            q01 = -sb * dt ** 2 / 2.0
            q11 = sb * dt
            # P' = F P F^T + Q  with  F = [[1, -dt], [0, 1]]
            p00 = self.p00 - 2.0 * dt * self.p01 + dt * dt * self.p11 + q00
            p01 = self.p01 - dt * self.p11 + q01
            self.p00, self.p01, self.p11 = p00, p01, self.p11 + q11
        self.t = t

    # -- update -----------------------------------------------------------------

    def update(self, t: float, measured_rad: float, sigma_rad: float) -> bool:
        """Apply one GNSS heading. Returns True if it was accepted."""
        if not _finite(t, measured_rad, sigma_rad) or sigma_rad <= 0.0:
            return False
        if not self.initialized:
            self.psi = _wrap_pi(measured_rad)
            self._init_covariance(sigma_rad)
            self.initialized = True
            self.t = self.last_accept_t = t
            self.reject_streak = 0
            self.rate_unknown_s = 0.0
            self.accepted += 1
            return True

        # The measurement describes the heading `latency` ago; bring it to now
        rate = self.last_rate - self.bias
        measured_rad = _wrap_pi(measured_rad + rate * self.cfg.heading_latency_s)
        innovation = _wrap_pi(measured_rad - self.psi)
        r = sigma_rad ** 2 + (rate * self.cfg.heading_latency_sigma_s) ** 2
        s = self.p00 + r
        self.last_nis = innovation * innovation / s
        if innovation * innovation > (self.cfg.heading_gate_sigma ** 2) * s:
            self.rejected += 1
            self.reject_streak += 1
            if self.reject_streak >= self.cfg.heading_reject_reset:
                # The filter has been disagreeing with the receiver for a run
                # of measurements: believe the measurement, keep the bias.
                self.psi = _wrap_pi(measured_rad)
                self._init_covariance(sigma_rad)
                self.last_accept_t = t
                self.reject_streak = 0
                self.rate_unknown_s = 0.0
                self.resets += 1
                self.accepted += 1
                return True
            return False

        self.reject_streak = 0
        k0, k1 = self.p00 / s, self.p01 / s
        self.psi = _wrap_pi(self.psi + k0 * innovation)
        self.bias += k1 * innovation
        limit = math.radians(self.cfg.max_bias_dps)
        if abs(self.bias) > limit:
            self.bias = math.copysign(limit, self.bias)
            self.bias_clamped += 1
        # Joseph form with H = [1 0]: stays symmetric and positive definite
        a, c = 1.0 - k0, -k1
        p00 = a * a * self.p00 + k0 * k0 * r
        p01 = a * (c * self.p00 + self.p01) + k0 * k1 * r
        p11 = c * c * self.p00 + 2.0 * c * self.p01 + self.p11 + k1 * k1 * r
        self.p00, self.p01, self.p11 = p00, p01, p11
        self.last_accept_t = t
        self.rate_unknown_s = 0.0           # the heading was just corrected; the clock restarts
        self.accepted += 1
        return True


# -- position / velocity ------------------------------------------------------------

class PositionFilter:
    def __init__(self, config: EstimatorConfig) -> None:
        self.cfg = config
        self._s_accel = config.accel_noise_mps2 ** 2     # m^2/s^5
        self.reset()

    def reset(self) -> None:
        self.initialized = False
        self.e = self.n = 0.0
        self.ve = self.vn = 0.0
        self.ppp = self.ppv = self.pvv = 0.0
        self.t: float | None = None
        self.last_accept_t: float | None = None
        self.last_source = ""
        self.reject_streak = 0
        self.last_nis = 0.0
        self.accepted = 0
        self.rejected = 0
        self.resets = 0

    @property
    def speed(self) -> float:
        return math.hypot(self.ve, self.vn)

    @property
    def sigma_m(self) -> float:
        return math.sqrt(max(self.ppp, 0.0))

    def _init_state(self, t: float, e: float, n: float, sigma: float, keep_velocity: bool) -> None:
        self.e, self.n = e, n
        if not keep_velocity:
            self.ve = self.vn = 0.0
        self.ppp = sigma ** 2
        self.ppv = 0.0
        self.pvv = self.cfg.initial_speed_sigma_mps ** 2
        self.initialized = True
        self.t = self.last_accept_t = t
        self.reject_streak = 0

    def predict(self, t: float) -> None:
        if not self.initialized or self.t is None or not _finite(t):
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        if dt > self.cfg.max_step_s:
            self.reset()
            return
        self.e += self.ve * dt
        self.n += self.vn * dt
        sa = self._s_accel
        q_pp, q_pv, q_vv = sa * dt ** 3 / 3.0, sa * dt ** 2 / 2.0, sa * dt
        # P' = F P F^T + Q  with  F = [[1, dt], [0, 1]]
        ppp = self.ppp + 2.0 * dt * self.ppv + dt * dt * self.pvv + q_pp
        ppv = self.ppv + dt * self.pvv + q_pv
        self.ppp, self.ppv, self.pvv = ppp, ppv, self.pvv + q_vv
        self.t = t

    def update(self, t: float, east: float, north: float, sigma_m: float, source: str = "") -> bool:
        """Apply one position fix (local metres) with its 1-sigma. The caller
        must have called predict(t) first so the state refers to the fix time."""
        if not _finite(t, east, north, sigma_m) or sigma_m <= 0.0:
            return False
        if not self.initialized:
            self._init_state(t, east, north, sigma_m, keep_velocity=False)
            self.last_source = source
            self.accepted += 1
            return True

        # The fix describes where the boat WAS `latency` ago: move it forward
        # by the estimated velocity, and charge the noise for the velocity's
        # own uncertainty and for the doubt about the latency itself.
        lat = self.cfg.gnss_latency_s
        ye = east + self.ve * lat - self.e
        yn = north + self.vn * lat - self.n
        r = (sigma_m ** 2 + self.pvv * lat ** 2
             + (self.speed * self.cfg.gnss_latency_sigma_s) ** 2)
        s = self.ppp + r
        d2 = (ye * ye + yn * yn) / s
        self.last_nis = d2 / 2.0              # per degree of freedom, so ~1 when consistent
        if d2 > self.cfg.position_gate_sigma ** 2:
            self.rejected += 1
            self.reject_streak += 1
            if self.reject_streak >= self.cfg.position_reject_reset:
                self._init_state(t, east, north, sigma_m, keep_velocity=True)
                self.last_source = source
                self.resets += 1
                self.accepted += 1
                return True
            return False

        self.reject_streak = 0
        k0, k1 = self.ppp / s, self.ppv / s
        self.e += k0 * ye
        self.n += k0 * yn
        self.ve += k1 * ye
        self.vn += k1 * yn
        a, c = 1.0 - k0, -k1
        ppp = a * a * self.ppp + k0 * k0 * r
        ppv = a * (c * self.ppp + self.ppv) + k0 * k1 * r
        pvv = c * c * self.ppp + 2.0 * c * self.ppv + self.pvv + k1 * k1 * r
        self.ppp, self.ppv, self.pvv = ppp, ppv, pvv
        self.last_accept_t = t
        self.last_source = source
        self.accepted += 1
        return True

    def status(self, now: float) -> EstimateStatus:
        if not self.initialized or self.last_accept_t is None:
            return EstimateStatus.NONE
        age = now - self.last_accept_t
        if age > self.cfg.max_dead_reckoning_s or self.sigma_m > self.cfg.max_position_sigma_m:
            return EstimateStatus.NONE
        if age <= FIX_TIMEOUT_S.get(self.last_source, 1.0):
            return EstimateStatus.OK
        return EstimateStatus.DEAD_RECKONING


# -- the facade the vehicle uses --------------------------------------------------------

@dataclass(frozen=True)
class Pose:
    """What navigation may use. Every field that can be unknown is None, and
    `heading_valid` / `status` say whether to believe the rest."""
    t: float
    status: EstimateStatus
    lat: float | None
    lon: float | None
    east_m: float | None
    north_m: float | None
    position_sigma_m: float | None
    velocity_east_mps: float
    velocity_north_mps: float
    speed_mps: float
    course_deg: float | None         # over ground; None when too slow to mean anything
    heading_valid: bool
    heading_deg: float               # meaningful only if heading_valid
    heading_sigma_deg: float | None
    gyro_bias_dps: float
    yaw_rate_dps: float
    roll_deg: float | None = None    # vehicle frame (AHRS_ORIENTATION applied); None = no recent attitude
    pitch_deg: float | None = None

    @property
    def position_valid(self) -> bool:
        return self.status != EstimateStatus.NONE


@dataclass
class EstimatorDiagnostics:
    heading_rejected: int = 0
    heading_resets: int = 0
    position_rejected: int = 0
    position_resets: int = 0
    imu_samples: int = 0
    imu_dropped: int = 0
    bias_clamped: int = 0
    frame_exceeded: bool = False


class Estimator:
    """Owns the two filters and the local frame. Call update() from ONE thread
    (the vehicle main loop) with everything that arrived since the last call."""

    def __init__(self, config: EstimatorConfig | None = None) -> None:
        self.cfg = config or EstimatorConfig()
        self.heading = HeadingFilter(self.cfg)
        self.position = PositionFilter(self.cfg)
        self.frame: geo.LocalFrame | None = None
        self.diag = EstimatorDiagnostics()
        self._last_fix_t = -math.inf
        self._last_heading_t = -math.inf
        self._last_gyro_t = -math.inf
        self._roll = 0.0
        self._pitch = 0.0
        self._attitude_t = -math.inf       # sample time of the last valid quaternion

    # -- IMU -----------------------------------------------------------------------

    def _sample_time(self, sample: "ImuData") -> float:
        """The Pi time of an IMU sample. t_pi comes from the clock-sync
        estimator and can be wrong early on, so it is only believed while
        it is consistent with when the frame actually arrived."""
        t = sample.t_pi
        if not _finite(t) or t > sample.rx_time + _SYNC_JITTER_S or t < sample.rx_time - 0.5:
            return sample.rx_time
        return t

    def _heading_rate(self, sample: "ImuData", orientation: int) -> float | None:
        if sample.quat_valid:
            roll, pitch, _ = quat_to_euler(vehicle_orientation(sample.quat, orientation))
            if _finite(roll, pitch):
                self._roll, self._pitch = roll, pitch
                self._attitude_t = max(self._attitude_t, self._sample_time(sample))
        if not sample.gyro_valid or not _finite(*sample.gyro):
            return None
        _, q, r = vehicle_vector(sample.gyro, orientation)
        cos_pitch = math.cos(self._pitch)
        if cos_pitch < self.cfg.min_cos_pitch:
            return None
        # world-frame heading rate from body rates (aerospace ZYX, z down)
        return (q * math.sin(self._roll) + r * math.cos(self._roll)) / cos_pitch

    def _feed_imu(self, samples: Iterable["ImuData"], orientation: int) -> None:
        for sample in samples:
            self.diag.imu_samples += 1
            t = self._sample_time(sample)
            rate = self._heading_rate(sample, orientation)
            if rate is None:
                self.diag.imu_dropped += 1
                continue
            self.heading.predict(t, rate)
            self._last_gyro_t = max(self._last_gyro_t, t)

    # -- GNSS ------------------------------------------------------------------------

    def _feed_heading(self, measurement: GnssHeading | None) -> None:
        if measurement is None or measurement.t <= self._last_heading_t:
            return
        self._last_heading_t = measurement.t
        sigma_deg = max(measurement.acc_deg if measurement.acc_deg is not None else 0.0,
                        self.cfg.heading_sigma_floor_deg)
        # The filter may already be past the measurement's arrival time (an
        # IMU sample newer than it was processed first): never go backwards.
        t = measurement.t if self.heading.t is None else max(measurement.t, self.heading.t)
        gyro_ok = measurement.t - self._last_gyro_t <= self.cfg.imu_timeout_s
        self.heading.predict(t, self.heading.last_rate if gyro_ok else None)
        self.heading.update(t, math.radians(measurement.heading_deg), math.radians(sigma_deg))

    def _feed_fix(self, fix: GnssFix | None) -> None:
        if fix is None or fix.t <= self._last_fix_t:
            return
        self._last_fix_t = fix.t
        if fix.quality == GnssQuality.NO_FIX or not _finite(fix.lat, fix.lon):
            return
        if self.frame is None:
            self.frame = geo.LocalFrame(fix.lat, fix.lon)
        east, north = self.frame.to_enu(fix.lat, fix.lon)
        if math.hypot(east, north) > self.cfg.max_frame_radius_m:
            self.diag.frame_exceeded = True
            return
        floor = _POSITION_SIGMA_FLOOR_M.get(fix.quality, 5.0)
        sigma = max(fix.h_acc_m if fix.h_acc_m is not None else 0.0, floor)
        t = fix.t if self.position.t is None else max(fix.t, self.position.t)
        self.position.predict(t)
        self.position.update(t, east, north, sigma, fix.source)

    # -- the tick ----------------------------------------------------------------------

    def update(self, now: float, imu_samples: Iterable["ImuData"] = (),
               fix: GnssFix | None = None, heading: GnssHeading | None = None,
               ahrs_orientation: int = 0) -> Pose:
        self._feed_imu(imu_samples, ahrs_orientation)
        self._feed_heading(heading)
        self._feed_fix(fix)

        # Bring both filters to `now`. Without a recent gyro sample the
        # heading has to be carried as "unknown rate".
        gyro_ok = now - self._last_gyro_t <= self.cfg.imu_timeout_s
        self.heading.predict(now, self.heading.last_rate if gyro_ok else None)
        self.position.predict(now)

        self.diag.heading_rejected = self.heading.rejected
        self.diag.heading_resets = self.heading.resets
        self.diag.position_rejected = self.position.rejected
        self.diag.position_resets = self.position.resets
        self.diag.bias_clamped = self.heading.bias_clamped
        return self._pose(now)

    def _yaw_rate_dps(self) -> float:
        """The turn rate the controllers should see: the gyro rate with the
        estimated bias removed; zero while there is no gyro (an unknown rate is
        not a rate of zero, but heading_valid is already False then)."""
        h = self.heading
        if not h.initialized or h.rate_unknown_s > 0.0:
            return 0.0
        return math.degrees(h.last_rate - h.bias)

    def _pose(self, now: float) -> Pose:
        status = self.position.status(now)
        lat = lon = east = north = sigma = None
        if status != EstimateStatus.NONE and self.frame is not None:
            east, north, sigma = self.position.e, self.position.n, self.position.sigma_m
            lat, lon = self.frame.to_geodetic(east, north)
        speed = self.position.speed if status != EstimateStatus.NONE else 0.0
        ve = self.position.ve if status != EstimateStatus.NONE else 0.0
        vn = self.position.vn if status != EstimateStatus.NONE else 0.0
        course = None
        if status != EstimateStatus.NONE and speed >= self.cfg.min_course_speed_mps:
            course = math.degrees(math.atan2(ve, vn)) % 360.0
        heading_valid = self.heading.valid(now)
        attitude_fresh = now - self._attitude_t <= ATTITUDE_FRESH_S
        return Pose(
            t=now, status=status, lat=lat, lon=lon, east_m=east, north_m=north,
            position_sigma_m=sigma, velocity_east_mps=ve, velocity_north_mps=vn,
            speed_mps=speed, course_deg=course,
            heading_valid=heading_valid, heading_deg=self.heading.heading_deg,
            heading_sigma_deg=self.heading.sigma_deg if self.heading.initialized else None,
            gyro_bias_dps=self.heading.bias_dps, yaw_rate_dps=self._yaw_rate_dps(),
            roll_deg=math.degrees(self._roll) if attitude_fresh else None,
            pitch_deg=math.degrees(self._pitch) if attitude_fresh else None,
        )
