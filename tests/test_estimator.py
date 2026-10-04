"""Phase 6 estimator: heading + position/velocity filters.

Everything here runs on synthetic data from a small boat simulation (truth
trajectory -> noisy gyro, delayed GNSS heading, delayed GNSS position). No
number below has been checked against a real BNO086 or RTD100; what these
tests establish is that the filters implement their model correctly and
behave safely, not that the tuning suits the hardware.

Independent check of the maths: a Kalman filter whose model matches the
simulation must have a mean normalised innovation squared (NIS) near 1.
A wrong Q, R, sign or covariance update moves that number away from 1 without
any test having to know the right answer.
"""
from __future__ import annotations

import math
import random

import pytest

from bahr_pilot import geo
from bahr_pilot.attitude import euler_to_quat
from bahr_pilot.estimator import (
    Estimator, EstimatorConfig, EstimateStatus, HeadingFilter, PositionFilter,
)
from bahr_pilot.gnss import GnssFix, GnssHeading, GnssQuality
from bahr_pilot.nucleo_link import ImuData

ORIGIN = (41.0, 29.0)
IMU_DT = 0.02          # 50 Hz, like the firmware
CFG = EstimatorConfig()


def _wrap(angle_deg: float) -> float:
    return geo.wrap_180(angle_deg)


def imu_sample(t: float, yaw_rate_rad: float, *, roll: float = 0.0, pitch: float = 0.0,
               valid: bool = True, body_rates: tuple[float, float, float] | None = None) -> ImuData:
    return ImuData(
        t_stm_us=int(t * 1e6) & 0xFFFFFFFF, t_pi=t, rx_time=t + 0.003,
        quat=euler_to_quat(roll, pitch, 0.0),
        gyro=body_rates if body_rates is not None else (0.0, 0.0, yaw_rate_rad),
        accel=(0.0, 0.0, 0.0), quat_valid=True, gyro_valid=valid, accel_valid=True, quat_accuracy=3,
    )


def gnss_heading(t: float, heading_deg: float, sigma_deg: float = 0.3) -> GnssHeading:
    return GnssHeading(t=t, heading_deg=heading_deg % 360.0, acc_deg=sigma_deg, baseline_m=1.0,
                       quality=GnssQuality.RTK_FIXED, pos_type="NARROW_INT")


def gnss_fix(t: float, east: float, north: float, *, quality=GnssQuality.RTK_FIXED,
             h_acc: float | None = 0.03, source: str = "rtd100") -> GnssFix:
    lat, lon = geo.LocalFrame(*ORIGIN).to_geodetic(east, north)
    return GnssFix(t=t, lat=lat, lon=lon, alt_m=0.0, quality=quality, satellites=20,
                   source=source, h_acc_m=h_acc)


# -- heading simulation ----------------------------------------------------------------

class HeadingSim:
    """Truth heading driven by `rate_dps(t)`; a gyro with a constant bias and
    white noise at 50 Hz; a GNSS heading at 1 Hz that is `latency` old when it
    arrives. Returns the estimator pose and the truth at every IMU tick."""

    def __init__(self, rate_dps, *, heading0=0.0, bias_dps=0.5, gyro_noise_dps=0.707,
                 gnss_sigma_deg=0.3, latency=CFG.heading_latency_s,
                 latency_jitter=CFG.heading_latency_sigma_s, seed=1, config=None,
                 gnss_outage=(math.inf, math.inf), imu_outage=(math.inf, math.inf)):
        self.rate_dps, self.bias, self.gyro_noise = rate_dps, bias_dps, gyro_noise_dps
        self.gnss_sigma, self.latency, self.latency_jitter = gnss_sigma_deg, latency, latency_jitter
        self.rng = random.Random(seed)
        self.truth = heading0
        self.est = Estimator(config)
        self.gnss_outage, self.imu_outage = gnss_outage, imu_outage
        self.t = 0.0
        self._next_gnss = 0.0
        self._history: list[tuple[float, float]] = []
        self.nis: list[float] = []

    def _truth_at(self, t: float) -> float:
        for ht, hv in reversed(self._history):
            if ht <= t:
                return hv
        return self._history[0][1]

    def step(self):
        self.t += IMU_DT
        t = self.t
        rate = self.rate_dps(t)
        self.truth = (self.truth + rate * IMU_DT) % 360.0
        self._history.append((t, self.truth))
        samples = []
        if not (self.imu_outage[0] <= t < self.imu_outage[1]):
            measured = math.radians(rate + self.bias + self.rng.gauss(0, self.gyro_noise))
            samples.append(imu_sample(t, measured))
        heading = None
        gnss_fed = False
        if t >= self._next_gnss:
            self._next_gnss += 1.0
            if not (self.gnss_outage[0] <= t < self.gnss_outage[1]):
                delay = self.latency + self.rng.gauss(0, self.latency_jitter)
                heading = gnss_heading(t, self._truth_at(t - delay) + self.rng.gauss(0, self.gnss_sigma),
                                       self.gnss_sigma)
                gnss_fed = True
        pose = self.est.update(t, samples, None, heading)
        if gnss_fed and self.est.heading.accepted + self.est.heading.rejected > 1:
            self.nis.append(self.est.heading.last_nis)
        return pose, self.truth

    def run(self, seconds):
        out = []
        for _ in range(int(seconds / IMU_DT)):
            out.append(self.step())
        return out


def _rms(values):
    return math.sqrt(sum(v * v for v in values) / len(values))


# -- heading: basic behaviour -------------------------------------------------------------

def test_heading_initialises_from_the_first_measurement_and_not_before():
    est = Estimator()
    assert not est.update(0.0, [imu_sample(0.0, 0.0)]).heading_valid
    pose = est.update(0.1, [], None, gnss_heading(0.1, 123.0))
    assert pose.heading_valid and pose.heading_deg == pytest.approx(123.0, abs=0.01)


def test_positive_gyro_rate_is_clockwise_so_heading_increases():
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 90.0))
    for k in range(1, 51):                       # 1 s at +20 deg/s
        pose = est.update(k * IMU_DT, [imu_sample(k * IMU_DT, math.radians(20.0))])
    assert pose.heading_deg == pytest.approx(110.0, abs=0.7)   # bias still unknown, so not exact


@pytest.mark.parametrize("start,rate", [(350.0, 10.0), (10.0, -10.0), (170.0, 10.0), (190.0, -10.0)])
def test_heading_wraps_across_north_and_across_south(start, rate):
    """The filter keeps its heading in [-180, 180), so the wrap that matters
    for the innovation is at SOUTH; north only exercises the output wrap."""
    run = HeadingSim(lambda t: rate, heading0=start, bias_dps=0.0).run(40)
    errors = [_wrap(p.heading_deg - truth) for p, truth in run if p.heading_valid][500:]
    assert max(abs(e) for e in errors) < 1.5


@pytest.mark.parametrize("state_deg,measured_deg", [(179.9, 180.1), (180.1, 179.9), (359.9, 0.1), (0.1, 359.9)])
def test_innovation_is_wrapped_at_the_branch_cut(state_deg, measured_deg):
    """Deterministic version of the wrap check: a state and a measurement 0.2
    degrees apart but on opposite sides of +-180 must be accepted as 0.2, not
    rejected as 359.8. (The continuous runs above cross the cut only once, and
    a 1 Hz measurement has to land in a ~0.2 s window to expose it.)"""
    f = HeadingFilter(CFG)
    f.update(0.0, math.radians(state_deg), math.radians(0.3))
    assert f.update(0.1, math.radians(measured_deg), math.radians(0.3)) is True
    assert f.rejected == 0
    midpoint = state_deg + _wrap(measured_deg - state_deg) / 2.0       # circular, not arithmetic, mean
    assert _wrap(f.heading_deg - midpoint) == pytest.approx(0.0, abs=0.2)


def test_gyro_carries_the_heading_between_one_hertz_measurements():
    """The reason the filter exists: holding each GNSS heading until the next
    one is several degrees out in a turn, the filter is not."""
    sim = HeadingSim(lambda t: 10.0, heading0=0.0)
    run = sim.run(60)
    settled = run[1500:]                          # after 30 s
    filtered = [_wrap(p.heading_deg - truth) for p, truth in settled]
    held, last = [], None
    for k, (_, truth) in enumerate(settled):
        if k % 50 == 0:
            last = truth
        held.append(_wrap(last - truth))
    assert _rms(filtered) < 1.0
    assert _rms(held) > 3.0


def test_gyro_bias_is_estimated():
    sim = HeadingSim(lambda t: 0.0, bias_dps=0.8)
    sim.run(180)
    assert sim.est.heading.bias_dps == pytest.approx(0.8, abs=0.15)


def test_heading_filter_is_statistically_consistent():
    """Mean NIS ~ 1 for a filter whose model matches the simulation."""
    sim = HeadingSim(lambda t: 8.0 * math.sin(t / 7.0), bias_dps=0.4, seed=5)
    sim.run(600)
    mean_nis = sum(sim.nis) / len(sim.nis)
    assert 0.5 < mean_nis < 1.6, mean_nis


# -- heading: safety ---------------------------------------------------------------------------

def test_one_wild_heading_is_rejected_and_does_not_move_the_estimate():
    sim = HeadingSim(lambda t: 0.0, heading0=200.0, bias_dps=0.0)
    sim.run(30)
    before = sim.est.heading.heading_deg
    pose = sim.est.update(sim.t + 0.001, [], None, gnss_heading(sim.t + 0.001, before + 90.0))
    assert sim.est.heading.rejected == 1
    assert _wrap(pose.heading_deg - before) == pytest.approx(0.0, abs=0.1)


def test_persistent_disagreement_resets_instead_of_locking_out():
    """A confident filter (gyro running, so its doubt stays small) that has
    ended up far from the receiver must not reject the truth forever."""
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 10.0))
    for k in range(1, 8 * 50 + 1):                      # 8 s of 50 Hz gyro samples, heading steady
        now = k * IMU_DT
        measurement = gnss_heading(now, 100.0) if k % 50 == 0 else None
        pose = est.update(now, [imu_sample(now, 0.0)], None, measurement)
    assert est.heading.rejected >= 4 and est.heading.resets == 1
    assert _wrap(est.heading.heading_deg - 100.0) == pytest.approx(0.0, abs=1.0)
    assert pose.heading_valid


def test_without_a_gyro_the_filter_is_unsure_enough_to_accept_a_large_correction():
    """The other side of the coin: no gyro means the doubt is huge, so a
    measurement 90 degrees away is believed rather than rejected."""
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 10.0))
    for k in range(1, 4):
        est.update(float(k), [], None, gnss_heading(float(k), 100.0))
    assert est.heading.rejected == 0
    assert _wrap(est.heading.heading_deg - 100.0) == pytest.approx(0.0, abs=1.0)


def test_heading_dies_quickly_without_a_gyro_and_slowly_with_one():
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 0.0, 0.3))
    # no gyro: the doubt grows by the boat's maximum yaw rate (60 deg/s)
    assert est.update(0.1, [], None, None).heading_valid          # 6 deg of doubt
    assert not est.update(0.25, [], None, None).heading_valid     # 15 deg
    assert not est.update(1.5, [], None, None).heading_valid

    sim = HeadingSim(lambda t: 0.0, gnss_outage=(5.0, math.inf))
    run = sim.run(20)

    def valid_at(seconds):
        return run[round(seconds / IMU_DT) - 1][0].heading_valid
    # the last heading before the outage was accepted at ~4 s, the limit is 10 s
    assert valid_at(13.0) and not valid_at(15.0)


def test_unknown_rate_doubt_does_not_depend_on_how_often_the_filter_is_stepped():
    """A persistent unknown turn rate gives doubt = rate x time, however finely
    time is sliced. (The first version added (rate x dt)^2 per step, which
    grew linearly and so depended on the loop rate.)"""
    sigmas = []
    for step in (0.01, 0.05, 0.25, 0.5):
        est = Estimator()
        est.update(0.0, [], None, gnss_heading(0.0, 0.0, 0.3))
        for k in range(1, round(0.5 / step) + 1):
            est.update(k * step, [], None, None)
        sigmas.append(est.heading.sigma_deg)
    assert max(sigmas) - min(sigmas) < 0.01
    assert sigmas[0] == pytest.approx(math.sqrt(0.3 ** 2 + (60.0 * 0.5) ** 2), rel=1e-3)


def test_heading_is_withdrawn_while_the_receiver_contradicts_it():
    """Found in the simulator: a frozen gyro made the filter reject the GNSS
    heading for seconds while still reporting its own frozen heading as valid,
    and the boat spun. Disagreement now withdraws the estimate until a
    measurement is accepted again."""
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 0.0))
    assert est.update(0.5, [imu_sample(0.5, 0.0)]).heading_valid
    pose = est.update(1.0, [imu_sample(1.0, 0.0)], None, gnss_heading(1.0, 90.0))   # contradicts: rejected
    assert est.heading.rejected == 1 and not pose.heading_valid
    pose = est.update(2.0, [imu_sample(2.0, 0.0)], None, gnss_heading(2.0, 0.2))    # agrees again
    assert pose.heading_valid and est.heading.reject_streak == 0


def test_heading_with_gyro_marked_invalid_counts_as_missing():
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 0.0))
    est.update(0.5, [imu_sample(0.5, 1.0, valid=False)])
    assert est.diag.imu_dropped == 1
    assert est.heading.sigma_deg > 1.0    # the doubt grew, the bogus rate was not integrated
    assert est.heading.heading_deg == pytest.approx(0.0, abs=0.01)


def test_nan_inputs_never_reach_the_state():
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 45.0))
    est.update(0.1, [imu_sample(0.1, float("nan"))])
    est.update(0.2, [], None, GnssHeading(0.2, float("nan"), 0.3, 1.0, GnssQuality.RTK_FIXED, "x"))
    bad_fix = GnssFix(0.2, float("nan"), 29.0, 0.0, GnssQuality.RTK_FIXED, 10, "rtd100", h_acc_m=0.03)
    pose = est.update(0.3, [], bad_fix, None)
    assert math.isfinite(pose.heading_deg) and est.heading.heading_deg == pytest.approx(45.0, abs=0.5)
    assert pose.status == EstimateStatus.NONE and est.frame is None


def test_time_going_backwards_is_ignored():
    est = Estimator()
    est.update(10.0, [], None, gnss_heading(10.0, 30.0))
    est.update(10.5, [imu_sample(10.5, 0.2)])
    snapshot = (est.heading.psi, est.heading.p00, est.heading.t)
    est.update(9.0, [imu_sample(9.0, 5.0)])
    assert (est.heading.psi, est.heading.p00, est.heading.t) == snapshot


def test_a_long_silence_restarts_the_filter_instead_of_trusting_it():
    est = Estimator()
    est.update(0.0, [], None, gnss_heading(0.0, 30.0))
    est.update(60.0, [imu_sample(60.0, 0.0)])
    assert not est.heading.initialized


def test_unsynced_imu_timestamps_fall_back_to_arrival_time():
    est = Estimator()
    stale = imu_sample(5.0, 0.0)
    stale = ImuData(**{**stale.__dict__, "t_pi": -100.0})
    future = ImuData(**{**stale.__dict__, "t_pi": 500.0})
    assert est._sample_time(stale) == pytest.approx(5.003)
    assert est._sample_time(future) == pytest.approx(5.003)
    assert est._sample_time(imu_sample(5.0, 0.0)) == pytest.approx(5.0)


def test_heading_rate_uses_roll_and_pitch():
    """psi_dot = (q sin(roll) + r cos(roll)) / cos(pitch)"""
    est = Estimator()
    roll, pitch = math.radians(30.0), math.radians(10.0)
    q, r = 0.2, 0.5
    rate = est._heading_rate(imu_sample(0.0, 0.0, roll=roll, pitch=pitch, body_rates=(0.1, q, r)), 0)
    assert rate == pytest.approx((q * math.sin(roll) + r * math.cos(roll)) / math.cos(pitch), rel=1e-6)


def test_extreme_pitch_makes_the_rate_unusable_rather_than_huge():
    est = Estimator()
    sample = imu_sample(0.0, 0.0, pitch=math.radians(85.0), body_rates=(0.0, 0.0, 1.0))
    assert est._heading_rate(sample, 0) is None


def test_covariance_stays_symmetric_positive_definite_over_a_long_run():
    sim = HeadingSim(lambda t: 15.0 * math.sin(t / 3.0), seed=11)
    sim.run(900)
    h = sim.est.heading
    assert h.p00 > 0 and h.p11 > 0 and h.p00 * h.p11 - h.p01 ** 2 > 0


# -- position / velocity simulation ----------------------------------------------------------------

class PositionSim:
    """Truth moves with white-noise acceleration (the filter's own model); a
    5 Hz fix of where the boat was `latency` ago, plus noise."""

    def __init__(self, *, v0=(1.5, 0.0), sigma=0.03, latency=CFG.gnss_latency_s,
                 latency_jitter=CFG.gnss_latency_sigma_s, seed=2, config=None, accel=CFG.accel_noise_mps2, fix_period=0.2, source="rtd100",
                 quality=GnssQuality.RTK_FIXED, outage=(math.inf, math.inf)):
        self.rng = random.Random(seed)
        self.e, self.n = 0.0, 0.0
        self.ve, self.vn = v0
        self.sigma, self.latency, self.accel = sigma, latency, accel
        self.latency_jitter = latency_jitter
        self.est = Estimator(config)
        self.t = 0.0
        self.period, self.source, self.quality, self.outage = fix_period, source, quality, outage
        self._next_fix = 0.0
        self._history: list[tuple[float, float, float]] = []
        self.nis: list[float] = []

    def _pos_at(self, t):
        for ht, he, hn in reversed(self._history):
            if ht <= t:
                return he, hn
        return self._history[0][1:]

    def step(self):
        self.t += IMU_DT
        t = self.t
        sd = self.accel * math.sqrt(IMU_DT)
        self.ve += self.rng.gauss(0, sd)
        self.vn += self.rng.gauss(0, sd)
        self.e += self.ve * IMU_DT
        self.n += self.vn * IMU_DT
        self._history.append((t, self.e, self.n))
        fix, fed = None, False
        if t >= self._next_fix:
            self._next_fix += self.period
            if not (self.outage[0] <= t < self.outage[1]):
                he, hn = self._pos_at(t - (self.latency + self.rng.gauss(0, self.latency_jitter)))
                fix = gnss_fix(t, he + self.rng.gauss(0, self.sigma), hn + self.rng.gauss(0, self.sigma),
                               h_acc=self.sigma, source=self.source, quality=self.quality)
                fed = True
        pose = self.est.update(t, [], fix, None)
        if fed and self.est.position.accepted + self.est.position.rejected > 2:
            self.nis.append(self.est.position.last_nis)
        return pose

    def error(self, pose):
        east, north = geo.LocalFrame(*ORIGIN).to_enu(pose.lat, pose.lon)
        return east - self.e, north - self.n


def test_position_and_velocity_converge_on_a_moving_boat():
    # truth acceleration 0.1 m/s^2 (a survey boat holding speed); the filter
    # itself is configured for 0.5 so it is deliberately not tuned to this
    sim = PositionSim(v0=(1.5, 0.0), accel=0.1)
    errors, speed_errors = [], []
    for k in range(int(60 / IMU_DT)):
        pose = sim.step()
        if sim.t > 20 and pose.status == EstimateStatus.OK:
            ex, ey = sim.error(pose)
            errors.append(math.hypot(ex, ey))
            speed_errors.append(math.hypot(pose.velocity_east_mps - sim.ve, pose.velocity_north_mps - sim.vn))
    # measured 0.086 m / 0.185 m/s; the position error is dominated by the
    # assumed doubt about the GNSS latency (0.05 s x 1.5 m/s = 7.5 cm)
    assert _rms(errors) < 0.12
    assert _rms(speed_errors) < 0.25


def test_course_over_ground_is_a_compass_bearing():
    """Compared with the TRUE course of the simulated boat (which wanders a
    little), over the last 10 s; the single-sample error is a few degrees of
    velocity noise, the mean must be ~0."""
    for v in ((0.0, 1.5), (1.5, 0.0), (0.0, -1.5), (-1.5, 0.0)):
        sim = PositionSim(v0=v, accel=0.01, seed=3)
        errors = []
        for _ in range(int(30 / IMU_DT)):
            pose = sim.step()
            if sim.t > 20:
                true_course = math.degrees(math.atan2(sim.ve, sim.vn)) % 360.0
                errors.append(_wrap(pose.course_deg - true_course))
        assert abs(sum(errors) / len(errors)) < 1.0, (v, errors[-1])
        assert max(abs(e) for e in errors) < 10.0


def test_course_is_unknown_when_the_boat_is_not_moving():
    sim = PositionSim(v0=(0.0, 0.0), accel=0.001, seed=4)
    for _ in range(int(20 / IMU_DT)):
        pose = sim.step()
    assert pose.speed_mps < 0.3 and pose.course_deg is None


def test_position_filter_is_statistically_consistent():
    """NEES: the squared error divided by the filter's own claimed variance,
    averaged, must be ~1 per degree of freedom. Unlike the innovation check
    it compares the estimate with the TRUTH, so it also catches errors that
    the measurements cannot reveal. The simulation includes the latency
    jitter the filter assumes."""
    sim = PositionSim(seed=8)
    nees = []
    for _ in range(int(300 / IMU_DT)):
        pose = sim.step()
        if sim.t > 20 and pose.status == EstimateStatus.OK:
            ex, ey = sim.error(pose)
            nees.append((ex * ex + ey * ey) / (2.0 * sim.est.position.ppp))
    mean_nees = sum(nees) / len(nees)
    assert 0.6 < mean_nees < 1.6, mean_nees        # measured ~1.2: mildly optimistic, not badly
    # The innovation includes a deliberate allowance for the latency doubt that
    # the velocity compensation slightly double counts, so it reads below 1
    # (measured ~0.6). Over-confidence would show as > 1.
    mean_nis = sum(sim.nis) / len(sim.nis)
    assert 0.2 < mean_nis < 1.6, mean_nis


def test_latency_compensation_removes_the_lag():
    """The fix describes where the boat WAS 0.1 s ago; at 1.5 m/s that is 15 cm
    if nobody corrects for it."""
    def bias(config):
        sim = PositionSim(v0=(2.0, 0.0), accel=0.05, seed=6, config=config)
        along = []
        for _ in range(int(60 / IMU_DT)):
            pose = sim.step()
            if sim.t > 20 and pose.status == EstimateStatus.OK:
                along.append(sim.error(pose)[0])
        return abs(sum(along) / len(along))
    compensated = bias(EstimatorConfig())
    uncompensated = bias(EstimatorConfig(gnss_latency_s=0.0))
    assert uncompensated > 0.12
    assert compensated < 0.04


def test_one_wild_fix_is_rejected():
    sim = PositionSim(seed=9)
    for _ in range(int(20 / IMU_DT)):
        sim.step()
    before = (sim.est.position.e, sim.est.position.n)
    jump = gnss_fix(sim.t + 0.001, sim.e + 50.0, sim.n, h_acc=0.03)
    pose = sim.est.update(sim.t + 0.001, [], jump, None)
    assert sim.est.position.rejected == 1
    assert math.hypot(sim.est.position.e - before[0], sim.est.position.n - before[1]) < 0.3
    assert pose.status == EstimateStatus.OK


def test_persistent_position_disagreement_resets_instead_of_locking_out():
    est = Estimator()
    est.update(0.0, [], gnss_fix(0.0, 0.0, 0.0))
    for k in range(1, 9):
        est.update(k * 0.2, [], gnss_fix(k * 0.2, 80.0, 0.0))
    assert est.position.resets == 1
    assert est.position.e == pytest.approx(80.0, abs=0.5)


def test_position_goes_dead_reckoning_then_unavailable_on_schedule():
    sim = PositionSim(seed=10, outage=(20.0, math.inf))
    states = {}
    for _ in range(int(30 / IMU_DT)):
        pose = sim.step()
        states[round(sim.t, 2)] = pose.status
    # the last fix (t=19.8..20) was accepted at ~19.8; rtd100 timeout 1 s, dead-reckoning limit 5 s
    assert states[20.2] == EstimateStatus.OK
    assert states[21.5] == EstimateStatus.DEAD_RECKONING
    assert states[24.0] == EstimateStatus.DEAD_RECKONING
    assert states[26.0] == EstimateStatus.NONE


def test_dead_reckoning_error_stays_inside_the_claimed_uncertainty():
    """Coasting drifts by (velocity error x time) - about half a metre after
    3 s here. What matters is that the filter SAYS so: the claimed sigma grows
    and the true error stays within 3 of it, so a consumer can decide."""
    sim = PositionSim(seed=12, accel=0.02, outage=(30.0, math.inf))
    sigmas, worst_ratio, worst = [], 0.0, 0.0
    for _ in range(int(34 / IMU_DT)):
        pose = sim.step()
        if sim.t > 31 and pose.status == EstimateStatus.DEAD_RECKONING:
            error = math.hypot(*sim.error(pose))
            worst, worst_ratio = max(worst, error), max(worst_ratio, error / pose.position_sigma_m)
            sigmas.append(pose.position_sigma_m)
    assert sigmas and sigmas[-1] > 2.0 * sigmas[0]      # the doubt visibly grows while coasting
    assert worst_ratio < 3.0
    assert worst < 1.5


def test_lost_position_is_reported_as_unknown_not_as_the_last_value():
    sim = PositionSim(seed=13, outage=(10.0, math.inf))
    for _ in range(int(20 / IMU_DT)):
        pose = sim.step()
    assert pose.status == EstimateStatus.NONE
    assert pose.lat is None and pose.lon is None and not pose.position_valid
    assert pose.speed_mps == 0.0 and pose.course_deg is None


def test_position_recovers_after_an_outage():
    sim = PositionSim(seed=14, outage=(10.0, 20.0))
    for _ in range(int(30 / IMU_DT)):
        pose = sim.step()
    assert pose.status == EstimateStatus.OK
    assert math.hypot(*sim.error(pose)) < 0.3


def test_one_hertz_echomap_does_not_count_as_dead_reckoning():
    """gnss.FIX_TIMEOUT_S gives the echoMAP 2.5 s; the estimator reuses it
    instead of inventing its own 1 s rule that would flag every echoMAP fix."""
    sim = PositionSim(seed=15, source="echomap", quality=GnssQuality.FIX_3D, sigma=2.5, fix_period=1.0)
    statuses = []
    for _ in range(int(30 / IMU_DT)):
        pose = sim.step()
        if sim.t > 5:
            statuses.append(pose.status)
    assert set(statuses) == {EstimateStatus.OK}


def test_claimed_accuracy_is_floored_by_fix_quality():
    est = Estimator()
    pose = est.update(0.0, [], gnss_fix(0.0, 0.0, 0.0, quality=GnssQuality.FIX_3D, h_acc=0.01))
    assert pose.position_sigma_m == pytest.approx(2.5)
    est = Estimator()
    pose = est.update(0.0, [], gnss_fix(0.0, 0.0, 0.0, quality=GnssQuality.RTK_FIXED, h_acc=None))
    assert pose.position_sigma_m == pytest.approx(0.03)


def test_the_same_fix_object_is_applied_once():
    est = Estimator()
    fix = gnss_fix(1.0, 0.0, 0.0)
    for k in range(10):
        est.update(1.0 + 0.05 * k, [], fix)
    assert est.position.accepted == 1 and est.position.rejected == 0


def test_no_fix_quality_and_out_of_range_fixes_are_ignored():
    est = Estimator()
    est.update(0.0, [], gnss_fix(0.0, 0.0, 0.0))
    est.update(1.0, [], gnss_fix(1.0, 1.0, 0.0, quality=GnssQuality.NO_FIX))
    assert est.position.accepted == 1
    far = gnss_fix(1.2, 30_000.0, 0.0)
    est.update(1.2, [], far)
    assert est.diag.frame_exceeded and est.position.accepted == 1


def test_position_covariance_stays_positive_definite():
    sim = PositionSim(seed=16)
    for _ in range(int(600 / IMU_DT)):
        sim.step()
    f = sim.est.position
    assert f.ppp > 0 and f.pvv > 0 and f.ppp * f.pvv - f.ppv ** 2 > 0


def test_the_local_frame_origin_is_the_first_fix():
    est = Estimator()
    est.update(0.0, [], gnss_fix(0.0, 12.0, -7.0))
    pose = est.update(0.0, [])
    assert (pose.east_m, pose.north_m) == pytest.approx((0.0, 0.0), abs=1e-6)
    expected = geo.LocalFrame(*ORIGIN).to_geodetic(12.0, -7.0)
    assert geo.distance_m(*expected, pose.lat, pose.lon) < 1e-3
    # later positions are relative to that first fix, in metres east/north
    est.update(10.0, [], gnss_fix(10.0, 12.0 + 3.0, -7.0 + 4.0, h_acc=0.001))
    assert math.hypot(est.position.e, est.position.n) == pytest.approx(5.0, abs=1.5)


def test_the_filters_do_not_share_state_between_instances():
    a, b = HeadingFilter(CFG), PositionFilter(CFG)
    a.update(0.0, 1.0, 0.01)
    assert not b.initialized and a.initialized
