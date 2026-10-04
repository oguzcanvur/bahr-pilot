"""StmClock: unwrapping, drift tracking and latency rejection, against
synthetic data with a known truth."""
from __future__ import annotations

import random

import pytest

from bahr_pilot.timesync import StmClock

WRAP = 1 << 32


def run(clock, seconds, *, stm_rate=1.0, stm_start_us=0, rate_hz=50, latency=(0.003, 0.023),
        seed=1, pi_start=1000.0):
    """Feeds `seconds` of frames. True time t (seconds since start); the STM
    counts stm_rate * t microseconds; arrival = pi_start + t + latency.
    Returns a list of (true_pi_time, estimated_pi_time)."""
    rng = random.Random(seed)
    out = []
    n = int(seconds * rate_hz)
    for i in range(n):
        t = i / rate_hz
        raw = int(stm_start_us + stm_rate * t * 1e6) % WRAP
        rx = pi_start + t + rng.uniform(*latency)
        est = clock.update(raw, rx)
        out.append((pi_start + t, est))
    return out


def tail_errors(samples, seconds):
    return [est - true for true, est in samples[-int(seconds * 50):]]


def test_ideal_clock_converges_to_the_minimum_latency():
    clock = StmClock()
    samples = run(clock, 20)
    errors = tail_errors(samples, 5)
    # the estimate lags the truth by (about) the smallest latency ever seen
    assert all(0.0015 < e < 0.0045 for e in errors), (min(errors), max(errors))
    assert abs(clock.drift_ppm) < 500


def test_scheduling_hiccups_do_not_move_the_estimate():
    """Occasional 200 ms late frames must be ignored, not averaged in."""
    clock = StmClock()
    rng = random.Random(7)
    pi_start = 50.0
    estimates = []
    for i in range(50 * 30):
        t = i / 50
        latency = 0.003 + (0.2 if rng.random() < 0.05 else rng.uniform(0, 0.01))
        estimates.append(clock.update(int(t * 1e6), pi_start + t + latency) - (pi_start + t))
    assert max(abs(e - 0.003) for e in estimates[-250:]) < 0.004


@pytest.mark.parametrize("rate", [1.01, 0.99, 1.0005])
def test_tracks_a_drifting_stm_clock(rate):
    """HSI is only good to ~1 %: seconds on the STM are 0.99-1.01 real ones."""
    clock = StmClock()
    samples = run(clock, 60, stm_rate=rate)
    errors = tail_errors(samples, 5)
    assert all(abs(e - 0.003) < 0.003 for e in errors), (min(errors), max(errors))
    # STM runs at `rate` x real time, so the Pi/STM tick ratio is 1/rate
    assert clock.drift_ppm == pytest.approx((1.0 / rate - 1.0) * 1e6, rel=0.2, abs=300)


def test_without_drift_tracking_a_one_percent_clock_would_be_far_off():
    """The reason the fit exists: 1 % over 60 s is 600 ms of error."""
    stm_seconds_at_60s = 60 * 1.01
    assert abs(stm_seconds_at_60s - 60) > 0.5


def test_continuous_across_the_32_bit_wrap():
    clock = StmClock()
    start = WRAP - 5_000_000  # 5 s before the wrap
    samples = run(clock, 20, stm_start_us=start)
    errors = [est - true for true, est in samples]
    assert max(errors[-200:]) - min(errors[-200:]) < 0.006
    estimates = [est for _, est in samples]
    assert all(b >= a - 0.02 for a, b in zip(estimates, estimates[1:]))  # no jump back at the wrap


def test_stm_reboot_resets_the_estimator_and_it_reconverges():
    clock = StmClock()
    run(clock, 30, stm_start_us=500_000_000)       # STM has been up for ~8 minutes
    assert clock.resets == 0
    # reboot: the STM clock restarts near zero while the Pi keeps going
    rng = random.Random(3)
    last = None
    for i in range(50 * 20):
        t = i / 50
        last = clock.update(int(t * 1e6), 5000.0 + t + rng.uniform(0.003, 0.02))
    assert clock.resets == 1
    assert abs(last - (5000.0 + 19.98)) < 0.006


def test_first_frames_work_before_enough_data_for_a_fit():
    clock = StmClock()
    est = clock.update(123_456, 1000.0 + 0.123456 + 0.004)
    assert est == pytest.approx(1000.0 + 0.123456 + 0.004)
    assert not clock.ready


def test_absurd_drift_is_clamped():
    clock = StmClock()
    # an STM running at twice the speed cannot be a crystal; the fit must not
    # follow it off a cliff
    run(clock, 30, stm_rate=2.0)
    assert abs(clock.drift_ppm) <= 50_000 + 1
