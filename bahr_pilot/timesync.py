"""Maps the Nucleo's microsecond clock onto the Pi's monotonic clock.

Why this is not trivial (docs/ARCHITECTURE.md, Phase 3):
  * The STM's timestamp is 32 bits of microseconds and wraps every 71.6 min.
  * Its clock comes from the internal HSI oscillator (no crystal), good to
    about +-1 %: a "second" on the STM may be 0.99-1.01 real seconds. Assuming
    the two clocks tick at the same rate would smear every IMU sample by up to
    10 ms per second of uptime.
  * Frames reach the Pi over a UART read by a non-realtime process, so the
    arrival time is the true time plus a variable latency that is never
    negative.

The last point is the lever: for every sample, offset = arrival - stm_time is
the true clock offset plus a latency >= 0, so the *smallest* offsets are the
closest to the truth. Taking the minimum per one-second bucket, then fitting a
straight line through the bucket minima, gives both the offset and the drift
rate while ignoring scheduling hiccups.

Accuracy to expect: the estimate sits above the true offset by the smallest
latency the link ever shows (the UART transmit time of a frame, ~2-4 ms at
115200 baud, plus Python scheduling) — a constant, so intervals are right and
absolute times are late by that much. Whatever the estimator achieves, the
IMU's own timestamp quality is limited by its 20 ms poll period (see
firmware/reflex/Core/Inc/imu.h).
"""
from __future__ import annotations

from collections import deque

_WRAP = 1 << 32
_HALF = 1 << 31

BUCKET_S = 1.0
MAX_BUCKETS = 60
MIN_BUCKETS_FOR_FIT = 4
MAX_DRIFT = 0.05  # |rate| beyond this (5 %) is not a crystal, it is a bug


class StmClock:
    def __init__(self) -> None:
        self._reset()

    def _reset(self) -> None:
        self._last_raw: int | None = None
        self._epoch = 0
        # per bucket: (stm_seconds, offset) of that bucket's minimum offset
        self._buckets: deque[tuple[float, float]] = deque(maxlen=MAX_BUCKETS)
        self._bucket_id: int | None = None
        self._offset = 0.0
        self._drift = 0.0
        self._fit_origin = 0.0
        self.resets = 0

    # -- unwrapping ------------------------------------------------------

    def _unwrap(self, raw: int) -> float | None:
        """STM seconds as a float, continuing across 32-bit wraps; None if the
        STM restarted (time went backwards), in which case state was reset."""
        raw &= _WRAP - 1
        if self._last_raw is not None:
            forward = (raw - self._last_raw) % _WRAP
            if forward >= _HALF:  # a step "backwards": the STM rebooted
                self._reset()
                self.resets += 1
                return self._unwrap(raw)
            if raw < self._last_raw:
                self._epoch += 1
        self._last_raw = raw
        return (raw + self._epoch * _WRAP) * 1e-6

    # -- the estimator ---------------------------------------------------

    def update(self, raw_us: int, rx_time_s: float) -> float:
        """Feeds one frame (its 32-bit STM timestamp and the Pi monotonic
        time it arrived at); returns the estimated Pi monotonic time of the
        sample itself."""
        stm_s = self._unwrap(raw_us)
        sample_offset = rx_time_s - stm_s

        bucket = int(stm_s / BUCKET_S)
        if bucket != self._bucket_id:
            self._bucket_id = bucket
            self._buckets.append((stm_s, sample_offset))
        else:
            best_stm, best_offset = self._buckets[-1]
            if sample_offset < best_offset:
                self._buckets[-1] = (stm_s, sample_offset)

        self._fit()
        return self.to_pi_time(stm_s)

    def _fit(self) -> None:
        # The newest bucket is still filling: its minimum is taken over only
        # a few samples, so it reads high (a lucky-late sample) and would
        # drag the line up at exactly the end we extrapolate from. Fit only
        # completed buckets once there are enough of them.
        points = list(self._buckets)
        if len(points) > MIN_BUCKETS_FOR_FIT:
            points = points[:-1]
        if len(points) < MIN_BUCKETS_FOR_FIT:
            self._offset = min(offset for _, offset in points)
            self._drift = 0.0
            self._fit_origin = points[-1][0]
            return
        origin = points[-1][0]  # fit relative to "now" so the numbers stay small
        xs = [s - origin for s, _ in points]
        ys = [o for _, o in points]
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        var_x = sum((x - mean_x) ** 2 for x in xs)
        slope = (sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / var_x) if var_x else 0.0
        slope = max(-MAX_DRIFT, min(MAX_DRIFT, slope))
        self._drift = slope
        self._offset = mean_y - slope * mean_x
        self._fit_origin = origin

    def to_pi_time(self, stm_s: float) -> float:
        return stm_s + self._offset + self._drift * (stm_s - self._fit_origin)

    @property
    def drift_ppm(self) -> float:
        """Estimated clock-rate error of the STM relative to the Pi, in ppm
        (positive: the Pi clock runs faster, i.e. the STM's seconds are
        long). Only meaningful after a few seconds of data."""
        return self._drift * 1e6

    @property
    def ready(self) -> bool:
        return len(self._buckets) >= MIN_BUCKETS_FOR_FIT
