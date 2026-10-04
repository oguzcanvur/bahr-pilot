"""Single-beam echo sounder readings: range check, spike rejection, quality (Phase 17).

The echoMAP sends a depth about once a second. Single-beam sounders produce a
characteristic set of bad readings, and every one of them ends up on the survey
map as a false peak or hole if it is passed on:

  * no bottom lock: the unit reports 0 or its maximum range;
  * spikes: a fish, a weed bed, bubbles under the transducer, an electrical glitch;
  * a few NOISY readings in a row (aerated water, a turn).

What this module does with each reading:

  1. RANGE   outside RNGFND1_MIN..RNGFND1_MAX (ArduPilot's rangefinder window): BAD.
  2. SPIKE   compared with where the recent accepted readings say the bottom should be
             NOW. Two predictions are made and the reading passes if it is within
             tolerance of EITHER: a straight line through the last five accepted
             readings (Theil-Sen, robust to one bad point; the settled trend) and, where
             a real slope exists (> 0.25 m/s), the line through the last two (the quick
             one, which follows a slope that is itself changing). The tolerance is max(SONAR_SPIKE, 5 % of the depth).
             Both are needed, found with a simulated survey: a median-only filter
             rejected an entire steady bank (0.63 m deeper every second), and a
             five-point trend alone lost track when the boat ACCELERATED onto the
             bank (the slope was changing faster than five points could follow),
             then rejected every reading for 20 m.
  3. STEP    ...unless it is not a spike but the bottom really changed: when 3
             consecutive rejected readings are consistent with each other (the same
             level, or one straight line, as when a slope begins), the new
             behaviour is adopted. The first reading that confirms it is SUSPECT,
             the ones after it GOOD. (The two readings before the confirmation were
             already reported BAD and are not revised: honest about what was known.)
  4. LOST    a filter that has rejected 6 readings in a row has lost track whatever
             the reason: it starts again (SUSPECT, "resynchronised") instead of
             rejecting for as long as the situation lasts.
  5. NOISE   even when accepted, a reading is SUSPECT while the recent readings scatter
             about their predicted values by more than 0.4 m (MAD-based sigma over the
             last 10 innovations; five points are too few to judge scatter).

The trend is extrapolated at most 3 s past the last accepted reading: a slope measured
from a few readings carries noise, and multiplying it by a long silence would make the
prediction worse than none. The first readings after start or after a gap (5 s) have no
history to be judged against: they are SUSPECT, and flagged `warming_up`, until three
have been accepted.

`depth_m` is the raw reading plus RNGFND1_OFFSET (the transducer's depth below the
waterline), unsmoothed - smoothing would blur the terrain the survey is for. It is
None for BAD readings. The reading's quality is what downstream QC (bathymetry.py)
builds on.
"""
from __future__ import annotations

import enum
import math
import statistics
from collections import deque
from dataclasses import dataclass


class DepthQuality(enum.IntEnum):
    BAD = 0         # rejected: not used
    SUSPECT = 1     # plausible but not confirmed
    GOOD = 2


@dataclass(frozen=True)
class SonarConfig:
    min_depth_m: float = 0.5            # RNGFND1_MIN
    max_depth_m: float = 100.0          # RNGFND1_MAX
    offset_m: float = 0.0               # RNGFND1_OFFSET: added to the reading (transducer draft)
    spike_abs_m: float = 0.5            # SONAR_SPIKE
    spike_rel: float = 0.05
    window: int = 5
    confirm: int = 3                    # consistent outliers needed to accept a new behaviour
    max_step_m: float = 1.5             # how far the 2nd of a run of outliers may be from the 1st (slope onset)
    resync_after: int = 6               # consecutive rejections after which the filter starts over
    quick_min_slope_mps: float = 0.25   # below this the two-point line is resolution flicker, not a slope
    noise_suspect_m: float = 0.4
    min_history: int = 3                # accepted readings before the filter trusts its own trend
    gap_s: float = 5.0                  # a silence longer than this forgets the history
    max_extrapolate_s: float = 3.0
    noise_window: int = 10
    min_noise_samples: int = 5


@dataclass(frozen=True)
class DepthReading:
    t: float
    raw_m: float
    depth_m: float | None               # raw + offset; None when BAD
    quality: DepthQuality
    reason: str = ""
    noise_m: float | None = None        # scatter of the recent readings about their prediction (sigma estimate)
    warming_up: bool = False            # accepted without history to judge it by (start, gap, resync)


def _trend(points: list[tuple[float, float]]) -> tuple[float, float]:
    """Theil-Sen line through (t, depth) points: (slope m/s, intercept at t = 0).
    One wild point among five does not move it."""
    slopes = [(v2 - v1) / (t2 - t1) for i, (t1, v1) in enumerate(points) for t2, v2 in points[i + 1:] if t2 > t1]
    slope = statistics.median(slopes) if slopes else 0.0
    intercept = statistics.median(v - slope * t for t, v in points)
    return slope, intercept


class SonarFilter:
    def __init__(self, config: SonarConfig | None = None) -> None:
        self.cfg = config or SonarConfig()
        self.reset()

    def reset(self) -> None:
        self._accepted: deque[tuple[float, float]] = deque(maxlen=self.cfg.window)    # (t, depth)
        self._outliers: list[tuple[float, float]] = []
        self._innovations: deque[float] = deque(maxlen=self.cfg.noise_window)   # reading minus prediction
        self._reject_streak = 0
        self._last_t: float | None = None
        self.counts = {q: 0 for q in DepthQuality}

    def _bad(self, t: float, raw: float, reason: str) -> DepthReading:
        self.counts[DepthQuality.BAD] += 1
        return DepthReading(t, raw, None, DepthQuality.BAD, reason)

    def _tolerance(self, depth: float) -> float:
        return max(self.cfg.spike_abs_m, self.cfg.spike_rel * abs(depth))

    def _outlier_is_consistent(self, t: float, raw: float) -> bool:
        """Does this rejected reading go with the rejected ones before it? The second may be
        within a step of the first (a slope has begun and we cannot yet know its rate); from the
        third on it must lie on the straight line through the previous two."""
        outliers = self._outliers
        if not outliers:
            return True
        if len(outliers) == 1:
            return abs(raw - outliers[0][1]) <= self._tolerance(outliers[0][1]) + self.cfg.max_step_m
        (t1, v1), (t2, v2) = outliers[-2], outliers[-1]
        slope = (v2 - v1) / (t2 - t1) if t2 > t1 else 0.0
        return abs(raw - (v2 + slope * (t - t2))) <= self._tolerance(v2)

    def update(self, t: float, raw_m: float) -> DepthReading:
        cfg = self.cfg
        if self._last_t is not None and t - self._last_t > cfg.gap_s:
            self.reset()
        if self._last_t is None or t > self._last_t:
            self._last_t = t
        if isinstance(raw_m, bool) or not isinstance(raw_m, (int, float)) or not math.isfinite(raw_m):
            return self._bad(t, math.nan, "not a number")
        raw = float(raw_m)
        if raw < cfg.min_depth_m:
            return self._bad(t, raw, f"below minimum depth {cfg.min_depth_m:g} m")
        if raw > cfg.max_depth_m:
            return self._bad(t, raw, f"beyond maximum depth {cfg.max_depth_m:g} m")

        reason = ""
        quality = DepthQuality.GOOD
        warming_up = False
        step_confirmed = False
        expected = None
        if len(self._accepted) >= cfg.min_history:
            points = list(self._accepted)
            slope, intercept = _trend(points)
            t_eff = min(t, points[-1][0] + cfg.max_extrapolate_s)
            expected = intercept + slope * t_eff
            (t1, v1), (t2, v2) = points[-2], points[-1]
            quick_slope = (v2 - v1) / (t2 - t1) if t2 > t1 else 0.0
            deviation = abs(raw - expected)
            if abs(quick_slope) > cfg.quick_min_slope_mps:
                # a real slope is present: also accept what the last two readings predict. (On a
                # flat bottom two readings 0.1 m apart give a 'slope' of 0.1 m/s that is only
                # resolution flicker, and must not widen the tolerance.)
                deviation = min(deviation, abs(raw - (v2 + quick_slope * (t_eff - t2))))
            if deviation > self._tolerance(expected):
                if not self._outlier_is_consistent(t, raw):
                    self._outliers = []                    # the rejected readings do not go together
                self._outliers.append((t, raw))
                self._reject_streak += 1
                if self._reject_streak >= cfg.resync_after:
                    # lost track, for whatever reason: start again from here
                    self._accepted.clear()
                    self._outliers = []
                    self._innovations.clear()
                    self._reject_streak = 0
                    quality, reason, warming_up = DepthQuality.SUSPECT, "resynchronised", True
                elif len(self._outliers) < cfg.confirm:
                    return self._bad(t, raw, f"spike: {deviation:.2f} m from the expected {expected:.2f} m")
                else:
                    # a new, consistent behaviour: the bottom really changed. The window becomes
                    # the agreeing readings (the current one is the last of them, already in).
                    self._accepted.clear()
                    self._accepted.extend(self._outliers)
                    self._outliers = []
                    self._innovations.clear()
                    self._reject_streak = 0
                    step_confirmed = True
                    quality, reason = DepthQuality.SUSPECT, "bottom step confirmed"
            else:
                self._outliers = []
                self._reject_streak = 0
                self._innovations.append(raw - expected)
        else:
            quality, reason, warming_up = DepthQuality.SUSPECT, "not enough history yet", True
        if not step_confirmed:
            self._accepted.append((t, raw))

        noise = None
        if len(self._innovations) >= cfg.min_noise_samples:
            centre = statistics.median(self._innovations)
            noise = 1.4826 * statistics.median(abs(v - centre) for v in self._innovations)
            if quality is DepthQuality.GOOD and noise > cfg.noise_suspect_m:
                quality, reason = DepthQuality.SUSPECT, f"noisy: sigma {noise:.2f} m"
        self.counts[quality] += 1
        return DepthReading(t, raw, raw + cfg.offset_m, quality, reason, noise, warming_up)
