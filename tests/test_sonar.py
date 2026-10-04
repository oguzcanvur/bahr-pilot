"""Phase 17: the echo-sounder filter (bahr_pilot/sonar.py)."""
from __future__ import annotations

import math
import random

import pytest

from bahr_pilot.sonar import DepthQuality, SonarConfig, SonarFilter

BAD, SUSPECT, GOOD = DepthQuality.BAD, DepthQuality.SUSPECT, DepthQuality.GOOD


def feed(sonar, values, start=0.0, step=1.0):
    return [sonar.update(start + k * step, v) for k, v in enumerate(values)]


def qualities(readings):
    return [r.quality for r in readings]


def warmed(config=None, level=5.0):
    """A filter that has seen a steady bottom long enough to trust its median."""
    sonar = SonarFilter(config)
    feed(sonar, [level, level + 0.1, level, level + 0.1, level, level + 0.1])
    return sonar


# -- the range window ----------------------------------------------------------------------------

@pytest.mark.parametrize("raw,fragment", [(0.0, "below minimum"), (0.49, "below minimum"), (-3.0, "below minimum"),
                                          (100.01, "beyond maximum"), (900.0, "beyond maximum")])
def test_readings_outside_the_range_are_bad(raw, fragment):
    reading = SonarFilter().update(0.0, raw)
    assert reading.quality == BAD and reading.depth_m is None and fragment in reading.reason


@pytest.mark.parametrize("raw", [float("nan"), float("inf"), -float("inf"), None, "5.0", True, [5.0]])
def test_garbage_is_bad_and_does_not_crash(raw):
    reading = SonarFilter().update(0.0, raw)
    assert reading.quality == BAD and reading.depth_m is None


def test_the_range_is_configurable_like_rngfnd1_min_and_max():
    sonar = SonarFilter(SonarConfig(min_depth_m=2.0, max_depth_m=30.0))
    assert sonar.update(0.0, 1.9).quality == BAD
    assert sonar.update(1.0, 2.0).quality != BAD
    assert sonar.update(2.0, 30.1).quality == BAD


def test_a_range_reject_does_not_disturb_the_history():
    sonar = warmed()
    sonar.update(10.0, 0.0)                                         # no bottom lock
    assert sonar.update(11.0, 5.0).quality == GOOD


# -- history, spikes and steps -----------------------------------------------------------------------------

def test_the_first_readings_are_suspect_until_three_have_been_accepted():
    readings = feed(SonarFilter(), [5.0, 5.1, 5.0, 5.1, 5.0])
    assert qualities(readings) == [SUSPECT, SUSPECT, SUSPECT, GOOD, GOOD]
    assert readings[0].reason == "not enough history yet"


def test_a_spike_in_either_direction_is_rejected_with_the_reason():
    sonar = warmed()
    up = sonar.update(10.0, 12.0)
    down = sonar.update(11.0, 1.0)
    assert up.quality == BAD and "spike" in up.reason and up.depth_m is None
    assert down.quality == BAD and "spike" in down.reason


def test_a_spike_leaves_no_trace_in_the_readings_after_it():
    sonar = warmed()
    sonar.update(10.0, 12.0)
    after = sonar.update(11.0, 5.1)
    assert after.quality == GOOD and after.depth_m == 5.1


def test_the_tolerance_is_the_larger_of_an_absolute_and_a_relative_limit():
    sonar = warmed(level=50.0)                                        # tolerance = max(0.5, 5 % of ~50 = 2.5 m)
    assert sonar.update(6.0, 52.0).quality != BAD                     # 2 m off at 50 m depth: inside 2.5 m
    sonar = warmed(level=50.0)
    assert sonar.update(6.0, 54.0).quality == BAD
    shallow = warmed(level=2.0)                                       # tolerance = max(0.5, 0.1) = 0.5 m
    assert shallow.update(6.0, 2.7).quality == BAD and shallow.update(7.0, 2.4).quality == GOOD


def test_a_steady_slope_is_not_mistaken_for_spikes():
    """The failure that led to the trend-based prediction: a boat climbing a 25 degree
    bank at 1.5 m/s sees the depth change by 0.63 m every second. A median-only filter
    rejected EVERY reading of such a slope (each differs from the median by more than
    the tolerance, and consecutive outliers do not 'agree')."""
    sonar = SonarFilter()
    rate = 1.5 * math.tan(math.radians(25.0))                          # 0.70 m/s of depth change
    readings = [sonar.update(float(t), round(20.0 - rate * t, 1)) for t in range(25)]
    assert all(r.quality != BAD for r in readings)
    assert all(r.quality == GOOD for r in readings[3:])
    assert [r.depth_m for r in readings] == [round(20.0 - rate * t, 1) for t in range(25)]   # passed on untouched


def test_a_spike_on_a_slope_is_still_caught():
    sonar = SonarFilter()
    rate = 0.7
    out = [sonar.update(float(t), round(20.0 - rate * t, 1) + (3.0 if t == 12 else 0.0)) for t in range(25)]
    assert [i for i, r in enumerate(out) if r.quality == BAD] == [12]


def test_the_slope_is_not_assumed_to_continue_across_a_turn():
    """A slope that stops and reverses (a ridge) is relearned within three readings."""
    sonar = SonarFilter()
    values = [round(10.0 + 0.5 * t, 1) for t in range(10)] + [round(14.5 - 0.5 * (t - 9), 1) for t in range(10, 20)]
    out = [sonar.update(float(t), v) for t, v in enumerate(values)]
    # a reversal looks like a step to the filter: two readings rejected, the third adopts the
    # new trend (the raw readings stay in the log with their flags: nothing is lost for review)
    assert [i for i, r in enumerate(out) if r.quality == BAD] == [10, 11]
    assert out[12].quality == SUSPECT and out[12].reason == "bottom step confirmed"
    assert all(r.quality == GOOD for r in out[13:])


def test_a_bottom_step_is_adopted_after_three_agreeing_readings():
    """The two readings before the confirmation stay BAD: they were rejected when they
    arrived and are not revised afterwards."""
    sonar = warmed(level=5.0)
    steps = feed(sonar, [8.0, 8.1, 8.0, 8.1, 8.0], start=10.0)
    assert qualities(steps) == [BAD, BAD, SUSPECT, GOOD, GOOD]
    assert steps[2].reason == "bottom step confirmed" and steps[2].depth_m == 8.0


def test_a_step_back_up_works_the_same_way():
    sonar = warmed(level=8.0)
    steps = feed(sonar, [4.0, 4.1, 4.0, 4.1], start=10.0)
    assert qualities(steps) == [BAD, BAD, SUSPECT, GOOD]


def test_disagreeing_outliers_never_make_a_step():
    sonar = warmed(level=5.0)
    steps = feed(sonar, [8.0, 12.0, 8.0, 12.0, 8.0], start=10.0)
    assert qualities(steps) == [BAD] * 5
    assert sonar.update(15.0, 5.1).quality == GOOD                    # still anchored at the old level


def test_two_agreeing_outliers_are_not_enough():
    sonar = warmed(level=5.0)
    steps = feed(sonar, [8.0, 8.1, 5.0], start=10.0)
    assert qualities(steps) == [BAD, BAD, GOOD]


def test_a_noisy_stretch_is_suspect_even_when_every_reading_is_inside_the_tolerance():
    """Aerated water: scatter of ~0.8 m. The tolerance is widened so nothing is rejected as a
    spike; the stretch must still not be reported as confirmed data."""
    rng = random.Random(4)
    sonar = SonarFilter(SonarConfig(spike_abs_m=5.0))
    readings = [sonar.update(float(t), round(10.0 + rng.gauss(0.0, 0.8), 1)) for t in range(40)]
    assert sum(r.quality == BAD for r in readings) == 0
    late = readings[10:]
    assert sum(r.quality == SUSPECT for r in late) / len(late) > 0.7
    assert all("noisy" in r.reason for r in late if r.quality == SUSPECT)
    assert statistics_median([r.noise_m for r in late]) == pytest.approx(0.8, abs=0.35)


def statistics_median(values):
    import statistics
    return statistics.median(values)


def test_a_quiet_bottom_reports_a_small_noise_figure():
    readings = feed(warmed(), [5.0, 5.1, 5.0, 5.1])
    assert readings[-1].quality == GOOD and readings[-1].noise_m < 0.2


# -- what comes out ---------------------------------------------------------------------------------------------------------

def test_the_depth_is_the_raw_reading_plus_the_offset_and_is_not_smoothed():
    sonar = SonarFilter(SonarConfig(offset_m=0.3))
    readings = feed(sonar, [5.0, 5.1, 5.0, 5.1, 5.3, 5.0])
    assert [r.depth_m for r in readings] == pytest.approx([5.3, 5.4, 5.3, 5.4, 5.6, 5.3])
    assert [r.raw_m for r in readings] == [5.0, 5.1, 5.0, 5.1, 5.3, 5.0]


def test_the_offset_does_not_move_the_range_window():
    sonar = SonarFilter(SonarConfig(min_depth_m=0.5, offset_m=1.0))
    assert sonar.update(0.0, 0.4).quality == BAD                        # the RAW reading is range-checked


def test_a_long_gap_forgets_the_history():
    sonar = warmed(level=5.0)
    after = sonar.update(500.0, 20.0)                                   # 8 minutes later, somewhere else entirely
    assert after.quality == SUSPECT and after.reason == "not enough history yet"


def test_a_short_gap_keeps_it():
    sonar = warmed(level=5.0)
    assert sonar.update(8.0, 20.0).quality == BAD                       # 3 s on: still the same patch of water


def test_the_trend_is_not_extrapolated_for_ever():
    """A slope of 0.7 m/s measured over five readings is only trusted 3 s ahead: a reading
    5 s later is judged against the value 3 s out, not 5 s out, so slope noise cannot grow."""
    sonar = SonarFilter()
    for t in range(6):
        sonar.update(float(t), round(10.0 + 0.7 * t, 1))
    assert sonar.update(8.0, round(10.0 + 0.7 * 8, 1)).quality == GOOD       # 3 s on: the full slope applies
    sonar = SonarFilter()
    for t in range(6):
        sonar.update(float(t), round(10.0 + 0.7 * t, 1))
    late = sonar.update(10.0, round(10.0 + 0.7 * 10, 1))                      # 5 s on: judged against 3 s of slope
    assert late.quality == BAD and "spike" in late.reason


def test_reset_and_the_quality_tally():
    sonar = SonarFilter()
    feed(sonar, [5.0, 5.1, 5.0, 5.1, 0.0, 5.0])
    assert sonar.counts == {BAD: 1, SUSPECT: 3, GOOD: 2}
    sonar.reset()
    assert sonar.counts == {BAD: 0, SUSPECT: 0, GOOD: 0}
    assert sonar.update(0.0, 5.0).reason == "not enough history yet"


def test_out_of_order_timestamps_do_not_crash_or_reset():
    sonar = warmed()
    assert sonar.update(3.0, 5.0).quality == GOOD                       # earlier than the last reading


# -- against a simulated sounder ----------------------------------------------------------------------------------------------

def simulate(seconds, truth, *, seed=1, sigma=0.05, spike_rate=0.08, zero_rate=0.03):
    """1 Hz readings of `truth(t)` with 0.1 m resolution, gaussian noise, random spikes
    (1-5 m off, either way) and no-lock zeros. Returns (readings, truths, was_spike)."""
    rng = random.Random(seed)
    sonar = SonarFilter()
    readings, truths, spikes = [], [], []
    for t in range(seconds):
        true = truth(t)
        raw = round(true + rng.gauss(0.0, sigma), 1)
        spiked = False
        roll = rng.random()
        if roll < zero_rate:
            raw = 0.0
        elif roll < zero_rate + spike_rate:
            raw = round(true + rng.choice((-1, 1)) * rng.uniform(1.0, 5.0), 1)
            spiked = True
        readings.append(sonar.update(float(t), max(raw, 0.0)))
        truths.append(true)
        spikes.append(spiked or raw == 0.0)
    return readings, truths, spikes


def test_spikes_and_dropouts_are_removed_from_a_sloping_bottom():
    readings, truths, bad_input = simulate(600, lambda t: 8.0 + 0.01 * t)
    accepted_spikes = sum(1 for r, b in zip(readings, bad_input) if b and r.quality != BAD)
    rejected_good = sum(1 for r, b in zip(readings, bad_input) if not b and r.quality == BAD)
    spikes = sum(bad_input)
    errors = [r.depth_m - truth for r, truth in zip(readings, truths) if r.quality != BAD]
    assert spikes > 40
    assert accepted_spikes / spikes < 0.05                               # measured: a few warm-up readings at most
    assert rejected_good / (len(readings) - spikes) < 0.03
    assert math.sqrt(sum(e * e for e in errors) / len(errors)) < 0.15    # what is passed on is close to the truth


def test_a_real_drop_off_is_followed_within_three_readings():
    """5 m -> 9 m at t = 100 on a clean sounder: the new level is GOOD by the 4th reading."""
    sonar = SonarFilter()
    out = []
    for t in range(140):
        out.append(sonar.update(float(t), 5.0 + (t % 2) * 0.1 if t < 100 else 9.0 + (t % 2) * 0.1))
    assert [r.quality for r in out[100:104]] == [BAD, BAD, SUSPECT, GOOD]
    assert all(r.quality == GOOD for r in out[104:])
    assert all(abs(r.depth_m - 9.05) < 0.1 for r in out[102:])


def test_the_filter_is_deterministic():
    a = simulate(200, lambda t: 6.0, seed=3)[0]
    b = simulate(200, lambda t: 6.0, seed=3)[0]
    assert [(r.quality, r.depth_m) for r in a] == [(r.quality, r.depth_m) for r in b]
