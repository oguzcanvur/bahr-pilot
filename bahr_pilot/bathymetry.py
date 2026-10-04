"""Bathymetry samples: position + depth, sampled by distance, quality-classified
(Phases 18 and 19).

Phase 17 (sonar.py) says how trustworthy a DEPTH READING is. This turns readings
into the product of the survey, a sample = where + how deep + how much to trust it:

  SAMPLING (18)  A sample is produced per distance travelled, not per second: a boat
                 that stops or crawls must not pile a hundred soundings on one spot,
                 and one that runs fast must not leave gaps. A sample is emitted
                 when the boat is `spacing_m` from the last one that counted;
                 stationary (below `min_speed_mps`) it emits nothing. Rejected
                 readings are emitted too (as INVALID, with the reason) and do not
                 reset the spacing, so the log shows WHY there is a hole instead of
                 just a hole.
                 The position is where the boat WAS when the echo was taken: the
                 estimated position moved back along the velocity by the age of the
                 reading (how long ago it arrived plus the sounder's own latency,
                 SONAR_LATENCY, which has not been measured).

  QUALITY (19)   VALID       use it
                 LOW_QUALITY plausible, but something about it is doubtful: a
                             SUSPECT sonar reading, a position known only to
                             within BATHY_MAX_HACC, dead-reckoned position, boat
                             too fast, boat heeled over more than BATHY_MAX_TILT
                 INVALID     do not use: BAD sonar reading, no position
                 Every reason is recorded, not just the first.

Both the corrected depth (below the waterline, with RNGFND1_OFFSET) and the raw
reading are kept, with the positioning quality, so a later processing step (tide,
sound velocity, a different threshold) can redo the classification.

Not done here: tide and sound-velocity corrections (post-processing), beam
footprint and tilt geometry beyond the flag, uncertainty propagation (TPU).
"""
from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import Sequence

from bahr_pilot import geo
from bahr_pilot.estimator import EstimateStatus, Pose
from bahr_pilot.sonar import DepthQuality, DepthReading


class SampleQuality(enum.IntEnum):
    INVALID = 0
    LOW_QUALITY = 1
    VALID = 2


@dataclass(frozen=True)
class BathyConfig:
    spacing_m: float = 1.0              # BATHY_SPACING
    min_speed_mps: float = 0.2          # slower than this is "stationary": no samples
    max_h_acc_m: float = 0.5            # BATHY_MAX_HACC
    max_speed_mps: float = 3.0          # BATHY_MAX_SPEED
    max_tilt_deg: float = 15.0          # BATHY_MAX_TILT
    latency_s: float = 0.0              # SONAR_LATENCY (unmeasured)


@dataclass(frozen=True)
class BathySample:
    seq: int
    t: float                            # monotonic seconds of the reading
    utc: float                          # wall-clock seconds, for tide correction later
    lat: float | None
    lon: float | None
    depth_m: float | None               # below the waterline (raw + offset); None if the reading was rejected
    raw_depth_m: float
    quality: SampleQuality
    reasons: tuple[str, ...]
    sonar_quality: DepthQuality
    gnss_quality: int                   # gnss.GnssQuality of the raw fix
    h_acc_m: float | None               # the estimator's 1-sigma position accuracy
    speed_mps: float
    heading_deg: float | None
    tilt_deg: float | None
    along_track_m: float                # distance travelled since the previous COUNTED sample


class BathymetryRecorder:
    def __init__(self, config: BathyConfig | None = None) -> None:
        self.cfg = config or BathyConfig()
        self.reset()

    def reset(self) -> None:
        self._last_counted: tuple[float, float] | None = None      # lat, lon of the last non-INVALID sample
        self._seq = 0
        self.counts = {q: 0 for q in SampleQuality}
        self.skipped_stationary = 0
        self.skipped_close = 0

    # -- the position of the echo ---------------------------------------------------------------

    def _position_of_the_echo(self, pose: Pose, age_s: float) -> tuple[float, float]:
        """Where the boat was `age_s` ago: the estimate moved back along its velocity."""
        if age_s <= 0.0 or (pose.velocity_east_mps == 0.0 and pose.velocity_north_mps == 0.0):
            return pose.lat, pose.lon
        age = min(age_s, 5.0)                                       # never reach far back on a stale reading
        return geo.LocalFrame(pose.lat, pose.lon).to_geodetic(-pose.velocity_east_mps * age,
                                                              -pose.velocity_north_mps * age)

    # -- classification -----------------------------------------------------------------------------

    def classify(self, reading: DepthReading, pose: Pose | None) -> tuple[SampleQuality, tuple[str, ...]]:
        cfg = self.cfg
        invalid: list[str] = []
        low: list[str] = []
        if reading.quality == DepthQuality.BAD:
            invalid.append(f"sonar: {reading.reason}")
        elif reading.quality == DepthQuality.SUSPECT:
            low.append(f"sonar suspect: {reading.reason}")
        if pose is None or not pose.position_valid:
            invalid.append("no position")
        else:
            if pose.status == EstimateStatus.DEAD_RECKONING:
                low.append("position dead-reckoned")
            sigma = pose.position_sigma_m
            if sigma is None or sigma > cfg.max_h_acc_m:
                low.append("position accuracy " + ("unknown" if sigma is None else f"{sigma:.2f} m > {cfg.max_h_acc_m:g} m"))
            if pose.speed_mps > cfg.max_speed_mps:
                low.append(f"speed {pose.speed_mps:.1f} m/s > {cfg.max_speed_mps:g} m/s")
            tilt = _tilt_deg(pose)
            if tilt is not None and tilt > cfg.max_tilt_deg:
                low.append(f"tilt {tilt:.0f} deg > {cfg.max_tilt_deg:g} deg")
        if invalid:
            return SampleQuality.INVALID, tuple(invalid + low)
        if low:
            return SampleQuality.LOW_QUALITY, tuple(low)
        return SampleQuality.VALID, ()

    # -- the step ----------------------------------------------------------------------------------------

    def update(self, reading: DepthReading, pose: Pose | None, now: float, utc: float,
               gnss_quality: int = 0) -> BathySample | None:
        """One depth reading with the pose at the time it is processed. Returns the
        sample to record, or None when this reading is not due (stationary, or too
        close to the previous sample)."""
        cfg = self.cfg
        quality, reasons = self.classify(reading, pose)
        position = None
        if pose is not None and pose.position_valid:
            position = self._position_of_the_echo(pose, now - reading.t + cfg.latency_s)

        along = 0.0
        if position is not None and pose.speed_mps < cfg.min_speed_mps:
            # stationary: nothing is recorded, not even a rejected reading (a hole needs a track)
            self.skipped_stationary += 1
            return None
        if quality != SampleQuality.INVALID and position is not None:
            if self._last_counted is not None:
                along = geo.distance_m(*self._last_counted, *position)
                if along < cfg.spacing_m:
                    self.skipped_close += 1
                    return None
            self._last_counted = position

        self._seq += 1
        self.counts[quality] += 1
        return BathySample(
            seq=self._seq, t=reading.t, utc=utc,
            lat=None if position is None else position[0], lon=None if position is None else position[1],
            depth_m=reading.depth_m, raw_depth_m=reading.raw_m, quality=quality, reasons=reasons,
            sonar_quality=reading.quality, gnss_quality=gnss_quality,
            h_acc_m=None if pose is None else pose.position_sigma_m,
            speed_mps=0.0 if pose is None else pose.speed_mps,
            heading_deg=pose.heading_deg if pose is not None and pose.heading_valid else None,
            tilt_deg=None if pose is None else _tilt_deg(pose), along_track_m=along)


def _tilt_deg(pose: Pose) -> float | None:
    """Angle of the boat's vertical from the true vertical, or None if attitude is unknown."""
    if pose.roll_deg is None or pose.pitch_deg is None:
        return None
    r, p = math.radians(pose.roll_deg), math.radians(pose.pitch_deg)
    return math.degrees(math.acos(max(-1.0, min(1.0, math.cos(r) * math.cos(p)))))


def summarize(samples: Sequence[BathySample]) -> dict[str, int]:
    """Counts per quality, for the log and the operator."""
    out = {q.name: 0 for q in SampleQuality}
    for s in samples:
        out[s.quality.name] += 1
    return out
