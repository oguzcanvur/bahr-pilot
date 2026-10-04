"""Mission data log (Phase 25): one folder per mission, plain files that open in
a spreadsheet, QGIS or a script.

    <base>/mission-20261004T142530/
        meta.json         what flew: software and protocol versions, every parameter (and which
                          differ from the defaults), the start time. Written when the mission starts.
        bathymetry.csv    EVERY sample the recorder produced, including the rejected ones with
                          their reasons: the survey product, and the audit trail of its holes.
        track.csv         where the boat was, 2 Hz: estimated position, speed, heading, accuracy.
        events.csv        everything the operator was told (failsafes, mode changes, arming,
                          warnings) with a timestamp and a severity.
        summary.json      written when the mission ends: duration, distance, sample counts per
                          quality, depth range of the usable samples, events by severity.

A mission is armed -> disarmed. Times are UTC seconds (`utc`), so a later tide or
sound-velocity correction can be applied against a tide gauge; the monotonic
`t` of a bathymetry sample ties it to the other logs of the same run.

Robust by construction, because the SD card of a boat will fill, glitch or be
pulled:
  * a row is flushed to the OS as soon as it is written (a power cut loses at most
    the row in flight) and everything is fsync'ed every few seconds and at the end;
  * ANY write error stops logging and is remembered (`error`), counted (`dropped`),
    and never raised: the vehicle loop must not die because the disk did;
  * the summary is rebuilt from the data written, so it is also correct after a crash if
    `summarize_folder` is run on a folder that has none.

This module only writes. Reading (and QGIS export) is left to ordinary tools: the files
are standard CSV/JSON.
"""
from __future__ import annotations

import csv
import json
import math
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping

from bahr_pilot import geo
from bahr_pilot.bathymetry import BathySample, SampleQuality
from bahr_pilot.estimator import Pose

FORMAT = 1
TRACK_HZ = 2.0
FSYNC_INTERVAL_S = 5.0
# The distance in the summary only advances when the boat has moved this far from the last
# counted point. Summing every 2 Hz step also sums the estimator's jitter: measured in the
# simulator, a 102 m run reported 104.7 m, the extra 2.7 m coming from 44 s spent stationary.
DISTANCE_MIN_STEP_M = 0.25

BATHY_COLUMNS = ("seq", "t", "utc", "lat", "lon", "depth_m", "raw_depth_m", "quality", "reasons", "sonar_quality",
                 "gnss_quality", "h_acc_m", "speed_mps", "heading_deg", "tilt_deg", "along_track_m")
TRACK_COLUMNS = ("utc", "lat", "lon", "speed_mps", "course_deg", "heading_deg", "heading_valid", "position_status",
                 "sigma_m", "roll_deg", "pitch_deg", "yaw_rate_dps", "mode")
EVENT_COLUMNS = ("utc", "level", "source", "text")

# MAVLink STATUSTEXT severities, named
SEVERITY_NAMES = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR", 4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}


def _fmt(value: Any, digits: int | None = None) -> str:
    """CSV cell: empty for unknown, fixed decimals for floats, 1/0 for booleans."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        if not math.isfinite(value):
            return ""
        return f"{value:.{digits}f}" if digits is not None else repr(value)
    return str(value)


class _Csv:
    def __init__(self, path: Path, columns: tuple[str, ...]) -> None:
        self.path = path
        self._file = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(columns)
        self._file.flush()

    def row(self, cells: list[str]) -> None:
        self._writer.writerow(cells)
        self._file.flush()

    def sync(self) -> None:
        self._file.flush()
        os.fsync(self._file.fileno())

    def close(self) -> None:
        try:
            self.sync()
        finally:
            self._file.close()


@dataclass
class _Tally:
    bathy: dict[str, int] = field(default_factory=lambda: {q.name: 0 for q in SampleQuality})
    depth_min: float | None = None
    depth_max: float | None = None
    track_points: int = 0
    distance_m: float = 0.0
    events: dict[str, int] = field(default_factory=dict)


class MissionLog:
    def __init__(self, base_dir: str | Path, clock: Callable[[], float] = time.time,
                 mono: Callable[[], float] = time.monotonic) -> None:
        self.base = Path(base_dir)
        self._clock = clock
        self._mono = mono
        self.folder: Path | None = None
        self.error: str | None = None            # the first write failure, which also stops logging
        self.dropped = 0                         # rows that could not be written
        self._files: dict[str, _Csv] = {}
        self._tally = _Tally()
        self._started_utc = 0.0
        self._last_track_t = -math.inf
        self._last_track_xy: tuple[float, float] | None = None
        self._last_sync = 0.0

    @property
    def active(self) -> bool:
        return self.folder is not None and self.error is None

    # -- lifecycle ------------------------------------------------------------------------------------

    def _new_folder(self) -> Path:
        stem = "mission-" + time.strftime("%Y%m%dT%H%M%S", time.gmtime(self._clock()))
        folder, n = self.base / stem, 1
        while folder.exists():                    # two missions in the same second
            n += 1
            folder = self.base / f"{stem}-{n}"
        folder.mkdir(parents=True)
        return folder

    def start(self, meta: Mapping[str, Any]) -> Path | None:
        """Open a new mission folder and write meta.json. Returns the folder, or None if the
        disk refused (see `error`)."""
        if self.folder is not None:
            self.stop("restarted")
        self.error, self.dropped = None, 0
        self._tally = _Tally()
        self._last_track_t, self._last_track_xy = -math.inf, None
        self._started_utc = self._clock()
        try:
            self.folder = self._new_folder()
            document = {"format": FORMAT, "started_utc": self._started_utc,
                        "started": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(self._started_utc)), **meta}
            (self.folder / "meta.json").write_text(json.dumps(document, indent=1, sort_keys=True, default=str),
                                                   encoding="utf-8")
            self._files = {"bathymetry": _Csv(self.folder / "bathymetry.csv", BATHY_COLUMNS),
                           "track": _Csv(self.folder / "track.csv", TRACK_COLUMNS),
                           "events": _Csv(self.folder / "events.csv", EVENT_COLUMNS)}
        except OSError as exc:
            self._fail(exc)
            return None
        self._last_sync = self._mono()
        return self.folder

    def stop(self, reason: str = "disarmed", extra: Mapping[str, Any] | None = None) -> Path | None:
        """Close the files and write summary.json. Safe to call twice or when nothing is open."""
        if self.folder is None:
            return None
        folder, self.folder = self.folder, None
        for log in self._files.values():
            try:
                log.close()
            except OSError as exc:
                self._fail(exc, stop_logging=False)
        self._files = {}
        t = self._tally
        summary = {"format": FORMAT, "ended_utc": self._clock(), "reason": reason,
                   "duration_s": round(self._clock() - self._started_utc, 1), "distance_m": round(t.distance_m, 1),
                   "track_points": t.track_points, "bathymetry": t.bathy,
                   "depth_min_m": t.depth_min, "depth_max_m": t.depth_max, "events": t.events,
                   "rows_dropped": self.dropped, "log_error": self.error, **(extra or {})}
        try:
            (folder / "summary.json").write_text(json.dumps(summary, indent=1, sort_keys=True), encoding="utf-8")
        except OSError as exc:
            self._fail(exc, stop_logging=False)
        return folder

    def _fail(self, exc: OSError, stop_logging: bool = True) -> None:
        if self.error is None:
            self.error = f"{type(exc).__name__}: {exc}"
        self.dropped += 1
        if stop_logging:
            for log in self._files.values():
                try:
                    log._file.close()
                except OSError:
                    pass
            self._files = {}

    def _write(self, kind: str, cells: list[str]) -> bool:
        log = self._files.get(kind)
        if log is None:
            if self.folder is not None:                  # logging was stopped by an error
                self.dropped += 1
            return False
        try:
            log.row(cells)
            if self._mono() - self._last_sync >= FSYNC_INTERVAL_S:
                for each in self._files.values():
                    each.sync()
                self._last_sync = self._mono()
        except OSError as exc:
            self._fail(exc)
            return False
        return True

    # -- what is written -------------------------------------------------------------------------------

    def bathymetry(self, s: BathySample) -> None:
        wrote = self._write("bathymetry", [
            str(s.seq), _fmt(s.t, 3), _fmt(s.utc, 3), _fmt(s.lat, 8), _fmt(s.lon, 8), _fmt(s.depth_m, 2),
            _fmt(s.raw_depth_m, 2), s.quality.name, "; ".join(s.reasons), s.sonar_quality.name, str(s.gnss_quality),
            _fmt(s.h_acc_m, 3), _fmt(s.speed_mps, 2), _fmt(s.heading_deg, 1), _fmt(s.tilt_deg, 1),
            _fmt(s.along_track_m, 2)])
        if not wrote:
            return
        self._tally.bathy[s.quality.name] += 1
        if s.quality != SampleQuality.INVALID and s.depth_m is not None:
            t = self._tally
            t.depth_min = s.depth_m if t.depth_min is None else min(t.depth_min, s.depth_m)
            t.depth_max = s.depth_m if t.depth_max is None else max(t.depth_max, s.depth_m)

    def track(self, now: float, pose: Pose | None, mode: int | None = None) -> None:
        """At most TRACK_HZ rows a second; nothing while the position is unknown."""
        if pose is None or not pose.position_valid or now - self._last_track_t < 1.0 / TRACK_HZ - 1e-9:
            return
        wrote = self._write("track", [
            _fmt(self._clock(), 3), _fmt(pose.lat, 8), _fmt(pose.lon, 8), _fmt(pose.speed_mps, 2),
            _fmt(pose.course_deg, 1), _fmt(pose.heading_deg if pose.heading_valid else None, 1),
            _fmt(pose.heading_valid), pose.status.name, _fmt(pose.position_sigma_m, 3), _fmt(pose.roll_deg, 1),
            _fmt(pose.pitch_deg, 1), _fmt(pose.yaw_rate_dps, 1), "" if mode is None else str(mode)])
        if not wrote:
            return
        self._last_track_t = now
        self._tally.track_points += 1
        if self._last_track_xy is None:
            self._last_track_xy = (pose.lat, pose.lon)
        else:
            step = geo.distance_m(*self._last_track_xy, pose.lat, pose.lon)
            if step >= DISTANCE_MIN_STEP_M:
                self._tally.distance_m += step
                self._last_track_xy = (pose.lat, pose.lon)

    def event(self, severity: int, source: str, text: str) -> None:
        level = SEVERITY_NAMES.get(severity, str(severity))
        if self._write("events", [_fmt(self._clock(), 3), level, source, text]):
            self._tally.events[level] = self._tally.events.get(level, 0) + 1


def summarize_folder(folder: str | Path) -> dict[str, Any]:
    """Rebuild a summary from the data files of a mission (for a folder whose
    summary.json is missing because the boat lost power): sample counts per quality
    and the depth range of the usable ones."""
    folder = Path(folder)
    counts = {q.name: 0 for q in SampleQuality}
    depth_min = depth_max = None
    with open(folder / "bathymetry.csv", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            counts[row["quality"]] = counts.get(row["quality"], 0) + 1
            if row["quality"] != "INVALID" and row["depth_m"]:
                depth = float(row["depth_m"])
                depth_min = depth if depth_min is None else min(depth_min, depth)
                depth_max = depth if depth_max is None else max(depth_max, depth)
    return {"bathymetry": counts, "depth_min_m": depth_min, "depth_max_m": depth_max}
