"""Vehicle parameters: ranges, and persistence across restarts (Phase 22).

Until this phase the parameters were a plain dict in RAM:
  * every restart threw away the operator's tuning (PID gains, fence, failsafe);
  * PARAM_SET accepted any number, NaN included. A NaN FS_TIMEOUT silently turns
    the GCS failsafe off (age > NaN is never true); RCMAP_THROTTLE = 0 sent the
    STM a channel number of -1;
  * at boot the Pi pushed its RAM DEFAULTS for the RC map down to the STM, over
    whatever calibration the STM had saved in its flash.

The ParamStore fixes all three. A value is accepted only if it is a finite number
inside the parameter's range (an integer, and one of an allowed set, where the
parameter is an enumeration); a rejected PARAM_SET leaves the old value, and the
caller echoes it back so the ground station shows what the vehicle really holds.
Accepted changes are written to a JSON file, atomically, a moment after the last
change (the GCS writes dozens in a burst), and only the values that differ from
the defaults, so a new default in a software update reaches vehicles that never
changed that parameter. Loading validates every entry the same way: a corrupt
file is set aside (`.corrupt`) and the defaults used; a bad single entry is
dropped and reported; the rest is kept.

`owner` says who ACTS on a parameter: "stm" ones (the RC map and mode-switch
bands) are mirrored to the STM over BAHR-LINK, which keeps its own flash copy so
manual driving works with the Pi dead. The Pi cannot read that copy back, so it is
the Pi's file that is authoritative once it exists.
"""
from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping

from bahr_pilot.modes import MODE_NAMES

FILE_FORMAT = 1
FLUSH_DELAY_S = 1.0


@dataclass(frozen=True)
class ParamSpec:
    minimum: float
    maximum: float
    integer: bool = False
    choices: frozenset[int] | None = None      # for enumerations: the only allowed values
    owner: str = "pi"                          # "pi", or "stm" when mirrored to the Nucleo


def _real(lo, hi, owner="pi"):
    return ParamSpec(float(lo), float(hi), owner=owner)


def _int(lo, hi, owner="pi"):
    return ParamSpec(float(lo), float(hi), integer=True, owner=owner)


def _enum(values, owner="pi"):
    return ParamSpec(float(min(values)), float(max(values)), integer=True, choices=frozenset(values), owner=owner)


_ONOFF = _int(0, 1)
_ACTION = _enum({0, 1, 2, 3, 4, 5})            # FS_ACTION / BATT_FS_*_ACT / FENCE_ACTION (ArduRover values)

SPECS: dict[str, ParamSpec] = {
    # navigation
    "WP_RADIUS": _real(0.5, 50),
    "CRUISE_SPEED": _real(0, 10),
    "CRUISE_THROTTLE": _real(0, 100),
    "NAVL1_PERIOD": _real(1, 60),
    "NAVL1_DAMPING": _real(0.1, 2),
    # steering / speed control (ranges equal the clamps in Navigator.configure)
    "ATC_STR_ANG_P": _real(0, 10),
    "ATC_STR_RAT_P": _real(0, 3), "ATC_STR_RAT_I": _real(0, 3), "ATC_STR_RAT_D": _real(0, 1),
    "ATC_STR_RAT_FF": _real(0, 3), "ATC_STR_RAT_FILT": _real(0.5, 50), "ATC_STR_RAT_MAX": _real(5, 360),
    "ATC_SPEED_P": _real(0, 3), "ATC_SPEED_I": _real(0, 3), "ATC_SPEED_D": _real(0, 1),
    "ATC_ACCEL_MAX": _real(0.1, 5),
    # failsafe
    "FS_GCS_ENABLE": _ONOFF,
    "FS_TIMEOUT": _real(1, 120),                  # never 0 or NaN: that would turn the link failsafe off
    "FS_ACTION": _ACTION,
    "FS_EKF_ACTION": _enum({0, 1, 2}),
    "BATT_LOW_VOLT": _real(0, 60), "BATT_CRT_VOLT": _real(0, 60),
    "BATT_FS_ENABLE": _ONOFF, "BATT_FS_LOW_ACT": _ACTION, "BATT_FS_CRT_ACT": _ACTION,
    # geofence
    "FENCE_ENABLE": _ONOFF, "FENCE_TYPE": _int(0, 15), "FENCE_ACTION": _ACTION,
    "FENCE_RADIUS": _real(0, 10000), "FENCE_MARGIN": _real(0, 100), "FENCE_COAST_DECEL": _real(0.05, 5),
    # sensors
    "AHRS_ORIENTATION": _enum({0, 2, 4, 6, 8}),
    # echo sounder filter and bathymetry sampling
    "RNGFND1_MIN": _real(0, 50), "RNGFND1_MAX": _real(0.5, 1000), "RNGFND1_OFFSET": _real(-10, 10),
    "SONAR_SPIKE": _real(0.05, 10), "SONAR_LATENCY": _real(0, 5),
    "BATHY_SPACING": _real(0.1, 100), "BATHY_MAX_HACC": _real(0.01, 50),
    "BATHY_MAX_SPEED": _real(0.1, 20), "BATHY_MAX_TILT": _real(1, 90),
    # RC map and mode switch (mirrored to the STM)
    "RCMAP_ROLL": _int(1, 16, "stm"), "RCMAP_THROTTLE": _int(1, 16, "stm"), "RCMAP_ARM": _int(1, 16, "stm"),
    "MODE_CH": _int(1, 16, "stm"),
}
SPECS.update({f"MODE{n}": _enum(set(MODE_NAMES), "stm") for n in range(1, 7)})
for _ch in range(1, 9):
    SPECS[f"RC{_ch}_MIN"] = _int(0, 2500, "stm")
    SPECS[f"RC{_ch}_MAX"] = _int(0, 2500, "stm")
    SPECS[f"RC{_ch}_TRIM"] = _int(0, 2500, "stm")
    SPECS[f"RC{_ch}_REVERSED"] = _int(0, 1, "stm")


def check_value(spec: ParamSpec, value) -> str | None:
    """None if `value` is acceptable for the parameter, else the reason it is not."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return "not a number"
    value = float(value)
    if not math.isfinite(value):
        return "not a finite number"
    if value < spec.minimum or value > spec.maximum:
        return f"outside {spec.minimum:g}..{spec.maximum:g}"
    if spec.integer and abs(value - round(value)) > 1e-6:
        return "must be a whole number"
    if spec.choices is not None and int(round(value)) not in spec.choices:
        return f"not one of {sorted(spec.choices)}"
    return None


@dataclass(frozen=True)
class SetResult:
    accepted: bool
    value: float            # the value the vehicle holds afterwards (the old one if rejected)
    reason: str = ""


class ParamStore:
    def __init__(self, defaults: Mapping[str, float], specs: Mapping[str, ParamSpec] = SPECS,
                 path: Path | str | None = None, clock: Callable[[], float] = time.monotonic,
                 flush_delay_s: float = FLUSH_DELAY_S) -> None:
        missing = set(defaults) - set(specs)
        if missing:
            raise ValueError(f"parameters without a range: {sorted(missing)}")
        self.defaults = dict(defaults)
        self.specs = specs
        self.values: dict[str, float] = dict(defaults)       # the live table; the vehicle reads this
        self.path = Path(path) if path else None
        self._clock = clock
        self._flush_delay = flush_delay_s
        self._dirty_since: float | None = None
        self.loaded_names: set[str] = set()                  # restored from the file at start-up
        self.write_error: str | None = None

    # -- changes ------------------------------------------------------------------------------

    def set(self, name: str, value) -> SetResult:
        spec = self.specs.get(name)
        if spec is None or name not in self.values:
            return SetResult(False, math.nan, "unknown parameter")
        reason = check_value(spec, value)
        if reason is not None:
            return SetResult(False, self.values[name], reason)
        value = float(round(value)) if spec.integer else float(value)
        if value != self.values[name]:
            self.values[name] = value
            self._dirty_since = self._dirty_since if self._dirty_since is not None else self._clock()
        return SetResult(True, value)

    def reset(self) -> None:
        """Back to the defaults, and forget the saved file."""
        self.values.update(self.defaults)
        self._dirty_since = None
        self.loaded_names.clear()
        if self.path is not None:
            try:
                self.path.unlink(missing_ok=True)
            except OSError as exc:
                self.write_error = str(exc)

    # -- persistence ---------------------------------------------------------------------------

    def _changed(self) -> dict[str, float]:
        return {name: v for name, v in self.values.items() if v != self.defaults[name]}

    def flush(self, force: bool = False) -> bool:
        """Write the file if a change is waiting and has waited long enough (or
        `force`). True if a write happened. A failed write is remembered in
        `write_error` and retried on the next call, never raised."""
        if self.path is None or self._dirty_since is None:
            return False
        if not force and self._clock() - self._dirty_since < self._flush_delay:
            return False
        payload = {"format": FILE_FORMAT, "params": self._changed()}
        tmp = self.path.with_name(self.path.name + ".tmp")
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(tmp, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=1, sort_keys=True)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp, self.path)           # atomic: a power cut leaves the old file or the new, never half
        except OSError as exc:
            self.write_error = str(exc)
            return False
        self.write_error = None
        self._dirty_since = None
        return True

    def load(self) -> list[str]:
        """Restore saved values. Returns human-readable warnings (empty when
        everything was fine or there was no file)."""
        warnings: list[str] = []
        if self.path is None or not self.path.exists():
            return warnings
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict) or not isinstance(data.get("params"), dict):
                raise ValueError("no 'params' table")
            fmt = data.get("format")
        except (OSError, ValueError) as exc:
            aside = self.path.with_name(self.path.name + ".corrupt")
            try:
                os.replace(self.path, aside)
            except OSError:
                pass
            return [f"saved parameters unreadable ({exc}); defaults used, file kept as {aside.name}"]
        if fmt != FILE_FORMAT:
            warnings.append(f"saved parameters have format {fmt}, this software reads {FILE_FORMAT}: reading what fits")
        for name, value in data["params"].items():
            spec = self.specs.get(name)
            if spec is None or name not in self.values:
                warnings.append(f"saved parameter {name} is unknown here, ignored")
                continue
            reason = check_value(spec, value)
            if reason is not None:
                warnings.append(f"saved {name}={value!r} ignored ({reason}); default kept")
                continue
            self.values[name] = float(round(value)) if spec.integer else float(value)
            if self.values[name] != self.defaults[name]:
                self.loaded_names.add(name)
        return warnings

    def reload(self) -> list[str]:
        """Throw away unsaved changes: defaults first, then the file on top."""
        self.values.update(self.defaults)
        self._dirty_since = None
        self.loaded_names.clear()
        return self.load()

    def has_saved(self, owner: str) -> bool:
        """Were any parameters of this owner restored from the file?"""
        return any(self.specs[name].owner == owner for name in self.loaded_names)
