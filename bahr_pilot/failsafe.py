"""Failsafe on the Pi (Phase 20): what to do when something the mission depends
on goes wrong.

Sources and the ArduRover parameters that choose their action (all already in
BAHR-GCS's dictionary):

    GCS_LOST          FS_GCS_ENABLE, FS_TIMEOUT, FS_ACTION
    BATTERY_LOW       BATT_FS_ENABLE, BATT_LOW_VOLT, BATT_FS_LOW_ACT
    BATTERY_CRITICAL  BATT_FS_ENABLE, BATT_CRT_VOLT, BATT_FS_CRT_ACT
    POSITION_LOST     FS_EKF_ACTION   (also covers the position estimator)
    HEADING_LOST      FS_EKF_ACTION   (no usable heading = no steering)
    GEOFENCE          FENCE_ENABLE, FENCE_ACTION, FENCE_MARGIN   (bahr_pilot/geofence.py)

Actions: REPORT (tell the operator, change nothing), RTL, HOLD (stop; the mode
becomes HOLD), TERMINATE (disarm). ArduPilot's SmartRTL values (3, 4) need a
recorded track, which this vehicle does not keep: 3 acts as RTL, 4 as HOLD.
An unknown value acts as HOLD - the safe reading of a corrupt parameter.

Design rules:
  * A condition must persist for its hold time before it fires, and be gone for
    CLEAR_HOLD_S before it clears, so a flickering input does not flood the
    operator or flap the mode. Battery voltage needs hysteresis on top: with
    the motors stopped the pack recovers and would clear its own failsafe.
  * Level-triggered: while a source is active and the vehicle is in an
    autonomous mode, the action keeps being required. An operator who selects
    AUTO again with the cause still present gets HOLD again; the mission does
    not resume by itself when the cause goes away either, the operator
    re-engages it.
  * The strongest active action wins: TERMINATE > HOLD > RTL > REPORT.
  * Position and heading are only watched in autonomous modes (in MANUAL the
    pilot is the navigator). GCS and battery are watched whenever armed, but
    modes other than the autonomous ones are never changed.
  * A GCS that has never been heard cannot be "lost" (as in ArduPilot).
  * Heading validity can flicker at the rate GNSS headings arrive when the
    gyro is gone (valid ~0.15 s per second), so it is judged by its AVAILABILITY
    over a window, not by an unbroken run of invalid ticks.

Pure logic with an injected clock: tests/test_failsafe.py.
"""
from __future__ import annotations

import enum
from collections import deque
from dataclasses import dataclass
from typing import Mapping

from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_RTL

AUTONOMOUS_MODES = frozenset({MODE_AUTO, MODE_GUIDED, MODE_RTL})

BATTERY_HOLD_S = 3.0              # voltage must stay low this long (load sag is not a failsafe)
BATTERY_HYSTERESIS_V = 0.4        # it must recover this far above the threshold to clear
POSITION_HOLD_S = 1.0
FENCE_HOLD_S = 0.5
# Without a gyro the heading is good for ~15 % of the time (0.15 s after each GNSS
# heading); with one it is ~100 %. The threshold sits between, and the 5 s minimum
# keeps the normal start-up wait for the first GNSS heading from looking like a loss.
HEADING_WINDOW_S = 6.0
HEADING_MIN_SPAN_S = 5.0          # do not judge availability on less data than this
HEADING_MIN_AVAILABILITY = 0.3
CLEAR_HOLD_S = 2.0


class Source(enum.Enum):
    GCS_LOST = "GCS link lost"
    BATTERY_LOW = "battery low"
    BATTERY_CRITICAL = "battery critical"
    POSITION_LOST = "position lost"
    HEADING_LOST = "heading lost"
    GEOFENCE = "fence breached"


class Action(enum.IntEnum):
    REPORT = 0
    RTL = 1
    HOLD = 2
    TERMINATE = 5

    @property
    def label(self) -> str:
        return {0: "report only", 1: "RTL", 2: "HOLD", 5: "terminate"}[int(self)]


# ArduPilot FS_ACTION / BATT_FS_*_ACT values -> what this vehicle does
_ACTION_BY_VALUE = {0: Action.REPORT, 1: Action.RTL, 2: Action.HOLD, 3: Action.RTL, 4: Action.HOLD,
                    5: Action.TERMINATE}


def action_from_param(value: float) -> Action:
    """Unknown or non-finite values act as HOLD."""
    try:
        return _ACTION_BY_VALUE.get(int(value), Action.HOLD)
    except (TypeError, ValueError, OverflowError):
        return Action.HOLD


@dataclass(frozen=True)
class Inputs:
    now: float                        # monotonic seconds
    armed: bool
    mode: int
    gcs_age_s: float | None           # since the last GCS message; None = never heard
    battery_v: float | None           # None = unknown
    position_ok: bool
    heading_ok: bool
    fence_margin_m: float | None = None   # signed distance to the fence, + inside; None = no fence applies


@dataclass(frozen=True)
class Event:
    kind: str                         # "triggered" or "cleared"
    source: Source
    action: Action
    text: str


class FailsafeMonitor:
    def __init__(self) -> None:
        self.active: dict[Source, Action] = {}
        self._raw_since: dict[Source, float] = {}      # since when the raw condition has been true
        self._clear_since: dict[Source, float] = {}    # since when it has been false while active
        self._heading_samples: deque[tuple[float, bool]] = deque()
        self._last_evaluated: set[Source] = set()

    def reset(self) -> None:
        self.active.clear()
        self._raw_since.clear()
        self._clear_since.clear()
        self._heading_samples.clear()

    # -- what the vehicle should do -----------------------------------------------------

    def required_action(self) -> Action | None:
        """The strongest action among the active sources, or None."""
        return max(self.active.values()) if self.active else None

    def stops_motors(self) -> bool:
        action = self.required_action()
        return action in (Action.HOLD, Action.TERMINATE)

    # -- the rules -------------------------------------------------------------------------

    def update(self, inp: Inputs, params: Mapping[str, float]) -> list[Event]:
        if not inp.armed:
            self.reset()          # nothing is being flown: forget everything, silently
            return []

        autonomous = inp.mode in AUTONOMOUS_MODES
        events: list[Event] = []
        for source, (raw, action, hold_s, clear_ok) in self._conditions(inp, params, autonomous).items():
            events.extend(self._step(source, raw, action, hold_s, clear_ok, inp.now))
        # A source that is no longer evaluated at all (disabled, or the mode left autonomy, e.g. because
        # the failsafe itself forced HOLD) is dropped WITHOUT saying "cleared": its cause may well
        # persist. The simulator showed "Failsafe cleared: position lost" 0.05 s after "position lost"
        # while the GNSS was still down, which reads as if the position had come back.
        for source in list(self.active):
            if source not in self._last_evaluated:
                self._forget(source)
        return events

    def _conditions(self, inp: Inputs, params: Mapping[str, float], autonomous: bool):
        """{source: (raw condition, action, hold time, clear-ok flag)} for every
        source that is enabled and applicable right now."""
        out = {}
        if params["FS_GCS_ENABLE"] > 0.5 and inp.gcs_age_s is not None:
            lost = inp.gcs_age_s > params["FS_TIMEOUT"]
            out[Source.GCS_LOST] = (lost, action_from_param(params["FS_ACTION"]), 0.0, not lost)
        if params["BATT_FS_ENABLE"] > 0.5 and inp.battery_v is not None:
            low, crit = params["BATT_LOW_VOLT"], params["BATT_CRT_VOLT"]
            v = inp.battery_v
            out[Source.BATTERY_LOW] = (v < low, action_from_param(params["BATT_FS_LOW_ACT"]), BATTERY_HOLD_S,
                                       v >= low + BATTERY_HYSTERESIS_V)
            if crit > 0.0:
                out[Source.BATTERY_CRITICAL] = (v < crit, action_from_param(params["BATT_FS_CRT_ACT"]),
                                                BATTERY_HOLD_S, v >= crit + BATTERY_HYSTERESIS_V)
        if params.get("FENCE_ENABLE", 0.0) > 0.5 and inp.fence_margin_m is not None:
            margin = inp.fence_margin_m
            # FENCE_MARGIN doubles as the hysteresis: once breached the boat has to be that
            # far back inside before the failsafe clears, so riding the boundary does not flap
            out[Source.GEOFENCE] = (margin < 0.0, action_from_param(params["FENCE_ACTION"]), FENCE_HOLD_S,
                                    margin >= params["FENCE_MARGIN"])
        ekf = int(params["FS_EKF_ACTION"])
        if ekf != 0 and autonomous:
            action = Action.REPORT if ekf == 2 else Action.HOLD
            out[Source.POSITION_LOST] = (not inp.position_ok, action, POSITION_HOLD_S, inp.position_ok)
            heading_lost = self._heading_unavailable(inp.now, inp.heading_ok)
            out[Source.HEADING_LOST] = (heading_lost, action, 0.0, not heading_lost)
        else:
            self._heading_samples.clear()
        self._last_evaluated = set(out)
        return out

    def _heading_unavailable(self, now: float, ok: bool) -> bool:
        samples = self._heading_samples
        samples.append((now, ok))
        while samples and now - samples[0][0] > HEADING_WINDOW_S:
            samples.popleft()
        if now - samples[0][0] < HEADING_MIN_SPAN_S:
            return False
        availability = sum(1 for _, good in samples if good) / len(samples)
        return availability < HEADING_MIN_AVAILABILITY

    def _step(self, source: Source, raw: bool, action: Action, hold_s: float, clear_ok: bool,
              now: float) -> list[Event]:
        if source not in self.active:
            if not raw:
                self._raw_since.pop(source, None)
                return []
            since = self._raw_since.setdefault(source, now)
            if now - since < hold_s:
                return []
            self.active[source] = action
            self._clear_since.pop(source, None)
            return [Event("triggered", source, action, f"Failsafe: {source.value} ({action.label})")]
        # active: keep the action current (a parameter may have changed), watch for clearing
        self.active[source] = action
        if not clear_ok:
            self._clear_since.pop(source, None)
            return []
        since = self._clear_since.setdefault(source, now)
        if now - since < CLEAR_HOLD_S:
            return []
        return self._clear(source)

    def _forget(self, source: Source) -> Action:
        action = self.active.pop(source, Action.REPORT)
        self._raw_since.pop(source, None)
        self._clear_since.pop(source, None)
        return action

    def _clear(self, source: Source) -> list[Event]:
        """The cause really went away (and stayed away for CLEAR_HOLD_S)."""
        return [Event("cleared", source, self._forget(source), f"Failsafe cleared: {source.value}")]
