"""Health table (Phase 24): one level per component, so "what is wrong with the boat" has
one answer.

    OK       working as it should
    WARNING  degraded: the vehicle still does its job, but the operator should know
             (dead reckoning, a quiet sonar, a slow loop, a low battery, no RC signal)
    ERROR    a function the vehicle needs is gone (no position, no heading, no IMU, a
             silent STM, a critical battery, a failed mission log)

The monitor is a pure function of what it is handed (`Snapshot`) plus its own memory: no
clocks, no I/O. The vehicle builds the snapshot once per loop and sends what comes out:
  * `update()` -> the TRANSITIONS to announce (STATUSTEXT, and from there the mission log's
    event file), debounced and rate limited so a flapping sensor cannot flood the operator;
  * `sys_status()` -> the SYS_STATUS sensor bit fields (present / enabled / health) and the
    main-loop load, which QGroundControl and Mission Planner show as sensor health.
    BAHR-GCS ignores the health bits (`_system_status_name` always says "OK"), so they
    change nothing on its screen.

This is a REPORT, not a failsafe: nothing here stops the motors or changes mode. Acting on a
bad condition is failsafe.py's job (its causes are shown here as the `failsafe` row, which
is not announced a second time because the failsafe monitor already says it).

Debounce: a worse level must hold ESCALATE_HOLD_S before the TABLE shows it (one lost frame is
not a fault), a better one RECOVER_HOLD_S (so a marginal sensor does not flap). The operator
is TOLD separately: nothing in the first STARTUP_GRACE_S after boot (sensors are waking up; a
fault that heals within it is never mentioned), and a part is announced at most once per
EVENT_MIN_INTERVAL_S, each announcement bringing the operator up to date with the table
rather than replaying every flap. The failsafes do not use this table.
"""
from __future__ import annotations

import enum
import math
from collections import deque
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping

from pymavlink import mavutil

from bahr_pilot.estimator import EstimateStatus
from bahr_pilot.failsafe import BATTERY_HYSTERESIS_V

ESCALATE_HOLD_S = 1.0
RECOVER_HOLD_S = 3.0
EVENT_MIN_INTERVAL_S = 5.0
STARTUP_GRACE_S = 10.0             # nothing is announced this soon after boot: GNSS and the sonar are still waking up

IMU_STALE_S = 0.5                  # no IMU frame for this long: 25 frames at the nominal rate
IMU_NOMINAL_HZ = 50.0              # BAHR-LINK IMU frame rate (docs/MAVLINK_PROTOCOL.md, STM -> Pi table)
IMU_MIN_RATE_FRACTION = 0.8        # below 40 Hz frames are being lost (UART errors drop frames silently)
IMU_RATE_WINDOW_S = 2.0
STM_STALE_S = 1.0                  # == vehicle.RC_TELEMETRY_FRESH_S (a test keeps them equal)
STM_RESTART_HOLD_S = 30.0          # an STM restart (watchdog, brown-out) stays visible for this long
SONAR_STALE_S = 5.0                # == SonarConfig.gap_s: after this the filter forgets its history
GCS_STALE_S = 2.0                  # warn before FS_TIMEOUT (default 3 s) turns it into a failsafe
LOOP_WARN_BUSY_S = 0.05            # one iteration longer than the 20 Hz tick: the loop rate halves
LOOP_ERROR_BUSY_S = 0.10           # == EstimatorConfig.imu_timeout_s: the heading filter starts to doubt itself
LOOP_WINDOW_S = 5.0
MAX_TEXT = 50                      # a STATUSTEXT is 50 bytes


class Level(enum.IntEnum):
    OK = 0
    WARNING = 1
    ERROR = 2


class Part(enum.Enum):
    GNSS = "GNSS"
    HEADING = "heading"
    IMU = "IMU"
    STM = "STM link"
    RC = "RC"
    BATTERY = "battery"
    SONAR = "sonar"
    GCS = "GCS link"
    STORAGE = "storage"
    LOOP = "main loop"
    FAILSAFE = "failsafe"


_SILENT = frozenset({Part.FAILSAFE})        # failsafe.py announces its own triggers and clears
# STATUSTEXT severities
_SEVERITY = {Level.OK: 6, Level.WARNING: 4, Level.ERROR: 3}


@dataclass(frozen=True)
class Snapshot:
    """Everything the monitor looks at, gathered by the vehicle once per loop. Ages are seconds
    on the vehicle's monotonic clock; None means "never"."""
    now: float
    autonomous: bool = False                     # the mode steers by itself (AUTO, RTL, ...)
    position: EstimateStatus | None = None       # None: no estimate yet
    heading_valid: bool = False
    imu_age_s: float | None = None
    imu_frames: int | None = None                # cumulative count, for the frame rate
    imu_data_valid: bool | None = None           # the STM's own "IMU good" flag; None = unknown
    stm_age_s: float | None = None
    stm_resets: int = 0                          # the STM clock has gone backwards this many times
    rc_link_up: bool | None = None               # None = unknown (STM silent)
    battery_v: float | None = None
    sonar_age_s: float | None = None
    sonar_suspect: bool = False                  # the newest reading was not confirmed
    gcs_age_s: float | None = None
    params_write_error: str | None = None
    log_configured: bool = False                 # --log-dir was given
    log_error: str | None = None
    loop_busy_max_s: float = 0.0                 # longest iteration in the last LOOP_WINDOW_S
    failsafes: tuple[str, ...] = ()              # labels of the active failsafe causes
    fence_enabled: bool = False
    fence_breached: bool = False


@dataclass(frozen=True)
class Transition:
    part: Part
    old: Level
    new: Level
    reason: str
    text: str                                    # fits a STATUSTEXT
    severity: int


@dataclass(frozen=True)
class Row:
    part: Part
    level: Level
    reason: str
    since: float | None                          # monotonic time the level last changed; None = never changed


@dataclass(frozen=True)
class SysStatus:
    present: int
    enabled: int
    health: int
    load: int                                    # main-loop busy time, parts per thousand


class LoopTimer:
    """How busy the main loop is. The vehicle calls `record` once per iteration with the time
    it started and how long the work took (not counting the sleep)."""

    def __init__(self, window_s: float = LOOP_WINDOW_S) -> None:
        self.window_s = window_s
        self._records: deque[tuple[float, float]] = deque()

    def record(self, start: float, busy_s: float) -> None:
        self._records.append((start, max(0.0, busy_s)))
        while self._records and start - self._records[0][0] > self.window_s:
            self._records.popleft()

    def max_busy_s(self) -> float:
        return max((busy for _, busy in self._records), default=0.0)

    def load_permille(self) -> int:
        """Busy time over elapsed time across the window (the last iteration has no known period yet)."""
        if len(self._records) < 2:
            return 0
        elapsed = self._records[-1][0] - self._records[0][0]
        if elapsed <= 0.0:
            return 0
        busy = sum(b for _, b in list(self._records)[:-1])
        return max(0, min(1000, round(1000.0 * busy / elapsed)))


@dataclass
class _State:
    reported: Level = Level.OK                    # what the table says (after the debounce)
    announced: Level = Level.OK                   # what the operator was last told
    reason: str = ""
    since: float | None = None
    last_event: float = -math.inf
    pending: Level | None = None
    pending_since: float = 0.0


class HealthMonitor:
    def __init__(self, present: Iterable[Part]) -> None:
        self.present = frozenset(present) | {Part.FAILSAFE}
        self._state = {part: _State() for part in self.present}
        self._raw: dict[Part, tuple[Level, str]] = {part: (Level.OK, "") for part in self.present}
        self._t0: float | None = None             # time of the first update: the start-up grace runs from here
        self._imu_history: deque[tuple[float, int]] = deque()
        self._stm_resets_seen: int | None = None
        self._stm_restart_until = -math.inf
        self._battery_low = False                 # hysteresis memory
        self._battery_critical = False

    # -- the table ---------------------------------------------------------------------------------

    def level(self, part: Part) -> Level:
        return self._state[part].reported if part in self._state else Level.OK

    def worst(self) -> Level:
        return max((s.reported for s in self._state.values()), default=Level.OK)

    def rows(self) -> list[Row]:
        return [Row(part, s.reported, s.reason, s.since) for part, s in self._state.items()]

    def table(self) -> dict[str, dict[str, str]]:
        """For meta.json: the table as it stands."""
        return {row.part.value: {"level": row.level.name, "reason": row.reason} for row in self.rows()}

    # -- evaluation --------------------------------------------------------------------------------

    def update(self, snap: Snapshot, params: Mapping[str, float]) -> list[Transition]:
        if self._t0 is None:
            self._t0 = snap.now
        raw = self._evaluate(snap, params)
        self._raw = raw
        out: list[Transition] = []
        for part, (level, reason) in raw.items():
            transition = self._step(part, level, reason, snap.now)
            if transition is not None:
                out.append(transition)
        return out

    def _step(self, part: Part, level: Level, reason: str, now: float) -> Transition | None:
        state = self._state[part]
        # 1. the table: follows the raw level through the debounce, nothing else
        if level == state.reported:
            state.pending = None
            state.reason = reason                 # the same level for a different reason
        else:
            if state.pending != level:
                state.pending, state.pending_since = level, now
            hold = ESCALATE_HOLD_S if level > state.reported else RECOVER_HOLD_S
            if now - state.pending_since >= hold:
                state.reported, state.reason, state.since, state.pending = level, reason, now, None
        # 2. the announcement: what the operator was last told, brought up to date once the
        #    start-up grace is over and this part has been quiet for EVENT_MIN_INTERVAL_S
        if state.reported == state.announced:
            return None
        if now - self._t0 < STARTUP_GRACE_S or now - state.last_event < EVENT_MIN_INTERVAL_S:
            return None
        old, state.announced, state.last_event = state.announced, state.reported, now
        if part in _SILENT:
            return None
        new = state.reported
        text = f"{part.value}: recovered" if new == Level.OK else f"{part.value}: {state.reason}"
        return Transition(part, old, new, state.reason, text[:MAX_TEXT], _SEVERITY[new])

    def _evaluate(self, s: Snapshot, params: Mapping[str, float]) -> dict[Part, tuple[Level, str]]:
        checks: dict[Part, Callable[[], tuple[Level, str]]] = {
            Part.GNSS: lambda: self._gnss(s),
            Part.HEADING: lambda: self._heading(s),
            Part.IMU: lambda: self._imu(s),
            Part.STM: lambda: self._stm(s),
            Part.RC: lambda: self._rc(s),
            Part.BATTERY: lambda: self._battery(s, params),
            Part.SONAR: lambda: self._sonar(s),
            Part.GCS: lambda: self._gcs(s),
            Part.STORAGE: lambda: self._storage(s),
            Part.LOOP: lambda: self._loop(s),
            Part.FAILSAFE: lambda: self._failsafe(s),
        }
        return {part: check() for part, check in checks.items() if part in self.present}

    # -- the checks ----------------------------------------------------------------------------------

    @staticmethod
    def _gnss(s: Snapshot) -> tuple[Level, str]:
        if s.position is None or s.position == EstimateStatus.NONE:
            return Level.ERROR, "no position"
        if s.position == EstimateStatus.DEAD_RECKONING:
            return Level.WARNING, "dead reckoning"
        return Level.OK, ""

    @staticmethod
    def _heading(s: Snapshot) -> tuple[Level, str]:
        return (Level.OK, "") if s.heading_valid else (Level.ERROR, "no valid heading")

    def _imu(self, s: Snapshot) -> tuple[Level, str]:
        rate = self._imu_rate(s)
        if s.imu_age_s is None or s.imu_age_s > IMU_STALE_S:
            return Level.ERROR, "no data"
        if s.imu_data_valid is False:
            return Level.WARNING, "data invalid"
        if rate is not None and rate < IMU_MIN_RATE_FRACTION * IMU_NOMINAL_HZ:
            return Level.WARNING, f"rate {rate:.0f} Hz"
        return Level.OK, ""

    def _imu_rate(self, s: Snapshot) -> float | None:
        """Frames per second over the last IMU_RATE_WINDOW_S, or None until that much has been seen."""
        if s.imu_frames is None:
            return None
        history = self._imu_history
        if history and s.imu_frames < history[-1][1]:
            history.clear()                       # the counter restarted
        history.append((s.now, s.imu_frames))
        while len(history) > 2 and s.now - history[1][0] >= IMU_RATE_WINDOW_S:
            history.popleft()
        t0, n0 = history[0]
        if s.now - t0 < IMU_RATE_WINDOW_S:
            return None
        return (s.imu_frames - n0) / (s.now - t0)

    def _stm(self, s: Snapshot) -> tuple[Level, str]:
        if self._stm_resets_seen is None:
            self._stm_resets_seen = s.stm_resets
        elif s.stm_resets > self._stm_resets_seen:
            self._stm_resets_seen = s.stm_resets
            self._stm_restart_until = s.now + STM_RESTART_HOLD_S
        if s.stm_age_s is None:
            return Level.ERROR, "no telemetry"
        if s.stm_age_s > STM_STALE_S:
            return Level.ERROR, f"silent {s.stm_age_s:.0f} s"
        if s.now < self._stm_restart_until:
            return Level.WARNING, f"restarted ({self._stm_resets_seen})"
        return Level.OK, ""

    @staticmethod
    def _rc(s: Snapshot) -> tuple[Level, str]:
        return (Level.WARNING, "no signal") if s.rc_link_up is False else (Level.OK, "")

    def _battery(self, s: Snapshot, params: Mapping[str, float]) -> tuple[Level, str]:
        v = s.battery_v
        if v is None:
            self._battery_low = self._battery_critical = False
            return Level.WARNING, "no reading"
        low, critical = params["BATT_LOW_VOLT"], params["BATT_CRT_VOLT"]
        # the same hysteresis as the failsafe, so the table and the failsafe agree about "recovered"
        self._battery_critical = critical > 0.0 and (
            v < critical or (self._battery_critical and v < critical + BATTERY_HYSTERESIS_V))
        self._battery_low = v < low or (self._battery_low and v < low + BATTERY_HYSTERESIS_V)
        if self._battery_critical:
            return Level.ERROR, f"{v:.1f} V critical"
        if self._battery_low:
            return Level.WARNING, f"{v:.1f} V low"
        return Level.OK, ""

    @staticmethod
    def _sonar(s: Snapshot) -> tuple[Level, str]:
        if s.sonar_age_s is None:
            return Level.WARNING, "no depth data"
        if s.sonar_age_s > SONAR_STALE_S:
            return Level.WARNING, f"no data for {s.sonar_age_s:.0f} s"
        if s.sonar_suspect:
            return Level.WARNING, "readings suspect"
        return Level.OK, ""

    @staticmethod
    def _gcs(s: Snapshot) -> tuple[Level, str]:
        if "GCS link lost" in s.failsafes:
            return Level.ERROR, "link lost"
        if s.gcs_age_s is not None and s.gcs_age_s > GCS_STALE_S:     # never heard is not lost
            return Level.WARNING, f"silent {s.gcs_age_s:.0f} s"
        return Level.OK, ""

    @staticmethod
    def _storage(s: Snapshot) -> tuple[Level, str]:
        if s.log_error is not None:
            return Level.ERROR, "mission log failed"       # survey data is being lost
        if s.params_write_error is not None:
            return Level.WARNING, "params not saved"
        return Level.OK, ""

    @staticmethod
    def _loop(s: Snapshot) -> tuple[Level, str]:
        ms = s.loop_busy_max_s * 1000.0
        if s.loop_busy_max_s > LOOP_ERROR_BUSY_S:
            return Level.ERROR, f"stalled {ms:.0f} ms"
        if s.loop_busy_max_s > LOOP_WARN_BUSY_S:
            return Level.WARNING, f"slow cycle {ms:.0f} ms"
        return Level.OK, ""

    @staticmethod
    def _failsafe(s: Snapshot) -> tuple[Level, str]:
        return (Level.ERROR, ", ".join(s.failsafes)) if s.failsafes else (Level.OK, "")

    # -- SYS_STATUS ----------------------------------------------------------------------------------

    def sys_status(self, snap: Snapshot, load_permille: int = 0) -> SysStatus:
        """The three sensor bit fields. `health` has a bit for every PRESENT sensor that is OK
        (a WARNING counts as not healthy: the data is degraded or missing); a ground station
        shows `enabled & ~health` as unhealthy. Parts this vehicle does not have set no bit."""
        bit = mavutil.mavlink
        has = self.present.__contains__
        ok = lambda part: part not in self._state or self._state[part].reported == Level.OK   # noqa: E731
        # (bit, present, enabled, healthy)
        table = (
            (bit.MAV_SYS_STATUS_SENSOR_3D_GYRO, has(Part.IMU), True, has(Part.IMU) and ok(Part.IMU)),
            (bit.MAV_SYS_STATUS_SENSOR_3D_ACCEL, has(Part.IMU), False, has(Part.IMU) and ok(Part.IMU)),  # present, unused
            (bit.MAV_SYS_STATUS_SENSOR_GPS, has(Part.GNSS), True, has(Part.GNSS) and ok(Part.GNSS)),
            (bit.MAV_SYS_STATUS_SENSOR_LASER_POSITION, has(Part.SONAR), True, has(Part.SONAR) and ok(Part.SONAR)),
            (bit.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, has(Part.RC), True, has(Part.RC) and ok(Part.RC)),
            (bit.MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS, has(Part.STM), True, has(Part.STM) and ok(Part.STM)),
            (bit.MAV_SYS_STATUS_SENSOR_BATTERY, has(Part.BATTERY), True, has(Part.BATTERY) and ok(Part.BATTERY)),
            (bit.MAV_SYS_STATUS_AHRS, has(Part.HEADING), True, has(Part.HEADING) and ok(Part.HEADING)),
            (bit.MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL, True, snap.autonomous,
             ok(Part.GNSS) and ok(Part.HEADING)),
            (bit.MAV_SYS_STATUS_LOGGING, snap.log_configured, True, snap.log_error is None),
            (bit.MAV_SYS_STATUS_GEOFENCE, True, snap.fence_enabled, not snap.fence_breached),
        )
        present = enabled = health = 0
        for mask, is_present, is_enabled, healthy in table:
            if not is_present:
                continue
            present |= mask
            if is_enabled:
                enabled |= mask
            if healthy:
                health |= mask
        return SysStatus(present, enabled, health, max(0, min(1000, int(load_permille))))
