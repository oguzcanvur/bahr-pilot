"""Real vehicle-side MAVLink process for BAHR — the Pi 4 "brain".

Speaks the same MAVLink wire format sim/fake_vehicle.py uses so BAHR-GCS
needs no changes to talk to it: UDP, connects out to the GCS on port 14550.
Where sim/fake_vehicle.py simulates sensors and motion to exercise the GCS,
this reads the real GNSS/depth sensors (bahr_pilot.sensors), drives the real
Nucleo over UART (bahr_pilot.nucleo_link), and forwards RTK corrections to
the GNSS (bahr_pilot.rtcm_forward) instead. The Nucleo firmware itself lives
alongside this package at firmware/reflex/ (an STM32CubeIDE
project — see that folder's own notes for building/flashing it).

    python -m bahr_pilot.vehicle --gcs-host 10.42.0.50 \\
        --nucleo-port /dev/serial0 \\
        --gnss-port /dev/serial/by-id/usb-...-RTD100 \\
        --echomap-port /dev/serial/by-id/usb-...-echomap

Known gaps (2026-10-01, see ROADMAP.md section 4):
  - BNO086 is now read (firmware/reflex/Core/Src/imu.c, SHTP/I2C, Game
    Rotation Vector report) and ATTITUDE's roll/pitch come from it when
    pi_link reports imu_valid — but that driver has never run against a
    real BNO086 (no hardware this session), and the axis convention
    relative to how it ends up physically mounted is unverified. Yaw is
    still always GNSS heading, by design (no magnetometer fusion).
  - pi_link's RC telemetry frame now also carries battery voltage
    (battery.c, ADC1) — SYS_STATUS reports a real voltage_battery when the
    Nucleo says it's valid — but battery.c's ADC divider ratio is an
    unmeasured placeholder, so treat the number as "roughly plausible",
    not calibrated, until checked against a multimeter.
  - Mission/parameter protocol is implemented but not yet exercised against
    a real mission upload from BAHR-GCS.
  - RC channel -> function mapping (RCMAP_ROLL/RCMAP_THROTTLE/RCMAP_ARM/
    RCMAP_OVERRIDE, RC1..8_MIN/MAX/TRIM/REVERSED) is forwarded to the
    Nucleo and drives real mixing there, using BAHR-GCS's existing
    parameter page and RadioPage calibration screen — but not yet verified
    against a real transmitter end-to-end (no hardware this session). Now
    persisted to the Nucleo's flash (settings.c), survives power-cycle.
  - Failsafes (bahr_pilot/failsafe.py): GCS link lost, battery low/critical,
    position or heading lost while navigating. FS_ACTION / BATT_FS_*_ACT /
    FS_EKF_ACTION choose report, RTL, HOLD or terminate; HOLD really switches
    the mode, and the mission does not resume by itself afterwards. The battery
    failsafe defaults to OFF: the voltage divider ratio is unmeasured, so
    tripping on an uncalibrated reading would be worse than not having it.
"""
from __future__ import annotations

import argparse
import collections
import math
import signal
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

from bahr_pilot import geo
from bahr_pilot.attitude import quat_to_euler, vehicle_orientation, vehicle_vector
from bahr_pilot.bathymetry import BathyConfig, BathymetryRecorder
from bahr_pilot.datalog import DataLogger
from bahr_pilot.diagnostics import HealthMonitor, LoopTimer, Part, Snapshot
from bahr_pilot.estimator import Estimator
from bahr_pilot.failsafe import AUTONOMOUS_MODES, Action, FailsafeMonitor, Inputs, Source
from bahr_pilot.geofence import FenceError, Geofence
from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_HOLD, MODE_MANUAL, MODE_NAMES, MODE_RTL
from bahr_pilot.navigation import Navigator
from bahr_pilot.nucleo_link import (
    ARM_BLOCK_BATTERY, ARM_BLOCK_IMU, ARM_BLOCK_SWITCH_ON, ARM_BLOCK_THROTTLE, ARM_STATE_ARMED,
    NucleoLink, PULSE_MAX_US, PULSE_MIN_US, PULSE_NEUTRAL_US,
)
from bahr_pilot.missionlog import MissionLog
from bahr_pilot.params import ParamStore
from bahr_pilot.rtcm_forward import RtcmForwarder
from bahr_pilot.sensors import GnssState, run_echomap_reader, run_rtd100_reader
from bahr_pilot.sonar import DepthQuality, SonarConfig, SonarFilter
from bahr_pilot.state import VehicleState
from bahr_pilot.versions import AUTOPILOT_VERSION, BAHR_LINK_VERSION, MISSION_FORMAT_VERSION, banner

TICK_HZ = 20.0

# Geofence look-ahead (see Vehicle._fence_margin)
# From 'stop' to the boat actually decelerating: loop and estimator latency, the STM's
# 100 %/s slew ramp (60 % throttle takes 0.6 s to reach neutral) and the ESC/propeller lag.
FENCE_REACTION_S = 1.0
FENCE_MIN_PREDICT_SPEED_MPS = 0.2

# MAVLink MAV_MISSION_TYPE
MISSION_TYPE_MISSION = 0
MISSION_TYPE_FENCE = 1
MISSION_TYPE_ALL = 255
TELEMETRY_HZ = 5.0
# Operator-actionable reasons the Nucleo refused to arm (arming.h), as
# STATUSTEXT for BAHR-GCS. NO_RC / BOOT_GRACE are deliberately absent: they
# are normal at power-up and would only be noise.
PREARM_TEXT = {
    ARM_BLOCK_SWITCH_ON: "PreArm: arm switch is on, cycle it off then on",
    ARM_BLOCK_THROTTLE: "PreArm: throttle stick not centred",
    ARM_BLOCK_IMU: "PreArm: IMU not ready",
    ARM_BLOCK_BATTERY: "PreArm: battery not ready",
}

# The Nucleo sends RC telemetry at 10 Hz; anything older than this is stale
# and must not drive mode decisions.
RC_TELEMETRY_FRESH_S = 1.0
SONAR_SUSPECT_OF_LAST = 3     # the sonar is "suspect" when this many of its last 5 readings were not good
# IMU frames arrive at ~50 Hz; a second without one means the IMU is gone.
IMU_FRESH_S = 1.0

# Mirrors firmware/reflex/Core/Inc/sbus.h's SBUS_CH_MIN/MID/MAX — the
# default RCn_MIN/MAX/TRIM below assume no calibration offset, until
# BAHR-GCS's RadioPage overwrites them with real captured values.
_SBUS_CH_MIN = 172.0
_SBUS_CH_MID = 992.0
_SBUS_CH_MAX = 1811.0

# A small real parameter set. Names reuse ArduPilot Rover's own wherever
# the meaning matches, because BAHR-GCS keys its UI on those names: it
# requests WP_RADIUS on connect to size its turn arcs, and gcs/param_meta.py
# already carries labels/ranges for every name below.
DEFAULT_PARAMS: dict[str, float] = {
    "WP_RADIUS": 3.0,         # m, waypoint acceptance radius
    # Feed-forward speed model, same contract as ArduRover: CRUISE_THROTTLE
    # percent of the forward pulse span is assumed to give CRUISE_SPEED m/s.
    # Open loop until a real speed controller exists (ROADMAP: phase 12).
    "CRUISE_SPEED": 1.5,      # m/s
    "CRUISE_THROTTLE": 60.0,  # %
    "FS_GCS_ENABLE": 1.0,     # act (FS_ACTION) when the GCS link goes quiet
    "FS_TIMEOUT": 3.0,        # s
    # What a failsafe does (bahr_pilot/failsafe.py), ArduRover's values:
    # 0 report only, 1 RTL, 2 HOLD, 5 terminate (disarm); SmartRTL 3/4 act as 1/2.
    "FS_ACTION": 2.0,
    # Position / heading estimate lost while navigating: 0 off, 1 HOLD, 2 report only.
    "FS_EKF_ACTION": 1.0,
    # Channel -> function mapping, 1-based channel numbers matching
    # ArduPilot Rover's own RCMAP_ROLL/RCMAP_THROTTLE convention (and
    # BAHR-GCS's existing parameter page, gcs/param_meta.py's "radio"
    # group). RCMAP_ARM isn't a stock ArduPilot parameter but follows the
    # same naming pattern.
    "RCMAP_ROLL": 1.0,      # steering
    "RCMAP_THROTTLE": 3.0,
    "RCMAP_ARM": 6.0,
    # Flight-mode switch, exactly ArduPilot's scheme (and BAHR-GCS's own
    # MODE_CH / MODE1..MODE6 parameter rows): one RC channel split into six
    # PWM bands, MODEn = the mode band n selects. Defaults suit a 3-position
    # switch (bands 1 / 4 / 6): MANUAL / HOLD / AUTO. The STM handles the
    # MANUAL bands on its own (sticks -> motors, no Pi involved); this
    # process applies every other mode. Replaces the old RCMAP_OVERRIDE.
    "MODE_CH": 5.0,
    "MODE1": float(MODE_MANUAL),
    "MODE2": float(MODE_MANUAL),
    "MODE3": float(MODE_MANUAL),
    "MODE4": float(MODE_HOLD),
    "MODE5": float(MODE_HOLD),
    "MODE6": float(MODE_AUTO),
    # Low-battery failsafe. BATT_LOW_VOLT is the same real ArduPilot
    # parameter already in gcs/param_meta.py's "battery" group (absolute
    # pack voltage, not per-cell — its own description already says to
    # calculate ~3.5V/cell into one absolute number), reused as-is rather
    # than inventing a parallel name. BATT_FS_ENABLE is new, bahr_pilot-
    # only, deliberately a plain on/off (unlike real ArduPilot's
    # BATT_FS_LOW_ACT multi-action enum — this vehicle only ever does one
    # thing on low battery: stop, same as every other failsafe here) —
    # see gcs/param_meta.py for its description. Defaults to OFF: the
    # Nucleo's ADC divider ratio (battery.c) is an unmeasured placeholder,
    # so tripping this by default on an uncalibrated reading would be
    # worse than not having it — turn it on only after checking the
    # reported voltage against a multimeter.
    "BATT_LOW_VOLT": 10.5,
    "BATT_CRT_VOLT": 0.0,         # 0 = no critical level, as in ArduPilot
    "BATT_FS_LOW_ACT": 2.0,       # same values as FS_ACTION
    "BATT_FS_CRT_ACT": 2.0,
    "BATT_FS_ENABLE": 0.0,
    # Geofence (bahr_pilot/geofence.py), ArduRover's names. Off by default: a fence
    # nobody drew must not stop a boat. FENCE_TYPE bits: 2 circle around home,
    # 4 the polygons/circles uploaded over MAVLink. FENCE_ACTION as FS_ACTION
    # (1 = RTL, falling back to HOLD when RTL is impossible).
    "FENCE_ENABLE": 0.0,
    "FENCE_TYPE": 6.0,
    "FENCE_ACTION": 1.0,
    "FENCE_RADIUS": 300.0,        # m
    "FENCE_MARGIN": 2.0,          # m; also the hysteresis for clearing a breach
    # bahr_pilot-only: how hard the hull slows down on its own once the motors are
    # idle (average, m/s^2). A boat does not brake, it glides, and drag falls with
    # speed, so this is far below ATC_ACCEL_MAX. 0.3 is the PLACEHOLDER simulator
    # boat (1.5 m/s glides 4.2 m); measure the real one with a coast-down test.
    "FENCE_COAST_DECEL": 0.3,
    # How the BNO086 board sits in the boat (ArduPilot's AHRS_ORIENTATION,
    # already in BAHR-GCS's parameter dictionary): 0 none, 2/4/6 yaw
    # 90/180/270, 8 roll 180. Wrong -> roll/pitch/rates come out in the wrong
    # axes. Unverified until the board is actually mounted.
    "AHRS_ORIENTATION": 0.0,
    # Echo sounder (bahr_pilot/sonar.py), ArduPilot's rangefinder names where they exist.
    # RNGFND1_OFFSET is the transducer's depth below the waterline, ADDED to every reading;
    # the min/max are checked on the raw reading. SONAR_SPIKE is the smallest jump from the
    # predicted bottom that is rejected as a spike. Thresholds are guesses until tuned on
    # real echoMAP data.
    "RNGFND1_MIN": 0.5,
    "RNGFND1_MAX": 100.0,
    "RNGFND1_OFFSET": 0.0,
    "SONAR_SPIKE": 0.5,
    # bahr_pilot-only: how late the sounding is relative to the position (s). Unmeasured.
    "SONAR_LATENCY": 0.0,
    # Bathymetry samples (bahr_pilot/bathymetry.py): one per BATHY_SPACING metres travelled;
    # worse than BATHY_MAX_HACC / BATHY_MAX_SPEED / BATHY_MAX_TILT makes a sample LOW_QUALITY.
    "BATHY_SPACING": 1.0,
    "BATHY_MAX_HACC": 0.5,
    "BATHY_MAX_SPEED": 3.0,
    "BATHY_MAX_TILT": 15.0,
    # Path following (bahr_pilot/pathfollow.py): ArduRover's own L1 parameters,
    # already in BAHR-GCS, set how far ahead on the line the boat aims
    # (lookahead = damping * period * ground speed / pi). Smaller period =
    # tighter on the line, more aggressive. Defaults are ArduRover's.
    "NAVL1_PERIOD": 10.0,
    "NAVL1_DAMPING": 0.75,
    # Heading and speed control (bahr_pilot/control.py), ArduRover's names.
    # Values are tuned on the PLACEHOLDER simulator boat (docs/SITL.md), scored
    # on the worst of three boats - a safe start, not the real boat's tuning.
    # The steering rate PID is in rad/s units like ArduPilot's.
    "ATC_STR_ANG_P": 0.75,
    "ATC_STR_RAT_P": 0.9,
    "ATC_STR_RAT_I": 0.03,
    "ATC_STR_RAT_D": 0.0,
    "ATC_STR_RAT_FF": 0.3,
    "ATC_STR_RAT_FILT": 10.0,
    "ATC_STR_RAT_MAX": 90.0,
    "ATC_SPEED_P": 0.25,
    "ATC_SPEED_I": 0.10,
    "ATC_SPEED_D": 0.0,
    "ATC_ACCEL_MAX": 1.0,
}
for _ch in range(1, 9):
    DEFAULT_PARAMS[f"RC{_ch}_MIN"] = _SBUS_CH_MIN
    DEFAULT_PARAMS[f"RC{_ch}_MAX"] = _SBUS_CH_MAX
    DEFAULT_PARAMS[f"RC{_ch}_TRIM"] = _SBUS_CH_MID
    DEFAULT_PARAMS[f"RC{_ch}_REVERSED"] = 0.0

_MODE_SLOT_PARAMS = tuple(f"MODE{n}" for n in range(1, 7))
_RC_MAP_PARAMS = {"RCMAP_ROLL", "RCMAP_THROTTLE", "RCMAP_ARM", "MODE_CH", *_MODE_SLOT_PARAMS}
_RC_MAP_PARAMS |= {
    f"RC{ch}_{suffix}" for ch in range(1, 9) for suffix in ("MIN", "MAX", "TRIM", "REVERSED")
}


class Vehicle:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.link = mavutil.mavlink_connection(
            f"udpout:{args.gcs_host}:{args.gcs_port}", source_system=1, source_component=1
        )
        self.state = VehicleState()
        self.navigator = Navigator()
        # Parameters: ranges, and a file that survives restarts (bahr_pilot/params.py).
        # self.params IS the store's live table, so everything that reads it keeps working.
        self.store = ParamStore(DEFAULT_PARAMS, path=getattr(args, "param_file", None))
        self._boot_notes = [f"Params: {text}" for text in self.store.load()]
        self.params = self.store.values
        self._param_names = sorted(self.params)
        self._write_error_reported = False

        self.nucleo: NucleoLink | None = None
        if args.nucleo_port:
            self.nucleo = NucleoLink(args.nucleo_port, args.nucleo_baud)
            # The STM keeps its own copy of the RC map in flash and the Pi cannot read it
            # back. Pushing the Pi's DEFAULTS at every boot would overwrite a calibration
            # made earlier (this is exactly what used to happen), so only push values the
            # operator actually saved on this Pi.
            if self.store.has_saved("stm"):
                self._push_rc_map()
            else:
                self.log("no saved RC map on the Pi: the Nucleo keeps the one in its flash")

        self._rc_calibrating = False

        self.rtcm: RtcmForwarder | None = None
        if args.gnss_port:
            self.rtcm = RtcmForwarder(args.gnss_port, args.gnss_baud)

        # Heading, position and velocity come from the estimator, fed every
        # loop with all IMU samples and the newest GNSS measurements.
        self.estimator = Estimator()
        self._display_heading_deg = 0.0   # last valid heading, for fields that cannot say "unknown"

        self._motor_test: tuple[int, int, float] | None = None  # motor, pulse_us, end_t

        # Set by DO_CHANGE_SPEED; None means "use CRUISE_SPEED".
        self.target_speed_mps: float | None = None

        # -1 = no telemetry seen yet, so the first valid reading applies the
        # switch's current band (ArduPilot does the same at boot).
        self._last_mode_slot = -1
        self._last_arm_state: int | None = None
        self._last_arm_block: int | None = None

        self.failsafe = FailsafeMonitor()
        self.geofence = Geofence()
        # Health table (bahr_pilot/diagnostics.py): a REPORT (statustext, SYS_STATUS bits), never an action.
        self.health = HealthMonitor(self._health_parts())
        self.loop_timer = LoopTimer()
        self._last_depth_t: float | None = None                     # when the newest sounding arrived
        self._recent_depth_bad: collections.deque = collections.deque(maxlen=5)   # last soundings not GOOD
        # Soundings: filtered (sonar.py), then turned into position-stamped, quality-classified
        # samples by distance travelled (bathymetry.py). The newest ones are kept here; Phase 25
        # writes them to the mission log.
        self.sonar = SonarFilter(self._sonar_config())
        self.bathymetry = BathymetryRecorder(self._bathy_config())
        self.bathy_samples: collections.deque = collections.deque(maxlen=2000)
        self._depth_to_send: float | None = None
        self._fence_items: list[tuple[int, float, float, float]] = []   # as uploaded, for download
        self._fence_upload: list[tuple[int, float, float, float]] | None = None
        self._fence_expected = 0

        self.data_logger: DataLogger | None = None
        # One folder per mission (armed -> disarmed): bathymetry, track, events, a summary
        # (bahr_pilot/missionlog.py). Like the raw log it only exists with --log-dir.
        self.mission_log: MissionLog | None = None
        self._mission_open = False
        self._log_error_reported = False
        if args.log_dir:
            self.data_logger = DataLogger(args.log_dir)
            self.log(f"logging raw sensor data to {self.data_logger.path}")
            self.mission_log = MissionLog(Path(args.log_dir) / "missions")

        self._boot_t = time.time()
        self._last_telemetry = 0.0

        # ONE GnssState shared by both readers: it decides which source's
        # position counts. (They used to get one each and overwrote each
        # other's values in the shared vehicle state, so position and heading
        # flapped between the RTD100's RTK fix and the echoMAP's weak GPS.)
        self.gnss = GnssState()
        self._last_gnss_source = ""
        if args.gnss_port:
            threading.Thread(
                target=run_rtd100_reader,
                args=(args.gnss_port, args.gnss_baud, self.gnss, self.state),
                daemon=True,
            ).start()
        if args.echomap_port:
            threading.Thread(
                target=run_echomap_reader,
                args=(args.echomap_port, args.echomap_baud, self.gnss, self.state),
                daemon=True,
            ).start()

    # -- helpers -------------------------------------------------------

    def log(self, message: str) -> None:
        print(f"[vehicle] {message}", flush=True)

    def _ms(self) -> int:
        return int((time.time() - self._boot_t) * 1000)

    def _ack(self, command: int, result: int) -> None:
        self.link.mav.command_ack_send(command, result)

    def _statustext(self, text: str, severity: int = 6) -> None:
        self.link.mav.statustext_send(severity, text.encode("ascii", "replace")[:50])
        self.log(text)
        self._log_event(severity, "vehicle", text)

    def _log_event(self, severity: int, source: str, text: str) -> None:
        log = getattr(self, "mission_log", None)         # statustexts can be sent before __init__ is done
        if log is not None and log.active:
            log.event(severity, source, text)

    def set_mode(self, mode: int, *, from_rc: bool = False) -> bool:
        """True if the vehicle is in `mode` afterwards."""
        if mode == self.state.mode:
            return True
        # While the RC mode switch sits in a MANUAL band the STM is driving
        # the motors from the sticks and nothing here can take that away, so
        # refuse (rather than pretend to) any other mode until the switch
        # moves. `from_rc` is the switch itself changing the mode.
        if not from_rc and mode != MODE_MANUAL and self._rc_in_manual():
            self._statustext("Mode refused: RC mode switch is in MANUAL", 4)
            return False
        # RTL has nowhere to go without a home position (ArduRover refuses too).
        if mode == MODE_RTL and self.state.home_lat is None:
            self._statustext("RTL refused: no home position", 4)
            return False
        if mode == MODE_RTL and not self._rtl_path_clear():
            self._statustext("RTL refused: path to home crosses the fence", 4)
            return False
        # Slot 0 is home, never a target. Entering AUTO from BAHR-GCS's mode
        # buttons (not MISSION_START) would otherwise sit on seq 0 and do
        # nothing; ArduRover starts from item 1 in that case too.
        if mode == MODE_AUTO and self.navigator.mission_seq < 1:
            self.navigator.mission_seq = 1
        self.state.mode = mode
        self.log(f"mode -> {MODE_NAMES.get(mode, mode)}")
        self._log_event(6, "mode", f"mode -> {MODE_NAMES.get(mode, mode)}")
        return True

    def _rtl_path_clear(self) -> bool:
        """RTL runs straight home: refuse it if that line crosses the fence (the
        failsafe then holds instead of driving across a no-go area)."""
        pose = self.state.pose
        if pose is None or not pose.position_valid or self.state.home_lat is None:
            return True                       # nothing to check against; the home check handles the rest
        return self.geofence.path_is_clear((pose.lat, pose.lon), (self.state.home_lat, self.state.home_lon),
                                           self.params)

    # -- RC mode switch ------------------------------------------------------

    def _rc_telemetry_fresh(self) -> bool:
        if self.nucleo is None:
            return False
        t = self.nucleo.telemetry
        return t.last_update > 0 and (time.monotonic() - t.last_update) < RC_TELEMETRY_FRESH_S

    def _rc_in_manual(self) -> bool:
        """True while the STM is driving the motors from the sticks."""
        return (self._rc_telemetry_fresh() and self.nucleo.telemetry.rc_link_up
                and self.nucleo.telemetry.override_active)

    def _manual_slot_mask(self) -> int:
        """Bit n set = mode-switch band n+1 is MANUAL (MODEn == MANUAL)."""
        mask = 0
        for index, name in enumerate(_MODE_SLOT_PARAMS):
            if int(self.params[name]) == MODE_MANUAL:
                mask |= 1 << index
        return mask

    def _report_gnss_source(self) -> None:
        """The echoMAP's own GPS is ~100x less accurate than the RTD100's:
        say so when the position source changes, in either direction."""
        source = self.state.gnss_source
        if source == self._last_gnss_source:
            return
        previous, self._last_gnss_source = self._last_gnss_source, source
        if source == "echomap":
            self._statustext("GNSS: RTD100 lost, using echoMAP GPS (low accuracy)", 4)
        elif source == "rtd100" and previous == "echomap":
            self._statustext("GNSS: RTD100 fix restored")
        elif source == "" and previous:
            self._statustext("GNSS: no usable fix", 4)

    def _report_rc_arming(self) -> None:
        """Tell the operator what the Nucleo's arm state machine did, once per
        change — a flipped switch that silently does nothing is the worst
        possible UX on a boat."""
        if not self._rc_telemetry_fresh():
            return
        t = self.nucleo.telemetry
        if t.arm_state != self._last_arm_state:
            if t.arm_state == ARM_STATE_ARMED:
                self._statustext("Armed (RC switch)")
            elif self._last_arm_state == ARM_STATE_ARMED:
                self._statustext("Disarmed (RC)")
            self._last_arm_state = t.arm_state
        if t.arm_block != self._last_arm_block:
            self._last_arm_block = t.arm_block
            text = PREARM_TEXT.get(t.arm_block)
            if text is not None and t.arm_state != ARM_STATE_ARMED:
                self._statustext(text, 4)

    def _apply_rc_mode_switch(self) -> None:
        """ArduPilot-style mode switch: when the switch moves to another band
        (and once at startup), the mode that band's MODEn parameter names
        becomes the vehicle mode; BAHR-GCS can still change it afterwards.

        MANUAL bands are special — the STM, not this process, decides them —
        so the vehicle mode simply follows the hardware: while the STM says
        the sticks are driving, the mode is MANUAL."""
        if not self._rc_telemetry_fresh():
            return
        t = self.nucleo.telemetry
        if t.rc_link_up and t.override_active:
            self.set_mode(MODE_MANUAL, from_rc=True)

        slot = t.mode_slot if t.rc_link_up else 0
        if slot == self._last_mode_slot:
            return
        self._last_mode_slot = slot
        if slot == 0:
            return  # signal lost: keep the mode; the STM already stops the motors
        mode = int(self.params[f"MODE{slot}"])
        if mode not in MODE_NAMES:
            self._statustext(f"RC switch: MODE{slot}={mode} is not supported", 4)
            return
        if self.set_mode(mode, from_rc=True):
            self._statustext(f"Mode {MODE_NAMES[mode]} (RC switch)")

    # -- incoming messages -----------------------------------------------

    def handle(self, msg) -> None:
        self.state.gcs_last_seen = time.monotonic()
        kind = msg.get_type()
        handler = getattr(self, f"_on_{kind.lower()}", None)
        if handler is None:
            return
        try:
            handler(msg)
        except Exception as exc:
            self.log(f"handler error on {kind}: {type(exc).__name__}: {exc}")

    def _on_command_long(self, msg) -> None:
        m = mavutil.mavlink
        location_commands = {m.MAV_CMD_DO_SET_HOME, m.MAV_CMD_DO_REPOSITION}
        scale = 1e7 if msg.command in location_commands else 1
        x = int(msg.param5 * scale)
        y = int(msg.param6 * scale)
        self._handle_command(msg.command, msg.param1, msg.param2, msg.param3,
                              msg.param4, x, y)

    def _on_command_int(self, msg) -> None:
        self._handle_command(msg.command, msg.param1, msg.param2, msg.param3,
                              msg.param4, msg.x, msg.y)

    def _handle_command(self, command, p1, p2, p3, p4, x, y) -> None:
        m = mavutil.mavlink
        accepted, denied = m.MAV_RESULT_ACCEPTED, m.MAV_RESULT_DENIED

        if command == m.MAV_CMD_DO_SET_MODE:
            ok = self.set_mode(int(p2))
            self._ack(command, accepted if ok else denied)

        elif command == m.MAV_CMD_COMPONENT_ARM_DISARM:
            want = p1 >= 0.5
            # MANUAL doesn't navigate, so it doesn't need a GPS fix to arm —
            # every other mode (AUTO/GUIDED/RTL/LOITER) does, since they all
            # need to know where the boat is.
            if want and self.state.mode != MODE_MANUAL and self.state.fix_type < 3:
                self._statustext("PreArm: need GPS fix", 4)
                self._ack(command, denied)
                return
            if want and self.state.mode == MODE_AUTO and not self.navigator.mission:
                self._statustext("PreArm: Mode not armable (AUTO with no mission)", 4)
                self._ack(command, denied)
                return
            self.state.armed = want
            self.log("ARMED" if want else "DISARMED")
            self._ack(command, accepted)

        elif command == m.MAV_CMD_DO_REPOSITION:
            change_mode = int(p2) & 1
            if self.state.mode != MODE_GUIDED and not change_mode:
                self._ack(command, denied)
                return
            if not self.set_mode(MODE_GUIDED):
                self._ack(command, denied)
                return
            self.navigator.guided_target = (x / 1e7, y / 1e7)
            self.log(f"go-to target {x / 1e7:.7f}, {y / 1e7:.7f}")
            self._ack(command, accepted)

        elif command == m.MAV_CMD_MISSION_START:
            if not self.navigator.mission:
                self._ack(command, denied)
                return
            if not self.set_mode(MODE_AUTO):
                self._ack(command, denied)
                return
            self.navigator.mission_seq = 1
            self.navigator.mission_paused = False
            self._ack(command, accepted)

        elif command == m.MAV_CMD_DO_PAUSE_CONTINUE:
            self.navigator.mission_paused = p1 < 0.5
            self._ack(command, accepted)

        elif command == m.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            ok = self.set_mode(MODE_RTL)
            self._ack(command, accepted if ok else denied)

        elif command == m.MAV_CMD_DO_CHANGE_SPEED:
            # param2 is a speed in m/s (BAHR-GCS sends param1=1 ground
            # speed, e.g. 1.5) — not a percentage. -1/0 means "no change".
            if p2 > 0:
                self.target_speed_mps = float(p2)
                self.log(f"target speed -> {self.target_speed_mps:.2f} m/s")
            self._ack(command, accepted)

        elif command == m.MAV_CMD_DO_SET_HOME:
            with self.state.lock:
                if p1 >= 0.5 and self.state.lat is not None:
                    self.state.home_lat, self.state.home_lon = self.state.lat, self.state.lon
                elif x or y:
                    self.state.home_lat, self.state.home_lon = x / 1e7, y / 1e7
            self._ack(command, accepted)

        elif command == m.MAV_CMD_PREFLIGHT_CALIBRATION:
            # Only the RC-calibration branch applies — no accelerometer or
            # compass calibration routine exists (no BNO086 driver yet).
            # Mirrors sim/fake_vehicle.py's exact p4 start/stop contract so
            # BAHR-GCS's RadioPage (which speaks only that contract) works
            # unchanged against the real vehicle.
            if p4 > 0 or (p4 == 0 and self._rc_calibrating and x == 0 and p1 == 0 and p2 == 0):
                self._rc_calibrating = p4 > 0
                self._statustext("RC calibration started" if self._rc_calibrating
                                  else "RC calibration finished")
                self._ack(command, accepted)
                return
            self._ack(command, m.MAV_RESULT_UNSUPPORTED)

        elif command == m.MAV_CMD_DO_MOTOR_TEST:
            motor = int(p1)
            percent = max(0.0, min(100.0, p3))
            timeout_s = max(0.1, p4)
            pulse = PULSE_NEUTRAL_US + int(5.0 * percent)  # 0-100% -> +0..+500us
            self._motor_test = (motor, pulse, time.time() + timeout_s)
            self._statustext(f"Motor {motor} test at {percent:g}% for {timeout_s:g}s")
            self._ack(command, accepted)

        elif command == m.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            # Not implemented — answering ACCEPTED (as an earlier version did)
            # made BAHR-GCS report a reboot that never happened.
            self._statustext("Reboot not supported yet", 4)
            self._ack(command, m.MAV_RESULT_UNSUPPORTED)

        elif command == m.MAV_CMD_PREFLIGHT_STORAGE:
            self._ack(command, self._preflight_storage(int(p1)))

        else:
            self._ack(command, m.MAV_RESULT_UNSUPPORTED)

    # -- mission protocol (slot 0 = home, mirrors sim/fake_vehicle.py) ---
    #
    # MISSION_COUNT / ITEM_INT / REQUEST_* / CLEAR_ALL carry a mission_type: 0 the
    # mission, 1 the geofence, 2 rally points. Until Phase 21 this code ignored
    # it, so a fence uploaded from QGC or Mission Planner would have OVERWRITTEN
    # the mission. Type 0 behaves exactly as before (including no reply to
    # CLEAR_ALL: BAHR-GCS clears and then counts every MISSION_ACK it sees).

    @staticmethod
    def _mission_type(msg) -> int:
        return int(getattr(msg, "mission_type", MISSION_TYPE_MISSION))

    def _mission_ack(self, result: int, mission_type: int) -> None:
        self.link.mav.mission_ack_send(255, 0, result, mission_type)

    def _on_mission_clear_all(self, msg) -> None:
        kind = self._mission_type(msg)
        if kind in (MISSION_TYPE_MISSION, MISSION_TYPE_ALL):
            self.navigator.mission.clear()
            self.navigator.mission_seq = 0
        if kind in (MISSION_TYPE_FENCE, MISSION_TYPE_ALL):
            self._clear_fence()
        if kind != MISSION_TYPE_MISSION:
            self._mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED, kind)

    def _clear_fence(self) -> None:
        self.geofence.clear()
        self._fence_items = []
        self._fence_upload = None

    def _on_mission_count(self, msg) -> None:
        kind = self._mission_type(msg)
        if kind == MISSION_TYPE_FENCE:
            self._fence_upload = []
            self._fence_expected = msg.count
            if msg.count == 0:
                self._finish_fence_upload()
            else:
                self.link.mav.mission_request_int_send(255, 0, 0, MISSION_TYPE_FENCE)
            return
        if kind != MISSION_TYPE_MISSION:      # rally points and anything else: refuse, do not misfile
            self._mission_ack(mavutil.mavlink.MAV_MISSION_UNSUPPORTED, kind)
            return
        self._expected_items = msg.count
        self._uploading = True
        self.navigator.mission = [(0.0, 0.0)] * msg.count
        self.link.mav.mission_request_int_send(255, 0, 0)

    def _on_mission_item_int(self, msg) -> None:
        kind = self._mission_type(msg)
        if kind == MISSION_TYPE_FENCE:
            self._on_fence_item(msg)
            return
        if kind != MISSION_TYPE_MISSION:
            return
        if not getattr(self, "_uploading", False):
            return
        if msg.seq < len(self.navigator.mission):
            self.navigator.mission[msg.seq] = (msg.x / 1e7, msg.y / 1e7)
        nxt = msg.seq + 1
        if nxt < self._expected_items:
            self.link.mav.mission_request_int_send(255, 0, nxt)
        else:
            self._uploading = False
            self.link.mav.mission_ack_send(255, 0, mavutil.mavlink.MAV_MISSION_ACCEPTED)
            self.log(f"mission stored: {len(self.navigator.mission)} items (slot 0 = home)")
            self._warn_waypoints_outside_fence()

    def _on_fence_item(self, msg) -> None:
        if self._fence_upload is None:
            return
        if msg.seq != len(self._fence_upload):
            self._fence_upload = None
            self._mission_ack(mavutil.mavlink.MAV_MISSION_INVALID_SEQUENCE, MISSION_TYPE_FENCE)
            return
        self._fence_upload.append((int(msg.command), float(msg.param1), msg.x / 1e7, msg.y / 1e7))
        if len(self._fence_upload) < self._fence_expected:
            self.link.mav.mission_request_int_send(255, 0, len(self._fence_upload), MISSION_TYPE_FENCE)
        else:
            self._finish_fence_upload()

    def _finish_fence_upload(self) -> None:
        items, self._fence_upload = self._fence_upload or [], None
        try:
            self.geofence.load(items)
        except FenceError as exc:
            self._statustext(f"Fence refused: {exc}", 4)
            self._mission_ack(mavutil.mavlink.MAV_MISSION_INVALID, MISSION_TYPE_FENCE)
            return
        self._fence_items = items
        self._mission_ack(mavutil.mavlink.MAV_MISSION_ACCEPTED, MISSION_TYPE_FENCE)
        self.log(f"fence stored: {len(items)} items")
        if items and self.params["FENCE_ENABLE"] < 0.5:
            self._statustext("Fence stored but FENCE_ENABLE is 0", 4)
        self._warn_waypoints_outside_fence()

    def _warn_waypoints_outside_fence(self) -> None:
        """A mission that leaves the fence will trigger it; say so at upload time."""
        home = (self.state.home_lat, self.state.home_lon) if self.state.home_lat is not None else None
        outside = self.geofence.waypoints_outside(self.navigator.mission, home, self.params)
        for index, margin in outside[:3]:
            self._statustext(f"Fence: waypoint {index} is {-margin:.0f} m outside", 4)
        if len(outside) > 3:
            self._statustext(f"Fence: {len(outside) - 3} more waypoints outside", 4)

    def _on_mission_request_list(self, msg) -> None:
        kind = self._mission_type(msg)
        if kind == MISSION_TYPE_FENCE:
            self.link.mav.mission_count_send(255, 0, len(self._fence_items), MISSION_TYPE_FENCE)
        elif kind == MISSION_TYPE_MISSION:
            self.link.mav.mission_count_send(255, 0, len(self.navigator.mission))
        else:
            self.link.mav.mission_count_send(255, 0, 0, kind)

    def _on_mission_request_int(self, msg) -> None:
        kind = self._mission_type(msg)
        if kind == MISSION_TYPE_FENCE:
            if msg.seq < len(self._fence_items):
                command, param1, lat, lon = self._fence_items[msg.seq]
                self.link.mav.mission_item_int_send(
                    255, 0, msg.seq, mavutil.mavlink.MAV_FRAME_GLOBAL, command, 0, 1,
                    param1, 0.0, 0.0, 0.0, int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0, MISSION_TYPE_FENCE)
            return
        if kind != MISSION_TYPE_MISSION:
            return
        if getattr(self, "_uploading", False) or msg.seq >= len(self.navigator.mission):
            return
        lat, lon = self.navigator.mission[msg.seq]
        self.link.mav.mission_item_int_send(
            255, 0, msg.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 2, 0, float("nan"),
            int(lat * 1e7), int(lon * 1e7), 0,
        )

    def _on_set_position_target_global_int(self, msg) -> None:
        self.set_mode(MODE_GUIDED)
        self.navigator.guided_target = (msg.lat_int / 1e7, msg.lon_int / 1e7)

    # -- parameter protocol (small real set, see DEFAULT_PARAMS) ---------

    def _send_param(self, name: str, index: int) -> None:
        self.link.mav.param_value_send(
            name.encode("ascii"), float(self.params[name]),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32, len(self._param_names), index,
        )

    def _on_param_request_list(self, msg) -> None:
        del msg
        for index, name in enumerate(self._param_names):
            self._send_param(name, index)

    @staticmethod
    def _param_name(raw) -> str:
        if isinstance(raw, bytes):
            raw = raw.decode("ascii", "replace")
        return str(raw).rstrip("\x00")

    def _on_param_request_read(self, msg) -> None:
        name = self._param_name(msg.param_id)
        index = int(msg.param_index)
        if 0 <= index < len(self._param_names):
            self._send_param(self._param_names[index], index)
        elif name in self.params:
            self._send_param(name, self._param_names.index(name))

    def _on_param_set(self, msg) -> None:
        name = self._param_name(msg.param_id)
        if name not in self.params:
            return
        result = self.store.set(name, float(msg.param_value))
        if not result.accepted:
            # keep the old value and say so: echoing it makes the ground station show
            # what the vehicle really holds instead of what was asked for
            self._statustext(f"Param {name} rejected: {result.reason}", 4)
        self._send_param(name, self._param_names.index(name))
        if result.accepted and name in _RC_MAP_PARAMS:
            self._push_rc_map()

    def _save_params(self, now: float | None = None, force: bool = False) -> None:
        """Write pending parameter changes (after a short quiet period unless forced)
        and tell the operator once if the disk will not take them."""
        wrote = self.store.flush(force=force)
        if self.store.write_error is not None and not self._write_error_reported:
            self._statustext("Params: cannot save, changes will be lost on restart", 4)
            self._write_error_reported = True
        elif wrote:
            self._write_error_reported = False

    def _preflight_storage(self, action: int) -> int:
        """MAV_CMD_PREFLIGHT_STORAGE param1: 0 reload from file, 1 save now, 2 reset to defaults."""
        m = mavutil.mavlink
        if action == 1:
            if self.store.path is None:
                self._statustext("Params: no parameter file configured", 4)
                return m.MAV_RESULT_DENIED
            self.store.flush(force=True)
            return m.MAV_RESULT_FAILED if self.store.write_error is not None else m.MAV_RESULT_ACCEPTED
        if action in (0, 2):
            if action == 2:
                self.store.reset()
                self._statustext("Params: reset to defaults")
            else:
                for note in self.store.reload():
                    self._statustext(f"Params: {note}", 4)
            if self.nucleo is not None:
                self._push_rc_map()
            return m.MAV_RESULT_ACCEPTED
        return m.MAV_RESULT_UNSUPPORTED

    def _channel_calibration(self, channel_number: int) -> tuple[int, int, int, bool]:
        return (
            int(self.params.get(f"RC{channel_number}_MIN", _SBUS_CH_MIN)),
            int(self.params.get(f"RC{channel_number}_MAX", _SBUS_CH_MAX)),
            int(self.params.get(f"RC{channel_number}_TRIM", _SBUS_CH_MID)),
            self.params.get(f"RC{channel_number}_REVERSED", 0.0) > 0.5,
        )

    def _push_rc_map(self) -> None:
        """Sends the current RCMAP_*/RCn_* parameters down to the Nucleo —
        called once at startup and again whenever BAHR-GCS writes one of
        them (via PARAM_SET, e.g. from the parameter page or RadioPage's
        calibration write)."""
        if self.nucleo is None:
            return
        throttle_ch = int(self.params["RCMAP_THROTTLE"])
        steering_ch = int(self.params["RCMAP_ROLL"])
        mode_ch = int(self.params["MODE_CH"])
        t_min, t_max, t_trim, t_rev = self._channel_calibration(throttle_ch)
        s_min, s_max, s_trim, s_rev = self._channel_calibration(steering_ch)
        m_min, m_max, _m_trim, _m_rev = self._channel_calibration(mode_ch)
        self.nucleo.send_rc_map(
            throttle_channel=throttle_ch - 1,
            steering_channel=steering_ch - 1,
            arm_channel=int(self.params["RCMAP_ARM"]) - 1,
            mode_channel=mode_ch - 1,
            throttle_min=t_min, throttle_max=t_max, throttle_trim=t_trim,
            steering_min=s_min, steering_max=s_max, steering_trim=s_trim,
            throttle_reversed=t_rev, steering_reversed=s_rev,
            mode_min=m_min, mode_max=m_max,
            manual_slot_mask=self._manual_slot_mask(),
        )

    # -- RC override (GCS joystick) and RTCM ------------------------------

    def _on_rc_channels_override(self, msg) -> None:
        def axis(raw):
            if raw in (0, 0xFFFF):
                return None
            return max(-1.0, min(1.0, (raw - 1500) / 400.0))

        self._rc_steering = axis(msg.chan1_raw)
        self._rc_throttle = axis(msg.chan3_raw)
        self._rc_last_seen = time.time()

    def _on_gps_rtcm_data(self, msg) -> None:
        if self.rtcm is not None:
            self.rtcm.handle_fragment(msg.flags, msg.len, bytes(msg.data))

    # -- IMU ---------------------------------------------------------------

    @staticmethod
    def _dop_cents(dop: float | None) -> int:
        """GPS_RAW_INT eph/epv: DOP x 100, 65535 = unknown."""
        return 65535 if dop is None else max(0, min(65534, int(dop * 100)))

    def _imu_fresh(self) -> bool:
        imu = self.nucleo.imu if self.nucleo is not None else None
        return imu is not None and (time.monotonic() - imu.rx_time) < IMU_FRESH_S

    def _attitude(self) -> tuple[float, float, float, float, float]:
        """(roll, pitch, rollspeed, pitchspeed, yawspeed), radians and rad/s,
        in the VEHICLE frame (AHRS_ORIENTATION applied). All zero when there
        is no fresh IMU data, and a field the IMU marks invalid stays zero
        rather than being passed on stale."""
        if not self._imu_fresh():
            return 0.0, 0.0, 0.0, 0.0, 0.0
        imu = self.nucleo.imu
        orientation = int(self.params["AHRS_ORIENTATION"])
        roll = pitch = 0.0
        if imu.quat_valid:
            roll, pitch, _ = quat_to_euler(vehicle_orientation(imu.quat, orientation))
        rates = (0.0, 0.0, 0.0)
        if imu.gyro_valid:
            rates = vehicle_vector(imu.gyro, orientation)
        return roll, pitch, rates[0], rates[1], rates[2]

    # -- battery -----------------------------------------------------------

    def _battery_status(self) -> tuple[int, int]:
        """(voltage_mv, battery_remaining_pct) for sys_status_send — MAVLink
        'unknown' sentinels (65535 / -1) unless the Nucleo has a fresh,
        valid battery reading. battery_remaining is always -1 (unknown):
        estimating a percentage needs a known pack capacity/cell count
        (BATT_CAPACITY in gcs/param_meta.py's real ArduPilot sense), which
        this vehicle doesn't track — only the raw voltage is real here."""
        if self.nucleo is None or not self.nucleo.telemetry.battery_valid:
            return 65535, -1
        return self.nucleo.telemetry.battery_mv, -1

    # -- failsafe ----------------------------------------------------------

    def _update_failsafes(self, now: float | None = None) -> None:
        """Evaluate the failsafe rules and enforce what they require. Called
        once per loop, before the motor command is computed. `now` is a
        monotonic clock (the simulator passes its own)."""
        now = time.monotonic() if now is None else now
        pose = self.state.pose
        seen = self.state.gcs_last_seen
        inputs = Inputs(
            now=now, armed=self.state.armed, mode=self.state.mode,
            gcs_age_s=(now - seen) if seen > 0.0 else None,    # same clock as `now`; 0.0 = never heard
            battery_v=self._battery_volts(),
            position_ok=pose is not None and pose.position_valid,
            heading_ok=pose is not None and pose.heading_valid,
            fence_margin_m=self._fence_margin(pose),
        )
        for event in self.failsafe.update(inputs, self.params):
            self._statustext(event.text, 2 if event.kind == "triggered" else 6)
        self._enforce_failsafe()

    def _battery_volts(self) -> float | None:
        telemetry = self.nucleo.telemetry if self.nucleo is not None else None
        return telemetry.battery_mv / 1000.0 if telemetry is not None and telemetry.battery_valid else None

    # -- health table ------------------------------------------------------

    def _health_parts(self) -> set[Part]:
        """The parts this vehicle has: everything below the STM exists only with an STM, the
        sonar only when its port was given. (Tests that fit a fake STM afterwards rebuild the monitor.)"""
        parts = {Part.GNSS, Part.HEADING, Part.GCS, Part.STORAGE, Part.LOOP}
        if self.nucleo is not None:
            parts |= {Part.STM, Part.IMU, Part.RC, Part.BATTERY}
        if self.args.echomap_port:
            parts.add(Part.SONAR)
        return parts

    def _health_snapshot(self, now: float) -> Snapshot:
        pose = self.state.pose
        nucleo = self.nucleo
        telemetry = nucleo.telemetry if nucleo is not None else None
        imu = nucleo.imu if nucleo is not None else None
        stm_age = (now - telemetry.last_update) if telemetry is not None and telemetry.last_update > 0 else None
        stm_fresh = stm_age is not None and stm_age <= RC_TELEMETRY_FRESH_S
        seen = self.state.gcs_last_seen
        log = self.mission_log
        return Snapshot(
            now=now,
            autonomous=self.state.mode in AUTONOMOUS_MODES,
            position=pose.status if pose is not None else None,
            heading_valid=pose is not None and pose.heading_valid,
            imu_age_s=(now - imu.rx_time) if imu is not None else None,
            imu_frames=nucleo.imu_frames if nucleo is not None else None,
            imu_data_valid=imu.gyro_valid if imu is not None else None,
            stm_age_s=stm_age,
            stm_resets=nucleo.clock.resets if nucleo is not None else 0,
            rc_link_up=telemetry.rc_link_up if stm_fresh else None,
            battery_v=self._battery_volts(),
            sonar_age_s=(now - self._last_depth_t) if self._last_depth_t is not None else None,
            sonar_suspect=sum(self._recent_depth_bad) >= SONAR_SUSPECT_OF_LAST,
            gcs_age_s=(now - seen) if seen > 0.0 else None,
            params_write_error=self.store.write_error,
            log_configured=log is not None,
            log_error=log.error if log is not None else None,
            loop_busy_max_s=self.loop_timer.max_busy_s(),
            failsafes=tuple(source.value for source in self.failsafe.active),
            fence_enabled=self.params["FENCE_ENABLE"] > 0.5,
            fence_breached=Source.GEOFENCE in self.failsafe.active,
        )

    def _update_health(self, now: float | None = None) -> None:
        """Update the health table and tell the operator what changed (after the failsafes, whose
        active causes it shows). `now` is the vehicle's monotonic clock (the simulator passes its own)."""
        now = time.monotonic() if now is None else now
        for transition in self.health.update(self._health_snapshot(now), self.params):
            self._statustext(transition.text, transition.severity)

    def _fence_margin(self, pose) -> float | None:
        """Signed distance to the fence, + inside; None when no fence applies or
        the position is not known (the position failsafe covers that case).

        Predictive: a boat cannot stop on the spot, so the fence is checked at
        the position AND at where the boat would come to rest if it stopped now,
        straight ahead along its velocity (reaction time FENCE_REACTION_S, then
        gliding to rest at FENCE_COAST_DECEL), and the smaller margin counts.
        Moving away from the fence therefore does not trigger it, moving towards
        it does so a stopping distance early instead of that far too late
        (measured in the simulator at 1.5 m/s: 2-6 m of overshoot without this;
        the stop is a GLIDE of ~4 m, not the 1 m an ATC_ACCEL_MAX brake would be)."""
        if pose is None or not pose.position_valid or self.params["FENCE_ENABLE"] < 0.5:
            return None
        home = (self.state.home_lat, self.state.home_lon) if self.state.home_lat is not None else None
        here = self.geofence.evaluate(pose.lat, pose.lon, home, self.params)
        if here is None:
            return None
        margin = here.margin_m
        speed = math.hypot(pose.velocity_east_mps, pose.velocity_north_mps)
        if speed > FENCE_MIN_PREDICT_SPEED_MPS:
            coast = max(self.params["FENCE_COAST_DECEL"], 0.05)
            stop = speed * FENCE_REACTION_S + speed * speed / (2.0 * coast)
            ahead = geo.LocalFrame(pose.lat, pose.lon).to_geodetic(
                pose.velocity_east_mps / speed * stop, pose.velocity_north_mps / speed * stop)
            predicted = self.geofence.evaluate(ahead[0], ahead[1], home, self.params)
            if predicted is not None:
                margin = min(margin, predicted.margin_m)
        return margin

    def _enforce_failsafe(self) -> None:
        """Level-triggered: for as long as a failsafe is active the vehicle
        may not stay in an autonomous mode that contradicts its action."""
        action = self.failsafe.required_action()
        if action is None or action == Action.REPORT:
            return
        if action == Action.TERMINATE and self.state.armed:
            self.state.armed = False
            self._statustext("Failsafe: disarmed", 2)
        if self.state.mode not in AUTONOMOUS_MODES:
            return
        if action == Action.RTL:
            if self.state.mode != MODE_RTL and not self.set_mode(MODE_RTL):
                self._statustext("Failsafe: RTL impossible, holding", 2)
                self.set_mode(MODE_HOLD)
        elif self.state.mode != MODE_HOLD:
            self.set_mode(MODE_HOLD)
            self._statustext("Failsafe: mode forced to HOLD", 2)

    # -- motor command + telemetry ---------------------------------------

    def _motor_command(self) -> tuple[int, int]:
        if self._motor_test is not None:
            motor, pulse, end_t = self._motor_test
            if time.time() < end_t:
                return (pulse, PULSE_NEUTRAL_US) if motor == 1 else (PULSE_NEUTRAL_US, pulse)
            self._motor_test = None

        if not self.state.armed or self.failsafe.stops_motors():
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US

        rc_fresh = time.time() - getattr(self, "_rc_last_seen", 0.0) < 1.5
        if self.state.mode == MODE_MANUAL and rc_fresh and getattr(self, "_rc_throttle", None) is not None:
            steering = getattr(self, "_rc_steering", 0.0) or 0.0
            throttle = self._rc_throttle
            span = (PULSE_MAX_US - PULSE_MIN_US) / 2
            # Motor 1 = LEFT; stick right (steering > 0) = left motor faster.
            m1 = PULSE_NEUTRAL_US + span * (throttle + steering)
            m2 = PULSE_NEUTRAL_US + span * (throttle - steering)
            return max(PULSE_MIN_US, min(PULSE_MAX_US, int(m1))), \
                max(PULSE_MIN_US, min(PULSE_MAX_US, int(m2)))

        self.navigator.configure(self.params)
        m1, m2, reached = self.navigator.step(
            self.state, self._cruise_fraction(), self.params["WP_RADIUS"], self._target_speed()
        )
        if reached:
            self._on_target_reached()
        return m1, m2

    def _target_speed(self) -> float:
        """The ground speed the speed controller regulates to, m/s."""
        speed = self.target_speed_mps if self.target_speed_mps is not None else self.params["CRUISE_SPEED"]
        return max(0.0, speed)

    def _cruise_fraction(self) -> float:
        """Forward throttle fraction (0..1) for the current target speed,
        from the CRUISE_SPEED/CRUISE_THROTTLE feed-forward pair."""
        cruise_speed = self.params["CRUISE_SPEED"]
        if cruise_speed <= 0:
            return 0.0
        speed = self.target_speed_mps if self.target_speed_mps is not None else cruise_speed
        fraction = (self.params["CRUISE_THROTTLE"] / 100.0) * (speed / cruise_speed)
        return max(0.0, min(1.0, fraction))

    def _on_target_reached(self) -> None:
        if self.state.mode == MODE_AUTO:
            self.link.mav.mission_item_reached_send(self.navigator.mission_seq)
            self.navigator.mission_seq += 1
            if self.navigator.mission_seq >= len(self.navigator.mission):
                self.set_mode(MODE_HOLD)
                self.navigator.mission_seq = max(0, len(self.navigator.mission) - 1)
        elif self.state.mode == MODE_GUIDED:
            self.navigator.guided_target = None
        elif self.state.mode == MODE_RTL:
            self.set_mode(MODE_HOLD)

    def _update_estimate(self) -> None:
        """Run the estimator on everything that arrived since the last loop
        and publish its pose. All times here are time.monotonic(), the clock
        GnssState and the IMU time sync use."""
        now = time.monotonic()
        samples = self.nucleo.take_imu_samples() if self.nucleo is not None else ()
        pose = self.estimator.update(
            now, samples, self.gnss.best_fix(now), self.gnss.best_heading(now),
            int(self.params["AHRS_ORIENTATION"]),
        )
        self.state.pose = pose
        if pose.heading_valid:
            self._display_heading_deg = pose.heading_deg

    def _sonar_config(self) -> SonarConfig:
        p = self.params
        low = p["RNGFND1_MIN"]
        return SonarConfig(min_depth_m=low, max_depth_m=max(p["RNGFND1_MAX"], low + 0.1),
                           offset_m=p["RNGFND1_OFFSET"], spike_abs_m=p["SONAR_SPIKE"])

    def _bathy_config(self) -> BathyConfig:
        p = self.params
        return BathyConfig(spacing_m=p["BATHY_SPACING"], max_h_acc_m=p["BATHY_MAX_HACC"],
                           max_speed_mps=p["BATHY_MAX_SPEED"], max_tilt_deg=p["BATHY_MAX_TILT"],
                           latency_s=p["SONAR_LATENCY"])

    def _update_bathymetry(self, now: float | None = None) -> None:
        """Run every new depth reading through the sonar filter and the sampler. Tuning
        changes take effect here (a changed sonar setting restarts the filter's history)."""
        now = time.monotonic() if now is None else now
        sonar_config = self._sonar_config()
        if sonar_config != self.sonar.cfg:
            self.sonar = SonarFilter(sonar_config)
        self.bathymetry.cfg = self._bathy_config()
        pose = self.state.pose
        for reading_time, raw in self.gnss.take_depth_readings():
            reading = self.sonar.update(reading_time, raw)
            self._last_depth_t = reading_time
            # a start-up reading is "not confirmed" only because the filter has nothing to compare it with
            self._recent_depth_bad.append(reading.quality != DepthQuality.GOOD and not reading.warming_up)
            if reading.quality != DepthQuality.BAD and not reading.warming_up:
                # the GCS gets filtered readings only, and not the first few after a start or a gap,
                # which the filter had nothing to judge against (one of them was a 1.5 m spike)
                self._depth_to_send = reading.depth_m
            if not self.state.armed:
                continue                                       # a boat on the bench does not survey
            sample = self.bathymetry.update(reading, pose, now, time.time(), self.state.gnss_quality)
            if sample is not None:
                self.bathy_samples.append(sample)
                if self.mission_log is not None:
                    self.mission_log.bathymetry(sample)

    def _mission_meta(self) -> dict:
        return {
            "software": {"autopilot": AUTOPILOT_VERSION, "bahr_link": BAHR_LINK_VERSION,
                         "mission_format": MISSION_FORMAT_VERSION},
            "params": dict(self.params),
            "params_changed": {name: value for name, value in self.params.items() if value != DEFAULT_PARAMS[name]},
            "args": {k: v for k, v in vars(self.args).items() if k != "log_dir"},
            "mode": MODE_NAMES.get(self.state.mode, self.state.mode),
            "home": [self.state.home_lat, self.state.home_lon],
            "mission_waypoints": len(self.navigator.mission),
            "health": self.health.table(),
        }

    def _update_mission_log(self, now: float | None = None) -> None:
        """Open a mission folder when the vehicle arms, close it when it disarms, and feed it
        the track while it is open. A disk error is reported once and never stops the loop."""
        log = self.mission_log
        if log is None:
            return
        now = time.monotonic() if now is None else now
        armed = self.state.armed
        if armed and not self._mission_open:
            self._mission_open = True
            self._log_error_reported = False
            folder = log.start(self._mission_meta())
            if folder is not None:
                self._statustext(f"Logging to {folder.name}")
        elif not armed and self._mission_open:
            self._mission_open = False
            self._log_event(6, "mission", "mission ended: disarmed")
            log.stop("disarmed")
        if self._mission_open and log.active:
            log.track(now, self.state.pose, self.state.mode)
        if log.error is not None and not self._log_error_reported:
            self._log_error_reported = True
            self.log(f"mission log failed: {log.error}")
            self.link.mav.statustext_send(4, b"Log: write failed, logging stopped")

    def _update_home(self) -> None:
        """Home = the first usable 3D-or-better fix. (The old code only looked
        at the very first loop iteration, so a first fix that was still 2D
        meant no home for the rest of the run.)"""
        if self.state.home_lat is None and self.state.lat is not None and self.state.fix_type >= 3:
            self.state.home_lat, self.state.home_lon = self.state.lat, self.state.lon

    def send_telemetry(self) -> None:
        m = mavutil.mavlink
        base_mode = m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.state.armed:
            base_mode |= m.MAV_MODE_FLAG_SAFETY_ARMED
        self.link.mav.heartbeat_send(
            m.MAV_TYPE_SURFACE_BOAT, m.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode, self.state.mode, m.MAV_STATE_ACTIVE,
        )

        now_ms = self._ms()
        pose = self.state.pose
        pos_ok = pose is not None and pose.position_valid
        head_ok = pose is not None and pose.heading_valid
        speed = pose.speed_mps if pos_ok else 0.0
        heading = pose.heading_deg if head_ok else self._display_heading_deg

        # GLOBAL_POSITION_INT is the FILTERED position (as on ArduPilot, where it
        # is the EKF output); GPS_RAW_INT is the receiver's own. hdg / cog
        # 65535 = unknown (BAHR-GCS keeps its last heading then, instead of
        # plotting a stale one as fact).
        hdg_cdeg = round(heading * 100) % 36000 if head_ok else 65535
        cog_cdeg = round(pose.course_deg * 100) % 36000 if pos_ok and pose.course_deg is not None else 65535
        vn_cms = round(pose.velocity_north_mps * 100) if pos_ok else 0
        ve_cms = round(pose.velocity_east_mps * 100) if pos_ok else 0
        self.link.mav.global_position_int_send(
            now_ms, int((pose.lat if pos_ok else 0.0) * 1e7), int((pose.lon if pos_ok else 0.0) * 1e7),
            int(self.state.alt_m * 1000), 0, vn_cms, ve_cms, 0, hdg_cdeg,
        )
        raw_lat = self.state.lat or 0.0
        raw_lon = self.state.lon or 0.0
        self.link.mav.gps_raw_int_send(
            now_ms * 1000, self.state.fix_type, int(raw_lat * 1e7), int(raw_lon * 1e7),
            int(self.state.alt_m * 1000), self._dop_cents(self.state.hdop), self._dop_cents(None),
            int(speed * 100), cog_cdeg, self.state.satellites,
        )
        self.link.mav.vfr_hud_send(speed, speed, int(heading), 0, self.state.alt_m, 0.0)

        roll, pitch, rollspeed, pitchspeed, yawspeed = self._attitude()
        # Yaw is the estimated heading (gyro carried, GNSS-corrected), never
        # the BNO086's own yaw (it has no magnetometer fusion, see
        # firmware/reflex/Core/Src/imu.c).
        self.link.mav.attitude_send(now_ms, roll, pitch, math.radians(heading),
                                     rollspeed, pitchspeed, yawspeed)

        voltage_mv, battery_pct = self._battery_status()
        status = self.health.sys_status(self._health_snapshot(time.monotonic()), self.loop_timer.load_permille())
        self.link.mav.sys_status_send(status.present, status.enabled, status.health, status.load,
                                      voltage_mv, -1, battery_pct, 0, 0, 0, 0, 0, 0)
        self.link.mav.mission_current_send(self.navigator.mission_seq)

        home = (self.state.home_lat, self.state.home_lon) if self.state.home_lat is not None else None
        target = self.navigator.active_target(self.state.mode, home)
        guidance = self.navigator.last_guidance
        if target is not None and pos_ok:
            wp_dist = int(geo.distance_m(pose.lat, pose.lon, *target))
            target_bearing = int(geo.bearing_deg(pose.lat, pose.lon, *target))
        else:
            wp_dist = 0
            target_bearing = 0
        # nav_bearing = where the follower wants the bow (the aim point on the
        # line); xtrack_error = metres off the line, + = right of it
        nav_bearing = int(guidance.desired_heading_deg) if guidance is not None else int(heading)
        xtrack = guidance.cross_track_m if guidance is not None else 0.0
        self.link.mav.nav_controller_output_send(
            0.0, 0.0, nav_bearing, target_bearing, wp_dist, 0.0, 0.0, xtrack
        )

        if self._depth_to_send is not None:
            # once per accepted sounding, filtered and offset: BAHR-GCS plots a point for every
            # DISTANCE_SENSOR it receives, so repeating the raw value at 5 Hz drew five copies of
            # each sounding and passed spikes straight onto the map
            self.link.mav.distance_sensor_send(
                now_ms, int(self.params["RNGFND1_MIN"] * 100), min(65535, int(self.params["RNGFND1_MAX"] * 100)),
                min(65535, max(0, round(self._depth_to_send * 100))),
                m.MAV_DISTANCE_SENSOR_ULTRASOUND, 1, m.MAV_SENSOR_ROTATION_PITCH_270, 0,
            )
            self._depth_to_send = None
        if self.state.water_temp_c is not None:
            self.link.mav.named_value_float_send(now_ms, b"water_temp", self.state.water_temp_c)

        if self.nucleo is not None and self.nucleo.telemetry.last_update > 0:
            ch = self.nucleo.telemetry.channels
            self.link.mav.rc_channels_send(now_ms, 16, *ch, 0, 0, 255)
        else:
            self.link.mav.rc_channels_send(now_ms, 0, *([0] * 18), 255)

    # -- main loop ---------------------------------------------------------

    def run(self) -> None:
        self.log(f"sending MAVLink to {self.args.gcs_host}:{self.args.gcs_port}")
        self.send_telemetry()  # prime the UDP socket (see sim/fake_vehicle.py note)
        self._statustext(banner())
        for note in self._boot_notes:
            self._statustext(note, 4)

        tick = 1.0 / TICK_HZ
        telemetry_interval = 1.0 / TELEMETRY_HZ

        while True:
            loop_start = time.monotonic()
            for _ in range(500):
                try:
                    msg = self.link.recv_match(blocking=False)
                except OSError:
                    break
                if msg is None:
                    break
                self.handle(msg)

            self.gnss.publish(self.state)  # expires stale data even when nothing arrives
            self._report_gnss_source()
            self._update_estimate()
            self._update_home()
            self._update_mission_log()      # before the soundings: the folder must exist for the first one
            self._update_bathymetry()
            self._update_failsafes()
            self._update_health()
            self._save_params()
            self._apply_rc_mode_switch()
            self._report_rc_arming()

            m1, m2 = self._motor_command()
            if self.nucleo is not None:
                self.nucleo.send_motors(m1, m2)

            now = time.time()
            if now - self._last_telemetry >= telemetry_interval:
                self._last_telemetry = now
                self.send_telemetry()
                if self.data_logger is not None:
                    telemetry = self.nucleo.telemetry if self.nucleo is not None else None
                    self.data_logger.log(self.state, telemetry)

            self.loop_timer.record(loop_start, time.monotonic() - loop_start)   # the work, not the sleep
            time.sleep(tick)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gcs-host", required=True, help="PC running BAHR-GCS")
    parser.add_argument("--gcs-port", type=int, default=14550)
    parser.add_argument("--nucleo-port", help="e.g. /dev/serial0 (Pi UART -> Nucleo USART3)")
    parser.add_argument("--nucleo-baud", type=int, default=115200)
    parser.add_argument("--gnss-port", help="RTD100, e.g. /dev/serial/by-id/...")
    parser.add_argument("--gnss-baud", type=int, default=115200)
    parser.add_argument("--echomap-port", help="echoMAP depth sounder")
    parser.add_argument("--echomap-baud", type=int, default=38400)
    parser.add_argument("--log-dir", help="write timestamped NDJSON sensor logs here (optional)")
    parser.add_argument("--param-file", default=str(Path.home() / ".bahr_pilot" / "params.json"),
                        help="where the tuned parameters are saved (default: %(default)s)")
    args = parser.parse_args()

    # systemd stops a service with SIGTERM, which would skip the final save below
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    vehicle = Vehicle(args)
    try:
        vehicle.run()
    except (KeyboardInterrupt, SystemExit):
        print("\n[vehicle] stopped")
    finally:
        vehicle._save_params(force=True)
        if vehicle.mission_log is not None:
            vehicle.mission_log.stop("shutdown")
        if vehicle.data_logger is not None:
            vehicle.data_logger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
