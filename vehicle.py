"""Real vehicle-side MAVLink process for BAHR — the Pi 4 "brain".

Speaks the same MAVLink wire format sim/fake_vehicle.py uses so BAHR-GCS
needs no changes to talk to it: UDP, connects out to the GCS on port 14550.
Where sim/fake_vehicle.py simulates sensors and motion to exercise the GCS,
this reads the real GNSS/depth sensors (bahr_pilot.sensors), drives the real
Nucleo over UART (bahr_pilot.nucleo_link), and forwards RTK corrections to
the GNSS (bahr_pilot.rtcm_forward) instead. The Nucleo firmware itself lives
alongside this package at bahr_pilot/firmware/reflex/ (an STM32CubeIDE
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
  - Low-battery failsafe (BATT_LOW_VOLT/BATT_FS_ENABLE params) stops the
    motors the same way a stale GCS link does, and defaults to OFF for
    exactly that reason — untested beyond unit-level param plumbing, for
    the same "no real battery/ADC" reason as the voltage reading itself.
"""
from __future__ import annotations

import argparse
import math
import threading
import time

from pymavlink import mavutil

from bahr_pilot.datalog import DataLogger
from bahr_pilot.modes import MODE_AUTO, MODE_GUIDED, MODE_HOLD, MODE_MANUAL, MODE_NAMES, MODE_RTL
from bahr_pilot.navigation import Navigator, bearing_deg, distance_m
from bahr_pilot.nucleo_link import NucleoLink, PULSE_MAX_US, PULSE_MIN_US, PULSE_NEUTRAL_US
from bahr_pilot.rtcm_forward import RtcmForwarder
from bahr_pilot.sensors import GnssState, run_echomap_reader, run_rtd100_reader
from bahr_pilot.state import VehicleState

TICK_HZ = 20.0
TELEMETRY_HZ = 5.0
GCS_LINK_TIMEOUT_S = 3.0
# How long the pack voltage must stay under threshold before the failsafe
# trips — a brief sag under load (e.g. a hard turn) shouldn't stop the boat.
BATT_FS_DEBOUNCE_S = 3.0

# Mirrors bahr_pilot/firmware/reflex/Core/Inc/sbus.h's SBUS_CH_MIN/MID/MAX — the
# default RCn_MIN/MAX/TRIM below assume no calibration offset, until
# BAHR-GCS's RadioPage overwrites them with real captured values.
_SBUS_CH_MIN = 172.0
_SBUS_CH_MID = 992.0
_SBUS_CH_MAX = 1811.0

# Real, vehicle-specific tunables — a small set, not ArduPilot's full
# parameter list (BAHR-GCS doesn't have a custom parameter page for this
# vehicle yet, see ROADMAP.md section 6; this is enough to not break the
# parameter protocol if something requests it).
DEFAULT_PARAMS: dict[str, float] = {
    "CRUISE_FRAC": 0.6,       # 0..1, fraction of PULSE span commanded at full speed
    "WP_RADIUS_M": 3.0,
    "GCS_FS_TIMEOUT_S": GCS_LINK_TIMEOUT_S,
    # Channel -> function mapping, 1-based channel numbers matching
    # ArduPilot Rover's own RCMAP_ROLL/RCMAP_THROTTLE convention (and
    # BAHR-GCS's existing parameter page, gcs/param_meta.py's "radio"
    # group). RCMAP_ARM/RCMAP_OVERRIDE aren't stock ArduPilot parameters
    # but follow the same naming pattern.
    "RCMAP_ROLL": 1.0,      # steering
    "RCMAP_THROTTLE": 3.0,
    "RCMAP_ARM": 6.0,
    "RCMAP_OVERRIDE": 5.0,
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
    "BATT_FS_ENABLE": 0.0,
}
for _ch in range(1, 9):
    DEFAULT_PARAMS[f"RC{_ch}_MIN"] = _SBUS_CH_MIN
    DEFAULT_PARAMS[f"RC{_ch}_MAX"] = _SBUS_CH_MAX
    DEFAULT_PARAMS[f"RC{_ch}_TRIM"] = _SBUS_CH_MID
    DEFAULT_PARAMS[f"RC{_ch}_REVERSED"] = 0.0

_RC_MAP_PARAMS = {"RCMAP_ROLL", "RCMAP_THROTTLE", "RCMAP_ARM", "RCMAP_OVERRIDE"}
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
        self.params = dict(DEFAULT_PARAMS)
        self._param_names = sorted(self.params)

        self.nucleo: NucleoLink | None = None
        if args.nucleo_port:
            self.nucleo = NucleoLink(args.nucleo_port, args.nucleo_baud)
            self._push_rc_map()

        self._rc_calibrating = False

        self.rtcm: RtcmForwarder | None = None
        if args.gnss_port:
            self.rtcm = RtcmForwarder(args.gnss_port, args.gnss_baud)

        self._prev_fix: tuple[float, float, float] | None = None  # lat, lon, t
        self._speed_mps = 0.0

        self._motor_test: tuple[int, int, float] | None = None  # motor, pulse_us, end_t

        self._batt_low_since: float | None = None
        self._batt_fs_warned = False

        self.data_logger: DataLogger | None = None
        if args.log_dir:
            self.data_logger = DataLogger(args.log_dir)
            self.log(f"logging raw sensor data to {self.data_logger.path}")

        self._boot_t = time.time()
        self._last_telemetry = 0.0

        if args.gnss_port:
            threading.Thread(
                target=run_rtd100_reader,
                args=(args.gnss_port, args.gnss_baud, GnssState(), self.state),
                daemon=True,
            ).start()
        if args.echomap_port:
            threading.Thread(
                target=run_echomap_reader,
                args=(args.echomap_port, args.echomap_baud, GnssState(), self.state),
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

    def set_mode(self, mode: int) -> None:
        if mode == self.state.mode:
            return
        self.state.mode = mode
        self.log(f"mode -> {MODE_NAMES.get(mode, mode)}")

    # -- incoming messages -----------------------------------------------

    def handle(self, msg) -> None:
        self.state.gcs_last_seen = time.time()
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
            self.set_mode(int(p2))
            self._ack(command, accepted)

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
            self.set_mode(MODE_GUIDED)
            self.navigator.guided_target = (x / 1e7, y / 1e7)
            self.log(f"go-to target {x / 1e7:.7f}, {y / 1e7:.7f}")
            self._ack(command, accepted)

        elif command == m.MAV_CMD_MISSION_START:
            if not self.navigator.mission:
                self._ack(command, denied)
                return
            self.navigator.mission_seq = 1
            self.navigator.mission_paused = False
            self.set_mode(MODE_AUTO)
            self._ack(command, accepted)

        elif command == m.MAV_CMD_DO_PAUSE_CONTINUE:
            self.navigator.mission_paused = p1 < 0.5
            self._ack(command, accepted)

        elif command == m.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.set_mode(MODE_RTL)
            self._ack(command, accepted)

        elif command == m.MAV_CMD_DO_CHANGE_SPEED:
            if p2 > 0:
                self.params["CRUISE_FRAC"] = max(0.05, min(1.0, p2 / 100.0))
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
            self._statustext("Reboot requested (not implemented)")
            self._ack(command, accepted)

        else:
            self._ack(command, m.MAV_RESULT_UNSUPPORTED)

    # -- mission protocol (slot 0 = home, mirrors sim/fake_vehicle.py) ---

    def _on_mission_clear_all(self, msg) -> None:
        del msg
        self.navigator.mission.clear()
        self.navigator.mission_seq = 0

    def _on_mission_count(self, msg) -> None:
        self._expected_items = msg.count
        self._uploading = True
        self.navigator.mission = [(0.0, 0.0)] * msg.count
        self.link.mav.mission_request_int_send(255, 0, 0)

    def _on_mission_item_int(self, msg) -> None:
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

    def _on_mission_request_list(self, msg) -> None:
        del msg
        self.link.mav.mission_count_send(255, 0, len(self.navigator.mission))

    def _on_mission_request_int(self, msg) -> None:
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
        self.params[name] = float(msg.param_value)
        self._send_param(name, self._param_names.index(name))
        if name in _RC_MAP_PARAMS:
            self._push_rc_map()

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
        t_min, t_max, t_trim, t_rev = self._channel_calibration(throttle_ch)
        s_min, s_max, s_trim, s_rev = self._channel_calibration(steering_ch)
        self.nucleo.send_rc_map(
            throttle_channel=throttle_ch - 1,
            steering_channel=steering_ch - 1,
            arm_channel=int(self.params["RCMAP_ARM"]) - 1,
            override_channel=int(self.params["RCMAP_OVERRIDE"]) - 1,
            throttle_min=t_min, throttle_max=t_max, throttle_trim=t_trim,
            steering_min=s_min, steering_max=s_max, steering_trim=s_trim,
            throttle_reversed=t_rev, steering_reversed=s_rev,
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

    def _battery_failsafe_active(self) -> bool:
        """Debounced low-voltage cutoff (BATT_FS_ENABLE/BATT_LOW_VOLT
        params) — mirrors the GCS-link-timeout failsafe's own 'default to
        stop' philosophy. Silently does nothing if there's no Nucleo or no
        valid battery reading yet, same as the rest of this project treats
        an absent sensor as 'can't fail safe on data we don't have', not
        as an error."""
        if self.params["BATT_FS_ENABLE"] <= 0.5:
            return False
        if self.nucleo is None or not self.nucleo.telemetry.battery_valid:
            return False
        threshold_mv = self.params["BATT_LOW_VOLT"] * 1000.0
        if self.nucleo.telemetry.battery_mv >= threshold_mv:
            self._batt_low_since = None
            self._batt_fs_warned = False
            return False

        now = time.time()
        if self._batt_low_since is None:
            self._batt_low_since = now
        if (now - self._batt_low_since) < BATT_FS_DEBOUNCE_S:
            return False
        if not self._batt_fs_warned:
            volts = self.nucleo.telemetry.battery_mv / 1000.0
            self._statustext(f"Battery low: {volts:.1f}V, motors stopped", severity=2)
            self._batt_fs_warned = True
        return True

    # -- motor command + telemetry ---------------------------------------

    def _motor_command(self) -> tuple[int, int]:
        if self._motor_test is not None:
            motor, pulse, end_t = self._motor_test
            if time.time() < end_t:
                return (pulse, PULSE_NEUTRAL_US) if motor == 1 else (PULSE_NEUTRAL_US, pulse)
            self._motor_test = None

        gcs_stale = (time.time() - self.state.gcs_last_seen) > self.params["GCS_FS_TIMEOUT_S"]
        if not self.state.armed or gcs_stale or self._battery_failsafe_active():
            return PULSE_NEUTRAL_US, PULSE_NEUTRAL_US

        rc_fresh = time.time() - getattr(self, "_rc_last_seen", 0.0) < 1.5
        if self.state.mode == MODE_MANUAL and rc_fresh and getattr(self, "_rc_throttle", None) is not None:
            steering = getattr(self, "_rc_steering", 0.0) or 0.0
            throttle = self._rc_throttle
            span = (PULSE_MAX_US - PULSE_MIN_US) / 2
            m1 = PULSE_NEUTRAL_US + span * (throttle - steering)
            m2 = PULSE_NEUTRAL_US + span * (throttle + steering)
            return max(PULSE_MIN_US, min(PULSE_MAX_US, int(m1))), \
                max(PULSE_MIN_US, min(PULSE_MAX_US, int(m2)))

        m1, m2, reached = self.navigator.step(self.state, self.params["CRUISE_FRAC"])
        if reached:
            self._on_target_reached()
        return m1, m2

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

    def _update_speed(self) -> None:
        if self.state.lat is None:
            return
        now = time.time()
        if self._prev_fix is not None:
            plat, plon, pt = self._prev_fix
            dt = now - pt
            if dt > 0.05:
                self._speed_mps = distance_m(plat, plon, self.state.lat, self.state.lon) / dt
                self._prev_fix = (self.state.lat, self.state.lon, now)
        else:
            self._prev_fix = (self.state.lat, self.state.lon, now)
            if self.state.home_lat is None and self.state.fix_type >= 3:
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
        lat = self.state.lat or 0.0
        lon = self.state.lon or 0.0
        heading = self.state.heading_deg

        self.link.mav.global_position_int_send(
            now_ms, int(lat * 1e7), int(lon * 1e7), int(self.state.alt_m * 1000), 0,
            0, 0, 0, int(heading * 100),
        )
        self.link.mav.gps_raw_int_send(
            now_ms * 1000, self.state.fix_type, int(lat * 1e7), int(lon * 1e7),
            int(self.state.alt_m * 1000), 65535, 65535, int(self._speed_mps * 100),
            int(heading * 100), self.state.satellites,
        )
        self.link.mav.vfr_hud_send(self._speed_mps, self._speed_mps, int(heading), 0,
                                    self.state.alt_m, 0.0)

        roll = pitch = 0.0
        if self.nucleo is not None and self.nucleo.telemetry.imu_valid:
            roll = math.radians(self.nucleo.telemetry.roll_deg)
            pitch = math.radians(self.nucleo.telemetry.pitch_deg)
        # Yaw is always GNSS heading, never the IMU's own (no magnetometer
        # fusion on the BNO086 side, see firmware/reflex/Core/Src/imu.c).
        self.link.mav.attitude_send(now_ms, roll, pitch, math.radians(heading), 0.0, 0.0, 0.0)

        voltage_mv, battery_pct = self._battery_status()
        self.link.mav.sys_status_send(0, 0, 0, 0, voltage_mv, -1, battery_pct, 0, 0, 0, 0, 0, 0)
        self.link.mav.mission_current_send(self.navigator.mission_seq)

        home = (self.state.home_lat, self.state.home_lon) if self.state.home_lat is not None else None
        target = self.navigator.active_target(self.state.mode, home)
        if target is not None and self.state.lat is not None:
            wp_dist = int(distance_m(self.state.lat, self.state.lon, *target))
            target_bearing = int(bearing_deg(self.state.lat, self.state.lon, *target))
        else:
            wp_dist = 0
            target_bearing = 0
        self.link.mav.nav_controller_output_send(
            0.0, 0.0, int(heading), target_bearing, wp_dist, 0.0, 0.0, 0.0
        )

        if self.state.depth_m is not None:
            self.link.mav.distance_sensor_send(
                now_ms, 20, 5000, int(self.state.depth_m * 100),
                m.MAV_DISTANCE_SENSOR_ULTRASOUND, 1, m.MAV_SENSOR_ROTATION_PITCH_270, 0,
            )
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

        tick = 1.0 / TICK_HZ
        telemetry_interval = 1.0 / TELEMETRY_HZ

        while True:
            for _ in range(500):
                try:
                    msg = self.link.recv_match(blocking=False)
                except OSError:
                    break
                if msg is None:
                    break
                self.handle(msg)

            self._update_speed()

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
    args = parser.parse_args()

    vehicle = Vehicle(args)
    try:
        vehicle.run()
    except KeyboardInterrupt:
        print("\n[vehicle] stopped")
    finally:
        if vehicle.data_logger is not None:
            vehicle.data_logger.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
