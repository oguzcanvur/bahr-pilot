# bahr_pilot

**BAHR** — *Bathymetric Autonomous Hydrographic Reconnaissance*

bahr_pilot is the custom autopilot for the BAHR survey boat: a Raspberry Pi 4
("brain") paired with an STM32 Nucleo-G431RB ("reflex") over UART, replacing
a Cube Orange / ArduPilot setup entirely. It speaks MAVLink to
[BAHR-GCS](https://github.com/oguzcanvur/bahr-gcs), the project's ground
control station, so the GCS needs no changes to fly it — the same way it
already flies a real ArduPilot vehicle or the GCS's own Python simulator.

Otonom bir keşif teknesi için, Cube Orange/ArduPilot'un yerini alan,
Raspberry Pi + STM32 Nucleo tabanlı kendi otopilotumuz.

## Why a custom autopilot

ArduPilot expects IMU/GPS/RC all wired directly to one flight controller.
This project's GNSS (a SweGeo RTD100, dual-antenna, Bynav-based) and depth
sounder (a Garmin echoMAP 42cv) don't fit that model cleanly, and the brain/
reflex split lets the two halves fail independently: if the Pi crashes or
loses power, the Nucleo's own failsafe table still holds — including a
dedicated hardware arm switch on the RC transmitter that works even with the
Pi completely dead.

## Architecture

```
            MAVLink / UDP : 14550
BAHR-GCS  <──────────────────────────>  bahr_pilot (Raspberry Pi 4)
 (PC)                                          │ UART, 115200 baud
                                                ▼
                                   Nucleo-G431RB (firmware/reflex/)
                                     │         │          │
                                 SBUS (RC)   I2C (IMU)   PWM (ESCs)
```

- **`bahr_pilot/`** (Python package) — the Pi-side MAVLink process
  (`vehicle.py`): telemetry, mode/arm/mission handling, and drivers for the
  GNSS, depth sounder, RTK correction relay, and the link to the Nucleo.
  Around it: the state estimator (`estimator.py`), line following
  (`pathfollow.py`) and heading/speed control (`control.py`), the failsafe
  rules (`failsafe.py`) and geofence with safe RTL (`geofence.py`), validated
  and persistent parameters (`params.py`), the sonar filter and
  position-stamped, quality-classified bathymetry (`sonar.py`,
  `bathymetry.py`), the per-mission data log (`missionlog.py`), and the health
  table behind SYS_STATUS (`diagnostics.py`). `bahr_pilot/sitl/` is a boat and
  sensor simulator the whole vehicle runs in (`docs/SITL.md`).
- **`firmware/reflex/`** — the Nucleo's C firmware (STM32CubeIDE project):
  reads the RC receiver (SBUS), the IMU (BNO086) and the battery voltage,
  drives the ESCs (PWM), and runs a small, strict failsafe state machine
  independent of the Pi, backed by a hardware watchdog.
- **Nucleo ↔ Pi protocol** (`bahr_pilot/nucleo_link.py` ↔
  `firmware/reflex/Core/Src/pi_link.c`) — a small bidirectional packet
  protocol over UART: motor commands and RC-channel-mapping configuration
  one way, RC channels + battery + roll/pitch telemetry the other.
- **`docs/`** — `BAHR_GCS_ARCHITECTURE.md` (what BAHR-GCS actually sends
  and expects, read from its code) and `ARCHITECTURE_REVIEW.md` (gap
  analysis against the target STM32/FreeRTOS + ROS 2 architecture, and the
  phase plan).

## Status

LED-blink on the Nucleo is the only piece actually verified on real
hardware so far. Everything else — the MAVLink wire protocol, the Pi↔Nucleo
packet protocol, the RC channel mixing, the BNO086 IMU driver, the battery
ADC reading, flash-persisted settings, the watchdog — has been checked at
the protocol/compile/unit-test level (a real BAHR-GCS instance talking to
`bahr_pilot.vehicle` over UDP, a `pytest` suite exercising the real
`NucleoLink`/`RtcmReassembler`/`Vehicle` code — see `tests/` — and a clean
firmware build) but not yet against the real GNSS, depth sounder, Nucleo,
BNO086, RC transmitter, or battery together.
See `CHANGELOG.md` for the detailed breakdown.

## Running it

From the repository root:

```bash
pip install -r requirements.txt
python -m bahr_pilot.vehicle --gcs-host <PC running BAHR-GCS> \
    --nucleo-port /dev/serial0 \
    --gnss-port /dev/serial/by-id/usb-...-RTD100 \
    --echomap-port /dev/serial/by-id/usb-...-echomap
```

All three `--*-port` flags are optional — omit any sensor that isn't
connected yet and that part just won't report data. `--log-dir` is also
optional and, if given, writes a timestamped NDJSON file of raw GNSS/
depth/IMU/battery readings there (`datalog.py`) for later post-processing,
and a folder `missions/mission-<time>/` for every arming (`meta.json`,
`bathymetry.csv` with the quality of every sounding, `track.csv`,
`events.csv`, `summary.json`; see `missionlog.py`). Tuned parameters are kept
in `--param-file` (default `~/.bahr_pilot/params.json`).

Building and flashing the Nucleo firmware uses STM32CubeIDE (or its bundled
`arm-none-eabi-gcc`/`make` from the command line) on `firmware/reflex/`.

### Testing

```bash
pip install -r requirements-dev.txt
pytest
```

The suite (`tests/`) runs entirely without hardware: it exercises the real
`NucleoLink` class against a `pyserial` loopback port, the real
`RtcmFragmenter`/`RtcmReassembler` round-trip, and `Vehicle`'s command/
parameter/mode-switch handling directly.

`tests/test_c_firmware_logic.py` additionally compiles real firmware C
(`mode_switch.c`, `pi_link.c` with the HAL stubbed in `tests/c/stubs/`) with
a host C compiler and checks it against the Python side byte for byte. It
looks for `$BAHR_CC`, then `gcc`/`cc`/`clang`, then the `ziglang` pip
package, and skips itself if none exists. Everything else in the firmware
(sensors, ESC output, failsafe priorities, flash writes) is only checked by
the `arm-none-eabi-gcc` build, until it runs on a real Nucleo.

### RC mode switch

One RC channel (`MODE_CH`, default 5) is split into six PWM bands exactly as
in ArduPilot, and `MODE1…MODE6` name the mode of each band (defaults for a
3-position switch: MANUAL / HOLD / AUTO). In a MANUAL band the Nucleo drives
the motors from the sticks itself, with no Pi or GCS involved; the arm
switch (`RCMAP_ARM`, default CH6) still has to be on. Other bands are applied
by the Pi when the switch moves and once at startup; BAHR-GCS can change the
mode in between, but not while the switch is in a MANUAL band.

### Deploying to the Pi

`deploy/bahr-pilot.service` (systemd unit) and `deploy/update.sh`
(`git pull` + dependency update + service restart) are templates for
running this as a long-lived service on the Pi and updating it over SSH —
copy the service file to `/etc/systemd/system/`, fill in the real
`/dev/serial/by-id/...` paths, `enable` it, and `update.sh` handles
updates after that. Neither has been tried against the real Pi yet.

## Related

- [BAHR-GCS](https://github.com/oguzcanvur/bahr-gcs) — the ground control
  station this autopilot talks to.

## License

MIT — see `LICENSE`.
