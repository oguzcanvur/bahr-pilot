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

- **`bahr_pilot/`** (this package) — the Pi-side MAVLink process
  (`vehicle.py`): telemetry, mode/arm/mission handling, navigation, and
  drivers for the GNSS, depth sounder, RTK correction relay, and the link to
  the Nucleo.
- **`firmware/reflex/`** — the Nucleo's C firmware (STM32CubeIDE project):
  reads the RC receiver (SBUS) and will read the IMU (BNO086, not yet
  implemented), drives the ESCs (PWM), and runs a small, strict failsafe
  state machine independent of the Pi.
- **Nucleo ↔ Pi protocol** (`bahr_pilot/nucleo_link.py` ↔
  `firmware/reflex/Core/Src/pi_link.c`) — a small bidirectional packet
  protocol over UART: motor commands and RC-channel-mapping configuration
  one way, raw RC channel telemetry the other.

## Status

LED-blink on the Nucleo is the only piece actually verified on real
hardware so far. Everything else — the MAVLink wire protocol, the Pi↔Nucleo
packet protocol, the RC channel mixing — has been checked at the protocol
level (a real BAHR-GCS instance talking to `bahr_pilot.vehicle` over UDP, and
a `pyserial` loopback port exercising the real Nucleo-link code) but not yet
against the real GNSS, depth sounder, Nucleo, or RC transmitter together.
See `CHANGELOG.md` for the detailed breakdown.

## Running it

```bash
pip install -r requirements.txt
python -m bahr_pilot.vehicle --gcs-host <PC running BAHR-GCS> \
    --nucleo-port /dev/serial0 \
    --gnss-port /dev/serial/by-id/usb-...-RTD100 \
    --echomap-port /dev/serial/by-id/usb-...-echomap
```

All three `--*-port` flags are optional — omit any sensor that isn't
connected yet and that part just won't report data.

Building and flashing the Nucleo firmware uses STM32CubeIDE (or its bundled
`arm-none-eabi-gcc`/`make` from the command line) on `firmware/reflex/`.

## Related

- [BAHR-GCS](https://github.com/oguzcanvur/bahr-gcs) — the ground control
  station this autopilot talks to.

## License

MIT — see `LICENSE`.
