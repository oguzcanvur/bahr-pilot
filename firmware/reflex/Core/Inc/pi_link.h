#ifndef PI_LINK_H
#define PI_LINK_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "sbus.h"

/* USART3 (PB10/PB11) now carries three frame kinds, distinguished by their
 * sync bytes. All multi-byte fields are big-endian.
 *
 * Pi -> Nucleo, motor command, 7 bytes (unchanged since 2026-10-01):
 *   [0]=0xA5 [1]=0x5A [2..3]=motor1_us [4..5]=motor2_us [6]=XOR checksum
 *   Pulses are absolute microseconds. Sent periodically (~20-50 Hz); a gap
 *   longer than PI_LINK_TIMEOUT_MS is a failsafe condition (see failsafe.c),
 *   not an error here. Superseded in practice once RC mixing (below) is
 *   configured and driving the vehicle, but still accepted — GUIDED/AUTO
 *   still drive through this path.
 *
 * Pi -> Nucleo, RC map config, 16 bytes (new 2026-10-01 — lets BAHR-GCS's
 * existing RCMAP_ROLL/RCMAP_THROTTLE/RC1..8_MIN/MAX/TRIM parameter UI and
 * RadioPage calibration screen actually reach the receiver, instead of the
 * hardcoded channel assignment this had before):
 *   [0]=0xC5 [1]=0x5C
 *   [2]=throttle_channel (0-based SBUS index)
 *   [3]=steering_channel (0-based SBUS index)
 *   [4]=arm_channel
 *   [5]=override_channel
 *   [6..7]=throttle_min [8..9]=throttle_max [10..11]=throttle_trim
 *   [12..13]=steering_min [14..15]=steering_max [16..17]=steering_trim
 *   [18]=flags (bit0=throttle_reversed, bit1=steering_reversed)
 *   [19]=XOR checksum of bytes 0-18
 *   The UART ISR only stashes a valid frame; PiLink_Process() (main loop)
 *   applies it and persists it to flash (settings.c) once no new frame has
 *   arrived for SETTINGS_SAVE_DEBOUNCE_MS and the arm switch is off — a
 *   page erase stalls this single-bank part's instruction fetch for tens
 *   of ms, so it never runs in an ISR or while armed, and a RadioPage
 *   calibration (dozens of PARAM_SETs) costs one erase, not dozens.
 *   Reloaded at boot; falls back to the compiled defaults below if flash
 *   is blank or fails its checksum.
 *   (Frame is 20 bytes total; comment above lists field offsets, not frame
 *   length rounding.)
 *
 * Nucleo -> Pi, RC telemetry, 42 bytes (36 bytes 2026-10-01, extended same
 * day to carry battery/IMU — makes the raw SBUS channels, failsafe status,
 * pack voltage and roll/pitch visible to the Pi, which previously only
 * ever saw zeros/unknowns for all of this):
 *   [0]=0xE5 [1]=0x5E
 *   [2..33]=16 x channel value (uint16 each, raw 11-bit SBUS range)
 *   [34]=status (bit0=armed, bit1=rc_link_up, bit2=override_active,
 *                bit3=pi_link_fresh, bit4=battery_valid, bit5=imu_valid)
 *   [35..36]=battery pack voltage, millivolts (uint16; 0/invalid if
 *            bit4 clear — see battery.c's divider-ratio caveat)
 *   [37..38]=roll, centidegrees, signed (int16; 0/invalid if bit5 clear)
 *   [39..40]=pitch, centidegrees, signed (int16; 0/invalid if bit5 clear)
 *   [41]=XOR checksum of bytes 0-40
 *   Sent every RC_LINK_TELEMETRY_PERIOD_MS from the main loop.
 */

#define PI_LINK_TIMEOUT_MS 500U
#define RC_LINK_TELEMETRY_PERIOD_MS 100U
#define SETTINGS_SAVE_DEBOUNCE_MS 1000U

typedef struct {
    uint8_t throttle_channel;
    uint8_t steering_channel;
    uint8_t arm_channel;
    uint8_t override_channel;
    uint16_t throttle_min;
    uint16_t throttle_max;
    uint16_t throttle_trim;
    uint16_t steering_min;
    uint16_t steering_max;
    uint16_t steering_trim;
    bool throttle_reversed;
    bool steering_reversed;
} RcMapConfig;

void PiLink_Init(void);

/* Last commanded pulse widths from the motor-command frame, in
 * microseconds. Valid only when PiLink_IsFresh() is true. */
uint16_t PiLink_GetMotor1Us(void);
uint16_t PiLink_GetMotor2Us(void);

/* True if a valid motor-command packet arrived within PI_LINK_TIMEOUT_MS. */
bool PiLink_IsFresh(void);

/* Current RC channel -> function mapping, defaults match ArduPilot Rover's
 * own RCMAP_ROLL=1/RCMAP_THROTTLE=3 convention (1-based on the GCS side,
 * stored 0-based here) so the out-of-the-box behaviour matches what a
 * BAHR-GCS user already expects without touching any parameter. */
const RcMapConfig *PiLink_GetRcMap(void);

/* Main-loop half of the RC map config path (see the frame description
 * above): applies a config frame the ISR received, and saves to flash when
 * it is safe to. Call every control tick with the current arm switch
 * state. */
void PiLink_Process(bool armed);

/* Sends the RC telemetry frame (see header comment). Call once per
 * RC_LINK_TELEMETRY_PERIOD_MS from the main loop. batteryValid/imuValid
 * false means "don't trust batteryMv/rollCdeg/pitchCdeg", not "send zero
 * as a real reading" — the Pi side treats the validity bits as
 * authoritative. */
void PiLink_SendTelemetry(bool armed, bool rcLinkUp, bool overrideActive,
                           bool batteryValid, uint16_t batteryMv,
                           bool imuValid, int16_t rollCdeg, int16_t pitchCdeg);

void PiLink_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void PiLink_UART_ErrorCallback(UART_HandleTypeDef *huart);
void PiLink_UART_TxCpltCallback(UART_HandleTypeDef *huart);

#endif /* PI_LINK_H */
