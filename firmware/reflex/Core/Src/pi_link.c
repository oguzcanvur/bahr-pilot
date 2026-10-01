#include "pi_link.h"
#include "esc.h"
#include <string.h>

#define MOTOR_FRAME_LEN  7U
#define MOTOR_SYNC0      0xA5U
#define MOTOR_SYNC1      0x5AU

#define CONFIG_FRAME_LEN 20U
#define CONFIG_SYNC0     0xC5U
#define CONFIG_SYNC1     0x5CU

#define TELEMETRY_FRAME_LEN 36U
#define TELEMETRY_SYNC0      0xE5U
#define TELEMETRY_SYNC1      0x5EU

typedef enum {
    RX_WAIT_SYNC0 = 0,
    RX_WAIT_SYNC1,
    RX_COLLECT_MOTOR,
    RX_COLLECT_CONFIG,
} RxState;

extern UART_HandleTypeDef huart3;

static uint8_t s_rxBuf[CONFIG_FRAME_LEN];
static uint8_t s_rxIndex;
static RxState s_rxState;
static uint8_t s_rxByte;

static volatile uint16_t s_motor1Us;
static volatile uint16_t s_motor2Us;
static volatile uint32_t s_lastRxTick;
static volatile bool s_everReceived;

/* Defaults match ArduPilot Rover's own RCMAP_ROLL=1 / RCMAP_THROTTLE=3
 * (1-based) convention, stored 0-based here — so a GCS user who never
 * touches these parameters gets the behaviour they'd expect anyway.
 * Arm/override channels match the placeholders failsafe.c used before
 * this became configurable. Min/max/trim span the full SBUS range, i.e.
 * no calibration offset, until BAHR-GCS's RadioPage writes real values. */
static RcMapConfig s_rcMap = {
    .throttle_channel = 2U,  /* CH3 */
    .steering_channel = 0U,  /* CH1 */
    .arm_channel = 5U,       /* CH6 */
    .override_channel = 4U,  /* CH5 */
    .throttle_min = SBUS_CH_MIN,
    .throttle_max = SBUS_CH_MAX,
    .throttle_trim = SBUS_CH_MID,
    .steering_min = SBUS_CH_MIN,
    .steering_max = SBUS_CH_MAX,
    .steering_trim = SBUS_CH_MID,
    .throttle_reversed = false,
    .steering_reversed = false,
};

static uint8_t s_txBuf[TELEMETRY_FRAME_LEN];
static volatile bool s_txBusy;

static uint8_t Checksum(const uint8_t *data, uint8_t len)
{
    uint8_t x = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        x ^= data[i];
    }
    return x;
}

static void DecodeMotorFrame(const uint8_t *f)
{
    if (Checksum(f, MOTOR_FRAME_LEN - 1U) != f[MOTOR_FRAME_LEN - 1U]) {
        return;
    }
    s_motor1Us = (uint16_t)((f[2] << 8) | f[3]);
    s_motor2Us = (uint16_t)((f[4] << 8) | f[5]);
    s_lastRxTick = HAL_GetTick();
    s_everReceived = true;
}

static void DecodeConfigFrame(const uint8_t *f)
{
    if (Checksum(f, CONFIG_FRAME_LEN - 1U) != f[CONFIG_FRAME_LEN - 1U]) {
        return;
    }
    RcMapConfig cfg;
    cfg.throttle_channel = f[2];
    cfg.steering_channel = f[3];
    cfg.arm_channel = f[4];
    cfg.override_channel = f[5];
    cfg.throttle_min = (uint16_t)((f[6] << 8) | f[7]);
    cfg.throttle_max = (uint16_t)((f[8] << 8) | f[9]);
    cfg.throttle_trim = (uint16_t)((f[10] << 8) | f[11]);
    cfg.steering_min = (uint16_t)((f[12] << 8) | f[13]);
    cfg.steering_max = (uint16_t)((f[14] << 8) | f[15]);
    cfg.steering_trim = (uint16_t)((f[16] << 8) | f[17]);
    cfg.throttle_reversed = (f[18] & 0x01U) != 0U;
    cfg.steering_reversed = (f[18] & 0x02U) != 0U;

    /* Reject channel indices SBUS can't actually index rather than storing
     * garbage that'd make steer_towards()-style math read out of bounds. */
    if (cfg.throttle_channel >= SBUS_NUM_CHANNELS || cfg.steering_channel >= SBUS_NUM_CHANNELS ||
        cfg.arm_channel >= SBUS_NUM_CHANNELS || cfg.override_channel >= SBUS_NUM_CHANNELS) {
        return;
    }
    s_rcMap = cfg;
}

void PiLink_Init(void)
{
    s_rxIndex = 0U;
    s_rxState = RX_WAIT_SYNC0;
    s_motor1Us = ESC_PULSE_NEUTRAL_US;
    s_motor2Us = ESC_PULSE_NEUTRAL_US;
    s_lastRxTick = 0U;
    s_everReceived = false;
    s_txBusy = false;
    HAL_UART_Receive_IT(&huart3, &s_rxByte, 1U);
}

uint16_t PiLink_GetMotor1Us(void)
{
    return s_motor1Us;
}

uint16_t PiLink_GetMotor2Us(void)
{
    return s_motor2Us;
}

bool PiLink_IsFresh(void)
{
    if (!s_everReceived) {
        return false;
    }
    return (HAL_GetTick() - s_lastRxTick) <= PI_LINK_TIMEOUT_MS;
}

const RcMapConfig *PiLink_GetRcMap(void)
{
    return &s_rcMap;
}

void PiLink_SendTelemetry(bool armed, bool rcLinkUp, bool overrideActive)
{
    if (s_txBusy) {
        return; /* previous frame still in flight; this is periodic, skip a beat */
    }

    s_txBuf[0] = TELEMETRY_SYNC0;
    s_txBuf[1] = TELEMETRY_SYNC1;
    for (uint8_t i = 0U; i < SBUS_NUM_CHANNELS; i++) {
        uint16_t ch = SBUS_GetChannel(i);
        s_txBuf[2U + 2U * i] = (uint8_t)(ch >> 8);
        s_txBuf[3U + 2U * i] = (uint8_t)(ch & 0xFFU);
    }
    uint8_t status = 0U;
    status |= armed ? 0x01U : 0U;
    status |= rcLinkUp ? 0x02U : 0U;
    status |= overrideActive ? 0x04U : 0U;
    status |= PiLink_IsFresh() ? 0x08U : 0U;
    s_txBuf[34] = status;
    s_txBuf[35] = Checksum(s_txBuf, TELEMETRY_FRAME_LEN - 1U);

    s_txBusy = true;
    if (HAL_UART_Transmit_IT(&huart3, s_txBuf, TELEMETRY_FRAME_LEN) != HAL_OK) {
        s_txBusy = false;
    }
}

void PiLink_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    switch (s_rxState) {
    case RX_WAIT_SYNC0:
        if (s_rxByte == MOTOR_SYNC0) {
            s_rxBuf[0] = s_rxByte;
            s_rxIndex = 1U;
            s_rxState = RX_WAIT_SYNC1;
        } else if (s_rxByte == CONFIG_SYNC0) {
            s_rxBuf[0] = s_rxByte;
            s_rxIndex = 1U;
            s_rxState = RX_WAIT_SYNC1;
        }
        break;

    case RX_WAIT_SYNC1:
        if (s_rxBuf[0] == MOTOR_SYNC0 && s_rxByte == MOTOR_SYNC1) {
            s_rxBuf[1] = s_rxByte;
            s_rxIndex = 2U;
            s_rxState = RX_COLLECT_MOTOR;
        } else if (s_rxBuf[0] == CONFIG_SYNC0 && s_rxByte == CONFIG_SYNC1) {
            s_rxBuf[1] = s_rxByte;
            s_rxIndex = 2U;
            s_rxState = RX_COLLECT_CONFIG;
        } else {
            s_rxState = RX_WAIT_SYNC0; /* desynced, start hunting again */
        }
        break;

    case RX_COLLECT_MOTOR:
        s_rxBuf[s_rxIndex++] = s_rxByte;
        if (s_rxIndex >= MOTOR_FRAME_LEN) {
            DecodeMotorFrame(s_rxBuf);
            s_rxState = RX_WAIT_SYNC0;
        }
        break;

    case RX_COLLECT_CONFIG:
        s_rxBuf[s_rxIndex++] = s_rxByte;
        if (s_rxIndex >= CONFIG_FRAME_LEN) {
            DecodeConfigFrame(s_rxBuf);
            s_rxState = RX_WAIT_SYNC0;
        }
        break;

    default:
        s_rxState = RX_WAIT_SYNC0;
        break;
    }

    HAL_UART_Receive_IT(&huart3, &s_rxByte, 1U);
}

void PiLink_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }
    s_rxState = RX_WAIT_SYNC0;
    s_rxIndex = 0U;
    HAL_UART_Receive_IT(&huart3, &s_rxByte, 1U);
}

void PiLink_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }
    s_txBusy = false;
}
