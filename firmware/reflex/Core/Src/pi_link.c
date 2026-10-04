#include "pi_link.h"
#include "esc.h"
#include "settings.h"
#include <string.h>

#define MOTOR_FRAME_LEN  7U
#define MOTOR_SYNC0      0xA5U
#define MOTOR_SYNC1      0x5AU

#define CONFIG_FRAME_LEN 25U
#define CONFIG_SYNC0     0xC5U
#define CONFIG_SYNC1     0x5CU

#define TELEMETRY_FRAME_LEN 44U
#define TELEMETRY_SYNC0      0xE5U
#define TELEMETRY_SYNC1      0x5EU

#define IMU_FRAME_LEN 28U
#define IMU_SYNC0     0xE6U
#define IMU_SYNC1     0x6EU

/* Outgoing frames go through a small byte ring so a telemetry frame and an
 * IMU frame can never be lost to each other (the old single buffer simply
 * skipped a frame whenever the previous one was still being sent). One
 * producer (the main loop) and one consumer (the TX-complete interrupt). */
#define TX_RING_SIZE 256U /* power of two */

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
    .mode_channel = 4U,      /* CH5 */
    .throttle_min = SBUS_CH_MIN,
    .throttle_max = SBUS_CH_MAX,
    .throttle_trim = SBUS_CH_MID,
    .steering_min = SBUS_CH_MIN,
    .steering_max = SBUS_CH_MAX,
    .steering_trim = SBUS_CH_MID,
    .throttle_reversed = false,
    .steering_reversed = false,
    .mode_min = SBUS_CH_MIN,
    .mode_max = SBUS_CH_MAX,
    .manual_slot_mask = 0x07U, /* MODE1..3 = MANUAL, same as the Pi's defaults */
};

/* Written by the UART ISR, consumed by PiLink_Process() in the main loop.
 * s_rcMap itself is only ever written from the main loop, so the control
 * loop never reads a half-updated mapping. */
static RcMapConfig s_pendingMap;
static volatile bool s_mapPending;
static volatile uint32_t s_lastConfigTick;
static bool s_saveDirty;

static uint8_t s_txRing[TX_RING_SIZE];
static volatile uint16_t s_txHead;  /* written by the producer only */
static volatile uint16_t s_txTail;  /* written by the consumer only */
static volatile bool s_txBusy;
static uint16_t s_txChunk;

/* Must be called with interrupts disabled, or from the TX-complete ISR. */
static void KickTx(void)
{
    if (s_txBusy || s_txHead == s_txTail) {
        return;
    }
    uint16_t pending = (uint16_t)((s_txHead - s_txTail) & (TX_RING_SIZE - 1U));
    uint16_t untilWrap = (uint16_t)(TX_RING_SIZE - s_txTail);
    s_txChunk = pending < untilWrap ? pending : untilWrap;
    s_txBusy = true;
    if (HAL_UART_Transmit_IT(&huart3, &s_txRing[s_txTail], s_txChunk) != HAL_OK) {
        s_txBusy = false;
    }
}

/* Queues a whole frame or nothing: a frame cut in half would desync the
 * receiver. Returns false (frame dropped) when the ring is too full. */
static bool EnqueueFrame(const uint8_t *frame, uint16_t len)
{
    __disable_irq();
    uint16_t room = (uint16_t)((s_txTail - s_txHead - 1U) & (TX_RING_SIZE - 1U));
    bool ok = room >= len;
    if (ok) {
        uint16_t head = s_txHead;
        for (uint16_t i = 0U; i < len; i++) {
            s_txRing[head] = frame[i];
            head = (uint16_t)((head + 1U) & (TX_RING_SIZE - 1U));
        }
        s_txHead = head;
        KickTx();
    }
    __enable_irq();
    return ok;
}

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
    cfg.mode_channel = f[5];
    cfg.throttle_min = (uint16_t)((f[6] << 8) | f[7]);
    cfg.throttle_max = (uint16_t)((f[8] << 8) | f[9]);
    cfg.throttle_trim = (uint16_t)((f[10] << 8) | f[11]);
    cfg.steering_min = (uint16_t)((f[12] << 8) | f[13]);
    cfg.steering_max = (uint16_t)((f[14] << 8) | f[15]);
    cfg.steering_trim = (uint16_t)((f[16] << 8) | f[17]);
    cfg.throttle_reversed = (f[18] & 0x01U) != 0U;
    cfg.steering_reversed = (f[18] & 0x02U) != 0U;
    cfg.mode_min = (uint16_t)((f[19] << 8) | f[20]);
    cfg.mode_max = (uint16_t)((f[21] << 8) | f[22]);
    cfg.manual_slot_mask = (uint8_t)(f[23] & 0x3FU); /* only 6 bands exist */

    /* Reject channel indices SBUS can't actually index rather than storing
     * garbage that'd make steer_towards()-style math read out of bounds. */
    if (cfg.throttle_channel >= SBUS_NUM_CHANNELS || cfg.steering_channel >= SBUS_NUM_CHANNELS ||
        cfg.arm_channel >= SBUS_NUM_CHANNELS || cfg.mode_channel >= SBUS_NUM_CHANNELS) {
        return;
    }
    s_pendingMap = cfg;
    s_lastConfigTick = HAL_GetTick();
    s_mapPending = true;
}

void PiLink_Process(bool armed)
{
    if (s_mapPending) {
        __disable_irq();
        RcMapConfig cfg = s_pendingMap;
        s_mapPending = false;
        __enable_irq();
        s_rcMap = cfg;
        s_saveDirty = true;
    }

    if (s_saveDirty && !armed &&
        (HAL_GetTick() - s_lastConfigTick) >= SETTINGS_SAVE_DEBOUNCE_MS) {
        Settings_SaveRcMap(&s_rcMap);
        s_saveDirty = false;
    }
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
    s_txHead = 0U;
    s_txTail = 0U;
    s_mapPending = false;
    s_saveDirty = false;
    s_lastConfigTick = 0U;

    RcMapConfig saved;
    if (Settings_LoadRcMap(&saved)) {
        s_rcMap = saved; /* overrides the compiled-in defaults above */
    }

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

void PiLink_SendTelemetry(bool armed, bool rcLinkUp, bool overrideActive,
                           bool batteryValid, uint16_t batteryMv,
                           bool imuValid, int16_t rollCdeg, int16_t pitchCdeg,
                           uint8_t modeSlot, uint8_t armInfo)
{
    uint8_t f[TELEMETRY_FRAME_LEN];
    f[0] = TELEMETRY_SYNC0;
    f[1] = TELEMETRY_SYNC1;
    for (uint8_t i = 0U; i < SBUS_NUM_CHANNELS; i++) {
        uint16_t ch = SBUS_GetChannel(i);
        f[2U + 2U * i] = (uint8_t)(ch >> 8);
        f[3U + 2U * i] = (uint8_t)(ch & 0xFFU);
    }
    uint8_t status = 0U;
    status |= armed ? 0x01U : 0U;
    status |= rcLinkUp ? 0x02U : 0U;
    status |= overrideActive ? 0x04U : 0U;
    status |= PiLink_IsFresh() ? 0x08U : 0U;
    status |= batteryValid ? 0x10U : 0U;
    status |= imuValid ? 0x20U : 0U;
    f[34] = status;
    f[35] = (uint8_t)(batteryMv >> 8);
    f[36] = (uint8_t)(batteryMv & 0xFFU);
    f[37] = (uint8_t)((uint16_t)rollCdeg >> 8);
    f[38] = (uint8_t)((uint16_t)rollCdeg & 0xFFU);
    f[39] = (uint8_t)((uint16_t)pitchCdeg >> 8);
    f[40] = (uint8_t)((uint16_t)pitchCdeg & 0xFFU);
    f[41] = modeSlot;
    f[42] = armInfo;
    f[43] = Checksum(f, TELEMETRY_FRAME_LEN - 1U);
    (void)EnqueueFrame(f, TELEMETRY_FRAME_LEN);
}

static int16_t Quantise(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static void PutI16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFFU);
}

void PiLink_SendImu(const ImuSample *s)
{
    uint8_t f[IMU_FRAME_LEN];
    f[0] = IMU_SYNC0;
    f[1] = IMU_SYNC1;
    f[2] = (uint8_t)(s->t_us >> 24);
    f[3] = (uint8_t)(s->t_us >> 16);
    f[4] = (uint8_t)(s->t_us >> 8);
    f[5] = (uint8_t)(s->t_us & 0xFFU);
    for (uint8_t i = 0U; i < 4U; i++) {
        PutI16(&f[6U + 2U * i], Quantise(s->quat[i], 16384.0f)); /* Q14 */
    }
    for (uint8_t i = 0U; i < 3U; i++) {
        PutI16(&f[14U + 2U * i], Quantise(s->gyro[i], 512.0f));  /* Q9, rad/s */
        PutI16(&f[20U + 2U * i], Quantise(s->accel[i], 256.0f)); /* Q8, m/s^2 */
    }
    uint8_t flags = 0U;
    flags |= s->quat_valid ? 0x01U : 0U;
    flags |= s->gyro_valid ? 0x02U : 0U;
    flags |= s->accel_valid ? 0x04U : 0U;
    flags |= (uint8_t)((s->quat_accuracy & 0x03U) << 4);
    f[26] = flags;
    f[27] = Checksum(f, IMU_FRAME_LEN - 1U);
    (void)EnqueueFrame(f, IMU_FRAME_LEN);
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
        } else if (s_rxByte == MOTOR_SYNC0 || s_rxByte == CONFIG_SYNC0) {
            s_rxBuf[0] = s_rxByte; /* this byte may itself start a frame (A5 A5 5A ...) */
            s_rxIndex = 1U;
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
    s_txTail = (uint16_t)((s_txTail + s_txChunk) & (TX_RING_SIZE - 1U));
    s_txBusy = false;
    KickTx();
}
