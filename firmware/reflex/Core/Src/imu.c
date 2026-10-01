#include "imu.h"
#include "main.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/* SHTP transport: channel numbers (sparkfun/SparkFun_BNO080_Arduino_Library,
 * src/SparkFun_BNO080_Arduino_Library.h). Input sensor reports arrive on
 * CHANNEL_REPORTS; Set Feature / Product ID requests go out on
 * CHANNEL_CONTROL; the reset command goes out on CHANNEL_EXECUTABLE. */
#define CHANNEL_COMMAND    0U
#define CHANNEL_EXECUTABLE 1U
#define CHANNEL_CONTROL    2U
#define CHANNEL_REPORTS    3U
#define NUM_CHANNELS       6U

#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8U
#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9U
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFDU
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08U
#define EXECUTABLE_RESET_COMPLETE 0x01U

#define BNO08X_I2C_ADDR_7BIT 0x4BU /* default strapping; some breakouts use 0x4A — unverified, no hardware this session */
#define I2C_TIMEOUT_MS 5U

#define GAME_ROTATION_VECTOR_INTERVAL_US 50000UL /* 50 ms = 20 Hz */
#define Q14_SCALE (1.0f / 16384.0f)

#define IMU_TIMEOUT_MS 1000U
#define RAD_TO_DEG 57.29577951308232f

static uint8_t s_txSeq[NUM_CHANNELS];
static uint8_t s_rxBuf[32]; /* header(4) + up to 28 payload bytes — a Game
                              * Rotation Vector report is 21 bytes total,
                              * comfortably under this */

static volatile bool s_valid;
static volatile float s_rollDeg;
static volatile float s_pitchDeg;
static volatile uint32_t s_lastReportTick;

static bool SendPacket(uint8_t channel, const uint8_t *data, uint8_t len)
{
    uint8_t header[4];
    uint16_t packetLen = (uint16_t)(len + 4U);
    header[0] = (uint8_t)(packetLen & 0xFFU);
    header[1] = (uint8_t)(packetLen >> 8);
    header[2] = channel;
    header[3] = s_txSeq[channel]++;

    uint8_t frame[4U + 17U]; /* 17 = largest payload we ever send (Set Feature) */
    if (len > 17U) {
        return false;
    }
    memcpy(&frame[0], header, 4U);
    memcpy(&frame[4], data, len);

    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(BNO08X_I2C_ADDR_7BIT << 1), frame,
                                    (uint16_t)(len + 4U), I2C_TIMEOUT_MS) == HAL_OK;
}

/* Peeks the 4-byte SHTP header, then (if a packet is pending) re-reads the
 * whole packet in one transaction — matching the reference driver's I2C
 * behaviour, where a short read doesn't advance the device's internal
 * pointer, only a transaction that reads the full declared length does.
 * Returns the payload length (0 if nothing pending / on any I2C error). */
static uint16_t ReceivePacket(void)
{
    uint8_t header[4];
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(BNO08X_I2C_ADDR_7BIT << 1), header, 4U,
                                I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    uint16_t rawLen = (uint16_t)(((uint16_t)header[1] << 8) | header[0]);
    rawLen &= (uint16_t)~(1U << 15); /* clear continuation bit, not handled here */
    if (rawLen <= 4U) {
        return 0U; /* empty packet */
    }
    if (rawLen > sizeof(s_rxBuf)) {
        rawLen = sizeof(s_rxBuf); /* truncate rather than overflow; we only care about small reports */
    }
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(BNO08X_I2C_ADDR_7BIT << 1), s_rxBuf, rawLen,
                                I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    return (uint16_t)(rawLen - 4U); /* payload length, header stripped */
}

static void DrainFor(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        ReceivePacket();
    }
}

static void EnableGameRotationVector(void)
{
    uint8_t payload[17] = {0};
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = SENSOR_REPORTID_GAME_ROTATION_VECTOR;
    /* payload[2]=feature flags, [3..4]=change sensitivity: 0, not used here */
    payload[5] = (uint8_t)((GAME_ROTATION_VECTOR_INTERVAL_US >> 0) & 0xFFU);
    payload[6] = (uint8_t)((GAME_ROTATION_VECTOR_INTERVAL_US >> 8) & 0xFFU);
    payload[7] = (uint8_t)((GAME_ROTATION_VECTOR_INTERVAL_US >> 16) & 0xFFU);
    payload[8] = (uint8_t)((GAME_ROTATION_VECTOR_INTERVAL_US >> 24) & 0xFFU);
    /* payload[9..16]=batch interval + sensor-specific config: 0, not used here */
    SendPacket(CHANNEL_CONTROL, payload, 17U);
}

void IMU_Init(void)
{
    memset(s_txSeq, 0, sizeof(s_txSeq));
    s_valid = false;
    s_rollDeg = 0.0f;
    s_pitchDeg = 0.0f;
    s_lastReportTick = 0U;

    uint8_t resetCmd = 0x01U;
    SendPacket(CHANNEL_EXECUTABLE, &resetCmd, 1U);
    DrainFor(50U);
    DrainFor(50U); /* two rounds, matching the reference driver's softReset() */

    uint8_t idReq[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0U};
    SendPacket(CHANNEL_CONTROL, idReq, 2U);
    DrainFor(20U); /* best-effort: we don't gate init on seeing the response,
                     * only on whether real Game Rotation Vector reports
                     * eventually show up (IMU_IsValid()) */

    EnableGameRotationVector();
}

void IMU_Update(void)
{
    uint16_t payloadLen = ReceivePacket();
    if (payloadLen < 17U) {
        goto check_timeout; /* too short to be a Game Rotation Vector report */
    }
    /* s_rxBuf layout (payload, header already stripped): [0..4]=Report Base
     * Timestamp (ID 0xFB + 4-byte delta, ignored — we don't need the
     * BNO086's internal clock), [5]=sensor report ID, [6]=sequence,
     * [7]=status, [8]=reserved/delay, [9..10]=i, [11..12]=j, [13..14]=k,
     * [15..16]=real, all int16 little-endian Q14. */
    if (s_rxBuf[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        int16_t rawI, rawJ, rawK, rawReal;
        memcpy(&rawI, &s_rxBuf[9], 2U);
        memcpy(&rawJ, &s_rxBuf[11], 2U);
        memcpy(&rawK, &s_rxBuf[13], 2U);
        memcpy(&rawReal, &s_rxBuf[15], 2U);

        float x = (float)rawI * Q14_SCALE;
        float y = (float)rawJ * Q14_SCALE;
        float z = (float)rawK * Q14_SCALE;
        float w = (float)rawReal * Q14_SCALE;

        /* Standard quaternion -> roll/pitch (aerospace ZYX convention).
         * Yaw deliberately not computed — GNSS heading is this project's
         * real yaw source (see imu.h). Axis sign/orientation relative to
         * how the BNO086 ends up physically mounted on the boat is
         * UNVERIFIED (no hardware this session) — check against a known
         * tilt once it's mounted, a sign flip or axis swap here is a
         * likely-needed correction, not a sign of a protocol bug. */
        s_rollDeg = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
        float sinp = 2.0f * (w * y - z * x);
        if (sinp > 1.0f) {
            sinp = 1.0f;
        } else if (sinp < -1.0f) {
            sinp = -1.0f;
        }
        s_pitchDeg = asinf(sinp) * RAD_TO_DEG;
        s_lastReportTick = HAL_GetTick();
        s_valid = true;
    }

check_timeout:
    if (s_valid && (HAL_GetTick() - s_lastReportTick) > IMU_TIMEOUT_MS) {
        s_valid = false;
    }
}

bool IMU_IsValid(void)
{
    return s_valid;
}

float IMU_GetRollDeg(void)
{
    return s_rollDeg;
}

float IMU_GetPitchDeg(void)
{
    return s_pitchDeg;
}
