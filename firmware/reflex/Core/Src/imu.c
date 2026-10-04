#include "imu.h"
#include "imu_reports.h"
#include "clock.h"
#include "main.h"
#include <string.h>
#include <math.h>

extern I2C_HandleTypeDef hi2c1;

/* SHTP transport (sparkfun/SparkFun_BNO080_Arduino_Library): channel numbers,
 * packet header, I2C read/write mechanics. */
#define CHANNEL_COMMAND    0U
#define CHANNEL_EXECUTABLE 1U
#define CHANNEL_CONTROL    2U
#define CHANNEL_REPORTS    3U
#define NUM_CHANNELS       6U

#define SHTP_REPORT_PRODUCT_ID_REQUEST  0xF9U
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFDU

#define BNO08X_I2C_ADDR_7BIT 0x4BU /* default strapping; some breakouts use 0x4A — unverified, no hardware yet */
#define I2C_TIMEOUT_MS 5U

/* Report rates. Gyro and orientation at 50 Hz feed the Pi's heading filter;
 * linear acceleration is only context. */
#define INTERVAL_GRV_US   20000UL
#define INTERVAL_GYRO_US  20000UL
#define INTERVAL_ACCEL_US 40000UL

#define MAX_PACKETS_PER_UPDATE 4U
#define REPORT_TIMEOUT_MS 500U
#define RAD_TO_DEG 57.29577951308232f

static uint8_t s_txSeq[NUM_CHANNELS];
static uint8_t s_rxBuf[48]; /* header(4) + payload; the largest packet we expect
                              * (timestamp 5 + GRV 12 + gyro 10 + accel 10 = 37) fits */
static uint8_t s_rxChannel;

static ImuSample s_sample;
static uint32_t s_quatTick, s_gyroTick, s_accelTick;
static uint32_t s_unparsedPackets;

static bool SendPacket(uint8_t channel, const uint8_t *data, uint8_t len)
{
    uint8_t frame[4U + 17U]; /* 17 = largest payload we ever send (Set Feature) */
    if (len > 17U) {
        return false;
    }
    uint16_t packetLen = (uint16_t)(len + 4U);
    frame[0] = (uint8_t)(packetLen & 0xFFU);
    frame[1] = (uint8_t)(packetLen >> 8);
    frame[2] = channel;
    frame[3] = s_txSeq[channel]++;
    memcpy(&frame[4], data, len);

    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(BNO08X_I2C_ADDR_7BIT << 1), frame,
                                    (uint16_t)(len + 4U), I2C_TIMEOUT_MS) == HAL_OK;
}

/* Peeks the 4-byte SHTP header, then (if a packet is pending) re-reads the
 * whole packet in one transaction — matching the reference driver's I2C
 * behaviour, where a short read doesn't advance the device's internal
 * pointer, only a transaction that reads the full declared length does.
 * Returns the payload length (0 if nothing pending / on any I2C error);
 * s_rxBuf[4..] holds the payload and s_rxChannel its SHTP channel. */
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
        rawLen = sizeof(s_rxBuf); /* truncated: the parser will report it */
    }
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(BNO08X_I2C_ADDR_7BIT << 1), s_rxBuf, rawLen,
                                I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    s_rxChannel = s_rxBuf[2];
    return (uint16_t)(rawLen - 4U);
}

static void DrainFor(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms) {
        ReceivePacket();
    }
}

static void EnableReport(uint8_t reportId, uint32_t intervalUs)
{
    uint8_t payload[17] = {0};
    payload[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    payload[1] = reportId;
    /* payload[2]=feature flags, [3..4]=change sensitivity: 0, not used here */
    payload[5] = (uint8_t)((intervalUs >> 0) & 0xFFU);
    payload[6] = (uint8_t)((intervalUs >> 8) & 0xFFU);
    payload[7] = (uint8_t)((intervalUs >> 16) & 0xFFU);
    payload[8] = (uint8_t)((intervalUs >> 24) & 0xFFU);
    /* payload[9..16]=batch interval + sensor-specific config: 0, not used here */
    SendPacket(CHANNEL_CONTROL, payload, 17U);
}

void IMU_Init(void)
{
    memset(s_txSeq, 0, sizeof(s_txSeq));
    memset(&s_sample, 0, sizeof(s_sample));
    s_quatTick = s_gyroTick = s_accelTick = 0U;
    s_unparsedPackets = 0U;

    uint8_t resetCmd = 0x01U;
    SendPacket(CHANNEL_EXECUTABLE, &resetCmd, 1U);
    DrainFor(50U);
    DrainFor(50U); /* two rounds, matching the reference driver's softReset() */

    uint8_t idReq[2] = {SHTP_REPORT_PRODUCT_ID_REQUEST, 0U};
    SendPacket(CHANNEL_CONTROL, idReq, 2U);
    DrainFor(20U); /* best-effort: init is not gated on seeing the response,
                     * only on real reports eventually showing up */

    EnableReport(IMU_REPORT_GAME_ROTATION_VECTOR, INTERVAL_GRV_US);
    EnableReport(IMU_REPORT_GYROSCOPE, INTERVAL_GYRO_US);
    EnableReport(IMU_REPORT_LINEAR_ACCELERATION, INTERVAL_ACCEL_US);
}

static uint32_t SampleTimeUs(uint64_t packetUs, int32_t delay100us)
{
    int64_t t = (int64_t)packetUs + (int64_t)delay100us * 100;
    if (t < 0) {
        t = 0;
    }
    return (uint32_t)((uint64_t)t & 0xFFFFFFFFU);
}

static void OnReport(const ImuReport *r, void *ctx)
{
    uint64_t packetUs = *(const uint64_t *)ctx;
    uint32_t tUs = SampleTimeUs(packetUs, r->delay_100us);
    uint32_t tick = HAL_GetTick();

    switch (r->report_id) {
    case IMU_REPORT_GAME_ROTATION_VECTOR:
        memcpy(s_sample.quat, r->v, 4U * sizeof(float));
        s_sample.quat_accuracy = r->accuracy;
        s_quatTick = tick;
        break;
    case IMU_REPORT_GYROSCOPE:
        memcpy(s_sample.gyro, r->v, 3U * sizeof(float));
        s_gyroTick = tick;
        break;
    case IMU_REPORT_LINEAR_ACCELERATION:
        memcpy(s_sample.accel, r->v, 3U * sizeof(float));
        s_accelTick = tick;
        break;
    default:
        return;
    }
    s_sample.t_us = tUs; /* newest report wins */
}

void IMU_Update(void)
{
    for (uint8_t i = 0U; i < MAX_PACKETS_PER_UPDATE; i++) {
        uint16_t payloadLen = ReceivePacket();
        if (payloadLen == 0U) {
            break; /* nothing pending */
        }
        if (s_rxChannel != CHANNEL_REPORTS) {
            continue; /* control/executable chatter, not a sensor report */
        }
        uint64_t packetUs = Clock_NowUs();
        if (ImuReports_Parse(&s_rxBuf[4], payloadLen, OnReport, &packetUs) != IMU_PARSE_OK) {
            s_unparsedPackets++;
        }
    }
}

static bool Fresh(uint32_t lastTick)
{
    return lastTick != 0U && (HAL_GetTick() - lastTick) <= REPORT_TIMEOUT_MS;
}

bool IMU_GetSample(ImuSample *out)
{
    s_sample.quat_valid = Fresh(s_quatTick);
    s_sample.gyro_valid = Fresh(s_gyroTick);
    s_sample.accel_valid = Fresh(s_accelTick);
    *out = s_sample;
    return s_sample.quat_valid || s_sample.gyro_valid || s_sample.accel_valid;
}

bool IMU_IsValid(void)
{
    return Fresh(s_quatTick);
}

/* Standard quaternion -> roll/pitch (aerospace ZYX convention). Axis sign/
 * orientation relative to how the BNO086 ends up mounted on the boat is
 * UNVERIFIED (no hardware yet): a sign flip or axis swap here is a likely
 * correction, not a protocol bug. */
float IMU_GetRollDeg(void)
{
    float x = s_sample.quat[0], y = s_sample.quat[1], z = s_sample.quat[2], w = s_sample.quat[3];
    return atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * RAD_TO_DEG;
}

float IMU_GetPitchDeg(void)
{
    float x = s_sample.quat[0], y = s_sample.quat[1], z = s_sample.quat[2], w = s_sample.quat[3];
    float sinp = 2.0f * (w * y - z * x);
    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }
    return asinf(sinp) * RAD_TO_DEG;
}
