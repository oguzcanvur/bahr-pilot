#include "imu_reports.h"

#define REPORT_BASE_TIMESTAMP   0xFBU
#define REPORT_TIMESTAMP_REBASE 0xFAU

#define Q(n) (1.0f / (float)(1 << (n)))

typedef struct {
    uint8_t id;
    uint8_t len;
} ReportLen;

/* Lengths from CEVA sh2.c's sh2ReportLens[]. Only the sensor reports this
 * firmware could plausibly be sent are listed; anything else makes the
 * packet unsplittable (IMU_PARSE_UNKNOWN_ID), exactly as in the reference
 * implementation. */
static const ReportLen kReportLens[] = {
    {0x01U, 10U},  /* accelerometer */
    {0x02U, 10U},  /* gyroscope, calibrated */
    {0x03U, 10U},  /* magnetic field, calibrated */
    {0x04U, 10U},  /* linear acceleration */
    {0x05U, 14U},  /* rotation vector */
    {0x06U, 10U},  /* gravity */
    {0x07U, 16U},  /* gyroscope, uncalibrated */
    {0x08U, 12U},  /* game rotation vector */
    {0x09U, 14U},  /* geomagnetic rotation vector */
    {0x28U, 14U},  /* AR/VR stabilised rotation vector */
    {0x29U, 12U},  /* AR/VR stabilised game rotation vector */
    {0x2AU, 14U},  /* gyro-integrated rotation vector */
    {REPORT_BASE_TIMESTAMP, 5U},
    {REPORT_TIMESTAMP_REBASE, 5U},
};

static uint8_t LengthOf(uint8_t id)
{
    for (unsigned i = 0U; i < sizeof kReportLens / sizeof kReportLens[0]; i++) {
        if (kReportLens[i].id == id) {
            return kReportLens[i].len;
        }
    }
    return 0U;
}

static int16_t ReadI16(const uint8_t *p)
{
    return (int16_t)(uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t ReadI32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

/* Fills r->v from the data bytes (report[4..]) according to the report type.
 * Returns 0 for report types this module does not hand to the sink. */
static int Decode(const uint8_t *report, ImuReport *r)
{
    const uint8_t *d = &report[4];
    switch (report[0]) {
    case IMU_REPORT_GAME_ROTATION_VECTOR:
    case IMU_REPORT_ROTATION_VECTOR:
        r->v[0] = (float)ReadI16(&d[0]) * Q(14);
        r->v[1] = (float)ReadI16(&d[2]) * Q(14);
        r->v[2] = (float)ReadI16(&d[4]) * Q(14);
        r->v[3] = (float)ReadI16(&d[6]) * Q(14);
        return 1;
    case IMU_REPORT_GYROSCOPE:
        r->v[0] = (float)ReadI16(&d[0]) * Q(9);
        r->v[1] = (float)ReadI16(&d[2]) * Q(9);
        r->v[2] = (float)ReadI16(&d[4]) * Q(9);
        r->v[3] = 0.0f;
        return 1;
    case IMU_REPORT_LINEAR_ACCELERATION:
    case IMU_REPORT_ACCELEROMETER:
        r->v[0] = (float)ReadI16(&d[0]) * Q(8);
        r->v[1] = (float)ReadI16(&d[2]) * Q(8);
        r->v[2] = (float)ReadI16(&d[4]) * Q(8);
        r->v[3] = 0.0f;
        return 1;
    default:
        return 0;
    }
}

ImuParseResult ImuReports_Parse(const uint8_t *payload, uint16_t len, ImuReportSink sink, void *ctx)
{
    uint16_t cursor = 0U;
    int32_t reference_delta = 0;  /* -timebase, 100 us units */

    while (cursor < len) {
        uint8_t id = payload[cursor];
        uint8_t report_len = LengthOf(id);
        if (report_len == 0U) {
            return IMU_PARSE_UNKNOWN_ID;
        }
        if ((uint32_t)cursor + report_len > len) {
            return IMU_PARSE_TRUNCATED;
        }
        const uint8_t *report = &payload[cursor];

        if (id == REPORT_BASE_TIMESTAMP) {
            reference_delta = -ReadI32(&report[1]);
        } else if (id == REPORT_TIMESTAMP_REBASE) {
            reference_delta += ReadI32(&report[1]);
        } else {
            ImuReport r;
            r.report_id = id;
            r.accuracy = (uint8_t)(report[2] & 0x03U);
            uint16_t delay = (uint16_t)(((uint16_t)(report[2] & 0xFCU) << 6) + report[3]);
            r.delay_100us = reference_delta + (int32_t)delay;
            if (Decode(report, &r) != 0 && sink != 0) {
                sink(&r, ctx);
            }
        }
        cursor = (uint16_t)(cursor + report_len);
    }
    return IMU_PARSE_OK;
}
