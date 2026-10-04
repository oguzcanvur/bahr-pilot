#ifndef IMU_REPORTS_H
#define IMU_REPORTS_H

#include <stdint.h>

/* BNO086 SH-2 sensor-report parsing. Pure logic (no HAL): tested on the host
 * with hand-built packets, tests/c/test_imu_reports.c.
 *
 * Everything below comes from CEVA's own SH-2 reference library
 * (github.com/ceva-dsp/sh2: sh2.c's report-length table and
 * sensorhubInputHdlr(), sh2_SensorValue.c's decoders), read 2026-10-02 —
 * not from memory. It has NOT been checked against a real BNO086 yet: the
 * test vectors are built from that source's layout, not captured from a
 * device.
 *
 * A SHTP input packet on the sensor channel is a back-to-back sequence of
 * reports, each starting with its report id and with a fixed length per id:
 *   0xFB  Base Timestamp  [id][int32 LE timebase, 100 us units]     5 bytes
 *   0xFA  Timestamp Rebase[id][int32 LE timebase]                    5 bytes
 *   then any number of sensor reports, each:
 *     [id][sequence][status][delay][data ...]
 *     status bits 1:0 = accuracy (0 unreliable .. 3 high)
 *     delay = ((status & 0xFC) << 6) + delay_byte, 100 us units
 *     data  = little-endian int16 values
 * A sample's time is:  host_time_of_INT + (-timebase + delay) * 100 us.
 * (With no INT pin wired the host time is the poll time, so the result is
 * only good to the poll period — see imu.c.) */

#define IMU_REPORT_ACCELEROMETER          0x01U
#define IMU_REPORT_GYROSCOPE              0x02U
#define IMU_REPORT_LINEAR_ACCELERATION    0x04U
#define IMU_REPORT_ROTATION_VECTOR        0x05U
#define IMU_REPORT_GAME_ROTATION_VECTOR   0x08U

typedef struct {
    uint8_t report_id;
    uint8_t accuracy;     /* 0..3 */
    int32_t delay_100us;  /* sample time relative to the packet's INT time, 100 us units */
    /* Rotation vectors: i, j, k, real (unit quaternion, i.e. x, y, z, w).
     * Gyroscope: x, y, z in rad/s. (Linear) acceleration: x, y, z in m/s^2. */
    float v[4];
} ImuReport;

typedef void (*ImuReportSink)(const ImuReport *report, void *ctx);

typedef enum {
    IMU_PARSE_OK = 0,
    IMU_PARSE_TRUNCATED = -1,   /* a report ran past the end of the payload */
    IMU_PARSE_UNKNOWN_ID = -2,  /* length unknown, so the rest cannot be split */
} ImuParseResult;

/* Walks one sensor-channel payload (SHTP header already stripped) and calls
 * `sink` for every gyroscope, linear-acceleration, accelerometer and
 * rotation-vector report. Other known reports are skipped by length. Reports
 * decoded before an error are still delivered. */
ImuParseResult ImuReports_Parse(const uint8_t *payload, uint16_t len, ImuReportSink sink, void *ctx);

#endif /* IMU_REPORTS_H */
