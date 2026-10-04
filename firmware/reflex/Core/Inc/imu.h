#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* BNO086 driver over I2C1 (PB7=SDA, PB8=SCL), SHTP/SH-2 protocol, polled.
 * Three reports are enabled: Game Rotation Vector (orientation, no
 * magnetometer so no compass interference — GNSS heading is still the real
 * yaw reference), calibrated Gyroscope and Linear Acceleration. Parsing is
 * in imu_reports.c, which follows CEVA's SH-2 reference source; this file's
 * I2C transport and feature-enable follow sparkfun/SparkFun_BNO080_Arduino_
 * Library. None of it has run against a real BNO086 yet (no hardware).
 *
 * Timing: the INT pin is not wired, so every sample's time is "poll time
 * plus the sensor's own delay field", good to about one poll period (20 ms).
 * Wiring INT to a spare GPIO and timestamping it with Clock_NowUs() in an
 * EXTI handler would remove that uncertainty — a hardware change worth
 * making before the IMU is used for anything precise. */

void IMU_Init(void);
void IMU_Update(void);

typedef struct {
    uint32_t t_us;         /* STM clock (clock.h), low 32 bits, of the newest report included */
    float quat[4];         /* unit quaternion x, y, z, w (BNO086 "i, j, k, real") */
    float gyro[3];         /* rad/s, sensor frame */
    float accel[3];        /* m/s^2, linear acceleration (gravity removed), sensor frame */
    bool quat_valid;
    bool gyro_valid;
    bool accel_valid;
    uint8_t quat_accuracy; /* 0 unreliable .. 3 high */
} ImuSample;

/* Fills *out with the latest values and per-field validity (a field is
 * valid while its report is fresh). Returns true if anything is valid. */
bool IMU_GetSample(ImuSample *out);

/* Orientation-derived roll/pitch in degrees, valid when IMU_IsValid().
 * Yaw is deliberately not exposed here — the IMU has no absolute heading. */
bool IMU_IsValid(void);
float IMU_GetRollDeg(void);
float IMU_GetPitchDeg(void);

#endif /* IMU_H */
