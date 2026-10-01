#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

/* BNO086 driver over I2C1 (PB7=SDA, PB8=SCL) — SHTP/SH-2 protocol,
 * "Game Rotation Vector" report only (no magnetometer fusion, so no
 * compass-interference worries; GNSS heading is still what drives yaw —
 * see ROADMAP.md section 4's "yön birleştirme" item). Protocol constants
 * below (channel numbers, report IDs, Set Feature frame layout, input
 * report byte offsets, Q14 scale) are taken from a real, known-working
 * open-source driver (sparkfun/SparkFun_BNO080_Arduino_Library, read
 * 2026-10-01), not reconstructed from memory — but this module itself has
 * never run against a real BNO086 (no hardware this session). Call
 * IMU_Init() once, then IMU_Update() periodically (main loop, e.g. every
 * 20-50 ms) to poll for new reports. */

void IMU_Init(void);
void IMU_Update(void);

/* Degrees, valid only when IMU_IsValid() is true. Yaw is deliberately not
 * exposed — GNSS heading is the real yaw source in this project, the IMU
 * only fills in roll/pitch that GNSS can't provide. */
bool IMU_IsValid(void);
float IMU_GetRollDeg(void);
float IMU_GetPitchDeg(void);

#endif /* IMU_H */
