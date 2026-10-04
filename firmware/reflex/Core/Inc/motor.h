#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* Differential-thrust motor control. Pure logic (no HAL) — it computes the
 * pulse widths and failsafe.c writes them to the ESCs — so it is compiled
 * and tested on a host PC (tests/c/test_motor.c).
 *
 * Conventions (fixed here, once):
 *   - Motor 1 = LEFT, motor 2 = RIGHT. This is what BAHR-GCS assumes
 *     (SERVO1_FUNCTION = ThrottleLeft, SERVO3 = ThrottleRight in its
 *     parameter dictionary). Which physical wire is which is only known at
 *     the first test on water; fix that with Motor config `reversed` or by
 *     swapping the two ESC leads, never by changing this convention.
 *   - A command is a number in [-1, +1]: +1 full forward, 0 stop, -1 full
 *     reverse. Pulse = 1500 us + 500 us * command.
 *   - Steering > 0 means turn RIGHT (stick right): left motor faster, right
 *     motor slower.
 *
 * Safety properties this module guarantees:
 *   - Output never leaves [1000, 2000] us and never exceeds max_pct.
 *   - Output changes by at most slew_pct_per_s (so a command that jumps from
 *     full ahead to full astern takes time, instead of slamming the drive
 *     train) — except Motor_Halt(), which is immediate: stopping is never
 *     rate-limited.
 *   - Zero command is exactly neutral (1500 us), with no dead-band creep. */

#define MOTOR_PULSE_NEUTRAL_US 1500
#define MOTOR_PULSE_SPAN_US    500

typedef struct {
    uint16_t slew_pct_per_s;  /* ArduPilot MOT_SLEWRATE; 0 = no limit */
    uint8_t max_pct;          /* ArduPilot MOT_THR_MAX, 0..100 */
    bool reversed[2];         /* ArduPilot SERVO1_REVERSED / SERVO3_REVERSED */
} MotorConfig;

#define MOTOR_CONFIG_DEFAULT {100U, 100U, {false, false}}

typedef struct {
    MotorConfig cfg;
    float current[2];         /* last commanded value per motor, -1..+1 */
    uint32_t last_ms;
    bool have_last;
} MotorState;

/* Stick deadband: 0 inside +-deadband, then rescaled so the output is still
 * continuous and reaches +-1 at full deflection. */
float Motor_ApplyDeadband(float x, float deadband);

/* Skid-steer mix with saturation handling: if either side would exceed
 * +-1, both are scaled down together so the turn *ratio* is kept (the boat
 * still turns the way it was told, just slower). */
void Motor_Mix(float throttle, float steering, float *left, float *right);

void Motor_Init(MotorState *m, const MotorConfig *cfg);

/* One control step toward the given targets. now_ms is the caller's
 * millisecond clock; the step size is derived from the elapsed time (capped
 * at 250 ms so a long gap cannot cause a jump). Returns the pulses for
 * motor 1 and motor 2. */
void Motor_Step(MotorState *m, uint32_t now_ms, float left_target, float right_target,
                uint16_t *motor1_us, uint16_t *motor2_us);

/* Immediate neutral on both motors, resetting the ramp state. */
void Motor_Halt(MotorState *m, uint16_t *motor1_us, uint16_t *motor2_us);

/* Pi motor frames carry absolute pulses; convert one to a command. */
float Motor_PulseToCommand(uint16_t pulse_us);

#endif /* MOTOR_H */
