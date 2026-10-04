#include "motor.h"

#define MOTOR_MAX_DT_MS 250U

static float Clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float Absf(float x)
{
    return x < 0.0f ? -x : x;
}

float Motor_ApplyDeadband(float x, float deadband)
{
    if (deadband <= 0.0f) {
        return Clampf(x, -1.0f, 1.0f);
    }
    if (deadband >= 1.0f) {
        return 0.0f;
    }
    float mag = Absf(x);
    if (mag <= deadband) {
        return 0.0f;
    }
    float scaled = (mag - deadband) / (1.0f - deadband);
    scaled = Clampf(scaled, 0.0f, 1.0f);
    return x < 0.0f ? -scaled : scaled;
}

void Motor_Mix(float throttle, float steering, float *left, float *right)
{
    float l = throttle + steering;
    float r = throttle - steering;
    float peak = Absf(l) > Absf(r) ? Absf(l) : Absf(r);
    if (peak > 1.0f) {
        l /= peak;
        r /= peak;
    }
    *left = l;
    *right = r;
}

void Motor_Init(MotorState *m, const MotorConfig *cfg)
{
    m->cfg = *cfg;
    m->current[0] = 0.0f;
    m->current[1] = 0.0f;
    m->last_ms = 0U;
    m->have_last = false;
}

float Motor_PulseToCommand(uint16_t pulse_us)
{
    return Clampf(((float)pulse_us - (float)MOTOR_PULSE_NEUTRAL_US) / (float)MOTOR_PULSE_SPAN_US,
                  -1.0f, 1.0f);
}

static uint16_t CommandToPulse(float command, bool reversed)
{
    if (reversed) {
        command = -command;
    }
    float us = (float)MOTOR_PULSE_NEUTRAL_US + (float)MOTOR_PULSE_SPAN_US * command;
    /* round to nearest so +-0.5 us of float noise never shows up as a
     * non-neutral pulse at command 0 */
    int32_t rounded = (int32_t)(us + (us >= 0.0f ? 0.5f : -0.5f));
    if (rounded < 1000) {
        rounded = 1000;
    } else if (rounded > 2000) {
        rounded = 2000;
    }
    return (uint16_t)rounded;
}

void Motor_Step(MotorState *m, uint32_t now_ms, float left_target, float right_target,
                uint16_t *motor1_us, uint16_t *motor2_us)
{
    uint32_t dt_ms = 0U;
    if (m->have_last) {
        dt_ms = now_ms - m->last_ms;
        if (dt_ms > MOTOR_MAX_DT_MS) {
            dt_ms = MOTOR_MAX_DT_MS;
        }
    }
    m->last_ms = now_ms;
    m->have_last = true;

    float cap = (float)m->cfg.max_pct / 100.0f;
    float targets[2] = {Clampf(left_target, -1.0f, 1.0f) * cap,
                        Clampf(right_target, -1.0f, 1.0f) * cap};

    float max_step = 2.0f; /* unlimited: any move fits */
    if (m->cfg.slew_pct_per_s > 0U) {
        max_step = ((float)m->cfg.slew_pct_per_s / 100.0f) * ((float)dt_ms / 1000.0f);
    }

    for (int i = 0; i < 2; i++) {
        float delta = targets[i] - m->current[i];
        if (dt_ms == 0U && m->cfg.slew_pct_per_s > 0U) {
            delta = 0.0f; /* no time has passed (or first step after a halt): no movement yet */
        }
        delta = Clampf(delta, -max_step, max_step);
        m->current[i] = Clampf(m->current[i] + delta, -1.0f, 1.0f);
    }

    *motor1_us = CommandToPulse(m->current[0], m->cfg.reversed[0]);
    *motor2_us = CommandToPulse(m->current[1], m->cfg.reversed[1]);
}

void Motor_Halt(MotorState *m, uint16_t *motor1_us, uint16_t *motor2_us)
{
    m->current[0] = 0.0f;
    m->current[1] = 0.0f;
    m->have_last = false;
    *motor1_us = (uint16_t)MOTOR_PULSE_NEUTRAL_US;
    *motor2_us = (uint16_t)MOTOR_PULSE_NEUTRAL_US;
}
